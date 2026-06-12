import math
from typing import List, Optional, Sequence, Tuple, Union

import numpy as np
from shapely import LineString, LinearRing, MultiLineString, MultiPoint, MultiPolygon, Point, Polygon
from shapely.affinity import rotate, translate
from shapely.geometry.base import BaseGeometry
from shapely.ops import nearest_points, polylabel, unary_union, voronoi_diagram

from nest_graph.board import board_context_from_geometry
from nest_graph.config import ProposeConfig, dedupe_transforms
from nest_graph.geometry import Geometry
from nest_graph.placement_scene import (
    PLACEMENT_EPSILON_RATIO,
    best_proposition,
    build_placement_scene,
    guidance_config_for_propose,
    guidance_config_for_scene,
    guidance_ray_direction_candidates,
    is_valid_placement,
    placement_footprint_inside_board,
    footprints_inside_board,
    proposition_translation,
    tiered_propositions,
)
from nest_graph.utils import get_shape_exteriors, get_shape_polygons_coords, transform_poly

from nest_graph.propose.context import _placement_contact_error
from nest_graph.propose.placement_common import outline_standoff_distance
from nest_graph.propose.geometry import ProposeGeometry
from nest_graph.propose.placements_guidance import (
    _candidate_from_proposition,
    _is_cast_move,
)
from nest_graph.propose.context import should_use_border_focus


def _score_placement_rule(
    coords: Tuple[float, float, float],
    rules,
    group_id: int,
) -> float:
    from nest_graph.elem_graph import score_transform
    return float(score_transform(
        rules, group_id, float(coords[0]), float(coords[1]), float(coords[2]),
    ))


def _rule_hybrid_geometry_mode(
    propose_cfg: ProposeConfig,
    base_shape: BaseGeometry,
) -> str:
    if (
        propose_cfg.border_focus_ranking
        and should_use_border_focus(base_shape, propose_cfg)
    ):
        return "border"
    if propose_cfg.use_contact_clearance_hybrid:
        return "contact_hybrid"
    return "contact"


def _score_placement_rule_hybrid(
    coords: Tuple[float, float, float],
    shape_to_place: Polygon,
    base_shape: BaseGeometry,
    propose_geom: ProposeGeometry,
    propose_cfg: ProposeConfig,
    pt_push: Point,
    min_dist: float,
    focal_shape: Optional[BaseGeometry],
    rules,
    group_id: int,
) -> float:
    geom_mode = _rule_hybrid_geometry_mode(propose_cfg, base_shape)
    if geom_mode == "border":
        score = _score_placement_border(
            coords, shape_to_place, propose_geom, pt_push, min_dist,
        )
    else:
        score = _score_placement_contact_hybrid(
            coords, shape_to_place, propose_geom, pt_push, min_dist, focal_shape,
            propose_cfg.contact_clearance_hybrid_weight,
            tightness_weight=propose_cfg.contact_tightness_hybrid_weight,
        )
    if score == float("-inf") or rules is None or rules.size() == 0:
        return score
    rule_score = _score_placement_rule(coords, rules, group_id)
    return score + propose_cfg.rule_ranking_weight * rule_score

def _neighbor_excess_gap_for_placed(
    placed_geom: Geometry,
    base_geoms: list[Geometry],
    min_dist: float,
) -> float:
    if not base_geoms:
        return 0.0
    from nest_graph.geometry import find_polygon_distances_bipartite
    results = find_polygon_distances_bipartite([placed_geom], base_geoms, aura=0.5)
    nearest = float("inf")
    for r in results:
        d = 0.0 if r.intersect else math.sqrt(r.distance_sq)
        nearest = min(nearest, d)
    if nearest >= float("inf"):
        return 0.0
    return max(0.0, nearest - min_dist)


def _score_placement_tightness(
    coords: Tuple[float, float, float],
    propose_geom: ProposeGeometry,
    pt_push: Point,
    min_dist: float,
) -> float:
    """Higher score = tighter outline kiss + lower neighbor excess gap."""
    placed_geom = propose_geom.placed_at(coords)
    if not propose_geom.is_valid_placement(placed_geom, pt_push, (coords[0], coords[1])):
        return float("-inf")
    kiss_err = abs(
        outline_standoff_distance(placed_geom, propose_geom.sheet) - min_dist,
    )
    excess = _neighbor_excess_gap_for_placed(
        placed_geom, propose_geom.base_geoms, min_dist,
    )
    return -(2.0 * excess + 1.0 * kiss_err)

def calculate_complex_score(base, placed, base_hull_area, centroid, pt_push, w_dist, w_dir, w_hull):
    # 1. Distance Component
    dist_to_center = Point(placed.centroid).distance(centroid)

    # 2. Directional Component (Dot Product)
    # We want to out of pt_push
    direction_score = -pt_push.distance(placed.centroid)

    pts = get_shape_polygons_coords(base) + get_shape_polygons_coords(placed)
    hull_growth = np.sqrt(max(0, MultiPoint(pts).convex_hull.area - base_hull_area))

    return (w_dist * dist_to_center) + (w_dir * direction_score) + (w_hull * hull_growth)


def finalize_propositions(propositions, top_n):
    """
    Sorts, deduplicates, and returns the top N propositions.
    """
    propositions.sort(key=lambda x: x['cost'])

    unique_props = []
    seen = set()
    for p in propositions:
        key = (round(p['coords'][0], 2), round(p['coords'][1], 2), round(p['coords'][2], 2))
        if key not in seen:
            unique_props.append(p['coords'])
            seen.add(key)
        if len(unique_props) >= top_n:
            break
    return unique_props


def propositions_to_ndarray(coords_list: Sequence[Tuple[float, float, float]]) -> np.ndarray:
    if not coords_list:
        return np.zeros((0, 3), dtype=np.float64)
    return np.asarray(coords_list, dtype=np.float64)


def base_shape_from_selection(
    polys: Sequence[BaseGeometry],
    selected_indices: Sequence[int],
) -> Union[Polygon, BaseGeometry]:
    placed = [polys[i] for i in selected_indices]
    if not placed:
        return Polygon()
    return unary_union(placed)


def _score_placement_coords(
    coords: Tuple[float, float, float],
    base_shape: BaseGeometry,
    shape_to_place: Polygon,
    boundary: BaseGeometry,
    pt_push: Point,
    min_dist: float,
    propose_geom: Optional[ProposeGeometry] = None,
) -> float:
    if propose_geom is not None:
        placed_geom = propose_geom.placed_at(coords)
        if not propose_geom.is_valid_placement(placed_geom, pt_push, (coords[0], coords[1])):
            return float("inf")
        placed = transform_poly(shape_to_place, coords)  # hull metrics still need Shapely
    else:
        placed = transform_poly(shape_to_place, coords)
        if not boundary.contains(placed):
            return float("inf")
        if not base_shape.is_empty:
            if base_shape.intersects(placed):
                return float("inf")
            if base_shape.distance(placed) < min_dist - 1e-6:
                return float("inf")
    base_hull_area = base_shape.convex_hull.area if not base_shape.is_empty else 0.0
    return calculate_complex_score(
        base_shape,
        placed,
        base_hull_area,
        boundary.centroid,
        pt_push,
        w_dist=0.001,
        w_dir=0.4,
        w_hull=0.1,
    )


def _score_placement_legacy(
    coords: Tuple[float, float, float],
    base_shape: BaseGeometry,
    shape_to_place: Polygon,
    boundary: BaseGeometry,
    pt_push: Point,
    min_dist: float,
    propose_geom: ProposeGeometry,
) -> float:
    placed_geom = propose_geom.placed_at(coords)
    if not propose_geom.is_valid_placement(placed_geom, pt_push, (coords[0], coords[1])):
        return float("inf")
    placed = transform_poly(shape_to_place, coords)
    base_hull_area = base_shape.convex_hull.area if not base_shape.is_empty else 0.0
    return calculate_complex_score(
        base_shape,
        placed,
        base_hull_area,
        boundary.centroid,
        pt_push,
        w_dist=0.001,
        w_dir=0.4,
        w_hull=0.1,
    )


def _score_placement_clearance(
    coords: Tuple[float, float, float],
    propose_geom: ProposeGeometry,
    pt_push: Point,
) -> float:
    placed_geom = propose_geom.placed_at(coords)
    if not propose_geom.is_valid_placement(placed_geom, pt_push, (coords[0], coords[1])):
        return float("-inf")
    g = propose_geom.placement_guidance(placed_geom, (coords[0], coords[1]), pt_push)
    if g.is_penetrating:
        return float("-inf")
    return float(g.clearance)


def _score_placement_hybrid(
    coords: Tuple[float, float, float],
    base_shape: BaseGeometry,
    shape_to_place: Polygon,
    boundary: BaseGeometry,
    pt_push: Point,
    min_dist: float,
    propose_geom: ProposeGeometry,
    propose_cfg: ProposeConfig,
) -> float:
    clearance = _score_placement_clearance(coords, propose_geom, pt_push)
    if clearance == float("-inf"):
        return float("-inf")
    legacy = _score_placement_legacy(
        coords, base_shape, shape_to_place, boundary, pt_push, min_dist, propose_geom,
    )
    if legacy == float("inf"):
        return float("-inf")
    w_c = propose_cfg.ranking_clearance_weight
    w_h = propose_cfg.ranking_hull_weight
    return w_c * clearance - w_h * legacy


def _score_placement_border(
    coords: Tuple[float, float, float],
    shape_to_place: Polygon,
    propose_geom: ProposeGeometry,
    pt_push: Point,
    min_dist: float,
) -> float:
    """Higher score = tighter fit to sheet border (lower exterior distance)."""
    placed_geom = propose_geom.placed_at(coords)
    if not propose_geom.is_valid_placement(placed_geom, pt_push, (coords[0], coords[1])):
        return float("-inf")
    err = abs(outline_standoff_distance(placed_geom, propose_geom.sheet) - min_dist)
    return -err


def _score_placement_contact(
    coords: Tuple[float, float, float],
    shape_to_place: Polygon,
    propose_geom: ProposeGeometry,
    pt_push: Point,
    min_dist: float,
    focal_shape: Optional[BaseGeometry] = None,
) -> float:
    """Higher score = tighter fit to sheet border and/or focal group."""
    placed_geom = propose_geom.placed_at(coords)
    if not propose_geom.is_valid_placement(placed_geom, pt_push, (coords[0], coords[1])):
        return float("-inf")
    err = _placement_contact_error(
        placed_geom, propose_geom.sheet, min_dist, focal_shape,
    )
    return -err


def _score_placement_contact_hybrid(
    coords: Tuple[float, float, float],
    shape_to_place: Polygon,
    propose_geom: ProposeGeometry,
    pt_push: Point,
    min_dist: float,
    focal_shape: Optional[BaseGeometry],
    clearance_weight: float,
    *,
    tightness_weight: float = 0.0,
) -> float:
    contact = _score_placement_contact(
        coords, shape_to_place, propose_geom, pt_push, min_dist, focal_shape,
    )
    if contact == float("-inf"):
        return float("-inf")
    clearance = _score_placement_clearance(coords, propose_geom, pt_push)
    if clearance == float("-inf"):
        score = contact
    else:
        score = contact + clearance_weight * clearance
    if tightness_weight > 0.0:
        tightness = _score_placement_tightness(
            coords, propose_geom, pt_push, min_dist,
        )
        if tightness > float("-inf"):
            score += tightness_weight * tightness
    return score


def _rank_score_for_mode(
    coords: Tuple[float, float, float],
    *,
    rank_mode: str,
    base_shape: BaseGeometry,
    shape_to_place: Polygon,
    boundary: BaseGeometry,
    propose_geom: ProposeGeometry,
    propose_cfg: ProposeConfig,
    pt_push: Point,
    min_dist: float,
    focal_shape: Optional[BaseGeometry],
    rules=None,
    group_id: int = 0,
) -> float:
    if rank_mode == "rule_hybrid":
        return _score_placement_rule_hybrid(
            coords, shape_to_place, base_shape, propose_geom, propose_cfg,
            pt_push, min_dist, focal_shape, rules, group_id,
        )
    if rank_mode == "border":
        return _score_placement_border(
            coords, shape_to_place, propose_geom, pt_push, min_dist,
        )
    if rank_mode == "contact":
        return _score_placement_contact(
            coords, shape_to_place, propose_geom, pt_push, min_dist, focal_shape,
        )
    if rank_mode == "contact_hybrid":
        return _score_placement_contact_hybrid(
            coords, shape_to_place, propose_geom, pt_push, min_dist, focal_shape,
            propose_cfg.contact_clearance_hybrid_weight,
            tightness_weight=propose_cfg.contact_tightness_hybrid_weight,
        )
    if rank_mode == "clearance":
        return _score_placement_clearance(coords, propose_geom, pt_push)
    if rank_mode == "hybrid":
        return _score_placement_hybrid(
            coords, base_shape, shape_to_place, boundary, pt_push, min_dist,
            propose_geom, propose_cfg,
        )
    legacy = _score_placement_legacy(
        coords, base_shape, shape_to_place, boundary, pt_push, min_dist, propose_geom,
    )
    return -legacy if legacy != float("inf") else float("-inf")


def cast_squeeze_ranked_coords(
    ranked: Sequence[Tuple[float, float, float]],
    shape_to_place: Polygon,
    propose_geom: ProposeGeometry,
    propose_cfg: ProposeConfig,
    pt_push: Point,
    min_dist: float,
    *,
    focal_shape: Optional[BaseGeometry],
    rank_mode: str,
    rules=None,
    group_id: int = 0,
    base_shape: BaseGeometry | None = None,
) -> List[Tuple[float, float, float]]:
    """Micro-refine top-K ranked coords with guidance cast moves."""
    k = min(max(propose_cfg.cast_squeeze_top_k, 0), len(ranked))
    if k <= 0:
        return list(ranked)
    out: list[tuple[float, float, float]] = []
    seen: set[tuple[float, float, float]] = set()
    for idx, coords in enumerate(ranked):
        if idx < k:
            coords = _cast_squeeze_one(
                coords,
                shape_to_place=shape_to_place,
                propose_geom=propose_geom,
                propose_cfg=propose_cfg,
                pt_push=pt_push,
                min_dist=min_dist,
                focal_shape=focal_shape,
                rank_mode=rank_mode,
                rules=rules,
                group_id=group_id,
                base_shape=base_shape or Polygon(),
            )
        key = (round(coords[0], 3), round(coords[1], 3), round(coords[2], 3))
        if key in seen:
            continue
        seen.add(key)
        out.append(coords)
    return out


def _cast_squeeze_one(
    coords: Tuple[float, float, float],
    *,
    shape_to_place: Polygon,
    propose_geom: ProposeGeometry,
    propose_cfg: ProposeConfig,
    pt_push: Point,
    min_dist: float,
    focal_shape: Optional[BaseGeometry],
    rank_mode: str,
    rules=None,
    group_id: int = 0,
    base_shape: BaseGeometry | None = None,
) -> Tuple[float, float, float]:
    placed_geom = propose_geom.placed_at(coords)
    if not propose_geom.is_valid_placement(placed_geom, pt_push, (coords[0], coords[1])):
        return coords
    best = coords
    best_score = _rank_score_for_mode(
        coords,
        rank_mode=rank_mode,
        base_shape=base_shape or Polygon(),
        shape_to_place=shape_to_place,
        boundary=propose_geom.boundary,
        propose_geom=propose_geom,
        propose_cfg=propose_cfg,
        pt_push=pt_push,
        min_dist=min_dist,
        focal_shape=focal_shape,
        rules=rules,
        group_id=group_id,
    )
    g = propose_geom.placement_guidance(placed_geom, (coords[0], coords[1]), pt_push)
    if g.is_penetrating:
        return coords
    x, y, theta = float(coords[0]), float(coords[1]), float(coords[2])
    for prop in g.propositions[: propose_cfg.guidance_max_propositions]:
        if not _is_cast_move(prop.move_type or ""):
            continue
        use_cast = not g.is_penetrating and _is_cast_move(prop.move_type or "")
        candidate = _candidate_from_proposition(
            x, y, theta, prop, use_full_cast=use_cast,
        )
        trial = propose_geom.placed_at(candidate)
        if not propose_geom.is_valid_placement(
            trial, pt_push, (candidate[0], candidate[1]),
        ):
            continue
        score = _rank_score_for_mode(
            candidate,
            rank_mode=rank_mode,
            base_shape=base_shape or Polygon(),
            shape_to_place=shape_to_place,
            boundary=propose_geom.boundary,
            propose_geom=propose_geom,
            propose_cfg=propose_cfg,
            pt_push=pt_push,
            min_dist=min_dist,
            focal_shape=focal_shape,
            rules=rules,
            group_id=group_id,
        )
        if score > best_score:
            best_score = score
            best = candidate
    return best


def _trim_candidates_stratified(
    candidates: Sequence[Tuple[float, float, float]],
    shape_to_place: Polygon,
    propose_geom: ProposeGeometry,
    pt_push: Point,
    min_dist: float,
    limit: int,
    focal_shape: Optional[BaseGeometry],
    *,
    contact_fraction: float,
    rank_mode: str,
    clearance_weight: float,
    propose_cfg: ProposeConfig,
    tightness_weight: float = 0.0,
    rules=None,
    group_id: int = 0,
    base_shape: BaseGeometry | None = None,
) -> List[Tuple[float, float, float]]:
    """Keep edge-fit and pocket candidates when trimming an oversized pool."""
    if limit <= 0 or not candidates:
        return []
    n_contact = max(1, min(limit, int(round(limit * contact_fraction))))
    n_clear = max(0, limit - n_contact)

    def score_fn(coords: Tuple[float, float, float]) -> float:
        if rank_mode == "rule_hybrid":
            return _score_placement_rule_hybrid(
                coords, shape_to_place, base_shape or Polygon(), propose_geom,
                propose_cfg, pt_push, min_dist, focal_shape, rules, group_id,
            )
        if rank_mode == "contact_hybrid":
            return _score_placement_contact_hybrid(
                coords, shape_to_place, propose_geom, pt_push, min_dist,
                focal_shape, clearance_weight,
                tightness_weight=tightness_weight,
            )
        return _score_placement_contact(
            coords, shape_to_place, propose_geom, pt_push, min_dist, focal_shape,
        )

    scored_contact: list[tuple[float, Tuple[float, float, float]]] = []
    seen: set[tuple[float, float, float]] = set()
    for coords in candidates:
        key = (round(coords[0], 3), round(coords[1], 3), round(coords[2], 3))
        if key in seen:
            continue
        seen.add(key)
        s = score_fn(coords)
        if s > float("-inf"):
            scored_contact.append((s, coords))
    scored_contact.sort(key=lambda x: x[0], reverse=True)
    picked = [coords for _, coords in scored_contact[:n_contact]]
    picked_keys = {
        (round(c[0], 3), round(c[1], 3), round(c[2], 3)) for c in picked
    }

    if n_clear <= 0:
        return picked

    remaining = [
        c for c in candidates
        if (round(c[0], 3), round(c[1], 3), round(c[2], 3)) not in picked_keys
    ]
    clearance_picked = _trim_candidates_by_clearance(
        remaining, propose_geom, pt_push, n_clear,
    )
    out = list(picked)
    for c in clearance_picked:
        key = (round(c[0], 3), round(c[1], 3), round(c[2], 3))
        if key not in picked_keys:
            out.append(c)
            picked_keys.add(key)
        if len(out) >= limit:
            break
    return out[:limit]


def select_guidance_cast_seeds(
    candidates: Sequence[Tuple[float, float, float]],
    limit: int,
    shape_to_place: Polygon,
    propose_geom: ProposeGeometry,
    pt_push: Point,
    min_dist: float,
    focal_shape: Optional[BaseGeometry] = None,
) -> List[Tuple[float, float, float]]:
    """Pick cast expansion seeds by kiss quality, not pool append order."""
    if limit <= 0 or not candidates:
        return []
    if len(candidates) <= limit:
        return list(candidates)
    n_contact = max(1, limit // 2)
    n_clear = max(0, limit - n_contact)
    contact_picked = _trim_candidates_by_contact(
        candidates,
        shape_to_place,
        propose_geom,
        pt_push,
        min_dist,
        n_contact,
        focal_shape,
    )
    picked_keys = {
        (round(c[0], 3), round(c[1], 3), round(c[2], 3)) for c in contact_picked
    }
    remaining = [
        c for c in candidates
        if (round(c[0], 3), round(c[1], 3), round(c[2], 3)) not in picked_keys
    ]
    clear_picked = _trim_candidates_by_clearance(
        remaining, propose_geom, pt_push, n_clear,
    )
    out = list(contact_picked)
    for c in clear_picked:
        key = (round(c[0], 3), round(c[1], 3), round(c[2], 3))
        if key not in picked_keys:
            out.append(c)
            picked_keys.add(key)
        if len(out) >= limit:
            break
    return out[:limit]


def _trim_candidates_by_contact(
    candidates: Sequence[Tuple[float, float, float]],
    shape_to_place: Polygon,
    propose_geom: ProposeGeometry,
    pt_push: Point,
    min_dist: float,
    limit: int,
    focal_shape: Optional[BaseGeometry] = None,
) -> List[Tuple[float, float, float]]:
    if limit <= 0 or not candidates:
        return []
    scored: list[tuple[float, Tuple[float, float, float]]] = []
    seen: set[tuple[float, float, float]] = set()
    for coords in candidates:
        key = (round(coords[0], 3), round(coords[1], 3), round(coords[2], 3))
        if key in seen:
            continue
        seen.add(key)
        score = _score_placement_contact(
            coords, shape_to_place, propose_geom, pt_push, min_dist, focal_shape,
        )
        if score > float("-inf"):
            scored.append((score, coords))
    scored.sort(key=lambda x: x[0], reverse=True)
    return [coords for _, coords in scored[:limit]]


def _trim_candidates_by_clearance(
    candidates: Sequence[Tuple[float, float, float]],
    propose_geom: ProposeGeometry,
    pt_push: Point,
    limit: int,
) -> List[Tuple[float, float, float]]:
    """Keep up to limit candidates with highest placement clearance."""
    if limit <= 0 or not candidates:
        return []
    scored: list[tuple[float, Tuple[float, float, float]]] = []
    seen: set[tuple[float, float, float]] = set()
    for coords in candidates:
        key = (round(coords[0], 3), round(coords[1], 3), round(coords[2], 3))
        if key in seen:
            continue
        seen.add(key)
        score = _score_placement_clearance(coords, propose_geom, pt_push)
        if score > float("-inf"):
            scored.append((score, coords))
    scored.sort(key=lambda x: x[0], reverse=True)
    return [coords for _, coords in scored[:limit]]


def _rank_proposal_coords(
    candidates: Sequence[Tuple[float, float, float]],
    base_shape: BaseGeometry,
    shape_to_place: Polygon,
    boundary: BaseGeometry,
    pt_push: Point,
    min_dist: float,
    max_n: int,
    propose_geom: ProposeGeometry,
    propose_cfg: ProposeConfig,
    *,
    rank_mode: Optional[str] = None,
    focal_shape: Optional[BaseGeometry] = None,
    rules=None,
    group_id: int = 0,
) -> List[Tuple[float, float, float]]:
    """Rank candidates; higher score is better for clearance/hybrid, lower for legacy."""
    scored: list[tuple[float, Tuple[float, float, float]]] = []
    seen: set[tuple[float, float, float]] = set()
    mode = rank_mode if rank_mode is not None else propose_cfg.ranking_mode
    for coords in candidates:
        key = (round(coords[0], 3), round(coords[1], 3), round(coords[2], 3))
        if key in seen:
            continue
        seen.add(key)
        if mode == "border":
            score = _score_placement_border(
                coords, shape_to_place, propose_geom, pt_push, min_dist,
            )
        elif mode == "contact":
            score = _score_placement_contact(
                coords, shape_to_place, propose_geom, pt_push, min_dist, focal_shape,
            )
        elif mode == "contact_hybrid":
            score = _score_placement_contact_hybrid(
                coords, shape_to_place, propose_geom, pt_push, min_dist, focal_shape,
                propose_cfg.contact_clearance_hybrid_weight,
                tightness_weight=propose_cfg.contact_tightness_hybrid_weight,
            )
        elif mode == "rule_hybrid":
            score = _score_placement_rule_hybrid(
                coords, shape_to_place, base_shape, propose_geom, propose_cfg,
                pt_push, min_dist, focal_shape, rules, group_id,
            )
        elif mode == "clearance":
            score = _score_placement_clearance(coords, propose_geom, pt_push)
        elif mode == "hybrid":
            score = _score_placement_hybrid(
                coords, base_shape, shape_to_place, boundary, pt_push, min_dist,
                propose_geom, propose_cfg,
            )
        else:
            score = -_score_placement_legacy(
                coords, base_shape, shape_to_place, boundary, pt_push, min_dist, propose_geom,
            )
        if score > float("-inf") and score < float("inf"):
            scored.append((score, coords))
        elif mode == "legacy" and score > float("-inf"):
            scored.append((score, coords))
    reverse = mode not in ("legacy",)
    scored.sort(key=lambda x: x[0], reverse=reverse)
    return [coords for _, coords in scored[:max_n]]
