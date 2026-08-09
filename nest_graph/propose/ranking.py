import math
from typing import List, Optional, Sequence, Tuple, Union

import numpy as np
from shapely import MultiPoint, Point, Polygon
from shapely.geometry.base import BaseGeometry
from shapely.ops import unary_union

from nest_graph.config import ProposeConfig
from nest_graph.elem_graph import score_transform
from nest_graph.geometry import Geometry, find_polygon_distances_bipartite
from nest_graph.placement_scene import (
    best_proposition,
    placement_clearance_epsilon,
)
from nest_graph.utils import get_shape_polygons_coords, transform_poly

from nest_graph.propose.context import placement_contact_error, should_use_border_focus
from nest_graph.propose.placement_outline import outline_ring_geom, outline_standoff_distance
from nest_graph.propose.geometry import ProposeGeometry, batch_valid_flags
from nest_graph.propose.placements_guidance import (
    candidate_from_proposition,
    is_cast_move,
)


def _score_placement_rule(
    coords: Tuple[float, float, float],
    rules,
    group_id: int,
    *,
    radius: float = 0.5,
) -> float:
    return float(score_transform(
        rules, group_id, float(coords[0]), float(coords[1]), float(coords[2]),
        radius=float(radius),
    ))


def _part_rule_radius(shape_to_place: Polygon, propose_geom: ProposeGeometry) -> float:
    try:
        return float(propose_geom.part.radius())
    except Exception:
        minx, miny, maxx, maxy = shape_to_place.bounds
        return 0.5 * math.hypot(maxx - minx, maxy - miny)


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
    void_pole: Point | None = None,
) -> float:
    # Private geom scorers only — do not re-enter _rank_score_for_mode (R15).
    geom_mode = _rule_hybrid_geometry_mode(propose_cfg, base_shape)
    if geom_mode == "border":
        score = _score_placement_border(
            coords, shape_to_place, propose_geom, pt_push, min_dist, propose_cfg,
        )
    else:
        score = _score_placement_contact_hybrid(
            coords, shape_to_place, propose_geom, pt_push, min_dist, focal_shape,
            propose_cfg.contact_clearance_hybrid_weight,
            tightness_weight=propose_cfg.contact_tightness_hybrid_weight,
            propose_cfg=propose_cfg,
            void_pole=void_pole,
        )
    if score == float("-inf") or rules is None or rules.size() == 0:
        return score
    rule_score = _score_placement_rule(
        coords, rules, group_id,
        radius=_part_rule_radius(shape_to_place, propose_geom),
    )
    return score + propose_cfg.rule_ranking_weight * rule_score

def _neighbor_excess_gap_for_placed(
    placed_geom: Geometry,
    base_geoms: list[Geometry],
    min_dist: float,
) -> float:
    if not base_geoms:
        return 0.0
    results = find_polygon_distances_bipartite([placed_geom], base_geoms, aura=0.5)
    nearest = float("inf")
    for r in results:
        d = 0.0 if r.intersect else math.sqrt(r.distance_sq)
        nearest = min(nearest, d)
    if nearest >= float("inf"):
        return 0.0
    return max(0.0, nearest - min_dist)


def calculate_excess_and_kiss(
    geom: Geometry,
    *,
    obstacles: list[Geometry],
    outline: BaseGeometry | None,
    min_dist: float,
    ring_geom: Geometry | None = None,
) -> tuple[float, float]:
    """Single tightness math: (excess_neighbor_gap, outline_kiss_err).

    Both single-placed and pack-total wrappers must call this SoT.
    """
    excess = _neighbor_excess_gap_for_placed(geom, obstacles, min_dist) if obstacles else 0.0
    if ring_geom is not None:
        kiss = abs(geom.standoff_distance(ring_geom) - min_dist)
    elif outline is not None:
        kiss = abs(outline_standoff_distance(geom, outline) - min_dist)
    else:
        kiss = 0.0
    return excess, kiss


def pack_neighbor_excess_gap(geoms: list[Geometry], min_dist: float) -> float:
    """Sum of per-part excess gaps vs nearest neighbor in a pack."""
    if len(geoms) < 2:
        return 0.0
    from nest_graph.geometry import find_polygon_distances

    results = find_polygon_distances(geoms, aura=0.5)
    nearest = [float("inf")] * len(geoms)
    for r in results:
        d = 0.0 if r.intersect else math.sqrt(r.distance_sq)
        if 0 <= r.polyA_idx < len(nearest):
            nearest[r.polyA_idx] = min(nearest[r.polyA_idx], d)
        if 0 <= r.polyB_idx < len(nearest):
            nearest[r.polyB_idx] = min(nearest[r.polyB_idx], d)
    return sum(
        max(0.0, nd - min_dist)
        for nd in nearest
        if nd < float("inf")
    )


def pack_tightness_cost(
    geoms: Sequence[Geometry] | Sequence[BaseGeometry],
    outline: BaseGeometry,
    min_dist: float,
) -> float:
    """Lower is tighter for a full pack (2*excess + kiss). Uses shared kiss math.

    Accepts native ``Geometry`` (preferred, no conversion) or Shapely polys.
    """
    if not geoms:
        return 0.0
    if any(not isinstance(g, Geometry) for g in geoms):
        from nest_graph.propose.placement_common import as_geometry

        geoms = [g for g in (as_geometry(p) for p in geoms) if g is not None]
        if not geoms:
            return 0.0
    ring = outline_ring_geom(outline) if outline is not None else None
    kiss = 0.0
    for g in geoms:
        _excess, k = calculate_excess_and_kiss(
            g, obstacles=[], outline=outline, min_dist=min_dist, ring_geom=ring,
        )
        kiss += k
    excess_gap = pack_neighbor_excess_gap(geoms, min_dist)
    return 2.0 * excess_gap + 1.0 * kiss


def score_placement_tightness(
    coords: Tuple[float, float, float],
    propose_geom: ProposeGeometry,
    pt_push: Point,
    min_dist: float,
) -> float:
    """Higher score = tighter outline kiss + lower neighbor excess gap."""
    placed_geom = propose_geom.placed_at(coords)
    if not propose_geom.valid_at(coords, pt_push):
        return float("-inf")
    excess, kiss_err = calculate_excess_and_kiss(
        placed_geom,
        obstacles=list(propose_geom.base_geoms),
        outline=propose_geom.sheet,
        min_dist=min_dist,
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


def finalize_propositions(propositions, top_n, *, coord_ndigits: int = 2):
    """
    Sorts by cost, deduplicates, and returns the top N proposition coords.
    """
    propositions.sort(key=lambda x: x['cost'])

    unique_props = []
    seen = set()
    for p in propositions:
        c = p['coords']
        key = (
            round(c[0], coord_ndigits),
            round(c[1], coord_ndigits),
            round(c[2], coord_ndigits),
        )
        if key not in seen:
            unique_props.append(c)
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
    propose_geom: ProposeGeometry,
) -> float:
    if not propose_geom.valid_at(coords, pt_push):
        return float("inf")
    # Shapely transform_poly + convex_hull is intentional for hull scoring only;
    # placement validity must go through ProposeGeometry.valid_at (above).
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


def _score_placement_legacy(
    coords: Tuple[float, float, float],
    base_shape: BaseGeometry,
    shape_to_place: Polygon,
    boundary: BaseGeometry,
    pt_push: Point,
    min_dist: float,
    propose_geom: ProposeGeometry,
) -> float:
    if not propose_geom.valid_at(coords, pt_push):
        return float("inf")
    # Hull scoring only: Shapely transform_poly + convex_hull; validity via valid_at.
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


def _batch_placement_feedback(
    candidates: Sequence[Tuple[float, float, float]],
    propose_geom: ProposeGeometry,
    pt_push: Point,
) -> dict[Tuple[float, float, float], object | None]:
    if not candidates:
        return {}
    guidance = batch_valid_flags(
        propose_geom, candidates, pt_push, return_guidance=True,
    )
    return {c: g for c, g in zip(candidates, guidance, strict=True)}


def _placement_feedback(
    coords: Tuple[float, float, float],
    propose_geom: ProposeGeometry,
    pt_push: Point,
    *,
    feedback_cache: dict[Tuple[float, float, float], object | None] | None = None,
):
    if feedback_cache is not None and coords in feedback_cache:
        cached = feedback_cache[coords]
        return cached
    placed_geom = propose_geom.placed_at(coords)
    if not placed_geom.fully_inside(propose_geom.board_geom):
        return None
    g = propose_geom.placement_guidance(placed_geom, (coords[0], coords[1]), pt_push)
    if g.is_penetrating:
        return None
    if propose_geom._min_dist > 0.0:
        margin = propose_geom._min_dist + placement_clearance_epsilon(
            propose_geom._min_dist, ratio=propose_geom._epsilon_ratio,
        )
        if float(g.clearance) < margin:
            return None
    return g


def _score_placement_clearance(
    coords: Tuple[float, float, float],
    propose_geom: ProposeGeometry,
    pt_push: Point,
    propose_cfg: ProposeConfig | None = None,
    min_dist: float = 0.0,
) -> float:
    g = _placement_feedback(coords, propose_geom, pt_push)
    if g is None:
        return float("-inf")
    score = float(g.clearance)
    if propose_cfg is not None and propose_cfg.cast_rank_boost > 0.0 and min_dist > 0.0:
        prop = best_proposition(g)
        if prop is not None and is_cast_move(prop.move_type or ""):
            score += propose_cfg.cast_rank_boost * min_dist
    return score


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
    propose_cfg: ProposeConfig | None = None,
) -> float:
    """Higher score = tighter fit to sheet border (lower exterior distance)."""
    g = _placement_feedback(coords, propose_geom, pt_push)
    if g is None:
        return float("-inf")
    placed_geom = propose_geom.placed_at(coords)
    err = abs(outline_standoff_distance(placed_geom, propose_geom.sheet) - min_dist)
    score = -err
    if propose_cfg is not None and propose_cfg.cast_rank_boost > 0.0 and min_dist > 0.0:
        prop = best_proposition(g)
        if prop is not None and is_cast_move(prop.move_type or ""):
            score += propose_cfg.cast_rank_boost * min_dist
    return score


def _score_placement_contact(
    coords: Tuple[float, float, float],
    shape_to_place: Polygon,
    propose_geom: ProposeGeometry,
    pt_push: Point,
    min_dist: float,
    focal_shape: Optional[BaseGeometry] = None,
    propose_cfg: ProposeConfig | None = None,
) -> float:
    """Higher score = tighter fit to sheet border and/or focal group."""
    g = _placement_feedback(coords, propose_geom, pt_push)
    if g is None:
        return float("-inf")
    placed_geom = propose_geom.placed_at(coords)
    err = placement_contact_error(
        placed_geom, propose_geom.sheet, min_dist, focal_shape,
    )
    score = -err
    if propose_cfg is not None and propose_cfg.cast_rank_boost > 0.0 and min_dist > 0.0:
        prop = best_proposition(g)
        if prop is not None and is_cast_move(prop.move_type or ""):
            score += propose_cfg.cast_rank_boost * min_dist
    return score


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
    propose_cfg: ProposeConfig | None = None,
    void_pole: Point | None = None,
) -> float:
    contact = _score_placement_contact(
        coords, shape_to_place, propose_geom, pt_push, min_dist, focal_shape,
        propose_cfg,
    )
    if contact == float("-inf"):
        return float("-inf")
    clearance = _score_placement_clearance(coords, propose_geom, pt_push)
    if clearance == float("-inf"):
        score = contact
    else:
        score = contact + clearance_weight * clearance
    if tightness_weight > 0.0:
        tightness = score_placement_tightness(
            coords, propose_geom, pt_push, min_dist,
        )
        if tightness > float("-inf"):
            score += tightness_weight * tightness
    if void_pole is not None and propose_cfg is not None:
        w = float(getattr(propose_cfg, "void_rank_pole_weight", 0.0) or 0.0)
        if w > 0.0:
            placed = propose_geom.placed_at(coords)
            if placed is not None:
                sheet = propose_geom.sheet
                minx, miny, maxx, maxy = sheet.bounds
                scale = float(math.hypot(maxx - minx, maxy - miny))
                if scale > 1e-12:
                    cx, cy = placed.center()
                    dist = float(Point(float(cx), float(cy)).distance(void_pole))
                    area_frac = float(shape_to_place.area) / max(float(sheet.area), 1e-12)
                    score += max(0.0, 1.0 - dist / scale) * area_frac * w
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
    void_pole: Point | None = None,
) -> float:
    """Dispatch table for ranking modes (SoT — do not OR with guidance valid_at)."""
    scorers = {
        "rule_hybrid": lambda: _score_placement_rule_hybrid(
            coords, shape_to_place, base_shape, propose_geom, propose_cfg,
            pt_push, min_dist, focal_shape, rules, group_id, void_pole=void_pole,
        ),
        "border": lambda: _score_placement_border(
            coords, shape_to_place, propose_geom, pt_push, min_dist, propose_cfg,
        ),
        "contact": lambda: _score_placement_contact(
            coords, shape_to_place, propose_geom, pt_push, min_dist, focal_shape,
            propose_cfg,
        ),
        "contact_hybrid": lambda: _score_placement_contact_hybrid(
            coords, shape_to_place, propose_geom, pt_push, min_dist, focal_shape,
            propose_cfg.contact_clearance_hybrid_weight,
            tightness_weight=propose_cfg.contact_tightness_hybrid_weight,
            propose_cfg=propose_cfg,
            void_pole=void_pole,
        ),
        "clearance": lambda: _score_placement_clearance(
            coords, propose_geom, pt_push, propose_cfg, min_dist,
        ),
        "hybrid": lambda: _score_placement_hybrid(
            coords, base_shape, shape_to_place, boundary, pt_push, min_dist,
            propose_geom, propose_cfg,
        ),
    }
    scorer = scorers.get(rank_mode)
    if scorer is not None:
        return scorer()
    legacy = _score_placement_legacy(
        coords, base_shape, shape_to_place, boundary, pt_push, min_dist, propose_geom,
    )
    return -legacy if legacy != float("inf") else float("-inf")


# Public alias for plan / call sites.
score_for_mode = _rank_score_for_mode


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
        key = (round(coords[0], 4), round(coords[1], 4), round(coords[2], 4))
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
    if not propose_geom.valid_at(coords, pt_push):
        return coords
    best = coords
    best_score = _rank_score_for_mode(
        coords,
        rank_mode=rank_mode,
        base_shape=base_shape or Polygon(),
        shape_to_place=shape_to_place,
        boundary=propose_geom.sheet,
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
        if not is_cast_move(prop.move_type or ""):
            continue
        use_cast = not g.is_penetrating and is_cast_move(prop.move_type or "")
        candidate = candidate_from_proposition(
            x, y, theta, prop,
        )
        trial = propose_geom.placed_at(candidate)
        if not propose_geom.valid(
            trial, pt_push, (candidate[0], candidate[1]),
        ):
            continue
        score = _rank_score_for_mode(
            candidate,
            rank_mode=rank_mode,
            base_shape=base_shape or Polygon(),
            shape_to_place=shape_to_place,
            boundary=propose_geom.sheet,
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
        key = (round(coords[0], 4), round(coords[1], 4), round(coords[2], 4))
        if key in seen:
            continue
        seen.add(key)
        s = score_fn(coords)
        if s > float("-inf"):
            scored_contact.append((s, coords))
    scored_contact.sort(key=lambda x: x[0], reverse=True)
    picked = [coords for _, coords in scored_contact[:n_contact]]
    picked_keys = {
        (round(c[0], 4), round(c[1], 4), round(c[2], 4)) for c in picked
    }

    if n_clear <= 0:
        return picked

    remaining = [
        c for c in candidates
        if (round(c[0], 4), round(c[1], 4), round(c[2], 4)) not in picked_keys
    ]
    clearance_picked = _trim_candidates_by_clearance(
        remaining, propose_geom, pt_push, n_clear,
    )
    out = list(picked)
    for c in clearance_picked:
        key = (round(c[0], 4), round(c[1], 4), round(c[2], 4))
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
        (round(c[0], 4), round(c[1], 4), round(c[2], 4)) for c in contact_picked
    }
    remaining = [
        c for c in candidates
        if (round(c[0], 4), round(c[1], 4), round(c[2], 4)) not in picked_keys
    ]
    clear_picked = _trim_candidates_by_clearance(
        remaining, propose_geom, pt_push, n_clear,
    )
    out = list(contact_picked)
    for c in clear_picked:
        key = (round(c[0], 4), round(c[1], 4), round(c[2], 4))
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
        key = (round(coords[0], 4), round(coords[1], 4), round(coords[2], 4))
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
    propose_cfg: ProposeConfig | None = None,
    min_dist: float = 0.0,
) -> List[Tuple[float, float, float]]:
    """Keep up to limit candidates with highest placement clearance."""
    if limit <= 0 or not candidates:
        return []
    feedback_cache = (
        _batch_placement_feedback(candidates, propose_geom, pt_push)
        if len(candidates) >= 8
        else None
    )
    scored: list[tuple[float, Tuple[float, float, float]]] = []
    seen: set[tuple[float, float, float]] = set()
    for coords in candidates:
        key = (round(coords[0], 4), round(coords[1], 4), round(coords[2], 4))
        if key in seen:
            continue
        seen.add(key)
        g = _placement_feedback(
            coords, propose_geom, pt_push, feedback_cache=feedback_cache,
        )
        if g is None:
            continue
        score = float(g.clearance)
        if propose_cfg is not None and propose_cfg.cast_rank_boost > 0.0 and min_dist > 0.0:
            prop = best_proposition(g)
            if prop is not None and is_cast_move(prop.move_type or ""):
                score += propose_cfg.cast_rank_boost * min_dist
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
    void_pole: Point | None = None,
) -> List[Tuple[float, float, float]]:
    """Rank candidates; higher score is better for clearance/hybrid, lower for legacy."""
    scored: list[tuple[float, Tuple[float, float, float]]] = []
    seen: set[tuple[float, float, float]] = set()
    mode = rank_mode if rank_mode is not None else propose_cfg.ranking_mode
    for coords in candidates:
        key = (round(coords[0], 4), round(coords[1], 4), round(coords[2], 4))
        if key in seen:
            continue
        seen.add(key)
        score = _rank_score_for_mode(
            coords,
            rank_mode=mode if mode is not None else "legacy",
            base_shape=base_shape,
            shape_to_place=shape_to_place,
            boundary=boundary,
            propose_geom=propose_geom,
            propose_cfg=propose_cfg,
            pt_push=pt_push,
            min_dist=min_dist,
            focal_shape=focal_shape,
            rules=rules,
            group_id=group_id,
            void_pole=void_pole,
        )
        if score > float("-inf") and score < float("inf"):
            scored.append((score, coords))
        elif mode == "legacy" and score > float("-inf"):
            scored.append((score, coords))
    reverse = mode not in ("legacy",)
    scored.sort(key=lambda x: x[0], reverse=reverse)
    return [coords for _, coords in scored[:max_n]]
