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
from nest_graph.propose.geometry import ProposeGeometry

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
        placed = transform_poly(shape_to_place, coords)
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
    placed = transform_poly(shape_to_place, coords)
    border_dist = float(placed.distance(propose_geom.sheet.exterior))
    err = abs(border_dist - min_dist)
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
    placed = transform_poly(shape_to_place, coords)
    err = _placement_contact_error(
        placed, propose_geom.sheet, min_dist, focal_shape,
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
) -> float:
    contact = _score_placement_contact(
        coords, shape_to_place, propose_geom, pt_push, min_dist, focal_shape,
    )
    if contact == float("-inf"):
        return float("-inf")
    clearance = _score_placement_clearance(coords, propose_geom, pt_push)
    if clearance == float("-inf"):
        return contact
    return contact + clearance_weight * clearance


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
) -> List[Tuple[float, float, float]]:
    """Keep edge-fit and pocket candidates when trimming an oversized pool."""
    if limit <= 0 or not candidates:
        return []
    n_contact = max(1, min(limit, int(round(limit * contact_fraction))))
    n_clear = max(0, limit - n_contact)

    def score_fn(coords: Tuple[float, float, float]) -> float:
        if rank_mode == "contact_hybrid":
            return _score_placement_contact_hybrid(
                coords, shape_to_place, propose_geom, pt_push, min_dist,
                focal_shape, clearance_weight,
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
