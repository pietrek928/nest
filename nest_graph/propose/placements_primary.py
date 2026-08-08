"""Perimeter walk, neighbor slide, and erosion proposers."""

from typing import List, Optional, Tuple

import numpy as np
from shapely import MultiPolygon, Point, Polygon
from shapely.affinity import rotate
from shapely.geometry.base import BaseGeometry
from shapely.ops import polylabel

from nest_graph.utils import get_shape_exteriors

from nest_graph.propose.context import search_region_for_placement
from nest_graph.propose.geometry import ProposeGeometry, filter_candidates_batch
from nest_graph.propose.ranking import (
    find_polygon_distances_bipartite,
    finalize_propositions,
)
from nest_graph.propose.placement_common import (
    obstacle_parts,
    placement_safe_zone,
    resolve_placement_angles,
)
from nest_graph.propose.placement_outline import slide_toward_obstacle
from nest_graph.propose.placement_perimeter import perimeter_ring_vertices


def _finalize_placement_propositions(
    propositions: list[dict],
    top_n: int,
) -> List[Tuple[float, float, float]]:
    return finalize_propositions(propositions, top_n, coord_ndigits=4)


def _score_and_keep_batch(
    raw: list[tuple[float, float, float]],
    costs: list[float],
    propose_geom: ProposeGeometry,
    pt_push: Point,
    top_n: int,
) -> List[Tuple[float, float, float]]:
    """Snap/emit-raw then one full-guidance batch filter; keep best top_n."""
    if not raw:
        return []
    # Oversample pool before filter for early-exit style proposers.
    order = sorted(range(len(raw)), key=lambda i: costs[i])
    take = min(len(order), max(top_n * 3, top_n))
    ordered = [raw[i] for i in order[:take]]
    kept_costs = {raw[i]: costs[i] for i in order[:take]}
    valid = filter_candidates_batch(propose_geom, ordered, pt_push)
    valid.sort(key=lambda c: kept_costs.get(c, 0.0))
    return valid[:top_n]


def propose_placements_perimeter_walk(
    base_shape: BaseGeometry,
    shape_to_place: Polygon,
    sheet: Polygon,
    min_dist: float,
    *,
    propose_geom: ProposeGeometry,
    pt_push: Point,
    use_free_region: bool,
    border_focus: bool,
    num_angles: int,
    top_n: int,
    placement_angles: np.ndarray | None = None,
) -> List[Tuple[float, float, float]]:
    """Trace the safe-zone boundary for kiss placements against walls or neighbors."""
    if base_shape is None:
        base_shape = Polygon()
    region = search_region_for_placement(
        base_shape, sheet, sheet, min_dist,
        use_free_region=use_free_region, border_focus=border_focus,
    )
    if region.is_empty:
        return []

    raw: list[tuple[float, float, float]] = []
    costs: list[float] = []
    angles = resolve_placement_angles(placement_angles, num_angles)

    for angle in angles:
        rotated = rotate(shape_to_place, angle, origin=(0, 0), use_radians=True)
        rc = rotated.centroid
        safe_zone = placement_safe_zone(region, base_shape, rotated, min_dist)
        if safe_zone.is_empty:
            continue

        for ring in get_shape_exteriors(safe_zone):
            for px, py in perimeter_ring_vertices(ring):
                dx = px - rc.x
                dy = py - rc.y
                coords = (dx, dy, float(angle))
                placed_geom = propose_geom.placed_at(coords)
                if propose_geom.base_geoms:
                    dists = find_polygon_distances_bipartite(
                        [placed_geom], propose_geom.base_geoms,
                    )
                    score = min(d.distance for d in dists) if dists else 10.0
                else:
                    score = placed_geom.standoff_distance(propose_geom.boundary_ring_geom)
                raw.append(coords)
                costs.append(float(score))

    return _score_and_keep_batch(raw, costs, propose_geom, pt_push, top_n)


def propose_placements_neighbor_slide(
    base_shape: BaseGeometry,
    shape_to_place: Polygon,
    sheet: Polygon,
    min_dist: float,
    *,
    propose_geom: ProposeGeometry,
    pt_push: Point,
    num_angles: int,
    top_n: int,
    placement_angles: np.ndarray | None = None,
) -> List[Tuple[float, float, float]]:
    """Slide toward each packed part along the contact normal until min_dist clearance."""
    if base_shape is None or base_shape.is_empty:
        return []

    obstacles = obstacle_parts(base_shape)
    if not obstacles:
        return []

    raw: list[tuple[float, float, float]] = []
    costs: list[float] = []
    angles = resolve_placement_angles(placement_angles, num_angles)

    for angle in angles:
        for obstacle in obstacles:
            coords = slide_toward_obstacle(
                shape_to_place,
                obstacle,
                float(angle),
                min_dist,
                sheet,
                propose_geom=propose_geom,
            )
            if coords is None:
                continue
            placed_geom = propose_geom.placed_at(coords)
            if propose_geom.base_geoms:
                dists = find_polygon_distances_bipartite(
                    [placed_geom], propose_geom.base_geoms,
                )
                score = min(d.distance for d in dists) if dists else 10.0
            else:
                score = 10.0
            raw.append(coords)
            costs.append(float(score))

    return _score_and_keep_batch(raw, costs, propose_geom, pt_push, top_n)


def propose_placements_erosion(
    base_shape: BaseGeometry,
    shape_to_place: Polygon,
    sheet: Polygon,
    min_dist: float,
    *,
    propose_geom: ProposeGeometry,
    pt_push: Point,
    use_free_region: bool,
    border_focus: bool,
    focal_shape: Optional[BaseGeometry],
    num_angles: int,
    top_n: int,
    placement_angles: np.ndarray | None = None,
) -> List[Tuple[float, float, float]]:
    """Polylabel seeds in large voids; perimeter walk handles edge kissing."""
    if base_shape is None:
        base_shape = Polygon()
    region = search_region_for_placement(
        base_shape, sheet, sheet, min_dist,
        use_free_region=use_free_region, border_focus=border_focus,
    )
    if region.is_empty:
        return []

    attract = (
        focal_shape.centroid
        if focal_shape is not None and not focal_shape.is_empty
        else region.centroid
    )
    raw: list[tuple[float, float, float]] = []
    costs: list[float] = []
    angles = resolve_placement_angles(placement_angles, num_angles)

    for angle in angles:
        rotated_shape = rotate(shape_to_place, angle, origin=(0, 0), use_radians=True)
        safe_zone = placement_safe_zone(region, base_shape, rotated_shape, min_dist)
        if safe_zone.is_empty:
            continue

        candidate_points: list[Point] = []
        polys = safe_zone.geoms if isinstance(safe_zone, MultiPolygon) else (safe_zone,)
        for poly in polys:
            if not isinstance(poly, Polygon) or poly.is_empty:
                continue
            candidate_points.append(polylabel(poly, tolerance=1.0))
            candidate_points.append(poly.representative_point())

        for pt in candidate_points:
            coords = (float(pt.x), float(pt.y), float(angle))
            raw.append(coords)
            costs.append(float(pt.distance(attract)))

    return _score_and_keep_batch(raw, costs, propose_geom, pt_push, top_n)