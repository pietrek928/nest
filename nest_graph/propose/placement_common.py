import math

import numpy as np
from shapely import MultiPolygon, Polygon
from shapely.geometry.base import BaseGeometry
from shapely.ops import unary_union

from nest_graph.geometry import Geometry
from nest_graph.propose.placement_axis import (
    CARDINAL_DIRECTIONS,
    _axis_push_from_seed,
    _axis_push_from_seed_geom,
    _axis_placement_valid_geom,
)
from nest_graph.propose.placement_outline import (
    _inward_at_contact,
    _nest_outline_ring,
    _outline_ring_geom,
    _slide_toward_obstacle,
    _snap_coords_along_exterior,
    _snap_coords_along_exterior_geom,
    _standoff_gap,
    outline_kiss_ok,
    outline_kiss_tolerance,
    outline_standoff_distance,
)
from nest_graph.propose.placement_perimeter import (
    _angles_for_edge_contact,
    _edge_inward_at_point,
    _exterior_anchor_points,
    _exterior_segment_lengths,
    _finalize_edge_propositions,
    _perimeter_ring_vertices,
    _segment_index_at_point,
    _select_stratified_by_segment,
)

_MAX_OBSTACLE_PARTS = 32

__all__ = [
    "CARDINAL_DIRECTIONS",
    "_angles_for_edge_contact",
    "_axis_placement_valid_geom",
    "_axis_push_from_seed",
    "_axis_push_from_seed_geom",
    "_bottom_left_sort_key",
    "_edge_inward_at_point",
    "_exterior_anchor_points",
    "_finalize_edge_propositions",
    "_inward_at_contact",
    "_nest_outline_ring",
    "_nfp_valid_region",
    "_obstacle_parts",
    "_outline_ring_geom",
    "_perimeter_ring_vertices",
    "_placement_safe_zone",
    "_select_stratified_by_segment",
    "_slide_toward_obstacle",
    "_snap_coords_along_exterior",
    "_snap_coords_along_exterior_geom",
    "_standoff_gap",
    "clear_of_geoms",
    "clear_of_polys",
    "cluster_seed_coords",
    "outline_kiss_ok",
    "outline_kiss_tolerance",
    "outline_standoff_distance",
    "placement_angle_grid",
    "resolve_placement_angles",
]


def _obstacle_parts(base_shape: BaseGeometry) -> list[Polygon]:
    """Split packed layout into individual polygons for per-neighbor slides."""
    if base_shape is None or base_shape.is_empty:
        return []
    if isinstance(base_shape, MultiPolygon):
        parts = [
            g for g in base_shape.geoms
            if isinstance(g, Polygon) and not g.is_empty
        ]
    elif isinstance(base_shape, Polygon):
        parts = [base_shape]
    else:
        parts = []
    return parts[:_MAX_OBSTACLE_PARTS]


def _boundary_alignment_angles(poly: Polygon) -> list[float]:
    out: list[float] = []
    coords = list(poly.exterior.coords)
    for i in range(len(coords) - 1):
        dx = coords[i + 1][0] - coords[i][0]
        dy = coords[i + 1][1] - coords[i][1]
        if abs(dx) + abs(dy) < 1e-9:
            continue
        edge = math.atan2(dy, dx)
        out.append(edge)
        out.append(edge + math.pi / 2)
        out.append(edge - math.pi / 2)
    return out


def placement_angle_grid(
    sheet: Polygon,
    base_shape: BaseGeometry | None,
    num_angles: int,
) -> np.ndarray:
    """Uniform grid plus board/obstacle edge-alignment angles."""
    grid = list(np.linspace(0, 2 * np.pi, num_angles, endpoint=False))
    extra: list[float] = []
    if isinstance(sheet, Polygon):
        extra.extend(_boundary_alignment_angles(sheet))
    if base_shape is not None and not base_shape.is_empty:
        for part in _obstacle_parts(base_shape):
            extra.extend(_boundary_alignment_angles(part))
    merged: list[float] = []
    seen: set[float] = set()
    for angle in grid + extra:
        key = round(angle, 3)
        if key in seen:
            continue
        seen.add(key)
        merged.append(float(angle))
    cap = min(len(merged), max(num_angles, num_angles * 3))
    return np.asarray(merged[:cap], dtype=np.float64)


def resolve_placement_angles(
    angles: np.ndarray | None,
    num_angles: int,
) -> np.ndarray:
    if angles is not None and len(angles) > 0:
        return np.asarray(angles, dtype=np.float64)
    return np.linspace(0, 2 * np.pi, num_angles, endpoint=False)


def cluster_seed_coords(
    candidates: list[tuple[float, float, float]],
    *,
    dist_tol: float = 1.0,
    angle_tol: float = 0.15,
) -> list[tuple[float, float, float]]:
    """Greedy spatial dedupe before cast-refine."""
    if len(candidates) <= 1:
        return list(candidates)
    kept: list[tuple[float, float, float]] = []
    for coords in candidates:
        if any(
            math.hypot(coords[0] - k[0], coords[1] - k[1]) < dist_tol
            and abs(coords[2] - k[2]) < angle_tol
            for k in kept
        ):
            continue
        kept.append(coords)
    return kept


def _rotated_max_dim(rotated: Polygon) -> float:
    bounds = rotated.bounds
    return max(bounds[2] - bounds[0], bounds[3] - bounds[1]) / 2


def _bottom_left_sort_key(px: float, py: float) -> tuple[float, float]:
    return (py, px)


def _nfp_valid_region(
    sheet: Polygon,
    base_shape: BaseGeometry,
    rotated: Polygon,
    min_dist: float,
) -> BaseGeometry:
    """SVGnest-style valid region: eroded sheet minus union of buffered obstacles."""
    total_buf = _rotated_max_dim(rotated) + min_dist
    eroded_sheet = sheet.buffer(-total_buf)
    if eroded_sheet.is_empty:
        return eroded_sheet
    parts = _obstacle_parts(base_shape)
    if parts:
        obstacle_union = unary_union([p.buffer(total_buf) for p in parts])
        eroded_sheet = eroded_sheet.difference(obstacle_union)
    return eroded_sheet


def _placement_safe_zone(
    region: BaseGeometry,
    base_shape: BaseGeometry,
    rotated: Polygon,
    min_dist: float,
) -> BaseGeometry:
    total_buf = _rotated_max_dim(rotated) + min_dist
    safe_zone = region.buffer(-total_buf)
    if safe_zone.is_empty:
        return safe_zone
    if base_shape is not None and not base_shape.is_empty:
        safe_zone = safe_zone.difference(base_shape.buffer(total_buf))
    return safe_zone


def clear_of_geoms(candidate: Geometry, others: list[Geometry], min_dist: float) -> bool:
    if not others:
        return True
    from nest_graph.geometry import find_polygon_distances_bipartite
    results = find_polygon_distances_bipartite([candidate], others, aura=0.5)
    for r in results:
        if r.intersect:
            return False
        if math.sqrt(r.distance_sq) < min_dist - 1e-9:
            return False
    return True


def clear_of_polys(poly, others: list, min_dist: float) -> bool:
    if isinstance(poly, Geometry):
        other_geoms = [
            o if isinstance(o, Geometry) else Geometry.from_shapely(o)
            for o in others
        ]
        return clear_of_geoms(poly, other_geoms, min_dist)
    for other in others:
        if poly.intersects(other):
            return False
        if poly.distance(other) < min_dist - 1e-9:
            return False
    return True
