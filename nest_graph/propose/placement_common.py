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
    placement_clearance_epsilon,
    placement_footprint_inside_board,
    footprints_inside_board,
    proposition_translation,
    tiered_propositions,
)
from nest_graph.utils import get_shape_exteriors, get_shape_polygons_coords, transform_poly

from nest_graph.propose.geometry import ProposeGeometry

_PERIMETER_VERTEX_CAP = 200
_MAX_OBSTACLE_PARTS = 32
_AXIS_PUSH_STEPS = 24
CARDINAL_DIRECTIONS: tuple[tuple[float, float], ...] = (
    (-1.0, 0.0),
    (1.0, 0.0),
    (0.0, -1.0),
    (0.0, 1.0),
)


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


def _slide_toward_obstacle(
    shape_to_place: Polygon,
    obstacle: Polygon,
    angle: float,
    min_dist: float,
    sheet: Polygon,
) -> Optional[Tuple[float, float, float]]:
    """Slide the part along the contact normal until it kisses this obstacle at min_dist."""
    rotated = rotate(shape_to_place, angle, origin=(0, 0), use_radians=True)
    p_obs, p_part = nearest_points(obstacle, rotated)
    dx = p_part.x - p_obs.x
    dy = p_part.y - p_obs.y
    dist = math.hypot(dx, dy)
    if dist < 1e-9:
        return None
    inward = (dx / dist, dy / dist)
    margin = min_dist + placement_clearance_epsilon(
        min_dist, ratio=PLACEMENT_EPSILON_RATIO,
    )
    return _snap_coords_along_exterior(
        shape_to_place,
        obstacle,
        p_obs,
        inward,
        angle,
        margin,
        container=sheet,
        standoff_pad=0.0,
    )


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


def _axis_placement_valid(
    placed: Polygon,
    sheet: Polygon,
    obstacle_geom: BaseGeometry,
    min_dist: float,
) -> bool:
    if not sheet.contains(placed):
        return False
    if obstacle_geom is not None and not obstacle_geom.is_empty:
        if obstacle_geom.intersects(placed):
            return False
        if float(placed.distance(obstacle_geom)) < min_dist - 1e-6:
            return False
    if float(placed.distance(sheet.exterior)) < min_dist - 1e-6:
        return False
    return True


def _axis_push_from_seed(
    rotated: Polygon,
    seed_dx: float,
    seed_dy: float,
    direction: Tuple[float, float],
    sheet: Polygon,
    obstacle_geom: BaseGeometry,
    min_dist: float,
) -> Optional[Tuple[float, float]]:
    """Push from seed along direction until contact; return total (dx, dy) from rotated origin."""
    dir_x, dir_y = direction
    placed = translate(rotated, seed_dx, seed_dy)
    if not _axis_placement_valid(placed, sheet, obstacle_geom, min_dist):
        return None

    min_x, min_y, max_x, max_y = sheet.bounds
    max_step = math.hypot(max_x - min_x, max_y - min_y)

    lo, hi = 0.0, max_step
    best = 0.0
    for _ in range(32):
        mid = (lo + hi) * 0.5
        trial = translate(
            rotated,
            seed_dx + dir_x * mid,
            seed_dy + dir_y * mid,
        )
        if _axis_placement_valid(trial, sheet, obstacle_geom, min_dist):
            best = mid
            lo = mid
        else:
            hi = mid

    return (seed_dx + dir_x * best, seed_dy + dir_y * best)


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


def _propose_valid_at(
    coords: Tuple[float, float, float],
    propose_geom: ProposeGeometry,
    pt_push: Point,
) -> bool:
    placed_g = propose_geom.placed_at(coords)
    return propose_geom.is_valid_placement(placed_g, pt_push, (coords[0], coords[1]))


def _perimeter_ring_vertices(ring: LineString | LinearRing) -> list[tuple[float, float]]:
    if ring.length < 1e-5:
        return []
    coords = list(ring.coords)
    if len(coords) > 1 and coords[0] == coords[-1]:
        coords = coords[:-1]
    if len(coords) <= _PERIMETER_VERTEX_CAP:
        return [(float(x), float(y)) for x, y in coords]
    stride = max(1, len(coords) // _PERIMETER_VERTEX_CAP)
    sampled = coords[::stride]
    return [(float(x), float(y)) for x, y in sampled]

def _exterior_anchor_points(geom: BaseGeometry, samples_per_edge: int) -> list[Point]:
    anchors: list[Point] = []
    for line in get_shape_exteriors(geom):
        if line.length <= 0:
            continue
        for t in np.linspace(0.02, 0.98, max(2, samples_per_edge)):
            anchors.append(line.interpolate(t, normalized=True))
    return anchors


def _snap_coords_along_exterior(
    shape_to_place: Polygon,
    boundary: BaseGeometry,
    contact: Point,
    inward: Tuple[float, float],
    angle: float,
    min_dist: float,
    *,
    container: Optional[Polygon] = None,
    standoff_pad: Optional[float] = None,
) -> Optional[Tuple[float, float, float]]:
    """Snap rotated part so nearest point on boundary sits at min_dist along inward."""
    ix, iy = inward
    rotated = rotate(shape_to_place, angle, origin=(0, 0), use_radians=True)
    rc = rotated.centroid
    standoff = boundary.exterior if isinstance(boundary, Polygon) else boundary
    bounds = rotated.bounds
    if standoff_pad is None:
        pad = max(bounds[2] - bounds[0], bounds[3] - bounds[1]) * 0.5
    else:
        pad = standoff_pad
    _, p_part = nearest_points(standoff, rotated)
    target = Point(
        contact.x + ix * (min_dist + pad),
        contact.y + iy * (min_dist + pad),
    )
    dx = target.x - p_part.x
    dy = target.y - p_part.y
    placed = translate(rotated, dx, dy)
    for _ in range(24):
        gap = float(placed.distance(standoff))
        inside = container is None or container.contains(placed)
        if gap >= min_dist - 1e-6 and inside:
            break
        placed = translate(placed, ix * min_dist * 0.2, iy * min_dist * 0.2)
    if float(placed.distance(standoff)) < min_dist - 1e-6:
        return None
    if container is not None and not container.contains(placed):
        return None
    seed_x = placed.centroid.x - rc.x
    seed_y = placed.centroid.y - rc.y
    return (seed_x, seed_y, angle)

