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

from nest_graph.propose.context import placement_free_region, search_region_for_placement
from nest_graph.propose.geometry import ProposeGeometry
from nest_graph.propose.placement_common import (
    CARDINAL_DIRECTIONS,
    _axis_push_from_seed,
    _bottom_left_sort_key,
    _nfp_valid_region,
    _obstacle_parts,
    _perimeter_ring_vertices,
    _placement_safe_zone,
    _propose_valid_at,
    _slide_toward_obstacle,
)

_BOTTOM_LEFT_VERTICES_PER_ANGLE = 8


def _finalize_placement_propositions(
    propositions: list[dict],
    top_n: int,
) -> List[Tuple[float, float, float]]:
    propositions.sort(key=lambda x: x["cost"])
    seen: set[tuple[float, float, float]] = set()
    out: list[tuple[float, float, float]] = []
    for p in propositions:
        c = p["coords"]
        key = (round(c[0], 2), round(c[1], 2), round(c[2], 1))
        if key in seen:
            continue
        seen.add(key)
        out.append(c)
        if len(out) >= top_n:
            break
    return out


def _free_region_seed_point(
    sheet: Polygon,
    base_shape: BaseGeometry,
    min_dist: float,
    pt_push: Point,
) -> Optional[Point]:
    free = placement_free_region(sheet, base_shape, min_dist)
    if free.is_empty:
        return None
    if isinstance(free, MultiPolygon):
        polys = [g for g in free.geoms if isinstance(g, Polygon) and not g.is_empty]
        if not polys:
            return None
        poly = max(polys, key=lambda p: p.area)
        return polylabel(poly, tolerance=1.0)
    if isinstance(free, Polygon):
        if free.contains(pt_push):
            return pt_push
        return free.representative_point()
    rep = free.representative_point()
    return rep if not rep.is_empty else None

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

    propositions: list[dict] = []
    angles = np.linspace(0, 2 * np.pi, num_angles, endpoint=False)

    for angle in angles:
        rotated = rotate(shape_to_place, angle, origin=(0, 0), use_radians=True)
        rc = rotated.centroid
        safe_zone = _placement_safe_zone(region, base_shape, rotated, min_dist)
        if safe_zone.is_empty:
            continue

        for ring in get_shape_exteriors(safe_zone):
            for px, py in _perimeter_ring_vertices(ring):
                dx = px - rc.x
                dy = py - rc.y
                coords = (dx, dy, float(angle))
                if not _propose_valid_at(coords, propose_geom, pt_push):
                    continue
                placed = transform_poly(shape_to_place, coords)
                if not base_shape.is_empty:
                    score = float(base_shape.distance(placed))
                else:
                    score = float(placed.distance(sheet.exterior))
                propositions.append({"coords": coords, "cost": score})

    return _finalize_placement_propositions(propositions, top_n)


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
) -> List[Tuple[float, float, float]]:
    """Slide toward each packed part along the contact normal until min_dist clearance."""
    if base_shape is None or base_shape.is_empty:
        return []

    obstacles = _obstacle_parts(base_shape)
    if not obstacles:
        return []

    propositions: list[dict] = []
    angles = np.linspace(0, 2 * np.pi, num_angles, endpoint=False)

    for angle in angles:
        for obstacle in obstacles:
            coords = _slide_toward_obstacle(
                shape_to_place,
                obstacle,
                float(angle),
                min_dist,
                sheet,
                propose_geom=propose_geom,
            )
            if coords is None:
                continue
            if not _propose_valid_at(coords, propose_geom, pt_push):
                continue
            placed_geom = propose_geom.placed_at(coords)
            score = min(
                placed_geom.distance(base_geom)
                for base_geom in propose_geom.base_geoms
            ) if propose_geom.base_geoms else 10.0
            propositions.append({"coords": coords, "cost": score})

    return _finalize_placement_propositions(propositions, top_n)


def propose_placements_nfp_vertices(
    base_shape: BaseGeometry,
    shape_to_place: Polygon,
    sheet: Polygon,
    min_dist: float,
    *,
    propose_geom: ProposeGeometry,
    pt_push: Point,
    num_angles: int,
    top_n: int,
) -> List[Tuple[float, float, float]]:
    """Place on vertices of sheet_eroded minus union of buffered obstacles (NFP-lite)."""
    if base_shape is None:
        base_shape = Polygon()

    propositions: list[dict] = []
    angles = np.linspace(0, 2 * np.pi, num_angles, endpoint=False)

    for angle in angles:
        rotated = rotate(shape_to_place, angle, origin=(0, 0), use_radians=True)
        rc = rotated.centroid
        valid = _nfp_valid_region(sheet, base_shape, rotated, min_dist)
        if valid.is_empty:
            continue

        for ring in get_shape_exteriors(valid):
            for px, py in _perimeter_ring_vertices(ring):
                dx = px - rc.x
                dy = py - rc.y
                coords = (dx, dy, float(angle))
                if not _propose_valid_at(coords, propose_geom, pt_push):
                    continue
                placed = transform_poly(shape_to_place, coords)
                if not base_shape.is_empty:
                    score = float(base_shape.distance(placed))
                else:
                    score = float(placed.distance(sheet.exterior))
                propositions.append({"coords": coords, "cost": score})

    return _finalize_placement_propositions(propositions, top_n)


def propose_placements_axis_push(
    base_shape: BaseGeometry,
    shape_to_place: Polygon,
    sheet: Polygon,
    min_dist: float,
    *,
    propose_geom: ProposeGeometry,
    pt_push: Point,
    num_angles: int,
    top_n: int,
) -> List[Tuple[float, float, float]]:
    """Push along ±x/±y from a free-region seed until obstacle or sheet contact."""
    if base_shape is None or base_shape.is_empty:
        return []

    seed_pt = _free_region_seed_point(sheet, base_shape, min_dist, pt_push)
    if seed_pt is None:
        return []

    obstacle_geom = base_shape.buffer(min_dist)
    propositions: list[dict] = []
    angles = np.linspace(0, 2 * np.pi, num_angles, endpoint=False)

    for angle in angles:
        rotated = rotate(shape_to_place, angle, origin=(0, 0), use_radians=True)
        rc = rotated.centroid
        seed_dx = seed_pt.x - rc.x
        seed_dy = seed_pt.y - rc.y

        for direction in CARDINAL_DIRECTIONS:
            pushed = _axis_push_from_seed(
                rotated,
                seed_dx,
                seed_dy,
                direction,
                sheet,
                obstacle_geom,
                min_dist,
                propose_geom=propose_geom,
                angle=float(angle),
            )
            if pushed is None:
                continue
            dx, dy = pushed
            coords = (dx, dy, float(angle))
            if not _propose_valid_at(coords, propose_geom, pt_push):
                continue
            placed = transform_poly(shape_to_place, coords)
            score = float(base_shape.distance(placed))
            propositions.append({"coords": coords, "cost": score})

    return _finalize_placement_propositions(propositions, top_n)


def propose_placements_bottom_left(
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
    vertices_per_angle: int = _BOTTOM_LEFT_VERTICES_PER_ANGLE,
) -> List[Tuple[float, float, float]]:
    """Anchor on lowest-left vertices of the placement safe-zone boundary."""
    if base_shape is None:
        base_shape = Polygon()
    region = search_region_for_placement(
        base_shape, sheet, sheet, min_dist,
        use_free_region=use_free_region, border_focus=border_focus,
    )
    if region.is_empty:
        return []

    propositions: list[dict] = []
    angles = np.linspace(0, 2 * np.pi, num_angles, endpoint=False)

    for angle in angles:
        rotated = rotate(shape_to_place, angle, origin=(0, 0), use_radians=True)
        rc = rotated.centroid
        safe_zone = _placement_safe_zone(region, base_shape, rotated, min_dist)
        if safe_zone.is_empty:
            continue

        vertices: list[tuple[float, float]] = []
        for ring in get_shape_exteriors(safe_zone):
            vertices.extend(_perimeter_ring_vertices(ring))
        if not vertices:
            continue

        vertices.sort(key=lambda v: _bottom_left_sort_key(v[0], v[1]))
        for px, py in vertices[:vertices_per_angle]:
            dx = px - rc.x
            dy = py - rc.y
            coords = (dx, dy, float(angle))
            if not _propose_valid_at(coords, propose_geom, pt_push):
                continue
            placed = transform_poly(shape_to_place, coords)
            bl_key = _bottom_left_sort_key(px, py)
            if not base_shape.is_empty:
                tie = float(base_shape.distance(placed))
            else:
                tie = float(placed.distance(sheet.exterior))
            cost = bl_key[0] * 1e6 + bl_key[1] + tie * 1e-3
            propositions.append({"coords": coords, "cost": cost})

    return _finalize_placement_propositions(propositions, top_n)


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
    propositions: list[dict] = []
    angles = np.linspace(0, 2 * np.pi, num_angles, endpoint=False)

    for angle in angles:
        rotated_shape = rotate(shape_to_place, angle, origin=(0, 0), use_radians=True)
        safe_zone = _placement_safe_zone(region, base_shape, rotated_shape, min_dist)
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
            if not _propose_valid_at(coords, propose_geom, pt_push):
                continue
            propositions.append({
                "coords": coords,
                "cost": float(pt.distance(attract)),
            })

    propositions.sort(key=lambda x: x["cost"])
    return [p["coords"] for p in propositions[:top_n]]

