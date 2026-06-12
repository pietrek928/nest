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


def _geometry_polygon(geom: Geometry) -> Polygon:
    coords = [(float(x), float(y)) for x, y in geom.vertices()]
    if len(coords) >= 2 and coords[0] == coords[-1]:
        coords = coords[:-1]
    return Polygon(coords)


def _geometry_centroid(geom: Geometry) -> tuple[float, float]:
    c = _geometry_polygon(geom).centroid
    return (float(c.x), float(c.y))


def _ring_geom_from_standoff(standoff: BaseGeometry) -> Optional[Geometry]:
    ring = _nest_outline_ring(standoff)
    if not hasattr(ring, "coords"):
        return None
    coords = list(ring.coords)
    if len(coords) < 2:
        return None
    if coords[0] == coords[-1]:
        coords = coords[:-1]
    return Geometry.from_ring(coords)


def _standoff_gap(placed: Geometry, standoff: BaseGeometry) -> float:
    ring_geom = _ring_geom_from_standoff(standoff)
    if ring_geom is not None:
        return placed.standoff_distance(ring_geom)
    if isinstance(standoff, Polygon):
        return placed.standoff_distance(Geometry.from_ring(
            list(standoff.exterior.coords)[:-1],
        ))
    return float(_geometry_polygon(placed).distance(standoff))
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
    *,
    propose_geom: Optional[ProposeGeometry] = None,
) -> Optional[Tuple[float, float, float]]:
    """Slide the part along the contact normal until it kisses this obstacle at min_dist."""
    if propose_geom is not None:
        ring_coords = list(obstacle.exterior.coords)
        if len(ring_coords) >= 2 and ring_coords[0] == ring_coords[-1]:
            ring_coords = ring_coords[:-1]
        obstacle_ring = Geometry.from_ring(ring_coords)
        rotated = propose_geom.part.rotate(angle)
        md = rotated.standoff_min_distance(obstacle_ring)
        p_part_x, p_part_y = md.closest_a
        p_obs_x, p_obs_y = md.closest_b
        dx = p_part_x - p_obs_x
        dy = p_part_y - p_obs_y
        dist = math.hypot(dx, dy)
        if dist < 1e-9:
            return None
        inward = (dx / dist, dy / dist)
        margin = min_dist + placement_clearance_epsilon(
            min_dist, ratio=PLACEMENT_EPSILON_RATIO,
        )
        return _snap_coords_along_exterior_geom(
            propose_geom.part,
            obstacle_ring,
            Point(p_obs_x, p_obs_y),
            inward,
            angle,
            margin,
            standoff=obstacle.exterior,
            board_geom=propose_geom.board_geom,
            standoff_pad=0.0,
        )

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


def _axis_placement_valid_geom(
    placed: Geometry,
    board_geom: Geometry,
    standoff: BaseGeometry,
    obstacle_geoms: list[Geometry],
    min_dist: float,
) -> bool:
    if not placed.footprint_inside(board_geom):
        return False
    for obstacle in obstacle_geoms:
        if placed.intersects(obstacle):
            return False
        if placed.distance(obstacle) < min_dist - 1e-6:
            return False
    ring_geom = _ring_geom_from_standoff(standoff)
    if ring_geom is not None:
        if placed.standoff_distance(ring_geom) < min_dist - 1e-6:
            return False
    elif _standoff_gap(placed, standoff) < min_dist - 1e-6:
        return False
    return True


def _axis_push_from_seed_geom(
    part_geom: Geometry,
    angle: float,
    seed_dx: float,
    seed_dy: float,
    direction: Tuple[float, float],
    board_geom: Geometry,
    standoff: BaseGeometry,
    obstacle_geoms: list[Geometry],
    min_dist: float,
    max_step: float,
) -> Optional[Tuple[float, float]]:
    """Push from seed along direction until contact; return total (dx, dy) from rotated origin."""
    dir_x, dir_y = direction
    rotated = part_geom.rotate(angle)
    placed = rotated.translate(seed_dx, seed_dy)
    if not _axis_placement_valid_geom(
        placed, board_geom, standoff, obstacle_geoms, min_dist,
    ):
        return None

    lo, hi = 0.0, max_step
    best = 0.0
    for _ in range(32):
        mid = (lo + hi) * 0.5
        trial = rotated.translate(
            seed_dx + dir_x * mid,
            seed_dy + dir_y * mid,
        )
        if _axis_placement_valid_geom(
            trial, board_geom, standoff, obstacle_geoms, min_dist,
        ):
            best = mid
            lo = mid
        else:
            hi = mid

    return (seed_dx + dir_x * best, seed_dy + dir_y * best)


def _axis_push_from_seed(
    rotated: Polygon,
    seed_dx: float,
    seed_dy: float,
    direction: Tuple[float, float],
    sheet: Polygon,
    obstacle_geom: BaseGeometry,
    min_dist: float,
    *,
    propose_geom: Optional[ProposeGeometry] = None,
    angle: float = 0.0,
) -> Optional[Tuple[float, float]]:
    """Push from seed along direction until contact; return total (dx, dy) from rotated origin."""
    dir_x, dir_y = direction
    min_x, min_y, max_x, max_y = sheet.bounds
    max_step = math.hypot(max_x - min_x, max_y - min_y)

    if propose_geom is not None and not obstacle_geom.is_empty:
        obstacle_geoms = [Geometry.from_shapely(obstacle_geom)]
        return _axis_push_from_seed_geom(
            propose_geom.part,
            angle,
            seed_dx,
            seed_dy,
            direction,
            propose_geom.board_geom,
            propose_geom.sheet.exterior,
            obstacle_geoms,
            min_dist,
            max_step,
        )

    placed = translate(rotated, seed_dx, seed_dy)
    if not _axis_placement_valid(placed, sheet, obstacle_geom, min_dist):
        return None

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

def _exterior_segment_lengths(sheet: Polygon) -> list[float]:
    coords = list(sheet.exterior.coords)
    seg_lens: list[float] = []
    for i in range(len(coords) - 1):
        p0, p1 = coords[i], coords[i + 1]
        seg_lens.append(math.hypot(p1[0] - p0[0], p1[1] - p0[1]))
    return seg_lens


def _segment_index_at_point(sheet: Polygon, contact: Point) -> int:
    coords = list(sheet.exterior.coords)
    best_i = 0
    best_dist = float("inf")
    for i in range(len(coords) - 1):
        seg = LineString([coords[i], coords[i + 1]])
        dist = seg.distance(contact)
        if dist < best_dist:
            best_dist = dist
            best_i = i
    return best_i


def _edge_tangent_angles(sheet: Polygon, segment_index: int) -> list[float]:
    coords = list(sheet.exterior.coords)
    if segment_index < 0 or segment_index >= len(coords) - 1:
        return []
    p0, p1 = coords[segment_index], coords[segment_index + 1]
    tang = math.atan2(p1[1] - p0[1], p1[0] - p0[0])
    return [tang + k * math.pi / 2.0 for k in range(4)]


def _edge_inward_at_point(
    sheet: Polygon,
    contact: Point,
) -> tuple[Point, tuple[float, float]] | None:
    """Return anchor on the containing edge segment and unit inward normal."""
    coords = list(sheet.exterior.coords)
    best_anchor: Point | None = None
    best_inward: tuple[float, float] | None = None
    best_dist = float("inf")
    for i in range(len(coords) - 1):
        p0 = coords[i]
        p1 = coords[i + 1]
        seg = LineString([p0, p1])
        dist = seg.distance(contact)
        if dist >= best_dist:
            continue
        dx = p1[0] - p0[0]
        dy = p1[1] - p0[1]
        seg_len = math.hypot(dx, dy)
        if seg_len < 1e-9:
            continue
        anchor = seg.interpolate(seg.project(contact))
        nx, ny = -dy / seg_len, dx / seg_len
        probe = Point(anchor.x + nx * 1e-4, anchor.y + ny * 1e-4)
        if not sheet.contains(probe):
            nx, ny = -nx, -ny
        best_dist = dist
        best_anchor = anchor
        best_inward = (nx, ny)
    if best_anchor is None or best_inward is None:
        return None
    return best_anchor, best_inward


def _exterior_anchor_points(geom: BaseGeometry, samples_per_edge: int) -> list[Point]:
    anchors: list[Point] = []
    for line in get_shape_exteriors(geom):
        if line.length <= 0:
            continue
        coords = list(line.coords)
        seg_lens: list[float] = []
        seg_lines: list[LineString] = []
        for i in range(len(coords) - 1):
            p0, p1 = coords[i], coords[i + 1]
            sl = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
            if sl > 1e-9:
                seg_lens.append(sl)
                seg_lines.append(LineString([p0, p1]))
        total = sum(seg_lens) or 1.0
        n_seg = len(seg_lines) or 1
        for sl, seg in zip(seg_lens, seg_lines, strict=True):
            n = max(2, int(round(samples_per_edge * sl / total * n_seg)))
            for t in np.linspace(0.02, 0.98, n):
                anchors.append(seg.interpolate(t, normalized=True))
    return anchors


def _snap_coords_along_exterior_geom(
    part_geom: Geometry,
    boundary_ring_geom: Geometry,
    contact: Point,
    inward: Tuple[float, float],
    angle: float,
    min_dist: float,
    *,
    standoff: BaseGeometry,
    board_geom: Optional[Geometry] = None,
    standoff_pad: Optional[float] = None,
) -> Optional[Tuple[float, float, float]]:
    """C++-backed snap along an exterior ring."""
    ix, iy = inward
    rotated = part_geom.rotate(angle)
    rotated_poly = _geometry_polygon(rotated)
    rcx, rcy = _geometry_centroid(rotated)
    if standoff_pad is None:
        pad = _rotated_max_dim(rotated_poly)
    else:
        pad = standoff_pad
    _, p_part = nearest_points(standoff, rotated_poly)
    p_part_x, p_part_y = float(p_part.x), float(p_part.y)
    target_x = contact.x + ix * (min_dist + pad)
    target_y = contact.y + iy * (min_dist + pad)
    dx = target_x - p_part_x
    dy = target_y - p_part_y
    placed = rotated.translate(dx, dy)
    ilen = math.hypot(ix, iy)
    if ilen < 1e-9:
        return None
    in_x, in_y = ix / ilen, iy / ilen
    rb = rotated_poly.bounds
    max_cast = max(rb[2] - rb[0], rb[3] - rb[1]) * 2.0 + min_dist * 4.0
    for _ in range(_AXIS_PUSH_STEPS):
        gap = placed.standoff_distance(boundary_ring_geom)
        inside = board_geom is None or placed.footprint_inside(board_geom)
        if gap >= min_dist - 1e-6 and inside:
            break
        deficit = min_dist - gap
        if deficit > 1e-9:
            placed = placed.translate(in_x * deficit, in_y * deficit)
        else:
            cast = placed.cast_slide(
                [boundary_ring_geom],
                (-in_x, -in_y),
                max_cast,
            )
            step = min_dist * 0.2
            if cast.intersects_path and cast.t_entry < step:
                step = max(cast.t_entry * 0.5, min_dist * 0.05)
            placed = placed.translate(in_x * step, in_y * step)
    if placed.standoff_distance(boundary_ring_geom) < min_dist - 1e-6:
        return None
    if board_geom is not None and not placed.footprint_inside(board_geom):
        return None
    pcx, pcy = _geometry_centroid(placed)
    return (pcx - rcx, pcy - rcy, angle)


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
    propose_geom: Optional[ProposeGeometry] = None,
) -> Optional[Tuple[float, float, float]]:
    """Snap rotated part so nearest point on boundary sits at min_dist along inward."""
    if propose_geom is not None and isinstance(boundary, Polygon):
        if boundary is propose_geom.sheet:
            boundary_ring_geom = propose_geom.boundary_ring_geom
        else:
            ring_coords = list(boundary.exterior.coords)
            if len(ring_coords) >= 2 and ring_coords[0] == ring_coords[-1]:
                ring_coords = ring_coords[:-1]
            boundary_ring_geom = Geometry.from_ring(ring_coords)
        board_geom = propose_geom.board_geom if container is not None else None
        standoff = boundary.exterior if isinstance(boundary, Polygon) else boundary
        return _snap_coords_along_exterior_geom(
            propose_geom.part,
            boundary_ring_geom,
            contact,
            inward,
            angle,
            min_dist,
            standoff=standoff,
            board_geom=board_geom,
            standoff_pad=standoff_pad,
        )

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
    for _ in range(_AXIS_PUSH_STEPS):
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


def _nest_outline_ring(outline: BaseGeometry):
    if isinstance(outline, Polygon):
        return outline.exterior
    if isinstance(outline, (LineString, LinearRing)):
        return outline
    if hasattr(outline, "exterior"):
        return outline.exterior
    return outline.boundary


def _outline_ring_geom(outline: BaseGeometry) -> Optional[Geometry]:
    ring = _nest_outline_ring(outline)
    if not hasattr(ring, "coords"):
        return None
    coords = list(ring.coords)
    if len(coords) < 2:
        return None
    if coords[0] == coords[-1]:
        coords = coords[:-1]
    return Geometry.from_ring(coords)


def outline_standoff_distance(poly, outline: BaseGeometry) -> float:
    if isinstance(poly, Geometry):
        ring_geom = _outline_ring_geom(outline)
        if ring_geom is not None:
            return poly.standoff_distance(ring_geom)
        return _standoff_gap(poly, _nest_outline_ring(outline))
    return float(poly.distance(_nest_outline_ring(outline)))


def outline_kiss_tolerance(min_dist: float, *, scale: float = 2.0) -> float:
    return max(min_dist * scale, 1e-6)


def outline_kiss_ok(
    poly,
    outline: BaseGeometry,
    min_dist: float,
    *,
    scale: float = 2.0,
) -> bool:
    tol = outline_kiss_tolerance(min_dist, scale=scale)
    err = abs(outline_standoff_distance(poly, outline) - min_dist)
    return err <= tol


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


def _inward_at_contact(
    boundary: BaseGeometry,
    contact: Point,
) -> tuple[Point, tuple[float, float]]:
    if isinstance(boundary, Polygon):
        edge_info = _edge_inward_at_point(boundary, contact)
        if edge_info is not None:
            return edge_info
    interior = boundary.representative_point()
    ox = contact.x - interior.x
    oy = contact.y - interior.y
    dist = math.hypot(ox, oy)
    if dist < 1e-9:
        return contact, (0.0, 1.0)
    return contact, (ox / dist, oy / dist)


def _angles_for_edge_contact(
    boundary: BaseGeometry,
    contact: Point,
    num_angles: int,
) -> list[float]:
    angles = list(np.linspace(0, 2 * np.pi, num_angles, endpoint=False))
    if isinstance(boundary, Polygon):
        seg_i = _segment_index_at_point(boundary, contact)
        angles.extend(_edge_tangent_angles(boundary, seg_i))
    return angles


def _select_stratified_by_segment(
    boundary: Polygon,
    propositions: list[dict],
    top_n: int,
    *,
    anchor_key: str = "anchor",
) -> list[dict]:
    """Reserve top_n slots per exterior segment proportional to edge length."""
    if not propositions:
        return []
    seg_lens = _exterior_segment_lengths(boundary)
    total = sum(seg_lens) or 1.0
    by_seg: dict[int, list[dict]] = {}
    for prop in propositions:
        anchor = prop[anchor_key]
        seg_i = _segment_index_at_point(boundary, anchor)
        by_seg.setdefault(seg_i, []).append(prop)

    picked: list[dict] = []
    for seg_i, seg_len in enumerate(seg_lens):
        bucket = sorted(by_seg.get(seg_i, []), key=lambda row: row["cost"])
        quota = max(2, int(round(top_n * seg_len / total)))
        picked.extend(bucket[:quota])

    seen: set[tuple[float, float, float]] = set()
    out: list[dict] = []
    for prop in sorted(picked, key=lambda row: row["cost"]):
        coords = prop["coords"]
        key = (round(coords[0], 2), round(coords[1], 2), round(coords[2], 1))
        if key in seen:
            continue
        seen.add(key)
        out.append(prop)
        if len(out) >= top_n:
            return out

    for prop in sorted(propositions, key=lambda row: row["cost"]):
        coords = prop["coords"]
        key = (round(coords[0], 2), round(coords[1], 2), round(coords[2], 1))
        if key in seen:
            continue
        seen.add(key)
        out.append(prop)
        if len(out) >= top_n:
            break
    return out


def _finalize_edge_propositions(
    propositions: list[dict],
    boundary: BaseGeometry,
    top_n: int,
) -> list[tuple[float, float, float]]:
    if isinstance(boundary, Polygon):
        selected = _select_stratified_by_segment(boundary, propositions, top_n)
    else:
        selected = sorted(propositions, key=lambda row: row["cost"])
    seen: set[tuple[float, float, float]] = set()
    out: list[tuple[float, float, float]] = []
    for prop in selected:
        coords = prop["coords"]
        key = (round(coords[0], 2), round(coords[1], 2), round(coords[2], 1))
        if key in seen:
            continue
        seen.add(key)
        out.append(coords)
        if len(out) >= top_n:
            break
    return out

