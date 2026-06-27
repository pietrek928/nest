import math
from typing import Optional, Tuple

from shapely import LineString, LinearRing, Point, Polygon
from shapely.affinity import rotate, translate
from shapely.geometry.base import BaseGeometry
from shapely.ops import nearest_points

from nest_graph.geometry import Geometry
from nest_graph.placement_clearance import PLACEMENT_EPSILON_RATIO, placement_clearance_epsilon
from nest_graph.utils import get_shape_exteriors

from nest_graph.propose.geometry import ProposeGeometry
from nest_graph.propose.placement_perimeter import edge_inward_at_point

_AXIS_PUSH_STEPS = 24


def _geometry_polygon(geom: Geometry) -> Polygon:
    coords = [(float(x), float(y)) for x, y in geom.vertices()]
    if len(coords) >= 2 and coords[0] == coords[-1]:
        coords = coords[:-1]
    return Polygon(coords)


def _geom_max_dim(geom: Geometry) -> float:
    minx, miny, maxx, maxy = geom.aabb()
    return max(maxx - minx, maxy - miny) / 2.0


def standoff_gap(placed: Geometry, standoff: BaseGeometry) -> float:
    ring_geom = outline_ring_geom(standoff)
    if ring_geom is not None:
        return placed.standoff_distance(ring_geom)
    if isinstance(standoff, Polygon):
        return placed.standoff_distance(Geometry.from_ring(
            list(standoff.exterior.coords)[:-1],
        ))
    return float(_geometry_polygon(placed).distance(standoff))


def _nest_outline_ring(outline: BaseGeometry):
    if isinstance(outline, Polygon):
        return outline.exterior
    if isinstance(outline, (LineString, LinearRing)):
        return outline
    if hasattr(outline, "exterior"):
        return outline.exterior
    return outline.boundary


def outline_ring_geom(
    outline: BaseGeometry,
    *,
    propose_geom: Optional[ProposeGeometry] = None,
) -> Optional[Geometry]:
    if propose_geom is not None and outline is propose_geom.sheet:
        return propose_geom.boundary_ring_geom
    if isinstance(outline, (LineString, LinearRing)):
        coords = list(outline.coords)
        if len(coords) < 2:
            return None
        if coords[0] == coords[-1]:
            coords = coords[:-1]
        if len(coords) < 2:
            return None
        return Geometry.from_ring(coords)
    rings: list[list[tuple[float, float]]] = []
    for line in get_shape_exteriors(outline):
        coords = list(line.coords)
        if len(coords) < 2:
            continue
        if coords[0] == coords[-1]:
            coords = coords[:-1]
        if len(coords) >= 2:
            rings.append([(float(x), float(y)) for x, y in coords])
    if not rings:
        ring = _nest_outline_ring(outline)
        if not hasattr(ring, "coords"):
            return None
        coords = list(ring.coords)
        if len(coords) < 2:
            return None
        if coords[0] == coords[-1]:
            coords = coords[:-1]
        if len(coords) < 2:
            return None
        return Geometry.from_ring(coords)
    if len(rings) == 1:
        return Geometry.from_ring(rings[0])
    return Geometry.from_rings(rings)


def outline_standoff_distance(poly, outline: BaseGeometry) -> float:
    if isinstance(poly, Geometry):
        ring_geom = outline_ring_geom(outline)
        if ring_geom is not None:
            return poly.standoff_distance(ring_geom)
        return standoff_gap(poly, _nest_outline_ring(outline))
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


def _snap_coords_along_exterior_geom(
    part_geom: Geometry,
    boundary_ring_geom: Geometry,
    contact: Point,
    inward: Tuple[float, float],
    angle: float,
    min_dist: float,
    *,
    board_geom: Optional[Geometry] = None,
    standoff_pad: Optional[float] = None,
) -> Optional[Tuple[float, float, float]]:
    """C++-backed snap along an exterior ring."""
    ix, iy = inward
    rotated = part_geom.rotate(angle)
    rcx, rcy = rotated.centroid()
    rcx, rcy = float(rcx), float(rcy)
    if standoff_pad is None:
        pad = _geom_max_dim(rotated)
    else:
        pad = standoff_pad
    md = rotated.standoff_min_distance(boundary_ring_geom)
    p_part_x, p_part_y = float(md.closest_a[0]), float(md.closest_a[1])
    target_x = contact.x + ix * (min_dist + pad)
    target_y = contact.y + iy * (min_dist + pad)
    dx = target_x - p_part_x
    dy = target_y - p_part_y
    placed = rotated.translate(dx, dy)
    ilen = math.hypot(ix, iy)
    if ilen < 1e-9:
        return None
    in_x, in_y = ix / ilen, iy / ilen
    minx, miny, maxx, maxy = rotated.aabb()
    max_cast = max(maxx - minx, maxy - miny) * 2.0 + min_dist * 4.0
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
    pcx, pcy = placed.centroid()
    return (float(pcx) - rcx, float(pcy) - rcy, angle)


def snap_coords_along_exterior(
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
    boundary_ring_geom: Optional[Geometry] = None,
) -> Optional[Tuple[float, float, float]]:
    """Snap rotated part so nearest point on boundary sits at min_dist along inward."""
    if propose_geom is not None:
        ring_geom = boundary_ring_geom
        if ring_geom is None:
            ring_geom = outline_ring_geom(boundary, propose_geom=propose_geom)
        if ring_geom is None:
            return None
        board_geom = propose_geom.board_geom if container is not None else None
        return _snap_coords_along_exterior_geom(
            propose_geom.part,
            ring_geom,
            contact,
            inward,
            angle,
            min_dist,
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


def inward_at_contact(
    boundary: BaseGeometry,
    contact: Point,
) -> tuple[Point, tuple[float, float]]:
    if isinstance(boundary, Polygon):
        edge_info = edge_inward_at_point(boundary, contact)
        if edge_info is not None:
            return edge_info
    interior = boundary.representative_point()
    ox = contact.x - interior.x
    oy = contact.y - interior.y
    dist = math.hypot(ox, oy)
    if dist < 1e-9:
        return contact, (0.0, 1.0)
    return contact, (ox / dist, oy / dist)


def slide_toward_obstacle(
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
        obstacle_ring = outline_ring_geom(obstacle)
        if obstacle_ring is None:
            return None
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
    return snap_coords_along_exterior(
        shape_to_place,
        obstacle,
        p_obs,
        inward,
        angle,
        margin,
        container=sheet,
        standoff_pad=0.0,
    )
