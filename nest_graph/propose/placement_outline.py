import math
from typing import TYPE_CHECKING, Optional, Tuple

from shapely import LineString, LinearRing, Point, Polygon
from shapely.geometry.base import BaseGeometry

from nest_graph.geometry import Geometry, snap_pose_to_ring
from nest_graph.placement_clearance import PLACEMENT_EPSILON_RATIO, placement_clearance_epsilon
from nest_graph.utils import get_shape_exteriors

from nest_graph.propose.placement_perimeter import edge_inward_at_point

if TYPE_CHECKING:
    from nest_graph.propose.geometry import ProposeGeometry


def _geometry_polygon(geom: Geometry) -> Polygon:
    coords = [(float(x), float(y)) for x, y in geom.vertices()]
    if len(coords) >= 2 and coords[0] == coords[-1]:
        coords = coords[:-1]
    return Polygon(coords)


def standoff_gap(placed: Geometry, standoff: BaseGeometry) -> float:
    """Alias of outline_standoff_distance for ring/outline targets."""
    return outline_standoff_distance(placed, standoff)


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
    propose_geom: Optional["ProposeGeometry"] = None,
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


def outline_standoff_distance(
    poly,
    outline: BaseGeometry,
    *,
    propose_geom: Optional["ProposeGeometry"] = None,
    ring: Optional[Geometry] = None,
) -> float:
    """Edge-to-edge standoff to the exterior outline ring (cached when available)."""
    ring_geom = ring
    if ring_geom is None:
        ring_geom = outline_ring_geom(outline, propose_geom=propose_geom)
    if isinstance(poly, Geometry):
        if ring_geom is not None:
            return poly.standoff_distance(ring_geom)
        return float(_geometry_polygon(poly).distance(_nest_outline_ring(outline)))
    return float(poly.distance(_nest_outline_ring(outline)))


def outline_kiss_tolerance(min_dist: float, *, scale: float = 2.0) -> float:
    return max(min_dist * scale, 1e-6)


def outline_kiss_ok(
    poly,
    outline: BaseGeometry,
    min_dist: float,
    *,
    scale: float = 2.0,
    propose_geom: Optional["ProposeGeometry"] = None,
    ring: Optional[Geometry] = None,
) -> bool:
    tol = outline_kiss_tolerance(min_dist, scale=scale)
    err = abs(
        outline_standoff_distance(
            poly, outline, propose_geom=propose_geom, ring=ring,
        )
        - min_dist
    )
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
    """C++ `snap_pose_to_ring` (24-step Geometry loop)."""
    return snap_pose_to_ring(
        part_geom,
        boundary_ring_geom,
        (float(contact.x), float(contact.y)),
        (float(inward[0]), float(inward[1])),
        float(angle),
        float(min_dist),
        board=board_geom,
        standoff_pad=standoff_pad,
    )


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
    """Snap rotated part so nearest point on boundary sits at min_dist along inward.

    Requires ``propose_geom`` (Geometry-backed path). Shapely twin removed (A0).
    """
    if propose_geom is None:
        raise TypeError(
            "snap_coords_along_exterior requires propose_geom "
            "(Shapely fallback removed; use ProposeGeometry)",
        )
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
    if propose_geom is None:
        raise TypeError(
            "slide_toward_obstacle requires propose_geom "
            "(Shapely fallback removed; use ProposeGeometry)",
        )
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
