import math
from typing import Optional, Tuple

from shapely.geometry.base import BaseGeometry

from nest_graph.geometry import Geometry

from nest_graph.propose.geometry import ProposeGeometry
from nest_graph.propose.placement_outline import _outline_ring_geom, _standoff_gap

CARDINAL_DIRECTIONS: tuple[tuple[float, float], ...] = (
    (-1.0, 0.0),
    (1.0, 0.0),
    (0.0, -1.0),
    (0.0, 1.0),
)


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
    ring_geom = _outline_ring_geom(standoff)
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
    seed_dx: float,
    seed_dy: float,
    direction: Tuple[float, float],
    obstacle_geom: BaseGeometry,
    min_dist: float,
    *,
    propose_geom: ProposeGeometry,
    angle: float,
) -> Optional[Tuple[float, float]]:
    """Push from seed along direction until contact; return total (dx, dy) from rotated origin."""
    if obstacle_geom is None or obstacle_geom.is_empty:
        return None
    min_x, min_y, max_x, max_y = propose_geom.sheet.bounds
    max_step = math.hypot(max_x - min_x, max_y - min_y)
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
