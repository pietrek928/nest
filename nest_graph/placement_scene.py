"""Unified placement validity and guidance via evaluate_local_placement."""

import math
from dataclasses import dataclass
from typing import Sequence, Tuple

from shapely.geometry import Point, Polygon
from shapely.geometry.base import BaseGeometry

from .board import board_context_from_geometry
from .geometry import Geometry, GuidanceConfig, evaluate_local_placement


def placement_outside_outer(placed: Geometry, sheet: Polygon) -> bool:
    pb = placed.bounds()
    minx, miny, maxx, maxy = sheet.bounds
    return pb[0] < minx or pb[2] > maxx or pb[1] < miny or pb[3] > maxy


PLACEMENT_EPSILON_RATIO = 0.05


def placement_clearance_epsilon(
    min_dist: float,
    *,
    ratio: float = PLACEMENT_EPSILON_RATIO,
) -> float:
    return max(1e-6, min_dist * ratio)


def _search_radius_for_bounds(
    board_bounds: tuple[float, float, float, float] | None,
) -> float:
    if board_bounds is None:
        return 5.0
    minx, miny, maxx, maxy = board_bounds
    diag = math.hypot(maxx - minx, maxy - miny)
    return max(5.0, 0.5 * diag)


def guidance_config_for_scene(
    min_dist: float,
    *,
    pt_push: Point | None = None,
    board_bounds: tuple[float, float, float, float] | None = None,
    epsilon_ratio: float = PLACEMENT_EPSILON_RATIO,
    for_propose: bool = False,
) -> GuidanceConfig:
    eps = placement_clearance_epsilon(min_dist, ratio=epsilon_ratio)
    cfg = GuidanceConfig()
    cfg.minimum_placing_distance = min_dist + eps
    cfg.search_radius = _search_radius_for_bounds(board_bounds)
    cfg.use_hole_seeking = False
    if for_propose and pt_push is not None:
        cfg.use_target_attractor = True
        cfg.use_gravity = False
        cfg.target_position = (float(pt_push.x), float(pt_push.y))
    else:
        cfg.use_target_attractor = False
        cfg.use_gravity = False
    return cfg


def guidance_config_for_propose(
    pt_push: Point,
    recovery: Point | None = None,
    *,
    min_dist: float = 0.0,
    board_bounds: tuple[float, float, float, float] | None = None,
    search_radius: float | None = None,
    epsilon_ratio: float = PLACEMENT_EPSILON_RATIO,
) -> GuidanceConfig:
    cfg = guidance_config_for_scene(
        min_dist,
        pt_push=pt_push,
        board_bounds=board_bounds,
        epsilon_ratio=epsilon_ratio,
        for_propose=True,
    )
    if search_radius is not None:
        cfg.search_radius = search_radius
    return cfg


@dataclass
class PlacementScene:
    sheet: Polygon
    void_geoms: list[Geometry]
    base_geoms: list[Geometry]
    part: Geometry

    def placed_at(self, coords: Tuple[float, float, float]) -> Geometry:
        return self.part.apply_transform(coords)

    def all_geometries(self, placed: Geometry) -> list[Geometry]:
        return [placed, *self.base_geoms, *self.void_geoms]

    def guidance(
        self,
        placed: Geometry,
        xy: Tuple[float, float],
        config: GuidanceConfig,
    ):
        return evaluate_local_placement(
            0, self.all_geometries(placed), (float(xy[0]), float(xy[1])), config
        )


def build_placement_scene(
    board: BaseGeometry,
    part: Geometry,
    base_geoms: list[Geometry] | None = None,
    *,
    padding: float = 0.0,
    user_holes: tuple[tuple[tuple[float, float], ...], ...] = (),
) -> PlacementScene:
    sheet, void_geoms = board_context_from_geometry(
        board, padding=padding, user_holes=user_holes
    )
    return PlacementScene(sheet, void_geoms, base_geoms or [], part)


def board_placement_valid(
    board: BaseGeometry,
    part: Geometry,
    placed: Geometry,
    *,
    min_dist: float = 0.0,
    config: GuidanceConfig | None = None,
    epsilon_ratio: float = PLACEMENT_EPSILON_RATIO,
) -> bool:
    scene = build_placement_scene(board, part)
    if config is None:
        bounds = None
        if hasattr(board, "bounds"):
            bounds = board.bounds
        config = guidance_config_for_scene(
            min_dist, board_bounds=bounds, epsilon_ratio=epsilon_ratio
        )
    cx, cy = placed.center()
    return is_valid_placement(
        scene, placed, (cx, cy), min_dist, config, epsilon_ratio=epsilon_ratio
    )


def is_valid_placement(
    scene: PlacementScene,
    placed: Geometry,
    xy: Tuple[float, float],
    min_dist: float,
    config: GuidanceConfig,
    *,
    epsilon_ratio: float = PLACEMENT_EPSILON_RATIO,
) -> bool:
    if placement_outside_outer(placed, scene.sheet):
        return False
    g = scene.guidance(placed, xy, config)
    if g.is_penetrating:
        return False
    if min_dist <= 0.0:
        return True
    margin = min_dist + placement_clearance_epsilon(min_dist, ratio=epsilon_ratio)
    return float(g.clearance) >= margin


def unit_vector(v: Tuple[float, float] | Sequence[float]) -> tuple[float, float]:
    x, y = float(v[0]), float(v[1])
    n = math.hypot(x, y)
    if n < 1e-12:
        return (0.0, 0.0)
    return (x / n, y / n)


def guidance_active_direction(
    g,
    *,
    push_xy: Tuple[float, float],
    recovery_xy: Tuple[float, float],
) -> tuple[float, float]:
    if g.is_penetrating:
        ex, ey = g.ejection_vector
        if math.hypot(ex, ey) > 1e-9:
            return unit_vector((ex, ey))
        return unit_vector((
            recovery_xy[0] - push_xy[0],
            recovery_xy[1] - push_xy[1],
        ))
    sx, sy = g.suggested_translation
    sug = unit_vector((sx, sy))
    push_u = unit_vector((
        push_xy[0] - recovery_xy[0],
        push_xy[1] - recovery_xy[1],
    ))
    if sug == (0.0, 0.0):
        return push_u
    return unit_vector((0.7 * sug[0] + 0.3 * push_u[0], 0.7 * sug[1] + 0.3 * push_u[1]))


def guidance_ray_direction_candidates(
    g,
    *,
    push_xy: Tuple[float, float],
    recovery_xy: Tuple[float, float],
) -> list[tuple[float, float]]:
    """Unit directions to score during ray nudge (primary + C++ alternatives)."""
    primary = guidance_active_direction(g, push_xy=push_xy, recovery_xy=recovery_xy)
    candidates: list[tuple[float, float]] = []
    seen: set[tuple[float, float]] = set()

    def add(v: tuple[float, float]) -> None:
        u = unit_vector(v)
        if u == (0.0, 0.0):
            return
        key = (round(u[0], 6), round(u[1], 6))
        if key not in seen:
            seen.add(key)
            candidates.append(u)

    add(primary)
    if g.is_penetrating:
        for alt in g.alternative_translations:
            add((float(alt[0]), float(alt[1])))
    return candidates
