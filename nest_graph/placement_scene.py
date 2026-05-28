"""Unified placement validity and guidance via evaluate_local_placement."""

import math
from dataclasses import dataclass
from typing import Sequence, Tuple

from shapely.geometry import Point, Polygon
from shapely.geometry.base import BaseGeometry

from .board import board_context_from_geometry
from .geometry import Geometry, GuidanceConfig, evaluate_local_placement


def placement_outside_outer(placed: Geometry, sheet: Polygon) -> bool:
    """Fast reject when placed AABB is outside the sheet bounding box."""
    pb = placed.bounds()
    minx, miny, maxx, maxy = sheet.bounds
    return pb[0] < minx or pb[2] > maxx or pb[1] < miny or pb[3] > maxy


def placement_footprint_inside_board(placed: Geometry, board_geom: Geometry) -> bool:
    """All additive vertices of placed inside board solid."""
    return placed.footprint_inside(board_geom)


def footprints_inside_board(
    placed: Sequence[Geometry],
    board_geom: Geometry,
) -> list[bool]:
    """Batch footprint containment vs the same board solid."""
    if not placed:
        return []
    return list(board_geom.footprint_inside_batch(list(placed)))


PLACEMENT_EPSILON_RATIO = 0.05


def placement_clearance_epsilon(
    min_dist: float,
    *,
    ratio: float = PLACEMENT_EPSILON_RATIO,
) -> float:
    return max(1e-6, min_dist * ratio)


def _board_diag(board_bounds: tuple[float, float, float, float] | None) -> float:
    if board_bounds is None:
        return 5.0
    minx, miny, maxx, maxy = board_bounds
    return math.hypot(maxx - minx, maxy - miny)


def _search_radius_for_bounds(
    board_bounds: tuple[float, float, float, float] | None,
) -> float:
    return max(5.0, 0.5 * _board_diag(board_bounds))


def _apply_guidance_scaling(
    cfg: GuidanceConfig,
    min_dist: float,
    board_bounds: tuple[float, float, float, float] | None,
    *,
    diversity_dist_ratio: float = 4.0,
    grid_step_ratio: float = 2.0,
) -> None:
    diag = _board_diag(board_bounds)
    cfg.diversity_distance_threshold = max(
        diversity_dist_ratio * min_dist,
        0.02 * diag,
    )
    cfg.grid_exploration_step = max(
        grid_step_ratio * min_dist,
        0.01 * diag,
    )


def guidance_config_for_scene(
    min_dist: float,
    *,
    pt_push: Point | None = None,
    board_bounds: tuple[float, float, float, float] | None = None,
    epsilon_ratio: float = PLACEMENT_EPSILON_RATIO,
    for_propose: bool = False,
    max_propositions: int = 5,
    use_tight_packing: bool = False,
    squeeze_weight: float = 0.4,
    enable_grid_exploration: bool = False,
    diversity_dist_ratio: float = 4.0,
    grid_step_ratio: float = 2.0,
) -> GuidanceConfig:
    eps = placement_clearance_epsilon(min_dist, ratio=epsilon_ratio)
    cfg = GuidanceConfig()
    cfg.minimum_placing_distance = min_dist + eps
    cfg.search_radius = _search_radius_for_bounds(board_bounds)
    cfg.use_hole_seeking = False
    cfg.max_propositions = max_propositions
    cfg.use_tight_packing = use_tight_packing
    cfg.squeeze_weight = squeeze_weight
    cfg.enable_grid_exploration = enable_grid_exploration
    _apply_guidance_scaling(
        cfg,
        min_dist,
        board_bounds,
        diversity_dist_ratio=diversity_dist_ratio,
        grid_step_ratio=grid_step_ratio,
    )
    if for_propose and pt_push is not None:
        cfg.use_target_attractor = True
        cfg.use_gravity = False
        cfg.target_position = (float(pt_push.x), float(pt_push.y))
    else:
        cfg.use_target_attractor = False
        cfg.use_gravity = False
    return cfg


def guidance_config_for_graph(
    min_dist: float,
    *,
    board_bounds: tuple[float, float, float, float] | None = None,
    epsilon_ratio: float = PLACEMENT_EPSILON_RATIO,
) -> GuidanceConfig:
    """Validity-only guidance for ``make_polygon_graph`` (no propose attractors)."""
    return guidance_config_for_scene(
        min_dist,
        board_bounds=board_bounds,
        epsilon_ratio=epsilon_ratio,
        for_propose=False,
        use_tight_packing=False,
        enable_grid_exploration=False,
    )


def guidance_config_for_propose(
    pt_push: Point,
    recovery: Point | None = None,
    *,
    min_dist: float = 0.0,
    board_bounds: tuple[float, float, float, float] | None = None,
    search_radius: float | None = None,
    epsilon_ratio: float = PLACEMENT_EPSILON_RATIO,
    max_propositions: int = 5,
    use_tight_packing: bool = True,
    squeeze_weight: float = 0.4,
    enable_grid_exploration: bool = True,
    diversity_dist_ratio: float = 4.0,
    grid_step_ratio: float = 2.0,
) -> GuidanceConfig:
    cfg = guidance_config_for_scene(
        min_dist,
        pt_push=pt_push,
        board_bounds=board_bounds,
        epsilon_ratio=epsilon_ratio,
        for_propose=True,
        max_propositions=max_propositions,
        use_tight_packing=use_tight_packing,
        squeeze_weight=squeeze_weight,
        enable_grid_exploration=enable_grid_exploration,
        diversity_dist_ratio=diversity_dist_ratio,
        grid_step_ratio=grid_step_ratio,
    )
    if search_radius is not None:
        cfg.search_radius = search_radius
    return cfg


def proposition_translation(prop) -> tuple[float, float]:
    tx, ty = prop.translation
    return (float(tx), float(ty))


def _proposition_tier(move_type: str) -> str:
    mt = move_type or ""
    if "Ejection" in mt and "Slide" not in mt:
        return "ejection"
    if "Slide" in mt:
        return "slide"
    if "Grid" in mt:
        return "grid"
    return "pack"


def propositions_by_tier(g) -> dict[str, list]:
    tiers: dict[str, list] = {
        "ejection": [],
        "slide": [],
        "pack": [],
        "grid": [],
    }
    for prop in g.propositions:
        tiers[_proposition_tier(prop.move_type)].append(prop)
    for tier in tiers.values():
        tier.sort(key=lambda p: float(p.heuristic_score), reverse=True)
    return tiers


def best_proposition(g, *, prefer_non_grid: bool = True):
    props = list(g.propositions)
    if not props:
        return None
    props.sort(key=lambda p: float(p.heuristic_score), reverse=True)
    if prefer_non_grid:
        for prop in props:
            if "Grid" not in (prop.move_type or ""):
                return prop
    return props[0]


def tiered_propositions(g, *, penetrating: bool | None = None) -> list:
    """Score-ordered props respecting ejection → slide → pack → grid schedule."""
    if penetrating is None:
        penetrating = bool(g.is_penetrating)
    tiers = propositions_by_tier(g)
    if penetrating:
        order = ("ejection", "slide", "pack", "grid")
    else:
        order = ("pack", "slide", "ejection", "grid")
    out: list = []
    for key in order:
        out.extend(tiers[key])
    if not out:
        out = list(g.propositions)
        out.sort(key=lambda p: float(p.heuristic_score), reverse=True)
    return out


@dataclass
class PlacementScene:
    sheet: Polygon
    board_geom: Geometry
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
    board_geom = Geometry.from_shapely(sheet)
    return PlacementScene(sheet, board_geom, void_geoms, base_geoms or [], part)


def placement_scene_for_part(
    sheet: Polygon,
    board_geom: Geometry,
    void_geoms: list[Geometry],
    part: Geometry,
    base_geoms: list[Geometry] | None = None,
) -> PlacementScene:
    return PlacementScene(sheet, board_geom, void_geoms, base_geoms or [], part)


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
    skip_footprint: bool = False,
) -> bool:
    if not skip_footprint and not placement_footprint_inside_board(placed, scene.board_geom):
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
    prop = best_proposition(g)
    if prop is not None:
        tx, ty = proposition_translation(prop)
        if math.hypot(tx, ty) > 1e-9:
            if g.is_penetrating:
                return unit_vector((tx, ty))
            sug = unit_vector((tx, ty))
            push_u = unit_vector((
                push_xy[0] - recovery_xy[0],
                push_xy[1] - recovery_xy[1],
            ))
            if sug == (0.0, 0.0):
                return push_u
            return unit_vector((
                0.7 * sug[0] + 0.3 * push_u[0],
                0.7 * sug[1] + 0.3 * push_u[1],
            ))
    return unit_vector((
        recovery_xy[0] - push_xy[0],
        recovery_xy[1] - push_xy[1],
    ))


def guidance_move_directions(
    g,
    *,
    push_xy: Tuple[float, float],
    recovery_xy: Tuple[float, float],
) -> list[tuple[float, float]]:
    """Unit directions from tiered propositions (primary first)."""
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

    add(guidance_active_direction(g, push_xy=push_xy, recovery_xy=recovery_xy))
    for prop in tiered_propositions(g):
        add(proposition_translation(prop))
    return candidates


def guidance_ray_direction_candidates(
    g,
    *,
    push_xy: Tuple[float, float],
    recovery_xy: Tuple[float, float],
) -> list[tuple[float, float]]:
    """Unit directions to score during ray nudge (primary + proposition menu)."""
    return guidance_move_directions(g, push_xy=push_xy, recovery_xy=recovery_xy)
