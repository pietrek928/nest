import math
from enum import StrEnum
from typing import List, Sequence, Tuple

import numpy as np
from shapely import Point, Polygon
from shapely.geometry.base import BaseGeometry

from nest_graph.config import ProposeConfig
from nest_graph.placement_scene import (
    guidance_config_for_board_edge_anchor,
    guidance_config_for_propose,
    guidance_kwargs_for_propose,
    proposition_translation,
    tiered_propositions,
)

from nest_graph.propose.geometry import ProposeGeometry
from nest_graph.propose.placements_edge import sample_placement_points_ribbon


class GuidanceMoveType(StrEnum):
    EXACT_NEIGHBOR_SNAP = "Exact Neighbor Snap"
    VERTEX_CORNER_MATCH = "Vertex Corner Match"
    CORNER_MATCH = "Corner Match"
    EXACT_GRAVITY_DOCK = "Exact Gravity Dock"
    SAFE_HOLE_SEEK = "Safe Hole Seek"
    HOLE_SEEK = "Hole Seek"
    LONG_RANGE_GRAVITY_DOCK = "Long Range Gravity Dock"
    FLOOR_WALK = "Floor Walk"


_CAST_MOVE_TYPES: frozenset[GuidanceMoveType] = frozenset({
    GuidanceMoveType.EXACT_NEIGHBOR_SNAP,
    GuidanceMoveType.VERTEX_CORNER_MATCH,
    GuidanceMoveType.CORNER_MATCH,
    GuidanceMoveType.EXACT_GRAVITY_DOCK,
    GuidanceMoveType.SAFE_HOLE_SEEK,
    GuidanceMoveType.LONG_RANGE_GRAVITY_DOCK,
    GuidanceMoveType.FLOOR_WALK,
})

_PROPOSITION_PRIORITIES: tuple[tuple[GuidanceMoveType, float], ...] = (
    (GuidanceMoveType.EXACT_NEIGHBOR_SNAP, 100.0),
    (GuidanceMoveType.VERTEX_CORNER_MATCH, 95.0),
    (GuidanceMoveType.CORNER_MATCH, 88.0),
    (GuidanceMoveType.FLOOR_WALK, 85.0),
    (GuidanceMoveType.EXACT_GRAVITY_DOCK, 80.0),
    (GuidanceMoveType.LONG_RANGE_GRAVITY_DOCK, 80.0),
    (GuidanceMoveType.SAFE_HOLE_SEEK, 70.0),
    (GuidanceMoveType.HOLE_SEEK, 70.0),
)


def is_cast_move(move_type: str) -> bool:
    mt = move_type or ""
    return any(tag in mt for tag in _CAST_MOVE_TYPES) or "Exact" in mt


def _proposition_sort_key(prop) -> tuple[float, float]:
    mt = prop.move_type or ""
    priority = 0.0
    for tag, score in _PROPOSITION_PRIORITIES:
        if tag in mt:
            priority = score
            break
    return (priority, float(prop.heuristic_score))


def sorted_guidance_propositions(g) -> list:
    props = tiered_propositions(g)
    props.sort(key=_proposition_sort_key, reverse=True)
    return props


def _merged_guidance_propositions(
    propose_geom: ProposeGeometry,
    placed,
    xy: tuple[float, float],
    pt_push: Point,
    theta: float,
) -> tuple[list, object]:
    """Dual-pass guidance: tight cast menu plus attractor pass when not border-focused."""
    if propose_geom._border_focus:
        g = propose_geom.placement_guidance(
            placed, xy, pt_push, target_angle_rad=theta, border_focus=True,
        )
        return sorted_guidance_propositions(g), g

    gkw = guidance_kwargs_for_propose(propose_geom._propose_cfg)
    tight_cfg = guidance_config_for_propose(
        pt_push,
        min_dist=propose_geom._min_dist,
        board_bounds=propose_geom._board_bounds,
        epsilon_ratio=propose_geom._epsilon_ratio,
        border_focus=False,
        target_angle_rad=theta,
        use_tight_packing=True,
        enable_grid_exploration=True,
        **{
            k: v
            for k, v in gkw.items()
            if k not in ("use_tight_packing", "enable_grid_exploration")
        },
    )
    tight_cfg.use_target_attractor = False
    tight_cfg.use_gravity = True

    g_tight = propose_geom.placement_guidance(
        placed, xy, pt_push, guidance_cfg=tight_cfg,
    )
    g_attr = propose_geom.placement_guidance(
        placed, xy, pt_push, target_angle_rad=theta, border_focus=False,
    )

    merged: list = []
    seen: set[tuple] = set()
    for g in (g_tight, g_attr):
        for prop in sorted_guidance_propositions(g):
            tx, ty = proposition_translation(prop)
            key = (
                round(tx, 6),
                round(ty, 6),
                round(float(prop.rotation_rad), 6),
                prop.move_type or "",
            )
            if key in seen:
                continue
            seen.add(key)
            merged.append(prop)
    merged.sort(key=_proposition_sort_key, reverse=True)
    g_state = g_tight if g_tight.is_penetrating else g_attr
    return merged, g_state


def _normalize_angle(theta: float) -> float:
    while theta > math.pi:
        theta -= 2 * math.pi
    while theta < -math.pi:
        theta += 2 * math.pi
    return theta


def propose_placements_guidance_walk(
    base_shape: BaseGeometry,
    shape_to_place: Polygon,
    sheet: Polygon,
    pt_push: Point,
    propose_geom: ProposeGeometry,
    *,
    min_dist: float,
    num_seeds: int = 12,
    walk_steps: int = 5,
    step_scale: float = 0.15,
    top_n: int = 8,
) -> List[Tuple[float, float, float]]:
    """Seed placements by stepping along C++ guidance propositions in free space."""
    seeds = list(sample_placement_points_ribbon(base_shape, shape_to_place, sheet, min_dist))
    if not seeds:
        seeds = [sheet.centroid if not base_shape.is_empty else pt_push]
    if len(seeds) > num_seeds:
        step = len(seeds) / num_seeds
        seeds = [seeds[int(i * step)] for i in range(num_seeds)]

    angle_grid = np.linspace(0, 2 * np.pi, max(num_seeds, 4), endpoint=False)
    out: list[tuple[float, float, float]] = []
    seen: set[tuple[float, float, float]] = set()
    for i, seed in enumerate(seeds):
        x, y = float(seed.x), float(seed.y)
        theta = float(angle_grid[i % len(angle_grid)])
        for _ in range(walk_steps):
            placed = propose_geom.placed_at((x, y, theta))
            props, g = _merged_guidance_propositions(
                propose_geom, placed, (x, y), pt_push, theta,
            )
            props = props[:3]
            if not props:
                break
            moved = False
            for prop in props:
                tx, ty = proposition_translation(prop)
                mag = math.hypot(tx, ty)
                if mag < 1e-9:
                    continue
                if not g.is_penetrating and is_cast_move(prop.move_type or ""):
                    step_len = mag
                else:
                    step_len = step_scale * max(min_dist, 1e-4)
                    if not g.is_penetrating:
                        step_len = step_scale * mag
                nx = x + tx / mag * step_len
                ny = y + ty / mag * step_len
                ntheta = theta
                if abs(float(prop.rotation_rad)) > 1e-6:
                    delta = float(prop.rotation_rad) - theta
                    while delta > np.pi:
                        delta -= 2 * np.pi
                    while delta < -np.pi:
                        delta += 2 * np.pi
                    ntheta = theta + delta * (1.0 if is_cast_move(prop.move_type or "") else 0.2)
                trial = propose_geom.placed_at((nx, ny, ntheta))
                if propose_geom.valid(trial, pt_push, (nx, ny)):
                    x, y, theta = nx, ny, ntheta
                    moved = True
                    break
            if not moved:
                break
        placed = propose_geom.placed_at((x, y, theta))
        if propose_geom.valid(placed, pt_push, (x, y)):
            key = (round(x, 3), round(y, 3), round(theta, 3))
            if key not in seen:
                seen.add(key)
                out.append((x, y, theta))
        if len(out) >= top_n:
            break
    return out[:top_n]


def _coords_too_close(
    coords: Tuple[float, float, float],
    existing: Sequence[Tuple[float, float, float]],
    dist_thresh: float,
    angle_thresh: float,
) -> bool:
    x, y, theta = coords
    for ex, ey, et in existing:
        if math.hypot(x - ex, y - ey) < dist_thresh:
            ang = abs(theta - et)
            while ang > math.pi:
                ang = abs(ang - 2 * math.pi)
            if ang < angle_thresh:
                return True
    return False


def candidate_from_proposition(
    x: float,
    y: float,
    theta: float,
    prop,
    *,
    use_full_cast: bool,
) -> Tuple[float, float, float]:
    tx, ty = proposition_translation(prop)
    if use_full_cast:
        nx, ny = x + tx, y + ty
        ntheta = _normalize_angle(theta + float(prop.rotation_rad))
    else:
        nx, ny = x + tx, y + ty
        ntheta = _normalize_angle(theta + float(prop.rotation_rad))
    return (nx, ny, ntheta)


def propose_placements_guidance_cast(
    seeds: Sequence[Tuple[float, float, float]],
    pt_push: Point,
    propose_geom: ProposeGeometry,
    propose_cfg: ProposeConfig,
    *,
    min_dist: float,
    top_n: int,
) -> List[Tuple[float, float, float]]:
    """Emit valid placements from C++ cast-based guidance propositions at each seed."""
    if not seeds or not propose_cfg.use_guidance_propositions:
        return []

    cap = min(top_n, max(1, propose_cfg.candidate_pool))
    dist_thresh = max(
        propose_cfg.guidance_diversity_dist_ratio * min_dist,
        0.02 * math.hypot(
            propose_geom._board_bounds[2] - propose_geom._board_bounds[0],
            propose_geom._board_bounds[3] - propose_geom._board_bounds[1],
        ),
    )
    angle_thresh = math.pi / 8.0
    n_seeds = min(len(seeds), propose_cfg.guidance_proposition_seed_count)
    out: list[tuple[float, float, float]] = []
    seen: set[tuple[float, float, float]] = set()

    for coords in seeds[:n_seeds]:
        x, y, theta = coords
        placed = propose_geom.placed_at(coords)
        props, g = _merged_guidance_propositions(
            propose_geom, placed, (x, y), pt_push, theta,
        )
        for prop in props:
            if len(out) >= cap:
                return out
            use_cast = not g.is_penetrating and is_cast_move(prop.move_type or "")
            candidate = candidate_from_proposition(
                x, y, theta, prop, use_full_cast=use_cast,
            )
            if _coords_too_close(candidate, out, dist_thresh, angle_thresh):
                continue
            key = (round(candidate[0], 4), round(candidate[1], 4), round(candidate[2], 4))
            if key in seen:
                continue
            trial = propose_geom.placed_at(candidate)
            if propose_geom.valid(trial, pt_push, (candidate[0], candidate[1])):
                seen.add(key)
                out.append(candidate)
    return out


def propose_placements_board_edge_guidance_cast(
    seed_anchors: Sequence[
        tuple[tuple[float, float, float], Point, tuple[float, float]]
    ],
    pt_push: Point,
    propose_geom: ProposeGeometry,
    propose_cfg: ProposeConfig,
    *,
    min_dist: float,
    top_n: int,
) -> List[Tuple[float, float, float]]:
    """Refine board-edge snap seeds with per-anchor gravity/target guidance casts."""
    if not seed_anchors:
        return []

    cap = min(top_n, max(1, propose_cfg.candidate_pool))
    dist_thresh = max(
        propose_cfg.guidance_diversity_dist_ratio * min_dist,
        0.02 * math.hypot(
            propose_geom._board_bounds[2] - propose_geom._board_bounds[0],
            propose_geom._board_bounds[3] - propose_geom._board_bounds[1],
        ),
    )
    angle_thresh = math.pi / 8.0
    n_seeds = min(len(seed_anchors), propose_cfg.board_edge_guidance_seeds)
    out: list[tuple[float, float, float]] = []
    seen: set[tuple[float, float, float]] = set()

    for coords, anchor, inward in seed_anchors[:n_seeds]:
        x, y, theta = coords
        placed = propose_geom.placed_at(coords)
        edge_cfg = guidance_config_for_board_edge_anchor(
            anchor,
            inward,
            min_dist=min_dist,
            board_bounds=propose_geom._board_bounds,
            epsilon_ratio=propose_cfg.placement_clearance_epsilon_ratio,
            target_angle_rad=theta,
            max_propositions=propose_cfg.guidance_max_propositions,
            use_tight_packing=propose_cfg.guidance_use_tight_packing,
            use_corner_alignment=propose_cfg.guidance_use_corner_alignment,
            enable_grid_exploration=propose_cfg.guidance_enable_grid,
            diversity_dist_ratio=propose_cfg.guidance_diversity_dist_ratio,
        )
        g = propose_geom.placement_guidance(
            placed, (x, y), pt_push, guidance_cfg=edge_cfg,
        )
        for prop in sorted_guidance_propositions(g):
            if len(out) >= cap:
                return out
            use_cast = not g.is_penetrating and is_cast_move(prop.move_type or "")
            candidate = candidate_from_proposition(
                x, y, theta, prop, use_full_cast=use_cast,
            )
            if _coords_too_close(candidate, out, dist_thresh, angle_thresh):
                continue
            key = (round(candidate[0], 4), round(candidate[1], 4), round(candidate[2], 4))
            if key in seen:
                continue
            trial = propose_geom.placed_at(candidate)
            if propose_geom.valid(trial, pt_push, (candidate[0], candidate[1])):
                seen.add(key)
                out.append(candidate)
    return out


def propose_placements_guidance_propositions(
    seeds: Sequence[Tuple[float, float, float]],
    pt_push: Point,
    propose_geom: ProposeGeometry,
    propose_cfg: ProposeConfig,
    *,
    min_dist: float,
) -> List[Tuple[float, float, float]]:
    """Expand top structured seeds using C++ guidance proposition menu."""
    if not seeds or not propose_cfg.use_guidance_propositions:
        return []
    cap = min(
        propose_cfg.guidance_max_propositions,
        max(1, propose_cfg.candidate_pool // 4),
    )
    return propose_placements_guidance_cast(
        seeds,
        pt_push,
        propose_geom,
        propose_cfg,
        min_dist=min_dist,
        top_n=cap,
    )
