"""NFP-lite empirical mates: find_closest_polygon_cast + polish_se2_part (Q89)."""


import math

from nest_graph.geometry import Geometry, find_closest_polygon_cast, polish_se2_part
from nest_graph.utils import relative_transform


def nfp_lite_relative(
    follow_base: Geometry,
    t_follow: tuple[float, float, float],
    anchor_solid: Geometry,
    *,
    other_obstacles: list[Geometry] | None = None,
    min_dist: float = 0.0,
    max_t: float = 10.0,
) -> tuple[float, float, float]:
    """
    Nudge follow pose toward contact with ``anchor_solid`` via cast + polish.

    Returns a refined world SE2 for the follow part (not relative).
    """
    fx, fy, fth = float(t_follow[0]), float(t_follow[1]), float(t_follow[2])
    try:
        ax, ay = anchor_solid.centroid()
    except Exception:
        return (fx, fy, fth)
    ax, ay = float(ax), float(ay)
    dx, dy = ax - fx, ay - fy
    dist0 = math.hypot(dx, dy)
    if dist0 < 1e-9:
        return (fx, fy, fth)
    ux, uy = dx / dist0, dy / dist0

    obs: list[Geometry] = [anchor_solid]
    if other_obstacles:
        obs.extend(other_obstacles)

    placed = follow_base.apply_transform(fx, fy, fth)
    try:
        cast = find_closest_polygon_cast(placed, obs, (ux, uy), float(max_t))
        if cast is not None and bool(cast.intersects_path):
            t_hit = float(cast.t_entry)
            if math.isfinite(t_hit) and 0.0 < t_hit < dist0 + 1.0:
                step = max(0.0, t_hit - max(float(min_dist), 1e-4))
                fx2, fy2 = fx + ux * step, fy + uy * step
                # Keep only if we moved closer to the anchor.
                if math.hypot(ax - fx2, ay - fy2) <= dist0 + 1e-9:
                    fx, fy = fx2, fy2
    except Exception:
        pass

    dirs = ((ux, uy), (-uy, ux), (uy, -ux))
    try:
        polished = polish_se2_part(
            follow_base,
            (fx, fy, fth),
            obs,
            None,
            dirs,
            n_angles=2,
            max_t=min(float(max_t), max(dist0, 1.0)),
            min_dist=float(min_dist),
            mode="pole",
            pole=(ax, ay),
        )
        if polished is not None:
            px, py, pth = float(polished[0]), float(polished[1]), float(polished[2])
            if math.hypot(ax - px, ay - py) <= dist0 + 1e-6:
                return (px, py, pth)
    except Exception:
        pass
    return (fx, fy, fth)


def nfp_lite_pair_relative(
    follow_base: Geometry,
    t_a: tuple[float, float, float],
    t_b: tuple[float, float, float],
    anchor_solid: Geometry,
    *,
    min_dist: float = 0.0,
) -> tuple[float, float, float]:
    """Refine B then return relative SE2 of B w.r.t. A (Q89 empirical mate)."""
    t_b2 = nfp_lite_relative(
        follow_base,
        t_b,
        anchor_solid,
        min_dist=min_dist,
    )
    return relative_transform(t_a, t_b2)
