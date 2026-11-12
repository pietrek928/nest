"""Hash-grid SE(2) NMS and AABB conflict-degree ranking helpers."""

import math
from typing import Sequence

from nest_graph.geometry import aabb_tree_self_overlap_counts
from nest_graph.utils import transform_poly


def pose_nms_bins(
    coords: tuple[float, float, float],
    eps: float,
    theta_tol: float,
) -> tuple[int, int, int]:
    two_pi = 2.0 * math.pi
    th = float(coords[2]) % two_pi
    if th < 0.0:
        th += two_pi
    return (int(coords[0] // eps), int(coords[1] // eps), int(th // theta_tol))


def pose_nms_neighborhood_occupied(
    grid: set[tuple[int, int, int]],
    xb: int,
    yb: int,
    tb: int,
) -> bool:
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            for dt in (-1, 0, 1):
                if (xb + dx, yb + dy, tb + dt) in grid:
                    return True
    return False


def apply_pose_nms(
    candidates: Sequence[tuple[float, float, float]],
    *,
    eps: float = 1.0,
    theta_tol: float = 0.15,
    stats_out: dict | None = None,
) -> list[tuple[float, float, float]]:
    """Keep score-ordered poses with SE(2) spatial-hash NMS (~O(n)).

    ``candidates`` must already be sorted best-first. Bin occupancy uses a
    3x3x3 neighborhood in (x, y, θ) hash space.
    """
    if not candidates:
        if stats_out is not None:
            stats_out["nms_kept"] = 0
            stats_out["nms_dropped"] = 0
        return []
    eps = max(float(eps), 1e-6)
    theta_tol = max(float(theta_tol), 1e-6)
    kept: list[tuple[float, float, float]] = []
    grid: set[tuple[int, int, int]] = set()
    dropped = 0
    for cand in candidates:
        xb, yb, tb = pose_nms_bins(cand, eps, theta_tol)
        if pose_nms_neighborhood_occupied(grid, xb, yb, tb):
            dropped += 1
            continue
        grid.add((xb, yb, tb))
        kept.append((float(cand[0]), float(cand[1]), float(cand[2])))
    if stats_out is not None:
        stats_out["nms_kept"] = len(kept)
        stats_out["nms_dropped"] = dropped
    return kept


def apply_conflict_degree_penalty(
    scored: Sequence[tuple[float, tuple[float, float, float]]],
    shape_to_place,
    *,
    lambda_pen: float = 0.05,
    max_overlap: int = 5,
    pad: float = 0.0,
    stats_out: dict | None = None,
) -> list[tuple[float, tuple[float, float, float]]]:
    """Reweight scores by AABB conflict degree among the candidate pool.

    ``scored`` is (score, coords) with higher score better. Returns resorted
    list. Uses native AABB BVH on transformed part bounds — not exact polygon
    intersections.
    """
    if len(scored) <= 1 or lambda_pen <= 0.0:
        if stats_out is not None:
            stats_out["conflict_penalty_applied"] = 0
        return list(scored)

    boxes: list[tuple[float, float, float, float]] = []
    for _score, coords in scored:
        try:
            poly = transform_poly(shape_to_place, coords)
            minx, miny, maxx, maxy = poly.bounds
            boxes.append(
                (
                    float(minx) - pad,
                    float(miny) - pad,
                    float(maxx) + pad,
                    float(maxy) + pad,
                )
            )
        except Exception:
            boxes.append(
                (
                    float(coords[0]),
                    float(coords[1]),
                    float(coords[0]) + 1e-3,
                    float(coords[1]) + 1e-3,
                )
            )
    try:
        overlaps = aabb_tree_self_overlap_counts(boxes)
    except ValueError:
        overlaps = [0] * len(boxes)
    penalized: list[tuple[float, tuple[float, float, float]]] = []
    applied = 0
    m_cap = max(int(max_overlap), 1)
    for i, (score, coords) in enumerate(scored):
        overlap = max(0, int(overlaps[i]))
        if overlap > 0:
            applied += 1
        factor = 1.0 - float(lambda_pen) * float(min(overlap, m_cap))
        penalized.append((float(score) * max(factor, 0.0), coords))
    penalized.sort(key=lambda x: x[0], reverse=True)
    if stats_out is not None:
        stats_out["conflict_penalty_applied"] = applied
    return penalized
