"""Hash-grid SE(2) NMS and STRtree conflict-degree ranking helpers."""

import math
from typing import Sequence

from shapely.geometry import box
from shapely.strtree import STRtree


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
    two_pi = 2.0 * math.pi

    def _bins(c: tuple[float, float, float]) -> tuple[int, int, int]:
        th = float(c[2]) % two_pi
        if th < 0.0:
            th += two_pi
        return (int(c[0] // eps), int(c[1] // eps), int(th // theta_tol))

    def _neighborhood_occupied(xb: int, yb: int, tb: int) -> bool:
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for dt in (-1, 0, 1):
                    if (xb + dx, yb + dy, tb + dt) in grid:
                        return True
        return False

    for cand in candidates:
        xb, yb, tb = _bins(cand)
        if _neighborhood_occupied(xb, yb, tb):
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
    list. Uses STRtree on transformed part bounds — not exact polygon
    intersections.
    """
    if len(scored) <= 1 or lambda_pen <= 0.0:
        if stats_out is not None:
            stats_out["conflict_penalty_applied"] = 0
        return list(scored)

    from nest_graph.utils import transform_poly

    geoms = []
    for _score, coords in scored:
        try:
            poly = transform_poly(shape_to_place, coords)
            minx, miny, maxx, maxy = poly.bounds
            geoms.append(box(minx - pad, miny - pad, maxx + pad, maxy + pad))
        except Exception:
            geoms.append(box(coords[0], coords[1], coords[0] + 1e-3, coords[1] + 1e-3))
    tree = STRtree(geoms)
    penalized: list[tuple[float, tuple[float, float, float]]] = []
    applied = 0
    m_cap = max(int(max_overlap), 1)
    for i, (score, coords) in enumerate(scored):
        try:
            hits = tree.query(geoms[i])
            # shapely 2 returns ndarray of indices; older may return geoms
            if hasattr(hits, "tolist"):
                overlap = max(0, len(hits) - 1)
            else:
                overlap = max(0, len(list(hits)) - 1)
        except Exception:
            overlap = 0
        if overlap > 0:
            applied += 1
        factor = 1.0 - float(lambda_pen) * float(min(overlap, m_cap))
        penalized.append((float(score) * max(factor, 0.0), coords))
    penalized.sort(key=lambda x: x[0], reverse=True)
    if stats_out is not None:
        stats_out["conflict_penalty_applied"] = applied
    return penalized
