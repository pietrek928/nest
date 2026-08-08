"""Post-DFS local SE(2) polish toward a void pole (coarse then fine)."""

import math
from typing import Sequence

import numpy as np
from shapely import Point, Polygon
from shapely.geometry.base import BaseGeometry
from shapely.ops import nearest_points, unary_union

from nest_graph.config import ProposeConfig
from nest_graph.propose.compaction import _pose_clear, selection_pairwise_independent
from nest_graph.propose.context import _cluster_merge_gap
from nest_graph.propose.placement_perimeter import edge_inward_at_point
from nest_graph.utils import transform_poly


def _slide_dirs(ux: float, uy: float) -> list[tuple[float, float]]:
    """Primary toward-pole plus ±45° / ±90° slides."""
    dirs = [(ux, uy)]
    for ang in (math.pi / 4, -math.pi / 4, math.pi / 2, -math.pi / 2):
        c, s = math.cos(ang), math.sin(ang)
        dirs.append((ux * c - uy * s, ux * s + uy * c))
    out: list[tuple[float, float]] = []
    for dx, dy in dirs:
        n = math.hypot(dx, dy)
        if n < 1e-12:
            continue
        out.append((dx / n, dy / n))
    return out


def exterior_tangent_dirs(
    sheet: Polygon,
    poly: BaseGeometry,
) -> list[tuple[float, float]] | None:
    """±unit tangents along sheet exterior at the nearest contact."""
    if sheet is None or sheet.is_empty or poly is None or poly.is_empty:
        return None
    try:
        contact, _ = nearest_points(sheet.exterior, poly)
    except Exception:
        return None
    info = edge_inward_at_point(sheet, contact)
    if info is None:
        return None
    _, (nx, ny) = info
    # Rotate inward normal 90° → tangents along exterior.
    return [(-ny, nx), (ny, -nx)]


def _is_board_adj(
    poly: BaseGeometry,
    sheet: Polygon,
    min_dist: float,
) -> bool:
    sentinel = sheet.exterior.buffer(float(min_dist) + 1e-3)
    gap = _cluster_merge_gap([poly], min_dist, sheet)
    try:
        return poly.buffer(gap).intersects(sentinel.buffer(gap))
    except Exception:
        return False


def local_se2_selection(
    sheet: Polygon,
    polys: list[BaseGeometry],
    transforms: list,
    group_ids: Sequence[int],
    selected_indices: Sequence[int],
    part_by_group: dict[int, Polygon],
    min_dist: float,
    propose_cfg: ProposeConfig,
    *,
    pole: Point | None = None,
    fixed_obstacles: Sequence[BaseGeometry] | None = None,
    board_adj_indices: Sequence[int] | None = None,
) -> tuple[list[BaseGeometry], list, dict]:
    """Slide/rotate selected parts; board_adj uses ±exterior tangent only.

    Coarse steps use ``>= 4×min_dist``; fine uses ``0.25×min_dist`` (compaction).
    Floating parts attract toward ``pole`` with ±45/±90 slides.
    """
    stats = {"attempted": 0, "accepted": 0, "moved": 0, "tangent_moves": 0}
    out_polys = list(polys)
    out_tr = [np.asarray(t, dtype=np.float64).reshape(3) for t in transforms]
    sel = [int(i) for i in selected_indices]
    if (
        not propose_cfg.enable_local_se2
        or sheet is None
        or sheet.is_empty
        or len(sel) < 1
    ):
        return out_polys, out_tr, stats
    # Floating polish needs a pole; board_adj tangent slides do not.
    if pole is None or pole.is_empty:
        # Still allow board_adj tangent-only passes.
        pass

    locked = [
        g for g in (fixed_obstacles or ())
        if g is not None and not g.is_empty
    ]
    n_angles = max(1, int(propose_cfg.local_se2_n_angles))
    angle_deltas = [(2.0 * math.pi * k) / n_angles for k in range(n_angles)]

    coarse_step = max(4.0 * float(min_dist), 1e-4)
    fine_step = max(0.25 * float(min_dist), 1e-4)
    phases = (
        (coarse_step, max(1, int(propose_cfg.local_se2_max_coarse_steps))),
        (fine_step, max(1, int(propose_cfg.local_se2_max_fine_steps))),
    )

    board_set = set(int(i) for i in (board_adj_indices or ()))
    if not board_set:
        board_set = {
            i for i in sel
            if out_polys[i] is not None
            and not out_polys[i].is_empty
            and _is_board_adj(out_polys[i], sheet, min_dist)
        }

    def _order_key(i: int) -> float:
        if out_polys[i] is None or out_polys[i].is_empty:
            return -1.0
        if pole is not None and not pole.is_empty:
            return float(out_polys[i].centroid.distance(pole))
        return 0.0

    order = sorted(sel, key=_order_key, reverse=True)

    for idx in order:
        poly = out_polys[idx]
        tr = out_tr[idx]
        gid = int(group_ids[idx])
        part = part_by_group.get(gid)
        if part is None or poly is None or poly.is_empty:
            continue
        is_rim = idx in board_set
        if not is_rim and (pole is None or pole.is_empty):
            continue
        stats["attempted"] += 1

        if is_rim:
            dirs = exterior_tangent_dirs(sheet, poly)
            if not dirs:
                continue
            # Accept any clear slide along wall (progress = distance moved).
            dist0 = 0.0
            use_pole_metric = False
        else:
            cx, cy = float(poly.centroid.x), float(poly.centroid.y)
            dx, dy = float(pole.x) - cx, float(pole.y) - cy
            dist0 = math.hypot(dx, dy)
            if dist0 < 1e-9:
                continue
            ux, uy = dx / dist0, dy / dist0
            dirs = _slide_dirs(ux, uy)
            use_pole_metric = True

        others = [
            out_polys[j]
            for j in sel
            if int(j) != int(idx) and out_polys[j] is not None and not out_polys[j].is_empty
        ]
        fixed = unary_union(locked + others) if (locked or others) else Polygon()

        best_poly = poly
        best_tr = tr.copy()
        best_dist = dist0
        best_slide = 0.0
        improved = False

        for step, max_steps in phases:
            for du, dv in dirs:
                for s in range(1, max_steps + 1):
                    ox, oy = du * step * s, dv * step * s
                    any_clear = False
                    for dtheta in angle_deltas:
                        cand_tr = np.array(
                            [
                                tr[0] + ox,
                                tr[1] + oy,
                                (float(tr[2]) + dtheta) % (2.0 * math.pi),
                            ],
                            dtype=np.float64,
                        )
                        cand = transform_poly(part, cand_tr)
                        if not _pose_clear(cand, sheet, fixed, min_dist):
                            continue
                        any_clear = True
                        prev_p, prev_t = out_polys[idx], out_tr[idx]
                        out_polys[idx] = cand
                        out_tr[idx] = cand_tr
                        ok = selection_pairwise_independent(out_polys, sel)
                        out_polys[idx] = prev_p
                        out_tr[idx] = prev_t
                        if not ok:
                            continue
                        if use_pole_metric:
                            d = float(cand.centroid.distance(pole))
                            if d + 1e-9 >= best_dist:
                                continue
                            best_poly = cand
                            best_tr = cand_tr
                            best_dist = d
                            improved = True
                        else:
                            slide = math.hypot(ox, oy)
                            if slide + 1e-9 <= best_slide:
                                continue
                            best_poly = cand
                            best_tr = cand_tr
                            best_slide = slide
                            improved = True
                    if not any_clear:
                        break

        if improved:
            out_polys[idx] = best_poly
            out_tr[idx] = best_tr
            stats["accepted"] += 1
            stats["moved"] += 1
            if is_rim:
                stats["tangent_moves"] += 1

    return out_polys, out_tr, stats
