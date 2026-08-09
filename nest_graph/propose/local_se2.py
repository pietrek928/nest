"""Post-DFS local SE(2) polish toward a void pole (native cast_slide)."""

import math
from typing import Sequence

import numpy as np
from shapely import Point, Polygon
from shapely.geometry.base import BaseGeometry

from nest_graph.config import ProposeConfig
from nest_graph.geometry import Geometry, polish_se2_part
from nest_graph.propose.compaction import selection_pairwise_independent
from nest_graph.propose.context import _cluster_merge_gap
from nest_graph.propose.placement_outline import outline_ring_geom
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
        from shapely.ops import nearest_points
        contact, _ = nearest_points(sheet.exterior, poly)
    except Exception:
        return None
    info = edge_inward_at_point(sheet, contact)
    if info is None:
        return None
    _, (nx, ny) = info
    # Rotate inward normal 90° → tangents along exterior.
    return [(-ny, nx), (ny, -nx)]


def _as_geometry(g) -> Geometry | None:
    if g is None:
        return None
    if isinstance(g, Geometry):
        return g
    if hasattr(g, "is_empty") and g.is_empty:
        return None
    return Geometry.from_shapely(g)


def _is_board_adj(
    poly: BaseGeometry,
    sheet: Polygon,
    min_dist: float,
) -> bool:
    """Board-adjacent if standoff ≤ min_dist + 2*gap (old sentinel+buffer merge)."""
    if poly is None or (hasattr(poly, "is_empty") and poly.is_empty):
        return False
    if sheet is None or sheet.is_empty:
        return False
    ring = outline_ring_geom(sheet)
    if ring is None:
        return False
    gap = _cluster_merge_gap([poly], min_dist, sheet)
    g = _as_geometry(poly)
    if g is None:
        return False
    try:
        return float(g.standoff_distance(ring)) <= float(min_dist) + 2.0 * gap + 1e-9
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
    """Slide/rotate selected parts via native polish_se2_part.

    Board_adj uses ±exterior tangent only. Floating parts attract toward
    ``pole`` with ±45/±90 slides.
    """
    stats = {
        "attempted": 0,
        "accepted": 0,
        "moved": 0,
        "tangent_moves": 0,
        "se2_native_hits": 0,
        "se2_native_accepted": 0,
    }
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

    coarse_step = max(4.0 * float(min_dist), 1e-4)
    fine_step = max(0.25 * float(min_dist), 1e-4)
    max_coarse = max(1, int(propose_cfg.local_se2_max_coarse_steps))
    max_fine = max(1, int(propose_cfg.local_se2_max_fine_steps))
    board_g = Geometry.from_shapely(sheet)
    minx, miny, maxx, maxy = sheet.bounds
    sheet_diag = math.hypot(maxx - minx, maxy - miny)

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
    part_geoms: dict[int, Geometry] = {}

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
        obs_geoms = [
            g for g in (_as_geometry(p) for p in (list(locked) + others)) if g is not None
        ]
        if gid not in part_geoms:
            part_geoms[gid] = Geometry.from_shapely(part)
        part_g = part_geoms[gid]

        stats["se2_native_hits"] += 1
        max_t = max(
            sheet_diag,
            dist0 if use_pole_metric else 0.0,
            coarse_step * max_coarse + fine_step * max_fine,
        )
        polished = polish_se2_part(
            part_g,
            (float(tr[0]), float(tr[1]), float(tr[2])),
            obs_geoms,
            board_g,
            dirs,
            n_angles=n_angles,
            max_t=float(max_t),
            min_dist=float(min_dist),
            mode="pole" if use_pole_metric else "slide",
            pole=(float(pole.x), float(pole.y)) if use_pole_metric else None,
        )
        if polished is None:
            continue
        bx, by, bth = polished
        cand_tr = np.array([bx, by, bth], dtype=np.float64)
        if (
            abs(cand_tr[0] - tr[0]) < 1e-12
            and abs(cand_tr[1] - tr[1]) < 1e-12
            and abs(((cand_tr[2] - tr[2] + math.pi) % (2 * math.pi)) - math.pi) < 1e-12
        ):
            continue
        cand = transform_poly(part, cand_tr)
        prev_p, prev_t = out_polys[idx], out_tr[idx]
        out_polys[idx] = cand
        out_tr[idx] = cand_tr
        ok = selection_pairwise_independent(out_polys, sel)
        if not ok:
            out_polys[idx] = prev_p
            out_tr[idx] = prev_t
            continue
        stats["accepted"] += 1
        stats["moved"] += 1
        stats["se2_native_accepted"] += 1
        if is_rim:
            stats["tangent_moves"] += 1

    return out_polys, out_tr, stats
