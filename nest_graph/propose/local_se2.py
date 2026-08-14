"""Post-DFS local SE(2) polish toward a void pole (native cast_slide)."""

import math
from typing import Sequence

import numpy as np
from shapely import Point, Polygon
from shapely.geometry.base import BaseGeometry
from shapely.ops import nearest_points

from nest_graph.board import board_context_from_geometry
from nest_graph.config import ProposeConfig
from nest_graph.geometry import Geometry, polish_se2_part, snap_pose_to_ring
from nest_graph.propose.placement_common import (
    as_geometry,
    is_board_adj,
    is_pose_clear,
    selection_pairwise_independent,
)
from nest_graph.propose.placement_outline import (
    inward_at_contact,
    outline_ring_geom,
    outline_standoff_distance,
)
from nest_graph.propose.placement_perimeter import edge_inward_at_point
from nest_graph.propose.selection_edit import SelectionEditCtx
from nest_graph.propose.void_topology import preferred_spine_pole
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
    return [(-ny, nx), (ny, -nx)]


def local_se2_selection(
    sheet: Polygon | SelectionEditCtx,
    polys: list[BaseGeometry] | None = None,
    transforms: list | None = None,
    group_ids: Sequence[int] | None = None,
    selected_indices: Sequence[int] | None = None,
    part_by_group: dict[int, Polygon] | None = None,
    min_dist: float | None = None,
    propose_cfg: ProposeConfig | None = None,
    *,
    pole: Point | None = None,
    poles: Sequence[Point] | None = None,
    fixed_obstacles: Sequence[BaseGeometry] | None = None,
    board_adj_indices: Sequence[int] | None = None,
) -> tuple[list[BaseGeometry], list, dict]:
    """Slide/rotate selected parts via native polish_se2_part.

    Prefer ``SelectionEditCtx`` as the first argument; legacy kwargs remain.
    Board_adj uses ±exterior tangent only. Floating parts attract toward the
    nearest spine pole with ±45/±90 slides when ``enable_gravity_compaction``
    is True. Empty ``poles`` skips pole pull (no SW/corner fallback).
    """
    void_geoms = None
    if isinstance(sheet, SelectionEditCtx):
        ctx = sheet
        sheet = ctx.sheet
        polys = ctx.polys
        transforms = ctx.transforms
        group_ids = ctx.group_ids
        selected_indices = ctx.selected_indices
        part_by_group = ctx.part_by_group
        min_dist = ctx.min_dist
        propose_cfg = ctx.propose_cfg
        pole = ctx.pole if pole is None else pole
        poles = ctx.poles if poles is None else poles
        fixed_obstacles = (
            ctx.fixed_obstacles if fixed_obstacles is None else fixed_obstacles
        )
        board_adj_indices = (
            ctx.board_adj_indices if board_adj_indices is None else board_adj_indices
        )
        void_geoms = ctx.void_geoms
    assert polys is not None and transforms is not None
    assert group_ids is not None and selected_indices is not None
    assert part_by_group is not None and min_dist is not None and propose_cfg is not None
    stats = {
        "attempted": 0,
        "accepted": 0,
        "moved": 0,
        "tangent_moves": 0,
        "se2_native_hits": 0,
        "se2_native_accepted": 0,
        "pole_distance_delta": 0.0,
        "theta_changed_count": 0,
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

    if void_geoms is None:
        _, void_geoms = board_context_from_geometry(sheet)
    voids = [
        g for g in (as_geometry(v) for v in (void_geoms or ()))
        if g is not None
    ]

    locked = [
        g for g in (fixed_obstacles or ())
        if g is not None and not g.is_empty
    ]
    n_angles = max(1, int(propose_cfg.local_se2_n_angles))

    coarse_step = max(4.0 * float(min_dist), 1e-4)
    max_coarse = max(1, int(getattr(propose_cfg, "local_se2_max_coarse_steps", 8)))
    minx, miny, maxx, maxy = sheet.bounds
    sheet_diag = math.hypot(maxx - minx, maxy - miny)

    board_ring = outline_ring_geom(sheet)
    board_set = set(int(i) for i in (board_adj_indices or ()))
    if not board_set:
        board_set = {
            i for i in sel
            if out_polys[i] is not None
            and not out_polys[i].is_empty
            and is_board_adj(out_polys[i], sheet, min_dist, ring=board_ring)
        }

    def _floater_pole(poly: BaseGeometry) -> Point | None:
        if poly is None or poly.is_empty:
            return None
        c = poly.centroid
        return preferred_spine_pole(float(c.x), float(c.y), poles, fallback=pole)

    def _order_key(i: int) -> float:
        if out_polys[i] is None or out_polys[i].is_empty:
            return -1.0
        p = _floater_pole(out_polys[i])
        if p is not None:
            return float(out_polys[i].centroid.distance(p))
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
        pull_pole = None if is_rim else _floater_pole(poly)
        if not is_rim and (
            not propose_cfg.enable_gravity_compaction
            or pull_pole is None
        ):
            continue
        stats["attempted"] += 1

        if is_rim:
            dirs = exterior_tangent_dirs(sheet, poly)
            if not dirs:
                continue
            dist0 = 0.0
            use_pole_metric = False
            # Late kiss dock: snap to outline when standoff is off ideal.
            ring = board_ring
            kiss_err = abs(
                outline_standoff_distance(poly, sheet, ring=ring) - float(min_dist)
            ) if ring is not None else 0.0
            if ring is not None and kiss_err > max(float(min_dist) * 0.5, 1e-4):
                try:
                    contact, _ = nearest_points(sheet.exterior, poly)
                except Exception:
                    contact = None
                if contact is not None:
                    snap_contact, inward = inward_at_contact(sheet, contact)
                    if gid not in part_geoms:
                        part_geoms[gid] = Geometry.from_shapely(part)
                    snapped = snap_pose_to_ring(
                        part_geoms[gid],
                        ring,
                        (float(snap_contact.x), float(snap_contact.y)),
                        (float(inward[0]), float(inward[1])),
                        float(tr[2]),
                        float(min_dist),
                        board=None,
                    )
                    if snapped is not None:
                        sx, sy, sth = snapped
                        snap_g = part_geoms[gid].apply_transform(sx, sy, sth)
                        others0 = [
                            out_polys[j]
                            for j in sel
                            if int(j) != int(idx)
                            and out_polys[j] is not None
                            and not out_polys[j].is_empty
                        ]
                        packed0 = [
                            g for g in (
                                as_geometry(p) for p in (list(locked) + others0)
                            ) if g is not None
                        ]
                        if is_pose_clear(snap_g, voids, packed0, float(min_dist)):
                            snap_poly = transform_poly(part, (sx, sy, sth))
                            prev_p, prev_t = out_polys[idx], out_tr[idx]
                            out_polys[idx] = snap_poly
                            out_tr[idx] = np.array([sx, sy, sth], dtype=np.float64)
                            if selection_pairwise_independent(out_polys, sel):
                                poly = snap_poly
                                tr = out_tr[idx]
                                stats["accepted"] += 1
                                stats["moved"] += 1
                                stats["tangent_moves"] += 1
                            else:
                                out_polys[idx] = prev_p
                                out_tr[idx] = prev_t
        else:
            cx, cy = float(poly.centroid.x), float(poly.centroid.y)
            dx, dy = float(pull_pole.x) - cx, float(pull_pole.y) - cy
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
        packed = [
            g for g in (as_geometry(p) for p in (list(locked) + others)) if g is not None
        ]
        obs_with_voids = [*voids, *packed]
        if gid not in part_geoms:
            part_geoms[gid] = Geometry.from_shapely(part)
        part_g = part_geoms[gid]

        stats["se2_native_hits"] += 1
        max_t = max(
            sheet_diag,
            dist0 if use_pole_metric else 0.0,
            coarse_step * max_coarse,
        )
        polished = polish_se2_part(
            part_g,
            (float(tr[0]), float(tr[1]), float(tr[2])),
            obs_with_voids,
            None,
            dirs,
            n_angles=n_angles,
            max_t=float(max_t),
            min_dist=float(min_dist),
            mode="pole" if use_pole_metric else "slide",
            pole=(float(pull_pole.x), float(pull_pole.y)) if use_pole_metric else None,
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
        elif use_pole_metric:
            d1 = math.hypot(
                float(cand.centroid.x) - float(pull_pole.x),
                float(cand.centroid.y) - float(pull_pole.y),
            )
            stats["pole_distance_delta"] += d1 - dist0
            dth = abs(((cand_tr[2] - tr[2] + math.pi) % (2 * math.pi)) - math.pi)
            if dth > 1e-9:
                stats["theta_changed_count"] += 1

    return out_polys, out_tr, stats
