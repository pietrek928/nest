"""Thin post-pack pass runner for demo / benches / evaluator."""

from typing import Sequence

from shapely import Point, Polygon
from shapely.geometry.base import BaseGeometry

from nest_graph.config import ProposeConfig
from nest_graph.propose.cluster_repack import (
    cluster_repack_selection,
    cluster_relocate_selection,
)
from nest_graph.propose.context import prep_free_space, void_ratio_threshold
from nest_graph.propose.local_se2 import local_se2_selection
from nest_graph.propose.selection_edit import SelectionEditCtx
from nest_graph.propose.void_topology import iterative_multi_poles


def run_post_pack_passes(
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
    poles: Sequence[Point] | None = None,
    fixed_obstacles: Sequence[BaseGeometry] | None = None,
    void_geoms: Sequence | None = None,
    board_adj_indices: Sequence[int] | None = None,
    allow_repack: bool = True,
    allow_relocate: bool = True,
    allow_local_se2: bool = True,
    void_poly=None,
    pt_push: Point | None = None,
    free_space=None,
    victim_indices: Sequence[int] | None = None,
    part_bases=None,
    board_geom=None,
    refresh_after_repack: bool = True,
    mean_part_area: float | None = None,
    native_pack_geoms_fn=None,
) -> tuple[list[BaseGeometry], list, list[int], dict]:
    """Run cluster_repack → cluster_relocate → local_se2 (demo parity).

    When ``refresh_after_repack`` and motif repack accepts with multi-pole,
    refresh free poles and re-run relocate+se2 (one post-pack gate).
    """
    stats: dict = {
        "repack": None,
        "relocate": None,
        "local_se2": None,
    }
    sel = list(selected_indices)
    out_polys = list(polys)
    out_tr = list(transforms)
    ctx = SelectionEditCtx(
        sheet=sheet,
        polys=out_polys,
        transforms=out_tr,
        group_ids=group_ids,
        selected_indices=sel,
        part_by_group=part_by_group,
        min_dist=min_dist,
        propose_cfg=propose_cfg,
        pole=pole,
        poles=poles,
        fixed_obstacles=fixed_obstacles,
        void_geoms=void_geoms,
        board_adj_indices=board_adj_indices,
        part_bases=part_bases,
        board_geom=board_geom,
    )
    if allow_repack and propose_cfg.enable_cluster_repack:
        out_polys, out_tr, sel, repack_stats = cluster_repack_selection(
            ctx,
            void_poly=void_poly,
            pt_push=pt_push if pt_push is not None else pole,
            free_space=free_space,
            victim_indices=victim_indices,
        )
        ctx.polys = out_polys
        ctx.transforms = out_tr
        ctx.selected_indices = sel
        stats["repack"] = repack_stats
    if allow_relocate and getattr(propose_cfg, "enable_cluster_relocate", True):
        out_polys, out_tr, reloc_stats = cluster_relocate_selection(ctx)
        ctx.polys = out_polys
        ctx.transforms = out_tr
        stats["relocate"] = reloc_stats
    if allow_local_se2:
        out_polys, out_tr, se2_stats = local_se2_selection(ctx)
        ctx.polys = out_polys
        ctx.transforms = out_tr
        stats["local_se2"] = se2_stats

    repack_stats = stats.get("repack") or {}
    poles_list = list(poles or ())
    if (
        refresh_after_repack
        and int(repack_stats.get("accepted", 0)) > 0
        and len(poles_list) > 1
    ):
        sel_geoms = [
            out_polys[i] for i in sel
            if out_polys[i] is not None and not out_polys[i].is_empty
        ]
        mean_part = float(mean_part_area) if mean_part_area is not None else 1.0
        pack_geoms = None
        if native_pack_geoms_fn is not None:
            pack_geoms = native_pack_geoms_fn(sel, out_tr, group_ids)
        free_snap = prep_free_space(
            sheet,
            sel_geoms,
            mean_part,
            min_dist,
            void_ratio_threshold=void_ratio_threshold(propose_cfg),
            pack_geoms=pack_geoms,
            snapshot=True,
        )
        reloc_pole = (
            free_snap.analysis.target_pt
            if free_snap.analysis.target_pt is not None
            else poles_list[min(1, len(poles_list) - 1)]
        )
        refresh_poles: list = []
        if (
            bool(propose_cfg.use_multi_pole_void)
            and free_snap.analysis.target_poly is not None
            and not free_snap.analysis.target_poly.is_empty
        ):
            xy_poles = iterative_multi_poles(
                free_snap.analysis.target_poly,
                min_dist=min_dist,
                max_poles=int(propose_cfg.multi_pole_max_poles),
            )
            if xy_poles:
                refresh_poles = [Point(x, y) for x, y in xy_poles]
                if reloc_pole is None:
                    reloc_pole = refresh_poles[0]
        if not refresh_poles and reloc_pole is not None:
            refresh_poles = [reloc_pole]
        if reloc_pole is not None and reloc_pole is not (pt_push or pole):
            ctx.pole = reloc_pole
            ctx.poles = refresh_poles
            out_polys, out_tr, reloc_stats2 = cluster_relocate_selection(ctx)
            ctx.polys = out_polys
            ctx.transforms = out_tr
            if int(reloc_stats2.get("accepted", 0)):
                stats["relocate"] = reloc_stats2
            out_polys, out_tr, se2_stats2 = local_se2_selection(ctx)
            if int(se2_stats2.get("moved", 0)):
                stats["local_se2"] = se2_stats2
    return out_polys, out_tr, list(sel), stats
