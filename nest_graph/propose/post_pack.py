"""Thin post-pack pass runner for benches without the full demo render loop."""

from typing import Sequence

from shapely import Point, Polygon
from shapely.geometry.base import BaseGeometry

from nest_graph.config import ProposeConfig
from nest_graph.propose.cluster_repack import cluster_repack_selection
from nest_graph.propose.compaction import compact_selection, selection_pairwise_independent
from nest_graph.propose.local_se2 import local_se2_selection
from nest_graph.propose.selection_edit import SelectionEditCtx


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
    gravity: Point | None = None,
    fixed_obstacles: Sequence[BaseGeometry] | None = None,
    board_adj_indices: Sequence[int] | None = None,
    allow_repack: bool = True,
    allow_compaction: bool = False,
    allow_local_se2: bool = True,
) -> tuple[list[BaseGeometry], list, list[int], dict]:
    """Run compact → cluster_repack → local_se2 gates using SelectionEditCtx."""
    stats: dict = {"compaction": None, "repack": None, "local_se2": None}
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
        fixed_obstacles=fixed_obstacles,
        board_adj_indices=board_adj_indices,
        gravity=gravity,
    )
    if allow_compaction and propose_cfg.enable_gravity_compaction and len(sel) >= 2:
        out_polys, out_tr = compact_selection(ctx)
        ctx.polys = out_polys
        ctx.transforms = out_tr
        stats["compaction"] = {"ok": selection_pairwise_independent(out_polys, sel)}
    if allow_repack and propose_cfg.enable_cluster_repack:
        out_polys, out_tr, sel, repack_stats = cluster_repack_selection(ctx)
        ctx.polys = out_polys
        ctx.transforms = out_tr
        ctx.selected_indices = sel
        stats["repack"] = repack_stats
    if allow_local_se2:
        out_polys, out_tr, se2_stats = local_se2_selection(ctx)
        stats["local_se2"] = se2_stats
    return out_polys, out_tr, list(sel), stats
