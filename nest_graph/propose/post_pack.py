"""Thin post-pack pass runner for demo / benches / evaluator."""

from typing import Sequence

from shapely import Point, Polygon
from shapely.geometry.base import BaseGeometry

from nest_graph.config import ProposeConfig
from nest_graph.propose.cluster_repack import (
    cluster_repack_selection,
    cluster_relocate_selection,
)
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
    fixed_obstacles: Sequence[BaseGeometry] | None = None,
    void_geoms: Sequence | None = None,
    board_adj_indices: Sequence[int] | None = None,
    allow_repack: bool = True,
    allow_relocate: bool = True,
    allow_local_se2: bool = True,
    void_poly=None,
    pt_push: Point | None = None,
    free_space=None,
) -> tuple[list[BaseGeometry], list, list[int], dict]:
    """Run cluster_repack → cluster_relocate → local_se2 (demo parity)."""
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
        fixed_obstacles=fixed_obstacles,
        void_geoms=void_geoms,
        board_adj_indices=board_adj_indices,
    )
    if allow_repack and propose_cfg.enable_cluster_repack:
        out_polys, out_tr, sel, repack_stats = cluster_repack_selection(
            ctx,
            void_poly=void_poly,
            pt_push=pt_push if pt_push is not None else pole,
            free_space=free_space,
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
        stats["local_se2"] = se2_stats
    return out_polys, out_tr, list(sel), stats
