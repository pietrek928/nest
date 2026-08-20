"""Thin post-pack pass runner for demo / benches / evaluator."""

from dataclasses import dataclass
from typing import Any, Callable, Sequence

from shapely import Point, Polygon
from shapely.geometry.base import BaseGeometry

from nest_graph.config import ProposeConfig
from nest_graph.propose.cluster_repack import (
    cluster_repack_selection,
    cluster_relocate_selection,
)
from nest_graph.propose.context import prep_free_space, void_ratio_threshold
from nest_graph.propose.local_se2 import local_se2_selection
from nest_graph.propose.pipeline import allow_void_repack
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
    repair_patterns: Sequence | None = None,
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
            repair_patterns=repair_patterns,
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


@dataclass
class PostPackPrep:
    """Shared post-pack free prep + repack gate (build_graph + evaluator)."""

    sheet_compact: Polygon
    void_geoms_post: Sequence | None
    free_snap_post: Any
    free_post: Any
    mean_part_post: float
    allow_repack: bool
    push_pt: Point | None
    reloc_poles: list
    hole_ok: bool
    stamp_victim: Sequence[int] | None
    repair_patterns: Sequence | None = None


def prepare_post_pack(
    *,
    sheet_compact: Polygon,
    void_geoms_post: Sequence | None,
    part_areas: Sequence[float],
    min_dist: float,
    cfg_propose: ProposeConfig,
    selected_polys: Sequence[int],
    polys: Sequence[BaseGeometry],
    group_id: Sequence[int],
    transform: Sequence,
    part_bases: dict,
    native_pack_geoms_fn: Callable,
    free_prep_mid: Any,
    free_info: Any,
    void_leak_stats: dict | None,
    propose_stats: dict | None,
) -> PostPackPrep:
    from nest_graph.decision.execute import schedule_prep_selection_free  # cycle: execute→heavy_polish→post_pack
    sel_geoms = [
        polys[i] for i in selected_polys
        if polys[i] is not None and not polys[i].is_empty
    ]
    sel_pack_geoms = (
        native_pack_geoms_fn(
            [group_id[i] for i in selected_polys],
            [transform[i] for i in selected_polys],
            part_bases,
        )
        if selected_polys else None
    )
    free_prep_post = schedule_prep_selection_free(
        phase="post",
        sheet=sheet_compact,
        part_areas=part_areas,
        min_dist=min_dist,
        cfg_propose=cfg_propose,
        packed_shapely=sel_geoms,
        pack_geoms=sel_pack_geoms,
        prior=free_prep_mid,
        selection_changed=True,
    )
    assert free_prep_post is not None
    free_post = free_prep_post.free_info
    allow_repack_flag = True
    if isinstance(void_leak_stats, dict):
        allow_repack_flag = allow_void_repack(
            free_kind=void_leak_stats.get("free_kind")
            or (free_info.kind if free_info is not None else None),
            n_void_nest=int(void_leak_stats.get("nest", 0)),
            n_void_refine=int(void_leak_stats.get("refine", 0)),
        )
    elif free_info is not None:
        allow_repack_flag = allow_void_repack(
            free_kind=free_info.kind,
            n_void_nest=0,
            n_void_refine=0,
        )
    hole_ok = int((propose_stats or {}).get("block_hole_accepted", 0) or 0) > 0
    emit_hull = int((propose_stats or {}).get("block_hole_emit_in_hull", 0) or 0)
    tried = int((propose_stats or {}).get("block_hole_tried", 0) or 0)
    if hole_ok:
        allow_repack_flag = False
    # Q213/Q214: hull reject → no cohort victim stamp, but keep allow_repack for BFS peel.
    hull_reject = (
        not hole_ok and tried > 0 and emit_hull > 0
    )
    if hull_reject and propose_stats is not None:
        propose_stats["block_hole_victim"] = None
        propose_stats["_repair_patterns"] = []
        propose_stats["repair_patterns_n"] = 0
    push_pt: Point | None = None
    reloc_poles: list = []
    if free_post.kind == "large_void" and free_post.target_pt is not None:
        push_pt = free_post.target_pt
        reloc_poles = [free_post.target_pt]
        if (
            bool(cfg_propose.use_multi_pole_void)
            and free_post.target_poly is not None
            and not free_post.target_poly.is_empty
        ):
            xy_poles = iterative_multi_poles(
                free_post.target_poly,
                min_dist=min_dist,
                max_poles=int(cfg_propose.multi_pole_max_poles),
            )
            if xy_poles:
                reloc_poles = [Point(x, y) for x, y in xy_poles]
                push_pt = reloc_poles[0]
    # Q224: sterile cohort stamp first; hull reject → BFS peel (victim=None).
    stamp_victim = None
    if not hole_ok and tried > 0 and emit_hull <= 0:
        stamp_victim = (propose_stats or {}).get("block_hole_victim")
    elif not hole_ok and tried <= 0:
        # No mid-3b attempt: allow BFS peel stamp on last leaf as before.
        stamp_victim = (propose_stats or {}).get("block_hole_victim")
    # Hull reject: force victim=None so cluster_repack BFS-peels (Q213).
    if hull_reject:
        stamp_victim = None
    repair_patterns = None
    if stamp_victim is not None and propose_stats is not None:
        repair_patterns = list(propose_stats.get("_repair_patterns") or ()) or None
    if hull_reject:
        repair_patterns = None
    return PostPackPrep(
        sheet_compact=sheet_compact,
        void_geoms_post=void_geoms_post,
        free_snap_post=free_prep_post.free_snap,
        free_post=free_post,
        mean_part_post=float(free_prep_post.mean_part),
        allow_repack=allow_repack_flag,
        push_pt=push_pt,
        reloc_poles=reloc_poles,
        hole_ok=hole_ok,
        stamp_victim=None if hole_ok else stamp_victim,
        repair_patterns=None if hole_ok else repair_patterns,
    )


def apply_post_pack_and_telem(
    prep: PostPackPrep,
    *,
    polys: list[BaseGeometry],
    transform: list,
    group_id: Sequence[int],
    selected_polys: Sequence[int],
    part_by_group: dict[int, Polygon],
    min_dist: float,
    cfg_propose: ProposeConfig,
    fixed_obstacles: Sequence[BaseGeometry] | None,
    part_bases: dict | None = None,
    native_pack_geoms_fn: Callable | None = None,
    void_leak_stats: dict | None = None,
) -> tuple[list[BaseGeometry], list, list[int], dict]:
    if prep.push_pt is None or prep.free_post.kind != "large_void":
        return list(polys), list(transform), list(selected_polys), {}
    if isinstance(void_leak_stats, dict):
        void_leak_stats["multi_pole_count"] = len(prep.reloc_poles)
    polys_out, tr_out, sel_out, pack_stats = run_post_pack_passes(
        prep.sheet_compact,
        list(polys),
        list(transform),
        group_id,
        selected_polys,
        part_by_group,
        min_dist,
        cfg_propose,
        pole=prep.push_pt,
        poles=prep.reloc_poles,
        fixed_obstacles=fixed_obstacles,
        void_geoms=prep.void_geoms_post,
        allow_repack=prep.allow_repack,
        allow_relocate=True,
        allow_local_se2=True,
        void_poly=prep.free_post.target_poly,
        pt_push=prep.push_pt,
        free_space=prep.free_snap_post,
        victim_indices=prep.stamp_victim,
        repair_patterns=list(prep.repair_patterns or ()) or None,
        part_bases=part_bases,
        refresh_after_repack=True,
        mean_part_area=prep.mean_part_post,
        native_pack_geoms_fn=native_pack_geoms_fn,
    )
    if isinstance(void_leak_stats, dict):
        void_leak_stats["repack"] = pack_stats.get("repack") or {
            "attempted": 0,
            "accepted": 0,
            "motif_accepted": 0,
            "skipped_refine_zero": int(not prep.allow_repack),
        }
        void_leak_stats["relocate"] = pack_stats.get("relocate") or {}
        void_leak_stats["local_se2"] = pack_stats.get("local_se2") or {}
        void_leak_stats["repack_attempted"] = int(
            (void_leak_stats["repack"] or {}).get("attempted", 0) or 0
        )
        void_leak_stats["repack_accepted"] = int(
            (void_leak_stats["repack"] or {}).get("accepted", 0) or 0
        )
        void_leak_stats["repack_motif_accepted"] = int(
            (void_leak_stats["repack"] or {}).get("motif_accepted", 0) or 0
        )
        void_leak_stats["repack_pattern_fallback"] = int(
            (void_leak_stats["repack"] or {}).get("pattern_fallback", 0) or 0
        )
    return polys_out, tr_out, list(sel_out), pack_stats
