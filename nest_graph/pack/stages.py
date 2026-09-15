"""Outer-iter pack stages (compose/refine/mid/first-pass/post)."""

import time
from typing import Any, Callable, Sequence

from nest_graph.graph import score_elems
from nest_graph.pack.ctx import (
    FirstPassBorderResult,
    MidPackStagesResult,
    PackIterCtx,
    RefinePackBox,
)
from nest_graph.pack.execute import execute_pack, schedule_prep_selection_free
from nest_graph.pack.geoms import selection_coverage_pct
from nest_graph.propose.block_replace import maybe_block_hole_renest
from nest_graph.propose.context import outline_coverage_ratio
from nest_graph.propose.heavy_polish import (
    apply_dfs_refinement,
    apply_refine_with_restore,
    polish_budget_for_iter,
)
from nest_graph.propose.placement_common import selection_pairwise_independent
from nest_graph.propose.post_pack import (
    PostPackPrep,
    apply_post_pack_and_telem,
)
from nest_graph.propose.selection_compose import (
    ComposedSelection,
    compose_and_nest_selection,
    compose_nest_kwargs,
    dual_nest_for,
    kiss_lock_subset,
    sheet_diag_from,
)
from nest_graph.propose.first_pass_border import (
    first_pass_border_ring_selection,
    first_pass_layered_selection,
)
from nest_graph.propose.void_selection import pin_nest_void_independent
from nest_graph.propose.motif_lock import partition_packed_for_grow_classify


def rim_before_for_selection(
    *,
    selected: Sequence[int],
    polys: list,
    sheet,
    min_dist: float,
    group_id: Sequence[int],
    transform: Sequence,
    part_bases: dict,
    native_geoms_fn: Callable,
    fallback: float = 0.0,
) -> float:
    """Shared rim_before oracle (build_graph + evaluator)."""
    try:
        nest_geoms = [
            polys[i] for i in selected
            if 0 <= int(i) < len(polys)
            and polys[i] is not None
            and not polys[i].is_empty
        ]
        pack_geoms = (
            native_geoms_fn(
                [group_id[i] for i in selected],
                [transform[i] for i in selected],
                part_bases,
            )
            if selected else None
        )
        return float(outline_coverage_ratio(
            nest_geoms, sheet, min_dist, pack_geoms=pack_geoms,
        ))
    except Exception:
        return float(fallback)


def _compose_from_ctx(
    ctx: PackIterCtx,
    box: RefinePackBox,
    *,
    cheap: bool = False,
    **overrides,
) -> None:
    """Shared compose+nest body for mid-pack and Uh post-rim."""
    assert ctx.native_geoms_fn is not None
    if (
        ctx.graph_native_geoms is not None
        and len(ctx.graph_native_geoms) == len(ctx.group_id)
    ):
        candidate_geoms = list(ctx.graph_native_geoms)
    else:
        candidate_geoms = ctx.native_geoms_fn(
            ctx.group_id, ctx.transform, ctx.part_bases,
        )
    # W1 first_packed: rebuild nest packed from transforms (emit SoT), then
    # prepend seed solids so obstacle union stays seeds∪nest without mutating
    # board void_geoms (3b / board_ctx SoT).
    void_geoms = list(overrides.pop("void_geoms", ctx.void_geoms) or [])
    packed_geoms = list(overrides.pop("packed_geoms", ctx.packed_geoms))
    pgid = overrides.pop("packed_group_id", ctx.packed_group_id)
    ptf = overrides.pop("packed_transform", ctx.packed_transform)
    grow_classify_seed: list = []
    grow_classify_mapped: list = []
    grow_classify_unmapped: list = []
    if (
        pgid is not None
        and ptf is not None
        and len(pgid) == len(ptf)
        and len(pgid) > 0
    ):
        try:
            rebuilt = ctx.native_geoms_fn(pgid, ptf, ctx.part_bases)
            if rebuilt:
                seed_geoms = [g for g in (ctx.seed_void_geoms or []) if g is not None]
                packed_geoms = list(seed_geoms) + list(rebuilt)
                if ctx.propose_stats is not None:
                    ctx.propose_stats["w1_obstacle_sot"] = 1
                    ctx.propose_stats["w1_packed_rebuilt_n"] = int(len(rebuilt))
                grow_classify_seed, grow_classify_mapped, grow_classify_unmapped = (
                    partition_packed_for_grow_classify(
                        seed_geoms=seed_geoms,
                        rebuilt_geoms=rebuilt,
                        packed_group_id=pgid,
                        packed_transform=ptf,
                        group_id=ctx.group_id,
                        transform=ctx.transform,
                    )
                )
                if ctx.propose_stats is not None:
                    # Counts only on propose_stats (no Geometry lists — avoid cache/leak churn).
                    ctx.propose_stats["grow_classify_seed_n"] = int(len(grow_classify_seed))
                    ctx.propose_stats["grow_classify_mapped_n"] = int(len(grow_classify_mapped))
                    ctx.propose_stats["grow_classify_unmapped_n"] = int(len(grow_classify_unmapped))
        except Exception:
            pass
    kwargs = compose_nest_kwargs(
        graph=ctx.graph,
        rule_sets=ctx.rule_sets,
        active_rules=overrides.pop("active_rules", ctx.active_rules),
        scores=list(overrides.pop("scores", ctx.scores)),
        polys=ctx.polys,
        group_id=ctx.group_id,
        transform=ctx.transform,
        candidate_geoms=candidate_geoms,
        packed_geoms=packed_geoms,
        part_areas=ctx.part_areas,
        free_info=overrides.pop("free_info", ctx.free_info),
        cfg=ctx.cfg,
        selection=ctx.sel,
        first_pass=overrides.pop("first_pass", ctx.first_pass),
        outline=ctx.sheet,
        min_dist=float(overrides.pop("min_dist", ctx.min_dist)),
        sheet_area=float(ctx.sheet_area),
        sheet_diag=float(ctx.sheet_diag),
        propose_stats=ctx.propose_stats,
        ngroups=int(ctx.ngroups),
        packed_group_id=pgid,
        packed_transform=ptf,
        last_leaf=bool(overrides.pop("last_leaf", ctx.is_last_leaf)),
        void_geoms=void_geoms,
        locked_seed=overrides.pop("locked_seed", ctx.locked_seed),
        dg=overrides.pop("dg", ctx.dg),
        motif_base=overrides.pop("motif_base", ctx.motif_base),
        survive_by_motif=overrides.pop("survive_by_motif", ctx.survive_by_motif),
        grow_classify_seed=grow_classify_seed,
        grow_classify_mapped=grow_classify_mapped,
        grow_classify_unmapped=grow_classify_unmapped,
    )
    kwargs.update(overrides)
    _compose_t0 = time.perf_counter()
    composed = compose_and_nest_selection(**kwargs)
    if ctx.propose_stats is not None:
        ctx.propose_stats["compose_ms"] = float(
            (time.perf_counter() - _compose_t0) * 1000.0
        )
    box.composed = composed
    large_void = bool(
        composed.free_info is not None
        and composed.free_info.kind == "large_void"
    )
    box.budget = polish_budget_for_iter(
        is_last_leaf=ctx.is_last_leaf,
        sel=ctx.sel,
        large_void=large_void,
        cheap_expand=cheap,
        near_last=ctx.near_last,
        on_plateau=bool(ctx.propose_stats.get("on_plateau", False)),
        free_remaining=large_void,
    )


def _refine_from_ctx(ctx: PackIterCtx, box: RefinePackBox) -> None:
    """Shared refine body for mid-pack."""
    composed = box.composed
    budget = box.budget
    assert composed is not None and budget is not None
    assert ctx.native_geoms_fn is not None
    rim_before = rim_before_for_selection(
        selected=composed.selected_nest,
        polys=ctx.polys,
        sheet=ctx.sheet,
        min_dist=float(ctx.min_dist),
        group_id=ctx.group_id,
        transform=ctx.transform,
        part_bases=ctx.part_bases,
        native_geoms_fn=ctx.native_geoms_fn,
        fallback=float(ctx.propose_stats.get("outline_cov", 0.0) or 0.0),
    )
    node_areas = [
        float(ctx.part_areas[int(g)]) if int(g) < len(ctx.part_areas) else 0.0
        for g in ctx.group_id
    ]
    box.selected = list(apply_refine_with_restore(
        budget=budget,
        apply_dfs_fn=apply_dfs_refinement,
        graph=ctx.graph,
        refine_rules=composed.refine_rules,
        selected_nest=composed.selected_nest,
        refine_scores=composed.refine_scores,
        sel_iter=ctx.sel,
        node_areas=node_areas,
        refine_seed=int(ctx.refine_seed),
        locked_indices=list(ctx.propose_stats.get("motif_locked") or ()),
        polys=ctx.polys,
        group_id=ctx.group_id,
        transform=ctx.transform,
        part_areas=ctx.part_areas,
        part_bases=ctx.part_bases,
        sheet=ctx.sheet,
        min_dist=float(ctx.min_dist),
        rim_before=rim_before,
        rim_reject=float(ctx.rim_reject),
        propose_stats=ctx.propose_stats,
        native_geoms_from_transforms_fn=ctx.native_geoms_fn,
        free_info=composed.free_info,
        free_poly=composed.free_poly,
        dg=ctx.dg,
        propose_cfg=ctx.cfg.propose,
    ))
    try:
        box.coverage = float(selection_coverage_pct(
            box.selected, ctx.group_id, ctx.part_areas,
            float(ctx.sheet_area or 1.0),
        )) / 100.0
    except Exception:
        box.coverage = 0.0


def run_compose_refine_pack(
    ctx: PackIterCtx,
    box: RefinePackBox,
    *,
    cheap: bool = False,
) -> None:
    """Single compose+refine body wired through execute_pack (Q148)."""
    stage = execute_pack(
        rim_only=False,
        heavy=False,
        compose_fn=lambda: _compose_from_ctx(ctx, box, cheap=cheap),
        refine_fn=lambda: _refine_from_ctx(ctx, box),
    )
    ctx.propose_stats.update(stage)
    ctx.propose_stats["execute_wired"] = 1


def run_mid_pack_stages(
    ctx: PackIterCtx,
    box: RefinePackBox,
    *,
    graph: Any,
    part_by_group: dict,
    void_geoms_compose: Sequence | None,
    archived_for_propose: Sequence | None,
    pin_all_blocked_streak: int,
    cheap: bool = False,
) -> tuple[MidPackStagesResult, int]:
    """Compose+refine via execute_pack, then 3b + pin (mid-pack after refine)."""
    run_compose_refine_pack(ctx, box, cheap=cheap)
    composed = box.composed
    budget = box.budget
    assert composed is not None and budget is not None
    selected_polys = list(box.selected)
    candidate_geoms = (
        ctx.native_geoms_fn(ctx.group_id, ctx.transform, ctx.part_bases)
        if ctx.native_geoms_fn is not None else None
    )
    if ctx.enable_3b:
        (
            selected_polys,
            ctx.polys,
            ctx.transform,
            ctx.group_id,
            candidate_geoms,
            _hole_telem,
        ) = maybe_block_hole_renest(
            selected=selected_polys,
            polys=list(ctx.polys),
            transforms=list(ctx.transform),
            group_id=list(ctx.group_id),
            candidate_geoms=candidate_geoms,
            refine_scores=composed.refine_scores,
            part_areas=ctx.part_areas,
            part_by_group=part_by_group,
            sheet=ctx.sheet,
            min_dist=float(ctx.min_dist),
            propose_cfg=ctx.cfg.propose,
            free_info=composed.free_info,
            free_poly=composed.free_poly,
            void_geoms=list(void_geoms_compose or []),
            cluster_patterns=archived_for_propose or None,
            polish_budget=budget,
            propose_stats=ctx.propose_stats,
        )
    pin_stats: dict = {}
    skip_pin = (
        int(getattr(ctx.cfg.propose, "pin_all_blocked_skip_after", 3) or 0) > 0
        and pin_all_blocked_streak
        >= int(getattr(ctx.cfg.propose, "pin_all_blocked_skip_after", 3) or 0)
    )
    if (
        bool(getattr(ctx.cfg.propose, "enable_void_nest_pin", True))
        and not skip_pin
        and composed.free_poly is not None
        and not composed.free_poly.is_empty
    ):
        n_graph = len(graph.collisions)
        extra_3b = [i for i in selected_polys if int(i) >= n_graph]
        selected_polys = pin_nest_void_independent(
            graph,
            composed.selected_nest,
            [i for i in selected_polys if int(i) < n_graph],
            ctx.polys,
            composed.free_poly,
            composed.refine_scores,
            stats_out=pin_stats,
        )
        if extra_3b:
            merged = list(extra_3b)
            for i in selected_polys:
                trial = merged + [int(i)]
                if selection_pairwise_independent(ctx.polys, trial):
                    merged.append(int(i))
            selected_polys = merged
    else:
        pin_stats = {
            "pin_candidates": 0,
            "pin_added": 0,
            "pin_blocked_collision": 0,
            "pin_ms": 0.0,
            "pin_skipped_streak": int(skip_pin),
        }
    pin_cands = int(pin_stats.get("pin_candidates", 0))
    pin_added = int(pin_stats.get("pin_added", 0))
    if pin_cands > 0 and pin_added == 0:
        pin_all_blocked_streak += 1
    elif not skip_pin:
        pin_all_blocked_streak = 0
    result = MidPackStagesResult(
        selected_polys=list(selected_polys),
        polys=list(ctx.polys),
        transform=list(ctx.transform),
        group_id=list(ctx.group_id),
        candidate_geoms=candidate_geoms,
        pin_stats=pin_stats,
        selected_nest=list(composed.selected_nest),
        refine_scores=list(composed.refine_scores),
        free_poly=composed.free_poly,
        free_info=composed.free_info,
        n_void_nest=int(composed.n_void_nest),
        boost_hits=dict(composed.boost_hits or {}),
        polish_budget=budget,
    )
    return result, pin_all_blocked_streak


def run_uh_post_rim_compose(
    ctx: PackIterCtx,
    box: RefinePackBox,
    *,
    rim_idxs: Sequence[int],
    rim_pack: Sequence,
    void_geoms: Sequence | None,
    kiss_lock: Sequence[int] | None,
    active_rules: Any,
    scores: Sequence[float],
) -> ComposedSelection:
    """Uh void compose via execute_pack rim_only stage (R4 first-pass profile)."""
    assert ctx.native_geoms_fn is not None

    def uh_void_work() -> None:
        _compose_from_ctx(
            ctx,
            box,
            active_rules=active_rules,
            scores=list(scores),
            packed_geoms=list(rim_pack),
            first_pass=True,
            packed_group_id=[
                int(ctx.group_id[i]) for i in rim_idxs if int(i) < len(ctx.group_id)
            ],
            packed_transform=[
                ctx.transform[i] for i in rim_idxs if int(i) < len(ctx.transform)
            ],
            last_leaf=True,
            void_geoms=void_geoms,
            locked_seed=kiss_lock or None,
        )

    stage = execute_pack(rim_only=True, uh_void_fn=uh_void_work)
    ctx.propose_stats.update(stage)
    ctx.propose_stats["execute_wired"] = 1
    assert box.composed is not None
    return box.composed


def run_first_pass_border_pack(
    ctx: PackIterCtx,
    *,
    seed_rules: Any,
    parts: list,
    p1,
    p2,
    board_ctx_outline: tuple,
    sel_iter: Any,
    active_rules: Any,
    make_polygon_graph_fn: Callable,
) -> FirstPassBorderResult:
    """First-pass border ring → layered saturation → optional Uh post-rim compose."""
    assert ctx.native_geoms_fn is not None
    scores = list(score_elems(ctx.graph, seed_rules))
    sheet, void_geoms_uh = board_ctx_outline
    min_dist = ctx.cfg.board_min_dist_for(ctx.sheet, first_pass=True)
    selected_polys = first_pass_border_ring_selection(
        ctx.graph, ctx.polys, ctx.sheet, min_dist, scores,
    )
    graph = ctx.graph
    polys = list(ctx.polys)
    group_id = list(ctx.group_id)
    transform = list(ctx.transform)
    free_info = ctx.free_info

    if ctx.cfg.propose.first_pass_layered_pack and selected_polys:
        graph, polys, group_id, transform, selected_polys = first_pass_layered_selection(
            ctx.cfg,
            ctx.sheet,
            parts,
            graph=graph,
            p1=p1,
            p2=p2,
            polys=polys,
            group_id=group_id,
            transform=transform,
            phase1_selected=list(selected_polys),
            rule_set=seed_rules,
            scores=scores,
            propose_stats=ctx.propose_stats,
            dg=ctx.dg,
            make_polygon_graph_fn=make_polygon_graph_fn,
            native_geoms_fn=ctx.native_geoms_fn,
        )

    try:
        md_rim = ctx.cfg.board_min_dist_for(ctx.sheet, first_pass=True)
        rim_geoms = [
            polys[i] for i in selected_polys
            if 0 <= int(i) < len(polys)
            and polys[i] is not None
            and not polys[i].is_empty
        ]
        rim_pack = ctx.native_geoms_fn(
            [group_id[i] for i in selected_polys],
            [transform[i] for i in selected_polys],
            ctx.part_bases,
        ) if selected_polys else []
        post_rim = schedule_prep_selection_free(
            phase="uh",
            sheet=sheet,
            part_areas=ctx.part_areas,
            min_dist=md_rim,
            cfg_propose=ctx.cfg.propose,
            packed_shapely=rim_geoms,
            pack_geoms=rim_pack,
            run_uh=True,
        )
        if post_rim is None:
            raise RuntimeError("uh free prep skipped")
        ctx.propose_stats["post_rim_free_kind"] = getattr(
            post_rim.free_info, "kind", None
        )
        ctx.propose_stats["post_rim_uh_candidate"] = int(
            ctx.propose_stats["post_rim_free_kind"] == "large_void"
        )
        print(
            f"r0 post_rim free={ctx.propose_stats['post_rim_free_kind']} "
            f"uh_candidate={ctx.propose_stats['post_rim_uh_candidate']}"
        )
        if ctx.propose_stats["post_rim_uh_candidate"] and selected_polys:
            scores_uh = list(score_elems(graph, active_rules))
            sheet_diag_uh = sheet_diag_from(sheet)
            rim_idxs = list(selected_polys)
            kiss_lock = kiss_lock_subset(
                polys, ctx.sheet, md_rim, rim_idxs, max_n=3,
            )
            uh_box = RefinePackBox()
            uh_ctx = PackIterCtx(
                graph=graph,
                polys=polys,
                group_id=group_id,
                transform=transform,
                part_areas=ctx.part_areas,
                part_bases=ctx.part_bases,
                cfg=ctx.cfg,
                sel=sel_iter,
                propose_stats=ctx.propose_stats,
                dg=ctx.dg,
                sheet=ctx.sheet,
                min_dist=md_rim,
                rule_sets=ctx.rule_sets,
                free_info=post_rim.free_info,
                sheet_diag=float(sheet_diag_uh),
                sheet_area=float(sheet.area) if sheet is not None else 0.0,
                ngroups=int(ctx.cfg.rules.ngroups),
                first_pass=True,
                enable_3b=False,
                native_geoms_fn=ctx.native_geoms_fn,
            )
            composed_uh = run_uh_post_rim_compose(
                uh_ctx,
                uh_box,
                rim_idxs=rim_idxs,
                rim_pack=rim_pack,
                void_geoms=void_geoms_uh,
                kiss_lock=kiss_lock or None,
                active_rules=active_rules,
                scores=scores_uh,
            )
            selected_polys = composed_uh.selected_nest
            free_info = composed_uh.free_info
            ctx.propose_stats["uh_post_rim"] = 1
            ctx.propose_stats["uh_n_void"] = int(composed_uh.n_void_nest)
            ctx.propose_stats["uh_n_sel"] = int(len(selected_polys))
            ctx.propose_stats["uh_kiss_lock_n"] = int(len(kiss_lock))
            print(
                f"uh post_rim void_nest={composed_uh.n_void_nest} "
                f"sel={len(selected_polys)} kiss_lock={len(kiss_lock)} "
                f"beam_unlocked={ctx.propose_stats.get('uh_beam_unlocked', 0)}"
            )
    except Exception:
        ctx.propose_stats["post_rim_free_kind"] = None
        ctx.propose_stats["post_rim_uh_candidate"] = 0
        ctx.propose_stats["uh_post_rim"] = 0

    return FirstPassBorderResult(
        graph=graph,
        polys=polys,
        group_id=group_id,
        transform=transform,
        selected_polys=list(selected_polys),
        free_info=free_info,
        old_len=0,
    )


def _run_post_pack_work(
    ctx: PackIterCtx,
    prep: PostPackPrep,
    *,
    selected_polys: Sequence[int],
    part_by_group: dict,
    fixed_obstacles: Sequence | None,
    void_leak_stats: dict | None,
    out: dict,
) -> None:
    part_bases = ctx.part_bases
    native_geoms_fn = ctx.native_geoms_fn

    def native_pack_geoms(sel, tr, gids):
        return native_geoms_fn(
            [gids[i] for i in sel],
            [tr[i] for i in sel],
            part_bases,
        ) if sel and native_geoms_fn is not None and part_bases is not None else None

    polys, transform, sel, pack_stats = apply_post_pack_and_telem(
        prep,
        polys=out["polys"],
        transform=out["transform"],
        group_id=ctx.group_id,
        selected_polys=out["sel"],
        part_by_group=part_by_group,
        min_dist=float(ctx.min_dist),
        cfg_propose=ctx.cfg.propose,
        fixed_obstacles=fixed_obstacles,
        part_bases=part_bases,
        native_pack_geoms_fn=native_pack_geoms if native_geoms_fn is not None else None,
        void_leak_stats=void_leak_stats,
    )
    out["polys"] = polys
    out["transform"] = transform
    out["sel"] = sel
    out["pack_stats"] = pack_stats


def run_post_pack_stage(
    ctx: PackIterCtx,
    prep: PostPackPrep,
    *,
    selected_polys: Sequence[int],
    part_by_group: dict,
    fixed_obstacles: Sequence | None,
    void_leak_stats: dict | None = None,
    polish_budget=None,
) -> tuple[list, list, list[int], dict]:
    """Post-pack repack/relocate/se2 when budget allows (no 3b)."""
    empty_stats: dict = {}
    if polish_budget is None or not polish_budget.run_post_pack:
        if isinstance(void_leak_stats, dict):
            void_leak_stats.setdefault("repack", {"attempted": 0, "accepted": 0})
            void_leak_stats.setdefault("relocate", {})
            void_leak_stats.setdefault("local_se2", {})
        return list(ctx.polys), list(ctx.transform), list(selected_polys), empty_stats
    if prep.push_pt is None or prep.free_post.kind != "large_void":
        return list(ctx.polys), list(ctx.transform), list(selected_polys), empty_stats
    out: dict = {
        "polys": list(ctx.polys),
        "transform": list(ctx.transform),
        "sel": list(selected_polys),
        "pack_stats": {},
    }
    stage = execute_pack(
        heavy=True,
        post_pack_fn=lambda: _run_post_pack_work(
            ctx,
            prep,
            selected_polys=selected_polys,
            part_by_group=part_by_group,
            fixed_obstacles=fixed_obstacles,
            void_leak_stats=void_leak_stats,
            out=out,
        ),
    )
    ctx.propose_stats.update(stage)
    return out["polys"], out["transform"], out["sel"], out["pack_stats"]


__all__ = [
    "rim_before_for_selection",
    "run_compose_refine_pack",
    "run_first_pass_border_pack",
    "run_mid_pack_stages",
    "run_post_pack_stage",
    "run_uh_post_rim_compose",
]
