"""Shared outer-iter pack body for build_graph and nesting_evaluator (Q148)."""

from dataclasses import dataclass, field
from typing import Any, Callable, Sequence

import numpy as np

from nest_graph.decision.epoch import materialize_final_selection
from nest_graph.decision.execute import (
    execute_pack,
    schedule_prep_selection_free,
)
from nest_graph.decision.motif_credit import (
    credit_motif_on_nest_survival,
    credit_void_niche_from_iter,
    niche_amaf_key,
)
from nest_graph.elem_graph import MacroRegion, score_elems
from nest_graph.propose.block_replace import maybe_block_hole_renest
from nest_graph.propose.context import outline_coverage_ratio
from nest_graph.propose.pattern_archive import note_motif_hollow_miss
from nest_graph.propose.placement_common import selection_pairwise_independent
from nest_graph.propose.telem import (
    VoidLeakGatherCtx,
    archive_void_elite_transforms,
    gather_void_leak_inputs,
    void_elite_count,
)
from nest_graph.propose.void_selection import (
    count_props_near_pole,
    pin_nest_void_independent,
    transform_row_key,
    void_pole_near_radius,
)
from nest_graph.propose.heavy_polish import (
    PolishBudget,
    apply_refine_with_restore,
    polish_budget_for_iter,
)
from nest_graph.propose.post_pack import (
    PostPackPrep,
    apply_post_pack_and_telem,
    prepare_post_pack,
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


@dataclass
class PackIterCtx:
    """One outer-iter pack profile + shared graph state."""

    graph: Any
    polys: list
    group_id: list
    transform: list
    part_areas: Sequence[float]
    part_bases: dict
    cfg: Any
    sel: Any
    propose_stats: dict
    dg: Any = None
    nest_state: Any = None
    sheet: Any = None
    min_dist: float = 0.0
    rule_sets: list = field(default_factory=list)
    active_rules: Any = None
    scores: list = field(default_factory=list)
    free_info: Any = None
    free_poly: Any = None
    void_geoms: Sequence | None = None
    packed_geoms: list = field(default_factory=list)
    packed_group_id: Sequence | None = None
    packed_transform: Sequence | None = None
    sheet_diag: float = 0.0
    sheet_area: float = 0.0
    ngroups: int = 0
    is_last_leaf: bool = False
    near_last: bool = False
    refine_seed: int = 0
    locked_indices: list = field(default_factory=list)
    rim_reject: float = 0.02
    first_pass: bool = False
    rim_emphasis: bool = False
    skip_mid_compose: bool = False
    skip_refine: bool = False
    uh_enabled: bool = False
    cheap_expand: bool = False
    enable_3b: bool = True
    locked_seed: Sequence | None = None
    native_geoms_fn: Callable | None = None
    apply_dfs_fn: Callable | None = None
    coverage_pct_fn: Callable | None = None


@dataclass
class RefinePackBox:
    """Mutable compose+refine scratch (replaces nested closures)."""

    composed: ComposedSelection | None = None
    budget: PolishBudget | None = None
    selected: list = field(default_factory=list)
    coverage: float = 0.0


@dataclass
class MidPackStagesResult:
    selected_polys: list
    polys: list
    transform: list
    group_id: list
    candidate_geoms: list | None
    pin_stats: dict
    selected_nest: list
    refine_scores: list
    free_poly: Any
    free_info: Any
    n_void_nest: int
    boost_hits: dict
    polish_budget: PolishBudget


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


def run_compose_refine_pack(
    ctx: PackIterCtx,
    box: RefinePackBox,
    *,
    cheap: bool = False,
) -> None:
    """Single compose+refine body wired through execute_pack (Q148)."""
    assert ctx.native_geoms_fn is not None
    assert ctx.apply_dfs_fn is not None

    dual_nest = dual_nest_for(ctx.free_info, last_leaf=ctx.is_last_leaf)
    candidate_geoms = ctx.native_geoms_fn(
        ctx.group_id, ctx.transform, ctx.part_bases,
    )

    def compose_fn() -> None:
        composed = compose_and_nest_selection(
            **compose_nest_kwargs(
                graph=ctx.graph,
                rule_sets=ctx.rule_sets,
                active_rules=ctx.active_rules,
                scores=list(ctx.scores),
                polys=ctx.polys,
                group_id=ctx.group_id,
                transform=ctx.transform,
                candidate_geoms=candidate_geoms,
                packed_geoms=list(ctx.packed_geoms),
                part_areas=ctx.part_areas,
                free_info=ctx.free_info,
                cfg=ctx.cfg,
                selection=ctx.sel,
                first_pass=ctx.first_pass,
                outline=ctx.sheet,
                min_dist=float(ctx.min_dist),
                sheet_area=float(ctx.sheet_area),
                sheet_diag=float(ctx.sheet_diag),
                propose_stats=ctx.propose_stats,
                ngroups=int(ctx.ngroups),
                packed_group_id=ctx.packed_group_id,
                packed_transform=ctx.packed_transform,
                last_leaf=ctx.is_last_leaf,
                void_geoms=ctx.void_geoms,
                locked_seed=ctx.locked_seed,
                dg=ctx.dg,
            ),
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

    def refine_fn() -> None:
        composed = box.composed
        budget = box.budget
        assert composed is not None and budget is not None
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
            apply_dfs_fn=ctx.apply_dfs_fn,
            graph=ctx.graph,
            refine_rules=composed.refine_rules,
            selected_nest=composed.selected_nest,
            refine_scores=composed.refine_scores,
            sel_iter=ctx.sel,
            node_areas=node_areas,
            refine_seed=int(ctx.refine_seed),
            locked_indices=list(ctx.locked_indices),
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
        if ctx.coverage_pct_fn is not None:
            try:
                box.coverage = float(ctx.coverage_pct_fn(
                    box.selected, ctx.group_id, ctx.part_areas,
                    float(ctx.sheet_area or 1.0),
                )) / 100.0
            except Exception:
                box.coverage = 0.0

    if ctx.skip_mid_compose and ctx.skip_refine:
        return
    compose_cb = None if ctx.skip_mid_compose else compose_fn
    refine_cb = None if ctx.skip_refine else refine_fn
    stage = execute_pack(
        rim_only=False,
        heavy=False,
        compose_fn=compose_cb,
        refine_fn=refine_cb,
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
            # Keep 3b hole appends; drop pin/graph rows that collide with them.
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
    candidate_geoms = ctx.native_geoms_fn(
        ctx.group_id, ctx.transform, ctx.part_bases,
    )

    def uh_void_fn() -> None:
        composed = compose_and_nest_selection(
            **compose_nest_kwargs(
                graph=ctx.graph,
                rule_sets=ctx.rule_sets,
                active_rules=active_rules,
                scores=list(scores),
                polys=ctx.polys,
                group_id=ctx.group_id,
                transform=ctx.transform,
                candidate_geoms=candidate_geoms,
                packed_geoms=list(rim_pack),
                part_areas=ctx.part_areas,
                free_info=ctx.free_info,
                cfg=ctx.cfg,
                selection=ctx.sel,
                first_pass=True,
                outline=ctx.sheet,
                min_dist=float(ctx.min_dist),
                sheet_area=float(ctx.sheet_area),
                sheet_diag=float(ctx.sheet_diag),
                propose_stats=ctx.propose_stats,
                ngroups=int(ctx.ngroups),
                packed_group_id=[
                    int(ctx.group_id[i]) for i in rim_idxs if int(i) < len(ctx.group_id)
                ],
                packed_transform=[
                    ctx.transform[i] for i in rim_idxs if int(i) < len(ctx.transform)
                ],
                last_leaf=True,
                void_geoms=void_geoms,
                locked_seed=kiss_lock or None,
                dg=ctx.dg,
            ),
        )
        box.composed = composed

    stage = execute_pack(rim_only=True, uh_void_fn=uh_void_fn)
    ctx.propose_stats.update(stage)
    ctx.propose_stats["execute_wired"] = 1
    assert box.composed is not None
    return box.composed


@dataclass
class FirstPassBorderResult:
    graph: Any
    polys: list
    group_id: list
    transform: list
    selected_polys: list[int]
    free_info: Any | None = None
    old_len: int = 0


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
                rim_emphasis=True,
                uh_enabled=True,
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


def run_post_pack_stage(
    ctx: PackIterCtx,
    prep: PostPackPrep,
    *,
    selected_polys: Sequence[int],
    part_by_group: dict,
    fixed_obstacles: Sequence | None,
    void_leak_stats: dict | None = None,
    polish_budget: PolishBudget | None = None,
) -> tuple[list, list, list[int], dict]:
    """Post-pack repack/relocate/se2 when budget allows (no 3b)."""
    budget = polish_budget
    empty_stats: dict = {}
    if budget is None or not budget.run_post_pack:
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
    native_fn = None
    if ctx.native_geoms_fn is not None and ctx.part_bases is not None:
        part_bases = ctx.part_bases
        native_geoms_fn = ctx.native_geoms_fn

        def native_fn(sel, tr, gids):
            return native_geoms_fn(
                [gids[i] for i in sel],
                [tr[i] for i in sel],
                part_bases,
            ) if sel else None

    def post_pack_fn() -> None:
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
            part_bases=ctx.part_bases,
            native_pack_geoms_fn=native_fn,
            void_leak_stats=void_leak_stats,
        )
        out["polys"] = polys
        out["transform"] = transform
        out["sel"] = sel
        out["pack_stats"] = pack_stats

    stage = execute_pack(heavy=True, post_pack_fn=post_pack_fn)
    ctx.propose_stats.update(stage)
    return out["polys"], out["transform"], out["sel"], out["pack_stats"]


def invalidate_cheap_cache(pack_cache: dict, *, reason: str) -> None:
    """Clear cheap_by_key when proposer context shifts (D0)."""
    cheap_map = pack_cache.get("cheap_by_key")
    if isinstance(cheap_map, dict) and cheap_map:
        cheap_map.clear()
        pack_cache["cache_invalidate_reason"] = str(reason)
        pack_cache["cache_invalidate_n"] = int(
            pack_cache.get("cache_invalidate_n", 0) or 0
        ) + 1


def maybe_invalidate_cheap_cache(
    pack_cache: dict,
    *,
    remaining_gids: Sequence[int] | None = None,
    void_elite_seeded: int | None = None,
    archive_elite_n: int | None = None,
) -> None:
    """D0 staleness guard: invalidate when pool/elite context shifts."""
    if remaining_gids is not None:
        rem_key = tuple(sorted(int(g) for g in remaining_gids))
        prev_rem = pack_cache.get("last_remaining_gids")
        if prev_rem is not None and prev_rem != rem_key:
            invalidate_cheap_cache(pack_cache, reason="remaining_gids")
        pack_cache["last_remaining_gids"] = rem_key
    if void_elite_seeded is not None:
        prev_elite = int(pack_cache.get("last_void_elite_seeded", -1))
        if prev_elite >= 0 and prev_elite != int(void_elite_seeded):
            invalidate_cheap_cache(pack_cache, reason="void_elite_seeded")
        pack_cache["last_void_elite_seeded"] = int(void_elite_seeded)
    if archive_elite_n is not None:
        prev_arch = int(pack_cache.get("last_archive_elite_n", -1))
        if prev_arch >= 0 and prev_arch != int(archive_elite_n):
            invalidate_cheap_cache(pack_cache, reason="archive_elite_n")
        pack_cache["last_archive_elite_n"] = int(archive_elite_n)


@dataclass
class VoidLeakOrchResult:
    void_elite_by_group: dict
    leak_dict: dict
    void_leak: str
    had_void_override: bool
    n_void_graph: int
    outline_cov: float
    proposer_keys: dict
    niche_telem: dict
    prev_void_nest: int


def run_void_leak_and_niche_credit(
    *,
    graph: Any,
    polys: list,
    group_id: list,
    transform: list,
    free_info: Any,
    free_poly: Any,
    selected_nest: Sequence[int],
    selected_polys: Sequence[int],
    refine_scores: Sequence[float],
    propose_stats: dict,
    plateau: Any,
    pin_stats: dict,
    pin_all_blocked_streak: int,
    n_void_nest: int,
    boost_hits: dict,
    void_pole_near_diag_ratio: float,
    proposer_counts: dict,
    sheet_diag: float,
    mcts_telem: dict,
    mcts_runner: Any,
    mcts_action: Any | None,
    cfg_propose: Any,
    prev_void_nest: int,
    pin_cands: int,
    pin_added: int,
    proposed_list: Sequence | None,
    stratified_void_elite_quota: int = 15,
    print_funnel: bool = True,
) -> VoidLeakOrchResult:
    """Void-elite archive + gather_void_leak + niche credit (build_graph + evaluator)."""
    n_props_pole = count_props_near_pole(
        proposed_list,
        getattr(free_info, "target_pt", None),
        void_pole_near_radius(
            float(sheet_diag),
            float(void_pole_near_diag_ratio),
        ),
    )
    archive_enabled = bool(getattr(cfg_propose, "enable_void_elite_archive", True))
    if (
        archive_enabled
        and bool(getattr(cfg_propose, "stop_elite_archive_when_pin_blocked", True))
        and pin_cands > 0
        and pin_added == 0
    ):
        densify_keep = int(
            (propose_stats.get("densify_stats") or {}).get("accepted", 0) or 0
        ) > 0
        if not densify_keep and int(n_props_pole) <= 0:
            archive_enabled = False
    void_elite_by_group = archive_void_elite_transforms(
        selected_nest=selected_nest,
        selected_refine=selected_polys,
        polys=polys,
        transforms=transform,
        group_ids=group_id,
        free_poly=free_poly,
        scores=refine_scores,
        max_keep=int(stratified_void_elite_quota),
        enabled=archive_enabled,
    )
    void_leak, leak_dict = gather_void_leak_inputs(
        VoidLeakGatherCtx(
            graph=graph,
            polys=polys,
            group_id=group_id,
            transform=transform,
            free_info=free_info,
            free_poly=free_poly,
            selected_nest=selected_nest,
            selected_polys=selected_polys,
            refine_scores=refine_scores,
            propose_stats=propose_stats,
            plateau=plateau,
            pin_stats=pin_stats,
            pin_all_blocked_streak=pin_all_blocked_streak,
            n_void_nest=n_void_nest,
            boost_hits=boost_hits,
            void_pole_near_diag_ratio=float(void_pole_near_diag_ratio),
            proposer_counts=proposer_counts,
            sheet_diag=float(sheet_diag),
            void_elite_by_group=void_elite_by_group,
            void_elite_count_fn=void_elite_count,
            mcts_telem=mcts_telem,
            mcts_runner=mcts_runner,
        )
    )
    had_void_override = bool(
        int(leak_dict.get("hijack", 0))
        or propose_stats.get("sat_override", False)
    )
    n_void_graph = int(leak_dict.get("graph", 0))
    outline_cov = float(leak_dict.get("outline_cov", 0.0))
    proposer_keys = (
        propose_stats.get("proposer_keys")
        or (propose_stats.get("densify_stats") or {}).get("proposer_keys")
        or {}
    )
    propose_stats["void_leak"] = leak_dict
    funnel = leak_dict["funnel"]
    corners = leak_dict["corners"]
    inward = leak_dict.get("inward") or {}
    propose_stats["r0_bottleneck"] = funnel["bottleneck"]
    if print_funnel:
        print(
            f"r0 bottleneck={funnel['bottleneck']} "
            f"funnel={funnel['funnel_stages']} "
            f"corners_in={corners['corner_in']} kept={corners['corner_kept']} "
            f"inward_e={int(inward.get('inward_emitted', 0))} "
            f"pool={int(inward.get('inward_pool', 0))} "
            f"kept={int(inward.get('inward_kept', 0))} "
            f"ray={int(inward.get('raycasting_emitted', 0))}/"
            f"{int(inward.get('raycasting_pool', 0))} "
            f"voronoi={int(inward.get('voronoi_emitted', 0))}/"
            f"{int(inward.get('voronoi_pool', 0))} "
            f"erosion={int(inward.get('erosion_emitted', 0))}/"
            f"{int(inward.get('erosion_pool', 0))}"
        )
        print(void_leak)
    bottleneck = str(funnel.get("bottleneck") or "")
    large_void = str(getattr(free_info, "kind", "") or "") == "large_void"
    hollow = bool(
        large_void
        and (bottleneck == "graph_to_nest" or int(n_void_nest) <= 0)
    )
    propose_stats["hollow_miss"] = hollow
    amaf_key = niche_amaf_key(mcts_action)
    if hollow and mcts_runner.agent is not None:
        mcts_runner.agent.note_macro_miss(mcts_action)
        if (
            mcts_action is not None
            and mcts_action.region == MacroRegion.Motif
            and int(mcts_action.motif_id) >= 0
        ):
            note_motif_hollow_miss(
                mcts_runner.motif_base, int(mcts_action.motif_id),
            )
    niche_telem = credit_void_niche_from_iter(
        mcts_runner.niche_archive,
        free_kind=str(getattr(free_info, "kind", "") or ""),
        bottleneck=bottleneck,
        n_void_nest=int(n_void_nest),
        n_void_graph=int(n_void_graph),
        prev_void_nest=int(prev_void_nest),
        polys=polys,
        transform=transform,
        group_id=group_id,
        selected=selected_nest,
        free_poly=free_poly,
        outline_cov=float(outline_cov),
        ttl=int(getattr(cfg_propose, "accepted_pattern_ttl", 4) or 4),
        max_seed=int(getattr(cfg_propose, "max_proposals", 64) or 64),
        amaf_key=amaf_key,
        proposer_keys=proposer_keys,
    )
    propose_stats.update(niche_telem)
    if isinstance(propose_stats.get("void_leak"), dict):
        propose_stats["void_leak"]["niche_pos"] = int(
            niche_telem.get("niche_pos", 0)
        )
        propose_stats["void_leak"]["niche_rescue"] = int(
            niche_telem.get("niche_rescue", 0)
        )
    new_prev = int(n_void_nest)
    feed = mcts_runner.niche_archive.last_feed_keys
    if feed and large_void:
        nest_keys = {
            transform_row_key(np.asarray(transform[i], dtype=np.float64))
            for i in selected_nest
            if 0 <= int(i) < len(transform)
        }
        placed = bool(feed & nest_keys)
        mcts_runner.niche_archive.note_place_outcome(placed)
    mcts_runner.niche_archive.last_feed_keys = set()
    return VoidLeakOrchResult(
        void_elite_by_group=void_elite_by_group,
        leak_dict=leak_dict,
        void_leak=void_leak,
        had_void_override=had_void_override,
        n_void_graph=n_void_graph,
        outline_cov=outline_cov,
        proposer_keys=proposer_keys,
        niche_telem=niche_telem,
        prev_void_nest=new_prev,
    )


def finalize_iter_mcts(
    runner,
    *,
    selected_polys: Sequence[int],
    group_id: Sequence[int],
    transform: Sequence,
    propose_stats: dict,
    mcts_telem: dict | None,
    motif_keys: dict | None,
    motif_ttl: int,
    credit_motif: bool,
    refine_bp: dict | None = None,
    emitted_bp: dict | None = None,
) -> None:
    """Post-pack DG materialize + Kind/Attach credit (Q165 outer leaf only)."""
    if runner.agent is not None and refine_bp is not None and emitted_bp is not None:
        tot_emit = float(sum(int(v) for v in emitted_bp.values()) or 0)
        tot_surv = float(sum(int(v) for v in refine_bp.values()) or 0)
        if tot_emit > 0.0:
            prop_h = min(1.0, tot_surv / tot_emit)
            if int(propose_stats.get("cache_hit", 0) or 0) > 0 and int(
                propose_stats.get("proposal_count", 0) or 0
            ) <= 0:
                prop_h *= 0.25
            runner.agent.realized["proposer_pb"] = float(prop_h)
    materialize_final_selection(runner.dg, selected_polys, propose_stats)
    if runner.agent is not None:
        credit_motif_on_nest_survival(
            runner.motif_base,
            selected_polys=selected_polys,
            group_id=group_id,
            transform=transform,
            motif_keys=motif_keys,
            ttl=int(motif_ttl),
            telem=mcts_telem,
            realized_out=runner.agent.realized,
            kind_survive=propose_stats.get("kind_survive_hist"),
            materialized_attach=int(propose_stats.get("materialized_attach", 0) or 0),
            member_hits=int(propose_stats.get("member_hits", 0) or 0),
            credit_motif=bool(credit_motif),
        )
    if isinstance(propose_stats.get("void_leak"), dict):
        propose_stats["void_leak"]["kind_survive"] = int(
            propose_stats.get("kind_survive", 0) or 0
        )
        propose_stats["void_leak"]["materialized_attach"] = int(
            propose_stats.get("materialized_attach", 0) or 0
        )


__all__ = [
    "MidPackStagesResult",
    "PackIterCtx",
    "RefinePackBox",
    "finalize_iter_mcts",
    "invalidate_cheap_cache",
    "maybe_invalidate_cheap_cache",
    "rim_before_for_selection",
    "run_compose_refine_pack",
    "run_mid_pack_stages",
    "run_post_pack_stage",
    "VoidLeakOrchResult",
    "run_void_leak_and_niche_credit",
    "run_uh_post_rim_compose",
]
