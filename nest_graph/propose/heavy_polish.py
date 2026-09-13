"""One polish path: DFS refine + restore, budgeted by dfs_passes/mode."""

import time
from dataclasses import dataclass
from typing import Any, Sequence

from nest_graph.config import DfsMode, SelectionConfig
from nest_graph.graph import (
    DfsDispatcherConfig,
    DfsMode as GraphDfsMode,
    FinalizeSelectionOptions,
    PoseGraph,
    RefineSelectionOptions,
    apply_dfs_refinement as _apply_dfs_refinement_native,
    finalize_selection,
    prune_selection_to_independent_set as _prune_selection_native,
)
from nest_graph.propose.block_replace import _packing_independent, lex_count_area_better
from nest_graph.propose.void_selection import _sel_area
from nest_graph.propose.context import outline_coverage_ratio
from nest_graph.propose.void_selection import count_selected_in_free


def _shrink_independent_locks(idxs: Sequence[int], graph) -> list[int]:
    """Greedy growing subset; empty if fewer than 2 members remain independent."""
    out: list[int] = []
    for raw in idxs:
        i = int(raw)
        if i < 0:
            continue
        trial = out + [i]
        if _packing_independent(trial, graph):
            out.append(i)
    return out if len(out) >= 2 else []


def apply_dg_refine_options(
    opts: RefineSelectionOptions,
    dg,
    propose_cfg,
) -> RefineSelectionOptions:
    """Q166/Q167/Q178: optional DG-aware C++ refine semantics."""
    if propose_cfg is None or not bool(getattr(propose_cfg, "dg_aware_refine", True)):
        return opts
    opts.dg_aware_refine = True
    opts.motif_fracture_penalty = float(
        getattr(propose_cfg, "motif_refine_fracture_penalty", 1.0) or 0.0
    )
    pairs: list[tuple[int, int]] = []
    if dg is not None:
        for m in getattr(dg, "motifs", ()) or ():
            a = int(getattr(m, "a", -1))
            b = int(getattr(m, "b", -1))
            if a >= 0 and b >= 0:
                pairs.append((a, b))
    opts.motif_join_pairs = pairs
    return opts


@dataclass(frozen=True, slots=True)
class PolishBudget:
    """Iteration budgets for one polish pass (mid vs last)."""

    dfs_passes: int
    dfs_mode: DfsMode
    dfs_max_tries: int | None
    run_3b: bool
    run_post_pack: bool
    freeze_improve_rules: bool

    @property
    def mcts_heavy(self) -> int:
        """Telem alias: last-leaf post stages on."""
        return int(bool(self.run_post_pack))


def polish_budget_mid(sel: SelectionConfig | None = None) -> PolishBudget:
    """Cheap expand / mid-iter: one finalize_end DFS pass; no 3b/se2."""
    tries = None
    if sel is not None:
        tries = min(int(sel.dfs_max_tries), 2)
    return PolishBudget(
        dfs_passes=1,
        dfs_mode=DfsMode.MERGED_LOOSE_TIGHT_FINALIZE_END,
        dfs_max_tries=tries,
        run_3b=False,
        run_post_pack=False,
        freeze_improve_rules=True,
    )


def polish_budget_last(sel: SelectionConfig | None = None) -> PolishBudget:
    """Last iter / Uh / always_heavy: shipped DFS + post stages allowed."""
    passes = 3
    tries = None
    mode = DfsMode.MERGED_LOOSE_TIGHT
    if sel is not None:
        passes = int(sel.dfs_passes)
        tries = int(sel.dfs_max_tries)
        mode = DfsMode(sel.dfs_mode)
    return PolishBudget(
        dfs_passes=passes,
        dfs_mode=mode,
        dfs_max_tries=tries,
        run_3b=True,
        run_post_pack=True,
        freeze_improve_rules=False,
    )


def polish_budget_for_iter(
    *,
    is_last_leaf: bool,
    sel: SelectionConfig | None = None,
    large_void: bool = False,
    cheap_expand: bool = False,
    near_last: bool = False,
    on_plateau: bool = False,
    free_remaining: bool = False,
) -> PolishBudget:
    """Hybrid schedule: last → full; cheap → mid no-3b; mid+large_void → mid+3b near last.

    Q190: plateau + residual free → force 3b (dual lex first elsewhere; 3b when stuck).
    """
    if is_last_leaf and not cheap_expand:
        return polish_budget_last(sel)
    mid = polish_budget_mid(sel)
    if cheap_expand or not large_void:
        if on_plateau and free_remaining and large_void:
            return PolishBudget(
                dfs_passes=mid.dfs_passes,
                dfs_mode=mid.dfs_mode,
                dfs_max_tries=mid.dfs_max_tries,
                run_3b=True,
                run_post_pack=False,
                freeze_improve_rules=mid.freeze_improve_rules,
            )
        return mid
    tries = mid.dfs_max_tries
    if sel is not None:
        tries = min(int(sel.dfs_max_tries), max(int(tries or 2), 3))
    run_3b = bool(near_last) or bool(on_plateau and free_remaining)
    return PolishBudget(
        dfs_passes=mid.dfs_passes,
        dfs_mode=mid.dfs_mode,
        dfs_max_tries=tries,
        run_3b=run_3b,
        run_post_pack=False,
        freeze_improve_rules=mid.freeze_improve_rules,
    )


def freeze_improve_rules(sel_iter, *, freeze: bool):
    """One site for improve_rules_rounds=0 (MCTS expand and/or plateau sterile)."""
    if not freeze:
        return sel_iter
    return sel_iter.model_copy(update={"improve_rules_rounds": 0})


def should_freeze_improve_rules(
    *,
    freeze_cheap_expand: bool,
    on_plateau: bool,
    plateau_streak: int,
    flat_iters: int,
    enable_incumbent_loop: bool,
) -> tuple[bool, str]:
    """OR of Q72 cheap-expand freeze and sustained-plateau freeze."""
    if freeze_cheap_expand:
        return True, "mcts_expand"
    if (
        on_plateau
        and enable_incumbent_loop
        and plateau_streak >= flat_iters + 2
    ):
        return True, "plateau"
    return False, ""


def run_improve_rules_rounds(
    improve_rules_fn,
    *,
    graphs,
    rule_sets: list,
    board,
    sel_iter,
    rng,
    score_options,
    mutation_presets: list | None = None,
    rule_score_penalty: float = 0.03,
    max_rules_per_set: int = 24,
    seed_offset: int = 0,
) -> list:
    """Single improve_rules loop (first-pass and mid-pack share this)."""
    for round_idx in range(int(sel_iter.improve_rules_rounds)):
        rule_sets = improve_rules_fn(
            graphs,
            rule_sets,
            sel_iter.rules_kept,
            board,
            mutation_presets=mutation_presets,
            rule_score_penalty=rule_score_penalty,
            elite_count=sel_iter.improve_rules_elite_count,
            seed=int(rng.integers(0, 2**31)) + round_idx + int(seed_offset),
            score_options=score_options,
            max_rules_per_set=max_rules_per_set,
        )
    return rule_sets


def apply_refine_with_restore(
    *,
    budget: PolishBudget,
    apply_dfs_fn,
    graph,
    refine_rules,
    selected_nest: Sequence[int],
    refine_scores,
    sel_iter,
    node_areas: Sequence[float],
    refine_seed: int,
    locked_indices: Sequence[int],
    polys: Sequence,
    group_id: Sequence[int],
    transform: Sequence,
    part_areas: Sequence[float],
    part_bases: dict,
    sheet,
    min_dist: float,
    rim_before: float,
    rim_reject: float,
    propose_stats: dict,
    native_geoms_from_transforms_fn,
    free_info: Any | None = None,
    free_poly: Any | None = None,
    dg=None,
    propose_cfg=None,
) -> list[int]:
    """
    DFS refine (always when budget.dfs_passes > 0) then **one** restore if
    rim-drop OR not lex-better OR void shed without lex win (U1).
    """
    nest_before_refine = list(selected_nest)
    propose_stats["dfs_passes"] = int(budget.dfs_passes)
    propose_stats["dfs_mode"] = str(
        budget.dfs_mode.value
        if hasattr(budget.dfs_mode, "value")
        else budget.dfs_mode
    )
    propose_stats["mcts_heavy"] = int(budget.mcts_heavy)
    propose_stats["run_post_pack"] = int(bool(budget.run_post_pack))
    propose_stats["run_3b"] = int(bool(budget.run_3b))

    if int(budget.dfs_passes) <= 0:
        propose_stats["refine_rejected"] = False
        propose_stats["rim_drop"] = 0.0
        propose_stats["refine_ms"] = 0.0
        propose_stats["void_refine_hold"] = 0
        propose_stats["pre_refine_lock_reject"] = 0
        return list(selected_nest)

    locked_work = [int(i) for i in locked_indices if int(i) >= 0]
    propose_stats["pre_refine_lock_reject"] = 0
    if (
        locked_work
        and graph is not None
        and not _packing_independent(locked_work, graph)
    ):
        locked_work = _shrink_independent_locks(locked_work, graph)
        propose_stats["pre_refine_lock_reject"] = 1
    locked_indices = locked_work

    dfs_kwargs: dict = {
        "selection": sel_iter,
        "node_areas": node_areas,
        "refine_seed": int(refine_seed),
        "locked_indices": list(locked_indices),
        "dfs_passes": int(budget.dfs_passes),
        "mode": budget.dfs_mode,
        "dg": dg,
        "propose_cfg": propose_cfg,
    }
    if budget.dfs_max_tries is not None:
        dfs_kwargs["dfs_max_tries"] = int(budget.dfs_max_tries)

    t0 = time.perf_counter()
    _, selected_polys, _ = apply_dfs_fn(
        graph,
        refine_rules,
        list(selected_nest),
        refine_scores,
        **dfs_kwargs,
    )
    propose_stats["refine_ms"] = (time.perf_counter() - t0) * 1000.0

    restore_refine = False
    void_refine_hold = 0
    rim_drop = 0.0
    if rim_reject > 0.0 and rim_before > 0.0:
        try:
            placed_after = [polys[i] for i in selected_polys]
            rim_after = float(outline_coverage_ratio(
                placed_after,
                sheet,
                min_dist,
                pack_geoms=native_geoms_from_transforms_fn(
                    [group_id[i] for i in selected_polys],
                    [transform[i] for i in selected_polys],
                    part_bases,
                ) if selected_polys else None,
            ))
        except Exception:
            rim_after = rim_before
        rim_drop = float(max(0.0, rim_before - rim_after))
        if rim_before - rim_after > rim_reject:
            restore_refine = True
            # Q192: rim drop alone must not undo a count-winning refine.
            if len(selected_polys) > len(nest_before_refine):
                restore_refine = False

    refine_lex_better = lex_count_area_better(
        old_count=len(nest_before_refine),
        old_area=_sel_area(nest_before_refine, group_id, part_areas),
        new_count=len(selected_polys),
        new_area=_sel_area(selected_polys, group_id, part_areas),
    )
    # Q192: do not restore solely for rim loss when count rises, or on count-tie
    # when void-fill rises.
    void_fill_rise = False
    count_rise = len(selected_polys) > len(nest_before_refine)
    count_tie = len(selected_polys) == len(nest_before_refine)
    if (
        free_info is not None
        and getattr(free_info, "kind", None) == "large_void"
        and free_poly is not None
        and not getattr(free_poly, "is_empty", True)
    ):
        nv_nest0 = count_selected_in_free(polys, nest_before_refine, free_poly)
        nv_ref0 = count_selected_in_free(polys, selected_polys, free_poly)
        void_fill_rise = int(nv_ref0) > int(nv_nest0)
    if not refine_lex_better:
        if count_rise or (count_tie and void_fill_rise):
            restore_refine = False
        else:
            restore_refine = True

    score_before = selection_score_sum(list(refine_scores), list(nest_before_refine))
    score_after = selection_score_sum(list(refine_scores), list(selected_polys))
    sel_n = max(len(nest_before_refine), 1)
    norm_score_delta = (score_after - score_before) / float(sel_n)
    score_eps = 1e-4
    if (
        restore_refine
        and free_info is not None
        and getattr(free_info, "kind", None) == "large_void"
        and norm_score_delta > score_eps
    ):
        restore_refine = False
        propose_stats["refine_score_accept"] = int(
            propose_stats.get("refine_score_accept", 0)
        ) + 1
    elif (
        restore_refine
        and count_tie
        and norm_score_delta > score_eps
    ):
        restore_refine = False
        propose_stats["refine_score_accept"] = int(
            propose_stats.get("refine_score_accept", 0)
        ) + 1

    # U1/R0: void shed without lex win → restore; also hold if refine empties void.
    # Hollow: count-up refine that sheds void *and* area must restore (lex count trap).
    # Score accept (Q245–246) wins over U1 when norm_score_delta already cleared restore.
    if (
        free_info is not None
        and getattr(free_info, "kind", None) == "large_void"
        and free_poly is not None
        and not getattr(free_poly, "is_empty", True)
        and norm_score_delta <= score_eps
    ):
        nv_nest = count_selected_in_free(polys, nest_before_refine, free_poly)
        nv_ref = count_selected_in_free(polys, selected_polys, free_poly)
        refine_count_better = len(selected_polys) > len(nest_before_refine)
        if (
            nv_nest > 0
            and nv_ref < nv_nest
            and (nv_nest - nv_ref) >= 1
            and not refine_count_better
            and not (count_tie and void_fill_rise)
        ):
            restore_refine = True
            void_refine_hold = 1
        elif (
            nv_nest > 0
            and int(nv_ref) + 2 <= int(nv_nest)
            and float(_sel_area(selected_polys, group_id, part_areas)) + 1e-12
            < float(_sel_area(nest_before_refine, group_id, part_areas))
        ):
            restore_refine = True
            void_refine_hold = 1
            propose_stats["void_refine_area_hold"] = 1
        elif (
            int(nv_nest) >= 8
            and (int(nv_nest) - int(nv_ref)) >= 4
            and float(_sel_area(selected_polys, group_id, part_areas)) + 1e-12
            < 1.02 * float(_sel_area(nest_before_refine, group_id, part_areas))
        ):
            # Shed ≥4 void without ≥2% area gain → keep nest.
            restore_refine = True
            void_refine_hold = 1
            propose_stats["void_refine_half_hold"] = 1

    if restore_refine:
        selected_polys = list(nest_before_refine)
        propose_stats["refine_rejected"] = True
    else:
        propose_stats["refine_rejected"] = False

    locked_set = {int(i) for i in locked_indices if int(i) >= 0}
    refine_lock_hold = 0
    refine_lock_escape = 0
    lock_n_compose = int(propose_stats.get("lock_n_compose", len(locked_set)) or len(locked_set))
    if locked_set:
        final_set = set(selected_polys)
        dropped = [i for i in locked_set if i not in final_set]
        if dropped and not restore_refine:
            area_locked = float(_sel_area(nest_before_refine, group_id, part_areas))
            area_refined = float(_sel_area(selected_polys, group_id, part_areas))
            indep_ok = _packing_independent(selected_polys, graph)
            # MotifJoin / compose locks: need ≥5% area + void rise to escape.
            motif_hold = (
                int(lock_n_compose) >= 2
                or int(propose_stats.get("compose_motif_hold", 0) or 0) > 0
            )
            escape_floor = 1.05 if motif_hold else 1.02
            escape_ok = (
                area_refined + 1e-12 >= escape_floor * area_locked
                and indep_ok
                and (refine_lex_better or void_fill_rise)
            )
            if motif_hold and escape_ok and not void_fill_rise:
                escape_ok = False
            if escape_ok:
                refine_lock_escape = 1
            else:
                selected_polys = list(nest_before_refine)
                restore_refine = True
                refine_lock_hold = 1
                propose_stats["refine_rejected"] = True
        elif dropped and restore_refine:
            refine_lock_hold = 1

    lock_n_refine = len([i for i in locked_set if i in set(selected_polys)])
    lock_survive = (
        float(lock_n_refine) / float(max(lock_n_compose, 1))
        if lock_n_compose > 0
        else 1.0
    )
    propose_stats["lock_n_refine"] = int(lock_n_refine)
    propose_stats["lock_n_final"] = int(lock_n_refine)
    propose_stats["refine_lock_hold"] = int(refine_lock_hold)
    propose_stats["refine_lock_escape"] = int(refine_lock_escape)
    propose_stats["lock_survive_refine"] = float(lock_survive)
    propose_stats["rim_drop"] = float(rim_drop)
    propose_stats["void_refine_hold"] = int(void_refine_hold)
    return list(selected_polys)


def prune_selection_to_independent_set(
    graph: PoseGraph,
    selected: list[int],
    scores: list[float] | None = None,
) -> list[int]:
    """Greedy MIS fallback (prefer finalize_selection for score-optimal drops)."""
    return list(_prune_selection_native(graph, selected, scores))


def _dfs_dispatcher_config(sel: SelectionConfig, *, propose_cfg=None) -> DfsDispatcherConfig:
    cfg = DfsDispatcherConfig()
    cfg.dfs_max_tries = int(sel.dfs_max_tries)
    cfg.dfs_refine_max_passes = int(sel.dfs_refine_max_passes)
    cfg.dfs_refine_max_stagnant_passes = int(sel.dfs_refine_max_stagnant_passes)
    cfg.dfs_refine_beam_width = int(sel.dfs_refine_beam_width)
    cfg.refine_explore_shuffle = bool(getattr(sel, "refine_explore_shuffle", False))
    cfg.dfs_growth_restarts = max(1, int(getattr(sel, "dfs_growth_restarts", 1) or 1))
    cfg.refine_lexicographic_area = bool(getattr(sel, "refine_lexicographic_area", True))
    cfg.dfs_finalize_repair_passes = int(sel.dfs_finalize_repair_passes)
    cfg.dfs_finalize_max_component = int(sel.dfs_finalize_max_component)
    cfg.dg_aware_refine = bool(
        getattr(propose_cfg, "dg_aware_refine", True) if propose_cfg is not None else True
    )
    cfg.motif_refine_fracture_penalty = float(
        getattr(propose_cfg, "motif_refine_fracture_penalty", 1.0) or 0.0
        if propose_cfg is not None
        else 1.0
    )
    return cfg


_CONFIG_TO_GRAPH_DFS: dict[str, GraphDfsMode] = {
    "nest_only": GraphDfsMode.NestOnly,
    "head_pipeline": GraphDfsMode.HeadPipeline,
    "strict_no_prune": GraphDfsMode.StrictNoPrune,
    "strict_prune": GraphDfsMode.StrictPrune,
    "legacy_alternating": GraphDfsMode.LegacyAlternating,
    "merged_loose_tight": GraphDfsMode.MergedLooseTight,
    "merged_loose_finalize_end": GraphDfsMode.MergedLooseFinalizeEnd,
    "merged_loose_tight_finalize_end": GraphDfsMode.MergedLooseTightFinalizeEnd,
    "merged_single_pass": GraphDfsMode.MergedSinglePass,
    "high_pass_loose": GraphDfsMode.HighPassLoose,
}


def _native_dfs_mode(mode: DfsMode | str) -> GraphDfsMode:
    key = str(mode.value) if isinstance(mode, DfsMode) else str(mode)
    return _CONFIG_TO_GRAPH_DFS.get(key, GraphDfsMode.MergedLooseTight)


def refine_options(
    sel: SelectionConfig,
    *,
    loose: bool,
    max_tries: int | None = None,
    node_areas: Sequence[float] | None = None,
    seed: int | None = None,
    dg=None,
    propose_cfg=None,
) -> RefineSelectionOptions:
    opts = RefineSelectionOptions()
    opts.max_tries = sel.dfs_max_tries if max_tries is None else max_tries
    opts.max_passes = sel.dfs_refine_max_passes
    opts.max_stagnant_passes = sel.dfs_refine_max_stagnant_passes
    opts.beam_width = sel.dfs_refine_beam_width
    opts.explore_shuffle = bool(getattr(sel, "refine_explore_shuffle", False))
    opts.growth_restarts = max(1, int(getattr(sel, "dfs_growth_restarts", 1) or 1))
    if seed is not None:
        opts.seed = int(seed) & 0xFFFFFFFF
    opts.lexicographic_area = bool(getattr(sel, "refine_lexicographic_area", True))
    if node_areas is not None and opts.lexicographic_area:
        opts.node_areas = [float(a) for a in node_areas]
    if loose:
        opts.min_collisions = 2
        opts.max_root_collisions = 2
    else:
        opts.min_collisions = 1
        opts.max_root_collisions = 1
    return apply_dg_refine_options(opts, dg, propose_cfg)


def finalize_options(
    sel: SelectionConfig,
    locked_indices: Sequence[int] | None = None,
) -> FinalizeSelectionOptions:
    opts = FinalizeSelectionOptions()
    opts.repair_passes = sel.dfs_finalize_repair_passes
    opts.max_exact_component_size = sel.dfs_finalize_max_component
    if locked_indices:
        opts.locked_indices = [int(i) for i in locked_indices]
    return opts


def loose_refine_options(
    sel: SelectionConfig,
    *,
    node_areas: Sequence[float] | None = None,
    seed: int | None = None,
    dg=None,
    propose_cfg=None,
) -> RefineSelectionOptions:
    return refine_options(
        sel, loose=True, node_areas=node_areas, seed=seed,
        dg=dg, propose_cfg=propose_cfg,
    )


def tight_refine_options(
    sel: SelectionConfig,
    *,
    node_areas: Sequence[float] | None = None,
    seed: int | None = None,
    dg=None,
    propose_cfg=None,
) -> RefineSelectionOptions:
    return refine_options(
        sel, loose=False, node_areas=node_areas, seed=seed,
        dg=dg, propose_cfg=propose_cfg,
    )


def strict_refine_options(
    sel: SelectionConfig,
    *,
    node_areas: Sequence[float] | None = None,
    seed: int | None = None,
    dg=None,
    propose_cfg=None,
) -> RefineSelectionOptions:
    opts = refine_options(
        sel, loose=False, node_areas=node_areas, seed=seed,
        dg=dg, propose_cfg=propose_cfg,
    )
    opts.min_collisions = 0
    opts.max_root_collisions = 0
    return opts


def head_loose_refine_options(
    sel: SelectionConfig,
    *,
    node_areas: Sequence[float] | None = None,
    seed: int | None = None,
    dg=None,
    propose_cfg=None,
) -> RefineSelectionOptions:
    """HEAD-style score DFS: allow transient overlaps during search."""
    return refine_options(
        sel, loose=True, node_areas=node_areas, seed=seed,
        dg=dg, propose_cfg=propose_cfg,
    )


def selection_score_sum(scores: list[float], selected: list[int]) -> float:
    return float(sum(scores[v] for v in selected))


def dfs_finalize_selection(graph, selected, scores, finalize_opts) -> list[int]:
    return list(finalize_selection(graph, selected, scores, finalize_opts))


def dfs_refine_seed(seed0: int | None, pass_i: int) -> int | None:
    if seed0 is None:
        return None
    return int(seed0) + int(pass_i) * 17


def apply_dfs_refinement(
    graph: PoseGraph,
    rule_set,
    selected: list[int],
    scores: list[float],
    *,
    dfs_passes: int | None = None,
    dfs_max_tries: int | None = None,
    mode: DfsMode | str | None = None,
    selection: SelectionConfig | None = None,
    node_areas: Sequence[float] | None = None,
    refine_seed: int | None = None,
    locked_indices: Sequence[int] | None = None,
    dg=None,
    propose_cfg=None,
) -> tuple[list[int], list[int], float]:
    """Refine selection; return (pre_finalize, final, score_sum_final).

    ``locked_indices`` are finalize-only (``insert_clear_locks``). DFS options
    must not receive them — force-on refine pins dropped count (Q47).
    """
    sel = selection if selection is not None else SelectionConfig()
    passes = dfs_passes if dfs_passes is not None else sel.dfs_passes
    max_tries = dfs_max_tries if dfs_max_tries is not None else sel.dfs_max_tries
    mode_val = mode if mode is not None else sel.dfs_mode
    locks = [int(i) for i in (locked_indices or [])]
    finalize_opts = finalize_options(sel, locked_indices=locks)
    areas = list(node_areas) if node_areas is not None else None
    seed = int(refine_seed) if refine_seed is not None else -1
    cfg = _dfs_dispatcher_config(sel, propose_cfg=propose_cfg)
    pre, final, score_sum = _apply_dfs_refinement_native(
        graph,
        rule_set,
        list(selected),
        list(scores),
        int(passes),
        int(max_tries),
        _native_dfs_mode(mode_val),
        cfg,
        finalize_opts,
        areas,
        seed,
        dg,
    )
    return list(pre), list(final), float(score_sum)
