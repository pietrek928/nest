"""One polish path: DFS refine + restore, budgeted by dfs_passes/mode."""

import time
from dataclasses import dataclass
from typing import Any, Sequence

from nest_graph.config import DfsMode, SelectionConfig
from nest_graph.elem_graph import (
    FinalizeSelectionOptions,
    PoseGraph,
    RefineSelectionOptions,
    finalize_selection,
    increase_score_dfs,
    increase_selection_dfs,
    refine_selection,
    sort_graph,
)
from nest_graph.propose.block_replace import _sel_area, lex_count_area_better
from nest_graph.propose.context import outline_coverage_ratio
from nest_graph.propose.void_selection import count_selected_in_free


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
) -> PolishBudget:
    """Hybrid schedule: last → full; cheap → mid no-3b; mid+large_void → mid+3b near last.

    Keeps Q69 cheap expand speed; mid large_void 3b only on near-last (time/scrap).
    """
    if is_last_leaf and not cheap_expand:
        return polish_budget_last(sel)
    mid = polish_budget_mid(sel)
    if cheap_expand or not large_void:
        return mid
    tries = mid.dfs_max_tries
    if sel is not None:
        tries = min(int(sel.dfs_max_tries), max(int(tries or 2), 3))
    return PolishBudget(
        dfs_passes=mid.dfs_passes,
        dfs_mode=mid.dfs_mode,
        dfs_max_tries=tries,
        # G1 hybrid: mid void 3b only near last (every mid-iter 3b blew time).
        run_3b=bool(near_last),
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
        return list(selected_nest)

    dfs_kwargs: dict = {
        "selection": sel_iter,
        "node_areas": node_areas,
        "refine_seed": int(refine_seed),
        "locked_indices": list(locked_indices),
        "dfs_passes": int(budget.dfs_passes),
        "mode": budget.dfs_mode,
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

    refine_lex_better = lex_count_area_better(
        old_count=len(nest_before_refine),
        old_area=_sel_area(nest_before_refine, group_id, part_areas),
        new_count=len(selected_polys),
        new_area=_sel_area(selected_polys, group_id, part_areas),
    )
    if not refine_lex_better:
        restore_refine = True

    # U1/R0: void shed without lex win → restore; also hold if refine empties void.
    if (
        free_info is not None
        and getattr(free_info, "kind", None) == "large_void"
        and free_poly is not None
        and not getattr(free_poly, "is_empty", True)
    ):
        nv_nest = count_selected_in_free(polys, nest_before_refine, free_poly)
        nv_ref = count_selected_in_free(polys, selected_polys, free_poly)
        if nv_nest > 0 and nv_ref < nv_nest:
            if (not refine_lex_better) or nv_ref <= 0 or nv_ref < max(1, int(0.5 * nv_nest)):
                restore_refine = True
                void_refine_hold = 1

    if restore_refine:
        selected_polys = list(nest_before_refine)
        propose_stats["refine_rejected"] = True
    else:
        propose_stats["refine_rejected"] = False
    propose_stats["rim_drop"] = float(rim_drop)
    propose_stats["void_refine_hold"] = int(void_refine_hold)
    return list(selected_polys)


def prune_selection_to_independent_set(
    graph: PoseGraph,
    selected: list[int],
    scores: list[float] | None = None,
) -> list[int]:
    """Greedy MIS fallback (prefer finalize_selection for score-optimal drops)."""
    if not selected:
        return []
    order = list(selected)
    if scores is not None and len(scores) == len(graph.group_id):
        order.sort(key=lambda v: scores[v], reverse=True)
    kept: list[int] = []
    kept_set: set[int] = set()
    for v in order:
        if any(u in kept_set for u in graph.collisions[v]):
            continue
        kept.append(v)
        kept_set.add(v)
    return kept


def refine_options(
    sel: SelectionConfig,
    *,
    loose: bool,
    max_tries: int | None = None,
    node_areas: Sequence[float] | None = None,
    seed: int | None = None,
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
    return opts


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
) -> RefineSelectionOptions:
    return refine_options(
        sel, loose=True, node_areas=node_areas, seed=seed,
    )


def tight_refine_options(
    sel: SelectionConfig,
    *,
    node_areas: Sequence[float] | None = None,
    seed: int | None = None,
) -> RefineSelectionOptions:
    return refine_options(
        sel, loose=False, node_areas=node_areas, seed=seed,
    )


def strict_refine_options(
    sel: SelectionConfig,
    *,
    node_areas: Sequence[float] | None = None,
    seed: int | None = None,
) -> RefineSelectionOptions:
    opts = refine_options(
        sel, loose=False, node_areas=node_areas, seed=seed,
    )
    opts.min_collisions = 0
    opts.max_root_collisions = 0
    return opts


def head_loose_refine_options(
    sel: SelectionConfig,
    *,
    node_areas: Sequence[float] | None = None,
    seed: int | None = None,
) -> RefineSelectionOptions:
    """HEAD-style score DFS: allow transient overlaps during search."""
    return refine_options(
        sel, loose=True, node_areas=node_areas, seed=seed,
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
) -> tuple[list[int], list[int], float]:
    """Refine selection; return (pre_finalize, final, score_sum_final).

    ``locked_indices`` are finalize-only (``insert_clear_locks``). DFS options
    must not receive them — force-on refine pins dropped count (Q47).
    """
    sel = selection if selection is not None else SelectionConfig()
    passes = dfs_passes if dfs_passes is not None else sel.dfs_passes
    max_tries = dfs_max_tries if dfs_max_tries is not None else sel.dfs_max_tries
    mode = DfsMode(mode if mode is not None else sel.dfs_mode)
    locks = [int(i) for i in (locked_indices or [])]
    finalize_opts = finalize_options(sel, locked_indices=locks)
    areas = list(node_areas) if node_areas is not None else None
    seed0 = refine_seed

    selected = list(selected)
    graph_sorted = sort_graph(graph, rule_set)
    graph_sorted_rev = sort_graph(graph, rule_set, reverse=True)
    pre_finalize = selected

    if mode == DfsMode.NEST_ONLY:
        return selected, selected, selection_score_sum(scores, selected)

    if mode == DfsMode.LEGACY_ALTERNATING:
        for _ in range(passes):
            selected = list(increase_selection_dfs(
                graph_sorted_rev, selected, max_tries,
            ))
            selected = list(increase_selection_dfs(graph, selected, max_tries))
            selected = list(increase_score_dfs(graph_sorted_rev, selected, scores))
            selected = list(increase_selection_dfs(
                graph_sorted, selected, max_tries,
            ))
            selected = list(increase_score_dfs(graph_sorted, selected, scores))
        pre_finalize = selected
        final = dfs_finalize_selection(graph, selected, scores, finalize_opts)
        return pre_finalize, final, selection_score_sum(scores, final)

    if mode == DfsMode.HEAD_PIPELINE:
        loose = head_loose_refine_options(sel)
        tight = RefineSelectionOptions()
        tight.min_collisions = 1
        tight.max_root_collisions = 2
        tight.max_passes = sel.dfs_refine_max_passes
        tight.max_stagnant_passes = sel.dfs_refine_max_stagnant_passes
        tight.beam_width = sel.dfs_refine_beam_width
        for _ in range(passes):
            selected = list(increase_selection_dfs(
                graph_sorted_rev, selected, max_tries,
            ))
            selected = list(increase_selection_dfs(graph, selected, max_tries))
            selected = list(increase_score_dfs(
                graph_sorted_rev, selected, scores, loose,
            ))
            selected = list(increase_selection_dfs(
                graph_sorted, selected, max_tries,
            ))
            selected = list(increase_score_dfs(graph_sorted, selected, scores, tight))
        pre_finalize = selected
        return pre_finalize, pre_finalize, selection_score_sum(scores, pre_finalize)

    if mode == DfsMode.STRICT_NO_PRUNE:
        for pass_i in range(passes):
            strict = strict_refine_options(
                sel, node_areas=areas, seed=dfs_refine_seed(seed0, pass_i),
            )
            selected = list(refine_selection(graph_sorted_rev, selected, scores, strict))
            selected = list(refine_selection(graph, selected, scores, strict))
        pre_finalize = selected
        return pre_finalize, pre_finalize, selection_score_sum(scores, pre_finalize)

    if mode == DfsMode.STRICT_PRUNE:
        for pass_i in range(passes):
            strict = strict_refine_options(
                sel, node_areas=areas, seed=dfs_refine_seed(seed0, pass_i),
            )
            selected = list(refine_selection(graph_sorted_rev, selected, scores, strict))
            selected = list(refine_selection(graph, selected, scores, strict))
        pre_finalize = selected
        final = prune_selection_to_independent_set(graph, selected, scores)
        return pre_finalize, final, selection_score_sum(scores, final)

    if mode == DfsMode.MERGED_SINGLE_PASS:
        final = selected
        for pass_i in range(passes):
            loose = loose_refine_options(
                sel, node_areas=areas, seed=dfs_refine_seed(seed0, pass_i),
            )
            selected = list(refine_selection(graph_sorted_rev, selected, scores, loose))
            pre_finalize = selected
            final = dfs_finalize_selection(graph, selected, scores, finalize_opts)
        return pre_finalize, final, selection_score_sum(scores, final)

    if mode == DfsMode.MERGED_LOOSE_FINALIZE_END:
        for pass_i in range(passes):
            loose = loose_refine_options(
                sel, node_areas=areas, seed=dfs_refine_seed(seed0, pass_i),
            )
            selected = list(refine_selection(graph_sorted_rev, selected, scores, loose))
        pre_finalize = selected
        final = dfs_finalize_selection(graph, selected, scores, finalize_opts)
        return pre_finalize, final, selection_score_sum(scores, final)

    if mode in (DfsMode.MERGED_LOOSE_TIGHT_FINALIZE_END, DfsMode.HIGH_PASS_LOOSE):
        for pass_i in range(passes):
            loose = loose_refine_options(
                sel, node_areas=areas, seed=dfs_refine_seed(seed0, pass_i),
            )
            tight = tight_refine_options(
                sel, node_areas=areas, seed=dfs_refine_seed(seed0, pass_i + 1),
            )
            selected = list(refine_selection(graph_sorted_rev, selected, scores, loose))
            selected = list(refine_selection(graph, selected, scores, tight))
        pre_finalize = selected
        final = dfs_finalize_selection(graph, selected, scores, finalize_opts)
        return pre_finalize, final, selection_score_sum(scores, final)

    # merged_loose_tight: finalize after each outer pass
    for pass_i in range(passes):
        loose = loose_refine_options(
            sel, node_areas=areas, seed=dfs_refine_seed(seed0, pass_i),
        )
        tight = tight_refine_options(
            sel, node_areas=areas, seed=dfs_refine_seed(seed0, pass_i + 1),
        )
        selected = list(refine_selection(graph_sorted_rev, selected, scores, loose))
        selected = list(refine_selection(graph, selected, scores, tight))
    pre_finalize = selected
    final = dfs_finalize_selection(graph, selected, scores, finalize_opts)
    grown = list(increase_selection_dfs(graph_sorted_rev, final, max_tries))
    grown = list(increase_selection_dfs(graph, grown, max_tries))
    if len(grown) > len(final):
        final = dfs_finalize_selection(graph, grown, scores, finalize_opts)
    return pre_finalize, final, selection_score_sum(scores, final)
