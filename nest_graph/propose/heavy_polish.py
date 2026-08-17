"""One polish path: DFS refine + restore, budgeted by dfs_passes/mode."""

import time
from dataclasses import dataclass
from typing import Any, Sequence

from nest_graph.config import DfsMode, SelectionConfig
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
