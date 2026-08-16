"""Best-leaf polish helpers: one refine-restore gate (rim OR lex), freeze rules."""


from typing import Sequence

from nest_graph.propose.block_replace import _sel_area, lex_count_area_better
from nest_graph.propose.context import outline_coverage_ratio


def freeze_improve_rules(sel_iter, *, freeze: bool):
    """One site for improve_rules_rounds=0 (MCTS expand and/or plateau sterile)."""
    if not freeze:
        return sel_iter
    return sel_iter.model_copy(update={"improve_rules_rounds": 0})


def should_freeze_improve_rules(
    *,
    do_heavy_polish: bool,
    on_plateau: bool,
    plateau_streak: int,
    flat_iters: int,
    enable_incumbent_loop: bool,
) -> tuple[bool, str]:
    """OR of Q72 cheap-expand freeze and sustained-plateau freeze. Returns (freeze, reason)."""
    if not do_heavy_polish:
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
    do_heavy_polish: bool,
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
) -> list[int]:
    """
    DFS refine (heavy only) then **one** restore if rim-drop OR not lex-better.

    Both predicates feed a single rewind to ``selected_nest`` (nest_before_refine).
    """
    nest_before_refine = list(selected_nest)
    if not do_heavy_polish:
        # Cheap expand: no DFS — skip restore predicates (selected == nest).
        propose_stats["refine_rejected"] = False
        propose_stats["rim_drop"] = 0.0
        return list(selected_nest)

    _, selected_polys, _ = apply_dfs_fn(
        graph,
        refine_rules,
        list(selected_nest),
        refine_scores,
        selection=sel_iter,
        node_areas=node_areas,
        refine_seed=int(refine_seed),
        locked_indices=list(locked_indices),
    )

    restore_refine = False
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

    if not lex_count_area_better(
        old_count=len(nest_before_refine),
        old_area=_sel_area(nest_before_refine, group_id, part_areas),
        new_count=len(selected_polys),
        new_area=_sel_area(selected_polys, group_id, part_areas),
    ):
        restore_refine = True

    if restore_refine:
        selected_polys = list(nest_before_refine)
        propose_stats["refine_rejected"] = True
    else:
        propose_stats["refine_rejected"] = False
    propose_stats["rim_drop"] = float(rim_drop)
    return list(selected_polys)
