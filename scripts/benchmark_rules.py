#!/usr/bin/env python3
"""Benchmark rule evolution: Tier-A score_rules vs Tier-B nest+DFS on finalists."""

import argparse
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from nest_graph.build_graph import (
    _build_transform_batch,
    _make_initial_rule_sets,
    active_rule_set,
    apply_dfs_refinement,
    improve_rules,
    make_polygon_graph,
    score_rule_sets_with_dfs,
)
from nest_graph.config import (
    BuildGraphConfig,
    ProposeConfig,
    SamplingConfig,
    SelectionConfig,
    score_rules_options,
)
from nest_graph.elem_graph import nest_by_graph, score_elems, score_rules


@dataclass
class RulesBenchRow:
    label: str
    seed: int
    tier_a_best: float
    tier_b_best: float
    dfs_parts: int
    rule_size: int
    improve_time_s: float
    total_time_s: float


def _prep_graph(cfg: BuildGraphConfig, seed: int):
    rng = np.random.default_rng(seed)
    sc = cfg.sampling
    sel = cfg.selection
    p_board = cfg.rules.board_polygon()
    p1 = cfg.rules.rect_polygon()
    p2 = cfg.rules.tri_polygon()
    parts = [(p1, 0), (p2, 1)]
    selected_t = (
        rng.uniform(-1, 1, (sc.initial_random, 3)) * sc.transform_scale,
        rng.uniform(-1, 1, (sc.initial_random, 3)) * sc.transform_scale,
    )
    history = (np.zeros((1, 3)), np.zeros((1, 3)))
    selected_t = _build_transform_batch(
        cfg, selected_t, history, rng,
        board=p_board, parts=parts, nest_state=None,
    )
    graph, _polys, _gid, _tr = make_polygon_graph(
        p_board,
        [(p1, selected_t[0]), (p2, selected_t[1])],
        min_dist=cfg.board_min_dist(),
        epsilon_ratio=cfg.propose.placement_clearance_epsilon_ratio,
    )
    return graph, p_board, sel, rng


def run_variant(
    cfg: BuildGraphConfig,
    *,
    seed: int,
    label: str,
    use_tier_b: bool,
    tier_b_top_k: int,
) -> RulesBenchRow:
    t0 = time.perf_counter()
    graph, p_board, sel, rng = _prep_graph(cfg, seed)
    rule_sets = _make_initial_rule_sets(cfg)
    graphs = [graph]

    t_improve = time.perf_counter()
    for round_idx in range(sel.improve_rules_rounds):
        rule_sets = improve_rules(
            graphs,
            rule_sets,
            sel.rules_kept,
            p_board,
            mutation_presets=cfg.rules.mutation_presets(),
            rule_score_penalty=sel.rule_score_penalty,
            elite_count=sel.improve_rules_elite_count,
            seed=int(rng.integers(0, 2**31)) + round_idx,
            score_options=score_rules_options(sel),
            max_rules_per_set=cfg.rules.max_rules_per_set,
        )
    improve_time = time.perf_counter() - t_improve

    opts = score_rules_options(sel)
    tier_a = score_rules([graph], rule_sets, opts)
    best_a = max(tier_a) if tier_a else 0.0
    best_idx = int(np.argmax(tier_a)) if tier_a else 0

    if use_tier_b:
        tier_b = score_rule_sets_with_dfs(
            graph, rule_sets, sel, top_k=tier_b_top_k,
        )
        best_b = max(tier_b) if tier_b else 0.0
        best_idx = int(np.argmax(tier_b)) if tier_b else best_idx
    else:
        best_b = best_a

    active = rule_sets[best_idx] if rule_sets else active_rule_set(rule_sets)
    scores = score_elems(graph, active)
    selected = list(nest_by_graph(graph, [active])[0])
    _, final, _ = apply_dfs_refinement(
        graph, active, selected, scores, selection=sel,
    )
    total = time.perf_counter() - t0
    return RulesBenchRow(
        label=label,
        seed=seed,
        tier_a_best=best_a,
        tier_b_best=best_b,
        dfs_parts=len(final),
        rule_size=active.size() if active else 0,
        improve_time_s=improve_time,
        total_time_s=total,
    )


def main() -> None:
    parser = argparse.ArgumentParser(description="Benchmark rule evolution fitness tiers")
    parser.add_argument("--seeds", type=int, nargs="+", default=[0, 1, 2])
    parser.add_argument("--tier-b", action="store_true", help="Score top-K with nest+DFS")
    parser.add_argument("--tier-b-top-k", type=int, default=4)
    parser.add_argument(
        "--out",
        type=Path,
        default=Path("docs/rules_benchmark_results.txt"),
    )
    args = parser.parse_args()

    cfg = BuildGraphConfig(
        sampling=SamplingConfig(
            random_per_iter=64,
            initial_random=64,
            max_transforms_per_group=300,
            seed=0,
        ),
        propose=ProposeConfig(max_proposals=12, candidate_pool=24),
        selection=SelectionConfig(
            improve_rules_rounds=2,
            rules_kept=32,
            improve_rules_elite_count=8,
        ),
    )

    rows: list[RulesBenchRow] = []
    for seed in args.seeds:
        rows.append(
            run_variant(
                cfg,
                seed=seed,
                label="tier_b" if args.tier_b else "tier_a",
                use_tier_b=args.tier_b,
                tier_b_top_k=args.tier_b_top_k,
            )
        )

    lines = [
        "# rules benchmark",
        f"tier_b={args.tier_b} top_k={args.tier_b_top_k}",
        "label seed tier_a_best tier_b_best dfs_parts rule_size improve_s total_s",
    ]
    for r in rows:
        lines.append(
            f"{r.label} {r.seed} {r.tier_a_best:.4f} {r.tier_b_best:.4f} "
            f"{r.dfs_parts} {r.rule_size} {r.improve_time_s:.3f} {r.total_time_s:.3f}"
        )
    text = "\n".join(lines) + "\n"
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(text)
    print(text)


if __name__ == "__main__":
    main()
