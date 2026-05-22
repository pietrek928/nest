#!/usr/bin/env python3
"""Benchmark first-iteration nesting quality for parameter tuning."""

from __future__ import annotations

import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from nest_graph.build_graph import (
    NestState,
    _build_transform_batch,
    _make_demo_rule_set,
    _make_seed_rule_sets,
    improve_rules,
    make_polygon_graph,
)
from nest_graph.config import BuildGraphConfig, ProposeConfig, SamplingConfig, SelectionConfig
from nest_graph.elem_graph import (
    increase_score_dfs,
    increase_selection_dfs,
    nest_by_graph,
    score_elems,
    sort_graph,
)


@dataclass
class FirstPassMetrics:
    name: str
    graph_nodes: int
    collision_edges: int
    selected_nest: int
    selected_dfs: int
    batch_g0: int
    batch_g1: int
    time_s: float


def _collision_edge_count(graph, n: int) -> int:
    return sum(len(graph._obj.collisions[i]) for i in range(n)) // 2


def run_first_pass(cfg: BuildGraphConfig, seed: int = 0) -> FirstPassMetrics:
    rng = np.random.default_rng(seed)
    sc = cfg.sampling
    sel = cfg.selection
    gc = cfg.graph

    p_board = cfg.rules.board_polygon()
    p1 = cfg.rules.rect_polygon()
    p2 = cfg.rules.tri_polygon()
    parts = [(p1, 0), (p2, 1)]

    selected_t = (
        rng.uniform(-1, 1, (sc.initial_random, 3)) * sc.transform_scale,
        rng.uniform(-1, 1, (sc.initial_random, 3)) * sc.transform_scale,
    )
    history = (np.zeros((1, 3)), np.zeros((1, 3)))
    rule_sets = _make_seed_rule_sets(cfg)
    rule_set = _make_demo_rule_set(cfg)

    t0 = time.perf_counter()
    selected_t = _build_transform_batch(
        cfg, selected_t, history, rng,
        board=p_board, parts=parts, nest_state=None,
    )
    graph, polys, group_id, transform = make_polygon_graph(
        p_board,
        [(p1, selected_t[0]), (p2, selected_t[1])],
        board_check=gc.board_check,
    )
    graphs = [graph]
    for _ in range(sel.improve_rules_rounds):
        rule_sets = improve_rules(
            graphs, rule_sets, sel.rules_kept, p_board,
            mutation_presets=cfg.rules.mutation_presets(),
            rule_score_penalty=sel.rule_score_penalty,
        )
    selected = list(nest_by_graph(graph, rule_sets[: sel.nest_rule_sets_used])[0])
    graph_sorted = sort_graph(graph, rule_set)
    graph_sorted_rev = sort_graph(graph, rule_set, reverse=True)
    scores = score_elems(graph, rule_set)
    for _ in range(sel.dfs_passes):
        selected = list(increase_selection_dfs(
            graph_sorted_rev, selected,
            sel.dfs_max_tries, sel.dfs_min_collisions_loose,
        ))
        selected = list(increase_selection_dfs(
            graph, selected,
            sel.dfs_max_tries, sel.dfs_min_collisions_tight,
        ))
        selected = list(increase_score_dfs(graph_sorted_rev, selected, scores))
        selected = list(increase_selection_dfs(
            graph_sorted, selected,
            sel.dfs_max_tries, sel.dfs_min_collisions_loose,
        ))
        selected = list(increase_score_dfs(graph_sorted, selected, scores))
    elapsed = time.perf_counter() - t0

    n = len(polys)
    return FirstPassMetrics(
        name="",
        graph_nodes=n,
        collision_edges=_collision_edge_count(graph, n),
        selected_nest=len(nest_by_graph(graph, rule_sets[:1])[0]),
        selected_dfs=len(selected),
        batch_g0=selected_t[0].shape[0],
        batch_g1=selected_t[1].shape[0],
        time_s=elapsed,
    )


def _cfg(
    name: str,
    *,
    sampling: dict[str, Any] | None = None,
    propose: dict[str, Any] | None = None,
    selection: dict[str, Any] | None = None,
) -> tuple[str, BuildGraphConfig]:
    s = SamplingConfig(**(sampling or {}))
    p = ProposeConfig(**(propose or {}))
    sel = SelectionConfig(**(selection or {}))
    return name, BuildGraphConfig(sampling=s, propose=p, selection=sel)


PRESETS: list[tuple[str, BuildGraphConfig]] = [
    _cfg("A_current_defaults"),
    _cfg("B_lean_propose", propose={
        "use_voronoi": False, "use_point_cloud": False,
        "max_proposals": 10, "candidate_pool": 10,
    }),
    _cfg("C_propose_heavy", propose={
        "max_proposals": 18, "candidate_pool": 16,
        "use_point_cloud": True, "point_cloud_particles": 10,
        "point_cloud_iterations": 12,
    }),
    _cfg("D_high_random", sampling={
        "random_per_iter": 192, "initial_random": 192,
        "max_transforms_per_group": 900,
    }),
    _cfg("E_shuffle_heavy", sampling={
        "shuffle_passes": 3, "shuffle_per_pass": 48,
        "shuffle_scale": (0.15, 0.15, 0.6),
    }),
    _cfg("F_fast_rules", selection={
        "improve_rules_rounds": 2, "dfs_passes": 2,
    }),
    _cfg("G_quality_rules", selection={
        "improve_rules_rounds": 6, "dfs_passes": 2,
    }),
    _cfg("H_first_pass_combo", sampling={
        "random_per_iter": 160, "initial_random": 160,
        "max_transforms_per_group": 750,
        "shuffle_passes": 2, "shuffle_per_pass": 40,
    }, propose={
        "max_proposals": 16, "candidate_pool": 14,
        "use_voronoi": True, "use_point_cloud": False,
        "erosion_num_angles": 8, "raycast_num_rays": 10,
    }, selection={"improve_rules_rounds": 4}),
    _cfg("I_propose_shuffle", sampling={
        "random_per_iter": 128, "shuffle_passes": 3, "shuffle_per_pass": 40,
    }, propose={
        "max_proposals": 14, "candidate_pool": 12,
        "use_point_cloud": False,
    }),
    _cfg("J_max_density", sampling={
        "random_per_iter": 256, "initial_random": 256,
        "max_transforms_per_group": 1000,
        "shuffle_passes": 2, "shuffle_per_pass": 32,
    }, propose={
        "max_proposals": 20, "candidate_pool": 16,
        "use_voronoi": True, "use_point_cloud": False,
    }, selection={"improve_rules_rounds": 4}),
    _cfg("K_recommended", sampling={
        "random_per_iter": 192, "initial_random": 256,
        "max_transforms_per_group": 900,
        "shuffle_passes": 2, "shuffle_per_pass": 36,
        "shuffle_scale": (0.14, 0.14, 0.55),
    }, propose={
        "max_proposals": 18, "candidate_pool": 14,
        "use_voronoi": True, "use_point_cloud": False,
        "erosion_num_angles": 8, "raycast_num_rays": 10,
    }, selection={"improve_rules_rounds": 4}),
]


def _format_table(rows: list[FirstPassMetrics]) -> str:
    lines = [
        "| preset | graph_nodes | edges | nest | dfs_sel | batch0 | batch1 | time_s |",
        "|--------|-------------|-------|------|---------|--------|--------|--------|",
    ]
    for r in rows:
        lines.append(
            f"| {r.name} | {r.graph_nodes} | {r.collision_edges} | "
            f"{r.selected_nest} | {r.selected_dfs} | {r.batch_g0} | {r.batch_g1} | {r.time_s:.2f} |"
        )
    return "\n".join(lines)


def main() -> None:
    seeds = (0, 1, 2)
    rows: list[FirstPassMetrics] = []
    for name, cfg in PRESETS:
        agg = []
        for seed in seeds:
            m = run_first_pass(cfg, seed=seed)
            m.name = name
            agg.append(m)
        rows.append(FirstPassMetrics(
            name=name,
            graph_nodes=int(np.mean([a.graph_nodes for a in agg])),
            collision_edges=int(np.mean([a.collision_edges for a in agg])),
            selected_nest=int(np.mean([a.selected_nest for a in agg])),
            selected_dfs=int(np.mean([a.selected_dfs for a in agg])),
            batch_g0=int(np.mean([a.batch_g0 for a in agg])),
            batch_g1=int(np.mean([a.batch_g1 for a in agg])),
            time_s=float(np.mean([a.time_s for a in agg])),
        ))

    rows.sort(key=lambda r: (r.selected_dfs, r.graph_nodes), reverse=True)
    table = _format_table(rows)
    print(table)
    best = rows[0]
    print(f"\nBest by dfs_sel: {best.name} (dfs={best.selected_dfs}, time={best.time_s:.2f}s)")

    out_path = Path(__file__).resolve().parents[1] / "docs" / "first_pass_tuning_results.txt"
    out_path.write_text(
        f"# Auto-generated {time.strftime('%Y-%m-%d %H:%M')}\n\n{table}\n\nBest: {best.name}\n",
        encoding="utf-8",
    )
    print(f"\nWrote {out_path}")


if __name__ == "__main__":
    main()
