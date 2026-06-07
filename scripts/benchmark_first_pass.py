#!/usr/bin/env python3
"""Benchmark first-iteration nesting quality and DFS refinement modes."""

import argparse
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from nest_graph.build_graph import (
    NestState,
    _build_transform_batch,
    _make_initial_rule_sets,
    active_rule_set,
    apply_dfs_refinement,
    improve_rules,
    make_polygon_graph,
)
from nest_graph.config import (
    BuildGraphConfig,
    ProposeConfig,
    SamplingConfig,
    SelectionConfig,
    score_rules_options,
)
from nest_graph.elem_graph import nest_by_graph, selection_is_independent


DFS_MODES = (
    "nest_only",
    "head_pipeline",
    "strict_no_prune",
    "strict_prune",
    "legacy_alternating",
    "merged_loose_tight",
    "merged_single_pass",
    "high_pass_loose",
)

PROPOSE_PRESETS: dict[str, dict[str, Any]] = {
    "baseline": {
        "use_free_region_search": False,
        "ranking_mode": "legacy",
        "smart_push_target": False,
        "use_contact_ranking": False,
    },
    "clearance": {
        "use_free_region_search": True,
        "ranking_mode": "clearance",
        "smart_push_target": True,
        "trim_candidates_by_clearance": True,
        "use_ribbon_seeds": True,
        "use_group_edge_seeds": False,
        "use_border_focus": False,
        "use_contact_ranking": False,
    },
    "shipped": {
        "use_free_region_search": True,
        "smart_push_target": True,
        "trim_candidates_by_clearance": True,
        "use_ribbon_seeds": True,
        "use_group_edge_seeds": True,
        "use_border_focus": True,
        "use_border_edge_seeds": True,
        "use_board_edge_seeds": True,
        "board_edge_guidance_refine": True,
        "border_focus_ranking": True,
        "use_contact_ranking": True,
        "candidate_pool": 32,
        "group_edge_samples_per_edge": 24,
        "sheet_edge_samples_per_edge": 24,
    },
    "free_clearance_pso": {
        "use_free_region_search": True,
        "ranking_mode": "clearance",
        "smart_push_target": True,
        "use_point_cloud": True,
        "use_contact_ranking": False,
    },
    "shipped_board_edge": {
        "use_free_region_search": True,
        "smart_push_target": True,
        "trim_candidates_by_clearance": True,
        "use_ribbon_seeds": True,
        "use_group_edge_seeds": True,
        "use_border_focus": True,
        "use_border_edge_seeds": True,
        "use_board_edge_seeds": True,
        "board_edge_guidance_refine": True,
        "border_focus_ranking": True,
        "use_contact_ranking": True,
    },
    "board_edge_heavy": {
        "use_free_region_search": True,
        "smart_push_target": True,
        "trim_candidates_by_clearance": True,
        "use_border_focus": True,
        "use_border_edge_seeds": True,
        "use_board_edge_seeds": True,
        "board_edge_guidance_refine": True,
        "board_edge_samples_per_edge": 48,
        "board_edge_guidance_seeds": 24,
        "guidance_max_propositions": 12,
        "border_focus_ranking": True,
        "use_ribbon_seeds": False,
        "use_group_edge_seeds": False,
        "use_voronoi": False,
        "use_guidance_propositions": False,
    },
}


@dataclass
class FirstPassMetrics:
    mode: str
    seed: int
    graph_nodes: int
    collision_edges: int
    nest_sel: int
    dfs_sel_raw: int
    dfs_sel_final: int
    score_sum_final: float
    prune_dropped: int
    time_s: float
    independent: bool


def _collision_edge_count(graph, n: int) -> int:
    return sum(len(graph.collisions[i]) for i in range(n)) // 2


def run_first_pass(
    cfg: BuildGraphConfig,
    *,
    seed: int = 0,
    mode: str = "merged_loose_tight",
    dfs_passes: int | None = None,
) -> FirstPassMetrics:
    if mode not in DFS_MODES:
        raise ValueError(f"unknown mode {mode!r}; choose from {DFS_MODES}")

    rng = np.random.default_rng(seed)
    sc = cfg.sampling
    sel = cfg.selection
    gc = cfg.graph
    passes = dfs_passes if dfs_passes is not None else sel.dfs_passes
    if mode == "high_pass_loose":
        passes = max(passes, 16)

    p_board = cfg.rules.board_polygon()
    p1 = cfg.rules.rect_polygon()
    p2 = cfg.rules.tri_polygon()
    parts = [(p1, 0), (p2, 1)]

    selected_t = (
        rng.uniform(-1, 1, (sc.initial_random, 3)) * sc.transform_scale,
        rng.uniform(-1, 1, (sc.initial_random, 3)) * sc.transform_scale,
    )
    history = (np.zeros((1, 3)), np.zeros((1, 3)))
    rule_sets = _make_initial_rule_sets(cfg)

    t0 = time.perf_counter()
    selected_t = _build_transform_batch(
        cfg, selected_t, history, rng,
        board=p_board, parts=parts, nest_state=None,
    )
    graph, polys, _group_id, _transform = make_polygon_graph(
        p_board,
        [(p1, selected_t[0]), (p2, selected_t[1])],
        min_dist=cfg.board_min_dist(),
        epsilon_ratio=cfg.propose.placement_clearance_epsilon_ratio,
    )
    graphs = [graph]
    for round_idx in range(sel.improve_rules_rounds):
        rule_sets = improve_rules(
            graphs, rule_sets, sel.rules_kept, p_board,
            mutation_presets=cfg.rules.mutation_presets(),
            rule_score_penalty=sel.rule_score_penalty,
            elite_count=sel.improve_rules_elite_count,
            seed=seed + round_idx,
            score_options=score_rules_options(sel),
            max_rules_per_set=cfg.rules.max_rules_per_set,
        )
    from nest_graph.elem_graph import score_elems

    active_rules = active_rule_set(rule_sets)
    scores = score_elems(graph, active_rules)
    selected = list(nest_by_graph(graph, rule_sets[: sel.nest_rule_sets_used])[0])
    nest_sel = len(selected)

    if mode == "nest_only":
        raw, final, score_sum = selected, selected, sum(scores[v] for v in selected)
    else:
        raw, final, score_sum = apply_dfs_refinement(
            graph,
            active_rules,
            selected,
            scores,
            dfs_passes=passes,
            dfs_max_tries=sel.dfs_max_tries,
            mode=mode,
        )

    elapsed = time.perf_counter() - t0
    n = len(polys)
    indep = selection_is_independent(graph, final)
    if not indep:
        raise AssertionError(f"mode={mode} seed={seed}: final selection overlaps")

    return FirstPassMetrics(
        mode=mode,
        seed=seed,
        graph_nodes=n,
        collision_edges=_collision_edge_count(graph, n),
        nest_sel=nest_sel,
        dfs_sel_raw=len(raw),
        dfs_sel_final=len(final),
        score_sum_final=score_sum,
        prune_dropped=len(raw) - len(final),
        time_s=elapsed,
        independent=indep,
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


def _j_max_density(propose_overrides: dict[str, Any] | None = None) -> tuple[str, BuildGraphConfig]:
    propose = dict(PROPOSE_PRESETS["shipped"])
    propose.setdefault("max_proposals", 20)
    propose.setdefault("use_voronoi", True)
    propose.setdefault("use_point_cloud", False)
    if propose_overrides:
        propose.update(propose_overrides)
    return _cfg(
        "J_max_density",
        sampling={
            "random_per_iter": 128,
            "random_per_iter_when_proposed": 48,
            "structured_jitter_per_proposal": 8,
            "structured_jitter_scale": (0.06, 0.06, 0.35),
            "initial_random": 256,
            "max_transforms_per_group": 900,
            "shuffle_passes": 4,
            "shuffle_per_pass": 48,
        },
        propose=propose,
        selection={"improve_rules_rounds": 4, "dfs_mode": "merged_loose_tight"},
    )


J_MAX_DENSITY = _j_max_density()


def _format_mode_table(rows: list[FirstPassMetrics]) -> str:
    by_mode: dict[str, list[FirstPassMetrics]] = {}
    for r in rows:
        by_mode.setdefault(r.mode, []).append(r)

    lines = [
        "| mode | nest_sel | dfs_raw | dfs_final | Δnest | dropped | score_sum | time_s |",
        "|------|----------|---------|-----------|-------|---------|-----------|--------|",
    ]
    for mode in DFS_MODES:
        if mode not in by_mode:
            continue
        agg = by_mode[mode]
        nest = float(np.mean([a.nest_sel for a in agg]))
        raw = float(np.mean([a.dfs_sel_raw for a in agg]))
        final = float(np.mean([a.dfs_sel_final for a in agg]))
        dropped = float(np.mean([a.prune_dropped for a in agg]))
        score = float(np.mean([a.score_sum_final for a in agg]))
        t = float(np.mean([a.time_s for a in agg]))
        lines.append(
            f"| {mode} | {nest:.1f} | {raw:.1f} | {final:.1f} | "
            f"{final - nest:+.1f} | {dropped:.1f} | {score:.2f} | {t:.2f} |"
        )
    return "\n".join(lines)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--modes",
        nargs="*",
        default=list(DFS_MODES),
        choices=DFS_MODES,
        help="DFS pipeline modes to benchmark",
    )
    parser.add_argument("--seeds", type=int, nargs="*", default=list(range(10)))
    parser.add_argument("--dfs-passes", type=int, default=None)
    parser.add_argument(
        "--git-spot-check",
        action="store_true",
        help="Also run high_pass_loose with dfs_passes=16 on seeds 0..2",
    )
    parser.add_argument(
        "--propose-preset",
        choices=sorted(PROPOSE_PRESETS),
        default=None,
        help="Override ProposeConfig for gap-fitting A/B (baseline, free_clearance, …)",
    )
    args = parser.parse_args()

    preset_label = args.propose_preset or "default"
    if args.propose_preset:
        _name, cfg = _j_max_density(PROPOSE_PRESETS[args.propose_preset])
        _name = f"{_name}+{args.propose_preset}"
    else:
        _name, cfg = J_MAX_DENSITY
    rows: list[FirstPassMetrics] = []
    for mode in args.modes:
        for seed in args.seeds:
            m = run_first_pass(
                cfg,
                seed=seed,
                mode=mode,
                dfs_passes=args.dfs_passes,
            )
            rows.append(m)
            print(
                f"seed={seed} mode={mode}: nest={m.nest_sel} "
                f"raw={m.dfs_sel_raw} final={m.dfs_sel_final} "
                f"dropped={m.prune_dropped} score={m.score_sum_final:.2f} "
                f"t={m.time_s:.2f}s"
            )

    if args.git_spot_check:
        for seed in (0, 1, 2):
            m = run_first_pass(
                cfg, seed=seed, mode="high_pass_loose", dfs_passes=16,
            )
            m.mode = "high_pass_loose_p16"
            rows.append(m)

    table = _format_mode_table(rows)
    print("\n" + table)

    delta_rows = [
        f"seed={r.seed} mode={r.mode}: dfs_final - nest_sel = {r.dfs_sel_final - r.nest_sel:+d}"
        for r in rows
    ]
    if delta_rows:
        print("\n# dfs_sel_final - nest_sel (propose → graph quality)")
        print("\n".join(delta_rows))

    out_path = Path(__file__).resolve().parents[1] / "docs" / "first_pass_tuning_results.txt"
    out_path.write_text(
        f"# Auto-generated {time.strftime('%Y-%m-%d %H:%M')}\n"
        f"# preset={_name} propose_preset={preset_label} seeds={list(args.seeds)}\n\n"
        f"{table}\n\n"
        f"## dfs_sel_final - nest_sel\n\n"
        + "\n".join(delta_rows)
        + "\n",
        encoding="utf-8",
    )
    print(f"\nWrote {out_path}")


if __name__ == "__main__":
    main()
