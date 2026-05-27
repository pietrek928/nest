#!/usr/bin/env python3
"""Benchmark DFS refinement modes and SelectionConfig tuning (isolated DFS timing)."""

import argparse
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from nest_graph.build_graph import (
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
from nest_graph.elem_graph import nest_by_graph, score_elems, selection_is_independent


DFS_MODES = (
    "nest_only",
    "merged_loose_finalize_end",
    "merged_loose_tight_finalize_end",
    "merged_single_pass",
    "merged_loose_tight",
)


@dataclass
class DfsBenchRow:
    label: str
    seed: int
    nest_sel: int
    dfs_sel_final: int
    delta_nest: int
    dfs_time_s: float
    total_time_s: float


def _shipped_cfg(selection: SelectionConfig) -> BuildGraphConfig:
    return BuildGraphConfig(
        sampling=SamplingConfig(
            random_per_iter=128,
            random_per_iter_when_proposed=48,
            structured_jitter_per_proposal=8,
            initial_random=256,
            max_transforms_per_group=900,
        ),
        propose=ProposeConfig(),
        selection=selection,
    )


def _prepare_graph(cfg: BuildGraphConfig, seed: int):
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
    rule_sets = _make_initial_rule_sets(cfg)

    t0 = time.perf_counter()
    selected_t = _build_transform_batch(
        cfg, selected_t, history, rng,
        board=p_board, parts=parts, nest_state=None,
    )
    graph, polys, _gid, _tr = make_polygon_graph(
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
    active_rules = active_rule_set(rule_sets)
    scores = score_elems(graph, active_rules)
    selected = list(nest_by_graph(graph, rule_sets[: sel.nest_rule_sets_used])[0])
    prep_time = time.perf_counter() - t0
    return graph, active_rules, selected, scores, len(selected), prep_time


def run_dfs_variant(
    cfg: BuildGraphConfig,
    *,
    seed: int,
    label: str,
) -> DfsBenchRow:
    graph, rule_set, selected, scores, nest_sel, prep_time = _prepare_graph(cfg, seed)
    t0 = time.perf_counter()
    _raw, final, _score = apply_dfs_refinement(
        graph, rule_set, selected, scores, selection=cfg.selection,
    )
    dfs_time = time.perf_counter() - t0
    if not selection_is_independent(graph, final):
        raise AssertionError(f"{label} seed={seed}: overlapping final selection")
    return DfsBenchRow(
        label=label,
        seed=seed,
        nest_sel=nest_sel,
        dfs_sel_final=len(final),
        delta_nest=len(final) - nest_sel,
        dfs_time_s=dfs_time,
        total_time_s=prep_time + dfs_time,
    )


def _selection_label(
    mode: str,
    passes: int,
    max_tries: int,
    max_passes: int,
    repair: int,
) -> str:
    return f"{mode}/p{passes}/t{max_tries}/mp{max_passes}/r{repair}"


def _aggregate(rows: list[DfsBenchRow]) -> str:
    by: dict[str, list[DfsBenchRow]] = {}
    for r in rows:
        by.setdefault(r.label, []).append(r)

    lines = [
        "| config | nest | dfs_final | Δnest | dfs_s | total_s |",
        "|--------|------|-----------|-------|-------|---------|",
    ]
    ranked: list[tuple[float, float, str]] = []
    for label, group in sorted(by.items()):
        nest = float(np.mean([g.nest_sel for g in group]))
        final = float(np.mean([g.dfs_sel_final for g in group]))
        delta = float(np.mean([g.delta_nest for g in group]))
        dfs_t = float(np.mean([g.dfs_time_s for g in group]))
        tot = float(np.mean([g.total_time_s for g in group]))
        lines.append(
            f"| `{label}` | {nest:.1f} | {final:.1f} | {delta:+.1f} | {dfs_t:.2f} | {tot:.2f} |",
        )
        ranked.append((final, dfs_t, label))
    lines.append("")
    lines.append("## By parts (dfs_final)")
    for final, dfs_t, label in sorted(ranked, reverse=True):
        lines.append(f"- **{final:.1f}** parts, dfs {dfs_t:.2f}s — `{label}`")
    lines.append("")
    lines.append("## By DFS speed (same nest_sel baseline)")
    for final, dfs_t, label in sorted(ranked, key=lambda x: x[1]):
        lines.append(f"- **{dfs_t:.2f}s** → {final:.1f} parts — `{label}`")
    return "\n".join(lines)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seeds", type=int, nargs="*", default=[0, 1, 2])
    parser.add_argument(
        "--quick",
        action="store_true",
        help="Smaller matrix (modes + pass count only)",
    )
    args = parser.parse_args()

    variants: list[tuple[str, SelectionConfig]] = []

    if args.quick:
        for mode in DFS_MODES:
            for passes in (2, 3):
                sel = SelectionConfig(dfs_mode=mode, dfs_passes=passes)
                variants.append((
                    _selection_label(mode, passes, sel.dfs_max_tries,
                                     sel.dfs_refine_max_passes, sel.dfs_finalize_repair_passes),
                    sel,
                ))
    else:
        for mode in DFS_MODES:
            for passes in (2, 3, 4):
                for max_tries in (2, 4, 8):
                    for max_passes in (8, 12, 16):
                        for repair in (4, 6, 8):
                            sel = SelectionConfig(
                                dfs_mode=mode,
                                dfs_passes=passes,
                                dfs_max_tries=max_tries,
                                dfs_refine_max_passes=max_passes,
                                dfs_finalize_repair_passes=repair,
                            )
                            variants.append((
                                _selection_label(
                                    mode, passes, max_tries, max_passes, repair,
                                ),
                                sel,
                            ))

    rows: list[DfsBenchRow] = []
    for label, sel in variants:
        cfg = _shipped_cfg(sel)
        for seed in args.seeds:
            row = run_dfs_variant(cfg, seed=seed, label=label)
            rows.append(row)
            print(
                f"{label} seed={seed}: nest={row.nest_sel} final={row.dfs_sel_final} "
                f"Δ={row.delta_nest:+d} dfs={row.dfs_time_s:.2f}s",
            )

    table = _aggregate(rows)
    print("\n" + table)

    out = Path(__file__).resolve().parents[1] / "docs" / "dfs_benchmark_results.txt"
    out.write_text(
        f"# DFS benchmark {time.strftime('%Y-%m-%d %H:%M')}\n"
        f"# seeds={list(args.seeds)} quick={args.quick}\n\n{table}\n",
        encoding="utf-8",
    )
    print(f"\nWrote {out}")


if __name__ == "__main__":
    main()
