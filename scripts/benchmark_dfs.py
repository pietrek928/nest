#!/usr/bin/env python3
"""Benchmark DFS refinement modes and SelectionConfig tuning (isolated DFS timing)."""

import argparse
import time
from dataclasses import dataclass
from typing import Any

import numpy as np

from nest_graph.config import BuildGraphConfig, SelectionConfig
from nest_graph.build_graph import apply_dfs_refinement
from nest_graph.elem_graph import selection_is_independent
from scripts.nesting_evaluator import NestingPipelineEvaluator
from scripts.nesting_fixtures import get_all_cases


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
    parser.add_argument("--cases", nargs="*", default=["demo_triangle_s1.0"])
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
                for max_tries in (2, 4):
                    sel = SelectionConfig(
                        dfs_mode=mode,
                        dfs_passes=passes,
                        dfs_max_tries=max_tries,
                    )
                    variants.append((
                        _selection_label(mode, passes, max_tries,
                                         sel.dfs_refine_max_passes, sel.dfs_finalize_repair_passes),
                        sel,
                    ))

    all_cases = get_all_cases()
    cases_to_run = [c for c in all_cases if c.name in args.cases]

    for case in cases_to_run:
        print(f"\n=== Case: {case.name} ===")
        rows: list[DfsBenchRow] = []
        
        for seed in args.seeds:
            # Prepare graph once per seed
            t0 = time.perf_counter()
            cfg = BuildGraphConfig()
            evaluator = NestingPipelineEvaluator(case, cfg)
            graph, rule_set, selected, scores, polys, group_id, transform = evaluator.prepare_first_iteration(seed)
            prep_time = time.perf_counter() - t0
            
            for label, sel_cfg in variants:
                t1 = time.perf_counter()
                _raw, final, _score = apply_dfs_refinement(
                    graph, rule_set, selected, scores, selection=sel_cfg,
                )
                dfs_time = time.perf_counter() - t1
                
                if not selection_is_independent(graph, final):
                    raise AssertionError(f"{label} seed={seed}: overlapping final selection")
                    
                rows.append(DfsBenchRow(
                    label=label,
                    seed=seed,
                    nest_sel=len(selected),
                    dfs_sel_final=len(final),
                    delta_nest=len(final) - len(selected),
                    dfs_time_s=dfs_time,
                    total_time_s=prep_time + dfs_time,
                ))
                
        print(_aggregate(rows))


if __name__ == "__main__":
    main()
