#!/usr/bin/env python3
"""End-to-end nesting quality benchmark using parameterized fixtures."""

import argparse
import sys
import time
from pathlib import Path

import numpy as np

from nest_graph.config import BuildGraphConfig, ProposeConfig
from scripts.nesting_evaluator import NestingPipelineEvaluator
from scripts.nesting_fixtures import get_all_cases


def _build_cfg(propose_name: str, dfs_mode: str) -> BuildGraphConfig:
    cfg = BuildGraphConfig()
    cfg.selection.dfs_mode = dfs_mode
    
    if propose_name == "shipped":
        pass
    elif propose_name == "local_compact":
        cfg.propose = ProposeConfig.local_compact_profile()
    elif propose_name == "no_propose":
        cfg.propose = ProposeConfig(
            use_neighbor_slide=False,
            use_axis_push=False,
            use_bottom_left=False,
            use_nfp_vertices=False,
            use_voronoi=False,
            use_point_cloud=False,
            use_guidance_walk=False,
            use_ribbon_seeds=False,
            use_group_edge_seeds=False,
            use_border_edge_seeds=False,
            use_guidance_propositions=False,
        )
    return cfg


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cases", nargs="*", help="Case names to run")
    parser.add_argument("--tags", nargs="*", help="Tags to run (e.g. dense, void)")
    parser.add_argument("--all", action="store_true", help="Run all cases (overrides default tight_pack)")
    parser.add_argument("--propose", nargs="*", default=["shipped"], choices=["shipped", "local_compact", "no_propose"])
    parser.add_argument("--dfs-modes", nargs="*", default=["merged_loose_tight"])
    parser.add_argument("--seeds", type=int, nargs="*", default=[0])
    parser.add_argument("--gate", action="store_true", help="Fail if baselines are not met")
    parser.add_argument("--update-baselines", action="store_true", help="Update baselines.json")
    args = parser.parse_args()

    all_cases = get_all_cases()
    cases_to_run = []
    
    if not args.cases and not args.tags and not args.all:
        args.tags = ["tight_pack"]
        
    for c in all_cases:
        if args.cases and c.name not in args.cases:
            continue
        if args.tags and not any(t in c.tags for t in args.tags):
            continue
        cases_to_run.append(c)

    if not cases_to_run:
        print("No cases matched.")
        sys.exit(1)

    results = []
    
    for case in cases_to_run:
        for propose in args.propose:
            for dfs_mode in args.dfs_modes:
                cfg = _build_cfg(propose, dfs_mode)
                evaluator = NestingPipelineEvaluator(case, cfg)
                
                case_metrics = []
                for seed in args.seeds:
                    print(f"Running {case.name} | {propose} | {dfs_mode} | seed={seed}...", flush=True)
                    metrics = evaluator.run_full_pipeline(seed)
                    case_metrics.append(metrics)
                    print(f"  -> parts={metrics.parts_final} area={metrics.area_coverage:.3f} ok={metrics.independent_ok} time={metrics.time_s:.2f}s")
                    
                # Aggregate
                avg_parts = np.mean([m.parts_final for m in case_metrics])
                avg_area = np.mean([m.area_coverage for m in case_metrics])
                avg_parts_delta = np.mean([m.parts_delta for m in case_metrics])
                avg_area_delta = np.mean([m.area_coverage_delta for m in case_metrics])
                avg_kiss = np.mean([m.kiss_fraction for m in case_metrics])
                avg_time = np.mean([m.time_s for m in case_metrics])
                all_indep = all(m.independent_ok for m in case_metrics)
                
                results.append({
                    "case": case.name,
                    "propose": propose,
                    "dfs_mode": dfs_mode,
                    "parts": avg_parts,
                    "area": avg_area,
                    "parts_delta": avg_parts_delta,
                    "area_delta": avg_area_delta,
                    "kiss": avg_kiss,
                    "time": avg_time,
                    "independent_ok": all_indep,
                })
                
    # Print table
    print("\n| Case | Propose | DFS | Parts | Area | ΔParts | ΔArea | Kiss | Time | Indep OK |")
    print("|------|---------|-----|-------|------|--------|-------|------|------|----------|")
    for r in results:
        print(f"| {r['case']} | {r['propose']} | {r['dfs_mode']} | {r['parts']:.1f} | {r['area']:.3f} | {r['parts_delta']:.1f} | {r['area_delta']:.3f} | {r['kiss']:.2f} | {r['time']:.2f}s | {r['independent_ok']} |")
        
    if args.gate:
        # We will implement gating later
        pass

if __name__ == "__main__":
    main()
