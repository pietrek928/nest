#!/usr/bin/env python3
"""End-to-end nesting quality benchmark using parameterized fixtures."""

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

from nest_graph.config import BuildGraphConfig, ProposeConfig
from scripts.nesting_evaluator import NestingPipelineEvaluator, metrics_meet_floors
from scripts.nesting_fixtures import get_all_cases, resolve_cases


def _build_cfg(propose_name: str, dfs_mode: str) -> BuildGraphConfig:
    cfg = BuildGraphConfig()
    cfg.selection.dfs_mode = dfs_mode

    if propose_name == "shipped":
        pass
    elif propose_name == "local_compact":
        cfg.propose = ProposeConfig.local_compact_profile()
    elif propose_name == "no_void_boost":
        cfg.propose = cfg.propose.model_copy(update={
            "void_island_score_boost": 0.0,
            "void_attractor_rule_weight": 0.0,
            "enable_gravity_compaction": False,
        })
    elif propose_name == "void_boost_high":
        cfg.propose = cfg.propose.model_copy(update={
            "void_island_score_boost": 128.0,
            "pocket_score_boost": 100.0,
            "void_attractor_rule_weight": 48.0,
        })
    elif propose_name == "no_void_yield_densify":
        cfg.propose = cfg.propose.model_copy(update={
            "enable_void_yield_densify_accept": False,
        })
    elif propose_name == "no_greedy_nest":
        cfg.propose = cfg.propose.model_copy(update={
            "void_greedy_nest_seed": False,
        })
    elif propose_name == "no_pocket_fit":
        cfg.propose = cfg.propose.model_copy(update={
            "use_pocket_fit": False,
        })
    elif propose_name == "no_small_prefer":
        cfg.propose = cfg.propose.model_copy(update={
            "small_part_void_score_boost": 0.0,
        })
    elif propose_name == "no_pocket_mis_boost":
        cfg.propose = cfg.propose.model_copy(update={
            "pocket_score_boost": 0.0,
        })
    elif propose_name == "no_cluster_repack":
        cfg.propose = cfg.propose.model_copy(update={
            "enable_cluster_repack": False,
        })
    elif propose_name == "no_cluster_copy":
        cfg.propose = cfg.propose.model_copy(update={
            "use_cluster_copy": False,
        })
    elif propose_name == "no_open_void_pocket":
        cfg.propose = cfg.propose.model_copy(update={
            "use_open_void_pocket": False,
        })
    elif propose_name == "no_local_se2":
        cfg.propose = cfg.propose.model_copy(update={
            "enable_local_se2": False,
        })
    elif propose_name == "no_cluster_relocate":
        cfg.propose = cfg.propose.model_copy(update={
            "enable_cluster_relocate": False,
        })
    elif propose_name == "no_densify_hijack":
        cfg.propose = cfg.propose.model_copy(update={
            "densify_on_void_hijack": False,
        })
    elif propose_name == "no_unified_reserve":
        cfg.propose = cfg.propose.model_copy(update={
            "unified_void_reserve": False,
        })
    elif propose_name == "no_void_contact_hybrid":
        cfg.propose = cfg.propose.model_copy(update={
            "void_seek_contact_hybrid": False,
        })
    elif propose_name == "no_motif_topo_anchors":
        cfg.propose = cfg.propose.model_copy(update={
            "motif_use_topo_anchors": False,
        })
    elif propose_name == "no_override":
        cfg.propose = cfg.propose.model_copy(update={
            "late_border_void_override_ratio": 0.0,
            "late_border_void_release_ratio": 0.0,
        })
    elif propose_name == "no_void_hijack":
        cfg.propose = cfg.propose.model_copy(update={
            "enable_void_large_hijack": False,
            "void_island_score_boost": 0.0,
            "void_attractor_rule_weight": 0.0,
            "enable_gravity_compaction": False,
        })
    elif propose_name == "no_void_rank":
        cfg.propose = cfg.propose.model_copy(update={
            "void_rank_pole_weight": 0.0,
        })
    elif propose_name == "gravity_void_pole":
        # Diagnose-only: gravity remains off in shipped; this preset enables it.
        cfg.propose = cfg.propose.model_copy(update={
            "enable_gravity_compaction": True,
        })
    elif propose_name == "void_pso":
        # OOS-3 gated: only enable when props_pole≈0 after OOS-1+4; off by default.
        cfg.propose = cfg.propose.model_copy(update={
            "use_point_cloud": True,
            "use_guidance_walk": True,
        })
    elif propose_name == "no_propose":
        cfg.propose = ProposeConfig(
            use_neighbor_slide=False,
            use_voronoi=False,
            use_point_cloud=False,
            use_guidance_walk=False,
            use_ribbon_seeds=False,
            use_group_edge_seeds=False,
            use_border_edge_seeds=False,
            use_guidance_propositions=False,
        )
    return cfg


def _parse_matrix(specs: list[str]) -> dict[str, list[str]]:
    """Parse axis=v1,v2 into dict."""
    out: dict[str, list[str]] = {}
    for spec in specs:
        if "=" not in spec:
            raise ValueError(f"matrix entry must be axis=v1,v2 got {spec!r}")
        axis, vals = spec.split("=", 1)
        out[axis.strip()] = [v.strip() for v in vals.split(",") if v.strip()]
    return out


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cases", nargs="*", help="Case names / family prefixes / tags")
    parser.add_argument("--tags", nargs="*", help="Tags to run (e.g. dense, void)")
    parser.add_argument("--family", type=str, default=None, help="Filter by family name")
    parser.add_argument("--all", action="store_true", help="Run all cases (overrides default tight_pack)")
    parser.add_argument(
        "--matrix",
        nargs="*",
        help="Expand axes e.g. pitch=1.6,1.8 (reserved; currently filters only)",
    )
    parser.add_argument("--force", action="store_true", help="Allow >40 matrix/suite runs")
    parser.add_argument(
        "--propose",
        nargs="*",
        default=["shipped"],
        choices=[
            "shipped",
            "local_compact",
            "no_propose",
            "no_void_boost",
            "void_boost_high",
            "no_void_yield_densify",
            "no_void_hijack",
            "no_override",
            "no_greedy_nest",
            "no_pocket_fit",
            "no_small_prefer",
            "no_pocket_mis_boost",
            "no_cluster_repack",
            "no_cluster_copy",
            "no_open_void_pocket",
            "no_local_se2",
            "no_cluster_relocate",
            "no_densify_hijack",
            "no_unified_reserve",
            "no_void_contact_hybrid",
            "no_motif_topo_anchors",
            "no_void_rank",
            "gravity_void_pole",
            "void_pso",
        ],
    )
    parser.add_argument("--dfs-modes", nargs="*", default=["merged_loose_tight"])
    parser.add_argument("--seeds", type=int, nargs="*", default=[0])
    parser.add_argument("--gate", action="store_true", help="Fail if case floors / baselines not met")
    parser.add_argument("--update-baselines", action="store_true", help="Update docs/nesting_baselines.json")
    parser.add_argument(
        "--baselines",
        type=Path,
        default=Path("docs/nesting_baselines.json"),
    )
    args = parser.parse_args()

    if args.matrix:
        _parse_matrix(args.matrix)  # validate syntax; expansion reserved for CaseSpec knobs

    if args.all:
        cases_to_run = get_all_cases()
    elif args.cases or args.tags or args.family:
        cases_to_run = resolve_cases(args.cases, tags=args.tags, family=args.family)
    else:
        cases_to_run = resolve_cases(tags=["tight_pack"])

    if not cases_to_run:
        print("No cases matched.")
        sys.exit(1)

    n_runs = len(cases_to_run) * len(args.propose) * len(args.dfs_modes) * len(args.seeds)
    if n_runs > 40 and not args.force:
        print(f"Refusing {n_runs} runs (>40); pass --force to override.")
        sys.exit(2)

    results = []
    baselines: dict = {}
    if args.baselines.exists():
        baselines = json.loads(args.baselines.read_text())

    gate_fail = False
    for case in cases_to_run:
        for propose in args.propose:
            for dfs_mode in args.dfs_modes:
                cfg = _build_cfg(propose, dfs_mode)
                evaluator = NestingPipelineEvaluator(case, cfg)

                case_metrics = []
                for seed in args.seeds:
                    print(
                        f"Running {case.name} | {propose} | {dfs_mode} | seed={seed}...",
                        flush=True,
                    )
                    metrics = evaluator.run_full_pipeline(seed)
                    case_metrics.append(metrics)
                    print(
                        f"  -> parts={metrics.parts_final} area={metrics.area_coverage:.3f} "
                        f"kiss_s={metrics.kiss_seed:.2f} kiss_o={metrics.kiss_outline:.2f} "
                        f"auc={metrics.density_auc:.2f} ok={metrics.independent_ok} "
                        f"free={metrics.free_kind}/{metrics.largest_free_over_part:.1f} "
                        f"void={metrics.void_props}/{metrics.void_graph}/"
                        f"{metrics.void_selected_nest}/{metrics.void_selected_refine} "
                        f"time={metrics.time_s:.2f}s"
                    )

                avg_parts = np.mean([m.parts_final for m in case_metrics])
                avg_area = np.mean([m.area_coverage for m in case_metrics])
                avg_parts_delta = np.mean([m.parts_delta for m in case_metrics])
                avg_area_delta = np.mean([m.area_coverage_delta for m in case_metrics])
                avg_kiss = np.mean([m.kiss_seed for m in case_metrics])
                avg_kiss_o = np.mean([m.kiss_outline for m in case_metrics])
                avg_kiss_st = np.mean([m.kiss_standoff for m in case_metrics])
                avg_auc = np.mean([m.density_auc for m in case_metrics])
                avg_time = np.mean([m.time_s for m in case_metrics])
                all_indep = all(m.independent_ok for m in case_metrics)

                row = {
                    "case": case.name,
                    "family": case.family,
                    "demand": case.demand_ratio,
                    "propose": propose,
                    "dfs_mode": dfs_mode,
                    "parts": float(avg_parts),
                    "area": float(avg_area),
                    "parts_delta": float(avg_parts_delta),
                    "area_delta": float(avg_area_delta),
                    "kiss": float(avg_kiss),
                    "kiss_outline": float(avg_kiss_o),
                    "kiss_standoff": float(avg_kiss_st),
                    "density_auc": float(avg_auc),
                    "time": float(avg_time),
                    "independent_ok": all_indep,
                }
                results.append(row)

                if args.gate:
                    fails = metrics_meet_floors(case_metrics[0], case.floors)
                    key = f"{case.name}|{propose}|{dfs_mode}"
                    if key in baselines:
                        b = baselines[key]
                        if avg_parts + 1e-9 < b.get("parts", 0) * 0.9:
                            fails.append("baseline_parts")
                        if avg_area + 1e-9 < b.get("area", 0) * 0.9:
                            fails.append("baseline_area")
                    if fails:
                        gate_fail = True
                        print(f"  GATE FAIL {case.name}: {fails}")

                if args.update_baselines:
                    key = f"{case.name}|{propose}|{dfs_mode}"
                    baselines[key] = {
                        "parts": float(avg_parts),
                        "area": float(avg_area),
                        "time": float(avg_time),
                        "kiss": float(avg_kiss),
                    }

    print(
        "\n| Case | Demand | Propose | DFS | Parts | Area | ΔParts | KissS | KissO | AUC | Time | Indep |"
    )
    print("|------|--------|---------|-----|-------|------|--------|-------|-------|-----|------|-------|")
    for r in results:
        print(
            f"| {r['case']} | {r['demand']:.2f} | {r['propose']} | {r['dfs_mode']} | "
            f"{r['parts']:.1f} | {r['area']:.3f} | {r['parts_delta']:.1f} | "
            f"{r['kiss']:.2f} | {r['kiss_outline']:.2f} | {r['density_auc']:.2f} | "
            f"{r['time']:.2f}s | {r['independent_ok']} |"
        )

    if args.update_baselines:
        args.baselines.parent.mkdir(parents=True, exist_ok=True)
        args.baselines.write_text(json.dumps(baselines, indent=2, sort_keys=True) + "\n")
        print(f"Wrote baselines to {args.baselines}")

    if args.gate and gate_fail:
        sys.exit(1)


if __name__ == "__main__":
    main()
