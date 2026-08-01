#!/usr/bin/env python3
"""Benchmark propose config presets for gap-fitting quality."""

import argparse
import sys
from typing import Any

import numpy as np

from nest_graph.config import BuildGraphConfig, ProposeConfig
from scripts.nesting_evaluator import NestingPipelineEvaluator, ProposeBenchmarkMetrics
from scripts.nesting_fixtures import resolve_cases


def shipped_propose_config(**overrides: Any) -> ProposeConfig:
    cfg = ProposeConfig()
    for key, val in overrides.items():
        setattr(cfg, key, val)
    return cfg


PROPOSE_BENCHMARK_PRESETS = {
    "shipped": shipped_propose_config(),
    "shipped_no_phase2": shipped_propose_config(
        use_neighbor_slide=False,
        use_axis_push=False,
        use_bottom_left=False,
        use_nfp_vertices=False,
    ),
    "shipped_no_guidance_cast": shipped_propose_config(
        use_guidance_propositions=False,
    ),
    "contact_rank": shipped_propose_config(
        use_contact_ranking=True,
        use_contact_clearance_hybrid=False,
        ranking_mode="contact",
    ),
    "clearance_rank": shipped_propose_config(
        use_contact_ranking=False,
        ranking_mode="clearance",
    ),
    "ribbon_heavy": shipped_propose_config(
        use_ribbon_seeds=True,
        use_group_edge_seeds=False,
        use_guidance_propositions=False,
        use_neighbor_slide=False,
        use_axis_push=False,
        use_bottom_left=False,
        use_nfp_vertices=False,
    ),
    "kiss_heavy": shipped_propose_config(
        use_neighbor_slide=True,
        placement_num_angles=24,
        guidance_proposition_seed_count=16,
        guidance_max_propositions=8,
        guidance_enable_grid=True,
        contact_trim_fraction=0.8,
        contact_clearance_hybrid_weight=0.1,
    ),
    "density_heavy": shipped_propose_config(),
    "local_compact": ProposeConfig.local_compact_profile(),
    "guidance_cast_heavy": shipped_propose_config(
        use_guidance_propositions=True,
        guidance_use_tight_packing=True,
        use_neighbor_slide=False,
        use_ribbon_seeds=False,
    ),
    "cast_first_quality": shipped_propose_config(
        use_guidance_propositions=True,
        guidance_use_tight_packing=True,
        guidance_enable_grid=True,
        guidance_cast_refine_top_k=16,
        cast_rank_boost=0.5,
        cast_squeeze_top_k=8,
        cast_squeeze_passes=1,
        trim_candidates_by_clearance=True,
        use_full_packed_obstacle=True,
    ),
    "free_clearance": shipped_propose_config(
        candidate_pool=12,
        trim_candidates_by_clearance=False,
        use_ribbon_seeds=False,
        use_contact_ranking=False,
    ),
    "ribbon_free": shipped_propose_config(
        candidate_pool=12,
        use_ribbon_seeds=True,
        use_contact_ranking=False,
    ),
}


def _format_table(rows: list[ProposeBenchmarkMetrics]) -> str:
    by_key: dict[tuple[str, str], list[ProposeBenchmarkMetrics]] = {}
    for r in rows:
        by_key.setdefault((r.preset, r.scenario), []).append(r)

    lines = [
        "| preset | scenario | valid | contact_min | clearance_min | kiss_frac | pool | final | graph+ | time_s |",
        "|--------|----------|-------|-------------|---------------|-----------|------|-------|--------|--------|",
    ]
    for (preset, scenario), agg in sorted(by_key.items()):
        valid = float(np.mean([a.valid_count for a in agg]))
        cd_min = float(np.mean([a.contact_dist_min for a in agg]))
        c_min = float(np.mean([a.top_clearance_min for a in agg]))
        kiss = float(np.mean([a.kiss_fraction for a in agg]))
        pool = float(np.mean([a.raw_pool_size for a in agg]))
        final = float(np.mean([a.final_count for a in agg]))
        delta = float(np.mean([a.graph_nodes_vs_random for a in agg]))
        t = float(np.mean([a.propose_time_s for a in agg]))
        lines.append(
            f"| {preset} | {scenario} | {valid:.1f} | {cd_min:.4f} | {c_min:.4f} | "
            f"{kiss:.2f} | {pool:.0f} | {final:.1f} | {delta:+.1f} | {t:.3f} |"
        )
    return "\n".join(lines)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--presets",
        nargs="*",
        default=["shipped"],
        help=f"Presets: {', '.join(PROPOSE_BENCHMARK_PRESETS)}",
    )
    parser.add_argument(
        "--cases",
        nargs="*",
        default=["border_then_fill"],
    )
    parser.add_argument("--seeds", type=int, nargs="*", default=list(range(3)))
    args = parser.parse_args()

    presets = args.presets or list(PROPOSE_BENCHMARK_PRESETS.keys())

    cases_to_run = resolve_cases(args.cases)

    if not cases_to_run:
        print("No cases matched.")
        sys.exit(1)

    rows: list[ProposeBenchmarkMetrics] = []
    
    for case in cases_to_run:
        for name in presets:
            if name not in PROPOSE_BENCHMARK_PRESETS:
                print(f"Unknown preset: {name}", file=sys.stderr)
                sys.exit(1)
                
            cfg = BuildGraphConfig()
            cfg.propose = PROPOSE_BENCHMARK_PRESETS[name]
            
            evaluator = NestingPipelineEvaluator(case, cfg)
            
            for seed in args.seeds:
                print(f"Running {name} on {case.name} seed={seed}...")
                metrics = evaluator.run_propose_only(seed, name)
                rows.append(metrics)

    print("\n" + _format_table(rows))


if __name__ == "__main__":
    main()
