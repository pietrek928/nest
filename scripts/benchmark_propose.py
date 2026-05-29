#!/usr/bin/env python3
"""Benchmark propose config presets for gap-fitting quality."""

import argparse
import json
import sys
from dataclasses import asdict
from pathlib import Path
from typing import Any

import numpy as np

from nest_graph.config import BuildGraphConfig
from scripts.benchmark_propose_common import (
    ProposeBenchmarkMetrics,
    run_propose_with_metrics,
    shipped_propose_config,
)


def _benchmark_propose_cfg(**overrides: Any):
    return shipped_propose_config(**overrides)


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
    "guidance_cast_heavy": shipped_propose_config(
        use_guidance_propositions=True,
        guidance_use_tight_packing=True,
        use_neighbor_slide=False,
        use_ribbon_seeds=False,
    ),
    "free_clearance": _benchmark_propose_cfg(
        candidate_pool=12,
        trim_candidates_by_clearance=False,
        use_ribbon_seeds=False,
        use_contact_ranking=False,
    ),
    "ribbon_free": _benchmark_propose_cfg(
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
        default=None,
        help=f"Presets: {', '.join(PROPOSE_BENCHMARK_PRESETS)}",
    )
    parser.add_argument(
        "--scenarios",
        nargs="*",
        default=["empty_base", "partial_pack", "two_clusters", "hole_board"],
    )
    parser.add_argument("--seeds", type=int, nargs="*", default=list(range(10)))
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("docs/propose_benchmark_results.txt"),
    )
    args = parser.parse_args()

    base_cfg = BuildGraphConfig()
    presets = args.presets or list(PROPOSE_BENCHMARK_PRESETS.keys())

    rows: list[ProposeBenchmarkMetrics] = []
    for name in presets:
        if name not in PROPOSE_BENCHMARK_PRESETS:
            print(f"Unknown preset: {name}", file=sys.stderr)
            sys.exit(1)
    for scenario in args.scenarios:
        for preset_name in presets:
            for seed in args.seeds:
                rows.append(
                    run_propose_with_metrics(
                        PROPOSE_BENCHMARK_PRESETS[preset_name],
                        scenario,
                        seed,
                        base_cfg,
                        preset_label=preset_name,
                    )
                )

    table = _format_table(rows)
    print(table)

    out_path = args.output
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with out_path.open("w") as f:
        f.write("# Propose preset benchmark\n\n")
        f.write(table)
        f.write("\n\n## Raw rows (JSON lines)\n\n")
        for r in rows:
            f.write(json.dumps(asdict(r)) + "\n")
    print(f"\nWrote {out_path}", file=sys.stderr)


if __name__ == "__main__":
    main()
