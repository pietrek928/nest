#!/usr/bin/env python3
"""Per-proposer ablation benchmark: one placement generator at a time."""

import argparse
import json
import sys
from dataclasses import asdict
from pathlib import Path

import numpy as np

from nest_graph.config import BuildGraphConfig
from nest_graph.propose import ALL_PROPOSER_NAMES
from scripts.benchmark_propose_common import (
    GUIDANCE_ABLATION_SEEDS,
    ProposeBenchmarkMetrics,
    ablation_propose_config,
    run_propose_with_metrics,
)


def _format_ablation_table(rows: list[ProposeBenchmarkMetrics]) -> str:
    by_key: dict[tuple[str, str], list[ProposeBenchmarkMetrics]] = {}
    for r in rows:
        by_key.setdefault((r.preset, r.scenario), []).append(r)

    lines = [
        "| proposer | scenario | valid | contact_min | kiss_frac | pool | final | time_s |",
        "|----------|----------|-------|-------------|-----------|------|-------|--------|",
    ]
    for (proposer, scenario), agg in sorted(by_key.items()):
        valid = float(np.mean([a.valid_count for a in agg]))
        cd_min = float(np.mean([a.contact_dist_min for a in agg]))
        kiss = float(np.mean([a.kiss_fraction for a in agg]))
        pool = float(np.mean([a.raw_pool_size for a in agg]))
        final = float(np.mean([a.final_count for a in agg]))
        t = float(np.mean([a.propose_time_s for a in agg]))
        lines.append(
            f"| {proposer} | {scenario} | {valid:.1f} | {cd_min:.4f} | "
            f"{kiss:.2f} | {pool:.0f} | {final:.1f} | {t:.3f} |"
        )
    return "\n".join(lines)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--proposers",
        nargs="*",
        default=None,
        help=f"Subset of: {', '.join(ALL_PROPOSER_NAMES)}",
    )
    parser.add_argument(
        "--scenarios",
        nargs="*",
        default=["partial_pack", "two_clusters", "empty_base"],
    )
    parser.add_argument("--seeds", type=int, nargs="*", default=list(range(10)))
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("docs/propose_ablation_results.txt"),
    )
    args = parser.parse_args()

    proposers = args.proposers or list(ALL_PROPOSER_NAMES)
    base_cfg = BuildGraphConfig()
    rows: list[ProposeBenchmarkMetrics] = []

    for name in proposers:
        if name not in ALL_PROPOSER_NAMES:
            print(f"Unknown proposer: {name}", file=sys.stderr)
            sys.exit(1)

    for scenario in args.scenarios:
        for name in proposers:
            cfg = ablation_propose_config(name)
            seeds = None
            if name == "guidance_propositions":
                seeds = GUIDANCE_ABLATION_SEEDS.get(scenario, [(0.5, 0.5, 0.0)])
            for seed in args.seeds:
                rows.append(
                    run_propose_with_metrics(
                        cfg,
                        scenario,
                        seed,
                        base_cfg,
                        preset_label=name,
                        enabled_proposers=frozenset({name}),
                        guidance_seed_coords=seeds,
                    )
                )

    table = _format_ablation_table(rows)
    print(table)

    out_path = args.output
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with out_path.open("w") as f:
        f.write("# Propose per-proposer ablation\n\n")
        f.write(table)
        f.write("\n\n## Raw rows (JSON lines)\n\n")
        for r in rows:
            f.write(json.dumps(asdict(r)) + "\n")
    print(f"\nWrote {out_path}", file=sys.stderr)


if __name__ == "__main__":
    main()
