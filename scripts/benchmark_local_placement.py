#!/usr/bin/env python3
"""Unified local placement movement benchmark: cast knobs, squeeze, walkers, stacks."""

import argparse
import json
import sys
import time
from dataclasses import asdict
from pathlib import Path
from typing import Any

import numpy as np

from nest_graph.config import BuildGraphConfig, ProposeConfig
from scripts.benchmark_propose_common import (
    GUIDANCE_ABLATION_SEEDS,
    ProposeBenchmarkMetrics,
    SCENARIO_WEIGHTS,
    composite_local_score,
    run_propose_with_metrics,
    run_propose_with_squeeze_metrics,
    shipped_propose_config,
)


def _shipped_base(**overrides: Any) -> ProposeConfig:
    return shipped_propose_config(**overrides)


LOCAL_PLACEMENT_PRESETS: dict[str, dict[str, Any]] = {
    "shipped": {},
    # A: C++ cast knobs
    "tight_only": {
        "use_guidance_propositions": True,
        "guidance_use_tight_packing": True,
        "guidance_use_corner_alignment": False,
        "guidance_enable_grid": False,
    },
    "tight_corner": {
        "use_guidance_propositions": True,
        "guidance_use_tight_packing": True,
        "guidance_use_corner_alignment": True,
        "guidance_enable_grid": False,
    },
    "tight_corner_grid": {
        "use_guidance_propositions": True,
        "guidance_use_tight_packing": True,
        "guidance_use_corner_alignment": True,
        "guidance_enable_grid": True,
    },
    "floor_walk_heavy": {
        "use_guidance_propositions": True,
        "guidance_use_tight_packing": False,
        "guidance_use_corner_alignment": False,
        "guidance_enable_grid": True,
    },
    "max_props_3": {"guidance_max_propositions": 3},
    "max_props_6": {"guidance_max_propositions": 6},
    "max_props_8": {"guidance_max_propositions": 8},
    "max_props_12": {"guidance_max_propositions": 12},
    # B: post-rank compaction
    "squeeze_off": {"cast_squeeze_top_k": 0, "cast_squeeze_passes": 0},
    "squeeze_4": {"cast_squeeze_top_k": 4, "cast_squeeze_passes": 1},
    "squeeze_8": {"cast_squeeze_top_k": 8, "cast_squeeze_passes": 1},
    "squeeze_16": {"cast_squeeze_top_k": 16, "cast_squeeze_passes": 1},
    "squeeze_8_double": {"cast_squeeze_top_k": 8, "cast_squeeze_passes": 2},
    # C: geometric local movers
    "neighbor_off": {"use_neighbor_slide": False},
    "neighbor_frac_25": {
        "use_neighbor_slide": True,
        "neighbor_slide_pool_fraction": 0.25,
    },
    "neighbor_frac_50": {
        "use_neighbor_slide": True,
        "neighbor_slide_pool_fraction": 0.5,
    },
    "guidance_walk_on": {"use_guidance_walk": True},
    "board_edge_refine_off": {"board_edge_guidance_refine": False},
    "board_edge_refine_on": {"board_edge_guidance_refine": True},
    # D: combined stacks
    "compact_light": {"cast_squeeze_top_k": 8, "cast_squeeze_passes": 1},
    "compact_cast": {
        "use_guidance_propositions": True,
        "guidance_use_tight_packing": True,
        "guidance_use_corner_alignment": True,
        "guidance_enable_grid": False,
        "cast_squeeze_top_k": 8,
        "cast_squeeze_passes": 1,
    },
    "compact_full": {
        "use_guidance_propositions": True,
        "guidance_use_tight_packing": True,
        "guidance_use_corner_alignment": True,
        "guidance_enable_grid": True,
        "use_guidance_walk": True,
        "cast_squeeze_top_k": 16,
        "cast_squeeze_passes": 1,
    },
    "compact_neighbor": {
        "use_guidance_propositions": False,
        "use_neighbor_slide": True,
        "cast_squeeze_top_k": 8,
        "cast_squeeze_passes": 1,
    },
    "local_compact": {},
}

FULL_BENCHMARK_PRESETS = (
    "shipped",
    "squeeze_off",
    "squeeze_8",
    "squeeze_16",
    "squeeze_8_double",
    "tight_corner",
    "tight_corner_grid",
    "compact_cast",
    "compact_light",
    "compact_full",
    "neighbor_off",
    "guidance_walk_on",
    "max_props_8",
    "local_compact",
)

QUICK_PRESETS = (
    "shipped",
    "squeeze_off",
    "squeeze_8",
    "squeeze_8_double",
    "tight_corner",
    "compact_cast",
    "compact_full",
    "guidance_walk_on",
    "neighbor_off",
)

DEFAULT_SCENARIOS = (
    "empty_base",
    "partial_pack",
    "two_clusters",
    "hole_board",
    "packed_border",
)


def propose_config_for_preset(name: str) -> ProposeConfig:
    if name == "local_compact":
        return ProposeConfig.local_compact_profile()
    if name not in LOCAL_PLACEMENT_PRESETS:
        raise ValueError(f"unknown preset {name!r}")
    return _shipped_base(**LOCAL_PLACEMENT_PRESETS[name])


def _format_table(rows: list[ProposeBenchmarkMetrics]) -> str:
    by_key: dict[tuple[str, str], list[ProposeBenchmarkMetrics]] = {}
    for r in rows:
        by_key.setdefault((r.preset, r.scenario), []).append(r)

    lines = [
        "| preset | scenario | contact_min | kiss | squeeze_delta | squeeze_moved | time_s | composite |",
        "|--------|----------|-------------|------|---------------|---------------|--------|-----------|",
    ]
    for (preset, scenario), agg in sorted(by_key.items()):
        cd = float(np.mean([a.contact_dist_min for a in agg]))
        kiss = float(np.mean([a.kiss_fraction for a in agg]))
        sd = float(np.mean([a.squeeze_contact_delta for a in agg]))
        sm = float(np.mean([a.squeeze_improved_count for a in agg]))
        t = float(np.mean([a.propose_time_s for a in agg]))
        comp = float(np.mean([composite_local_score(a) for a in agg]))
        lines.append(
            f"| {preset} | {scenario} | {cd:.4f} | {kiss:.2f} | {sd:.4f} | {sm:.1f} | {t:.3f} | {comp:.3f} |",
        )
    return "\n".join(lines)


def rank_presets(rows: list[ProposeBenchmarkMetrics]) -> list[tuple[str, float]]:
    by_preset: dict[str, list[float]] = {}
    for r in rows:
        w = SCENARIO_WEIGHTS.get(r.scenario, 1.0)
        by_preset.setdefault(r.preset, []).append(
            composite_local_score(r) * w,
        )
    ranked = [
        (name, float(np.mean(scores)))
        for name, scores in by_preset.items()
    ]
    ranked.sort(key=lambda x: x[1], reverse=True)
    return ranked


def run_benchmark(
    presets: list[str],
    scenarios: list[str],
    seeds: list[int],
    *,
    use_squeeze_metrics: bool = True,
) -> list[ProposeBenchmarkMetrics]:
    base_cfg = BuildGraphConfig()
    rows: list[ProposeBenchmarkMetrics] = []
    runner = (
        run_propose_with_squeeze_metrics if use_squeeze_metrics
        else run_propose_with_metrics
    )
    for preset in presets:
        cfg = propose_config_for_preset(preset)
        for scenario in scenarios:
            g_seeds = GUIDANCE_ABLATION_SEEDS.get(scenario)
            for seed in seeds:
                rows.append(
                    runner(
                        cfg,
                        scenario,
                        seed,
                        base_cfg,
                        preset_label=preset,
                        guidance_seed_coords=g_seeds,
                    )
                )
    return rows


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--presets",
        nargs="*",
        default=list(LOCAL_PLACEMENT_PRESETS.keys()),
    )
    parser.add_argument(
        "--scenarios",
        nargs="*",
        default=list(DEFAULT_SCENARIOS),
    )
    parser.add_argument("--seeds", type=int, nargs="*", default=list(range(10)))
    parser.add_argument(
        "--full",
        action="store_true",
        help="Full preset matrix with seeds 0-9",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("docs/local_placement_benchmark.txt"),
    )
    parser.add_argument(
        "--json",
        type=Path,
        default=Path("docs/local_placement_benchmark.json"),
    )
    args = parser.parse_args()

    seeds = list(range(3)) if args.quick else args.seeds
    if args.full:
        presets = list(FULL_BENCHMARK_PRESETS)
        seeds = list(range(10))
        scenarios = list(DEFAULT_SCENARIOS)
    elif args.quick:
        presets = list(QUICK_PRESETS)
        scenarios = ["partial_pack", "two_clusters", "empty_base"]
    else:
        presets = args.presets
        scenarios = args.scenarios
    for name in presets:
        if name not in LOCAL_PLACEMENT_PRESETS:
            print(f"Unknown preset: {name}", file=sys.stderr)
            sys.exit(1)

    t0 = time.perf_counter()
    rows = run_benchmark(presets, scenarios, seeds)
    elapsed = time.perf_counter() - t0

    table = _format_table(rows)
    ranked = rank_presets(rows)
    print(table)
    print("\n## Weighted ranking")
    for preset, score in ranked:
        print(f"- **{score:.3f}** — `{preset}`")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w") as f:
        f.write("# Local placement movement benchmark\n\n")
        f.write(f"Seeds: {seeds}\n")
        f.write(f"Runtime: {elapsed:.1f}s\n\n")
        f.write(table)
        f.write("\n\n## Weighted ranking\n\n")
        for preset, score in ranked:
            f.write(f"- **{score:.3f}** — `{preset}`\n")

    args.json.parent.mkdir(parents=True, exist_ok=True)
    with args.json.open("w") as f:
        json.dump([asdict(r) for r in rows], f, indent=2)

    print(f"\nWrote {args.output} and {args.json} ({elapsed:.1f}s)")


if __name__ == "__main__":
    main()
