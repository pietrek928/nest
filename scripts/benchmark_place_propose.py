#!/usr/bin/env python3
"""Place-aware propose benchmark: graph yield per zone profile and scenario."""

import argparse
import json
import sys
import time
from dataclasses import asdict
from pathlib import Path
from typing import Any

import numpy as np

from nest_graph.config import PLACE_ZONES, BuildGraphConfig, ProposeConfig
from scripts.benchmark_propose_common import (
    ProposeBenchmarkMetrics,
    SCENARIO_WEIGHTS,
    composite_place_score,
    run_place_propose_metrics,
    shipped_propose_config,
)

PLACE_SCENARIOS = (
    "empty_base",
    "partial_pack",
    "two_clusters",
    "hole_board",
    "packed_border",
    "border_gap",
    "interior_after_border",
)


def _place_presets() -> dict[str, ProposeConfig]:
    shipped = shipped_propose_config()
    presets: dict[str, ProposeConfig] = {
        "shipped": shipped,
        "local_compact": ProposeConfig.local_compact_profile(),
        "place_routed": shipped.model_copy(update={
            "place_profiles_enabled": True,
        }),
    }
    for zone in PLACE_ZONES:
        if zone == "empty_border":
            continue
        presets[f"place_{zone}"] = ProposeConfig.for_place(
            zone,
            base=shipped,
        ).model_copy(update={"place_profiles_enabled": True})
    return presets


def _aggregate(
    rows: list[ProposeBenchmarkMetrics],
) -> list[tuple[str, str, float, float, float]]:
    buckets: dict[tuple[str, str], list[ProposeBenchmarkMetrics]] = {}
    for row in rows:
        buckets.setdefault((row.preset, row.scenario), []).append(row)
    out: list[tuple[str, str, float, float, float]] = []
    for (preset, scenario), group in sorted(buckets.items()):
        weight = SCENARIO_WEIGHTS.get(scenario, 1.0)
        scores = [composite_place_score(r) * weight for r in group]
        yields = [r.graph_yield for r in group]
        nodes = [r.graph_nodes for r in group]
        out.append((
            preset,
            scenario,
            float(np.mean(scores)),
            float(np.mean(yields)),
            float(np.mean(nodes)),
        ))
    return out


def _markdown_table(agg: list[tuple[str, str, float, float, float]]) -> str:
    lines = [
        "# Place propose benchmark",
        "",
        "| preset | scenario | composite | graph_yield | graph_nodes |",
        "|--------|----------|-----------|-------------|-------------|",
    ]
    for preset, scenario, comp, gy, gn in agg:
        lines.append(
            f"| {preset} | {scenario} | {comp:.3f} | {gy:.3f} | {gn:.1f} |",
        )
    by_preset: dict[str, list[float]] = {}
    for preset, _scenario, comp, _gy, _gn in agg:
        by_preset.setdefault(preset, []).append(comp)
    lines.extend(["", "## Mean composite by preset", ""])
    ranked = sorted(
        ((preset, float(np.mean(vals))) for preset, vals in by_preset.items()),
        key=lambda row: row[1],
        reverse=True,
    )
    for preset, mean in ranked:
        lines.append(f"- **{preset}**: {mean:.3f}")
    return "\n".join(lines) + "\n"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--presets",
        nargs="*",
        default=None,
        help="Preset names (default: shipped, local_compact, place_routed, place_*)",
    )
    parser.add_argument(
        "--scenarios",
        nargs="*",
        default=list(PLACE_SCENARIOS),
    )
    parser.add_argument("--seeds", type=int, nargs="*", default=list(range(10)))
    parser.add_argument(
        "--quick",
        action="store_true",
        help="Seeds 0–2 only",
    )
    parser.add_argument(
        "--write-doc",
        action="store_true",
        help="Write docs/place_propose_benchmark.md",
    )
    args = parser.parse_args()

    seeds = list(range(3)) if args.quick else list(args.seeds)
    all_presets = _place_presets()
    preset_names = args.presets or [
        "shipped", "local_compact", "place_routed",
        "place_border_gap", "place_interior_pocket", "place_cluster_edge",
        "place_inter_cluster",
    ]
    for name in preset_names:
        if name not in all_presets:
            print(f"unknown preset: {name}", file=sys.stderr)
            sys.exit(1)

    base_cfg = BuildGraphConfig()
    rows: list[ProposeBenchmarkMetrics] = []
    t0 = time.perf_counter()
    for preset in preset_names:
        cfg = all_presets[preset]
        for scenario in args.scenarios:
            for seed in seeds:
                row = run_place_propose_metrics(
                    cfg,
                    scenario,
                    seed,
                    base_cfg,
                    preset_label=preset,
                )
                rows.append(row)
                print(json.dumps(asdict(row), default=float))

    agg = _aggregate(rows)
    table = _markdown_table(agg)
    print("\n" + table)
    print(f"elapsed={time.perf_counter() - t0:.1f}s seeds={seeds}")

    if args.write_doc:
        doc = Path(__file__).resolve().parents[1] / "docs" / "place_propose_benchmark.md"
        doc.write_text(
            table + f"\n_Generated {time.strftime('%Y-%m-%d %H:%M')} "
            f"seeds={seeds}_\n",
            encoding="utf-8",
        )
        print(f"Wrote {doc}")


if __name__ == "__main__":
    main()
