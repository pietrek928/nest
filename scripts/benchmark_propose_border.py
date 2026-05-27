#!/usr/bin/env python3
"""Benchmark propose presets for placing a part against the sheet border (empty board)."""

import argparse
import json
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import numpy as np
from shapely.geometry import Point, Polygon

from nest_graph.board import board_context_from_geometry
from nest_graph.config import BuildGraphConfig, ProposeConfig
from nest_graph.propose import (
    ProposeGeometry,
    propose_coords_with_strategy,
    propositions_to_ndarray,
)
from nest_graph.utils import normalize_poly, transform_poly


def _benchmark_cfg(**overrides: Any) -> ProposeConfig:
    base: dict[str, Any] = {
        "max_proposals": 12,
        "candidate_pool": 32,
        "use_voronoi": True,
        "use_point_cloud": False,
        "use_free_region_search": True,
        "ranking_mode": "clearance",
        "smart_push_target": True,
        "trim_candidates_by_clearance": True,
        "use_ribbon_seeds": False,
        "use_border_focus": False,
        "use_border_edge_seeds": False,
        "border_focus_ranking": False,
    }
    base.update(overrides)
    return ProposeConfig(**base)


BORDER_PRESETS: dict[str, ProposeConfig] = {
    "centroid_clearance": _benchmark_cfg(),
    "ribbon_only": _benchmark_cfg(use_ribbon_seeds=True),
    "border_focus": _benchmark_cfg(
        use_border_focus=True,
        use_border_edge_seeds=True,
        border_focus_ranking=True,
    ),
    "border_ribbon": _benchmark_cfg(
        use_ribbon_seeds=True,
        use_border_focus=True,
        use_border_edge_seeds=True,
        border_focus_ranking=True,
    ),
    "shipped": _benchmark_cfg(
        use_ribbon_seeds=True,
        use_border_focus=True,
        use_border_edge_seeds=True,
        border_focus_ranking=True,
        trim_candidates_by_clearance=True,
    ),
}


@dataclass
class BorderBenchmarkRow:
    preset: str
    seed: int
    valid_count: int
    border_dist_mean: float
    border_dist_min: float
    border_slack_mean: float
    clearance_mean: float
    propose_time_s: float


def _triangle_board() -> Polygon:
    return Polygon([(0, 0), (1.2, 0), (0, 1.1)])


def _rect_part():
    return normalize_poly(Polygon([(0, 0), (0.12, 0), (0.12, 0.08), (0, 0.08)]))


def _border_stats(
    coords_list,
    board: Polygon,
    part_poly: Polygon,
    min_dist: float,
    pt_push: Point,
    epsilon_ratio: float,
) -> tuple[int, float, float, float, float]:
    sheet, _ = board_context_from_geometry(board)
    geom = ProposeGeometry(board, Polygon(), part_poly, min_dist, epsilon_ratio=epsilon_ratio)
    border_dists: list[float] = []
    slacks: list[float] = []
    clearances: list[float] = []
    valid = 0
    for coords in coords_list:
        placed = geom.placed_at(coords)
        if not geom.is_valid_placement(placed, pt_push, (coords[0], coords[1])):
            continue
        valid += 1
        placed_poly = transform_poly(part_poly, coords)
        bd = float(placed_poly.distance(sheet.exterior))
        border_dists.append(bd)
        slacks.append(bd - min_dist)
        g = geom.placement_guidance(placed, (coords[0], coords[1]), pt_push)
        if not g.is_penetrating:
            clearances.append(float(g.clearance))
    if not border_dists:
        return valid, 0.0, 0.0, 0.0, 0.0
    return (
        valid,
        float(np.mean(border_dists)),
        float(np.min(border_dists)),
        float(np.mean(slacks)),
        float(np.mean(clearances)) if clearances else 0.0,
    )


def run_row(preset: str, seed: int, cfg: BuildGraphConfig) -> BorderBenchmarkRow:
    board = _triangle_board()
    part = _rect_part()
    base_shape = Polygon()
    propose_cfg = cfg.propose
    min_dist = cfg.board_min_dist()
    eps = propose_cfg.placement_clearance_epsilon_ratio
    push = Point(board.centroid)

    t0 = time.perf_counter()
    coords = propose_coords_with_strategy(
        base_shape, part, board, propose_cfg,
        min_dist=min_dist, pt_push=push,
    )
    elapsed = time.perf_counter() - t0

    valid, bd_mean, bd_min, slack_mean, c_mean = _border_stats(
        coords, board, part, min_dist, push, eps,
    )
    return BorderBenchmarkRow(
        preset=preset,
        seed=seed,
        valid_count=valid,
        border_dist_mean=bd_mean,
        border_dist_min=bd_min,
        border_slack_mean=slack_mean,
        clearance_mean=c_mean,
        propose_time_s=elapsed,
    )


def _format_table(rows: list[BorderBenchmarkRow]) -> str:
    by_preset: dict[str, list[BorderBenchmarkRow]] = {}
    for r in rows:
        by_preset.setdefault(r.preset, []).append(r)
    lines = [
        "| preset | valid | border_dist_mean | border_dist_min | border_slack_mean | clearance_mean | time_s |",
        "|--------|-------|------------------|-----------------|-------------------|----------------|--------|",
    ]
    for preset, agg in sorted(by_preset.items()):
        lines.append(
            f"| {preset} | {np.mean([a.valid_count for a in agg]):.1f} | "
            f"{np.mean([a.border_dist_mean for a in agg]):.4f} | "
            f"{np.mean([a.border_dist_min for a in agg]):.4f} | "
            f"{np.mean([a.border_slack_mean for a in agg]):.4f} | "
            f"{np.mean([a.clearance_mean for a in agg]):.4f} | "
            f"{np.mean([a.propose_time_s for a in agg]):.3f} |"
        )
    return "\n".join(lines)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--presets", nargs="*", default=list(BORDER_PRESETS.keys()))
    parser.add_argument("--seeds", type=int, nargs="*", default=list(range(10)))
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("docs/propose_border_benchmark_results.txt"),
    )
    args = parser.parse_args()

    cfg = BuildGraphConfig()
    rows: list[BorderBenchmarkRow] = []
    for preset in args.presets:
        if preset not in BORDER_PRESETS:
            print(f"Unknown preset: {preset}", file=sys.stderr)
            sys.exit(1)
        run_cfg = cfg.model_copy(deep=True)
        run_cfg.propose = BORDER_PRESETS[preset]
        for seed in args.seeds:
            rows.append(run_row(preset, seed, run_cfg))

    table = _format_table(rows)
    print(table)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w") as f:
        f.write("# Border propose benchmark (empty triangle board, rect part)\n\n")
        f.write("Lower `border_dist_min` / `border_slack_mean` = tighter fit to sheet edge.\n\n")
        f.write(table)
        f.write("\n\n## Raw rows\n\n")
        for r in rows:
            f.write(json.dumps(asdict(r)) + "\n")
    print(f"\nWrote {args.output}", file=sys.stderr)


if __name__ == "__main__":
    main()
