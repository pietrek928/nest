#!/usr/bin/env python3
"""Benchmark propose config presets for gap-fitting quality."""

import argparse
import json
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import numpy as np
from shapely.geometry import Point, Polygon

from nest_graph.build_graph import make_polygon_graph
from nest_graph.config import BuildGraphConfig, ProposeConfig
from nest_graph.propose import (
    ProposeGeometry,
    base_shape_from_selection,
    obstacle_shape_for_propose,
    propose_coords_with_strategy,
    propositions_to_ndarray,
)
from nest_graph.utils import normalize_poly, transform_poly


def _benchmark_propose_cfg(**overrides: Any) -> ProposeConfig:
    base: dict[str, Any] = {
        "max_proposals": 12,
        "candidate_pool": 12,
        "use_voronoi": True,
        "use_point_cloud": False,
        "use_free_region_search": True,
        "ranking_mode": "clearance",
        "smart_push_target": True,
        "trim_candidates_by_clearance": True,
        "use_ribbon_seeds": False,
    }
    base.update(overrides)
    return ProposeConfig(**base)


PROPOSE_BENCHMARK_PRESETS: dict[str, ProposeConfig] = {
    "free_clearance": _benchmark_propose_cfg(
        trim_candidates_by_clearance=False,
        use_ribbon_seeds=False,
    ),
    "clearance_pool": _benchmark_propose_cfg(use_ribbon_seeds=False),
    "ribbon_free": _benchmark_propose_cfg(use_ribbon_seeds=True),
    "shipped": _benchmark_propose_cfg(
        use_ribbon_seeds=True,
        use_border_focus=True,
        use_border_edge_seeds=True,
        border_focus_ranking=True,
        trim_candidates_by_clearance=True,
    ),
    "hybrid_rank": _benchmark_propose_cfg(
        ranking_mode="hybrid",
        use_ribbon_seeds=False,
    ),
    "multi_erosion": _benchmark_propose_cfg(
        multi_site_erosion=True,
        use_ribbon_seeds=False,
    ),
    "free_pso_light": _benchmark_propose_cfg(
        use_point_cloud=True,
        point_cloud_particles=6,
        point_cloud_iterations=8,
        use_ribbon_seeds=False,
    ),
    "free_walk_combo": _benchmark_propose_cfg(
        use_guidance_walk=True,
        use_ribbon_seeds=False,
    ),
}


@dataclass
class ProposeBenchmarkRow:
    preset: str
    scenario: str
    seed: int
    valid_count: int
    top_clearance_mean: float
    top_clearance_min: float
    graph_nodes: int
    graph_nodes_vs_random: int
    propose_time_s: float


def _triangle_board() -> Polygon:
    return Polygon([(0, 0), (1.2, 0), (0, 1.1)])


def _rect_poly():
    return normalize_poly(Polygon([(0, 0), (0.1, 0), (0.1, 0.1), (0, 0.1)]))


def _tri_poly():
    return normalize_poly(Polygon([(0, 0), (0.15, 0), (0, 0.07)]))


def _scenario_partial_pack(seed: int) -> tuple[Polygon, Polygon, Polygon, list]:
    board = _triangle_board()
    rect = _rect_poly()
    tri = _tri_poly()
    rng = np.random.default_rng(seed)
    t_rect = np.array([0.35, 0.25, rng.uniform(0, 0.5)])
    placed_rect = transform_poly(rect, t_rect)
    selected_polys = [placed_rect]
    selected_indices = [0]
    return board, tri, rect, (selected_polys, selected_indices)


def _scenario_empty_base(seed: int) -> tuple[Polygon, Polygon, Polygon, list]:
    board = _triangle_board()
    rect = _rect_poly()
    tri = _tri_poly()
    return board, rect, tri, ([], [])


def _scenario_two_clusters(seed: int) -> tuple[Polygon, Polygon, Polygon, list]:
    board = _triangle_board()
    rect = _rect_poly()
    tri = _tri_poly()
    rng = np.random.default_rng(seed)
    t1 = np.array([0.08, 0.08, rng.uniform(0, 0.3)])
    t2 = np.array([0.55, 0.55, rng.uniform(0, 0.3)])
    selected_polys = [transform_poly(rect, t1), transform_poly(rect, t2)]
    selected_indices = [0, 1]
    return board, tri, rect, (selected_polys, selected_indices)


def _clearance_stats(
    coords_list,
    board: Polygon,
    base_shape,
    part_poly: Polygon,
    min_dist: float,
    pt_push: Point,
    epsilon_ratio: float,
) -> tuple[int, float, float]:
    if not coords_list:
        return 0, 0.0, 0.0
    geom = ProposeGeometry(
        board, base_shape, part_poly, min_dist, epsilon_ratio=epsilon_ratio,
    )
    clearances: list[float] = []
    valid = 0
    for coords in coords_list:
        placed = geom.placed_at(coords)
        if not geom.is_valid_placement(placed, pt_push, (coords[0], coords[1])):
            continue
        valid += 1
        g = geom.placement_guidance(placed, (coords[0], coords[1]), pt_push)
        if not g.is_penetrating:
            clearances.append(float(g.clearance))
    if not clearances:
        return valid, 0.0, 0.0
    return valid, float(np.mean(clearances)), float(np.min(clearances))


def _graph_node_count(
    board: Polygon,
    part_poly: Polygon,
    proposals: np.ndarray,
    seed: int,
) -> tuple[int, int]:
    rng = np.random.default_rng(seed)
    random_t = rng.uniform(-0.2, 0.2, (8, 3)) * [0.4, 0.4, np.pi]
    graph_rand, _, _, _ = make_polygon_graph(
        board, [(part_poly, random_t)], min_dist=0.0,
    )
    n_rand = len(graph_rand.elems)
    if proposals.shape[0] == 0:
        return n_rand, 0
    graph_both, _, _, _ = make_polygon_graph(
        board, [(part_poly, np.concatenate([random_t, proposals]))],
        min_dist=0.0,
    )
    return n_rand, len(graph_both.elems)


def run_propose_benchmark_preset(
    preset_name: str,
    scenario: str,
    seed: int,
    base_cfg: BuildGraphConfig,
) -> ProposeBenchmarkRow:
    propose_cfg = PROPOSE_BENCHMARK_PRESETS[preset_name]
    cfg = base_cfg.model_copy(deep=True)
    cfg.propose = propose_cfg
    min_dist = cfg.board_min_dist()
    eps = propose_cfg.placement_clearance_epsilon_ratio

    placed_polys: list = []
    if scenario == "empty_base":
        board, part_poly, _, _ = _scenario_empty_base(seed)
        base_shape = Polygon()
    elif scenario == "partial_pack":
        board, part_poly, _, (selected_polys, selected_indices) = _scenario_partial_pack(seed)
        base_shape = base_shape_from_selection(selected_polys, selected_indices)
        placed_polys = [selected_polys[i] for i in selected_indices]
    elif scenario == "two_clusters":
        board, part_poly, _, (selected_polys, selected_indices) = _scenario_two_clusters(seed)
        base_shape = base_shape_from_selection(selected_polys, selected_indices)
        placed_polys = [selected_polys[i] for i in selected_indices]
    else:
        raise ValueError(scenario)

    obstacle = obstacle_shape_for_propose(placed_polys, part_poly, min_dist)
    push = Point(board.centroid)
    if not obstacle.is_empty:
        push = obstacle.centroid

    t0 = time.perf_counter()
    coords = propose_coords_with_strategy(
        obstacle,
        part_poly,
        board,
        propose_cfg,
        min_dist=min_dist,
        pt_push=push,
    )
    elapsed = time.perf_counter() - t0

    valid, c_mean, c_min = _clearance_stats(
        coords, board, obstacle, part_poly, min_dist, push, eps,
    )
    proposals = propositions_to_ndarray(coords)
    n_rand, n_both = _graph_node_count(board, part_poly, proposals, seed)

    return ProposeBenchmarkRow(
        preset=preset_name,
        scenario=scenario,
        seed=seed,
        valid_count=valid,
        top_clearance_mean=c_mean,
        top_clearance_min=c_min,
        graph_nodes=n_both,
        graph_nodes_vs_random=max(0, n_both - n_rand),
        propose_time_s=elapsed,
    )


def _format_table(rows: list[ProposeBenchmarkRow]) -> str:
    by_key: dict[tuple[str, str], list[ProposeBenchmarkRow]] = {}
    for r in rows:
        by_key.setdefault((r.preset, r.scenario), []).append(r)

    lines = [
        "| preset | scenario | valid | clearance_mean | clearance_min | graph+ | Δnodes | time_s |",
        "|--------|----------|-------|----------------|---------------|--------|--------|--------|",
    ]
    for (preset, scenario), agg in sorted(by_key.items()):
        valid = float(np.mean([a.valid_count for a in agg]))
        c_mean = float(np.mean([a.top_clearance_mean for a in agg]))
        c_min = float(np.mean([a.top_clearance_min for a in agg]))
        nodes = float(np.mean([a.graph_nodes for a in agg]))
        delta = float(np.mean([a.graph_nodes_vs_random for a in agg]))
        t = float(np.mean([a.propose_time_s for a in agg]))
        lines.append(
            f"| {preset} | {scenario} | {valid:.1f} | {c_mean:.4f} | {c_min:.4f} | "
            f"{nodes:.1f} | {delta:+.1f} | {t:.3f} |"
        )
    return "\n".join(lines)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--presets",
        nargs="*",
        default=None,
        help=f"Config presets: {', '.join(PROPOSE_BENCHMARK_PRESETS)}",
    )
    parser.add_argument(
        "--scenarios",
        nargs="*",
        default=["empty_base", "partial_pack", "two_clusters"],
    )
    parser.add_argument("--seeds", type=int, nargs="*", default=list(range(5)))
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("docs/propose_benchmark_results.txt"),
    )
    args = parser.parse_args()

    base_cfg = BuildGraphConfig()
    presets = args.presets or list(PROPOSE_BENCHMARK_PRESETS.keys())

    rows: list[ProposeBenchmarkRow] = []
    for name in presets:
        if name not in PROPOSE_BENCHMARK_PRESETS:
            print(f"Unknown preset: {name}", file=sys.stderr)
            sys.exit(1)
    for scenario in args.scenarios:
        for preset_name in presets:
            for seed in args.seeds:
                rows.append(
                    run_propose_benchmark_preset(preset_name, scenario, seed, base_cfg)
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
