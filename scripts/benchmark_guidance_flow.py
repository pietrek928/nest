#!/usr/bin/env python3
"""Benchmark propose/guidance flows and rank for shipped defaults."""

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
from nest_graph.build_graph import (
    NestState,
    _append_selection_window,
    _build_transform_batch,
    _make_initial_rule_sets,
    active_rule_set,
    apply_dfs_refinement,
    improve_rules,
    make_polygon_graph,
)
from nest_graph.config import BuildGraphConfig, ProposeConfig
from nest_graph.config import score_rules_options
from nest_graph.elem_graph import nest_by_graph, score_elems, selection_is_independent
from nest_graph.propose import (
    ProposeGeometry,
    base_shape_from_selection,
    obstacle_shape_for_propose,
    propose_coords_with_strategy,
    propositions_to_ndarray,
)
from nest_graph.utils import normalize_poly, transform_poly

# Flows to compare (production-relevant)
FLOW_PRESETS: dict[str, dict[str, Any]] = {
    "shipped_no_props": {
        "use_guidance_propositions": False,
        "use_ribbon_seeds": True,
        "use_border_focus": True,
        "use_border_edge_seeds": True,
        "use_group_edge_seeds": True,
        "use_contact_ranking": True,
        "use_contact_clearance_hybrid": True,
        "use_stratified_contact_trim": True,
        "border_focus_ranking": True,
        "max_proposals": 24,
        "candidate_pool": 48,
    },
    "shipped_props": {
        "use_guidance_propositions": True,
        "use_ribbon_seeds": True,
        "use_border_focus": True,
        "use_border_edge_seeds": True,
        "use_group_edge_seeds": True,
        "use_contact_ranking": True,
        "use_contact_clearance_hybrid": True,
        "use_stratified_contact_trim": True,
        "border_focus_ranking": True,
        "max_proposals": 24,
        "candidate_pool": 48,
        "guidance_use_tight_packing": True,
        "guidance_use_corner_alignment": True,
        "guidance_enable_grid": False,
        "guidance_max_propositions": 6,
    },
    "props_no_grid": {
        "use_guidance_propositions": True,
        "guidance_enable_grid": False,
        "guidance_use_corner_alignment": True,
        "max_proposals": 24,
        "candidate_pool": 48,
        "use_ribbon_seeds": True,
        "use_group_edge_seeds": True,
        "use_contact_ranking": True,
        "use_contact_clearance_hybrid": True,
    },
    "props_floor_walk": {
        "use_guidance_propositions": True,
        "guidance_enable_grid": True,
        "guidance_use_corner_alignment": True,
        "max_proposals": 24,
        "candidate_pool": 48,
        "use_ribbon_seeds": True,
        "use_group_edge_seeds": True,
        "use_contact_ranking": True,
        "use_contact_clearance_hybrid": True,
    },
    "props_no_corner": {
        "use_guidance_propositions": True,
        "guidance_use_corner_alignment": False,
        "guidance_enable_grid": False,
        "max_proposals": 24,
        "candidate_pool": 48,
        "use_ribbon_seeds": True,
        "use_group_edge_seeds": True,
        "use_contact_ranking": True,
        "use_contact_clearance_hybrid": True,
    },
    "props_no_tight_pack": {
        "use_guidance_propositions": True,
        "guidance_use_tight_packing": False,
        "guidance_use_corner_alignment": True,
        "max_proposals": 24,
        "candidate_pool": 48,
        "use_ribbon_seeds": True,
        "use_group_edge_seeds": True,
        "use_contact_ranking": True,
        "use_contact_clearance_hybrid": True,
    },
    "props_max3": {
        "use_guidance_propositions": True,
        "guidance_max_propositions": 3,
        "guidance_proposition_seed_count": 4,
        "max_proposals": 24,
        "candidate_pool": 48,
        "use_ribbon_seeds": True,
        "use_group_edge_seeds": True,
        "use_contact_ranking": True,
        "use_contact_clearance_hybrid": True,
    },
}


def _propose_cfg(name: str) -> ProposeConfig:
    base = ProposeConfig().model_dump()
    base.update(FLOW_PRESETS[name])
    return ProposeConfig(**base)


def _triangle_board() -> Polygon:
    return Polygon([(0, 0), (1.2, 0), (0, 1.1)])


def _rect_poly():
    return normalize_poly(Polygon([(0, 0), (0.1, 0), (0.1, 0.1), (0, 0.1)]))


def _tri_poly():
    return normalize_poly(Polygon([(0, 0), (0.15, 0), (0, 0.07)]))


@dataclass
class ProposeRow:
    flow: str
    scenario: str
    seed: int
    valid: int
    clearance_min: float
    border_err_min: float
    time_s: float


@dataclass
class PipelineRow:
    flow: str
    seed: int
    parts: int
    border_err: float
    time_s: float


def _partial_pack(seed: int):
    board = _triangle_board()
    rect, tri = _rect_poly(), _tri_poly()
    rng = np.random.default_rng(seed)
    t = np.array([0.35, 0.25, rng.uniform(0, 0.5)])
    polys = [transform_poly(rect, t)]
    return board, tri, polys, [0]


def _empty_base(seed: int):
    return _triangle_board(), _rect_poly(), [], []


def _two_clusters(seed: int):
    board = _triangle_board()
    rect, tri = _rect_poly(), _tri_poly()
    rng = np.random.default_rng(seed)
    polys = [
        transform_poly(rect, np.array([0.08, 0.08, rng.uniform(0, 0.3)])),
        transform_poly(rect, np.array([0.55, 0.55, rng.uniform(0, 0.3)])),
    ]
    return board, tri, polys, [0, 1]


def run_propose(flow: str, scenario: str, seed: int) -> ProposeRow:
    cfg = BuildGraphConfig()
    propose = _propose_cfg(flow)
    min_dist = cfg.board_min_dist()
    eps = propose.placement_clearance_epsilon_ratio

    if scenario == "empty_base":
        board, part, polys, idx = _empty_base(seed)
        base = Polygon()
    elif scenario == "partial_pack":
        board, part, polys, idx = _partial_pack(seed)
        base = base_shape_from_selection(polys, idx)
    elif scenario == "two_clusters":
        board, part, polys, idx = _two_clusters(seed)
        base = base_shape_from_selection(polys, idx)
    else:
        raise ValueError(scenario)

    placed = [polys[i] for i in idx]
    obstacle = obstacle_shape_for_propose(placed, part, min_dist)
    push = obstacle.centroid if not obstacle.is_empty else board.centroid
    sheet, _ = board_context_from_geometry(board)

    t0 = time.perf_counter()
    coords = propose_coords_with_strategy(
        obstacle, part, board, propose,
        min_dist=min_dist, pt_push=push,
    )
    elapsed = time.perf_counter() - t0

    geom = ProposeGeometry(
        board, obstacle, part, min_dist,
        epsilon_ratio=eps, propose_cfg=propose,
    )
    clearances: list[float] = []
    border_errs: list[float] = []
    valid = 0
    for c in coords:
        placed_g = geom.placed_at(c)
        if not geom.valid(placed_g, push, (c[0], c[1])):
            continue
        valid += 1
        g = geom.placement_guidance(placed_g, (c[0], c[1]), push)
        if not g.is_penetrating:
            clearances.append(float(g.clearance))
        shapely_placed = transform_poly(part, c)
        border_errs.append(abs(float(shapely_placed.distance(sheet.exterior)) - min_dist))

    return ProposeRow(
        flow=flow,
        scenario=scenario,
        seed=seed,
        valid=valid,
        clearance_min=min(clearances) if clearances else 0.0,
        border_err_min=min(border_errs) if border_errs else float("inf"),
        time_s=elapsed,
    )


def run_pipeline(flow: str, seed: int, n_iters: int = 3) -> PipelineRow:
    cfg = BuildGraphConfig.benchmark_aligned(seed=seed)
    cfg = cfg.model_copy(update={"propose": _propose_cfg(flow)})
    rng = np.random.default_rng(seed)
    sc, sel, gc = cfg.sampling, cfg.selection, cfg.graph
    p_board = cfg.rules.board_polygon()
    sheet, _ = board_context_from_geometry(p_board)
    p1, p2 = cfg.rules.rect_polygon(), cfg.rules.tri_polygon()
    parts = [(p1, 0), (p2, 1)]
    min_dist = cfg.board_min_dist()

    selected_t = (
        rng.uniform(-1, 1, (sc.initial_random, 3)) * sc.transform_scale,
        rng.uniform(-1, 1, (sc.initial_random, 3)) * sc.transform_scale,
    )
    rule_sets = _make_initial_rule_sets(cfg)
    nest_state = None
    graphs: list = []
    selection_window: list = []
    selected_polys: list[int] = []
    polys: list = []

    t0 = time.perf_counter()
    for _ in range(n_iters):
        selected_t = _build_transform_batch(
            cfg, selected_t, (np.zeros((1, 3)), np.zeros((1, 3))), rng,
            board=p_board, parts=parts, nest_state=nest_state,
            selection_window=selection_window,
        )
        graph, polys, group_id, transform = make_polygon_graph(
            p_board,
            [(p1, selected_t[0]), (p2, selected_t[1])],
            min_dist=min_dist,
            epsilon_ratio=cfg.propose.placement_clearance_epsilon_ratio,
        )
        graphs = graphs[-gc.graphs_window:]
        graphs.append(graph)
        for round_idx in range(sel.improve_rules_rounds):
            rule_sets = improve_rules(
                graphs, rule_sets, sel.rules_kept, p_board,
                mutation_presets=cfg.rules.mutation_presets(),
                rule_score_penalty=sel.rule_score_penalty,
                elite_count=sel.improve_rules_elite_count,
                seed=int(rng.integers(0, 2**31)) + round_idx,
                score_options=score_rules_options(sel),
                max_rules_per_set=cfg.rules.max_rules_per_set,
            )
        active = active_rule_set(rule_sets)
        scores = score_elems(graph, active)
        selected = list(nest_by_graph(graph, rule_sets[: sel.nest_rule_sets_used])[0])
        _, selected_polys, _ = apply_dfs_refinement(
            graph, active, selected, scores, selection=sel,
        )
        assert selection_is_independent(graph, selected_polys)
        selected_t = ([], [])
        for i in selected_polys:
            gi = group_id[i]
            selected_t[gi].append(transform[i])
        selected_t = tuple(np.array(t) for t in selected_t)
        _append_selection_window(selection_window, selected_t, gc.graphs_window)
        nest_state = NestState(polys, group_id, transform, list(selected_polys))

    border_errs = [
        abs(float(polys[i].distance(sheet.exterior)) - min_dist)
        for i in selected_polys
    ]
    return PipelineRow(
        flow=flow,
        seed=seed,
        parts=len(selected_polys),
        border_err=float(min(border_errs)) if border_errs else float("nan"),
        time_s=time.perf_counter() - t0,
    )


def _aggregate_propose(rows: list[ProposeRow]) -> dict[str, dict]:
    out: dict[str, dict] = {}
    for flow in FLOW_PRESETS:
        sub = [r for r in rows if r.flow == flow]
        if not sub:
            continue
        pack = [r for r in sub if r.scenario in ("partial_pack", "two_clusters")]
        empty = [r for r in sub if r.scenario == "empty_base"]
        out[flow] = {
            "pack_clearance_min": float(np.mean([r.clearance_min for r in pack])) if pack else 0.0,
            "pack_valid": float(np.mean([r.valid for r in pack])) if pack else 0.0,
            "empty_border_err": float(np.mean([r.border_err_min for r in empty])) if empty else 999.0,
            "time_s": float(np.mean([r.time_s for r in sub])),
        }
    return out


def _aggregate_pipeline(rows: list[PipelineRow]) -> dict[str, dict]:
    out: dict[str, dict] = {}
    for flow in FLOW_PRESETS:
        sub = [r for r in rows if r.flow == flow]
        if not sub:
            continue
        out[flow] = {
            "parts_mean": float(np.mean([r.parts for r in sub])),
            "parts_min": float(np.min([r.parts for r in sub])),
            "border_err": float(np.mean([r.border_err for r in sub])),
            "time_s": float(np.mean([r.time_s for r in sub])),
        }
    return out


def _score_flow(prop: dict, pipe: dict) -> float:
    """Higher is better. Parts dominate; lower pack clearance + border err better."""
    parts = pipe.get("parts_mean", 0.0)
    pack_c = prop.get("pack_clearance_min", 1.0)
    border = prop.get("empty_border_err", 1.0)
    time_penalty = 0.02 * (prop.get("time_s", 0.0) + pipe.get("time_s", 0.0))
    # lower clearance on pack = tighter (good): reward small clearance
    tight_bonus = max(0.0, 0.15 - pack_c) * 20.0
    border_bonus = max(0.0, 0.02 - border) * 50.0
    return parts * 10.0 + tight_bonus + border_bonus - time_penalty


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seeds", type=int, nargs="*", default=list(range(10)))
    parser.add_argument("--pipeline-iters", type=int, default=3)
    parser.add_argument("--out", type=Path, default=Path("docs/guidance_flow_benchmark.txt"))
    args = parser.parse_args()

    flows = list(FLOW_PRESETS.keys())
    scenarios = ["empty_base", "partial_pack", "two_clusters"]

    propose_rows: list[ProposeRow] = []
    for flow in flows:
        for scenario in scenarios:
            for seed in args.seeds:
                propose_rows.append(run_propose(flow, scenario, seed))
                print(f"propose {flow} {scenario} seed={seed} ok", file=sys.stderr)

    pipe_rows: list[PipelineRow] = []
    for flow in flows:
        for seed in args.seeds:
            pipe_rows.append(run_pipeline(flow, seed, args.pipeline_iters))
            print(f"pipeline {flow} seed={seed} parts={pipe_rows[-1].parts}", file=sys.stderr)

    prop_agg = _aggregate_propose(propose_rows)
    pipe_agg = _aggregate_pipeline(pipe_rows)

    ranked = []
    for flow in flows:
        p, pi = prop_agg.get(flow, {}), pipe_agg.get(flow, {})
        ranked.append((flow, _score_flow(p, pi), p, pi))
    ranked.sort(key=lambda x: x[1], reverse=True)
    winner = ranked[0][0]

    lines = [
        "# Guidance flow benchmark",
        f"seeds={list(args.seeds)} pipeline_iters={args.pipeline_iters}",
        "",
        "## Ranked flows (composite score: parts + tight pack + border - time)",
        "",
        "| rank | flow | score | parts_mean | pack_clear_min | empty_border | propose_s | pipeline_s |",
        "|------|------|-------|------------|----------------|--------------|-----------|------------|",
    ]
    for i, (flow, score, p, pi) in enumerate(ranked, 1):
        lines.append(
            f"| {i} | {flow} | {score:.2f} | {pi.get('parts_mean', 0):.1f} | "
            f"{p.get('pack_clearance_min', 0):.4f} | {p.get('empty_border_err', 0):.4f} | "
            f"{p.get('time_s', 0):.2f} | {pi.get('time_s', 0):.1f} |"
        )
    lines.extend(["", f"**Recommended flow: `{winner}`**", ""])
    lines.append("## Propose detail by scenario")
    lines.append("")
    lines.append("| flow | scenario | valid | clearance_min | border_err | time_s |")
    lines.append("|------|----------|-------|---------------|------------|--------|")
    by_fs: dict[tuple[str, str], list[ProposeRow]] = {}
    for r in propose_rows:
        by_fs.setdefault((r.flow, r.scenario), []).append(r)
    for (flow, scenario), agg in sorted(by_fs.items()):
        lines.append(
            f"| {flow} | {scenario} | {np.mean([a.valid for a in agg]):.1f} | "
            f"{np.mean([a.clearance_min for a in agg]):.4f} | "
            f"{np.mean([a.border_err_min for a in agg]):.4f} | "
            f"{np.mean([a.time_s for a in agg]):.3f} |"
        )

    text = "\n".join(lines) + "\n"
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(text)
    sidecar = args.out.with_suffix(".json")
    sidecar.write_text(json.dumps({
        "winner": winner,
        "ranked": [{"flow": f, "score": s} for f, s, _, _ in ranked],
        "propose": [asdict(r) for r in propose_rows],
        "pipeline": [asdict(r) for r in pipe_rows],
    }, indent=2))
    print(text)
    print(f"\nWrote {args.out} and {sidecar}", file=sys.stderr)
    print(f"WINNER: {winner}", file=sys.stderr)


if __name__ == "__main__":
    main()
