#!/usr/bin/env python3
"""A/B first-pass border pipeline: guidance refine on vs off (fixed seed)."""

import argparse
import copy
import time

import numpy as np

from nest_graph.build_graph import (
    _border_tightness_cost,
    _build_transform_batch,
    _first_pass_border_ring_selection,
    _first_pass_layered_selection,
    _guidance_border_refine,
    _make_initial_rule_sets,
    active_rule_set,
    make_polygon_graph,
    score_elems,
)
from nest_graph.config import BuildGraphConfig, SamplingConfig


def _coverage_pct(selected, group_id, part_areas, board_area):
    from nest_graph.build_graph import _selection_coverage_pct
    return _selection_coverage_pct(selected, group_id, part_areas, board_area)


def _run_pre_refine(cfg: BuildGraphConfig) -> dict:
    rng = cfg.apply_seed()
    sc = cfg.sampling
    p_board = cfg.rules.board_polygon()
    p1 = cfg.rules.rect_polygon()
    p2 = cfg.rules.tri_polygon()
    parts = [(p1, 0), (p2, 1)]
    board_area = p_board.area
    part_areas = (p1.area, p2.area)

    selected_t = (
        rng.uniform(-1, 1, (sc.initial_random, 3)) * sc.transform_scale,
        rng.uniform(-1, 1, (sc.initial_random, 3)) * sc.transform_scale,
    )
    history = (np.zeros((1, 3)), np.zeros((1, 3)))
    selection_window: list = []

    t0 = time.perf_counter()
    selected_t = _build_transform_batch(
        cfg, selected_t, history, rng,
        board=p_board, parts=parts, nest_state=None,
        selection_window=selection_window, first_pass=True,
    )
    graph, polys, group_id, transform = make_polygon_graph(
        p_board,
        [(p1, selected_t[0]), (p2, selected_t[1])],
        min_dist=cfg.board_min_dist(first_pass=True),
        epsilon_ratio=cfg.placement_epsilon_ratio(first_pass=True),
    )
    seed_rules = active_rule_set(_make_initial_rule_sets(cfg))
    scores = score_elems(graph, seed_rules)
    min_dist = cfg.board_min_dist(first_pass=True)
    ring = _first_pass_border_ring_selection(
        graph, polys, p_board, min_dist, scores,
    )
    _, polys2, gid2, tr2, selected = _first_pass_layered_selection(
        cfg, p_board, parts,
        graph=graph, p1=p1, p2=p2,
        selected_t=selected_t, history=history, rng=rng,
        selection_window=selection_window,
        polys=polys, group_id=group_id, transform=transform,
        phase1_selected=list(ring),
        rule_set=seed_rules, scores=scores,
        selection=cfg.selection,
        skip_guidance_refine=True,
    )
    pack_polys = [polys2[i] for i in selected]
    pack_gids = [gid2[i] for i in selected]
    pack_tr = [np.asarray(tr2[i], dtype=np.float64) for i in selected]
    pre_cost = _border_tightness_cost(pack_polys, p_board, min_dist)
    elapsed = time.perf_counter() - t0
    return {
        "pool": len(polys),
        "ring": len(ring),
        "count": len(selected),
        "cov": _coverage_pct(selected, gid2, part_areas, board_area),
        "tightness": pre_cost,
        "time_s": elapsed,
        "board": p_board,
        "parts": parts,
        "min_dist": min_dist,
        "pack_polys": pack_polys,
        "pack_gids": pack_gids,
        "pack_tr": pack_tr,
    }


def _apply_refine(
    cfg: BuildGraphConfig,
    pre: dict,
    *,
    refine_passes: int,
) -> dict:
    cfg = cfg.model_copy(deep=True)
    cfg.propose.first_pass_guidance_refine_passes = refine_passes
    polys = copy.deepcopy(pre["pack_polys"])
    gids = list(pre["pack_gids"])
    trs = [np.asarray(t, dtype=np.float64) for t in pre["pack_tr"]]
    t0 = time.perf_counter()
    refined_polys, refined_gids, refined_tr = _guidance_border_refine(
        cfg,
        pre["board"],
        pre["parts"],
        outline=pre["board"],
        pack_polys=polys,
        pack_gids=gids,
        pack_tr=trs,
    )
    cost = _border_tightness_cost(refined_polys, pre["board"], pre["min_dist"])
    return {
        "refine_passes": refine_passes,
        "count": len(refined_polys),
        "tightness": cost,
        "delta": cost - pre["tightness"],
        "time_s": time.perf_counter() - t0,
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument(
        "--augment-max", type=int, default=None,
        help="Override first_pass_sequential_augment_max (default: config value)",
    )
    args = parser.parse_args()
    base = BuildGraphConfig(sampling=SamplingConfig(seed=args.seed))
    if args.augment_max is not None:
        base.propose.first_pass_sequential_augment_max = args.augment_max

    print(f"seed={args.seed} augment_max={base.propose.first_pass_sequential_augment_max}")
    pre = _run_pre_refine(base)
    print(
        f"  pre-refine: ring={pre['ring']} count={pre['count']} "
        f"cov={pre['cov']:.1f}% pool={pre['pool']} "
        f"tightness={pre['tightness']:.4f} time={pre['time_s']:.1f}s"
    )
    for passes in (0, base.propose.first_pass_guidance_refine_passes):
        m = _apply_refine(base, pre, refine_passes=passes)
        print(
            f"  refine={m['refine_passes']:d}: count={m['count']} "
            f"tightness={m['tightness']:.4f} delta={m['delta']:+.4f} "
            f"time={m['time_s']:.1f}s"
        )


if __name__ == "__main__":
    main()
