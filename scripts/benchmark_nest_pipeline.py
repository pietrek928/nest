#!/usr/bin/env python3
"""End-to-end nest pipeline benchmark: propose presets × DFS modes × mini build loop."""

import argparse
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
from shapely.geometry import Point, Polygon

from nest_graph.board import board_context_from_geometry
from nest_graph.build_graph import (
    NestState,
    _build_transform_batch,
    _make_initial_rule_sets,
    active_rule_set,
    apply_dfs_refinement,
    improve_rules,
    make_polygon_graph,
)
from nest_graph.config import (
    BuildGraphConfig,
    ProposeConfig,
    SamplingConfig,
    SelectionConfig,
    score_rules_options,
)
from nest_graph.elem_graph import nest_by_graph, score_elems, selection_is_independent
from nest_graph.utils import transform_poly

DFS_MODES = (
    "merged_single_pass",
    "merged_loose_tight",
    "merged_loose_finalize_end",
    "merged_loose_tight_finalize_end",
)

STRUCTURED_SAMPLING: dict[str, Any] = {
    "random_per_iter": 128,
    "random_per_iter_when_proposed": 48,
    "structured_jitter_per_proposal": 8,
    "structured_jitter_scale": (0.06, 0.06, 0.35),
    "initial_random": 256,
    "max_transforms_per_group": 900,
    "shuffle_passes": 2,
    "shuffle_per_pass": 32,
}


def _propose_preset(name: str, **overrides: Any) -> ProposeConfig:
    shipped: dict[str, Any] = ProposeConfig().model_dump()
    shipped_prev = dict(shipped)
    shipped_prev.update({
        "max_proposals": 20,
        "candidate_pool": 32,
        "erosion_num_angles": 6,
        "raycast_num_rays": 8,
        "raycast_num_angles": 6,
        "use_contact_clearance_hybrid": False,
        "use_stratified_contact_trim": False,
    })
    presets: dict[str, dict[str, Any]] = {
        "shipped_prev": shipped_prev,
        "legacy": {
            "max_proposals": 20,
            "use_free_region_search": False,
            "ranking_mode": "legacy",
            "smart_push_target": False,
            "use_ribbon_seeds": False,
            "use_group_edge_seeds": False,
            "use_border_focus": False,
            "use_border_edge_seeds": False,
            "use_contact_ranking": False,
        },
        "clearance": {
            **shipped,
            "use_group_edge_seeds": False,
            "use_border_focus": False,
            "use_border_edge_seeds": False,
            "use_contact_ranking": False,
            "ranking_mode": "clearance",
        },
        "shipped": shipped,
    }
    base = dict(presets[name])
    base.update(overrides)
    return ProposeConfig(**base)


PROPOSE_PRESET_NAMES = ("shipped_prev", "shipped")


@dataclass
class PipelineRow:
    propose: str
    dfs_mode: str
    seed: int
    iters: int
    parts_final: int
    graph_nodes_final: int
    score_sum_final: float
    border_min_final: float
    time_s: float


def _mean_border_standoff(
    board: Polygon,
    sheet: Polygon,
    polys: list,
    selected: list[int],
    min_dist: float,
) -> float:
    if not selected:
        return float("nan")
    errs = []
    for i in selected:
        placed = polys[i]
        err = abs(float(placed.distance(sheet.exterior)) - min_dist)
        errs.append(err)
    return float(min(errs)) if errs else float("nan")


def run_pipeline(
    cfg: BuildGraphConfig,
    *,
    seed: int,
    dfs_mode: str,
    n_iters: int,
) -> PipelineRow:
    if dfs_mode not in DFS_MODES:
        raise ValueError(f"unknown dfs_mode {dfs_mode!r}")

    rng = np.random.default_rng(seed)
    sc = cfg.sampling
    sel = cfg.selection
    gc = cfg.graph

    p_board = cfg.rules.board_polygon()
    p_sheet = cfg.rules.board_sheet_polygon()
    sheet, _ = board_context_from_geometry(p_board)
    p1 = cfg.rules.rect_polygon()
    p2 = cfg.rules.tri_polygon()
    parts = [(p1, 0), (p2, 1)]
    min_dist = cfg.board_min_dist()

    selected_t = (
        rng.uniform(-1, 1, (sc.initial_random, 3)) * sc.transform_scale,
        rng.uniform(-1, 1, (sc.initial_random, 3)) * sc.transform_scale,
    )
    history = (np.zeros((1, 3)), np.zeros((1, 3)))
    rule_sets = _make_initial_rule_sets(cfg)
    nest_state: NestState | None = None
    graphs: list = []

    t0 = time.perf_counter()
    parts_final = 0
    graph_nodes_final = 0
    score_sum_final = 0.0
    selected_polys: list[int] = []
    polys: list = []

    for _ in range(n_iters):
        selected_t = _build_transform_batch(
            cfg,
            selected_t,
            history,
            rng,
            board=p_board,
            parts=parts,
            nest_state=nest_state,
        )
        graph, polys, group_id, transform = make_polygon_graph(
            p_board,
            [(p1, selected_t[0]), (p2, selected_t[1])],
            min_dist=min_dist,
            epsilon_ratio=cfg.propose.placement_clearance_epsilon_ratio,
        )
        graphs.append(graph)
        graphs = graphs[-gc.graphs_window :]
        for round_idx in range(sel.improve_rules_rounds):
            rule_sets = improve_rules(
                graphs,
                rule_sets,
                sel.rules_kept,
                p_board,
                mutation_presets=cfg.rules.mutation_presets(),
                rule_score_penalty=sel.rule_score_penalty,
                elite_count=sel.improve_rules_elite_count,
                seed=int(rng.integers(0, 2**31)) + round_idx,
                score_options=score_rules_options(sel),
                max_rules_per_set=cfg.rules.max_rules_per_set,
            )
        active_rules = active_rule_set(rule_sets)
        scores = score_elems(graph, active_rules)
        selected = list(nest_by_graph(graph, rule_sets[: sel.nest_rule_sets_used])[0])
        sel_run = sel.model_copy(update={"dfs_mode": dfs_mode})
        _, selected_polys, score_sum_final = apply_dfs_refinement(
            graph,
            active_rules,
            selected,
            scores,
            selection=sel_run,
        )
        if not selection_is_independent(graph, selected_polys):
            raise AssertionError(
                f"overlap after refine propose={cfg.propose} dfs={dfs_mode} seed={seed}",
            )
        parts_final = len(selected_polys)
        graph_nodes_final = len(polys)

        selected_t = ([], [])
        for i in selected_polys:
            gi = group_id[i]
            selected_t[gi].append(transform[i])
        selected_t = tuple(np.array(t) for t in selected_t)
        nest_state = NestState(
            polys=polys,
            group_id=group_id,
            transform=transform,
            selected_indices=list(selected_polys),
        )

    border_min = _mean_border_standoff(
        p_board, sheet, polys, selected_polys, min_dist,
    )
    return PipelineRow(
        propose="",
        dfs_mode=dfs_mode,
        seed=seed,
        iters=n_iters,
        parts_final=parts_final,
        graph_nodes_final=graph_nodes_final,
        score_sum_final=score_sum_final,
        border_min_final=border_min,
        time_s=time.perf_counter() - t0,
    )


def _build_cfg(
    propose_name: str,
    dfs_mode: str,
    *,
    selection_overrides: dict[str, Any] | None = None,
) -> BuildGraphConfig:
    sel_data = BuildGraphConfig().selection.model_dump()
    sel_data["dfs_mode"] = dfs_mode
    if selection_overrides:
        sel_data.update(selection_overrides)
    return BuildGraphConfig(
        sampling=SamplingConfig(**STRUCTURED_SAMPLING),
        propose=_propose_preset(propose_name),
        selection=SelectionConfig(**sel_data),
    )


def _aggregate_table(rows: list[PipelineRow]) -> str:
    keys: dict[tuple[str, str], list[PipelineRow]] = {}
    for r in rows:
        keys.setdefault((r.propose, r.dfs_mode), []).append(r)

    lines = [
        "| propose | dfs_mode | parts_final | graph_nodes | border_err_min | score_sum | time_s |",
        "|---------|----------|-------------|-------------|----------------|-----------|--------|",
    ]
    ranked: list[tuple[float, str, str]] = []
    for (propose, dfs_mode), group in sorted(keys.items()):
        parts = float(np.mean([g.parts_final for g in group]))
        nodes = float(np.mean([g.graph_nodes_final for g in group]))
        border = float(np.nanmean([g.border_min_final for g in group]))
        score = float(np.mean([g.score_sum_final for g in group]))
        t = float(np.mean([g.time_s for g in group]))
        lines.append(
            f"| {propose} | {dfs_mode} | {parts:.1f} | {nodes:.0f} | {border:.4f} | {score:.2f} | {t:.2f} |",
        )
        ranked.append((parts, propose, dfs_mode))
    lines.append("")
    lines.append("## Ranking by mean parts_final")
    for parts, propose, dfs_mode in sorted(ranked, reverse=True):
        lines.append(f"- **{parts:.1f}** — `{propose}` + `{dfs_mode}`")
    return "\n".join(lines)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--propose",
        nargs="*",
        default=list(PROPOSE_PRESET_NAMES),
        choices=PROPOSE_PRESET_NAMES,
    )
    parser.add_argument(
        "--dfs-modes",
        nargs="*",
        default=list(DFS_MODES),
        choices=DFS_MODES,
    )
    parser.add_argument("--seeds", type=int, nargs="*", default=list(range(5)))
    parser.add_argument("--iters", type=int, default=3)
    parser.add_argument(
        "--compare-dfs-tuning",
        action="store_true",
        help="Also run legacy DFS budget (passes=4, tries=8, tight_finalize_end)",
    )
    args = parser.parse_args()

    rows: list[PipelineRow] = []
    configs: list[tuple[str, str, BuildGraphConfig]] = []
    for propose in args.propose:
        for dfs_mode in args.dfs_modes:
            configs.append((propose, dfs_mode, _build_cfg(propose, dfs_mode)))
            if args.compare_dfs_tuning and dfs_mode == "merged_loose_tight_finalize_end":
                configs.append((
                    propose,
                    "legacy_dfs_budget",
                    _build_cfg(
                        propose,
                        "merged_loose_tight_finalize_end",
                        selection_overrides={
                            "dfs_passes": 4,
                            "dfs_max_tries": 8,
                            "dfs_refine_max_passes": 32,
                            "dfs_finalize_repair_passes": 8,
                        },
                    ),
                ))

    for propose, dfs_mode, cfg in configs:
        run_mode = cfg.selection.dfs_mode
        for seed in args.seeds:
            row = run_pipeline(cfg, seed=seed, dfs_mode=run_mode, n_iters=args.iters)
            row.propose = propose
            row.dfs_mode = dfs_mode
            rows.append(row)
            print(
                f"propose={propose} dfs={dfs_mode} seed={seed}: "
                f"parts={row.parts_final} nodes={row.graph_nodes_final} "
                f"border_err={row.border_min_final:.4f} t={row.time_s:.2f}s",
            )

    table = _aggregate_table(rows)
    print("\n" + table)

    out = Path(__file__).resolve().parents[1] / "docs" / "nest_pipeline_benchmark_results.txt"
    out.write_text(
        f"# Nest pipeline benchmark {time.strftime('%Y-%m-%d %H:%M')}\n"
        f"# seeds={list(args.seeds)} iters={args.iters}\n\n{table}\n",
        encoding="utf-8",
    )
    print(f"\nWrote {out}")


if __name__ == "__main__":
    main()
