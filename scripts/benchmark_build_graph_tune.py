#!/usr/bin/env python3
"""Compare build-loop knobs (DFS, sampling, rules) on the demo nest pipeline."""

import argparse
import json
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import numpy as np

from nest_graph.config import BuildGraphConfig, SamplingConfig, SelectionConfig
from scripts.benchmark_nest_pipeline import run_pipeline


@dataclass
class TuneRow:
    label: str
    seed: int
    parts_final: int
    graph_nodes: int
    border_err: float
    score_sum: float
    time_s: float


def _run_label(
    label: str,
    cfg: BuildGraphConfig,
    *,
    dfs_mode: str,
    seeds: list[int],
    iters: int,
) -> list[TuneRow]:
    rows: list[TuneRow] = []
    for seed in seeds:
        r = run_pipeline(cfg, seed=seed, dfs_mode=dfs_mode, n_iters=iters)
        rows.append(
            TuneRow(
                label=label,
                seed=seed,
                parts_final=r.parts_final,
                graph_nodes=r.graph_nodes_final,
                border_err=r.border_min_final,
                score_sum=r.score_sum_final,
                time_s=r.time_s,
            )
        )
    return rows


def _aggregate_table(rows: list[TuneRow]) -> str:
    by: dict[str, list[TuneRow]] = {}
    for r in rows:
        by.setdefault(r.label, []).append(r)

    lines = [
        "| config | parts | nodes | border_err | score | time_s |",
        "|--------|-------|-------|------------|-------|--------|",
    ]
    ranked: list[tuple[float, float, str]] = []
    for label, group in sorted(by.items()):
        parts = float(np.mean([g.parts_final for g in group]))
        nodes = float(np.mean([g.graph_nodes for g in group]))
        border = float(np.mean([g.border_err for g in group]))
        score = float(np.mean([g.score_sum for g in group]))
        t = float(np.mean([g.time_s for g in group]))
        lines.append(
            f"| `{label}` | {parts:.1f} | {nodes:.0f} | {border:.4f} | {score:.2f} | {t:.1f} |",
        )
        ranked.append((parts, t, label))
    lines.append("")
    lines.append("## Ranking (parts_final)")
    for parts, t, label in sorted(ranked, reverse=True):
        lines.append(f"- **{parts:.1f}** parts, {t:.1f}s — `{label}`")
    return "\n".join(lines)


def _base_cfg(**overrides: Any) -> BuildGraphConfig:
    cfg = BuildGraphConfig()
    for key, val in overrides.items():
        if hasattr(cfg.sampling, key):
            setattr(cfg.sampling, key, val)
        elif hasattr(cfg.selection, key):
            setattr(cfg.selection, key, val)
        elif hasattr(cfg.propose, key):
            setattr(cfg.propose, key, val)
        else:
            raise KeyError(key)
    return cfg


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seeds", type=int, nargs="*", default=list(range(5)))
    parser.add_argument("--iters", type=int, default=3)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("docs/build_graph_tuning_results.txt"),
    )
    args = parser.parse_args()

    variants: list[tuple[str, BuildGraphConfig, str]] = []

    dfs_modes = (
        "merged_loose_tight",
        "merged_loose_tight_finalize_end",
        "merged_single_pass",
        "merged_loose_finalize_end",
    )
    for mode in dfs_modes:
        variants.append((f"dfs/{mode}", _base_cfg(), mode))

    winner_mode = "merged_loose_tight"
    for rounds in (2, 4):
        variants.append((
            f"rules/improve_{rounds}r",
            _base_cfg(improve_rules_rounds=rounds),
            winner_mode,
        ))
    for passes in (3, 4):
        variants.append((
            f"dfs_passes/{passes}",
            _base_cfg(dfs_passes=passes),
            winner_mode,
        ))
    for proposed in (48, 64):
        variants.append((
            f"sampling/rand_when_proposed_{proposed}",
            _base_cfg(random_per_iter_when_proposed=proposed),
            winner_mode,
        ))
    for jitter in (8, 12):
        variants.append((
            f"sampling/jitter_per_prop_{jitter}",
            _base_cfg(structured_jitter_per_proposal=jitter),
            winner_mode,
        ))

    rows: list[TuneRow] = []
    for label, cfg, dfs_mode in variants:
        batch = _run_label(label, cfg, dfs_mode=dfs_mode, seeds=args.seeds, iters=args.iters)
        rows.extend(batch)
        parts = float(np.mean([r.parts_final for r in batch]))
        print(f"{label}: parts={parts:.1f}")

    table = _aggregate_table(rows)
    print("\n" + table)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        f"# Build-graph tuning {time.strftime('%Y-%m-%d %H:%M')}\n"
        f"# seeds={list(args.seeds)} iters={args.iters}\n\n{table}\n\n"
        "## Raw rows\n\n",
        encoding="utf-8",
    )
    with args.output.open("a") as f:
        for r in rows:
            f.write(json.dumps(asdict(r)) + "\n")
    print(f"\nWrote {args.output}")


if __name__ == "__main__":
    main()
