#!/usr/bin/env python3
"""OFAT densify factor sweep → early (+ late finalists) JSONL."""

import argparse
import itertools
import json
import subprocess
import sys
from pathlib import Path

import yaml

from scripts.tune.cfg_overrides import cfg_sig


def _load_factors(path: Path) -> dict:
    return yaml.safe_load(path.read_text()) or {}


def _ofat_cfgs(ofat: dict) -> list[list[str]]:
    """One override list per OFAT cell (including baselines once per axis mid)."""
    rows: list[list[str]] = []
    seen: set[str] = set()
    # Always include empty (shipped) once.
    rows.append([])
    seen.add(cfg_sig({}))
    for axis, values in (ofat or {}).items():
        for v in values:
            specs = [f"{axis}={v}"]
            sig = cfg_sig({axis: v})
            if sig in seen:
                continue
            seen.add(sig)
            rows.append(specs)
    return rows


def _cartesian_cfgs(axes: dict) -> list[list[str]]:
    if not axes:
        return []
    keys = list(axes.keys())
    vals = [list(axes[k]) for k in keys]
    rows: list[list[str]] = []
    for combo in itertools.product(*vals):
        rows.append([f"{k}={v}" for k, v in zip(keys, combo)])
    return rows


def _run(cmd: list[str]) -> int:
    print("+", " ".join(cmd), flush=True)
    return int(subprocess.call(cmd))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--factors",
        type=Path,
        default=Path("scripts/tune/densify_factors.yaml"),
    )
    parser.add_argument(
        "--jsonl",
        type=Path,
        default=Path("artifacts/tune/decision_matrix.jsonl"),
    )
    parser.add_argument("--max-runs", type=int, default=None)
    parser.add_argument("--force", action="store_true")
    parser.add_argument(
        "--late-only-top",
        type=int,
        default=None,
        help="Override late_top_m; 0 skips late",
    )
    parser.add_argument(
        "--skip-late",
        action="store_true",
        help="Early OFAT only",
    )
    parser.add_argument(
        "--analyze-first",
        action="store_true",
        help="After early, run analyze to pick late finalists from JSONL",
    )
    args = parser.parse_args()

    factors = _load_factors(args.factors)
    tag = str(factors.get("tag") or "void_fill")
    seed = int(factors.get("seed") or 0)
    propose = str(factors.get("propose") or "shipped")
    max_early = int(args.max_runs or factors.get("max_early_runs") or 24)
    late_m = int(
        args.late_only_top
        if args.late_only_top is not None
        else factors.get("late_top_m") or 4
    )
    ckpt = str(
        factors.get("checkpoint")
        or "artifacts/checkpoints/void_fill_seed0_iter20.npz"
    )
    extra = int(factors.get("extra_iters") or 8)

    early_rows = _ofat_cfgs(factors.get("ofat") or {})
    early_rows.extend(_cartesian_cfgs(factors.get("finalist_cartesian") or {}))
    if len(early_rows) > max_early and not args.force:
        print(
            f"Refusing {len(early_rows)} early runs (max={max_early}); "
            f"pass --force or raise max_early_runs",
            file=sys.stderr,
        )
        sys.exit(2)

    py = sys.executable
    for specs in early_rows:
        cmd = [
            py,
            "scripts/benchmark_pipeline.py",
            "--tags",
            tag,
            "--seeds",
            str(seed),
            "--propose",
            propose,
            "--jsonl",
            str(args.jsonl),
        ]
        if specs:
            cmd.append("--cfg")
            cmd.extend(specs)
        rc = _run(cmd)
        if rc != 0:
            sys.exit(rc)

    if args.skip_late or late_m <= 0:
        return

    if args.analyze_first:
        report = Path("artifacts/tune/analyze_report.json")
        rc = _run(
            [
                py,
                "scripts/tune/analyze_decision_matrix.py",
                "--jsonl",
                str(args.jsonl),
                "--report",
                str(report),
                "--presets-out",
                "artifacts/tune/densify_presets.yaml",
                "--top-k",
                str(max(late_m, 3)),
            ]
        )
        if rc != 0:
            sys.exit(rc)

    # Late finalists: unique early cfg_sig by area descending.
    early: list[dict] = []
    if args.jsonl.is_file():
        for line in args.jsonl.read_text().splitlines():
            if not line.strip():
                continue
            row = json.loads(line)
            if row.get("phase") == "early":
                early.append(row)
    early.sort(
        key=lambda r: (
            float(r.get("area") or 0.0),
            int(r.get("parts") or 0),
        ),
        reverse=True,
    )
    seen_sig: set[str] = set()
    finalists: list[dict] = []
    for row in early:
        sig = str(row.get("cfg_sig") or "")
        if not sig or sig in seen_sig:
            continue
        seen_sig.add(sig)
        finalists.append(row)
        if len(finalists) >= late_m:
            break

    for row in finalists:
        specs = list(row.get("cfg_overrides") or [])
        cmd = [
            py,
            "scripts/benchmark_late_checkpoint.py",
            "--checkpoint",
            ckpt,
            "--extra-iters",
            str(extra),
            "--seed",
            str(seed),
            "--case",
            tag,
            "--jsonl",
            str(args.jsonl),
        ]
        if specs:
            cmd.append("--cfg")
            cmd.extend(specs)
        rc = _run(cmd)
        if rc != 0:
            sys.exit(rc)


if __name__ == "__main__":
    main()
