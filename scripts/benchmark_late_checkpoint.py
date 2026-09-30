#!/usr/bin/env python3
"""Late dual from frozen NestState checkpoint — JSONL rows (phase=late)."""

import argparse
import contextlib
import io
import json
import os
import re
import sys
from pathlib import Path

from nest_graph.build_graph import run_build_graph
from nest_graph.config import BuildGraphConfig
from scripts.tune.cfg_overrides import (
    apply_cfg_overrides,
    cfg_sig,
    effective_propose_cfg_dict,
    parse_cfg_value,
)

_LATE_DELTA_RE = re.compile(
    r"\[checkpoint\] late Δcov=([+-]?\d+(?:\.\d+)?)\s+Δparts=([+-]?\d+)"
    r".*final cov=(\d+(?:\.\d+)?)\s+parts=(\d+)"
    r".*ckpt cov=(\d+(?:\.\d+)?)\s+parts=(\d+)"
)

_COMPOSE_TELEM_KEYS = (
    ("compose_sz", r"compose_sz=(\d+)"),
    ("member_hits", r"member_hits=(\d+)"),
    ("mat_motif", r"mat_motif=(\d+)"),
    ("join_n", r"join_n=(\d+)"),
    ("cohorts_n", r"cohorts_n=(\d+)"),
    ("union_n", r"union_n=(\d+)"),
    ("seq_full", r"seq_full=(\d+)"),
    ("seq_clear", r"seq_clear=(\d+)"),
    ("compose_hold", r"compose_hold=(\d+)"),
    ("lock_n_compose", r"lock_n_compose=(\d+)"),
    ("lock_n_materialize", r"lock_n_materialize=(\d+)"),
    ("lattice_anchors_added", r"lattice_anchors_added=(\d+)"),
    ("lattice_anchors_kept", r"lattice_anchors_kept=(\d+)"),
    ("cluster_copy_emitted", r"cluster_copy_emitted=(\d+)"),
    ("cluster_copy_nest_n", r"cluster_copy_nest_n=(\d+)"),
    ("incumbent_hold", r"incumbent_hold=(\d+)"),
    ("void_override", r"void_override=(\d+)"),
    ("motif_adj_hits", r"motif_adj_hits=(\d+)"),
    ("hybrid_pick_adj_soft", r"hybrid_pick_adj_soft=(\d+)"),
)


def _parse_late_line(text: str) -> dict | None:
    for line in text.splitlines():
        m = _LATE_DELTA_RE.search(line)
        if not m:
            continue
        return {
            "delta_cov": float(m.group(1)),
            "delta_parts": int(m.group(2)),
            "area": float(m.group(3)),
            "parts": int(m.group(4)),
            "ckpt_cov": float(m.group(5)),
            "ckpt_parts": int(m.group(6)),
        }
    return None


def _parse_compose_funnel(text: str) -> dict:
    """Last-seen compose/lattice telem from void_leak / letter lines."""
    out: dict[str, int] = {}
    lat_m = None
    for line in text.splitlines():
        for key, pat in _COMPOSE_TELEM_KEYS:
            m = re.search(pat, line)
            if m:
                out[key] = int(m.group(1))
        lat = re.search(r"\blat=(\d+)/(\d+)\b", line)
        if lat:
            lat_m = lat
    if lat_m is not None:
        out.setdefault("lattice_anchors_added", int(lat_m.group(1)))
        out.setdefault("lattice_anchors_kept", int(lat_m.group(2)))
    return out


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--checkpoint",
        type=Path,
        default=Path("artifacts/checkpoints/void_fill_seed0_iter20.npz"),
    )
    parser.add_argument("--extra-iters", type=int, default=8)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument(
        "--cfg",
        nargs="*",
        default=[],
        help="Same ProposeConfig/SelectionConfig overrides as benchmark_pipeline",
    )
    parser.add_argument("--jsonl", type=Path, default=None)
    parser.add_argument("--case", type=str, default="void_fill")
    args = parser.parse_args()

    if not args.checkpoint.is_file():
        print(f"Missing checkpoint: {args.checkpoint}", file=sys.stderr)
        sys.exit(2)

    cfg = apply_cfg_overrides(BuildGraphConfig(), list(args.cfg or []))
    eff = effective_propose_cfg_dict(cfg)
    for spec in list(args.cfg or []):
        if "=" not in spec:
            continue
        key, raw = spec.split("=", 1)
        key = key.strip()
        if key.startswith("selection."):
            continue
        if key.startswith("propose."):
            key = key.split(".", 1)[1]
        eff[key] = parse_cfg_value(raw)
    sig = cfg_sig(eff)

    cfg = cfg.model_copy(
        update={
            "output": cfg.output.model_copy(
                update={
                    "resume_checkpoint": str(args.checkpoint),
                    "extra_iters": int(args.extra_iters),
                    "seed": int(args.seed),
                }
            )
        }
    )
    os.environ["NEST_SEED"] = str(int(args.seed))

    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        run_build_graph(cfg)
    text = buf.getvalue()
    print(text, end="" if text.endswith("\n") else "\n")

    parsed = _parse_late_line(text)
    if parsed is None:
        print("Failed to parse [checkpoint] late Δ line", file=sys.stderr)
        sys.exit(3)

    row = {
        "phase": "late",
        "case": str(args.case),
        "seed": int(args.seed),
        "cfg_sig": sig,
        "cfg": dict(eff),
        "cfg_overrides": list(args.cfg or []),
        "delta_cov": float(parsed["delta_cov"]),
        "delta_parts": int(parsed["delta_parts"]),
        "area": float(parsed["area"]),
        "parts": int(parsed["parts"]),
        "ckpt_cov": float(parsed["ckpt_cov"]),
        "ckpt_parts": int(parsed["ckpt_parts"]),
        "extra_iters": int(args.extra_iters),
        "independent_ok": True,
        "void_leak": _parse_compose_funnel(text),
    }
    print(
        f"late Δcov={row['delta_cov']:+.4f} Δparts={row['delta_parts']:+d} "
        f"cfg_sig={sig}"
    )
    funnel = row["void_leak"]
    if funnel:
        print(
            "compose_funnel "
            + " ".join(f"{k}={v}" for k, v in sorted(funnel.items()))
        )
    if args.jsonl is not None:
        args.jsonl.parent.mkdir(parents=True, exist_ok=True)
        with args.jsonl.open("a", encoding="utf-8") as fh:
            fh.write(json.dumps(row, default=str) + "\n")


if __name__ == "__main__":
    main()
