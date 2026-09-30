#!/usr/bin/env python3
"""Analyze early(+late) JSONL → composite, corr, RF+permutation, Pareto, presets."""

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np
import yaml

from scripts.tune.cfg_overrides import cfg_sig

DEFAULT_WEIGHTS = {
    "area": {"weight": 1.0, "higher_is_better": True},
    "parts": {"weight": 0.15, "higher_is_better": True},
    "time_s": {"weight": 0.05, "higher_is_better": False},
    "delta_cov": {"weight": 0.5, "higher_is_better": True},
    "delta_parts": {"weight": 0.1, "higher_is_better": True},
}


def _load_weights(path: Path | None) -> dict:
    if path is None or not path.is_file():
        return dict(DEFAULT_WEIGHTS)
    raw = yaml.safe_load(path.read_text()) or {}
    return raw.get("metrics") or raw or dict(DEFAULT_WEIGHTS)


def _load_jsonl(path: Path) -> list[dict]:
    rows: list[dict] = []
    if not path.is_file():
        return rows
    for line in path.read_text().splitlines():
        if line.strip():
            rows.append(json.loads(line))
    return rows


def _composite(row: dict, weights: dict, norms: dict) -> float:
    if not bool(row.get("independent_ok", True)):
        return float("-inf")
    score = 0.0
    for key, meta in weights.items():
        if key not in row and key not in ("delta_cov", "delta_parts", "time_s"):
            continue
        if key not in row:
            continue
        w = float(meta.get("weight") or 0.0)
        if w == 0.0:
            continue
        hib = bool(meta.get("higher_is_better", True))
        val = float(row[key])
        lo, hi = norms.get(key, (val, val))
        span = max(hi - lo, 1e-9)
        unit = (val - lo) / span
        if not hib:
            unit = 1.0 - unit
        score += w * unit
    return float(score)


def _norms(rows: list[dict], keys: list[str]) -> dict:
    out: dict = {}
    for key in keys:
        vals = [float(r[key]) for r in rows if key in r and r[key] is not None]
        if not vals:
            continue
        out[key] = (min(vals), max(vals))
    return out


def _spearman(xs: list[float], ys: list[float]) -> float:
    n = len(xs)
    if n < 3:
        return float("nan")
    rx = np.argsort(np.argsort(np.asarray(xs, dtype=float)))
    ry = np.argsort(np.argsort(np.asarray(ys, dtype=float)))
    return float(np.corrcoef(rx, ry)[0, 1])


def _pareto_mask(points: np.ndarray, maximize: list[bool]) -> np.ndarray:
    n = points.shape[0]
    keep = np.ones(n, dtype=bool)
    for i in range(n):
        if not keep[i]:
            continue
        for j in range(n):
            if i == j or not keep[j]:
                continue
            dom = True
            strict = False
            for d, hib in enumerate(maximize):
                a, b = points[i, d], points[j, d]
                if hib:
                    if b < a:
                        dom = False
                        break
                    if b > a:
                        strict = True
                else:
                    if b > a:
                        dom = False
                        break
                    if b < a:
                        strict = True
            if dom and strict:
                keep[i] = False
                break
    return keep


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--jsonl",
        type=Path,
        default=Path("artifacts/tune/decision_matrix.jsonl"),
    )
    parser.add_argument(
        "--weights",
        type=Path,
        default=Path("scripts/tune/matrix_weights.yaml"),
    )
    parser.add_argument(
        "--report",
        type=Path,
        default=Path("artifacts/tune/analyze_report.json"),
    )
    parser.add_argument(
        "--presets-out",
        type=Path,
        default=Path("artifacts/tune/densify_presets.yaml"),
    )
    parser.add_argument("--top-k", type=int, default=3)
    args = parser.parse_args()

    weights = _load_weights(args.weights)
    rows = _load_jsonl(args.jsonl)
    if not rows:
        print(f"No rows in {args.jsonl}", file=sys.stderr)
        sys.exit(2)

    # Join late onto early by cfg_sig when both present.
    late_by_sig = {
        str(r.get("cfg_sig")): r for r in rows if r.get("phase") == "late"
    }
    early = [r for r in rows if r.get("phase") == "early"]
    joined: list[dict] = []
    for r in early:
        j = dict(r)
        late = late_by_sig.get(str(r.get("cfg_sig")))
        if late:
            j["delta_cov"] = late.get("delta_cov")
            j["delta_parts"] = late.get("delta_parts")
            j["late_area"] = late.get("area")
            j["late_parts"] = late.get("parts")
        joined.append(j)
    if not joined:
        joined = list(rows)

    metric_keys = [k for k in weights if any(k in r for r in joined)]
    norms = _norms(joined, metric_keys)
    for r in joined:
        r["composite"] = _composite(r, weights, norms)

    # Factor columns from cfg dicts.
    factor_keys: list[str] = sorted(
        {
            k
            for r in joined
            for k in (r.get("cfg") or {})
            if isinstance((r.get("cfg") or {}).get(k), (int, float, bool))
        }
    )
    corr: dict[str, float] = {}
    for fk in factor_keys:
        xs = []
        ys = []
        for r in joined:
            if not math.isfinite(float(r.get("composite", float("nan")))):
                continue
            if fk not in (r.get("cfg") or {}):
                continue
            xs.append(float(r["cfg"][fk]))
            ys.append(float(r["composite"]))
        corr[fk] = _spearman(xs, ys) if len(xs) >= 3 else float("nan")

    importance: dict[str, float] = {}
    perm_importance: dict[str, float] = {}
    if factor_keys and len(joined) >= 4:
        try:
            from sklearn.ensemble import RandomForestRegressor
            from sklearn.inspection import permutation_importance

            X = np.asarray(
                [
                    [float((r.get("cfg") or {}).get(fk, 0.0) or 0.0) for fk in factor_keys]
                    for r in joined
                    if math.isfinite(float(r.get("composite", float("nan"))))
                ],
                dtype=float,
            )
            y_arr = np.asarray(
                [
                    float(r["composite"])
                    for r in joined
                    if math.isfinite(float(r.get("composite", float("nan"))))
                ],
                dtype=float,
            )
            if X.shape[0] >= 4 and np.unique(y_arr).size > 1:
                rf = RandomForestRegressor(
                    n_estimators=64, random_state=0, max_depth=3
                )
                rf.fit(X, y_arr)
                importance = {
                    fk: float(v) for fk, v in zip(factor_keys, rf.feature_importances_)
                }
                perm = permutation_importance(
                    rf, X, y_arr, n_repeats=8, random_state=0
                )
                perm_importance = {
                    fk: float(v)
                    for fk, v in zip(factor_keys, perm.importances_mean)
                }
        except Exception as exc:  # noqa: BLE001 — analyze must still emit presets
            importance = {"_error": str(exc)}

    # Pareto on area / time (and delta_cov when present).
    objs = ["area", "time_s"]
    if any("delta_cov" in r for r in joined):
        objs.append("delta_cov")
    pts = []
    idx = []
    for i, r in enumerate(joined):
        if not all(k in r for k in objs):
            continue
        pts.append([float(r[k]) for k in objs])
        idx.append(i)
    pareto_sigs: list[str] = []
    if pts:
        mask = _pareto_mask(
            np.asarray(pts, dtype=float),
            maximize=[True, False, True][: len(objs)],
        )
        for keep, i in zip(mask, idx):
            if keep:
                pareto_sigs.append(str(joined[i].get("cfg_sig")))

    ranked = sorted(
        [r for r in joined if math.isfinite(float(r.get("composite", float("nan"))))],
        key=lambda r: float(r["composite"]),
        reverse=True,
    )
    top_k = max(1, int(args.top_k))
    # Preset 0 = baseline (empty overrides); then top distinct non-baseline.
    presets = [{"id": 0, "name": "baseline", "overrides": {}, "online_ok": True}]
    used = {cfg_sig({})}
    for r in ranked:
        ov_map: dict = {}
        for spec in r.get("cfg_overrides") or []:
            if isinstance(spec, str) and "=" in spec:
                k, v = spec.split("=", 1)
                ov_map[k.strip()] = yaml.safe_load(v) if v.strip() else v
        if not ov_map:
            continue
        sig = str(r.get("cfg_sig") or cfg_sig(ov_map))
        if sig in used:
            continue
        used.add(sig)
        presets.append(
            {
                "id": len(presets),
                "name": f"top{len(presets)}",
                "overrides": ov_map,
                "composite": float(r["composite"]),
                "area": float(r.get("area") or 0.0),
                "cfg_sig": sig,
                "online_ok": False,
            }
        )
        if len(presets) >= top_k:
            break

    report = {
        "n_rows": len(rows),
        "n_joined": len(joined),
        "corr_spearman": corr,
        "rf_importance": importance,
        "permutation_importance": perm_importance,
        "pareto_cfg_sigs": pareto_sigs,
        "ranked": [
            {
                "cfg_sig": r.get("cfg_sig"),
                "composite": r.get("composite"),
                "area": r.get("area"),
                "parts": r.get("parts"),
                "delta_cov": r.get("delta_cov"),
                "cfg_overrides": r.get("cfg_overrides"),
            }
            for r in ranked[:12]
        ],
        "presets": presets,
    }
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2, default=str) + "\n")
    args.presets_out.parent.mkdir(parents=True, exist_ok=True)
    args.presets_out.write_text(
        yaml.safe_dump({"presets": presets}, sort_keys=False)
    )
    print(f"Wrote {args.report}")
    print(f"Wrote {args.presets_out} ({len(presets)} presets)")
    for p in presets:
        print(f"  preset {p['id']}: {p.get('overrides') or {}}")


if __name__ == "__main__":
    main()
