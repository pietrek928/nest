"""Unit tests for densify decision-matrix helpers (Phase A)."""

import json
import subprocess
import sys
from pathlib import Path

from nest_graph.config import BuildGraphConfig
from scripts.tune.analyze_decision_matrix import _composite
from scripts.tune.cfg_overrides import (
    apply_cfg_overrides,
    cfg_sig,
    effective_propose_cfg_dict,
)


def test_apply_cfg_overrides_propose_and_selection():
    cfg = BuildGraphConfig()
    base_boost = float(cfg.propose.void_island_score_boost)
    out = apply_cfg_overrides(
        cfg,
        [
            "void_island_score_boost=32",
            "selection.dfs_passes=1",
        ],
    )
    assert out.propose.void_island_score_boost == 32.0
    assert out.selection.dfs_passes == 1
    assert cfg.propose.void_island_score_boost == base_boost


def test_cfg_sig_stable():
    a = {"void_island_score_boost": 64.0}
    b = {"void_island_score_boost": 64.0}
    assert cfg_sig(a) == cfg_sig(b)
    assert cfg_sig(a) != cfg_sig({"void_island_score_boost": 32.0})


def test_analyze_composite_indep_false():
    weights = {
        "area": {"weight": 1.0, "higher_is_better": True},
    }
    norms = {"area": (0.0, 1.0)}
    assert _composite({"area": 0.9, "independent_ok": False}, weights, norms) == float(
        "-inf"
    )
    assert _composite({"area": 0.9, "independent_ok": True}, weights, norms) > 0


def test_analyze_smoke_presets(tmp_path: Path):
    jsonl = tmp_path / "m.jsonl"
    rows = [
        {
            "phase": "early",
            "cfg_sig": "aaa",
            "cfg": {"void_island_score_boost": 64},
            "cfg_overrides": [],
            "area": 0.50,
            "parts": 40,
            "time_s": 10.0,
            "independent_ok": True,
        },
        {
            "phase": "early",
            "cfg_sig": "bbb",
            "cfg": {"void_island_score_boost": 96},
            "cfg_overrides": ["void_island_score_boost=96"],
            "area": 0.55,
            "parts": 45,
            "time_s": 12.0,
            "independent_ok": True,
        },
    ]
    jsonl.write_text("\n".join(json.dumps(r) for r in rows) + "\n")
    report = tmp_path / "report.json"
    presets = tmp_path / "presets.yaml"
    rc = subprocess.call(
        [
            sys.executable,
            "scripts/tune/analyze_decision_matrix.py",
            "--jsonl",
            str(jsonl),
            "--report",
            str(report),
            "--presets-out",
            str(presets),
            "--top-k",
            "2",
        ]
    )
    assert rc == 0
    assert report.is_file()
    assert presets.is_file()
    assert "baseline" in presets.read_text()


def test_effective_propose_cfg_dict_keys():
    cfg = BuildGraphConfig()
    d = effective_propose_cfg_dict(cfg)
    assert "void_island_score_boost" in d
    assert "motif_score_boost" in d
    assert "cascade_explorer_budget_scale" in d
