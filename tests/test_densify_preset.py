"""Tests for densify preset apply + cheap key preset_id."""

from nest_graph.config import BuildGraphConfig
from nest_graph.pack.cache_key import cheap_pack_cache_key
from nest_graph.pack.densify_preset import (
    apply_densify_preset,
    densify_preset_ids,
    load_densify_presets,
)


def test_apply_densify_preset_model_copy():
    cfg = BuildGraphConfig()
    base = float(cfg.propose.void_island_score_boost)
    presets = [
        {"id": 0, "overrides": {}},
        {"id": 1, "overrides": {"void_island_score_boost": 96}},
    ]
    out = apply_densify_preset(cfg, 1, presets)
    assert out.propose.void_island_score_boost == 96.0
    assert cfg.propose.void_island_score_boost == base
    assert apply_densify_preset(cfg, 0, presets) is cfg or (
        apply_densify_preset(cfg, 0, presets).propose.void_island_score_boost == base
    )


def test_load_presets_always_has_baseline():
    presets = load_densify_presets("artifacts/tune/densify_presets.yaml")
    assert densify_preset_ids(presets)[0] == 0
    assert any(int(p["id"]) == 0 for p in presets)
    # v1: non-baseline lack online_ok → only id 0 for macros
    assert densify_preset_ids(presets) == [0]
    assert densify_preset_ids(
        [{"id": 0}, {"id": 1, "online_ok": True}]
    ) == [0, 1]


def test_cheap_key_includes_preset():
    action = type("A", (), {"motif_id": -1, "rule_id": 0, "preset_id": 2})()
    key = cheap_pack_cache_key("void_seek", action)
    assert key[-1] == 2
    assert cheap_pack_cache_key("void_seek", action) != cheap_pack_cache_key(
        "void_seek", type("B", (), {"motif_id": -1, "rule_id": 0, "preset_id": 0})()
    )
