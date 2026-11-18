"""Q264: cheap cache hit still feeds inject_cohorts_from_patterns (no DG mutate)."""

from unittest.mock import MagicMock

from nest_graph.pack.cheap import pack_execute_snapshot
from nest_graph.pack.cache_key import cheap_lock_fingerprint, cheap_pack_cache_key


def test_cache_hit_runs_inject_before_return(monkeypatch):
    calls: list[tuple] = []

    def fake_inject(pats, gid, tf, propose_stats):
        calls.append((list(pats), list(gid), list(tf), propose_stats))
        propose_stats["motif_graph_hit_n"] = int(
            propose_stats.get("motif_graph_hit_n", 0) or 0
        ) + 3
        propose_stats["motif_cohorts"] = [{"leader_gid": 0}]
        return 1

    monkeypatch.setattr(
        "nest_graph.pack.cheap.inject_cohorts_from_patterns",
        fake_inject,
    )

    action = type("A", (), {"motif_id": 1, "rule_id": 0, "preset_id": 0, "part_gid": 0})()
    locked = [0, 1]
    lock_fp = cheap_lock_fingerprint(locked)
    key = cheap_pack_cache_key(
        "void_seek",
        action,
        compose_sz=len(locked),
        cohort_sig=7,
        lock_fp=lock_fp,
        pack_preset_id=0,
    )
    cached_snap = MagicMock(name="BoardSnapshot")
    pack_cache = {
        "ready": True,
        "cheap_by_key": {key: cached_snap},
        "cheap_telem_by_key": {key: {}},
        "cheap_compose_by_key": {
            key: {
                "compose_group_id": [0, 1],
                "compose_transform": [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)],
                "compose_sel": [0, 1],
                "motif_locked": list(locked),
            }
        },
        "motif_locked": list(locked),
        "motif_cohort_sig": 7,
        "pack_preset_id": 0,
        "propose_stats": {},
        "cache_lookup_n": 0,
        "cache_hit_n": 0,
    }
    pats = [object()]
    out = pack_execute_snapshot(
        MagicMock(name="parent"),
        zone="void_seek",
        action=action,
        patterns=pats,
        pack_cache=pack_cache,
        rule_sets=[],
        sel=MagicMock(),
        cfg=MagicMock(),
        apply_dfs_fn=MagicMock(),
        native_geoms_fn=MagicMock(),
        coverage_pct_fn=MagicMock(),
    )
    assert out is cached_snap
    assert int(pack_cache["cache_hit_n"]) == 1
    assert calls, "hit path must call inject"
    assert calls[0][0] == pats
    assert pack_cache["propose_stats"]["motif_graph_hit_n"] == 3
    assert pack_cache["last_execute_telem"].get("cache_hit_inject") == 1
    assert pack_cache["last_execute_telem"].get("cache_hit") == 1


def test_cache_hit_empty_patterns_skips_inject(monkeypatch):
    monkeypatch.setattr(
        "nest_graph.pack.cheap.inject_cohorts_from_patterns",
        lambda *a, **k: (_ for _ in ()).throw(AssertionError("no inject")),
    )
    action = type("A", (), {"motif_id": 2, "rule_id": 0, "preset_id": 0, "part_gid": 0})()
    key = cheap_pack_cache_key("rim", action, compose_sz=0, cohort_sig=0, lock_fp=0)
    cached_snap = MagicMock(name="BoardSnapshot")
    pack_cache = {
        "ready": True,
        "cheap_by_key": {key: cached_snap},
        "cheap_telem_by_key": {key: {}},
        "cheap_compose_by_key": {
            key: {
                "compose_group_id": [0],
                "compose_transform": [(0.0, 0.0, 0.0)],
            }
        },
        "motif_locked": [],
        "motif_cohort_sig": 0,
        "pack_preset_id": 0,
        "propose_stats": {},
        "cache_lookup_n": 0,
        "cache_hit_n": 0,
    }
    out = pack_execute_snapshot(
        MagicMock(name="parent"),
        zone="rim",
        action=action,
        patterns=[],
        pack_cache=pack_cache,
        rule_sets=[],
        sel=MagicMock(),
        cfg=MagicMock(),
        apply_dfs_fn=MagicMock(),
        native_geoms_fn=MagicMock(),
        coverage_pct_fn=MagicMock(),
    )
    assert out is cached_snap
    assert "cache_hit_inject" not in (pack_cache.get("last_execute_telem") or {})
