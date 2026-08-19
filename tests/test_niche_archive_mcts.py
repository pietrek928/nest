"""Tests for MacroNicheArchive, Progressive Bias, Motif TTL floor, browse helpers."""

import math

import numpy as np

from nest_graph.decision.browse import should_browse_tip
from nest_graph.decision.mcts import MctsAgent
from nest_graph.decision.motif_credit import merge_void_elite_with_archive
from nest_graph.decision.niche_archive import MacroNicheArchive, NICHE_RING_H
from nest_graph.decision.ram_budget import evaluate_ram_band, ram_budget_mb
from nest_graph.decision.types import BoardSnapshot
from nest_graph.elem_graph import DecisionArena, MacroAction, MacroRegion, MotifBase, MotifRecord, Se2


def test_progressive_bias_no_1e9():
    arena = DecisionArena()
    agent = MctsAgent(arena=arena, motif_base=MotifBase())
    snap = BoardSnapshot(remaining_gids=(0, 1), coverage=0.0, free_kind="large_void")
    a = agent.pick_expand_action(snap.remaining_gids, parent_id=0, snapshot=snap)
    assert a is not None
    # Score path must be finite Progressive Bias (never 1e9 sentinel).
    key = agent._action_key(a)
    score = agent._amaf_pick_score(key, 0)
    assert score < 1e8
    assert math.isfinite(score)


def test_macro_niche_archive_buckets_dict():
    arch = MacroNicheArchive()
    key = (1, 0, -1)
    arch.append_positive(key, rows=[(0, 1.0, 2.0, 0.0)], void_nest=1)
    assert int(arch.size) == 1
    assert int(arch.total_hits()) == 1
    assert arch.buckets


def test_macro_niche_archive_last_feed_keys_roundtrip():
    arch = MacroNicheArchive()
    keys = {
        (round(1.2345, 4), round(6.789, 4), round(0.0, 4)),
        (round(0.1, 4), round(0.2, 4), round(1.5708, 4)),
    }
    arch.last_feed_keys = set(keys)
    got = set(tuple(k) for k in arch.last_feed_keys)
    assert got == keys
    nest_keys = set(keys)
    assert bool(got & nest_keys)
    arch.last_feed_keys = set()
    assert not arch.last_feed_keys


def test_macro_niche_archive_ring_and_negative_meta_only():
    arch = MacroNicheArchive()
    key = (1, 0, -1)
    arch.append_positive(
        key,
        rows=[(0, 1.0, 2.0, 0.0), (1, 3.0, 4.0, 0.5)],
        void_nest=3,
    )
    arch.append_negative(key, void_nest=0)
    bucket = arch.get(key)
    assert bucket.hits == 1
    assert bucket.misses == 1
    assert len(bucket.ring) <= NICHE_RING_H
    assert bucket.ring[-1].polarity == -1
    assert len(bucket.ring[-1].rows) == 0
    by_g = arch.active_by_group(2)
    assert 0 in by_g and len(by_g[0]) >= 1


def test_merge_void_elite_60_40():
    cur = {0: [np.array([0.0, 0.0, 0.0]), np.array([1.0, 0.0, 0.0])]}
    arch = {0: [np.array([2.0, 0.0, 0.0]), np.array([3.0, 0.0, 0.0])]}
    merged = merge_void_elite_with_archive(
        cur, arch, elite_quota=10, void_seek=True
    )
    assert len(merged[0]) <= 10
    # Should include both current and archive under void_seek
    xs = {round(float(r[0]), 4) for r in merged[0]}
    assert 0.0 in xs
    assert 2.0 in xs or 3.0 in xs


def test_motif_ttl_floor_protects_accepted():
    base = MotifBase()
    r = MotifRecord()
    r.gid_a = 1
    r.gid_b = 2
    r.relative = Se2(1.0, 0.0, 0.0)
    r.gci = 0.9
    r.compactness = 0.8
    r.area_a = 10.0
    r.area_b = 3.0
    mid = base.upsert(r, 0.5, ttl=1)
    assert mid == 0
    assert base.at(0).accept_count >= 1
    # Age once would hit 0; Q113 floors at 1 when accept_count > 0
    base.age(1)
    assert base.size() == 1
    assert base.at(0).ttl_remaining == 1


def test_motif_credit_accept():
    base = MotifBase()
    r = MotifRecord()
    r.gid_a = 0
    r.gid_b = 1
    r.relative = Se2(2.0, 0.0, 0.0)
    r.gci = 0.5
    r.compactness = 0.7
    r.area_a = 1.0
    r.area_b = 1.0
    mid = base.upsert(r, 0.0, ttl=4)
    before = base.at(mid).accept_count
    assert base.credit_accept(mid, 4)
    assert base.at(mid).accept_count == before + 1


def test_browse_k_cadence():
    assert should_browse_tip(iter_idx=2, n_iters=10, is_last_leaf=False)
    assert not should_browse_tip(iter_idx=0, n_iters=10, is_last_leaf=False)
    assert should_browse_tip(iter_idx=0, n_iters=10, is_last_leaf=True)


def test_note_macro_miss_and_tombstone():
    arena = DecisionArena()
    agent = MctsAgent(arena=arena, motif_base=MotifBase())
    a = MacroAction()
    a.region = MacroRegion.Rim
    a.rule_id = 0
    a.motif_id = -1
    agent.note_macro_miss(a)
    key = agent._action_key(a)
    region_i = int(key[0])
    assert int(arena.amaf_misses(region_i, int(a.rule_id), int(a.motif_id))) == 1
    child = arena.add_node(0, a)
    agent.age_and_tombstone(spine_id=0, idle_t=1, force=True)
    # child visits=0 < 2 and forced → tombstoned
    assert child in agent._tombstoned or arena.visits(child) >= 2


def test_ram_budget_default_2gb():
    assert ram_budget_mb() == 2048.0
    band = evaluate_ram_band(default_graphs_window=24, default_n_sims=4)
    assert band.band in ("ok", "pressure", "critical")
    assert band.rss_mb >= 0.0
