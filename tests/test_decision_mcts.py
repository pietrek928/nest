import math

from nest_graph.decision.mcts import MctsAgent, leaf_reward
from nest_graph.geometry import nfp_lite_relative
from nest_graph.decision.runner import MacroMctsRunner
from nest_graph.decision.types import BoardSnapshot
from nest_graph.elem_graph import DecisionArena, MacroAction, MacroRegion, MotifBase, MotifRecord, Se2
from nest_graph.geometry import Geometry
from shapely.geometry import box


def test_decision_arena_python_roundtrip():
    arena = DecisionArena()
    a = MacroAction()
    a.region = MacroRegion.Void
    a.rule_id = 2
    cid = arena.add_node(arena.root_id(), a)
    arena.record_visit(cid, 0.4)
    assert arena.visits(cid) == 1
    assert abs(arena.total_reward(cid) - 0.4) < 1e-6


def test_motif_base_upsert_order_invariant():
    base = MotifBase()
    r = MotifRecord()
    r.gid_a = 1
    r.gid_b = 2
    r.relative = Se2(1.0, 0.0, 0.0)
    r.gci = 0.9
    r.compactness = 0.8
    r.area_a = 10.0
    r.area_b = 3.0
    assert base.upsert(r, 0.5) == 0
    r2 = MotifRecord()
    r2.gid_a = 2
    r2.gid_b = 1
    r2.relative = Se2(-1.0, 0.0, 0.0)
    r2.gci = 0.5
    r2.compactness = 0.8
    r2.area_a = 3.0
    r2.area_b = 10.0
    assert base.upsert(r2, 0.5) == 0
    assert base.size() == 1


def test_mcts_runner_stub_sims():
    runner = MacroMctsRunner()
    root = BoardSnapshot(
        packed_gids=(),
        packed_transforms=(),
        remaining_gids=(10, 11, 12),
        coverage=0.0,
    )
    best = runner.run(root, n_sims=8)
    assert len(best.packed_gids) >= 1
    assert runner.agent is not None
    assert runner.agent.telem["pw_expand"] >= 1
    assert runner.agent.telem["from_shapely_count"] == 0


def test_arena_snapshot_size_invariant():
    arena = DecisionArena()
    assert arena.size() == 1
    root = BoardSnapshot(remaining_gids=(0, 1), coverage=0.1, free_kind="large_void")
    arena.set_snapshot(0, root)
    assert abs(arena.snapshot(0).coverage - 0.1) < 1e-6
    assert arena.snapshot(0).has_remaining
    assert list(arena.snapshot(0).remaining_gids) == [0, 1]
    a = MacroAction()
    a.region = MacroRegion.Void
    cid = arena.add_node(0, a)
    assert arena.size() == 2
    child = arena.snapshot(cid)
    assert abs(child.coverage - 0.1) < 1e-6
    assert child.free_kind == "large_void"
    child.coverage = 0.2
    assert abs(arena.snapshot(0).coverage - 0.1) < 1e-6
    assert abs(arena.snapshot(cid).coverage - 0.2) < 1e-6


def test_leaf_reward_terms():
    snap = BoardSnapshot(
        coverage=0.5,
        kiss_pairs=2,
        packed_gids=(1, 2),
        mean_compactness=0.8,
        void_fill=0.4,
        rim_fill=0.2,
    )
    r = leaf_reward(snap)
    # cov + 0.5*void + 0.1*rim + kiss/comp > coverage alone
    assert r > 0.5 + 0.5 * 0.4 + 0.1 * 0.2 - 1e-9
    assert abs(r - leaf_reward(snap, lam_void=0.0, lam_rim=0.0) - 0.22) < 1e-6
    # Q108: large_void raises void weight vs same fills without the kind.
    snap_base = BoardSnapshot(
        coverage=0.5,
        void_fill=0.4,
        rim_fill=0.2,
        packed_gids=(1,),
    )
    snap_void = BoardSnapshot(
        coverage=0.5,
        void_fill=0.4,
        rim_fill=0.2,
        free_kind="large_void",
        packed_gids=(1,),
    )
    assert leaf_reward(snap_void) > leaf_reward(snap_base)


def test_polish_patterns_at_inject_keeps_on_missing_bases():
    from nest_graph.propose.pattern_archive import polish_patterns_at_inject
    from nest_graph.propose.placements_pattern import ClusterPattern

    pat = ClusterPattern(
        members=((0, (0.0, 0.0, 0.0)), (1, (1.0, 0.0, 0.0))),
        part_count=2,
        ref_transform=(0.0, 0.0, 0.0),
    )
    telem: dict = {}
    out = polish_patterns_at_inject([pat], {}, min_dist=0.1, telem=telem)
    assert len(out) == 1
    assert out[0].members[1][1] == (1.0, 0.0, 0.0)
    assert telem.get("nfp_lite_keep", 0) >= 1


def test_nfp_lite_does_not_flee_anchor():
    follow = Geometry.from_shapely(box(0, 0, 1, 1))
    anchor = Geometry.from_shapely(box(2, 0, 3, 1))
    t0 = (0.0, 0.0, 0.0)
    t1 = nfp_lite_relative(follow, t0, anchor, min_dist=0.0, max_t=5.0)
    d0 = abs(2.5 - t0[0])
    d1 = abs(2.5 - t1[0])
    assert d1 <= d0 + 1e-6


def test_upsert_from_contacts_kissing_pair():
    from nest_graph.decision.slave_pack import upsert_from_contacts

    a = Geometry.from_shapely(box(0, 0, 1, 1))
    b = Geometry.from_shapely(box(1.0, 0, 2.0, 1))
    base = MotifBase()
    n = upsert_from_contacts(
        base,
        [a, b],
        [1, 2],
        [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)],
        gap=0.1,
        min_compactness=0.35,
        ttl=4,
    )
    assert n >= 1
    assert base.size() >= 1
    assert base.at(0).gci > 0.0
    assert base.at(0).compactness >= 0.35


def test_arena_amaf_is_ucb_and_pick_sot():
    arena = DecisionArena()
    agent = MctsAgent(arena=arena, motif_base=MotifBase())
    a = MacroAction()
    a.region = MacroRegion.Void
    a.rule_id = 0
    a.motif_id = -1
    cid = arena.add_node(arena.root_id(), a)
    agent.backprop(cid, 0.8)
    region_i = int(getattr(a.region, "value", a.region))
    assert int(arena.amaf_visits(region_i, 0, -1)) >= 1
    assert abs(float(arena.amaf_mean(region_i, 0, -1)) - 0.8) < 1e-5
    score = agent._ucb(cid, parent_visits=2)
    assert math.isfinite(score)
    assert int(agent.telem["amaf_hits"]) >= 1
    key = agent._action_key(a)
    pick = agent._amaf_pick_score(key, 0)
    assert math.isfinite(pick)
    assert pick < 1e8
    agent.note_macro_miss(a)
    assert int(arena.amaf_misses(region_i, 0, -1)) == 1
    pick_miss = agent._amaf_pick_score(key, 0)
    assert pick_miss < pick


def test_stamp_arena_amaf_sets_hits():
    from nest_graph.decision.execute import stamp_arena_amaf

    runner = MacroMctsRunner()
    stats: dict = {"free_kind": "large_void"}
    parent, _action = stamp_arena_amaf(
        runner,
        selected_polys=[],
        group_id=[],
        transform=[],
        ngroups=2,
        coverage_pct=10.0,
        propose_stats=stats,
        parent_id=0,
    )
    assert parent >= 0
    assert int(stats.get("amaf_hits", 0)) >= 1
    # Packed-all-groups: remaining empty, still stamps a Void/Sheet action.
    stats2: dict = {"free_kind": "large_void"}
    runner2 = MacroMctsRunner()
    parent2, _a2 = stamp_arena_amaf(
        runner2,
        selected_polys=[0],
        group_id=[0, 1],
        transform=[(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)],
        ngroups=2,
        coverage_pct=20.0,
        propose_stats=stats2,
        parent_id=0,
    )
    # One group packed → remaining still has 1; also cover empty-remaining via both packed.
    stats3: dict = {"free_kind": "large_void"}
    runner3 = MacroMctsRunner()
    parent3, _a3 = stamp_arena_amaf(
        runner3,
        selected_polys=[0, 1],
        group_id=[0, 1],
        transform=[(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)],
        ngroups=2,
        coverage_pct=40.0,
        propose_stats=stats3,
        parent_id=0,
    )
    assert parent2 >= 0 and parent3 >= 0
    assert int(stats3.get("amaf_hits", 0)) >= 1


def test_amaf_pick_reads_realized_kind_attach():
    from nest_graph.decision.motif_credit import credit_motif_on_nest_survival

    arena = DecisionArena()
    agent = MctsAgent(arena=arena, motif_base=MotifBase())
    a = MacroAction()
    a.region = MacroRegion.Void
    a.rule_id = 0
    a.motif_id = -1
    key = agent._action_key(a)
    base = agent._amaf_pick_score(key, 0, free_kind="")
    credit_motif_on_nest_survival(
        agent.motif_base,
        selected_polys=[0, 1],
        group_id=[0, 1],
        transform=[(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)],
        motif_keys=None,
        ttl=4,
        telem={},
        realized_out=agent.realized,
        kind_survive=(0, 4, 0, 0),
        materialized_attach=2,
        member_hits=4,
    )
    assert int(agent.realized["sel_n"]) == 2
    assert agent.realized["kind"][1] == 4
    boosted = agent._amaf_pick_score(key, 0, free_kind="")
    assert boosted > base


def test_run_mcts_multi_sim_does_not_upsert_contacts():
    import inspect

    from nest_graph.decision.execute import run_mcts_multi_sim
    from nest_graph.decision.slave_pack import cheap_expand_slave

    assert "upsert_from_contacts(" not in inspect.getsource(run_mcts_multi_sim)
    assert "upsert_from_contacts(" not in inspect.getsource(cheap_expand_slave)


def test_finalize_iter_mcts_cache_hit_damps_proposer_pb():
    from nest_graph.decision.pack_loop import finalize_iter_mcts
    from nest_graph.elem_graph import DecisionGraph, PoseGraph

    runner = MacroMctsRunner()
    g = PoseGraph()
    g.reserve_elems(1)
    g.append_elem_at(0, 0.5, 0.5, 0.0, 0.5, 0.5, 0.25)
    runner.dg.replace_poses(g)
    stats = {
        "cache_hit": 1,
        "proposal_count": 0,
        "proposal_keys": {0: {(0.5, 0.5, 0.0)}},
        "motif_keys": {},
    }
    finalize_iter_mcts(
        runner,
        selected_polys=[0],
        group_id=[0],
        transform=[(0.5, 0.5, 0.0)],
        propose_stats=stats,
        mcts_telem={},
        motif_keys={},
        motif_ttl=4,
        credit_motif=False,
        refine_bp={"cluster_copy": 1},
        emitted_bp={"cluster_copy": 4},
    )
    assert abs(float(runner.agent.realized["proposer_pb"]) - 0.0625) < 1e-6
