from nest_graph.decision.mcts import MctsAgent, leaf_reward
from nest_graph.decision.nfp_lite import nfp_lite_relative
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
