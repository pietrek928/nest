"""DecisionGraph copy-in, attach skip, and bind_epoch MemberOf join."""

from nest_graph.decision.epoch import bind_epoch, materialize_selection
from nest_graph.decision.runner import MacroMctsRunner
from nest_graph.elem_graph import DecisionGraph, MacroAction, MacroRegion, PoseGraph, nest_by_scores
from nest_graph.propose.void_selection import transform_row_key


def _two_poses() -> PoseGraph:
    g = PoseGraph()
    g.reserve_elems(2)
    g.append_elem_at(0, 0.5, 0.5, 0.0, 0.5, 0.5, 0.25)
    g.append_elem_at(1, 1.6, 0.5, 0.0, 1.6, 0.5, 0.25)
    return g


def test_replace_poses_copy_in_keeps_macros():
    dg = DecisionGraph()
    a = MacroAction()
    a.region = MacroRegion.Void
    child = dg.macros().add_node(0, a)
    dg.macros().record_visit(child, 0.4)
    src = _two_poses()
    dg.replace_poses(src)
    assert len(src.group_id) == 2
    assert len(dg.poses().group_id) == 2
    assert dg.macros().size() == 2
    assert dg.attach_n() == 0
    src.append_elem_at(0, 9.0, 9.0, 0.0, 9.0, 9.0, 0.25)
    assert len(dg.poses().group_id) == 2


def test_add_attach_skips_collision():
    dg = DecisionGraph()
    g = _two_poses()
    g.add_collision(0, 1)
    dg.replace_poses(g)
    dg.add_attach(0, 1)
    assert dg.attach_n() == 0


def test_bind_epoch_stamps_kind_keys_and_attach():
    g = _two_poses()
    g.add_attract(0, 1, 8.0)
    dg = DecisionGraph()
    k0 = transform_row_key((0.5, 0.5, 0.0))
    k1 = transform_row_key((1.6, 0.5, 0.0))
    stats = {
        "mcts_zone": "void_seek",
        "proposal_keys": {0: {k0}, 1: {k1}},
        "motif_keys": {},
    }
    bind_epoch(dg, g, stats, [0, 1], [(0.5, 0.5, 0.0), (1.6, 0.5, 0.0)])
    assert stats["attach_n"] == 1
    assert stats["kind_n"] == 2
    assert k0 in stats["kind_keys"][0]
    assert int(dg.pose_kind[0]) == int(getattr(MacroRegion.Void, "value", 1))
    assert dg.mutex_n() == 0


def test_bind_epoch_history_stays_untagged():
    g = _two_poses()
    dg = DecisionGraph()
    k0 = transform_row_key((0.5, 0.5, 0.0))
    stats = {
        "mcts_zone": "void_seek",
        "proposal_keys": {0: {k0}},
        "motif_keys": {},
    }
    bind_epoch(dg, g, stats, [0, 1], [(0.5, 0.5, 0.0), (1.6, 0.5, 0.0)])
    assert int(dg.pose_kind[0]) == int(getattr(MacroRegion.Void, "value", 1))
    assert int(dg.pose_kind[1]) == 255
    assert 1 not in stats["kind_keys"]


def test_bind_epoch_stamps_epoch_keys_when_proposals_empty():
    g = _two_poses()
    dg = DecisionGraph()
    k1 = transform_row_key((1.6, 0.5, 0.0))
    stats = {
        "mcts_zone": "void_seek",
        "proposal_keys": {},
        "motif_keys": {},
        "epoch_keys": {1: {k1}},
    }
    bind_epoch(dg, g, stats, [0, 1], [(0.5, 0.5, 0.0), (1.6, 0.5, 0.0)])
    assert stats["kind_n"] == 1
    assert k1 in stats["kind_keys"][1]
    assert int(dg.pose_kind[1]) == int(getattr(MacroRegion.Void, "value", 1))
    assert int(dg.pose_kind[0]) == 255
    bind_epoch(None, _two_poses(), {}, [0], [(0.0, 0.0, 0.0)])


def test_materialize_selection_flags_surviving_attach():
    g = _two_poses()
    g.add_attract(0, 1, 8.0)
    dg = DecisionGraph()
    k0 = transform_row_key((0.5, 0.5, 0.0))
    k1 = transform_row_key((1.6, 0.5, 0.0))
    stats = {
        "mcts_zone": "void_seek",
        "proposal_keys": {0: {k0}, 1: {k1}},
        "motif_keys": {},
    }
    bind_epoch(dg, g, stats, [0, 1], [(0.5, 0.5, 0.0), (1.6, 0.5, 0.0)])
    telem = materialize_selection(dg, [0, 1], stats)
    assert telem["materialized_attach"] == 1
    assert telem["member_hits"] == 2
    assert stats["materialized_attach"] == 1
    assert int(telem.get("kind_survive", 0)) == 2
    assert list(telem.get("kind_survive_hist") or [])[1] == 2


def test_nest_by_scores_on_decision_graph():
    dg = DecisionGraph()
    g = _two_poses()
    g.add_collision(0, 1)
    dg.replace_poses(g)
    sel = nest_by_scores(dg, [2.0, 1.0])
    assert list(sel) == [0]


def test_runner_arena_is_dg_macros():
    runner = MacroMctsRunner()
    a = MacroAction()
    a.region = MacroRegion.Rim
    cid = runner.arena.add_node(0, a)
    assert runner.dg.macros().size() == 2
    assert runner.dg.macros().visits(cid) == 0
    assert runner.agent is not None
    assert runner.agent.arena.size() == 2
