"""Letter A: continuous void_fit_ratio + DecisionGraph.conflict_p soft Macro."""

from nest_graph.free_kind_util import void_phantom
from nest_graph.graph import (
    DecisionGraph,
    FreeKind,
    PathKind,
    node_macro,
    node_pose,
)
from nest_graph.pack.execute import void_fit_r_from_stats, void_fit_ratio_from_stats


def test_void_fit_ratio_from_stats_continuous():
    assert void_fit_ratio_from_stats({"fit_in_r": 0.5, "fit_need_r": 1.0}) == 0.5
    assert void_fit_ratio_from_stats({"fit_in_r": 2.0, "fit_need_r": 1.0}) == 1.0
    assert void_fit_r_from_stats({"fits_part": 0}) == 0.0
    # Fallback to 0/1 bit when radii missing.
    assert void_fit_ratio_from_stats({"fits_part": 1}) == 1.0
    assert void_fit_ratio_from_stats({"fits_part": 0}) == 0.0


def test_conflict_p_macro_soft_pose_hard():
    dg = DecisionGraph()
    # Empty poses — conflict_p Macro↔Pose soft path does not need collisions.
    macro = node_macro(0)
    pose = node_pose(0)
    assert macro.kind == PathKind.Macro
    assert not dg.conflicts(macro, pose)
    p = float(dg.conflict_p(macro, pose, FreeKind.LargeVoid, 0.5))
    assert 0.0 < p < 0.85
    assert abs(p - 0.85 * 0.5) < 1e-5
    assert abs(float(dg.conflict_p(macro, pose, FreeKind.LargeVoid, 0.0)) - 0.85) < 1e-6
    assert abs(float(dg.conflict_p(macro, pose, FreeKind.Empty, 0.0))) < 1e-9
    assert void_phantom(FreeKind.LargeVoid, 0.0)
    assert not void_phantom(FreeKind.LargeVoid, 1.0)


def test_amaf_phantom_keeps_swiss_demote_and_conflict_p_telem():
    from nest_graph.graph import DecisionArena, MacroAction, MacroRegion, MotifBase, MctsAgent

    arena = DecisionArena()
    agent = MctsAgent(arena=arena, motif_base=MotifBase())
    a = MacroAction()
    a.region = MacroRegion.Void
    a.rule_id = 0
    key = agent._action_key(a)
    base = float(agent._amaf_pick_score(key, 0, free_kind=FreeKind.Empty))
    phantom0 = float(
        agent._amaf_pick_score(
            key,
            0,
            free_kind=FreeKind.LargeVoid,
            void_fit_r=0.0,
            void_fit_ratio=0.5,
        )
    )
    # Swiss magnitude −0.2 regardless of continuous ratio (dual research).
    assert abs((base - 0.2) - phantom0) < 1e-5
    assert int(agent.telem.get("extend_scaled_n", 0) or 0) >= 1
    assert int(agent.telem.get("conflict_p_soft_n", 0) or 0) >= 1
