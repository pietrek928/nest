"""L2: survive_rank uses capped kind bins for non-motif macros."""

from nest_graph.graph import MacroAction, MacroRegion, survive_rank


def test_survive_rank_motif_unchanged():
    a = MacroAction()
    a.region = MacroRegion.Motif
    a.motif_id = 2
    realized = {"survive_by_motif": {2: 5}, "survive_motif_n": 7, "kind": (9, 9, 9, 9)}
    assert abs(float(survive_rank(a, realized)) - (5.0 + 0.01 * 7)) < 1e-5


def test_survive_rank_void_uses_capped_kind():
    a = MacroAction()
    a.region = MacroRegion.Void
    a.motif_id = -1
    realized = {"survive_by_motif": {}, "survive_motif_n": 2, "kind": (0, 40, 0, 0)}
    # Cap kSurviveRankKindCap=3 + 0.01 * survive_motif_n
    assert abs(float(survive_rank(a, realized)) - (3.0 + 0.02)) < 1e-5


def test_survive_rank_rim_zero_when_kind_empty():
    a = MacroAction()
    a.region = MacroRegion.Rim
    a.motif_id = -1
    realized = {"survive_by_motif": {1: 9}, "survive_motif_n": 0, "kind": (0, 0, 0, 0)}
    assert abs(float(survive_rank(a, realized))) < 1e-6
