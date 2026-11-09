"""H0: 3a/3b + niche AMAF keys assemble into void_leak for letter census."""

from types import SimpleNamespace

from nest_graph.propose.telem import build_void_leak_dict
from nest_graph.graph import FreeKind


def test_void_leak_includes_block_and_niche_amaf_keys():
    propose_stats = {
        "block_cohort_accepted": 1,
        "block_cohort_related": 2,
        "block_hole_accepted": 0,
        "block_hole_tried": 1,
        "block_hole_emit_in_hull": 4,
        "block_hole_mate_n": 2,
        "block_hole_dcount": 0,
        "block_hole_darea": 0.0,
        "block_hole_reject_reason": "lex",
        "run_3b": 1,
        "niche_amaf_region": 1,
        "niche_amaf_rule": 0,
        "niche_amaf_motif": -1,
        "repair_mode": 1,
        "repair_patterns_n": 2,
        "kind_survive_hist": [1, 4, 0, 2],
        "relocate": {"attempted": 3, "accepted": 1, "moved": 2},
        "local_se2": {"attempted": 5, "moved": 1},
        "motif_sequential_partial": 2,
        "motif_sequential_rcl": 7,
    }
    leak = build_void_leak_dict(
        free_kind=FreeKind.LargeVoid,
        max_void_ratio=0.5,
        n_void_props=0,
        n_props_pole=0,
        hijack=False,
        n_void_graph=10,
        n_void_nest=0,
        n_void_refine=0,
        nest_refine_delta=0,
        outline_cov=0.0,
        sat_override=False,
        rim_progress=0.0,
        plateau=SimpleNamespace(on_plateau=False, streak=0),
        pin_all_blocked_streak=0,
        propose_stats=propose_stats,
        zones=[],
        pf_em=0,
        pf_att=0,
        pf_sel=0,
        pin_stats={},
        pf_surv=0,
        pocket_key_hits=0,
        motif_key_hits=0,
        island_hits=0,
        small_hits=0,
        densify_f=0,
        densify_a=0,
        densify_reason=None,
        pocket_skip=[],
        emitted_bp={},
        pool_bp={},
        nest_bp={},
        refine_bp={},
        pocket_by_tag={},
        prop_accept="",
        densify={"multi_pole_count": 2},
        elite_seeded=0,
        elite_next=0,
        cc_e=0,
        cc_p=0,
        sp_e=0,
        sp_p=0,
        fsc_e=0,
        fsc_p=0,
        n_patterns=0,
        attract_edges=0,
        attract_pairs_selected=0,
        attract_bonus=0.0,
    )
    assert leak["block_cohort_accepted"] == 1
    assert leak["block_hole_tried"] == 1
    assert leak["block_hole_reject_reason"] == "lex"
    assert leak["run_3b"] == 1
    assert leak["niche_amaf_region"] == 1
    assert leak["niche_amaf_rule"] == 0
    assert leak["niche_amaf_motif"] == -1
    assert list(leak["kind_survive_hist"])[:4] == [1, 4, 0, 2]
    assert leak["relocate_attempted"] == 3
    assert leak["relocate_accepted"] == 1
    assert leak["relocate_moved"] == 2
    assert leak["local_se2_attempted"] == 5
    assert leak["local_se2_moved"] == 1
    assert leak["motif_sequential_partial"] == 2
    assert leak["motif_sequential_rcl"] == 7
    assert leak["multi_pole_count"] == 2
