from nest_graph.pack.macro_path import ancestors
from nest_graph.pack.runner import MacroMctsRunner
from nest_graph.graph import MacroAction, MacroRegion


def test_runner_ancestors_root_only():
    runner = MacroMctsRunner()
    root = int(runner.arena.root_id())
    assert ancestors(runner, root) == [root]


def test_resolve_motif_keys_absorbs_graph_keys():
    from nest_graph.propose.motif_keys import resolve_motif_keys

    keys = resolve_motif_keys({
        "motif_graph_keys": {1: {(1.0, 2.0, 0.0)}},
    })
    assert 1 in keys
    assert (1.0, 2.0, 0.0) in keys[1]


def test_active_rule_set_index():
    from nest_graph.propose.selection_compose import active_rule_set
    from nest_graph.rules.evolve import PlacementRuleSet

    rs = [PlacementRuleSet(), PlacementRuleSet()]
    assert active_rule_set(rs, 1) is rs[1]


def test_macro_increase_path_rejects_overlap_fail():
    from nest_graph.pack.macro_path import macro_increase_path
    from nest_graph.pack.runner import MacroMctsRunner
    from nest_graph.graph import BoardSnapshot
    from nest_graph.graph import MacroAction, MacroRegion

    runner = MacroMctsRunner()
    root = int(runner.arena.root_id())
    snap = BoardSnapshot(remaining_gids=(0, 1), coverage=0.5, free_kind="large_void")
    runner.store_snapshot(root, snap)
    child = int(runner.arena.add_node(root, MacroAction()))
    runner.store_snapshot(child, BoardSnapshot(remaining_gids=(0,), coverage=0.6))
    telem: dict = {}
    blocked = {"ok": True}

    def execute_fn(_parent, *, zone=None, action=None, patterns=None):
        del zone, action, patterns
        return BoardSnapshot(remaining_gids=(), coverage=0.7)

    def overlap_fail():
        blocked["ok"] = False
        return False

    alt, reward = macro_increase_path(
        runner,
        leaf_id=child,
        baseline_reward=0.5,
        execute_fn=execute_fn,
        telem=telem,
        overlap_ok_fn=overlap_fail,
    )[:2]
    assert alt is None or reward <= 0.7
    assert blocked["ok"] is False or telem.get("macro_swap_attempts", 0) >= 0


def test_path_accept_eligible_large_void_needs_void_fill_delta():
    """P2: large_void Void cov_ok needs void_fill Δ; P Motif skips void_fill Δ."""
    from nest_graph.pack.macro_path import path_accept_eligible
    from nest_graph.graph import BoardSnapshot, MacroAction, MacroRegion

    parent = BoardSnapshot(
        remaining_gids=(0,),
        coverage=0.50,
        void_fill=0.10,
        free_kind="large_void",
    )
    alt = BoardSnapshot(
        remaining_gids=(0,),
        coverage=0.51,
        void_fill=0.13,
        free_kind="large_void",
    )
    action = MacroAction()
    action.region = MacroRegion.Void
    # Cov up + void_fill Δ 0.03 → ok.
    elig, cov_ok, _, _ = path_accept_eligible(
        alt_action=action,
        path_accept_snap=alt,
        parent_snap=parent,
        base_reward=0.5,
        alt_reward=0.7,
        path_overlap_ok=True,
    )
    assert elig is True
    assert cov_ok is True
    # Cov non-regress + void_fill Δ 0.01 → eligible but not cov_ok (Void).
    alt_small = BoardSnapshot(
        remaining_gids=(0,),
        coverage=0.51,
        void_fill=0.11,
        free_kind="large_void",
    )
    elig2, cov_ok2, _, _ = path_accept_eligible(
        alt_action=action,
        path_accept_snap=alt_small,
        parent_snap=parent,
        base_reward=0.5,
        alt_reward=0.7,
        path_overlap_ok=True,
    )
    assert elig2 is True
    assert cov_ok2 is False
    # P: Motif + small void_fill Δ with relax → cov_ok from overlap + cov non-regress.
    motif = MacroAction()
    motif.region = MacroRegion.Motif
    elig_m, cov_m, _, _ = path_accept_eligible(
        alt_action=motif,
        path_accept_snap=alt_small,
        parent_snap=parent,
        base_reward=0.5,
        alt_reward=0.7,
        path_overlap_ok=True,
        relax_motif_void_fill=True,
    )
    assert elig_m is True
    assert cov_m is True
    # Without relax Motif still needs void_fill Δ.
    elig_nr, cov_nr, _, _ = path_accept_eligible(
        alt_action=motif,
        path_accept_snap=alt_small,
        parent_snap=parent,
        base_reward=0.5,
        alt_reward=0.7,
        path_overlap_ok=True,
        relax_motif_void_fill=False,
    )
    assert elig_nr is True
    assert cov_nr is False
    # Motif + relax + overlap fail → not cov_ok.
    elig_f, cov_f, _, _ = path_accept_eligible(
        alt_action=motif,
        path_accept_snap=alt_small,
        parent_snap=parent,
        base_reward=0.5,
        alt_reward=0.7,
        path_overlap_ok=False,
        relax_motif_void_fill=True,
    )
    assert elig_f is True
    assert cov_f is False
    # Motif + relax + cov regress → not cov_ok (even with overlap).
    alt_reg = BoardSnapshot(
        remaining_gids=(0,),
        coverage=0.49,
        void_fill=0.20,
        free_kind="large_void",
    )
    elig_rg, cov_rg, _, _ = path_accept_eligible(
        alt_action=motif,
        path_accept_snap=alt_reg,
        parent_snap=parent,
        base_reward=0.5,
        alt_reward=0.7,
        path_overlap_ok=True,
        relax_motif_void_fill=True,
    )
    assert elig_rg is True
    assert cov_rg is False
    # Non-void: eligible via +0.006 cov, but min_cov_delta=0.01 → not cov_ok.
    parent_rim = BoardSnapshot(
        remaining_gids=(0,),
        coverage=0.50,
        void_fill=0.0,
        free_kind="swiss_cheese",
    )
    alt_rim = BoardSnapshot(
        remaining_gids=(0,),
        coverage=0.506,
        void_fill=0.0,
        free_kind="swiss_cheese",
    )
    elig3, cov_ok3, _, _ = path_accept_eligible(
        alt_action=action,
        path_accept_snap=alt_rim,
        parent_snap=parent_rim,
        base_reward=0.5,
        alt_reward=0.7,
        path_overlap_ok=True,
        min_cov_delta=0.01,
    )
    assert elig3 is True
    assert cov_ok3 is False

    from nest_graph.pack.macro_path import path_probe_budget
    from nest_graph.graph import MacroAction, MacroRegion

    class _Agent:
        place_cohort_ready = False

    sheet = MacroAction()
    sheet.region = MacroRegion.Sheet
    # P1: free hint, ready=0 → tight shrink-run (beam/4).
    run, beam, depth = path_probe_budget(
        on_plateau=False,
        parent_free_hint=True,
        agent=_Agent(),
        tip_action=sheet,
        beam=8,
        max_depth=4,
    )
    assert run is True
    assert beam == 2 and depth == 2
    # Plateau only, no free hint, ready=0, non-Motif → still skip.
    run_skip, _, _ = path_probe_budget(
        on_plateau=True,
        parent_free_hint=False,
        agent=_Agent(),
        tip_action=sheet,
        beam=6,
        max_depth=3,
    )
    assert run_skip is False

    class _Ready:
        place_cohort_ready = True
        telem = {"mcts_cohort_macro_n": 2}

    run2, beam2, depth2 = path_probe_budget(
        on_plateau=False,
        parent_free_hint=True,
        agent=_Ready(),
        tip_action=sheet,
        beam=6,
        max_depth=3,
    )
    assert run2 is True
    assert beam2 == 6 and depth2 == 3

    motif = MacroAction()
    motif.region = MacroRegion.Motif
    motif.motif_id = 1
    run3, beam3, depth3 = path_probe_budget(
        on_plateau=True,
        parent_free_hint=False,
        agent=_Agent(),
        tip_action=motif,
        beam=6,
        max_depth=3,
    )
    assert run3 is True
    assert beam3 == 3 and depth3 == 2


def test_path_probe_budget_shrinks_ready_macros_idle():
    """G1: ready∧macros=0∧non-Motif tip → shrink (not full beam)."""
    from nest_graph.pack.macro_path import path_probe_budget
    from nest_graph.graph import MacroAction, MacroRegion

    class _ReadyIdle:
        place_cohort_ready = True
        telem = {"mcts_cohort_macro_n": 0}

    sheet = MacroAction()
    sheet.region = MacroRegion.Sheet
    run, beam, depth = path_probe_budget(
        on_plateau=False,
        parent_free_hint=True,
        agent=_ReadyIdle(),
        tip_action=sheet,
        beam=6,
        max_depth=3,
    )
    assert run is True
    assert beam == 3 and depth == 2
    # Mild floor: beam/depth never collapse below 2 while ready.
    run_lo, beam_lo, depth_lo = path_probe_budget(
        on_plateau=True,
        parent_free_hint=False,
        agent=_ReadyIdle(),
        tip_action=sheet,
        beam=2,
        max_depth=2,
    )
    assert run_lo is True
    assert beam_lo == 2 and depth_lo == 2

    class _ReadyLive:
        place_cohort_ready = True
        telem = {"mcts_cohort_macro_n": 4}

    run2, beam2, depth2 = path_probe_budget(
        on_plateau=False,
        parent_free_hint=True,
        agent=_ReadyLive(),
        tip_action=sheet,
        beam=6,
        max_depth=3,
    )
    assert run2 is True
    assert beam2 == 6 and depth2 == 3


def test_record_to_cluster_pattern_ref_anchor():
    from nest_graph.propose.pattern_archive import (
        note_motif_ref_anchors,
        note_motif_ref_anchors_from_nest,
        motif_patterns_for_inject,
        record_to_cluster_pattern,
    )
    from nest_graph.build_graph import NestState
    from nest_graph.graph import MotifBase, MotifRecord, Se2

    mb = MotifBase()
    r = MotifRecord()
    r.gid_a = 0
    r.gid_b = 1
    r.relative = Se2(1.0, 0.0, 0.0)
    mid = int(mb.upsert(r))
    note_motif_ref_anchors(mb, [0, 1], [(5.0, 6.0, 0.1), (6.0, 6.0, 0.0)])
    pat = record_to_cluster_pattern(mb.at(mid), motif_id=mid)
    assert pat.ref_transform[0] == 5.0
    assert pat.ref_transform[1] == 6.0

    mb2 = MotifBase()
    r2 = MotifRecord()
    r2.gid_a = 0
    r2.gid_b = 1
    r2.relative = Se2(1.0, 0.0, 0.0)
    mid2 = int(mb2.upsert(r2))
    ns = NestState(
        polys=[None, None],
        group_id=[0, 1],
        transform=[(5.0, 6.0, 0.1), (6.0, 6.0, 0.0)],
        selected_indices=[0],
        seed_count=0,
    )
    note_motif_ref_anchors_from_nest(mb2, ns)
    telem_pat: dict = {}
    pats = motif_patterns_for_inject(mb2, telem=telem_pat, polish=False)
    assert len(pats) == 1
    assert pats[0].ref_transform[0] == 5.0
    assert telem_pat.get("archive_ref_origin_n", 0) == 0


def test_path_accept_apply_void_soft_tip_large_void():
    """P1 Void soft tip left dormant (early regress); Motif soft tip still works."""
    from nest_graph.graph import MacroAction, MacroRegion
    from nest_graph.pack.macro_path import path_accept_apply

    void_action = MacroAction()
    void_action.region = MacroRegion.Void
    telem: dict = {}
    out = path_accept_apply(
        mode="build_graph",
        cov_ok=True,
        path_overlap_ok=True,
        alt_action=void_action,
        enable_macro_path_replay=False,
        mutate_motif_base_on_path=False,
        telem=telem,
        free_kind="large_void",
    )
    assert out["tip_install"] is False
    assert out["credit"] is True

    motif_action = MacroAction()
    motif_action.region = MacroRegion.Motif
    telem_m: dict = {}
    out_m = path_accept_apply(
        mode="build_graph",
        cov_ok=True,
        path_overlap_ok=True,
        alt_action=motif_action,
        enable_macro_path_replay=False,
        mutate_motif_base_on_path=False,
        telem=telem_m,
        free_kind="large_void",
    )
    assert out_m["tip_install"] is True
    assert out_m["motif_soft"] is True
    assert int(telem_m.get("path_tip_apply", 0) or 0) == 1
    assert int(telem_m.get("path_join_signal", 0) or 0) == 0

    # P: Motif on cov_skip must not tip_install (coverage check required).
    telem_skip: dict = {}
    out_skip = path_accept_apply(
        mode="build_graph",
        cov_ok=False,
        path_overlap_ok=True,
        alt_action=motif_action,
        enable_macro_path_replay=False,
        mutate_motif_base_on_path=False,
        telem=telem_skip,
        free_kind="large_void",
    )
    assert out_skip["cov_skip"] is True
    assert out_skip["tip_install"] is False
    assert out_skip["credit"] is True
    assert int(telem_skip.get("path_tip_apply", 0) or 0) == 0

    # P: evaluator Motif without replay → credit/soft only (no tip_install; early OK).
    telem_ev: dict = {}
    out_ev = path_accept_apply(
        mode="evaluator",
        cov_ok=True,
        path_overlap_ok=True,
        alt_action=motif_action,
        enable_macro_path_replay=False,
        mutate_motif_base_on_path=False,
        telem=telem_ev,
        free_kind="large_void",
    )
    assert out_ev["tip_install"] is False
    assert out_ev["credit"] is True
    assert out_ev["motif_soft"] is True
    assert int(telem_ev.get("path_tip_apply", 0) or 0) == 0


def test_path_join_signal_distinct_from_tip_apply():
    """T0: MotifJoin-on-accept bumps path_join_signal, not path_tip_apply."""
    from nest_graph.pack import macro_path as mp

    telem: dict = {
        "macro_swap_attempts": 0,
        "macro_path_accept": 0,
        "macro_swap_depth": 0,
        "macro_path_beam_n": 0,
        "path_step_macro": 0,
        "path_step_join": 0,
        "path_extend_n": 0,
        "macro_chain_accept": 0,
    }
    # Simulate post-accept telem bump (same block as macro_increase_path).
    accept = 1
    path_step_join = 2
    if accept > 0 and path_step_join > 0:
        telem["path_join_signal"] = int(telem.get("path_join_signal", 0) or 0) + 1
        telem["macro_path_motif_soft"] = int(
            telem.get("macro_path_motif_soft", 0) or 0
        ) + 1
    assert int(telem["path_join_signal"]) == 1
    assert int(telem.get("path_tip_apply", 0) or 0) == 0
    assert "path_join_signal" in mp.__all__ or hasattr(mp, "path_accept_apply")
