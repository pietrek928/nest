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
