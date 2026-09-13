import numpy as np

from nest_graph.propose.motif_keys import (
    cluster_copy_lock_pool,
    merge_motif_cohorts,
    proposer_indices_on_graph,
)
from nest_graph.propose.void_selection import pose_key_to_index
from nest_graph.utils import transform_row_key


def test_merge_motif_cohorts_dedupes_leader_sig():
    key = transform_row_key((1.0, 2.0, 0.0))
    cohort = {
        "leader_key": key,
        "leader_gid": 0,
        "motif_id": 3,
        "member_keys": [(0, key)],
    }
    merged = merge_motif_cohorts([cohort], [dict(cohort), cohort])
    assert len(merged) == 1
    assert merged[0]["motif_id"] == 3


def test_proposer_indices_on_graph():
    tr = np.array([[1.0, 2.0, 0.0], [5.0, 5.0, 90.0]], dtype=np.float64)
    key = transform_row_key(tr[0])
    pk = {"cluster_copy": {key}}
    idxs = proposer_indices_on_graph([0, 0], tr, pk, "cluster_copy")
    assert idxs == [0]


def test_cluster_copy_lock_pool_unions_cohort_members():
    tr = [(1.0, 2.0, 0.0), (5.0, 6.0, 0.0), (9.0, 9.0, 0.0)]
    group_id = [0, 0, 1]
    key_map = pose_key_to_index(group_id, tr)
    k0 = transform_row_key(tr[0])
    k2 = transform_row_key(tr[2])
    cohorts = [{
        "leader_key": k0,
        "leader_gid": 0,
        "member_keys": [(0, k0), (1, k2)],
    }]
    pool = cluster_copy_lock_pool(group_id, tr, {"cluster_copy": set()}, cohorts, key_map)
    assert 0 in pool and 2 in pool


def test_hybrid_compose_pick_cc_survival():
    from nest_graph.propose.motif_lock import hybrid_compose_pick

    telem: dict = {}
    assert hybrid_compose_pick(
        graph=None,
        lock=[0, 1],
        area_cand=0.91,
        area_orig=1.0,
        void_cand=0,
        void_orig=0,
        cc_n2=1,
        lex_better=False,
        telem=telem,
    )
    assert int(telem.get("hybrid_pick_wins", 0)) == 1


def test_hybrid_compose_pick_rejects_area():
    from nest_graph.propose.motif_lock import hybrid_compose_pick

    telem: dict = {}
    assert not hybrid_compose_pick(
        graph=None,
        lock=[0, 1],
        area_cand=0.50,
        area_orig=1.0,
        void_cand=0,
        void_orig=0,
        cc_n2=0,
        lex_better=False,
        telem=telem,
    )
    assert int(telem.get("hybrid_pick_reject_area", 0)) == 1


def test_motif_join_lock_sets_independent_pairs():
    from types import SimpleNamespace

    from nest_graph.propose.motif_lock import motif_join_lock_sets

    class _G:
        collisions = [[1], [0], []]

    dg = SimpleNamespace(motifs=[
        SimpleNamespace(a=0, b=2),
        SimpleNamespace(a=0, b=1),  # colliding — skip
    ])
    locks = motif_join_lock_sets(dg, _G(), max_locks=4)
    assert locks == [[0, 2]]


def test_hybrid_compose_pick_join_soft():
    from nest_graph.propose.motif_lock import hybrid_compose_pick

    telem: dict = {}
    assert hybrid_compose_pick(
        graph=None,
        lock=[0, 1],
        area_cand=0.85,
        area_orig=1.0,
        void_cand=2,
        void_orig=1,
        cc_n2=0,
        lex_better=False,
        join_prefer=True,
        count_cand=10,
        count_orig=10,
        telem=telem,
    )
    assert int(telem.get("hybrid_pick_join_soft", 0)) == 1
    # Soft floor rejects count regression.
    assert not hybrid_compose_pick(
        graph=None,
        lock=[0, 1],
        area_cand=0.85,
        area_orig=1.0,
        void_cand=2,
        void_orig=1,
        cc_n2=0,
        lex_better=False,
        join_prefer=True,
        count_cand=8,
        count_orig=10,
        telem={},
    )
    assert not hybrid_compose_pick(
        graph=None,
        lock=[0, 1],
        area_cand=0.85,
        area_orig=1.0,
        void_cand=2,
        void_orig=1,
        cc_n2=0,
        lex_better=False,
        join_prefer=False,
        telem={},
    )
    assert hybrid_compose_pick(
        graph=None,
        lock=[0, 1],
        area_cand=0.91,
        area_orig=1.0,
        void_cand=1,
        void_orig=1,
        cc_n2=1,
        lex_better=False,
        join_prefer=True,
        telem={},
    )


def test_hybrid_compose_pick_unlocked_void():
    from nest_graph.propose.motif_lock import hybrid_compose_pick

    telem: dict = {}
    assert hybrid_compose_pick(
        graph=None,
        lock=[],
        lock_len=0,
        unlocked_void=True,
        area_cand=0.89,
        area_orig=1.0,
        void_cand=3,
        void_orig=1,
        cc_n2=0,
        lex_better=False,
        telem=telem,
    )
    assert int(telem.get("hybrid_pick_wins", 0)) == 1
    assert not hybrid_compose_pick(
        graph=None,
        lock=[],
        lock_len=0,
        unlocked_void=False,
        area_cand=0.89,
        area_orig=1.0,
        void_cand=3,
        void_orig=1,
        cc_n2=0,
        lex_better=False,
        telem={},
    )
