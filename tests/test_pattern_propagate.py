"""Pattern propagation: compactness, archive TTL, pole-first lattice, motif lock."""

from shapely.geometry import Polygon, Point, box

from nest_graph.config import ProposeConfig
from nest_graph.propose.cluster_repack import extract_capped_subpatterns
from nest_graph.propose.motif_lock import (
    LargeVoidMotifPlateau,
    sequential_accept_motif_cohorts,
)
from nest_graph.propose.placements_pattern import (
    ClusterPattern,
    age_accepted_pattern_archive,
    archive_accepted_patterns,
    cluster_pattern_from_indices,
    extract_cluster_patterns,
    merge_cluster_patterns,
    motif_lattice_offsets,
    void_seek_motif_anchors,
)


def _square(x: float, y: float, s: float = 10.0) -> Polygon:
    return box(x, y, x + s, y + s)


def test_compactness_prefers_dense_over_stringy():
    # Dense 2x2 block vs stringy chain — mixed extract should rank dense first.
    dense = [
        _square(0, 0),
        _square(10.5, 0),
        _square(0, 10.5),
        _square(10.5, 10.5),
    ]
    stringy = [
        _square(100, 0),
        _square(112, 0),
        _square(124, 0),
        _square(136, 0),
    ]
    placed = dense + stringy
    gids = [0] * 8
    tr = [
        (0.0, 0.0, 0.0), (10.5, 0.0, 0.0), (0.0, 10.5, 0.0), (10.5, 10.5, 0.0),
        (100.0, 0.0, 0.0), (112.0, 0.0, 0.0), (124.0, 0.0, 0.0), (136.0, 0.0, 0.0),
    ]
    sheet = box(-5, -5, 200, 80)
    patterns = extract_cluster_patterns(
        placed, gids, tr, min_dist=1.0, max_patterns=2, sheet=sheet,
        min_compactness=0.0,
    )
    assert len(patterns) >= 1
    # First pattern should be the dense block (ref near origin, not x≈100).
    assert patterns[0].ref_transform[0] < 50.0


def test_archive_ttl_reset_and_age():
    pat = ClusterPattern(
        members=((0, (0.0, 0.0, 0.0)), (0, (10.0, 0.0, 0.0))),
        part_count=2,
        ref_transform=(0.0, 0.0, 0.0),
    )
    arch = archive_accepted_patterns([], [pat], ttl=3, max_keep=4)
    assert len(arch) == 1
    assert arch[0].ttl_remaining == 3
    assert arch[0].accept_count == 1
    arch = age_accepted_pattern_archive(arch)
    assert arch[0].ttl_remaining == 2
    arch = archive_accepted_patterns(arch, [pat], ttl=3, max_keep=4)
    assert arch[0].ttl_remaining == 3
    assert arch[0].accept_count == 2
    arch = age_accepted_pattern_archive(arch)
    arch = age_accepted_pattern_archive(arch)
    arch = age_accepted_pattern_archive(arch)
    assert arch == []


def test_merge_prefers_contact_then_archive_then_synth():
    contact = [
        ClusterPattern(
            members=((0, (0.0, 0.0, 0.0)), (0, (1.0, 0.0, 0.0))),
            part_count=2,
            ref_transform=(0.0, 0.0, 0.0),
        )
    ]
    archived = [
        ClusterPattern(
            members=((1, (0.0, 0.0, 0.0)), (1, (0.0, 2.0, 0.0))),
            part_count=2,
            ref_transform=(0.0, 0.0, 0.0),
        )
    ]
    synth = [
        ClusterPattern(
            members=((2, (0.0, 0.0, 0.0)), (2, (3.0, 0.0, 0.0))),
            part_count=2,
            ref_transform=(0.0, 0.0, 0.0),
        )
    ]
    merged = merge_cluster_patterns(
        contact, synth, max_patterns=2, archived=archived,
    )
    assert len(merged) == 2
    assert merged[0].members[0][0] == 0
    assert merged[1].members[0][0] == 1


def test_lattice_offsets_and_pole_sort_top_k():
    pat = ClusterPattern(
        members=((0, (0.0, 0.0, 0.0)), (0, (20.0, 0.0, 0.0))),
        part_count=2,
        ref_transform=(50.0, 50.0, 0.0),
    )
    offs = motif_lattice_offsets(pat)
    assert (20.0, 0.0) in offs
    assert (-20.0, 0.0) in offs
    sheet = box(0, 0, 200, 200)
    cfg = ProposeConfig(
        enable_motif_lattice=True,
        motif_lattice_depth=2,
        motif_lattice_top_k=4,
        enable_motif_mirror_anchors=False,
        cluster_copy_anchor_seeds=1,
    )
    pole = Point(100.0, 50.0)
    anchors = void_seek_motif_anchors(
        sheet,
        Polygon(),
        min_dist=1.0,
        propose_cfg=cfg,
        void_pole=pole,
        patterns=[pat],
    )
    # Pole seeds + lattice truncated; no AABB mirrors.
    assert len(anchors) <= 20
    stats: dict = {}
    void_seek_motif_anchors(
        sheet,
        Polygon(),
        min_dist=1.0,
        propose_cfg=cfg,
        void_pole=pole,
        patterns=[pat],
        lattice_stats_out=stats,
    )
    assert int(stats.get("lattice_anchors_kept", 0)) <= 4


def test_nest_by_scores_keeps_locks():
    from nest_graph.elem_graph import (
        Circle,
        ElemGraph,
        SelectMode,
        SelectOptions,
        Vec2,
        nest_by_scores,
    )

    g = ElemGraph()
    for i, x in enumerate((0.0, 1.0, 2.0, 4.0)):
        g.append_elem(0, Vec2(x=x, y=0.0), Circle.from_center_radius(x, 0.0, 0.1))
    g.add_collision_pair(0, 1)
    g.add_collision_pair(1, 2)
    opts = SelectOptions()
    opts.mode = SelectMode.greedy_score
    opts.local_swap = False
    opts.locked_indices = [3]
    out = nest_by_scores(g, [10.0, 9.0, 8.0, 1.0], opts)
    assert 3 in out
    assert 0 in out
    assert 1 not in out


def test_sequential_accept_skips_missing_followers():
    class _G:
        collisions = [[], [], []]

    geoms = [
        _square(0, 0),
        _square(12, 0),
        _square(24, 0),
    ]
    # Only one member present → skip (need ≥2 in-graph).
    cohorts = [{
        "leader_key": (0.0, 0.0, 0.0),
        "leader_gid": 0,
        "member_keys": [
            (0, (0.0, 0.0, 0.0)),
            (0, (99.0, 0.0, 0.0)),
            (0, (98.0, 0.0, 0.0)),
        ],
    }]
    locked, telem = sequential_accept_motif_cohorts(
        graph=_G(),
        scores=[1.0, 1.0, 1.0],
        group_id=[0, 0, 0],
        transform=[(0.0, 0.0, 0.0), (12.0, 0.0, 0.0), (24.0, 0.0, 0.0)],
        cohorts=cohorts,
        candidate_geoms=geoms,
        void_geoms=[],
        packed_geoms=[],
        min_dist=1.0,
        pole=Point(0, 0),
        max_accept=3,
    )
    assert locked == []
    assert telem["motif_sequential_skipped_missing"] == 1


def test_sequential_accept_partial_when_some_missing():
    class _G:
        collisions = [[], [], []]

    geoms = [_square(0, 0), _square(12, 0), _square(24, 0)]
    cohorts = [{
        "leader_key": (0.0, 0.0, 0.0),
        "leader_gid": 0,
        "member_keys": [
            (0, (0.0, 0.0, 0.0)),
            (0, (12.0, 0.0, 0.0)),
            (0, (99.0, 0.0, 0.0)),
        ],
    }]
    locked, telem = sequential_accept_motif_cohorts(
        graph=_G(),
        scores=[1.0, 1.0, 1.0],
        group_id=[0, 0, 0],
        transform=[(0.0, 0.0, 0.0), (12.0, 0.0, 0.0), (24.0, 0.0, 0.0)],
        cohorts=cohorts,
        candidate_geoms=geoms,
        void_geoms=[],
        packed_geoms=[],
        min_dist=1.0,
        pole=Point(0, 0),
        max_accept=3,
    )
    assert set(locked) == {0, 1}
    assert telem["motif_sequential_full"] == 1
    assert telem["motif_sequential_partial"] == 1


def test_sequential_accept_full_motif_growing_clear():
    class _G:
        collisions = [[], []]

    geoms = [_square(0, 0), _square(12, 0)]
    cohorts = [{
        "leader_key": (0.0, 0.0, 0.0),
        "leader_gid": 0,
        "member_keys": [
            (0, (0.0, 0.0, 0.0)),
            (0, (12.0, 0.0, 0.0)),
        ],
    }]
    locked, telem = sequential_accept_motif_cohorts(
        graph=_G(),
        scores=[2.0, 1.0],
        group_id=[0, 0],
        transform=[(0.0, 0.0, 0.0), (12.0, 0.0, 0.0)],
        cohorts=cohorts,
        candidate_geoms=geoms,
        void_geoms=[],
        packed_geoms=[],
        min_dist=1.0,
        pole=Point(0, 0),
        max_accept=3,
    )
    assert set(locked) == {0, 1}
    assert telem["motif_sequential_full"] == 1


def test_refine_keeps_independent_locks():
    from nest_graph.elem_graph import (
        ElemGraph,
        RefineSelectionOptions,
        Circle,
        Vec2,
        refine_selection,
    )

    g = ElemGraph()
    for i, x in enumerate((0.0, 2.0, 4.0)):
        g.append_elem(0, Vec2(x=x, y=0.0), Circle.from_center_radius(x, 0.0, 0.1))
    g.add_collision_pair(0, 1)
    scores = [1.0, 100.0, 1.0]
    opts = RefineSelectionOptions()
    opts.max_passes = 8
    opts.seed = 1
    opts.locked_indices = [0, 2]
    refined = refine_selection(g, [0], scores, opts)
    assert 0 in refined
    assert 2 in refined
    assert 1 not in refined


def test_peel_sort_by_compactness_no_hard_min():
    dense = [_square(0, 0), _square(10.5, 0), _square(0, 10.5)]
    stringy = [_square(100, 0), _square(115, 0), _square(130, 0)]
    placed = dense + stringy
    gids = [0] * 6
    tr = [
        (0.0, 0.0, 0.0), (10.5, 0.0, 0.0), (0.0, 10.5, 0.0),
        (100.0, 0.0, 0.0), (115.0, 0.0, 0.0), (130.0, 0.0, 0.0),
    ]
    sheet = box(-5, -5, 200, 80)
    pats = extract_capped_subpatterns(
        placed, gids, tr, min_dist=1.0, max_members=3, sheet=sheet, max_patterns=2,
    )
    assert pats
    shared = cluster_pattern_from_indices([0, 1, 2], placed, gids, tr)
    assert shared is not None
    assert shared.part_count == 3


def test_large_void_motif_plateau_q27():
    tr = LargeVoidMotifPlateau(flat_iters=3, cov_eps=1.0)
    assert not tr.update(free_kind="swiss_cheese", cov=50.0, cluster_copy_refine=2)
    assert not tr.update(free_kind="large_void", cov=40.0, cluster_copy_refine=1)
    assert not tr.update(free_kind="large_void", cov=40.4, cluster_copy_refine=1)
    assert not tr.update(free_kind="large_void", cov=40.5, cluster_copy_refine=1)
    assert tr.update(free_kind="large_void", cov=40.6, cluster_copy_refine=2)
    assert tr.ready
    # Reset on non-large_void.
    assert not tr.update(free_kind="full", cov=40.6, cluster_copy_refine=2)
    assert not tr.ready
