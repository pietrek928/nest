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


def test_motif_base_ttl_reset_and_age():
    from nest_graph.graph import MotifBase, MotifRecord, Se2

    base = MotifBase()
    r = MotifRecord()
    r.gid_a = 0
    r.gid_b = 1
    r.relative = Se2(10.0, 0.0, 0.0)
    r.gci = 0.8
    r.compactness = 0.9
    r.area_a = 2.0
    r.area_b = 1.0
    assert base.upsert(r, 0.0, 3) == 0
    assert base.at(0).ttl_remaining == 3
    assert base.at(0).accept_count == 1
    assert base.age(1) == 0
    assert base.at(0).ttl_remaining == 2
    assert base.upsert(r, 0.0, 3) == 0
    assert base.at(0).ttl_remaining == 3
    assert base.at(0).accept_count == 2
    assert base.age(1) == 0
    assert base.at(0).ttl_remaining == 2
    assert base.age(1) == 0
    assert base.at(0).ttl_remaining == 1
    # Q113: accept_count > 0 floors TTL at 1 (proven motifs are not dropped by age).
    assert base.age(1) == 0
    assert base.size() == 1
    assert base.at(0).ttl_remaining == 1


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
    from nest_graph.graph import (
        Circle,
        PoseGraph,
        SelectMode,
        SelectOptions,
        Vec2,
        nest_by_scores,
    )

    g = PoseGraph()
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


def test_sequential_accept_partial_scene_subset():
    """Q17/Q262 hybrid: Scene locks ≥2 present members, skip graze failures."""
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


def test_w1_packed_penetrating_accepts_touch_rejects_overlap():
    """W1 first_packed: hybrid factory still accepts touch, rejects overlap."""
    from nest_graph.geometry import Geometry
    from nest_graph.propose.motif_lock import _motif_grow_accept_factory

    packed = [Geometry.from_shapely(_square(0, 0))]
    touch = Geometry.from_shapely(_square(10.5, 0))
    overlap = Geometry.from_shapely(_square(5, 0))
    telem: dict = {}
    accept = _motif_grow_accept_factory(
        [], 1.0, packed_penetrating=True, touch_telem=telem,
    )
    assert accept(touch, packed)
    assert int(telem.get("grow_member_touch_n", 0) or 0) >= 1
    assert not accept(overlap, packed)


def test_partition_packed_for_grow_classify_separates_mapped():
    """P0/P2: rebuilt poses in the graph key map → mapped; others → unmapped."""
    from nest_graph.propose.motif_lock import partition_packed_for_grow_classify

    seed = [_square(100, 100)]
    rebuilt = [_square(0, 0), _square(50, 0), _square(90, 0)]
    seeds, mapped, unmapped = partition_packed_for_grow_classify(
        seed_geoms=seed,
        rebuilt_geoms=rebuilt,
        packed_group_id=[0, 0, 1],
        packed_transform=[
            (0.0, 0.0, 0.0),
            (50.0, 0.0, 0.0),
            (90.0, 0.0, 0.0),
        ],
        group_id=[0, 0],
        transform=[(0.0, 0.0, 0.0), (12.0, 0.0, 0.0)],
    )
    assert len(seeds) == 1
    assert len(mapped) == 1  # (0, 0,0,0) hits graph
    assert len(unmapped) == 2  # (50,0) and gid=1 pose not in graph


def test_sequential_grow_hard_only_skips_soft_mapped():
    """P2: mapped soft incumbents are not grow obstacles; seeds stay hard."""
    from nest_graph.geometry import Geometry

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
    # Soft mapped overlaps candidate 0 — would block if treated as hard.
    soft_mapped = [Geometry.from_shapely(_square(0, 0))]
    hard_seed = [Geometry.from_shapely(_square(200, 200))]
    locked, telem = sequential_accept_motif_cohorts(
        graph=_G(),
        scores=[2.0, 1.0],
        group_id=[0, 0],
        transform=[(0.0, 0.0, 0.0), (12.0, 0.0, 0.0)],
        cohorts=cohorts,
        candidate_geoms=geoms,
        void_geoms=[],
        packed_geoms=hard_seed + soft_mapped,
        min_dist=1.0,
        pole=Point(0, 0),
        max_accept=3,
        classify_seed=hard_seed,
        classify_mapped=soft_mapped,
        classify_unmapped=[],
    )
    assert set(locked) == {0, 1}
    assert telem["motif_sequential_full"] == 1
    assert int(telem.get("soft_incumbent_n", 0) or 0) == 1


def test_motif_coemit_foreign_packing_clear():
    """P1: foreign member rejected on packed overlap; clear → foreign_out + multi-gid cohort."""
    from nest_graph.propose.geometry import ProposeGeometry
    from nest_graph.propose.placements_pattern import stamp_motif_leader_follower
    from nest_graph.geometry import Geometry
    from nest_graph.utils import transform_row_key

    sheet = box(0, 0, 40, 40)
    part0 = _square(0, 0, 2.0)
    part1 = _square(0, 0, 2.0)
    # Packed obstacle blocks foreign pose only (leader at anchor stays clear).
    packed_obs = [Geometry.from_shapely(_square(17, 10, 2.0))]
    geom = ProposeGeometry(
        sheet,
        box(0, 0, 0.1, 0.1),
        part0,
        0.05,
        propose_cfg=ProposeConfig(),
        full_packed_geoms=packed_obs,
    )
    pat = ClusterPattern(
        members=(
            (0, (0.0, 0.0, 0.0)),
            (1, (12.0, 0.0, 0.0)),
        ),
        part_count=2,
        ref_transform=(0.0, 0.0, 0.0),
    )
    part_by_group = {0: part0, 1: part1}
    foreign_blocked: dict[int, list] = {}
    skips_blocked: dict[str, int] = {}
    cohorts_blocked: list = []
    out_blocked = stamp_motif_leader_follower(
        [pat],
        0,
        part0,
        propose_geom=geom,
        anchors=[(5.0, 10.0, 0.0)],
        top_n=8,
        part_by_group=part_by_group,
        skip_reasons=skips_blocked,
        cohorts_out=cohorts_blocked,
        foreign_out=foreign_blocked,
    )
    # Full motif fails (foreign overlaps packed) → fallback leader only, no coemit.
    assert 1 not in foreign_blocked or not foreign_blocked[1]
    assert int(skips_blocked.get("coemit_followers", 0) or 0) == 0
    assert not any(
        len({gid for gid, _ in c.get("member_keys") or ()}) > 1
        for c in cohorts_blocked
    )
    assert any(abs(r[0] - 5.0) < 0.2 and abs(r[1] - 10.0) < 0.2 for r in out_blocked)

    # R1 soft schedule: omit nest-packed from foreign obs → coemit despite packed_obs.
    foreign_soft: dict[int, list] = {}
    skips_soft: dict[str, int] = {}
    cohorts_soft: list = []
    out_soft = stamp_motif_leader_follower(
        [pat],
        0,
        part0,
        propose_geom=geom,
        anchors=[(5.0, 10.0, 0.0)],
        top_n=8,
        part_by_group=part_by_group,
        skip_reasons=skips_soft,
        cohorts_out=cohorts_soft,
        foreign_out=foreign_soft,
        hard_packed_geoms=[],
    )
    assert int(skips_soft.get("soft_foreign_incumbent_n", 0) or 0) >= 1
    assert int(skips_soft.get("coemit_followers", 0) or 0) >= 1
    assert 1 in foreign_soft and foreign_soft[1]
    assert any(
        len({gid for gid, _ in c.get("member_keys") or ()}) > 1
        for c in cohorts_soft
    )
    assert any(abs(r[0] - 5.0) < 0.2 and abs(r[1] - 10.0) < 0.2 for r in out_soft)

    # Clear sheet: foreign at (10,10)+(12,0)=(22,10) is free.
    geom_clear = ProposeGeometry(
        sheet,
        box(0, 0, 0.1, 0.1),
        part0,
        0.05,
        propose_cfg=ProposeConfig(),
        full_packed_geoms=[],
    )
    foreign_ok: dict[int, list] = {}
    skips_ok: dict[str, int] = {}
    cohorts_ok: list = []
    out_ok = stamp_motif_leader_follower(
        [pat],
        0,
        part0,
        propose_geom=geom_clear,
        anchors=[(10.0, 10.0, 0.0)],
        top_n=8,
        part_by_group=part_by_group,
        skip_reasons=skips_ok,
        cohorts_out=cohorts_ok,
        foreign_out=foreign_ok,
    )
    assert len(out_ok) >= 1
    assert 1 in foreign_ok and len(foreign_ok[1]) >= 1
    assert int(skips_ok.get("coemit_followers", 0) or 0) >= 1
    multi = [
        c for c in cohorts_ok
        if len({int(gid) for gid, _ in (c.get("member_keys") or ())}) >= 2
    ]
    assert multi
    gids = {int(gid) for gid, _ in multi[0]["member_keys"]}
    assert gids == {0, 1}
    foreign_key = transform_row_key(foreign_ok[1][0])
    assert (1, foreign_key) in {
        (int(g), k) for g, k in multi[0]["member_keys"]
    }


def test_refine_keeps_independent_locks():
    from nest_graph.graph import (
        PoseGraph,
        RefineSelectionOptions,
        Circle,
        Vec2,
        refine_selection,
    )

    g = PoseGraph()
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


def test_inject_cohorts_from_patterns_and_dedup():
    from nest_graph.propose.placements_pattern import ClusterPattern
    from nest_graph.propose.pattern_archive import (
        inject_cohorts_from_patterns,
        merge_motif_hits,
    )

    pat = ClusterPattern(
        members=((0, (0.0, 0.0, 0.0)), (1, (5.0, 0.0, 0.0))),
        part_count=2,
        ref_transform=(0.0, 0.0, 0.0),
        motif_id=7,
    )
    group_id = [0, 1, 0, 1]
    transform = [
        (0.0, 0.0, 0.0),
        (5.0, 0.0, 0.0),
        (20.0, 0.0, 0.0),
        (25.0, 0.0, 0.0),
    ]
    stats: dict = {}
    n = inject_cohorts_from_patterns([pat], group_id, transform, stats)
    assert n >= 1
    assert int(stats.get("motif_graph_hit_n", 0)) >= 1
    assert len(stats.get("motif_cohorts") or ()) >= 1
    n_before = len(stats["motif_cohorts"])
    merge_motif_hits(stats, None, list(stats["motif_cohorts"]))
    assert len(stats["motif_cohorts"]) == n_before


def test_motif_keys_cohort_helpers():
    from nest_graph.propose.motif_keys import (
        boost_score_indices,
        cohort_keys_from_cohorts,
        cohort_member_indices,
    )

    cohorts = [{
        "leader_gid": 0,
        "leader_key": (1.23456, 2.34567, 0.0),
        "member_keys": [(0, (1.23456, 2.34567, 0.0)), (1, (5.0, 0.0, 0.0))],
    }]
    keys = cohort_keys_from_cohorts(cohorts)
    assert (1.2346, 2.3457, 0.0) in keys[0]
    lookup = {(0, (1.2346, 2.3457, 0.0)): 3, (1, (5.0, 0.0, 0.0)): 7}
    idxs, missing = cohort_member_indices(cohorts[0], lookup)
    assert idxs == [3, 7]
    assert missing == 0
    scores = [1.0, 2.0, 3.0, 4.0]
    n = boost_score_indices(scores, [1, 3], 0.5)
    assert n == 2
    assert scores[1] == 2.5
    assert scores[3] == 4.5


def test_rank_cohorts_prefers_motif_complete_and_survive():
    """P3: one RCL lex prefers complete MotifBase pairs, then survive_mid."""
    from nest_graph.propose.motif_lock import _rank_cohorts
    from nest_graph.propose.void_selection import pose_key_to_index

    group_id = [0, 0, 1]
    transform = [(0.0, 0.0, 0.0), (12.0, 0.0, 0.0), (50.0, 0.0, 0.0)]
    key_map = pose_key_to_index(group_id, transform)
    scores = [1.0, 1.0, 9.0]
    incomplete = {
        "leader_key": (50.0, 0.0, 0.0),
        "leader_gid": 1,
        "member_keys": [
            (1, (50.0, 0.0, 0.0)),
            (1, (99.0, 0.0, 0.0)),
        ],
        "motif_id": 1,
    }
    complete = {
        "leader_key": (0.0, 0.0, 0.0),
        "leader_gid": 0,
        "member_keys": [
            (0, (0.0, 0.0, 0.0)),
            (0, (12.0, 0.0, 0.0)),
        ],
        "motif_id": 0,
    }
    ranked = _rank_cohorts(
        [incomplete, complete],
        key_map,
        scores,
        Point(100.0, 0.0),
        survive_by_motif={0: 0, 1: 9},
    )
    assert ranked[0]["motif_id"] == 0

    complete_low = dict(complete)
    complete_high = {
        "leader_key": (12.0, 0.0, 0.0),
        "leader_gid": 0,
        "member_keys": [
            (0, (0.0, 0.0, 0.0)),
            (0, (12.0, 0.0, 0.0)),
        ],
        "motif_id": 2,
    }
    ranked2 = _rank_cohorts(
        [complete_low, complete_high],
        key_map,
        [1.0, 1.0, 1.0],
        Point(0.0, 0.0),
        survive_by_motif={0: 1, 2: 5},
    )
    assert ranked2[0]["motif_id"] == 2

    from nest_graph.graph import MotifBase, MotifRecord, Se2

    base = MotifBase()
    rec = MotifRecord()
    rec.gid_a = 0
    rec.gid_b = 1
    rec.relative = Se2(50.0, 0.0, 0.0)
    rec.gci = 0.8
    rec.compactness = 0.9
    rec.area_a = 1.0
    rec.area_b = 1.0
    mid = int(base.upsert(rec, 0.0, 3))
    pair_complete = {
        "leader_key": (0.0, 0.0, 0.0),
        "leader_gid": 0,
        "member_keys": [
            (0, (0.0, 0.0, 0.0)),
            (1, (50.0, 0.0, 0.0)),
        ],
        "motif_id": mid,
    }
    pair_incomplete = {
        "leader_key": (0.0, 0.0, 0.0),
        "leader_gid": 0,
        "member_keys": [
            (0, (0.0, 0.0, 0.0)),
            (0, (12.0, 0.0, 0.0)),
        ],
        "motif_id": mid,
    }
    ranked3 = _rank_cohorts(
        [pair_incomplete, pair_complete],
        key_map,
        [1.0, 1.0, 1.0],
        Point(0.0, 0.0),
        motif_base=base,
        survive_by_motif={},
    )
    assert ranked3[0] is pair_complete
