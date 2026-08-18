"""Unified void-fill SoT: motif leader-follower, densify lex, free_space_cloud, allow_repack."""

import inspect

import numpy as np
from shapely.geometry import Point, box

from nest_graph.build_graph import archive_void_elite_transforms, void_elite_count
from nest_graph.config import ProposeConfig, subsample_transforms_with_pinned
from nest_graph.propose.geometry import ProposeGeometry
from nest_graph.propose.pipeline import (
    _count_transforms_in_void,
    _count_transforms_pole_near,
    _proposal_key,
    _proposer_enabled,
    allow_void_repack,
)
from nest_graph.propose.placements_free_space_cloud import (
    propose_placements_free_space_cloud,
)
from nest_graph.propose.placements_pattern import (
    ClusterPattern,
    emit_packing_clear,
    stamp_motif_leader_follower,
    void_seek_motif_anchors,
)
from nest_graph.propose.void_selection import void_pole_near_radius
from nest_graph.proposer_names import PlaceZone, ProposerName, ZONE_PROPOSERS


def test_allow_void_repack_predicate():
    assert allow_void_repack(free_kind="large_void", n_void_nest=0, n_void_refine=0)
    assert allow_void_repack(free_kind="gap", n_void_nest=5, n_void_refine=2)
    assert not allow_void_repack(free_kind="gap", n_void_nest=2, n_void_refine=2)


def test_allow_repack_fallback_without_void_leak():
    """Parity with build_graph: no leak + non-large_void → False."""
    assert not allow_void_repack(free_kind="gap", n_void_nest=0, n_void_refine=0)
    assert allow_void_repack(free_kind="large_void", n_void_nest=0, n_void_refine=0)


def test_void_seek_includes_free_space_cloud():
    zone = ZONE_PROPOSERS[PlaceZone.VOID_SEEK]
    assert ProposerName.FREE_SPACE_CLOUD in zone
    assert ProposerName.CLUSTER_COPY in zone


def test_stratified_quotas_default_fifteen():
    cfg = ProposeConfig()
    assert cfg.stratified_void_elite_quota == 15
    assert cfg.stratified_history_quota == 15


def test_stamp_motif_leader_follower_fallback(build_graph_config):
    """Full-motif may fail; same-group leader packing-clear still emits."""
    sheet = box(0, 0, 20, 20)
    board = ProposeGeometry(
        sheet,
        box(0, 0, 0.1, 0.1),
        box(0, 0, 1, 1),
        0.05,
        propose_cfg=ProposeConfig(),
        full_packed_geoms=[],
    )
    pat = ClusterPattern(
        members=(
            (0, (0.0, 0.0, 0.0)),
            (1, (3.0, 0.0, 0.0)),
        ),
        part_count=2,
        ref_transform=(5.0, 5.0, 0.0),
    )
    anchors = [(10.0, 10.0, 0.0)]
    skips: dict[str, int] = {}
    out = stamp_motif_leader_follower(
        [pat],
        0,
        box(0, 0, 1, 1),
        propose_geom=board,
        anchors=anchors,
        top_n=8,
        skip_reasons=skips,
    )
    assert len(out) >= 1
    assert any(abs(r[0] - 10.0) < 0.1 and abs(r[1] - 10.0) < 0.1 for r in out)


def test_pole_near_count():
    part = box(0, 0, 1, 1)
    pole = Point(5.0, 5.0)
    arr = np.array([[5.0, 5.0, 0.0], [50.0, 50.0, 0.0]], dtype=np.float64)
    n = _count_transforms_pole_near(
        arr, pole, part, sheet_diag=100.0, near_ratio=0.25,
    )
    assert n == 1
    assert _count_transforms_in_void(arr, box(4, 4, 6, 6), part) == 1


def test_pinned_union_keeps_half_mixer():
    """Densify prefix + old mixer: never replace a larger pool with densify alone."""
    rng = np.random.default_rng(0)
    old = np.array([[float(i), 0.0, 0.0] for i in range(8)], dtype=np.float64)
    densify = np.array([[100.0 + i, 1.0, 0.0] for i in range(3)], dtype=np.float64)
    max_n = 8
    half = max_n // 2
    pinned = densify[: min(len(densify), half)]
    out = subsample_transforms_with_pinned(old, pinned, max_n, rng)
    assert out.shape[0] == max_n
    assert out.shape[0] >= max(1, len(old) // 2)
    keys = {(round(float(r[0]), 4), round(float(r[1]), 4)) for r in out}
    for row in pinned:
        assert (round(float(row[0]), 4), round(float(row[1]), 4)) in keys


def test_void_pole_near_radius_shared():
    assert void_pole_near_radius(100.0, 0.25) == 25.0
    assert void_pole_near_radius(0.0, 0.25) == 0.0
    cfg = ProposeConfig()
    assert abs(
        void_pole_near_radius(80.0, cfg.void_pole_near_diag_ratio) - 20.0
    ) < 1e-12


def test_emit_packing_clear_matches_cloud_filter(build_graph_config):
    sheet = box(0, 0, 20, 20)
    part = box(0, 0, 1, 1)
    cfg = ProposeConfig()
    geom = ProposeGeometry(
        sheet, box(0, 0, 0.1, 0.1), part, 0.1, propose_cfg=cfg, full_packed_geoms=[],
    )
    assert emit_packing_clear(geom, (10.0, 10.0, 0.0))
    assert not emit_packing_clear(geom, (100.0, 100.0, 0.0))


def test_free_space_cloud_emits_in_void(build_graph_config):
    sheet = box(0, 0, 30, 30)
    void = box(10, 10, 25, 25)
    obstacle = box(0, 0, 8, 8)
    part = box(0, 0, 1.5, 1.5)
    cfg = ProposeConfig(use_free_space_cloud=True, free_space_cloud_samples=32)
    geom = ProposeGeometry(
        sheet, obstacle, part, 0.1, propose_cfg=cfg, full_packed_geoms=[],
    )
    out = propose_placements_free_space_cloud(
        void, propose_geom=geom, propose_cfg=cfg, top_n=16,
    )
    assert len(out) > 0
    for x, y, _a in out:
        assert void.covers(Point(x, y))
    keys = {_proposal_key(c) for c in out}
    assert len(keys) == len(out)


def test_proposer_enabled_blocks_cloud_when_excluded():
    enabled = frozenset(
        p.value for p in ZONE_PROPOSERS[PlaceZone.VOID_SEEK]
        if p != ProposerName.FREE_SPACE_CLOUD
    )
    assert not _proposer_enabled("free_space_cloud", enabled)
    assert _proposer_enabled(
        "free_space_cloud", ProposeConfig.proposers_for_place("void_seek"),
    )


def test_void_seek_motif_anchors_prefer_pole():
    sheet = box(0, 0, 20, 20)
    base = box(0, 0, 2, 2)
    cfg = ProposeConfig(cluster_copy_anchor_seeds=4)
    anchors = void_seek_motif_anchors(
        sheet,
        base,
        min_dist=0.2,
        propose_cfg=cfg,
        void_pole=Point(15.0, 15.0),
        patterns=[],
    )
    assert any(abs(a[0] - 15.0) < 1e-6 and abs(a[1] - 15.0) < 1e-6 for a in anchors)


def test_archive_void_elite_still_works():
    free = box(0, 0, 10, 10)
    polys = [box(1, 1, 2, 2), box(3, 3, 4, 4)]
    transforms = [np.array([1.5, 1.5, 0.0]), np.array([3.5, 3.5, 0.1])]
    archive = archive_void_elite_transforms(
        selected_nest=[0, 1],
        selected_refine=[0],
        polys=polys,
        transforms=transforms,
        group_ids=[0, 0],
        free_poly=free,
        scores=[1.0, 5.0],
        max_keep=15,
        enabled=True,
    )
    assert void_elite_count(archive) == 1


def test_cluster_repack_passes_cascade_zone():
    """Contract: pattern_fallback propose wires cascade_zone=zone."""
    from nest_graph.propose import cluster_repack as cr

    src = inspect.getsource(cr.cluster_repack_selection)
    assert "cascade_zone=zone" in src


def test_explorer_emits_free_space_cloud_on_void_seek():
    from nest_graph.propose.context import FreeSpaceAnalysis, FreeSpaceSnapshot
    from nest_graph.propose.pipeline import collect_propose_candidates

    sheet = box(0, 0, 30, 30)
    void = box(10, 10, 25, 25)
    obstacle = box(0, 0, 8, 8)
    part = box(0, 0, 1.5, 1.5)
    cfg = ProposeConfig(
        use_free_space_cloud=True,
        free_space_cloud_samples=32,
        use_voronoi=False,
        use_point_cloud=False,
        use_neighbor_slide=False,
        use_ribbon_seeds=False,
        use_pocket_fit=False,
        use_cluster_copy=False,
        use_guidance_propositions=False,
        candidate_pool=16,
        max_proposals=16,
        placement_num_angles=4,
    )
    geom = ProposeGeometry(
        sheet, obstacle, part, 0.1, propose_cfg=cfg, full_packed_geoms=[],
    )
    snap = FreeSpaceSnapshot(
        analysis=FreeSpaceAnalysis(
            kind="large_void",
            max_void_ratio=5.0,
            largest_area=float(void.area),
            target_poly=void,
        ),
    )
    counts: dict[str, int] = {}
    keys: dict[str, set] = {}
    collect_propose_candidates(
        obstacle,
        part,
        sheet,
        cfg,
        min_dist=0.1,
        pt_push=Point(15, 15),
        propose_geom=geom,
        enabled_proposers=frozenset({"free_space_cloud"}),
        proposer_counts=counts,
        proposer_keys=keys,
        free_space=snap,
        cascade_zone="void_seek",
    )
    assert counts.get("free_space_cloud", 0) > 0
    assert keys.get("free_space_cloud")


def test_densify_reuses_explorer_cloud_without_second_emit():
    from nest_graph.propose.pipeline import _free_space_cloud_coords

    keys = {(1.0, 2.0, 0.0), (3.0, 4.0, 0.1)}
    arr = np.array([[1.0, 2.0, 0.0], [9.0, 9.0, 0.0], [3.0, 4.0, 0.1]])
    cloud, reused = _free_space_cloud_coords(
        collect_cloud_keys=keys,
        arr=arr,
        void_poly=box(0, 0, 10, 10),
        propose_geom=None,  # unused on reuse
        propose_cfg=ProposeConfig(),
        top_n=8,
        allowed_angles=None,
    )
    assert reused is True
    assert {(round(c[0], 4), round(c[1], 4), round(c[2], 4)) for c in cloud} == keys


def test_densify_cloud_fallback_when_collect_ranked_out(monkeypatch):
    from nest_graph.propose.pipeline import _free_space_cloud_coords

    calls = {"n": 0}

    def _fake_cloud(*_a, **_k):
        calls["n"] += 1
        return [(5.0, 5.0, 0.0)]

    monkeypatch.setattr(
        "nest_graph.propose.pipeline.propose_placements_free_space_cloud",
        _fake_cloud,
    )
    cloud, reused = _free_space_cloud_coords(
        collect_cloud_keys={(1.0, 2.0, 0.0)},
        arr=np.array([[9.0, 9.0, 0.0]]),
        void_poly=box(0, 0, 10, 10),
        propose_geom=None,
        propose_cfg=ProposeConfig(),
        top_n=8,
        allowed_angles=None,
    )
    assert reused is False
    assert calls["n"] == 1
    assert cloud == [(5.0, 5.0, 0.0)]


def test_densify_inner_collect_disables_cloud_reemit():
    from nest_graph.propose import pipeline as pl

    src = inspect.getsource(pl._void_seek_densify)
    assert '"use_free_space_cloud": False' in src
    assert "_free_space_cloud_coords" in src
    assert "void_seek_free" in inspect.getsource(pl.proposed_transforms_for_groups)
