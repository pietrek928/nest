"""Unified void-fill SoT: motif leader-follower, densify lex, free_space_cloud, allow_repack."""

import inspect

import numpy as np
from shapely.geometry import Point, box

from nest_graph.build_graph import archive_void_elite_transforms, void_elite_count
from nest_graph.config import ProposeConfig
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
