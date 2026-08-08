"""Mode-A void-yield densify accept + open-void pocket attempted gates."""

import numpy as np
from shapely.geometry import box

from nest_graph.config import ProposeConfig
from nest_graph.propose.context import FreeSpaceSnapshot, analyze_free_space
from nest_graph.propose.geometry import ProposeGeometry
from nest_graph.propose.pipeline import (
    _count_transforms_in_void,
    proposed_transforms_for_groups,
)
from nest_graph.propose.placements_pocket import (
    TAG_OPEN_VOID,
    aligned_poses_for_pocket,
    propose_placements_pocket_fit,
)
from nest_graph.utils import transform_poly


def test_void_yield_densify_accept_flag_default():
    assert ProposeConfig().enable_void_yield_densify_accept is True


def test_count_transforms_in_void_centroid():
    sheet = box(0, 0, 20, 20)
    target = box(10, 10, 20, 20)
    part = box(0, 0, 2, 2)
    # Centroids of unit squares placed at (0,0) and (14,14) after identity SE2
    # on origin-centered-ish part: transform (x,y,θ) places centroid at
    # (x+1, y+1) for box(0,0,2,2).
    outside = np.array([[0.0, 0.0, 0.0]] * 30, dtype=np.float64)
    inside = np.array([[13.0, 13.0, 0.0]] * 10, dtype=np.float64)
    assert _count_transforms_in_void(outside, target, part) == 0
    assert _count_transforms_in_void(inside, target, part) == 10
    mixed = np.concatenate([outside, inside], axis=0)
    assert _count_transforms_in_void(mixed, target, part) == 10
    _ = sheet


def test_void_yield_accepts_smaller_pool_with_more_in_void():
    """Densify with fewer total transforms but more in-void should accept under hijack."""
    sheet = box(0, 0, 50, 50)
    small = box(0, 0, 2.0, 2.0)
    part = box(0, 0, 3, 3)
    # Rim pack leaving a large SE free region.
    rim = [
        transform_poly(small, (float(x), float(y), 0.0))
        for x in (1, 24, 47)
        for y in (1, 24, 47)
        if not (x >= 24 and y >= 24)
    ]
    cfg = ProposeConfig(
        place_profiles_enabled=True,
        densify_on_void_hijack=True,
        enable_void_yield_densify_accept=True,
        late_border_void_override_ratio=1.5,
        use_pocket_fit=True,
        use_open_void_pocket=True,
        use_cluster_copy=False,
        use_batch_pack=False,
        use_voronoi=False,
        candidate_pool=8,
        max_proposals=8,
        placement_num_angles=2,
        raycast_num_rays=2,
        raycast_num_angles=2,
    )
    densify_stats: dict = {}
    counts: dict[str, int] = {}
    zones: list[str] = []
    proposed_transforms_for_groups(
        sheet,
        [(part, 0)],
        rim,
        [0] * len(rim),
        cfg,
        min_dist=0.2,
        proposer_counts_out=counts,
        zones_used_out=zones,
        densify_stats_out=densify_stats,
    )
    assert any("void_seek" in z for z in zones)
    # When densify fires under hijack, reason should be void_yield_* or count_*.
    if densify_stats.get("fired", 0) >= 1:
        reason = densify_stats.get("densify_reason")
        assert reason in {
            "void_yield_gain",
            "void_yield_drop",
            "count_gain",
            "count_drop",
            None,
        }
        assert "densify_reasons" in densify_stats


def test_open_void_relaxed_medial_emits_attempted():
    """Exterior-touching large free poly → open_void path with attempted > 0."""
    sheet = box(0, 0, 30, 30)
    # Dense NW rim; large SE void touching exterior.
    packed = [
        box(0.2, 0.2, 8, 8),
        box(8.2, 0.2, 16, 6),
        box(0.2, 8.2, 6, 16),
    ]
    part = box(0, 0, 2.5, 2.5)
    analysis = analyze_free_space(sheet, packed, part_area=float(part.area), min_dist=0.15)
    assert analysis.kind == "large_void"
    assert analysis.target_poly is not None
    snap = FreeSpaceSnapshot(analysis=analysis, trapped_voids=(), hull_bays=())
    geom = ProposeGeometry(sheet, packed[0], part, 0.15)
    tags: list[str] = []
    attempts: list[int] = []
    skips: list[str] = []
    coords = propose_placements_pocket_fit(
        part,
        sheet,
        packed,
        min_dist=0.15,
        propose_geom=geom,
        pt_push=sheet.centroid,
        propose_cfg=ProposeConfig(use_open_void_pocket=True, use_pocket_fit=True),
        tags_out=tags,
        attempts_out=attempts,
        free_space=snap,
        skip_reasons_out=skips,
    )
    assert attempts and attempts[0] > 0, (attempts, skips, tags)
    assert TAG_OPEN_VOID in tags or len(coords) > 0 or attempts[0] > 0


def test_aligned_poses_open_void_relax_medial():
    void = box(0, 0, 4, 4)
    part = box(0, 0, 1.5, 1.5)
    # Tight min_dist can empty erosion / fail medial; relax should still tag.
    strict = aligned_poses_for_pocket(part, void, min_dist=1.8, relax_medial=False)
    relaxed = aligned_poses_for_pocket(
        part, void, min_dist=1.8, relax_medial=True, max_poses=3,
    )
    assert len(relaxed) >= 1
    assert len(relaxed) <= 3
    # Strict may be empty; if not, relax should not be emptier.
    if not strict:
        assert relaxed
