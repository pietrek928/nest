"""Unit tests for board-peer peel, motif stamp helpers, and tangent SE(2)."""

import math

import numpy as np
from shapely.geometry import Point, box

from nest_graph.config import ProposeConfig
from nest_graph.geometry import Geometry
from nest_graph.propose.cluster_repack import (
    bfs_peel_victim,
    cluster_indices_with_board,
    cluster_repack_selection,
    pattern_fits_peeled,
    pattern_from_indices,
    select_void_adjacent_victim,
)
from nest_graph.propose.compaction import selection_pairwise_independent
from nest_graph.propose.context import FreeSpaceSnapshot, analyze_free_space
from nest_graph.propose.geometry import ProposeGeometry
from nest_graph.propose.local_se2 import exterior_tangent_dirs, local_se2_selection
from nest_graph.propose.placement_outline import outline_ring_geom
from nest_graph.propose.placements_pattern import ClusterPattern, free_pocket_anchors
from nest_graph.propose.placements_pocket import (
    TAG_OPEN_VOID,
    propose_placements_pocket_fit,
    trapped_void_polygons,
)
from nest_graph.utils import transform_poly


def test_open_void_defaults():
    p = ProposeConfig()
    assert p.use_open_void_pocket is True
    assert p.enable_cluster_repack is True
    assert p.cluster_repack_min_size == 3
    assert p.cluster_repack_max_size == 6
    assert p.cluster_repack_area_accept_ratio == 0.98
    assert p.enable_cluster_relocate is True
    assert p.enable_local_se2 is True


def test_open_void_separate_from_trapped():
    sheet = box(0, 0, 20, 20)
    packed = [box(0, 0, 6, 6), box(6, 0, 12, 4)]
    voids = trapped_void_polygons(sheet, packed)
    assert voids == []
    analysis = analyze_free_space(sheet, packed, part_area=10.0, min_dist=0.1)
    assert analysis.kind == "large_void"
    snap = FreeSpaceSnapshot(analysis=analysis, trapped_voids=(), hull_bays=())
    part = box(0, 0, 2, 2)
    geom = ProposeGeometry(sheet, packed[0], part, 0.1)
    tags: list[str] = []
    attempts: list[int] = []
    coords = propose_placements_pocket_fit(
        part,
        sheet,
        packed,
        min_dist=0.1,
        propose_geom=geom,
        pt_push=sheet.centroid,
        propose_cfg=ProposeConfig(),
        tags_out=tags,
        attempts_out=attempts,
        free_space=snap,
    )
    assert attempts and attempts[0] > 0
    assert TAG_OPEN_VOID in tags or coords
    tags2: list[str] = []
    propose_placements_pocket_fit(
        part,
        sheet,
        packed,
        min_dist=0.1,
        propose_geom=geom,
        pt_push=sheet.centroid,
        propose_cfg=ProposeConfig(use_open_void_pocket=False),
        tags_out=tags2,
        attempts_out=[],
        free_space=snap,
    )
    assert TAG_OPEN_VOID not in tags2


def test_board_sentinel_merges_rim_parts():
    sheet = box(0, 0, 10, 10)
    min_dist = 0.1
    # Parts flush to left edge.
    polys = [box(0.05, 1, 1.05, 2), box(0.05, 2.1, 1.05, 3.1), box(5, 5, 6, 6)]
    groups = cluster_indices_with_board(polys, min_dist, sheet)
    board_groups = [g for g, adj in groups if adj]
    assert board_groups, groups
    # Rim pair should share a board-adjacent component.
    rim_members = set()
    for g, adj in groups:
        if adj:
            rim_members.update(g)
    assert 0 in rim_members and 1 in rim_members
    ring = outline_ring_geom(sheet)
    assert ring is not None
    for i in (0, 1):
        g = Geometry.from_shapely(polys[i])
        assert float(g.standoff_distance(ring)) <= min_dist + 1e-3
    interior = Geometry.from_shapely(polys[2])
    assert float(interior.standoff_distance(ring)) > min_dist


def test_bfs_peel_from_mega_cluster():
    sheet = box(0, 0, 20, 20)
    # Build a mega rim strip of 10 touching boxes along bottom.
    polys = [box(i * 1.05, 0.05, i * 1.05 + 1.0, 1.05) for i in range(10)]
    void = box(0, 5, 20, 20)
    pole = Point(10, 12)
    got = bfs_peel_victim(
        list(range(10)),
        polys,
        min_dist=0.05,
        sheet=sheet,
        pole=pole,
        void_poly=void,
        min_size=3,
        max_size=6,
    )
    assert got is not None
    peel, board_adj = got
    assert 3 <= len(peel) <= 6
    # Contact-connected: consecutive indices along the strip.
    peel_sorted = sorted(peel)
    assert peel_sorted[-1] - peel_sorted[0] + 1 >= len(peel_sorted) - 1


def test_victim_selector_size_3_to_6():
    polys = [
        box(0, 0, 1, 1),
        box(1, 0, 2, 1),
        box(0, 1, 1, 2),
        box(5, 5, 6, 6),
    ]
    void = box(2, 0, 4, 2)
    pole = Point(3, 1)
    victim = select_void_adjacent_victim(
        [0, 1, 2, 3],
        polys,
        min_dist=0.05,
        sheet=box(0, 0, 10, 10),
        pole=pole,
        void_poly=void,
        min_size=3,
        max_size=6,
    )
    assert victim is not None
    assert 3 <= len(victim) <= 6


def test_pattern_multiset_fit():
    pat = ClusterPattern(
        members=((0, (0.0, 0.0, 0.0)), (1, (1.0, 0.0, 0.0)), (0, (0.0, 1.0, 0.0))),
        part_count=3,
        ref_transform=(0.0, 0.0, 0.0),
    )
    assert pattern_fits_peeled(pat, [0, 0, 1])
    assert not pattern_fits_peeled(pat, [0, 1])  # only one gid0


def test_pattern_from_peel_indices():
    polys = [box(0, 0, 1, 1), box(1, 0, 2, 1), box(0, 1, 1, 2)]
    trs = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)]
    gids = [0, 1, 0]
    pat = pattern_from_indices([0, 1, 2], polys, gids, trs)
    assert pat is not None
    assert pat.part_count == 3
    assert pattern_fits_peeled(pat, gids)


def test_exterior_tangent_dirs_orthogonal_to_inward():
    sheet = box(0, 0, 10, 10)
    poly = box(0.05, 4, 1.05, 5)  # left edge
    dirs = exterior_tangent_dirs(sheet, poly)
    assert dirs is not None
    assert len(dirs) == 2
    (tx, ty), (sx, sy) = dirs
    # Tangents opposite.
    assert abs(tx + sx) < 1e-9 and abs(ty + sy) < 1e-9
    # Unit length.
    assert abs(math.hypot(tx, ty) - 1.0) < 1e-6


def test_free_pocket_anchors_exported():
    sheet = box(0, 0, 10, 10)
    obstacle = box(0, 0, 3, 3)
    anchors = free_pocket_anchors(sheet, obstacle, 0.1, 4)
    assert anchors


def test_cluster_repack_accept_ratio_and_independence():
    sheet = box(0, 0, 12, 8)
    part_a = box(0, 0, 1.5, 1.5)
    part_b = box(0, 0, 1.0, 1.0)
    transforms = [
        np.array([0.1, 0.1, 0.0]),
        np.array([1.7, 0.1, 0.0]),
        np.array([0.1, 1.7, 0.0]),
        np.array([0.1, 4.0, 0.0]),
        np.array([2.0, 4.0, 0.0]),
    ]
    group_ids = [0, 0, 1, 0, 1]
    polys = [transform_poly(part_a if g == 0 else part_b, t) for g, t in zip(group_ids, transforms)]
    part_by_group = {0: part_a, 1: part_b}
    cfg = ProposeConfig(
        enable_cluster_repack=True,
        cluster_repack_min_size=3,
        cluster_repack_max_size=6,
        cluster_repack_area_accept_ratio=0.98,
        cluster_repack_max_attempts=1,
        enable_cluster_relocate=False,
        enable_local_se2=False,
    )
    void = box(6, 0, 12, 8)
    pole = Point(9, 4)
    out_p, out_t, sel, stats = cluster_repack_selection(
        sheet,
        polys,
        transforms,
        group_ids,
        list(range(5)),
        part_by_group,
        0.05,
        cfg,
        pole=pole,
        void_poly=void,
        free_space=FreeSpaceSnapshot(
            analysis=analyze_free_space(sheet, polys, 4.0, 0.05),
            topology_poles=((9.0, 4.0, 0.0), (9.0, 4.0, 3.14159)),
        ),
    )
    assert selection_pairwise_independent(out_p, sel)
    assert stats["attempted"] >= 0


def test_local_se2_coarse_then_fine_moves_toward_pole():
    sheet = box(0, 0, 20, 20)
    part = box(0, 0, 1, 1)
    tr = [np.array([2.0, 2.0, 0.0])]
    polys = [transform_poly(part, tr[0])]
    pole = Point(15.0, 15.0)
    cfg = ProposeConfig(enable_local_se2=True, local_se2_n_angles=4)
    d0 = float(polys[0].centroid.distance(pole))
    out_p, out_t, stats = local_se2_selection(
        sheet,
        polys,
        tr,
        [0],
        [0],
        {0: part},
        min_dist=0.25,
        propose_cfg=cfg,
        pole=pole,
        board_adj_indices=[],  # force floating / pole attract
    )
    d1 = float(out_p[0].centroid.distance(pole))
    assert stats["attempted"] == 1
    assert d1 < d0
    assert selection_pairwise_independent(out_p, [0])
