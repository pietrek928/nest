"""Unit tests for mid-pack free-space analysis and local_se2 gravity gate."""

import numpy as np
from shapely.geometry import Point, box

from nest_graph.config import ProposeConfig
from nest_graph.propose.cluster_repack import cluster_relocate_selection
from nest_graph.propose.context import analyze_free_space
from nest_graph.propose.local_se2 import local_se2_selection
from nest_graph.propose.placement_common import (
    is_pose_clear,
    selection_pairwise_independent,
)
from nest_graph.propose.selection_edit import SelectionEditCtx
from nest_graph.utils import transform_poly


def test_analyze_free_space_large_void():
    sheet = box(0, 0, 10, 10)
    part = box(0, 0, 1, 1)
    packed = [transform_poly(part, (1.0, 1.0, 0.0))]
    info = analyze_free_space(sheet, packed, part_area=1.0, min_dist=0.1)
    assert info.kind == "large_void"
    assert info.max_void_ratio > 2.5
    assert info.target_pt is not None
    assert info.target_poly is not None


def test_analyze_free_space_swiss_cheese():
    sheet = box(0, 0, 3, 3)
    # Near-full obstacle leaves only a thin rim (~2.24 area < 2.5 * part).
    packed = [box(0.2, 0.2, 2.8, 2.8)]
    info = analyze_free_space(sheet, packed, part_area=1.0, min_dist=0.05)
    assert info.kind == "swiss_cheese", info
    assert info.max_void_ratio <= 2.5


def test_gravity_gate_off_skips_floater_pole_pull():
    sheet = box(0, 0, 20, 20)
    part = box(0, 0, 1, 1)
    tr0 = np.array([2.0, 2.0, 0.0])
    polys = [transform_poly(part, tr0)]
    pole = Point(15.0, 15.0)
    cfg = ProposeConfig(
        enable_local_se2=True,
        enable_gravity_compaction=False,
        local_se2_n_angles=4,
    )
    ctx = SelectionEditCtx(
        sheet=sheet,
        polys=list(polys),
        transforms=[tr0.copy()],
        group_ids=[0],
        selected_indices=[0],
        part_by_group={0: part},
        min_dist=0.25,
        propose_cfg=cfg,
        pole=pole,
        board_adj_indices=[],
    )
    out_p, out_t, stats = local_se2_selection(ctx)
    assert stats["attempted"] == 0
    assert abs(out_t[0][0] - tr0[0]) < 1e-12
    assert abs(out_t[0][1] - tr0[1]) < 1e-12
    assert abs(out_p[0].centroid.distance(polys[0].centroid)) < 1e-9


def test_gravity_gate_on_moves_floater_toward_pole():
    sheet = box(0, 0, 20, 20)
    part = box(0, 0, 1, 1)
    tr0 = np.array([2.0, 2.0, 0.0])
    polys = [transform_poly(part, tr0)]
    pole = Point(15.0, 15.0)
    cfg = ProposeConfig(
        enable_local_se2=True,
        enable_gravity_compaction=True,
        local_se2_n_angles=4,
    )
    d0 = float(polys[0].centroid.distance(pole))
    ctx = SelectionEditCtx(
        sheet=sheet,
        polys=list(polys),
        transforms=[tr0.copy()],
        group_ids=[0],
        selected_indices=[0],
        part_by_group={0: part},
        min_dist=0.25,
        propose_cfg=cfg,
        pole=pole,
        board_adj_indices=[],
    )
    out_p, out_t, stats = local_se2_selection(ctx)
    d1 = float(out_p[0].centroid.distance(pole))
    assert d1 < d0
    assert stats["pole_distance_delta"] < 0.0
    assert selection_pairwise_independent(out_p, [0])
    from nest_graph.geometry import Geometry

    cand_g = Geometry.from_shapely(out_p[0])
    assert is_pose_clear(cand_g, [], [], 0.25)


def test_local_se2_uses_nearest_spine_pole():
    sheet = box(0, 0, 24, 24)
    part = box(0, 0, 1, 1)
    tr0 = np.array([2.0, 2.0, 0.0])
    polys = [transform_poly(part, tr0)]
    near = Point(18.0, 2.5)
    far = Point(2.5, 18.0)
    cfg = ProposeConfig(
        enable_local_se2=True,
        enable_gravity_compaction=True,
        local_se2_n_angles=4,
    )
    ctx = SelectionEditCtx(
        sheet=sheet,
        polys=list(polys),
        transforms=[tr0.copy()],
        group_ids=[0],
        selected_indices=[0],
        part_by_group={0: part},
        min_dist=0.25,
        propose_cfg=cfg,
        pole=far,
        poles=[near, far],
        board_adj_indices=[],
    )
    d_near0 = float(polys[0].centroid.distance(near))
    d_far0 = float(polys[0].centroid.distance(far))
    out_p, _out_t, stats = local_se2_selection(ctx)
    d_near1 = float(out_p[0].centroid.distance(near))
    d_far1 = float(out_p[0].centroid.distance(far))
    assert stats["attempted"] == 1
    assert d_near1 < d_near0
    assert (d_near0 - d_near1) > (d_far0 - d_far1)


def test_local_se2_empty_poles_skips_pull():
    sheet = box(0, 0, 20, 20)
    part = box(0, 0, 1, 1)
    tr0 = np.array([2.0, 2.0, 0.0])
    polys = [transform_poly(part, tr0)]
    cfg = ProposeConfig(
        enable_local_se2=True,
        enable_gravity_compaction=True,
        local_se2_n_angles=4,
    )
    ctx = SelectionEditCtx(
        sheet=sheet,
        polys=list(polys),
        transforms=[tr0.copy()],
        group_ids=[0],
        selected_indices=[0],
        part_by_group={0: part},
        min_dist=0.25,
        propose_cfg=cfg,
        pole=Point(15.0, 15.0),
        poles=(),
        board_adj_indices=[],
    )
    out_p, out_t, stats = local_se2_selection(ctx)
    assert stats["attempted"] == 0
    assert abs(out_t[0][0] - tr0[0]) < 1e-12
    assert abs(out_t[0][1] - tr0[1]) < 1e-12
    assert abs(out_p[0].centroid.distance(polys[0].centroid)) < 1e-9


def test_cluster_relocate_uses_nearest_spine_pole():
    sheet = box(0, 0, 24, 24)
    part = box(0, 0, 1, 1)
    trs = [
        np.array([2.0, 2.0, 0.0]),
        np.array([3.15, 2.0, 0.0]),
    ]
    polys = [transform_poly(part, t) for t in trs]
    near = Point(16.0, 2.5)
    far = Point(2.5, 16.0)
    cfg = ProposeConfig(enable_cluster_relocate=True)
    blob0 = polys[0].union(polys[1]).centroid
    d_near0 = float(blob0.distance(near))
    ctx = SelectionEditCtx(
        sheet=sheet,
        polys=list(polys),
        transforms=[t.copy() for t in trs],
        group_ids=[0, 0],
        selected_indices=[0, 1],
        part_by_group={0: part},
        min_dist=0.2,
        propose_cfg=cfg,
        pole=far,
        poles=[near, far],
        board_adj_indices=[],
    )
    out_p, _out_t, stats = cluster_relocate_selection(ctx)
    assert stats["attempted"] >= 1
    blob1 = out_p[0].union(out_p[1]).centroid
    assert float(blob1.distance(near)) < d_near0


def test_cluster_relocate_empty_poles_skips_pull():
    sheet = box(0, 0, 20, 20)
    part = box(0, 0, 1, 1)
    trs = [
        np.array([2.0, 2.0, 0.0]),
        np.array([3.15, 2.0, 0.0]),
    ]
    polys = [transform_poly(part, t) for t in trs]
    cfg = ProposeConfig(enable_cluster_relocate=True)
    ctx = SelectionEditCtx(
        sheet=sheet,
        polys=list(polys),
        transforms=[t.copy() for t in trs],
        group_ids=[0, 0],
        selected_indices=[0, 1],
        part_by_group={0: part},
        min_dist=0.2,
        propose_cfg=cfg,
        pole=Point(15.0, 15.0),
        poles=(),
        board_adj_indices=[],
    )
    out_p, out_t, stats = cluster_relocate_selection(ctx)
    assert stats["attempted"] == 0
    assert abs(out_t[0][0] - trs[0][0]) < 1e-12
    assert abs(out_p[0].centroid.distance(polys[0].centroid)) < 1e-9
