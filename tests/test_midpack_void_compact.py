from nest_graph.graph import FreeKind
"""Unit tests for mid-pack free-space analysis and local_se2 gravity gate."""

import numpy as np
from shapely.geometry import Point, box

import pytest

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
from nest_graph.free_kind_util import is_large_void


def test_analyze_free_space_large_void():
    sheet = box(0, 0, 10, 10)
    part = box(0, 0, 1, 1)
    packed = [transform_poly(part, (1.0, 1.0, 0.0))]
    info = analyze_free_space(sheet, packed, part_area=1.0, min_dist=0.1)
    assert is_large_void(info)
    assert info.max_void_ratio > 2.5
    assert info.target_pt is not None
    assert info.target_poly is not None
    assert info.fits_part is True


def test_void_fits_part_phantom_sliver():
    from nest_graph.propose.context import void_fits_part

    # Long thin corridor: large area ratio but tiny inradius vs equal-area disk.
    sheet = box(0, 0, 20, 0.4)
    assert not void_fits_part(sheet, part_area=4.0, min_dist=0.05)
    fat = box(0, 0, 10, 10)
    assert void_fits_part(fat, part_area=1.0, min_dist=0.05)


def test_analyze_free_space_swiss_cheese():
    sheet = box(0, 0, 3, 3)
    # Near-full obstacle leaves only a thin rim (~2.24 area < 2.5 * part).
    packed = [box(0.2, 0.2, 2.8, 2.8)]
    info = analyze_free_space(sheet, packed, part_area=1.0, min_dist=0.05)
    assert info.kind == FreeKind.SwissCheese, info
    assert info.max_void_ratio <= 2.5


def test_swiss_fit_shape_uses_part_extents_not_disk():
    """Kind stays mean-area Swiss; placeability uses rem0 part_extents."""
    sheet = box(0, 0, 3, 3)
    packed = [box(0.2, 0.2, 2.8, 2.8)]
    rem0 = box(0.0, 0.0, 0.08, 0.08)
    info_disk = analyze_free_space(
        sheet, packed, part_area=1.0, min_dist=0.01
    )
    info_shape = analyze_free_space(
        sheet, packed, part_area=1.0, min_dist=0.01, fit_shape=rem0
    )
    assert info_shape.kind == FreeKind.SwissCheese
    assert info_disk.kind == FreeKind.SwissCheese
    # Disk need for area=1 is ~0.56; rem0 max extent is 0.08.
    assert info_disk.fit_need_r > 0.5
    assert info_shape.fit_need_r == pytest.approx(0.08, rel=1e-6)
    assert info_shape.fits_part is True
    assert info_shape.target_pt is not None


def test_fit_any_component_not_only_largest():
    """A smaller placeable pocket wins target over a larger thin crescent."""
    from shapely.ops import unary_union

    sheet = box(0.0, 0.0, 10.0, 10.0)
    # Leave a thin bottom strip (large area, tiny inradius) and a square pocket.
    obs = unary_union(
        [
            box(0.0, 0.35, 10.0, 7.5),
            box(0.0, 7.5, 7.5, 10.0),
            box(9.2, 7.5, 10.0, 10.0),
        ]
    )
    packed = [obs]
    rem0 = box(0.0, 0.0, 0.6, 0.6)
    info = analyze_free_space(
        sheet, packed, part_area=1.0, min_dist=0.02, fit_shape=rem0
    )
    assert info.fits_part is True
    assert info.target_pt is not None
    # Pocket is in the top-right ~[7.5,9.2]x[7.5,10]; not the thin strip center.
    assert float(info.target_pt.y) > 7.0


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
