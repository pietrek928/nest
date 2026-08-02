"""Unit tests for mid-pack free-space analysis and gravity compaction."""

import numpy as np
from shapely.geometry import Polygon, box

from nest_graph.propose.compaction import (
    compact_selection,
    selection_pairwise_independent,
    sheet_gravity_point,
)
from nest_graph.propose.context import analyze_free_space
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


def test_sheet_gravity_is_min_x_plus_y():
    sheet = Polygon([(0, 0), (1.2, 0), (0, 1.1)])
    g = sheet_gravity_point(sheet)
    assert abs(g.x) < 1e-9 and abs(g.y) < 1e-9


def test_compact_selection_reduces_gravity_distance():
    sheet = box(0, 0, 8, 8)
    part = box(0, 0, 1, 1)
    # Two parts far from the gravity corner (0,0), with a gap between them.
    transforms = [
        np.array([5.0, 5.0, 0.0]),
        np.array([5.0, 6.5, 0.0]),
    ]
    polys = [transform_poly(part, t) for t in transforms]
    before = sum(p.centroid.distance(sheet_gravity_point(sheet)) for p in polys)
    out_polys, out_tr = compact_selection(
        sheet,
        polys,
        transforms,
        group_ids=[0, 0],
        selected_indices=[0, 1],
        part_by_group={0: part},
        min_dist=0.15,
    )
    assert selection_pairwise_independent(out_polys, [0, 1], 0.15, require_clearance=True)
    after = sum(p.centroid.distance(sheet_gravity_point(sheet)) for p in out_polys)
    assert after < before - 1e-6
    # Angles preserved.
    assert abs(out_tr[0][2] - transforms[0][2]) < 1e-12
    assert abs(out_tr[1][2] - transforms[1][2]) < 1e-12
