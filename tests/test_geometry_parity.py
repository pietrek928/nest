"""Python Geometry↔Shapely parity cases migrated from nest_graph/geometry/tests/."""

import math

import pytest
from shapely.geometry import Polygon, box

from nest_graph.geometry import Geometry, snap_pose_to_ring
from tests.fixtures.shapes import (
    c_shape,
    donut,
    l_shape,
    simple_square,
    square_with_hole,
    unit_square,
)
from tests.geometry_oracle import (
    assert_batch_distances_match_shapely,
    assert_batch_intersects_match_shapely,
    assert_contains_point_matches,
    assert_distance_close,
    assert_intersects_matches_shapely,
    assert_pose_intersects_matches_shapely,
)


def _quad(x0, y0, x1, y1) -> Polygon:
    return box(float(x0), float(y0), float(x1), float(y1))


def _hex_tc3() -> Polygon:
    # From test_distance.cc polygon_tc3_spec_hex (shifted for overlap with 2x2 box).
    return Polygon([(0, 0), (2, -1), (4, 0), (4, 2), (2, 3), (0, 2)])


# --- intersect (test_intersect.cc ideas) ---


@pytest.mark.parametrize(
    "a,b,shapely_hit,packing_hit",
    [
        (_quad(0, 0, 2, 2), _quad(5, 0, 7, 2), False, False),  # disjoint
        (_quad(0, 0, 2, 2), _quad(2, 2, 4, 4), True, False),  # vertex kiss
        (_quad(0, 0, 2, 2), _quad(2, 0, 4, 2), True, False),  # edge kiss
        (_quad(0, 0, 2, 2), _quad(1, 1, 3, 3), True, True),  # overlap
    ],
    ids=["disjoint", "touch_vertex", "touch_edge", "overlap"],
)
def test_intersect_pair_parity(a, b, shapely_hit, packing_hit):
    assert bool(a.intersects(b)) is shapely_hit
    ga, gb = Geometry.from_shapely(a), Geometry.from_shapely(b)
    assert bool(ga.intersects(gb)) is packing_hit
    assert bool(gb.intersects(ga)) is packing_hit
    if packing_hit == shapely_hit:
        assert_intersects_matches_shapely(a, b)
        assert_intersects_matches_shapely(b, a)


def test_intersect_batch_hit_and_miss():
    left = _quad(0, 0, 2, 2)
    right_sep = _quad(10, 0, 12, 2)
    overlap = _quad(1, 1, 3, 3)
    assert_batch_intersects_match_shapely([left, right_sep])
    assert_batch_intersects_match_shapely([left, overlap])


def test_intersect_three_polygons_subset_pairs():
    base = _quad(0, 0, 2, 2)
    orphan = _quad(100, 100, 101, 101)
    buddy = _quad(1, 1, 3, 3)
    assert_batch_intersects_match_shapely([base, orphan, buddy])


def test_intersect_stress_tc1_giant_floor():
    p0 = _quad(-100, -5, 100, 0)
    p1 = _quad(-5, -2, 5, 8)
    p2 = _quad(15, 5, 25, 15)
    p3 = _quad(200, 0, 210, 10)
    assert_batch_intersects_match_shapely([p0, p1, p2, p3])


def test_intersect_stress_tc2_c_trapped_donut():
    trapped = _quad(4, 4, 6, 6)
    core = _quad(24, 4, 26, 6)
    # Fixture donut is 0..10 with hole; C++ uses 20..30 — shift our donut.
    d = donut(outer=10.0, hole_min=2.0, hole_max=8.0)
    d = Polygon(
        [(c[0] + 20, c[1]) for c in d.exterior.coords],
        [[(c[0] + 20, c[1]) for c in d.interiors[0].coords]],
    )
    assert_batch_intersects_match_shapely([c_shape(), trapped, d, core])


def test_intersect_walls_vs_bullets_pairwise():
    wall_l = _quad(0, 0, 1, 10)
    wall_r = _quad(10, 0, 11, 10)
    bullet1 = _quad(0.5, 5, 1.5, 6)
    bullet2 = _quad(5, 5, 6, 6)
    assert_batch_intersects_match_shapely([wall_l, wall_r, bullet1, bullet2])


# --- distance (test_distance.cc ideas) ---


def test_distance_close_separated_squares():
    a = _quad(0, 0, 2, 2)
    b = _quad(5, 0, 7, 2)
    assert_distance_close(a, b)


def test_distance_tc1_floor_pairs():
    p0 = _quad(-100, -5, 100, 0)
    p1 = _quad(-5, -2, 5, 8)
    p2 = _quad(15, 5, 25, 15)
    assert_distance_close(p0, p1)  # intersect / penetration
    assert_distance_close(p0, p2)  # gap 5 → dist 5


def test_distance_tc2_c_gap_and_donut_core():
    trapped = _quad(4, 4, 6, 6)
    assert_distance_close(c_shape(), trapped)
    # Hole-core: assert non-intersect parity (hole distance semantics differ vs Shapely).
    d = donut(outer=10.0, hole_min=2.0, hole_max=8.0)
    d = Polygon(
        [(c[0] + 20, c[1]) for c in d.exterior.coords],
        [[(c[0] + 20, c[1]) for c in d.interiors[0].coords]],
    )
    core = _quad(24, 4, 26, 6)
    assert_intersects_matches_shapely(d, core)
    assert not d.intersects(core)


def test_distance_tc3_hex_overlap():
    assert_distance_close(_quad(0, 0, 2, 2), _hex_tc3())


def test_distance_tc6_collinear_seam_hover():
    tile_l = _quad(-10, -5, 0, 0)
    tile_r = _quad(0, -5, 10, 0)
    hover = _quad(-2, 2, 2, 4)
    assert_distance_close(tile_l, hover)
    assert_distance_close(tile_r, hover)


def test_distance_tc7_kissing_edge():
    a = _quad(0, 0, 2, 2)
    b = _quad(2, 0, 4, 2)
    assert_distance_close(a, b)


def test_distance_batch_planar_seam():
    tile_l = _quad(-10, -5, 0, 0)
    tile_r = _quad(0, -5, 10, 0)
    hover = _quad(-1, 5, 1, 7)  # gap 5 → dist^2 25 in C++ telemetry variant
    assert_batch_distances_match_shapely([tile_l, tile_r, hover], aura=10.0)


def test_distance_swallowed_containment():
    # test_gap_fixes: outer/inner mid-list still reports contact/intersect.
    outer = _quad(0, 0, 10, 10)
    inner = _quad(3, 3, 5, 5)
    left = _quad(-20, 0, -18, 2)
    right = _quad(20, 0, 22, 2)
    assert_batch_distances_match_shapely([left, right, outer, inner], aura=1.0)


# --- transform pose intersect ---


def test_pose_intersect_translate_rotate_parity():
    p1 = unit_square()
    p2 = unit_square()
    assert_pose_intersects_matches_shapely(
        p1, (0.5, 0.0, 0.0), p2, (0.0, 0.0, 0.0),
    )
    assert_pose_intersects_matches_shapely(
        p1, (3.0, 0.0, 0.0), p2, (0.0, 0.0, math.pi / 4),
    )
    assert_pose_intersects_matches_shapely(
        simple_square(), (1.0, 1.0, math.pi / 2), l_shape(), (0.0, 0.0, 0.0),
    )


# --- contains_point ---


@pytest.mark.parametrize(
    "factory,x,y",
    [
        (simple_square, 1.0, 1.0),
        (simple_square, -0.5, 1.0),
        (square_with_hole, 0.5, 0.5),
        (square_with_hole, 2.0, 2.0),  # in hole
        (c_shape, 1.0, 3.0),
        (c_shape, 4.0, 3.0),  # in notch
        (donut, 1.0, 1.0),
        (donut, 5.0, 5.0),  # hole
    ],
    ids=[
        "sq_in", "sq_out", "hole_shell", "hole_void",
        "c_in", "c_notch", "donut_shell", "donut_void",
    ],
)
def test_contains_point_parity(factory, x, y):
    poly = factory()
    assert_contains_point_matches(poly, x, y)


# --- snap_pose_to_ring (bindings exist; mirror test_snap_pose.cc) ---


def test_snap_pose_to_ring_docks_rect_to_board_edge():
    board = Geometry.from_shapely(_quad(0, 0, 10, 10))
    part = Geometry.from_shapely(_quad(0, 0, 1, 0.5))
    ring = board
    min_dist = 0.05
    snapped = snap_pose_to_ring(
        part, ring, (5.0, 0.0), (0.0, 1.0), 0.0, min_dist, board=board,
    )
    assert snapped is not None
    dx, dy, ang = snapped
    assert math.isclose(ang, 0.0, abs_tol=1e-9)
    assert dy > 0.0
    placed = part.rotate(ang).translate(dx, dy)
    gap = placed.standoff_distance(ring)
    assert gap + 1e-5 >= min_dist
    assert placed.footprint_inside(board)


def test_snap_pose_to_ring_rejects_zero_inward():
    board = Geometry.from_shapely(_quad(0, 0, 4, 4))
    part = Geometry.from_shapely(unit_square())
    snapped = snap_pose_to_ring(
        part, board, (2.0, 0.0), (0.0, 0.0), 0.0, 0.1, board=board,
    )
    assert snapped is None
