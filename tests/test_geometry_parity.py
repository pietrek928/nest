"""Python Geometry↔Shapely parity suite (packing Penetrating SoT)."""

import math

import pytest
from shapely.geometry import Point, Polygon, box

from nest_graph.geometry import Geometry, batch_packing_clear, snap_pose_to_ring
from tests.fixtures.shapes import (
    c_shape_10,
    concave_hole_crossing_bar,
    cw_hole_donut,
    donut,
    l_shape,
    multi_unit_squares,
    nest_triangle_board,
    off_center_donut,
    plate_with_l_hole,
    simple_square,
    skinny_rect,
    square_with_hole,
    tessellated_circle,
    unit_square,
)
from tests.geometry_oracle import (
    assert_area_matches_shapely,
    assert_batch_distances_match_shapely,
    assert_batch_packing_collide,
    assert_contains_point_matches,
    assert_distance_close,
    assert_packing_collide,
    assert_pose_packing_collide,
    assert_scene_margin_vs_packing,
    shapely_packing_collide,
)


def _quad(x0, y0, x1, y1) -> Polygon:
    return box(float(x0), float(y0), float(x1), float(y1))


def _hex_tc3() -> Polygon:
    return Polygon([(0, 0), (2, -1), (4, 0), (4, 2), (2, 3), (0, 2)])


# --- construction ---


def test_from_shapely_accepts_point():
    g = Geometry.from_shapely(Point(1.5, 2.5))
    assert g.area() == 0.0
    assert g.contains_point(1.5, 2.5) is False
    assert "POINT" in g.to_string().upper()


def test_from_shapely_outline_drops_holes():
    poly = square_with_hole()
    outlined = Geometry.from_shapely_outline(poly)
    full = Geometry.from_shapely(poly)
    assert outlined.contains_point(2.0, 2.0) is True  # hole ignored → filled
    assert full.contains_point(2.0, 2.0) is False
    with pytest.raises(ValueError, match="no usable exterior rings"):
        Geometry.from_shapely_outline(Point(0, 0))


# --- area ---


@pytest.mark.parametrize(
    "factory",
    [unit_square, simple_square, nest_triangle_board, square_with_hole, donut, multi_unit_squares, cw_hole_donut],
    ids=["unit", "sq2", "tri", "holed4", "donut10", "multi", "cw_hole"],
)
def test_area_matches_shapely(factory):
    poly = factory()
    assert_area_matches_shapely(poly)


def test_cw_hole_stays_subtractive():
    poly = cw_hole_donut()
    g = Geometry.from_shapely(poly)
    assert g.contains_point(3.0, 3.0) is False
    assert_area_matches_shapely(poly, g=g)


# --- centroid / aabb / bounds ---


def test_centroid_convex_triangle_tight():
    poly = nest_triangle_board()
    g = Geometry.from_shapely(poly)
    cx, cy = g.centroid()
    # plane_centroid is vertex-mean of additive parts; for a triangle equals area centroid.
    assert math.isclose(cx, poly.centroid.x, abs_tol=1e-6)
    assert math.isclose(cy, poly.centroid.y, abs_tol=1e-6)


def test_off_center_donut_centroid_not_hole_biased():
    poly = off_center_donut()
    g = Geometry.from_shapely(poly)
    cx, cy = g.centroid()
    # Vertex-mean of outer ring only (subtractive skipped) → near plate center.
    assert cx > 4.0
    assert cy > 4.0


def test_aabb_vs_bounds_skinny_rect():
    poly = skinny_rect()
    g = Geometry.from_shapely(poly)
    ax0, ay0, ax1, ay1 = g.aabb()
    bx0, by0, bx1, by1 = g.bounds()
    assert math.isclose(ax0, poly.bounds[0], abs_tol=1e-9)
    assert math.isclose(ay0, poly.bounds[1], abs_tol=1e-9)
    assert math.isclose(ax1, poly.bounds[2], abs_tol=1e-9)
    assert math.isclose(ay1, poly.bounds[3], abs_tol=1e-9)
    assert bx0 <= ax0 + 1e-12
    assert by0 <= ay0 + 1e-12
    assert bx1 >= ax1 - 1e-12
    assert by1 >= ay1 - 1e-12


# --- intersect ---


@pytest.mark.parametrize(
    "a,b,shapely_hit,packing_hit",
    [
        (_quad(0, 0, 2, 2), _quad(5, 0, 7, 2), False, False),
        (_quad(0, 0, 2, 2), _quad(2, 2, 4, 4), True, False),
        (_quad(0, 0, 2, 2), _quad(2, 0, 4, 2), True, False),
        (_quad(0, 0, 2, 2), _quad(1, 1, 3, 3), True, True),
    ],
    ids=["disjoint", "touch_vertex", "touch_edge", "overlap"],
)
def test_intersect_pair_parity(a, b, shapely_hit, packing_hit):
    assert bool(a.intersects(b)) is shapely_hit
    assert shapely_packing_collide(a, b) is packing_hit
    assert_packing_collide(a, b, expect=packing_hit)


def test_intersect_batch_hit_and_miss():
    left = _quad(0, 0, 2, 2)
    assert_batch_packing_collide([left, _quad(10, 0, 12, 2)])
    assert_batch_packing_collide([left, _quad(1, 1, 3, 3)])
    assert_batch_packing_collide([left, _quad(2, 0, 4, 2)])  # kiss → packing clear


def test_intersect_three_polygons_subset_pairs():
    assert_batch_packing_collide(
        [_quad(0, 0, 2, 2), _quad(100, 100, 101, 101), _quad(1, 1, 3, 3)]
    )


def test_intersect_stress_tc1_giant_floor():
    assert_batch_packing_collide(
        [
            _quad(-100, -5, 100, 0),
            _quad(-5, -2, 5, 8),
            _quad(15, 5, 25, 15),
            _quad(200, 0, 210, 10),
        ]
    )


def test_intersect_stress_tc2_c10_trapped_donut():
    trapped = _quad(4, 4, 6, 6)
    core = _quad(24, 4, 26, 6)
    d = donut(outer=10.0, hole_min=2.0, hole_max=8.0)
    d = Polygon(
        [(c[0] + 20, c[1]) for c in d.exterior.coords],
        [[(c[0] + 20, c[1]) for c in d.interiors[0].coords]],
    )
    assert_batch_packing_collide([c_shape_10(), trapped, d, core])


def test_intersect_walls_vs_bullets_pairwise():
    assert_batch_packing_collide(
        [
            _quad(0, 0, 1, 10),
            _quad(10, 0, 11, 10),
            _quad(0.5, 5, 1.5, 6),
            _quad(5, 5, 6, 6),
        ]
    )


def test_concave_hole_crossing_bar_packing_collide():
    plate = plate_with_l_hole()
    bar = concave_hole_crossing_bar()
    assert shapely_packing_collide(plate, bar) is True
    assert_packing_collide(plate, bar, expect=True)
    probe = _quad(2.5, 2.5, 3.5, 3.5)
    assert shapely_packing_collide(plate, probe) is False
    assert_packing_collide(plate, probe, expect=False)


# --- distance ---


def test_distance_close_separated_squares():
    assert_distance_close(_quad(0, 0, 2, 2), _quad(5, 0, 7, 2))


def test_distance_tc1_floor_pairs():
    p0 = _quad(-100, -5, 100, 0)
    assert_distance_close(p0, _quad(-5, -2, 5, 8))
    assert_distance_close(p0, _quad(15, 5, 25, 15))


def test_distance_tc2_c10_gap_and_donut_core():
    trapped = _quad(4, 4, 6, 6)
    assert_distance_close(c_shape_10(), trapped)
    d = donut(outer=10.0, hole_min=2.0, hole_max=8.0)
    d = Polygon(
        [(c[0] + 20, c[1]) for c in d.exterior.coords],
        [[(c[0] + 20, c[1]) for c in d.interiors[0].coords]],
    )
    core = _quad(24, 4, 26, 6)
    assert_packing_collide(d, core, expect=False)
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
    ga, gb = Geometry.from_shapely(a), Geometry.from_shapely(b)
    assert ga.intersects(gb) is False
    assert ga.distance(gb) <= 1e-4


def test_distance_batch_planar_seam():
    assert_batch_distances_match_shapely(
        [_quad(-10, -5, 0, 0), _quad(0, -5, 10, 0), _quad(-1, 5, 1, 7)],
        aura=10.0,
    )


def test_distance_swallowed_containment():
    assert_batch_distances_match_shapely(
        [
            _quad(-20, 0, -18, 2),
            _quad(20, 0, 22, 2),
            _quad(0, 0, 10, 10),
            _quad(3, 3, 5, 5),
        ],
        aura=1.0,
    )


def test_distance_needle_outside_circle():
    circle = tessellated_circle()
    needle = Polygon([(6.0, -0.1), (10.0, 0.0), (6.0, 0.1)])
    assert_distance_close(circle, needle)


# --- transforms / poses ---


def test_pose_intersect_translate_rotate_parity():
    p1 = unit_square()
    p2 = unit_square()
    assert_pose_packing_collide(p1, (0.5, 0.0, 0.0), p2, (0.0, 0.0, 0.0))
    assert_pose_packing_collide(p1, (3.0, 0.0, 0.0), p2, (0.0, 0.0, math.pi / 4))
    assert_pose_packing_collide(
        simple_square(), (1.0, 1.0, math.pi / 2), l_shape(), (0.0, 0.0, 0.0),
    )


# --- contains_point ---


@pytest.mark.parametrize(
    "factory,x,y",
    [
        (simple_square, 1.0, 1.0),
        (simple_square, -0.5, 1.0),
        (square_with_hole, 0.5, 0.5),
        (square_with_hole, 2.0, 2.0),
        (c_shape_10, 1.0, 5.0),
        (c_shape_10, 5.0, 5.0),
        (donut, 1.0, 1.0),
        (donut, 5.0, 5.0),
    ],
    ids=[
        "sq_in", "sq_out", "hole_shell", "hole_void",
        "c10_in", "c10_pocket", "donut_shell", "donut_void",
    ],
)
def test_contains_point_parity(factory, x, y):
    assert_contains_point_matches(factory(), x, y)


# --- batch_packing_clear ---


def test_batch_packing_clear_matches_scalar():
    part = Geometry.from_shapely(unit_square())
    obs = [Geometry.from_shapely(_quad(3, 0, 5, 2))]
    rows = [
        (0.0, 0.0, 0.0),   # disjoint clear
        (2.0, 0.0, 0.0),   # edge kiss clear
        (2.5, 0.0, 0.0),   # overlap not clear
        (10.0, 0.0, 0.0),
    ]
    flags = list(batch_packing_clear(part, rows, obs))
    for (x, y, th), flag in zip(rows, flags, strict=True):
        placed = part.rotate(th).translate(x, y)
        assert flag is (not any(placed.intersects(o) for o in obs))


def test_batch_packing_clear_angle_cache_neighbors():
    part = Geometry.from_shapely(skinny_rect())
    obs = [Geometry.from_shapely(_quad(0, 3, 100, 5))]
    theta = 0.1
    rows = [
        (0.0, 0.0, theta),
        (0.0, 0.0, theta + 4e-7),
        (0.0, 0.0, theta + 6e-7),
    ]
    flags = list(batch_packing_clear(part, rows, obs))
    for (x, y, th), flag in zip(rows, flags, strict=True):
        placed = part.rotate(th).translate(x, y)
        assert flag is (not any(placed.intersects(o) for o in obs))


def test_batch_packing_clear_holed_obstacle():
    part = Geometry.from_shapely(_quad(0, 0, 0.5, 0.5))
    obs = [Geometry.from_shapely(square_with_hole())]
    # Inside hole void → packing clear; on shell → collide
    rows = [(1.5, 1.5, 0.0), (0.1, 0.1, 0.0)]
    flags = list(batch_packing_clear(part, rows, obs))
    assert flags[0] is True
    assert flags[1] is False


def test_scene_margin_rejects_near_gap():
    cand = _quad(0, 0, 1, 1)
    wall = _quad(1.2, 0, 2.2, 1)  # gap 0.2
    assert_scene_margin_vs_packing(cand, [wall], min_dist=0.5)


# --- snap_pose_to_ring ---


def test_snap_pose_to_ring_docks_rect_to_board_edge():
    board = Geometry.from_shapely(_quad(0, 0, 10, 10))
    part = Geometry.from_shapely(_quad(0, 0, 1, 0.5))
    min_dist = 0.05
    snapped = snap_pose_to_ring(
        part, board, (5.0, 0.0), (0.0, 1.0), 0.0, min_dist,
        board=board, standoff_pad=0.0, dock_outer_extreme=True,
    )
    assert snapped is not None
    dx, dy, ang = snapped
    assert math.isclose(ang, 0.0, abs_tol=1e-9)
    assert dy > 0.0
    placed = part.rotate(ang).translate(dx, dy)
    gap = placed.standoff_distance(board)
    assert gap + 1e-5 >= min_dist
    assert abs(gap - min_dist) <= 0.05 * min_dist + 1e-5
    assert placed.footprint_inside(board)


def test_snap_pose_to_ring_pad0_left_and_top():
    board = Geometry.from_shapely(_quad(0, 0, 10, 10))
    part = Geometry.from_shapely(_quad(0, 0, 1, 0.5))
    min_dist = 0.05
    for contact, inward in (((0.0, 5.0), (1.0, 0.0)), ((5.0, 10.0), (0.0, -1.0))):
        snapped = snap_pose_to_ring(
            part, board, contact, inward, 0.0, min_dist,
            board=board, standoff_pad=0.0, dock_outer_extreme=True,
        )
        assert snapped is not None
        dx, dy, ang = snapped
        placed = part.rotate(ang).translate(dx, dy)
        gap = placed.standoff_distance(board)
        assert abs(gap - min_dist) <= 0.05 * min_dist + 1e-5
        assert placed.footprint_inside(board)


def test_snap_pose_to_ring_rejects_zero_inward():
    board = Geometry.from_shapely(_quad(0, 0, 4, 4))
    part = Geometry.from_shapely(unit_square())
    snapped = snap_pose_to_ring(
        part, board, (2.0, 0.0), (0.0, 0.0), 0.0, 0.1, board=board,
    )
    assert snapped is None
