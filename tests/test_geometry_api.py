import math

import pytest
from shapely.geometry import LineString, Point, Polygon
from shapely.ops import nearest_points

from nest_graph.geometry import (
    Geometry,
    find_all_polygon_casts,
    find_closest_polygon_cast,
    min_distance_pair,
)
from nest_graph.utils import transform_poly


@pytest.fixture
def simple_square():
    return Polygon([(0, 0), (2, 0), (2, 2), (0, 2)])


@pytest.fixture
def square_with_hole():
    outer = [(0, 0), (4, 0), (4, 4), (0, 4)]
    hole = [(1, 1), (3, 1), (3, 3), (1, 3)]
    return Polygon(outer, [hole])


def _square_coords():
    return [(0, 0), (1, 0), (1, 1), (0, 1)]


def test_from_convex_polygon_matches_from_shapely(simple_square):
    g_conv = Geometry.from_convex_polygon(_square_coords())
    g_shapely = Geometry.from_shapely(simple_square)
    probe = Polygon([(0.5, 0.5), (1.5, 0.5), (1.5, 1.5), (0.5, 1.5)])
    other = Geometry.from_shapely(probe)
    assert g_conv.intersects(other) == g_shapely.intersects(other)
    assert g_conv.contains_point(0.5, 0.5) == g_shapely.contains_point(0.5, 0.5)


def test_apply_transform_matches_transform_poly():
    p = Polygon(_square_coords())
    t = (1.2, 0.3, 0.25)
    base = Geometry.from_convex_polygon(_square_coords())
    placed = base.apply_transform(*t)
    shapely_placed = transform_poly(p, t)
    geom_corners = list(placed.vertices())
    if len(geom_corners) > 1 and geom_corners[0] == geom_corners[-1]:
        geom_corners = geom_corners[:-1]
    shapely_corners = list(shapely_placed.exterior.coords)[:-1]
    assert len(geom_corners) == len(shapely_corners)
    for (gx, gy), (sx, sy) in zip(geom_corners, shapely_corners):
        assert math.isclose(gx, sx, abs_tol=1e-5)
        assert math.isclose(gy, sy, abs_tol=1e-5)


def test_center_radius_bounds_consistency():
    base = Geometry.from_convex_polygon(_square_coords())
    cx, cy = base.center()
    r = base.radius()
    xmin, ymin, xmax, ymax = base.bounds()
    assert math.isclose(xmax - xmin, 2 * r, rel_tol=1e-5)
    assert math.isclose(ymax - ymin, 2 * r, rel_tol=1e-5)
    assert math.isclose(0.5 * (xmin + xmax), cx, rel_tol=1e-6)
    assert math.isclose(0.5 * (ymin + ymax), cy, rel_tol=1e-6)
    assert r >= math.hypot(cx - 1, cy - 1)


@pytest.mark.parametrize(
    "x,y,expected",
    [
        (0.5, 0.5, True),
        (3.0, 3.0, False),
    ],
)
def test_contains_point_square(simple_square, x, y, expected):
    g = Geometry.from_shapely(simple_square)
    assert g.contains_point(x, y) == simple_square.contains(Point(x, y))


def test_contains_point_donut(square_with_hole):
    g = Geometry.from_shapely(square_with_hole)
    assert g.contains_point(0.5, 0.5) == square_with_hole.contains(Point(0.5, 0.5))
    assert g.contains_point(2.0, 2.0) == square_with_hole.contains(Point(2.0, 2.0))


@pytest.mark.parametrize(
    "p2_coords,expected_intersect",
    [
        ([(3, 0), (5, 0), (5, 2), (3, 2)], False),
        ([(1, 1), (3, 1), (3, 3), (1, 3)], True),
        ([(2, 0), (4, 0), (4, 2), (2, 2)], True),
    ],
)
def test_intersects_param(simple_square, p2_coords, expected_intersect):
    p2 = Polygon(p2_coords)
    g1 = Geometry.from_shapely(simple_square)
    g2 = Geometry.from_shapely(p2)
    assert g1.intersects(g2) == simple_square.intersects(p2)
    assert g1.intersects(g2) == expected_intersect


def test_intersects_any_batch(simple_square):
    g1 = Geometry.from_shapely(simple_square)
    g2 = Geometry.from_shapely(Polygon([(3, 0), (5, 0), (5, 2), (3, 2)]))
    g3 = Geometry.from_shapely(Polygon([(1, 1), (3, 1), (3, 3), (1, 3)]))
    assert g1.intersects_any([g2, g3]) == (g1.intersects(g2) or g1.intersects(g3))


def test_translate_rotate_smoke():
    g = Geometry.from_convex_polygon(_square_coords())
    moved = g.translate(1.0, 2.0)
    cx, cy = moved.center()
    assert math.isclose(cx, 1.5, rel_tol=1e-5)
    assert math.isclose(cy, 2.5, rel_tol=1e-5)
    turned = g.rotate(math.pi / 2)
    assert turned.intersects(moved) or not turned.intersects(moved)


def test_from_ring_matches_linestring_distance(simple_square):
    ring = Geometry.from_ring(list(simple_square.exterior.coords)[:-1])
    part = Geometry.from_convex_polygon(_square_coords())
    ls = LineString(list(simple_square.exterior.coords))
    shapely_part = Polygon(_square_coords())
    assert math.isclose(
        part.distance(ring),
        shapely_part.distance(ls),
        abs_tol=1e-4,
    )


def test_min_distance_closest_points_match_shapely():
    part = Geometry.from_convex_polygon([(0, 0), (1, 0), (1, 1), (0, 1)])
    wall = Geometry.from_ring([(3, 0), (3, 2)])
    md = min_distance_pair(part, wall)
    sp = Polygon([(0, 0), (1, 0), (1, 1), (0, 1)])
    sl = LineString([(3, 0), (3, 2)])
    pa, pb = nearest_points(sp, sl)
    assert math.isclose(md.distance, sp.distance(sl), abs_tol=1e-4)
    assert math.isclose(md.closest_a[0], pa.x, abs_tol=1e-4)
    assert math.isclose(md.closest_a[1], pa.y, abs_tol=1e-4)
    assert math.isclose(md.closest_b[0], pb.x, abs_tol=1e-4)
    assert math.isclose(md.closest_b[1], pb.y, abs_tol=1e-4)


def test_cast_slide_entry_distance():
    part = Geometry.from_convex_polygon([(0, 0), (1, 0), (1, 1), (0, 1)])
    wall = Geometry.from_ring([(3, 0), (3, 2)])
    cast = find_closest_polygon_cast(part, [wall], (1.0, 0.0), 10.0)
    assert cast.intersects_path
    assert math.isclose(cast.t_entry, 2.0, abs_tol=1e-4)


def test_find_all_polygon_casts_returns_hits():
    part = Geometry.from_convex_polygon([(0, 0), (1, 0), (1, 1), (0, 1)])
    wall = Geometry.from_ring([(3, 0), (3, 2)])
    hits = find_all_polygon_casts(part, [wall], (1.0, 0.0), 10.0)
    assert hits
    assert any(h.intersects_path for h in hits)


def test_centroid_convex_square():
    g = Geometry.from_convex_polygon(_square_coords())
    cx, cy = g.centroid()
    coords = [(float(x), float(y)) for x, y in g.vertices()]
    if coords and coords[0] == coords[-1]:
        coords = coords[:-1]
    sp = Polygon(coords)
    assert math.isclose(cx, sp.centroid.x, abs_tol=0.15)
    assert math.isclose(cy, sp.centroid.y, abs_tol=0.15)


def test_fully_inside_matches_contains(simple_square, square_with_hole):
    board = Geometry.from_shapely(simple_square)
    inside = Geometry.from_convex_polygon([(0.5, 0.5), (1.5, 0.5), (1.5, 1.5), (0.5, 1.5)])
    outside = Geometry.from_convex_polygon([(3, 0), (4, 0), (4, 1), (3, 1)])
    assert inside.fully_inside(board) == simple_square.contains(
        Polygon([(0.5, 0.5), (1.5, 0.5), (1.5, 1.5), (0.5, 1.5)])
    )
    assert not outside.fully_inside(board)

    donut = Geometry.from_shapely(square_with_hole)
    in_ring = Geometry.from_convex_polygon([(0.2, 0.2), (0.8, 0.2), (0.8, 0.8), (0.2, 0.8)])
    in_hole = Geometry.from_convex_polygon([(1.5, 1.5), (2.5, 1.5), (2.5, 2.5), (1.5, 2.5)])
    assert in_ring.fully_inside(donut) == square_with_hole.contains(
        Polygon([(0.2, 0.2), (0.8, 0.2), (0.8, 0.8), (0.2, 0.8)])
    )
    assert not in_hole.fully_inside(donut)


def test_standoff_distance_matches_shapely_exterior():
    part = Geometry.from_convex_polygon([(0, 0), (1, 0), (1, 1), (0, 1)])
    wall = Geometry.from_ring([(3, 0), (3, 2)])
    sp = Polygon([(0, 0), (1, 0), (1, 1), (0, 1)])
    sl = LineString([(3, 0), (3, 2)])
    assert math.isclose(part.standoff_distance(wall), sp.distance(sl), abs_tol=1e-4)
    md = part.standoff_min_distance(wall)
    pa, pb = nearest_points(sp, sl)
    assert math.isclose(md.closest_a[0], pa.x, abs_tol=1e-4)
    assert math.isclose(md.closest_b[0], pb.x, abs_tol=1e-4)


def test_from_shapely_linestring_ring():
    ls = LineString([(0, 0), (4, 0), (4, 4), (0, 4), (0, 0)])
    g = Geometry.from_shapely(ls)
    probe = Geometry.from_convex_polygon([(2, -1), (3, -1), (3, 0), (2, 0)])
    sp = Polygon([(2, -1), (3, -1), (3, 0), (2, 0)])
    assert math.isclose(g.distance(probe), sp.distance(ls), abs_tol=1e-3)
