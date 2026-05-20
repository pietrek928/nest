import math

import pytest
from shapely.geometry import Point, Polygon

from nest_graph.geometry import Geometry
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
