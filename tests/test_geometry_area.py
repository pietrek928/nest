import pytest

from nest_graph.geometry import Geometry


def test_unit_square_area():
    square = Geometry.from_convex_polygon([(0, 0), (1, 0), (1, 1), (0, 1)])
    assert square.area() == pytest.approx(1.0)


def test_geometry_area_matches_shapely_for_triangle():
    from shapely.geometry import Polygon

    tri = Polygon([(0, 0), (1.2, 0), (0, 1.1)])
    geom = Geometry.from_shapely(tri)
    assert geom.area() == pytest.approx(tri.area, rel=1e-3, abs=1e-4)
