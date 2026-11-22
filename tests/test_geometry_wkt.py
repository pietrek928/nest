"""WKT from_string / to_string roundtrips for Geometry."""

import math

from shapely.geometry import LineString, MultiLineString, MultiPoint, Point, Polygon, box
from shapely import wkt as shapely_wkt

from nest_graph.geometry import Geometry
from tests.fixtures.shapes import donut, simple_square


def test_point_roundtrip():
    g = Geometry.from_string("POINT (1 2)")
    assert g.area() == 0.0
    back = Geometry.from_string(g.to_string())
    assert "POINT" in back.to_wkt().upper()


def test_linestring_open_area_zero():
    g = Geometry.from_shapely(LineString([(0, 0), (3, 0), (3, 2)]))
    assert g.area() == 0.0
    assert g.contains_point(1.0, 0.0) is False
    assert "LINESTRING" in g.to_string().upper()


def test_polygon_wkt_matches_from_shapely():
    poly = simple_square()
    g_s = Geometry.from_shapely(poly)
    g_w = Geometry.from_wkt(poly.wkt)
    assert math.isclose(g_s.area(), g_w.area(), abs_tol=1e-9)
    assert g_s.contains_point(0.5, 0.5) == g_w.contains_point(0.5, 0.5)


def test_donut_wkt_roundtrip_area():
    poly = donut()
    g = Geometry.from_shapely(poly)
    g2 = Geometry.from_string(g.to_string())
    assert math.isclose(g.area(), g2.area(), rel_tol=1e-6)
    assert g.contains_point(5.0, 5.0) is False
    assert g2.contains_point(5.0, 5.0) is False


def test_multipoint_and_multilinestring():
    mp = Geometry.from_shapely(MultiPoint([(0, 0), (1, 1)]))
    assert "MULTIPOINT" in mp.to_string().upper()
    ml = Geometry.from_shapely(MultiLineString([[(0, 0), (1, 0)], [(2, 0), (3, 1)]]))
    assert ml.area() == 0.0
    assert "MULTILINESTRING" in ml.to_string().upper()


def test_from_wkt_alias():
    g = Geometry.from_wkt(box(0, 0, 2, 2).wkt)
    assert math.isclose(g.area(), 4.0, abs_tol=1e-9)
