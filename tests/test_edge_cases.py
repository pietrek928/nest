"""Edge-case geometry tests compared against Shapely."""

import math

import pytest
from shapely.geometry import Polygon
from nest_graph.geometry import Geometry, find_polygon_intersections, find_polygon_distances


def _pair_intersects(geoms, i, j):
    hits = find_polygon_intersections(geoms)
    return (i, j) in {(a, b) for a, b in hits}


def test_touching_shared_edge():
    """Two squares sharing the edge x=5 should intersect."""
    a = Polygon([(0, 0), (5, 0), (5, 5), (0, 5)])
    b = Polygon([(5, 0), (10, 0), (10, 5), (5, 5)])
    geoms = [Geometry.from_shapely(a), Geometry.from_shapely(b)]
    assert a.intersects(b)
    assert _pair_intersects(geoms, 0, 1)


def test_touching_single_vertex():
    """Squares touching at one corner should intersect (Shapely intersects includes touches)."""
    a = Polygon([(0, 0), (2, 0), (2, 2), (0, 2)])
    b = Polygon([(2, 2), (4, 2), (4, 4), (2, 4)])
    geoms = [Geometry.from_shapely(a), Geometry.from_shapely(b)]
    assert a.intersects(b)
    assert _pair_intersects(geoms, 0, 1)


def test_exact_overlap():
    """Identical polygons should intersect."""
    ring = [(0, 0), (4, 0), (4, 3), (0, 3)]
    a = Polygon(ring)
    b = Polygon(ring)
    geoms = [Geometry.from_shapely(a), Geometry.from_shapely(b)]
    assert a.intersects(b)
    assert _pair_intersects(geoms, 0, 1)


def test_nested_in_hole_disjoint():
    """A polygon inside a hole of another should not intersect the host solid."""
    outer = [(0, 0), (20, 0), (20, 20), (0, 20)]
    hole = [(8, 8), (12, 8), (12, 12), (8, 12)]
    host = Polygon(outer, [hole])
    inner = Polygon([(9, 9), (11, 9), (11, 11), (9, 11)])
    geoms = [Geometry.from_shapely(host), Geometry.from_shapely(inner)]
    assert not host.intersects(inner)
    assert not _pair_intersects(geoms, 0, 1)


def test_containment_reports_intersection():
    """A small polygon fully inside another should intersect."""
    outer = Polygon([(0, 0), (10, 0), (10, 10), (0, 10)])
    inner = Polygon([(3, 3), (7, 3), (7, 7), (3, 7)])
    geoms = [Geometry.from_shapely(outer), Geometry.from_shapely(inner)]
    assert outer.intersects(inner)
    assert _pair_intersects(geoms, 0, 1)


def test_disjoint_positive_distance():
    """Separated polygons: distance should match Shapely."""
    a = Polygon([(0, 0), (2, 0), (2, 2), (0, 2)])
    b = Polygon([(5, 0), (7, 0), (7, 2), (5, 2)])
    geoms = [Geometry.from_shapely(a), Geometry.from_shapely(b)]
    shapely_dist = a.distance(b)
    results = find_polygon_distances(geoms, aura=10.0)
    assert len(results) == 1
    our_dist = math.sqrt(results[0].distance_sq)
    assert math.isclose(our_dist, shapely_dist, rel_tol=1e-4, abs_tol=1e-5)
    assert not results[0].intersect
