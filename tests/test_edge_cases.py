"""Edge-case geometry tests — packing SoT is Penetrating, not Shapely intersects."""

import math

from shapely.geometry import Polygon

from nest_graph.geometry import find_polygon_distances
from tests.geometry_oracle import assert_packing_collide


def test_touching_shared_edge_is_kiss_not_packing():
    """Squares sharing x=5: Shapely intersects, packing collide False."""
    a = Polygon([(0, 0), (5, 0), (5, 5), (0, 5)])
    b = Polygon([(5, 0), (10, 0), (10, 5), (5, 5)])
    assert a.intersects(b) and a.touches(b)
    assert_packing_collide(a, b, expect=False)


def test_touching_single_vertex_is_kiss_not_packing():
    a = Polygon([(0, 0), (2, 0), (2, 2), (0, 2)])
    b = Polygon([(2, 2), (4, 2), (4, 4), (2, 4)])
    assert a.intersects(b) and a.touches(b)
    assert_packing_collide(a, b, expect=False)


def test_exact_overlap_is_packing_hit():
    ring = [(0, 0), (4, 0), (4, 3), (0, 3)]
    a = Polygon(ring)
    b = Polygon(ring)
    assert_packing_collide(a, b, expect=True)


def test_nested_in_hole_disjoint():
    outer = [(0, 0), (20, 0), (20, 20), (0, 20)]
    hole = [(8, 8), (12, 8), (12, 12), (8, 12)]
    host = Polygon(outer, [hole])
    inner = Polygon([(9, 9), (11, 9), (11, 11), (9, 11)])
    assert not host.intersects(inner)
    assert_packing_collide(host, inner, expect=False)


def test_containment_reports_packing_hit():
    outer = Polygon([(0, 0), (10, 0), (10, 10), (0, 10)])
    inner = Polygon([(3, 3), (7, 3), (7, 7), (3, 7)])
    assert_packing_collide(outer, inner, expect=True)


def test_disjoint_positive_distance():
    a = Polygon([(0, 0), (2, 0), (2, 2), (0, 2)])
    b = Polygon([(5, 0), (7, 0), (7, 2), (5, 2)])
    from nest_graph.geometry import Geometry

    geoms = [Geometry.from_shapely(a), Geometry.from_shapely(b)]
    shapely_dist = a.distance(b)
    results = find_polygon_distances(geoms, aura=10.0)
    assert len(results) == 1
    our_dist = math.sqrt(results[0].distance_sq)
    assert math.isclose(our_dist, shapely_dist, rel_tol=1e-4, abs_tol=1e-5)
    assert not results[0].intersect
