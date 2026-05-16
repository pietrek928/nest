import pytest
from shapely.geometry import Polygon, MultiPolygon, Point
import math

from nest_graph.geometry import (
    Geometry,
    find_polygon_distances,
    find_polygon_intersections,
)


# --- Fixtures ---

@pytest.fixture
def simple_square():
    return Polygon([(0, 0), (2, 0), (2, 2), (0, 2)])

@pytest.fixture
def square_with_hole():
    outer = [(0, 0), (4, 0), (4, 4), (0, 4)]
    hole = [(1, 1), (3, 1), (3, 3), (1, 3)]
    return Polygon(outer, [hole])

@pytest.fixture
def multi_square():
    poly1 = Polygon([(0, 0), (2, 0), (2, 2), (0, 2)])
    poly2 = Polygon([(4, 4), (6, 4), (6, 6), (4, 6)])
    return MultiPolygon([poly1, poly2])

@pytest.fixture
def tessellated_circle():
    num_segments = 128
    radius = 5.0
    circle_pts = []
    for i in range(num_segments):
        angle = 2.0 * math.pi * i / num_segments
        circle_pts.append((radius * math.cos(angle), radius * math.sin(angle)))
    return Polygon(circle_pts)


# --- Construction Tests ---

@pytest.mark.parametrize("poly_fixture", [
    "simple_square",
    "square_with_hole",
    "multi_square"
])
def test_geometry_from_shapely_valid(poly_fixture, request):
    shapely_poly = request.getfixturevalue(poly_fixture)
    geom = Geometry.from_shapely(shapely_poly)
    assert geom is not None

def test_geometry_from_shapely_invalid_type():
    pt = Point(0, 0)
    with pytest.raises(ValueError, match="no usable polygon rings"):
        Geometry.from_shapely(pt)


# --- Intersection Tests ---

@pytest.mark.parametrize("p2_coords, expected_intersect", [
    ([(3, 0), (5, 0), (5, 2), (3, 2)], False), # Disjoint
    ([(1, 1), (3, 1), (3, 3), (1, 3)], True),  # Overlapping
    ([(2, 0), (4, 0), (4, 2), (2, 2)], True),  # Touching edge
])
def test_intersection(simple_square, p2_coords, expected_intersect):
    p2 = Polygon(p2_coords)
    
    # Verify Shapely ground truth
    assert simple_square.intersects(p2) == expected_intersect
    
    g1 = Geometry.from_shapely(simple_square)
    g2 = Geometry.from_shapely(p2)
    
    results = find_polygon_intersections([g1, g2])
    assert (len(results) > 0) == expected_intersect
    
    if expected_intersect:
        poly_a, poly_b = results[0]
        assert poly_a == 0
        assert poly_b == 1


# --- Distance Tests ---

@pytest.mark.parametrize("p2_coords, expected_dist", [
    ([(3, 0), (5, 0), (5, 2), (3, 2)], 1.0), # Disjoint, dist=1.0
    ([(0, 3), (2, 3), (2, 5), (0, 5)], 1.0), # Disjoint above, dist=1.0
    ([(3, 3), (5, 3), (5, 5), (3, 5)], math.sqrt(2.0)), # Disjoint diagonal, dist=sqrt(2)
])
def test_distance_simple(simple_square, p2_coords, expected_dist):
    p2 = Polygon(p2_coords)
    
    # Verify Shapely ground truth
    shapely_dist = simple_square.distance(p2)
    assert math.isclose(shapely_dist, expected_dist)
    
    g1 = Geometry.from_shapely(simple_square)
    g2 = Geometry.from_shapely(p2)
    
    results = find_polygon_distances([g1, g2], aura=5.0)
    assert len(results) == 1
    assert not results[0].intersect
    assert math.isclose(math.sqrt(results[0].distance_sq), shapely_dist, rel_tol=1e-5)

def test_distance_complex_shapes(tessellated_circle):
    # A sharp star/needle polygon outside the circle
    needle = Polygon([(6.0, -0.1), (10.0, 0.0), (6.0, 0.1)])
    
    shapely_dist = tessellated_circle.distance(needle)
    
    g_circle = Geometry.from_shapely(tessellated_circle)
    g_needle = Geometry.from_shapely(needle)
    
    results = find_polygon_distances([g_circle, g_needle], aura=2.0)
    assert len(results) == 1
    assert not results[0].intersect
    assert math.isclose(math.sqrt(results[0].distance_sq), shapely_dist, rel_tol=1e-5)