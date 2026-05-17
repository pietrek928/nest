import pytest
from shapely.geometry import Polygon
from nest_graph.geometry import Geometry, find_polygon_intersections, find_polygon_distances

def test_concave_intersection():
    # L-shape: (0,0) to (3,0) to (3,1) to (1,1) to (1,3) to (0,3)
    l_shape = Polygon([(0,0), (3,0), (3,1), (1,1), (1,3), (0,3)])
    
    # A small square inside the "crotch" of the L-shape, NOT intersecting
    # Crotch is at (1,1) to (3,1) and (1,1) to (1,3).
    # Square at (1.5, 1.5) to (2.5, 2.5)
    square_out = Polygon([(1.5, 1.5), (2.5, 1.5), (2.5, 2.5), (1.5, 2.5)])
    
    # A small square completely inside the L-shape
    square_in = Polygon([(0.1, 0.1), (0.9, 0.1), (0.9, 0.9), (0.1, 0.9)])
    
    g_l = Geometry.from_shapely(l_shape)
    g_out = Geometry.from_shapely(square_out)
    g_in = Geometry.from_shapely(square_in)
    
    # Should NOT intersect
    res_out = find_polygon_intersections([g_l, g_out])
    assert len(res_out) == 0, "Should not intersect"
    
    # Should intersect (containment)
    res_in = find_polygon_intersections([g_l, g_in])
    assert len(res_in) == 1, "Should intersect (contained)"

