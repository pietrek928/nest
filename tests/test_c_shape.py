import pytest
from shapely.geometry import Polygon
from nest_graph.geometry import Geometry, find_polygon_intersections

def test_c_shape():
    c_shape = Polygon([(0,0), (3,0), (3,3), (0,3), (0,2), (2,2), (2,1), (0,1)])
    
    # Point in the hole (should be outside)
    square_hole = Polygon([(1.4, 1.1), (1.6, 1.1), (1.6, 1.3), (1.4, 1.3)])
    
    g_c = Geometry.from_shapely(c_shape)
    g_hole = Geometry.from_shapely(square_hole)
    
    res = find_polygon_intersections([g_c, g_hole])
    assert len(res) == 0, "Should not intersect (square is in the hole)"

