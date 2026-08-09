"""Packing collision: edge kiss is independent; real overlap is not."""

from shapely.geometry import box

from nest_graph.geometry import Geometry, find_polygon_intersections
from nest_graph.propose.compaction import selection_pairwise_independent


def test_edge_kiss_not_packing_collision():
    a = Geometry.from_shapely(box(0, 0, 2, 2))
    b = Geometry.from_shapely(box(2, 0, 4, 2))
    assert find_polygon_intersections([a, b]) == []
    assert not a.intersects(b)
    assert selection_pairwise_independent([box(0, 0, 2, 2), box(2, 0, 4, 2)], [0, 1])


def test_vertex_kiss_not_packing_collision():
    a = Geometry.from_shapely(box(0, 0, 2, 2))
    b = Geometry.from_shapely(box(2, 2, 4, 4))
    assert find_polygon_intersections([a, b]) == []
    assert not a.intersects(b)


def test_real_overlap_is_packing_collision():
    a = Geometry.from_shapely(box(0, 0, 2, 2))
    c = Geometry.from_shapely(box(1, 1, 3, 3))
    assert find_polygon_intersections([a, c]) == [(0, 1)]
    assert a.intersects(c)
    assert not selection_pairwise_independent(
        [box(0, 0, 2, 2), box(1, 1, 3, 3)], [0, 1]
    )
