"""D0: footprint_inside_batch matches scalar footprint_inside (no solid vector copy)."""

from shapely.geometry import box

from nest_graph.geometry import Geometry


def test_footprint_inside_batch_matches_scalar():
    board = Geometry.from_shapely(box(0, 0, 10, 10))
    inside = Geometry.from_shapely(box(1, 1, 2, 2))
    outside = Geometry.from_shapely(box(9, 9, 12, 12))
    edge = Geometry.from_shapely(box(9.5, 9.5, 10.5, 10.5))
    flags = board.footprint_inside_batch([inside, outside, edge])
    assert list(flags) == [
        bool(inside.footprint_inside(board)),
        bool(outside.footprint_inside(board)),
        bool(edge.footprint_inside(board)),
    ]
    assert flags[0] is True
    assert flags[1] is False
