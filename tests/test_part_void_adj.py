"""part_void_adj Geometry parity vs Shapely."""

from shapely.geometry import box

from nest_graph.geometry import Geometry
from nest_graph.propose.placement_common import part_void_adj
from tests.geometry_oracle import (
    assert_distance_close,
    assert_intersects_matches_shapely,
)


def test_part_void_adj_overlap():
    part = box(0, 0, 2, 2)
    void = box(1, 1, 4, 4)
    pg = Geometry.from_shapely(part)
    vg = Geometry.from_shapely(void)
    assert part_void_adj(part, void, 0.1, void_geom=vg, part_geom=pg)
    assert_intersects_matches_shapely(part, void)


def test_part_void_adj_near_miss():
    part = box(0, 0, 1, 1)
    void = box(1.12, 0, 4, 4)
    min_dist = 0.1
    assert part_void_adj(part, void, min_dist)
    assert part_void_adj(
        Geometry.from_shapely(part),
        void,
        min_dist,
        void_geom=Geometry.from_shapely(void),
    )


def test_part_void_adj_separated():
    part = box(0, 0, 1, 1)
    void = box(5, 5, 7, 7)
    pg = Geometry.from_shapely(part)
    vg = Geometry.from_shapely(void)
    assert not part_void_adj(part, void, 0.1, void_geom=vg, part_geom=pg)
    assert_distance_close(part, void, ga=pg, gb=vg)
