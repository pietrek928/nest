"""Tests for LNS ruin-and-recreate helpers."""

from shapely.geometry import Point, Polygon

from nest_graph.propose.lns_rebuild import (
    apply_lns_destroy,
    lns_accept,
    void_frontier_destroy_indices,
)


def test_void_frontier_destroy_prefers_near_void():
    void = Polygon([(0, 0), (1, 0), (1, 1), (0, 1)])
    polys = [
        Polygon([(0.1, 0.1), (0.3, 0.1), (0.3, 0.3), (0.1, 0.3)]),  # near void
        Polygon([(5, 5), (5.2, 5), (5.2, 5.2), (5, 5.2)]),  # far
        Polygon([(0.2, 0.2), (0.4, 0.2), (0.4, 0.4), (0.2, 0.4)]),  # near
    ]
    idxs = void_frontier_destroy_indices(
        polys, [0, 1, 2], pole=Point(0.5, 0.5), void_poly=void, destroy_fraction=0.5,
    )
    assert 1 not in idxs or idxs[0] != 1
    assert idxs[0] in (0, 2)


def test_lns_accept_prefers_more_parts():
    assert lns_accept(
        old_count=5, old_area=1.0, old_cov=0.5,
        new_count=6, new_area=0.9, new_cov=0.4,
    )


def test_apply_lns_destroy():
    assert apply_lns_destroy([1, 2, 3, 4], [2, 4]) == [1, 3]
