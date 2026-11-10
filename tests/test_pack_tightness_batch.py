"""P1b: pack_tightness_cost / score_placement_tightness via batch_rank."""

import pytest
from shapely.geometry import Point, box

from nest_graph.propose.ranking import (
    calculate_excess_and_kiss,
    pack_neighbor_excess_gap,
    pack_tightness_cost,
    score_placement_tightness,
)
from tests.fixtures.propose_emit import make_propose_geom, tightness_pack_geoms


@pytest.mark.parametrize("n,gap", [(2, 0.2), (3, 0.15), (4, 0.1)])
def test_pack_tightness_cost_matches_oracle(n, gap):
    geoms, outline, min_dist = tightness_pack_geoms(n=n, gap=gap)
    got = pack_tightness_cost(geoms, outline, min_dist)
    from nest_graph.placement.outline import outline_ring_geom

    ring = outline_ring_geom(outline)
    kiss = 0.0
    for g in geoms:
        _e, k = calculate_excess_and_kiss(
            g, obstacles=[], outline=outline, min_dist=min_dist, ring_geom=ring,
        )
        kiss += k
    want = 2.0 * pack_neighbor_excess_gap(geoms, min_dist) + kiss
    assert got == pytest.approx(want, rel=1e-5, abs=1e-5)


def test_score_placement_tightness_finite_clear():
    geom = make_propose_geom()
    score = score_placement_tightness((10.0, 10.0, 0.0), geom, Point(10, 10), 0.1)
    assert score > float("-inf")
    assert score <= 0.0


def test_score_placement_tightness_far_outside_neginf():
    geom = make_propose_geom()
    score = score_placement_tightness((500.0, 500.0, 0.0), geom, Point(10, 10), 0.1)
    assert score == float("-inf")
