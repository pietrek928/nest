"""P7: mate/pattern clearance uses native intersects + hull area."""

from shapely.geometry import box

from nest_graph.propose.placements_pattern import (
    _pair_hull_area,
    synthesize_mate_patterns,
)


def test_pair_hull_area_native_positive():
    poly = box(0, 0, 1, 1)
    area = _pair_hull_area(poly, (1.2, 0.0, 0.0))
    assert area > 1.0
    assert area < float("inf")


def test_synthesize_mate_patterns_no_penetrate():
    parts = [(box(0, 0, 1, 1), 0)]
    pats = synthesize_mate_patterns(parts, min_dist=0.1, max_patterns=2)
    # May be empty for axis-aligned box; must not raise.
    assert isinstance(pats, list)
