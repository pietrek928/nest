"""Unit tests for void-island MIS boost experiment helpers."""

import numpy as np
from shapely.geometry import Point, box

from nest_graph.build_graph import (
    _boost_void_island_scores,
    _make_void_attractor_rule_set,
    _merge_void_attractor_into_rule_sets,
)
from nest_graph.config import ProposeConfig
from nest_graph.elem_graph import PlacementRuleSet, PointPlaceRule, Vec2
from nest_graph.utils import transform_poly


def test_gravity_compaction_disabled_by_default():
    assert ProposeConfig().enable_gravity_compaction is False
    assert ProposeConfig().void_island_score_boost == 64.0
    assert ProposeConfig().void_attractor_rule_weight == 16.0
    assert ProposeConfig().enable_void_large_hijack is True


def test_boost_void_island_scores_only_inside_free():
    free = box(5, 5, 10, 10)
    polys = [
        transform_poly(box(0, 0, 1, 1), (6.5, 6.5, 0.0)),  # inside free
        transform_poly(box(0, 0, 1, 1), (1.0, 1.0, 0.0)),  # outside
    ]
    scores = [1.0, 2.0]
    # No pole → flat factor 1.0 (legacy path).
    n = _boost_void_island_scores(polys, scores, free, weight=64.0)
    assert n == 1
    assert scores[0] == 65.0
    assert scores[1] == 2.0


def test_boost_void_island_distance_to_pole():
    free = box(0, 0, 10, 10)
    pole = Point(5.0, 5.0)
    # transform_poly shifts by translation; unit box centroid is (+0.5,+0.5).
    near = transform_poly(box(0, 0, 1, 1), (4.5, 4.5, 0.0))
    far = transform_poly(box(0, 0, 1, 1), (8.5, 8.5, 0.0))
    scores = [0.0, 0.0]
    n = _boost_void_island_scores(
        [near, far],
        scores,
        free,
        weight=64.0,
        pole=pole,
        sheet_diag=10.0 * np.sqrt(2.0),
    )
    assert n == 2
    assert scores[0] > scores[1]
    assert abs(scores[0] - 64.0) < 1e-6  # centroid on pole


def test_boost_void_island_zero_weight_noop():
    free = box(0, 0, 10, 10)
    polys = [transform_poly(box(0, 0, 1, 1), (1.0, 1.0, 0.0))]
    scores = [1.0]
    n = _boost_void_island_scores(polys, scores, free, weight=0.0)
    assert n == 0
    assert scores[0] == 1.0


def test_void_attractor_rule_set_has_point_per_group():
    pole = Point(0.4, 0.3)
    rs = _make_void_attractor_rule_set(pole, ngroups=2, radius=0.1, weight=16.0)
    assert len(rs.point_rules) == 2
    assert all(abs(pr.w - 16.0) < 1e-9 for pr in rs.point_rules)


def test_merge_void_attractor_prepends():
    base = PlacementRuleSet()
    base.append_rule(PointPlaceRule(pos=Vec2(x=0, y=0), r=0.1, w=0.1, group=0))
    attractor = _make_void_attractor_rule_set(
        Point(1, 1), ngroups=1, radius=0.2, weight=16.0,
    )
    merged = _merge_void_attractor_into_rule_sets(
        [base],
        attractor,
        nest_rule_sets_used=1,
        max_rules_per_set=24,
    )
    assert len(merged) == 1
    assert len(merged[0].point_rules) >= 2
    assert merged[0].point_rules[0].w == 16.0
