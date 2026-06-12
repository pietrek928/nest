import numpy as np
from shapely.geometry import Point, Polygon

from nest_graph.config import ProposeConfig
from nest_graph.elem_graph import PlacementRuleSet, PointPlaceRule, Vec2
from nest_graph.propose.context import effective_ranking_mode
from nest_graph.propose.geometry import ProposeGeometry
from nest_graph.propose.pipeline import proposed_transforms_for_groups
from nest_graph.propose.ranking import _score_placement_rule_hybrid


def test_effective_ranking_mode_rule_hybrid_when_rules_present():
    cfg = ProposeConfig(use_rule_ranking=True, use_contact_ranking=True)
    rules = PlacementRuleSet()
    rules.append_rule(PointPlaceRule(pos=Vec2(x=0.5, y=0.5), r=0.2, w=1.0, group=0))
    base = Polygon([(0, 0), (1, 0), (1, 1), (0, 1)])
    assert effective_ranking_mode(cfg, base, rules=rules) == "rule_hybrid"


def test_rule_hybrid_prefers_rule_center(nest_board, rect_poly):
    rules = PlacementRuleSet()
    target = (0.4, 0.35)
    rules.append_rule(PointPlaceRule(
        pos=Vec2(x=target[0], y=target[1]), r=0.25, w=1.0, group=0,
    ))
    cfg = ProposeConfig(
        use_rule_ranking=True,
        rule_ranking_weight=1.0,
        use_contact_ranking=True,
        use_contact_clearance_hybrid=True,
        ranking_mode="contact_hybrid",
    )
    min_dist = 0.02
    geom = ProposeGeometry(
        nest_board,
        Polygon(),
        rect_poly,
        min_dist,
        propose_cfg=cfg,
    )
    near = (target[0], target[1], 0.0)
    far = (1.0, 1.0, 0.0)
    pt_push = Point(nest_board.centroid)
    near_score = _score_placement_rule_hybrid(
        near, rect_poly, Polygon(), geom, cfg, pt_push, min_dist, None, rules, 0,
    )
    far_score = _score_placement_rule_hybrid(
        far, rect_poly, Polygon(), geom, cfg, pt_push, min_dist, None, rules, 0,
    )
    assert near_score > far_score


def test_score_transform_peak_at_rule_center():
    from nest_graph.elem_graph import score_transform

    rules = PlacementRuleSet()
    target = (0.35, 0.3)
    rules.append_rule(PointPlaceRule(
        pos=Vec2(x=target[0], y=target[1]), r=0.25, w=1.0, group=0,
    ))
    near = score_transform(rules, 0, target[0], target[1], 0.0)
    far = score_transform(rules, 0, 2.0, 2.0, 0.0)
    assert near > far

def test_proposed_transforms_accepts_rules(nest_board, rect_poly, tri_poly):
    rules = PlacementRuleSet()
    rules.append_rule(PointPlaceRule(pos=Vec2(x=0.2, y=0.2), r=0.2, w=0.5, group=0))
    rules.append_rule(PointPlaceRule(pos=Vec2(x=0.7, y=0.7), r=0.2, w=0.5, group=1))
    cfg = ProposeConfig(use_rule_ranking=True, max_proposals=5, candidate_pool=16)
    out = proposed_transforms_for_groups(
        nest_board,
        [(rect_poly, 0), (tri_poly, 1)],
        [],
        [],
        cfg,
        min_dist=0.02,
        rules=rules,
    )
    assert out[0].shape[0] > 0
    assert out[1].shape[0] > 0
