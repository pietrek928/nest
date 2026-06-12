import pytest

from nest_graph.build_graph import (
    NestState,
    _inject_repulsor_rules,
    truncate_rule_set,
)
from nest_graph.config import BuildGraphConfig, RulesConfig
from nest_graph.elem_graph import (
    PlacementRuleSet,
    PointPlaceRule,
    Vec2,
    score_transform,
)


def test_truncate_rule_set_keeps_negative_weights():
    rs = PlacementRuleSet()
    rs.append_rule(PointPlaceRule(pos=Vec2(x=0, y=0), r=0.1, w=0.05, group=0))
    rs.append_rule(PointPlaceRule(pos=Vec2(x=1, y=1), r=0.1, w=-0.9, group=0))
    rs.append_rule(PointPlaceRule(pos=Vec2(x=2, y=2), r=0.1, w=0.2, group=0))
    trimmed = truncate_rule_set(rs, 2)
    weights = sorted(abs(pr.w) for pr in trimmed.point_rules)
    assert weights == pytest.approx([0.2, 0.9])
    assert any(pr.w < 0 for pr in trimmed.point_rules)


def test_score_transform_repulsor_at_center():
    rules = PlacementRuleSet()
    rules.append_rule(PointPlaceRule(pos=Vec2(x=0.5, y=0.5), r=0.2, w=-0.5, group=0))
    at_center = score_transform(rules, 0, 0.5, 0.5, 0.0)
    far = score_transform(rules, 0, 2.0, 2.0, 0.0)
    assert at_center < far
    assert at_center < 0.0


def test_inject_repulsor_rules_adds_negative_point_rules(nest_board):
    cfg = BuildGraphConfig(
        rules=RulesConfig(use_repulsor_rules=True, repulsor_weight=-0.2, ngroups=2),
    )
    base = PlacementRuleSet()
    base.append_rule(PointPlaceRule(pos=Vec2(x=0, y=0), r=0.1, w=0.5, group=0))
    rule_sets = [base]
    nest_state = NestState(polys=[], group_id=[], transform=[], selected_indices=[])
    out = _inject_repulsor_rules(rule_sets, cfg, nest_board, nest_state)
    assert len(out) == 1
    neg = [pr for pr in out[0].point_rules if pr.w < 0]
    assert len(neg) >= 2
    assert all(pr.group in (0, 1) for pr in neg)
