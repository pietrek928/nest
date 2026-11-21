"""CLI / K-lift smoke for parametrized build_graph."""

import numpy as np
from shapely.geometry import box

from nest_graph.build_graph import NestState, history_from_nest_state
from nest_graph.rules.evolve import make_demo_rule_set, make_seed_rule_sets
from nest_graph.config import BuildGraphConfig
from nest_graph.shapes_catalog import (
    DemoPackingCase,
    apply_demo_case_to_rules,
    demo_case_from_flags,
    parse_parts_flag,
)


def test_parse_parts_three_kinds():
    names, scales, counts = parse_parts_flag("rect:1.2:40,tri:1.2:30,strip:1.2:20")
    assert names == ("rect", "tri", "strip")
    assert scales == (1.2, 1.2, 1.2)
    assert counts == (40, 30, 20)


def test_demo_case_from_flags_k3():
    case = demo_case_from_flags(
        board="rect:18x18",
        parts="rect:1.2:40,tri:1.2:30,strip:1.2:20",
    )
    assert len(case.groups) == 3
    assert case.group_counts == (40, 30, 20)
    rules = apply_demo_case_to_rules(BuildGraphConfig().rules, case)
    assert rules.ngroups == 3


def test_history_from_nest_state_keeps_gid2():
    nest = NestState(
        polys=[box(0, 0, 1, 1)] * 3,
        group_id=[0, 1, 2],
        transform=[
            np.array([0.0, 0.0, 0.0]),
            np.array([1.0, 0.0, 0.0]),
            np.array([2.0, 0.0, 0.0]),
        ],
        selected_indices=[0, 1, 2],
    )
    hist = history_from_nest_state(nest, ngroups=3)
    assert len(hist) == 3
    assert hist[2].shape == (1, 3)
    assert float(hist[2][0, 0]) == 2.0


def test_seed_and_demo_rules_ngroups3():
    cfg = BuildGraphConfig()
    cfg = cfg.model_copy(
        update={"rules": cfg.rules.model_copy(update={"ngroups": 3})}
    )
    seeds = make_seed_rule_sets(cfg)
    assert len(seeds) == 1
    groups = {int(r.group) for r in seeds[0].point_rules}
    assert groups == {0, 1, 2}
    demo = make_demo_rule_set(cfg)
    demo_groups = {int(r.group) for r in demo.point_rules}
    assert 0 in demo_groups and 1 in demo_groups and 2 in demo_groups


def test_default_demo_rules_k2_place_count():
    """K=2 keeps four legacy place rules (1 for g0, 3 for g1)."""
    cfg = BuildGraphConfig()
    demo = make_demo_rule_set(cfg)
    place = list(demo.point_rules)
    assert len(place) == 4
