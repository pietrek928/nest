"""Unit tests for void greedy nest seed, pole props, and attractor radius."""

import numpy as np
from shapely.geometry import Point, box

from nest_graph.build_graph import (
    _count_props_near_pole,
    _void_attractor_radius,
    _zones_have_void_hijack,
)
from nest_graph.config import ProposeConfig
from nest_graph.elem_graph import (
    Circle,
    ElemGraph,
    SelectMode,
    SelectOptions,
    Vec2,
    nest_by_scores,
)


def test_void_greedy_nest_seed_default_on():
    assert ProposeConfig().late_border_void_release_ratio == 1.5


def test_void_attractor_radius_scales_with_diag():
    r = _void_attractor_radius(min_dist=0.05, sheet_diag=10.0, place_rule_radius=0.2)
    assert abs(r - 2.5) < 1e-9


def test_count_props_near_pole():
    pole = Point(5.0, 5.0)
    props = [
        np.array([[5.1, 5.0, 0.0], [9.0, 9.0, 0.0]], dtype=np.float64),
    ]
    n = _count_props_near_pole(props, pole, radius=1.0)
    assert n == 1
    assert _count_props_near_pole(props, pole, radius=0.0) == 0
    assert _count_props_near_pole(None, pole, radius=1.0) == 0


def test_zones_have_void_hijack():
    assert _zones_have_void_hijack(["border_gap→void_seek(large_void)"])
    assert not _zones_have_void_hijack(["border_gap", "cluster_edge→void_seek"])
    assert not _zones_have_void_hijack([])


def test_nest_by_scores_prefers_high_score():
    g = ElemGraph()
    g.append_elem(0, Vec2(x=0.0, y=0.0), Circle.from_center_radius(0.0, 0.0, 0.1))
    g.append_elem(0, Vec2(x=1.0, y=0.0), Circle.from_center_radius(1.0, 0.0, 0.1))
    g.append_elem(0, Vec2(x=0.0, y=1.0), Circle.from_center_radius(0.0, 1.0, 0.1))
    g.add_collision_pair(0, 1)
    scores = [1.0, 100.0, 50.0]
    opts = SelectOptions()
    opts.mode = SelectMode.greedy_score
    opts.local_swap = False
    sel = nest_by_scores(g, scores, opts)
    assert 1 in sel
    assert 0 not in sel
    assert 2 in sel
