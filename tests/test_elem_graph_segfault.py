"""Regression tests for elem_graph segfaults (empty rule groups, seed=0 main loop)."""

import os
import subprocess
import sys

import numpy as np
import pytest
from shapely.geometry import Polygon

from nest_graph.build_graph import (
    improve_rules,
    make_polygon_graph,
    transform_history,
    transform_selection,
)
from nest_graph.elem_graph import (
    Circle,
    ElemGraph,
    PlacementRuleSet,
    PointAngleRule,
    PointPlaceRule,
    Vec2,
    increase_score_dfs,
    increase_selection_dfs,
    nest_by_graph,
    score_elems,
    score_rules,
    sort_graph,
)
from nest_graph.utils import normalize_poly

BOARD = Polygon([(0, 0), (1.2, 0), (0, 1.1)])
RECT = normalize_poly(Polygon([(0, 0), (0.1, 0), (0.1, 0.1), (0, 0.1)]))
TRI = normalize_poly(Polygon([(0, 0), (0.15, 0), (0, 0.07)]))


def test_nest_by_graph_rules_for_missing_group_do_not_segfault():
    """Rules targeting group 1 must not OOB when the graph only has group-0 nodes."""
    graph = ElemGraph()
    graph.append_elem(0, Vec2(x=0.5, y=0.5), Circle.from_center_radius(0.5, 0.5, 0.1))
    graph.append_elem(0, Vec2(x=0.2, y=0.2), Circle.from_center_radius(0.2, 0.2, 0.05))

    rules = PlacementRuleSet()
    rules.append_rule(PointPlaceRule(pos=Vec2(x=0, y=0), r=0.2, w=1.0, group=0))
    rules.append_rule(PointPlaceRule(pos=Vec2(x=0.7, y=0.7), r=0.2, w=1.0, group=1))

    scores = score_elems(graph, rules)
    assert len(scores.scores) == 2
    selected = nest_by_graph(graph, [rules])[0]
    assert len(selected) >= 1
    score_rules([graph], [rules])


def test_score_rules_seed0_iter4_graph_regression():
    """Reproduces former segfault: seed=0, iteration 4, group-0-only graph + group-1 rules."""
    np.random.seed(0)

    first_rule_set = PlacementRuleSet()
    first_rule_set.append_rule(PointPlaceRule(pos=Vec2(x=0, y=0), r=0.1, w=0.1, group=0))
    first_rule_set.append_rule(PointPlaceRule(pos=Vec2(x=0, y=0), r=0.1, w=0.1, group=1))
    rule_sets = [first_rule_set]

    rule_set = PlacementRuleSet()
    for pos, g in [((0, 0), 0), ((0.7, 0.7), 1), ((0, 1.1), 1), ((1.2, 0), 1)]:
        rule_set.append_rule(
            PointPlaceRule(pos=Vec2(x=pos[0], y=pos[1]), r=0.2, w=1, group=g)
        )
    for a in [np.pi / 4, np.pi * 5 / 4]:
        rule_set.append_rule(
            PointAngleRule(pos=Vec2(x=0, y=1.1), r=0.2, a=a, w=0.1, group=1)
        )
        rule_set.append_rule(
            PointAngleRule(pos=Vec2(x=1.2, y=0), r=0.2, a=a, w=0.1, group=1)
        )

    selected_t = [
        np.random.rand(128, 3) * [1.5, 1.5, 2 * np.pi],
        np.random.rand(128, 3) * [1.5, 1.5, 2 * np.pi],
    ]
    history = [np.zeros((1, 3)), np.zeros((1, 3))]
    graphs = []

    def run_iteration():
        nonlocal selected_t, history, graphs, rule_sets
        rng = np.random.default_rng()
        s0 = [np.random.rand(256, 3) * [1.5, 1.5, 2 * np.pi], history[0]]
        if selected_t[0].shape[0] > 0:
            s0.append(selected_t[0])
            s0.extend(transform_selection(selected_t[0], 4, rng))
            s0.extend(transform_history(history[0], 2, rng))
        s1 = [np.random.rand(256, 3) * [1.5, 1.5, 2 * np.pi], history[1]]
        if selected_t[1].shape[0] > 0:
            s1.append(selected_t[1])
            s1.extend(transform_selection(selected_t[1], 4, rng))
            s1.extend(transform_history(history[1], 2, rng))
        selected_t = [np.concatenate(s0), np.concatenate(s1)]
        graph, _polys, gid, trans = make_polygon_graph(
            BOARD, [(RECT, selected_t[0]), (TRI, selected_t[1])]
        )
        graphs.append(graph)
        graphs = graphs[-12:]
        for _ in range(8):
            rule_sets = improve_rules(graphs, rule_sets, 64, BOARD)
        sel = nest_by_graph(graph, rule_sets[:1])[0]
        gs = sort_graph(graph, rule_set)
        gsr = sort_graph(graph, rule_set, reverse=True)
        sc = score_elems(graph, rule_set)
        for _ in range(2):
            sel = increase_selection_dfs(gsr, sel, 8)
            sel = increase_selection_dfs(gs, sel, 8)
            sel = increase_score_dfs(gsr, sel, sc)
            sel = increase_selection_dfs(gs, sel, 8)
            sel = increase_score_dfs(gs, sel, sc)
        selected_t = [[], []]
        for i in sel:
            selected_t[gid[i]].append(trans[i])
        selected_t = tuple(np.array(x) for x in selected_t)
        if len(history[0]) and len(selected_t[0]):
            history[0] = np.unique(
                np.concatenate([selected_t[0], history[0]]), axis=0
            )[5000:, :]
        return graph

    for _ in range(3):
        run_iteration()

    g4 = run_iteration()

    # Former crash: rules reference group 1 while graph may have no group-1 nodes.
    score_rules([g4], [rule_sets[0]])
    score_rules(graphs, rule_sets)


def test_score_rules_synthetic_group0_only_graph():
    """Minimal deterministic case: only group-0 nodes, rules include group 1."""
    graph = ElemGraph()
    for i in range(5):
        graph.append_elem(
            0,
            Vec2(x=0.1 * i, y=0.05 * i),
            Circle.from_center_radius(0.1 * i, 0.05 * i, 0.05),
        )
    rules = PlacementRuleSet()
    rules.append_rule(PointPlaceRule(pos=Vec2(x=0, y=0), r=0.2, w=1.0, group=0))
    rules.append_rule(PointPlaceRule(pos=Vec2(x=1.0, y=1.0), r=0.2, w=1.0, group=1))
    nest_by_graph(graph, [rules])
    score_rules([graph], [rules])


def test_build_graph_module_seed0_no_segfault():
    """Full main() driver with seed 0 must complete without SIGSEGV."""
    env = {**os.environ, "NEST_BUILD_GRAPH_ITERS": "3"}
    proc = subprocess.run(
        [sys.executable, "-c", "import numpy as np; np.random.seed(0); import nest_graph.build_graph as bg; bg.main()"],
        cwd=os.path.dirname(os.path.dirname(__file__)),
        env=env,
        capture_output=True,
        timeout=300,
    )
    assert proc.returncode == 0, proc.stderr.decode()[-3000:]
