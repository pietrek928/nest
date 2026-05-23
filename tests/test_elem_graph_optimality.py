"""Optimality, independence, and monotonicity tests for elem_graph selection."""

from typing import Iterable, List, Sequence

import pytest

from nest_graph.config import _make_select_options
from nest_graph.elem_graph import (
    Circle,
    ElemGraph,
    PlacementRuleSet,
    RefineSelectionOptions,
    ScoreRulesOptions,
    Vec2,
    nest_by_graph,
    refine_selection_dfs,
    score_rules,
)


def assert_independent_set(g: ElemGraph, selected: Sequence[int]) -> None:
    sel = set(selected)
    n = len(g.group_id)
    for v in sel:
        assert 0 <= v < n
        for u in g.collisions[v]:
            assert u not in sel or u == v, f"collision edge {v}-{u} in selection"


def score_sum(scores: Sequence[float], selected: Sequence[int]) -> float:
    return sum(scores[i] for i in selected)


def brute_force_mis(
    n: int,
    edges: Iterable[tuple[int, int]],
    weights: Sequence[float],
) -> tuple[float, List[int]]:
    """Maximum-weight independent set by bitmask (n <= 16)."""
    adj = [0] * n
    for a, b in edges:
        if a == b:
            continue
        adj[a] |= 1 << b
        adj[b] |= 1 << a

    best_w = -1.0
    best_mask = 0
    for mask in range(1 << n):
        ok = True
        w = 0.0
        for i in range(n):
            if mask & (1 << i):
                if mask & adj[i]:
                    ok = False
                    break
                w += weights[i]
        if ok and w > best_w:
            best_w = w
            best_mask = mask
    best_sel = [i for i in range(n) if best_mask & (1 << i)]
    return best_w, best_sel


def _append_elem(g: ElemGraph, group_id: int, x: float, y: float, radius: float = 0.5):
    c = Circle.from_center_radius(x, y, radius)
    g.append_elem_at(group_id, x, y, 0.0, c.center.x, c.center.y, c.r_sq)


def _build_graph(
    n: int,
    edges: Sequence[tuple[int, int]],
    scores: Sequence[float] | None = None,
) -> tuple[ElemGraph, List[float]]:
    g = ElemGraph()
    for i in range(n):
        _append_elem(g, 0, float(i), 0.0)
    for a, b in edges:
        g.add_collision_pair(a, b)
    w = list(scores) if scores is not None else [1.0] * n
    return g, w


def _path3() -> tuple[ElemGraph, List[float]]:
    return _build_graph(3, [(0, 1), (1, 2)], [10.0, 1.0, 10.0])


def _triangle() -> tuple[ElemGraph, List[float]]:
    return _build_graph(3, [(0, 1), (1, 2), (0, 2)], [5.0, 6.0, 4.0])


def _claw() -> tuple[ElemGraph, List[float]]:
    return _build_graph(4, [(0, 1), (0, 2), (0, 3)], [1.0, 10.0, 9.0, 8.0])


def _weighted_star() -> tuple[ElemGraph, List[float]]:
    return _build_graph(5, [(0, i) for i in range(1, 5)], [100.0, 3.0, 4.0, 5.0, 6.0])


@pytest.mark.parametrize(
    "factory",
    [_path3, _triangle, _claw, _weighted_star],
)
def test_nest_by_graph_returns_independent_set(factory):
    g, scores = factory()
    rules = PlacementRuleSet()
    rules.append_point_at(Vec2(0.0, 0.0), 1.0, 1.0, 0)
    selected = nest_by_graph(
        g, [rules], select=_make_select_options("weighted_greedy", True, "sum")
    )[0]
    assert_independent_set(g, selected)
    assert score_sum(scores, selected) >= 0


def test_brute_force_matches_known_instances():
    cases = [
        (3, [(0, 1), (1, 2)], [10.0, 1.0, 10.0], 20.0),
        (3, [(0, 1), (1, 2), (0, 2)], [5.0, 6.0, 4.0], 6.0),
        (4, [(0, 1), (0, 2), (0, 3)], [1.0, 10.0, 9.0, 8.0], 27.0),
        (5, [(0, i) for i in range(1, 5)], [100.0, 3.0, 4.0, 5.0, 6.0], 100.0),
    ]
    for n, edges, weights, expected in cases:
        opt_w, _ = brute_force_mis(n, edges, weights)
        assert abs(opt_w - expected) < 1e-5


def test_refine_monotonic_vs_greedy():
    g, scores = _path3()
    rules = PlacementRuleSet()
    rules.append_point_at(Vec2(0.0, 0.0), 0.5, 1.0, 0)
    greedy = nest_by_graph(
        g, [rules], select=_make_select_options("weighted_greedy", False, "sum")
    )[0]
    opts = RefineSelectionOptions()
    opts.min_score_delta = 0.01
    opts.max_passes = 16
    opts.max_depth = 16
    opts.seed = 1
    opts.beam_width = 4
    refined = refine_selection_dfs(g, greedy, scores, opts)
    assert_independent_set(g, refined)
    assert score_sum(scores, refined) >= score_sum(scores, greedy)


def test_refine_reaches_brute_opt_on_small_graphs():
    cases = [
        (3, [(0, 1), (1, 2)], [10.0, 1.0, 10.0]),
        (4, [(0, 1), (0, 2), (0, 3)], [1.0, 10.0, 9.0, 8.0]),
    ]
    opts = RefineSelectionOptions()
    opts.min_score_delta = 0.01
    opts.max_passes = 32
    opts.max_depth = 32
    opts.seed = 42
    opts.beam_width = 8
    for n, edges, weights in cases:
        g, scores = _build_graph(n, edges, weights)
        opt_w, opt_sel = brute_force_mis(n, edges, weights)
        start = [1] if n == 3 else [0]
        refined = refine_selection_dfs(g, start, scores, opts)
        assert_independent_set(g, refined)
        assert score_sum(scores, refined) >= opt_w - 1e-4


def test_score_rules_prefers_better_rules():
    g, _ = _path3()
    rules_low = PlacementRuleSet()
    rules_low.append_point_at(Vec2(1.0, 0.0), 0.5, 1.0, 0)
    rules_high = PlacementRuleSet()
    rules_high.append_point_at(Vec2(0.0, 0.0), 0.5, 1.0, 0)
    rules_high.append_point_at(Vec2(2.0, 0.0), 0.5, 1.0, 0)
    opts = ScoreRulesOptions()
    opts.rule_complexity_penalty = 0.0
    opts.count_weight = 0.0
    opts.mean_score_weight = 0.0
    opts.selection_score_only = True
    opts.select = _make_select_options("weighted_greedy", True, "sum")
    fitness = score_rules([g], [rules_low, rules_high], opts)
    assert fitness[1] > fitness[0]
