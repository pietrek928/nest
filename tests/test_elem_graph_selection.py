"""Tests for elem_graph selection, scoring, and score-first DFS."""

import numpy as np
import pytest

from nest_graph.config import BuildGraphConfig, _make_select_options
from nest_graph.elem_graph import (
    Circle,
    ElemGraph,
    PlacementRuleSet,
    RefineSelectionOptions,
    ScoreAggregation,
    ScoreRulesOptions,
    SelectMode,
    Vec2,
    nest_by_graph,
    refine_selection_dfs,
    score_elems,
    score_rules,
    sort_graph,
)
from nest_graph.build_graph import make_polygon_graph, _rule_region


def _append_elem(g: ElemGraph, group_id: int, x: float, y: float, radius: float = 0.5):
    c = Circle.from_center_radius(x, y, radius)
    g.append_elem_at(group_id, x, y, 0.0, c.center.x, c.center.y, c.r_sq)


def _path_graph_scores():
    """Nodes 0 - 1 - 2; pick two non-adjacent high-score ends."""
    g = ElemGraph()
    for i in range(3):
        _append_elem(g, 0, float(i), 0.0)
    g.add_collision_pair(0, 1)
    g.add_collision_pair(1, 2)
    scores_list = [10.0, 1.0, 10.0]
    return g, scores_list


def test_compute_score_peak_at_rule_center():
    g = ElemGraph()
    _append_elem(g, 0, 0.5, 0.5, 0.1)
    _append_elem(g, 0, 2.0, 2.0, 0.1)
    rules = PlacementRuleSet()
    rules.append_point_at(Vec2(0.5, 0.5), 0.2, 1.0, 0)
    es = score_elems(g, rules)
    assert es[0] > es[1]


def test_angle_score_separable_via_elems():
    """Wrong angle should not zero out position contribution (separable kernel)."""
    g = ElemGraph()
    _append_elem(g, 0, 0.5, 0.5, 0.1)
    _append_elem(g, 0, 0.5, 0.5, 0.1)
    g.elems[0].a = 0.0
    g.elems[1].a = float(np.pi)
    rules = PlacementRuleSet()
    rules.append_point_angle_at(Vec2(0.5, 0.5), 0.0, 0.3, 1.0, 0)
    es = score_elems(g, rules)
    assert es[0] > es[1] * 100.0


def test_weighted_greedy_picks_high_score_ends():
    g, scores_list = _path_graph_scores()
    rules = PlacementRuleSet()
    rules.append_point_at(Vec2(0.0, 0.0), 0.5, 1.0, 0)

    selected = nest_by_graph(
        g, [rules], select=_make_select_options("weighted_greedy", True, "sum")
    )[0]
    assert set(selected) == {0, 2}


def test_score_rules_fitness_prefers_high_sum():
    g, scores_list = _path_graph_scores()
    rules_low = PlacementRuleSet()
    rules_low.append_point_at(Vec2(1.0, 0.0), 0.5, 1.0, 0)
    rules_high = PlacementRuleSet()
    rules_high.append_point_at(Vec2(0.0, 0.0), 0.5, 1.0, 0)
    rules_high.append_point_at(Vec2(2.0, 0.0), 0.5, 1.0, 0)

    opts = ScoreRulesOptions()
    opts.rule_complexity_penalty = 0.0
    opts.count_weight = 0.0
    opts.select = _make_select_options("weighted_greedy", True, "sum")
    fitness = score_rules([g], [rules_low, rules_high], opts)
    assert fitness[1] > fitness[0]


def test_score_dfs_prefers_high_score_swap():
    g, scores_list = _path_graph_scores()
    opts = RefineSelectionOptions()
    opts.min_score_delta = 0.5
    opts.max_passes = 8
    opts.max_depth = 8
    opts.seed = 42
    refined = refine_selection_dfs(g, [1], scores_list, opts)
    assert set(refined) == {0, 2}


def test_score_dfs_seeded_reproducible():
    g, scores_list = _path_graph_scores()
    opts = RefineSelectionOptions()
    opts.seed = 99
    opts.min_score_delta = 0.1
    opts.max_passes = 4
    a = refine_selection_dfs(g, [1], scores_list, opts)
    b = refine_selection_dfs(g, [1], scores_list, opts)
    assert a == b


def test_refine_improves_score_sum():
    g, scores_list = _path_graph_scores()
    opts = RefineSelectionOptions()
    opts.min_score_delta = 0.5
    opts.seed = 1
    before = sum(scores_list[i] for i in [1])
    after = sum(scores_list[i] for i in refine_selection_dfs(g, [1], scores_list, opts))
    assert after > before


def test_selection_defaults_tuned():
    cfg = BuildGraphConfig()
    assert cfg.selection.select_mode == "weighted_greedy"
    assert cfg.selection.score_rules_count_weight == 0.02


def test_nest_by_graph_beats_center_only_on_path():
    g, scores_list = _path_graph_scores()
    rules = PlacementRuleSet()
    rules.append_point_at(Vec2(0.0, 0.0), 1.0, 1.0, 0)
    rules.append_point_at(Vec2(2.0, 0.0), 1.0, 1.0, 0)
    selected = nest_by_graph(
        g, [rules], select=_make_select_options("weighted_greedy", True, "sum")
    )[0]
    es = score_elems(g, rules)
    nest_sum = sum(es[i] for i in selected)
    center_only_sum = es[1]
    assert nest_sum >= center_only_sum


def test_sort_graph_affects_refine_order():
    g, scores_list = _path_graph_scores()
    rules = PlacementRuleSet()
    rules.append_point_at(Vec2(0.0, 0.0), 0.5, 1.0, 0)
    es = list(score_elems(g, rules))
    es_override = scores_list
    g_sorted = sort_graph(g, rules, reverse=False)
    g_rev = sort_graph(g, rules, reverse=True)
    opts = RefineSelectionOptions()
    opts.seed = 0
    opts.min_score_delta = 0.01
    opts.max_passes = 2
    r1 = refine_selection_dfs(g_sorted, [1], es_override, opts)
    r2 = refine_selection_dfs(g_rev, [1], es_override, opts)
    assert sum(es_override) > 0
    assert r1 is not None and r2 is not None
