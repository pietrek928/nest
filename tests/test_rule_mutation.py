import numpy as np
import pytest

from nest_graph.build_graph import dedupe_rule_sets, improve_rules
from nest_graph.config import (
    BuildGraphConfig,
    RulesConfig,
    _make_rule_mutation_settings,
    score_rules_options,
)
from nest_graph.elem_graph import (
    Circle,
    PlacementRuleSet,
    ScoreRulesOptions,
    Vec2,
    augment_rules,
    score_rules,
)
from nest_graph.build_graph import make_polygon_graph, _rule_region


@pytest.fixture
def board_region(nest_board):
    return _rule_region(nest_board)


@pytest.fixture
def mutation_preset(board_region):
    return _make_rule_mutation_settings(
        region=board_region,
        dpos=0.1,
        dw=0.1,
        da=np.pi / 8,
        insert_p=0.2,
        remove_p=0.01,
        mutate_p=0.5,
        ngroups=2,
        max_inserts_per_type=2,
    )


@pytest.fixture
def seed_rules(board_region):
    rules = PlacementRuleSet()
    rules.append_point_at(
        Vec2(board_region.center.x, board_region.center.y), 0.1, 1.0, 0
    )
    return [rules]


def test_augment_rules_seeded_reproducible(seed_rules, mutation_preset):
    a = augment_rules(seed_rules, mutation_preset, seed=42)
    b = augment_rules(seed_rules, mutation_preset, seed=42)
    assert len(a) == len(b)
    assert a[0].point_rules[0].pos[0] == pytest.approx(
        b[0].point_rules[0].pos[0]
    )


def test_augment_rules_positions_inside_region(seed_rules, mutation_preset, board_region):
    children = augment_rules(seed_rules, mutation_preset, seed=7)
    assert children
    for child in children:
        for pr in child.point_rules:
            dx = pr.pos[0] - board_region.center.x
            dy = pr.pos[1] - board_region.center.y
            dist_sq = dx * dx + dy * dy
            assert dist_sq <= board_region.r_sq * 1.01 + 1e-6


def test_max_inserts_per_type_caps_growth(seed_rules, board_region):
    preset = _make_rule_mutation_settings(
        region=board_region,
        dpos=0.1,
        dw=0.1,
        da=np.pi / 8,
        insert_p=0.9,
        remove_p=0.0,
        mutate_p=0.0,
        ngroups=2,
        max_inserts_per_type=1,
    )
    children = augment_rules(seed_rules, preset, seed=99)
    assert children
    for child in children:
        assert child.size() <= seed_rules[0].size() + 4


def test_dedupe_rule_sets():
    r = PlacementRuleSet()
    r.append_point_at(Vec2(0.1, 0.2), 0.1, 1.0, 0)
    dup = PlacementRuleSet()
    dup.append_point_at(Vec2(0.1, 0.2), 0.1, 1.0, 0)
    out = dedupe_rule_sets([r, dup, r])
    assert len(out) == 1


def test_score_rules_latest_graph_only(nest_board, rect_poly, small_transforms):
    transforms = small_transforms(8, seed=1)
    g_old, _, _, _ = make_polygon_graph(nest_board, [(rect_poly, transforms[:4])])
    g_new, _, _, _ = make_polygon_graph(nest_board, [(rect_poly, transforms[4:])])
    rules = [PlacementRuleSet()]
    rules[0].append_point_at(Vec2(0.5, 0.5), 0.2, 1.0, 0)
    opts_all = ScoreRulesOptions()
    opts_all.latest_graph_only = False
    all_scores = score_rules([g_old, g_new], rules, opts_all)
    opts_latest = ScoreRulesOptions()
    opts_latest.latest_graph_only = True
    latest_scores = score_rules([g_old, g_new], rules, opts_latest)
    assert latest_scores[0] == pytest.approx(all_scores[0] / 2, rel=1e-5, abs=1e-5) or (
        latest_scores[0] != all_scores[0]
    )


def test_improve_rules_elitist_smoke(nest_board, build_graph_config):
    rules = [PlacementRuleSet()]
    presets = build_graph_config.rules.mutation_presets()
    sel = build_graph_config.selection
    improved = improve_rules(
        [],
        rules,
        2,
        nest_board,
        mutation_presets=presets,
        rule_score_penalty=sel.rule_score_penalty,
        elite_count=1,
        seed=0,
        score_options=score_rules_options(sel),
        max_rules_per_set=build_graph_config.rules.max_rules_per_set,
    )
    assert 1 <= len(improved) <= 2


def test_improve_rules_beats_seed_on_graph(
    nest_board, rect_poly, small_transforms, build_graph_config,
):
    transforms = small_transforms(16, seed=3)
    graph, _, _, _ = make_polygon_graph(nest_board, [(rect_poly, transforms)])
    seed_rules = PlacementRuleSet()
    seed_rules.append_point_at(Vec2(0.5, 0.5), 0.2, 1.0, 0)
    sel = build_graph_config.selection
    improved = improve_rules(
        [graph],
        [seed_rules],
        1,
        nest_board,
        mutation_presets=build_graph_config.rules.mutation_presets(),
        rule_score_penalty=sel.rule_score_penalty,
        elite_count=1,
        seed=1,
        score_options=score_rules_options(sel),
        max_rules_per_set=build_graph_config.rules.max_rules_per_set,
    )
    assert len(improved) == 1
    assert improved[0].size() <= build_graph_config.rules.max_rules_per_set
    opts = score_rules_options(sel)
    seed_score = score_rules([graph], [seed_rules], opts)[0]
    best_score = score_rules([graph], improved, opts)[0]
    assert best_score >= seed_score


def test_mutation_presets_include_max_inserts():
    cfg = RulesConfig()
    presets = cfg.mutation_presets()
    assert len(presets) == 3
    for preset in presets:
        assert preset.max_inserts_per_type == cfg.max_inserts_per_type


def test_selection_defaults_tuned():
    cfg = BuildGraphConfig()
    assert cfg.selection.improve_rules_rounds == 2
    assert cfg.selection.improve_rules_elite_count == 16
    assert cfg.selection.score_rules_latest_graph_only is True
    assert cfg.selection.select_mode == "weighted_greedy"
    assert cfg.selection.score_rules_count_weight == 0.02
