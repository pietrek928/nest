#include <algorithm>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "graph/graph.h"
#include "rules/rules_util.h"
#include "scoring/scoring.h"
#include "tests/elem_graph_test_helpers.h"

TEST_CASE("nest_by_graph path returns independent set", "[selection]") {
    const std::vector<float> scores = {10.0f, 1.0f, 10.0f};
    PoseGraph g = path_graph(3, scores);
    PlacementRuleSet rules = point_rule_at(0.0f, 0.0f);

    SelectOptions opts;
    opts.mode = SelectMode::WeightedGreedy;
    opts.local_swap = true;
    opts.aggregation = ScoreAggregation::Sum;

    const auto nested = nest_by_graph(g, {rules}, opts);
    REQUIRE(nested.size() == 1);
    const auto &selected = nested[0];
    REQUIRE(is_independent_set(g, selected));
    REQUIRE(selected.size() == 2);
    REQUIRE(selected[0] == 0);
    REQUIRE(selected[1] == 2);  // weighted ends on a path
}

TEST_CASE("score_elems peaks at rule center", "[selection]") {
    PoseGraph g;
    append_elem_at(g, 0, 0.5f, 0.5f, 0.01f);
    append_elem_at(g, 0, 2.0f, 2.0f, 0.01f);
    PlacementRuleSet rules = point_rule_at(0.5f, 0.5f, 0.2f, 1.0f);

    const std::vector<float> es = score_elems(g, rules, ScoreAggregation::Sum);
    REQUIRE(es.size() == 2);
    REQUIRE(es[0] > es[1]);
}

TEST_CASE("score_transform matches score_elems at rule center", "[selection]") {
    PoseGraph g;
    append_elem_at(g, 0, 0.5f, 0.5f, 0.0f);
    PlacementRuleSet rules = point_rule_at(0.5f, 0.5f, 0.2f, 1.0f);

    const float direct = score_transform(
        rules, 0, Vec2f({0.5f, 0.5f}), 0.0f, ScoreAggregation::Sum);
    const std::vector<float> es = score_elems(g, rules, ScoreAggregation::Sum);
    REQUIRE(es.size() == 1);
    REQUIRE(direct == Catch::Approx(es[0]).margin(1e-5f));
}

TEST_CASE("nest_by_scores keeps non-adjacent locks", "[selection]") {
    const std::vector<float> scores = {1.0f, 100.0f, 1.0f, 1.0f};
    PoseGraph g = path_graph(4, scores);
    SelectOptions opts;
    opts.local_swap = true;
    opts.locked_indices = {0, 2};
    const auto selected = nest_by_scores(g, scores, opts);
    REQUIRE(is_independent_set(g, selected));
    REQUIRE(std::find(selected.begin(), selected.end(), 0) != selected.end());
    REQUIRE(std::find(selected.begin(), selected.end(), 2) != selected.end());
    REQUIRE(std::find(selected.begin(), selected.end(), 1) == selected.end());
}

TEST_CASE("nest_by_scores drops later lock that collides with earlier lock", "[selection]") {
    const std::vector<float> scores = {1.0f, 1.0f, 1.0f};
    PoseGraph g = path_graph(3, scores);
    SelectOptions opts;
    opts.local_swap = false;
    opts.locked_indices = {0, 1};
    const auto selected = nest_by_scores(g, scores, opts);
    REQUIRE(is_independent_set(g, selected));
    REQUIRE(std::find(selected.begin(), selected.end(), 0) != selected.end());
    REQUIRE(std::find(selected.begin(), selected.end(), 1) == selected.end());
}

TEST_CASE("local_swap cannot eject a lock", "[selection]") {
    const std::vector<float> scores = {1.0f, 100.0f};
    PoseGraph g = path_graph(2, scores);
    SelectOptions opts;
    opts.local_swap = true;
    opts.locked_indices = {0};
    const auto selected = nest_by_scores(g, scores, opts);
    REQUIRE(selected.size() == 1);
    REQUIRE(selected[0] == 0);
}

TEST_CASE("two_swap fires on a path and cannot eject a lock", "[selection]") {
    // 0-1-2-3; scores make 1-for-2 profitable (replace 1 with 0 and 2).
    const std::vector<float> scores = {10.0f, 1.0f, 10.0f, 1.0f};
    PoseGraph g = path_graph(4, scores);
    SelectOptions opts;
    opts.mode = SelectMode::GreedyScore;
    opts.local_swap = true;
    const auto selected = nest_by_scores(g, scores, opts);
    REQUIRE(is_independent_set(g, selected));
    REQUIRE(selected.size() >= 2);

    opts.locked_indices = {1};
    const auto locked_sel = nest_by_scores(g, scores, opts);
    REQUIRE(is_independent_set(g, locked_sel));
    REQUIRE(std::find(locked_sel.begin(), locked_sel.end(), 1) != locked_sel.end());
}

TEST_CASE("score_transform uses provided radius for circle rules", "[selection]") {
    PlacementRuleSet rules;
    append_circle_place_rule_at(rules, Circle2f(Vec2f({0.0f, 0.0f}), 1.0f), 1.0f, 1.0f, 0);

    const float small = score_transform(
        rules, 0, Vec2f({0.0f, 0.0f}), 0.0f, ScoreAggregation::Sum, 0.1f);
    const float large = score_transform(
        rules, 0, Vec2f({0.0f, 0.0f}), 0.0f, ScoreAggregation::Sum, 2.0f);
    REQUIRE(large >= small);
}
