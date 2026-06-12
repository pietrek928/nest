#include <catch2/catch_test_macros.hpp>

#include "graph/graph.h"
#include "rules/rules_util.h"
#include "scoring/scoring.h"
#include "tests/elem_graph_test_helpers.h"

TEST_CASE("nest_by_graph path returns independent set", "[selection]") {
    const std::vector<float> scores = {10.0f, 1.0f, 10.0f};
    ElemGraph g = path_graph(3, scores);
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
    ElemGraph g;
    append_elem_at(g, 0, 0.5f, 0.5f, 0.01f);
    append_elem_at(g, 0, 2.0f, 2.0f, 0.01f);
    PlacementRuleSet rules = point_rule_at(0.5f, 0.5f, 0.2f, 1.0f);

    const std::vector<float> es = score_elems(g, rules, ScoreAggregation::Sum);
    REQUIRE(es.size() == 2);
    REQUIRE(es[0] > es[1]);
}

TEST_CASE("score_transform matches score_elems at rule center", "[selection]") {
    ElemGraph g;
    append_elem_at(g, 0, 0.5f, 0.5f, 0.0f);
    PlacementRuleSet rules = point_rule_at(0.5f, 0.5f, 0.2f, 1.0f);

    const float direct = score_transform(
        rules, 0, Vec2f({0.5f, 0.5f}), 0.0f, ScoreAggregation::Sum);
    const std::vector<float> es = score_elems(g, rules, ScoreAggregation::Sum);
    REQUIRE(es.size() == 1);
    REQUIRE(direct == Approx(es[0]).margin(1e-5f));
}

TEST_CASE("score_transform negative weight repels at center", "[selection]") {
    PlacementRuleSet rules;
    append_point_place_rule_at(rules, Vec2f({0.5f, 0.5f}), 0.2f, -1.0f, 0);
    const float at_center = score_transform(
        rules, 0, Vec2f({0.5f, 0.5f}), 0.0f, ScoreAggregation::Sum);
    const float far = score_transform(
        rules, 0, Vec2f({2.0f, 2.0f}), 0.0f, ScoreAggregation::Sum);
    REQUIRE(at_center < far);
}
