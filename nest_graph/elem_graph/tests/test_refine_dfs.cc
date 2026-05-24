#include <catch2/catch_test_macros.hpp>

#include "graph/graph.h"
#include "tests/elem_graph_test_helpers.h"

TEST_CASE("increase_selection_dfs stays independent", "[refine]") {
    ElemGraph g = path_graph(5, {1, 1, 1, 1, 1});
    std::vector<int> selected = {0, 2, 4};
    REQUIRE(is_independent_set(g, selected));

    const std::vector<int> grown = increase_selection_dfs(g, selected, 8);
    REQUIRE(is_independent_set(g, grown));
}

TEST_CASE("increase_score_dfs stays independent", "[refine]") {
    ElemGraph g = path_graph(5, {10, 1, 8, 1, 9});
    std::vector<float> scores = {10.0f, 1.0f, 8.0f, 1.0f, 9.0f};
    std::vector<int> selected = {0, 2, 4};
    REQUIRE(is_independent_set(g, selected));

    RefineSelectionOptions opts;
    opts.max_passes = 4;
    opts.seed = 1;

    const std::vector<int> refined =
        increase_score_dfs(g, selected, scores, opts);
    REQUIRE(is_independent_set(g, refined));
}

TEST_CASE("refine_selection returns independent set", "[refine]") {
    ElemGraph g = path_graph(6, {5, 1, 4, 1, 3, 2});
    std::vector<float> scores = {5.0f, 1.0f, 4.0f, 1.0f, 3.0f, 2.0f};
    std::vector<int> selected = {0, 2, 4};

    RefineSelectionOptions opts;
    opts.max_tries = 4;
    opts.min_collisions = 2;
    opts.max_root_collisions = 2;

    const std::vector<int> refined = refine_selection(g, selected, scores, opts);
    REQUIRE(is_independent_set(g, refined));
}

TEST_CASE("finalize_selection repairs overlapping selection", "[refine]") {
    ElemGraph g = star_graph(3);
    std::vector<float> scores = {3.0f, 2.0f, 2.0f, 2.0f};
    const std::vector<int> overlapping = {0, 1, 2};

    REQUIRE_FALSE(is_independent_set(g, overlapping));

    const std::vector<int> finalized =
        finalize_selection(g, overlapping, scores);
    REQUIRE(is_independent_set(g, finalized));
    REQUIRE(finalized.size() >= 2);
}
