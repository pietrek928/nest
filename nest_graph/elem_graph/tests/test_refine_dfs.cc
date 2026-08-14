#include <algorithm>

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

TEST_CASE("refine cannot overlap a pin with a new node", "[refine]") {
    ElemGraph g = path_graph(3, {1.0f, 100.0f, 1.0f});
    std::vector<float> scores = {1.0f, 100.0f, 1.0f};
    RefineSelectionOptions opts;
    opts.max_passes = 8;
    opts.max_tries = 4;
    opts.seed = 1;
    opts.locked_indices = {0};
    const auto refined = refine_selection(g, {0}, scores, opts);
    REQUIRE(is_independent_set(g, refined));
    REQUIRE(std::find(refined.begin(), refined.end(), 0) != refined.end());
}

TEST_CASE("finalize cannot drop a lock for score", "[refine]") {
    ElemGraph g = path_graph(3, {1.0f, 100.0f, 1.0f});
    std::vector<float> scores = {1.0f, 100.0f, 1.0f};
    FinalizeSelectionOptions opts;
    opts.locked_indices = {0, 2};
    const auto finalized = finalize_selection(g, {0, 1, 2}, scores, opts);
    REQUIRE(is_independent_set(g, finalized));
    REQUIRE(std::find(finalized.begin(), finalized.end(), 0) != finalized.end());
    REQUIRE(std::find(finalized.begin(), finalized.end(), 2) != finalized.end());
    REQUIRE(std::find(finalized.begin(), finalized.end(), 1) == finalized.end());
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

TEST_CASE("kiss attract cannot beat part count in finalize", "[refine]") {
    // Path 0-1-2: max independent is {0,2}. Huge attract on {0,2} must not
    // drop to the high-score singleton {1}.
    std::vector<float> scores = {1.0f, 100.0f, 1.0f};
    ElemGraph g = path_graph(3, scores);
    add_attract_pair(g, 0, 2, 1000.0f);
    const auto finalized = finalize_selection(g, {0, 1, 2}, scores);
    REQUIRE(is_independent_set(g, finalized));
    REQUIRE(finalized.size() == 2);
    REQUIRE(std::find(finalized.begin(), finalized.end(), 0) != finalized.end());
    REQUIRE(std::find(finalized.begin(), finalized.end(), 2) != finalized.end());
}
