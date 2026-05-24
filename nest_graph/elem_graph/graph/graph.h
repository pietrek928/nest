#pragma once

#include <cstdint>
#include <vector>

#include "rules/rules.h"


typedef struct ElemPlace {
    Vec2f pos;
    float a;
} ElemPlace;

typedef struct ElemGroup {
    int count_limit;
    float priority;
} ElemGroup;

typedef struct ElemGraph {
    std::vector<Tvertex> group_id;
    std::vector<ElemPlace> elems;
    std::vector<Circle2f> coords;
    std::vector<std::vector<Tvertex>> collisions;

    auto size() const { return group_id.size(); }
} ElemGraph;

enum class ScoreAggregation : int { Sum = 0, Max = 1 };
enum class SelectMode : int { GreedyScore = 0, WeightedGreedy = 1 };

struct SelectOptions {
    SelectMode mode = SelectMode::WeightedGreedy;
    bool local_swap = true;
    ScoreAggregation aggregation = ScoreAggregation::Sum;
};

struct RefineSelectionOptions {
    float min_score_delta = 1e-6f;
    int max_passes = 32;
    int max_depth = 64;
    std::uint32_t seed = 0;
    bool explore_shuffle = false;
    int beam_width = 2;
    int max_root_collisions = 2;
    int max_tries = 0;
    int min_collisions = 0;
};

struct FinalizeSelectionOptions {
    int repair_passes = 8;
    int max_exact_component_size = 18;
};

struct FinalizeSelectionStats {
    int repair_passes_used = 0;
    int optimal_components = 0;
    int greedy_fallback_components = 0;
    int nodes_dropped = 0;
};

struct ScoreRulesOptions {
    float rule_complexity_penalty = 0.0f;
    bool latest_graph_only = false;
    float mean_score_weight = 0.0f;
    float count_weight = 0.02f;
    bool selection_score_only = false;
    SelectOptions select{};
};

std::vector<std::vector<Tvertex>> nest_by_graph(
    const ElemGraph &g,
    const std::vector<PlacementRuleSet> &cases,
    const SelectOptions &select = SelectOptions{}
);

ElemGraph sort_graph(const ElemGraph &g, const PlacementRuleSet &rules, bool reverse);

std::vector<Tscore> score_elems(
    const ElemGraph &g,
    const PlacementRuleSet &rules,
    ScoreAggregation aggregation = ScoreAggregation::Sum);

std::vector<Tscore> score_rules(
    const std::vector<ElemGraph> &graphs,
    const std::vector<PlacementRuleSet> &rule_sets,
    const ScoreRulesOptions &options = ScoreRulesOptions{}
);

std::vector<Tvertex> refine_selection_dfs(
    const ElemGraph &g,
    const std::vector<Tvertex> &selected_nodes,
    const std::vector<Tscore> &scores,
    const RefineSelectionOptions &options = RefineSelectionOptions{}
);

std::vector<Tvertex> increase_selection_dfs(
    const ElemGraph &g,
    const std::vector<Tvertex> &selected_nodes,
    int max_tries);

std::vector<Tvertex> increase_score_dfs(
    const ElemGraph &g,
    const std::vector<Tvertex> &selected_nodes,
    const std::vector<Tscore> &scores,
    const RefineSelectionOptions &options = RefineSelectionOptions{}
);

std::vector<Tvertex> refine_selection(
    const ElemGraph &g,
    const std::vector<Tvertex> &selected_nodes,
    const std::vector<Tscore> &scores,
    const RefineSelectionOptions &options = RefineSelectionOptions{}
);

std::vector<Tvertex> finalize_selection(
    const ElemGraph &g,
    const std::vector<Tvertex> &selected_nodes,
    const std::vector<Tscore> &scores,
    const FinalizeSelectionOptions &options = FinalizeSelectionOptions{},
    FinalizeSelectionStats *stats = nullptr);

bool selection_is_independent(
    const ElemGraph &g, const std::vector<Tvertex> &selected_nodes);
