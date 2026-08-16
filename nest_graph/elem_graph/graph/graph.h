#pragma once

#include <cstdint>
#include <vector>

#include "rules/rules.h"


typedef struct PosePlace {
    Vec2f pos;
    float a;
} PosePlace;

typedef struct ElemGroup {
    int count_limit;
    float priority;
} ElemGroup;

typedef struct AttractEdge {
    Tvertex target;
    float w;
} AttractEdge;

typedef struct PoseGraph {
    std::vector<Tvertex> group_id;
    std::vector<PosePlace> elems;
    std::vector<Circle2f> coords;
    std::vector<std::vector<Tvertex>> collisions;
    std::vector<std::vector<AttractEdge>> attract;

    auto size() const { return group_id.size(); }
} PoseGraph;

enum class ScoreAggregation : int { Sum = 0, Max = 1 };
enum class SelectMode : int { GreedyScore = 0, WeightedGreedy = 1 };

struct SelectOptions {
    SelectMode mode = SelectMode::WeightedGreedy;
    bool local_swap = true;

    ScoreAggregation aggregation = ScoreAggregation::Sum;
    std::vector<Tvertex> locked_indices;
};

struct RefineSelectionOptions {
    float min_score_delta = 1e-6f;
    /** Hard cap on refinement sweeps (safety). */
    int max_passes = 1024;
    /** Stop after this many consecutive sweeps with no score gain. */
    int max_stagnant_passes = 4;
    int max_depth = 64;
    std::uint32_t seed = 0;
    bool explore_shuffle = false;
    /** Independent growth shuffles from seed+r (deterministic stand-in for
     *  former random_device growth). 1 = single stream. */
    int growth_restarts = 1;
    int beam_width = 2;
    int max_root_collisions = 2;
    int max_tries = 0;
    int min_collisions = 0;
    /** Optional per-node areas (same length as graph); empty disables area lex. */
    std::vector<float> node_areas;
    /** Prefer count, then area sum, then score sum when accepting refine results. */
    bool lexicographic_area = false;
    std::vector<Tvertex> locked_indices;
};

struct FinalizeSelectionOptions {
    int repair_passes = 8;
    int max_exact_component_size = 18;
    std::vector<Tvertex> locked_indices;
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
    const PoseGraph &g,
    const std::vector<PlacementRuleSet> &cases,
    const SelectOptions &select = SelectOptions{}
);

// Weighted MIS using a precomputed score vector (does not call compute_scores).
std::vector<Tvertex> nest_by_scores(
    const PoseGraph &g,
    const std::vector<Tscore> &scores,
    const SelectOptions &select = SelectOptions{}
);

PoseGraph sort_graph(const PoseGraph &g, const PlacementRuleSet &rules, bool reverse);

std::vector<Tscore> score_elems(
    const PoseGraph &g,
    const PlacementRuleSet &rules,
    ScoreAggregation aggregation = ScoreAggregation::Sum);

std::vector<Tscore> score_rules(
    const std::vector<PoseGraph> &graphs,
    const std::vector<PlacementRuleSet> &rule_sets,
    const ScoreRulesOptions &options = ScoreRulesOptions{}
);

std::vector<Tvertex> refine_selection_dfs(
    const PoseGraph &g,
    const std::vector<Tvertex> &selected_nodes,
    const std::vector<Tscore> &scores,
    const RefineSelectionOptions &options = RefineSelectionOptions{}
);

std::vector<Tvertex> increase_selection_dfs(
    const PoseGraph &g,
    const std::vector<Tvertex> &selected_nodes,
    int max_tries);

std::vector<Tvertex> increase_score_dfs(
    const PoseGraph &g,
    const std::vector<Tvertex> &selected_nodes,
    const std::vector<Tscore> &scores,
    const RefineSelectionOptions &options = RefineSelectionOptions{}
);

std::vector<Tvertex> refine_selection(
    const PoseGraph &g,
    const std::vector<Tvertex> &selected_nodes,
    const std::vector<Tscore> &scores,
    const RefineSelectionOptions &options = RefineSelectionOptions{}
);

std::vector<Tvertex> finalize_selection(
    const PoseGraph &g,
    const std::vector<Tvertex> &selected_nodes,
    const std::vector<Tscore> &scores,
    const FinalizeSelectionOptions &options = FinalizeSelectionOptions{},
    FinalizeSelectionStats *stats = nullptr);

// Greedy score-ordered independent set over ``verts`` (collision-graph).
std::vector<int> greedy_weighted_mis(
    const std::vector<Tvertex> &verts,
    const std::vector<Tscore> &scores,
    const PoseGraph &g,
    const std::vector<Tvertex> &locked_indices = {},
    const unsigned char *kept_outside = nullptr);

bool selection_is_independent(
    const PoseGraph &g, const std::vector<Tvertex> &selected_nodes);
