#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "decision_graph.h"
#include "pose/pose_graph.h"
#include "rules/rules.h"

enum class DfsMode : int {
    NestOnly = 0,
    HeadPipeline = 1,
    StrictNoPrune = 2,
    StrictPrune = 3,
    LegacyAlternating = 4,
    MergedLooseTight = 5,
    MergedLooseFinalizeEnd = 6,
    MergedLooseTightFinalizeEnd = 7,
    MergedSinglePass = 8,
    HighPassLoose = 9,
};

inline DfsMode dfs_mode_from_string(const std::string &s) {
    if (s == "nest_only") {
        return DfsMode::NestOnly;
    }
    if (s == "head_pipeline") {
        return DfsMode::HeadPipeline;
    }
    if (s == "strict_no_prune") {
        return DfsMode::StrictNoPrune;
    }
    if (s == "strict_prune") {
        return DfsMode::StrictPrune;
    }
    if (s == "legacy_alternating") {
        return DfsMode::LegacyAlternating;
    }
    if (s == "merged_loose_tight") {
        return DfsMode::MergedLooseTight;
    }
    if (s == "merged_loose_finalize_end") {
        return DfsMode::MergedLooseFinalizeEnd;
    }
    if (s == "merged_loose_tight_finalize_end") {
        return DfsMode::MergedLooseTightFinalizeEnd;
    }
    if (s == "merged_single_pass") {
        return DfsMode::MergedSinglePass;
    }
    if (s == "high_pass_loose") {
        return DfsMode::HighPassLoose;
    }
    return DfsMode::MergedLooseTight;
}

struct DfsDispatcherConfig {
    int dfs_max_tries = 8;
    int dfs_refine_max_passes = 1024;
    int dfs_refine_max_stagnant_passes = 4;
    int dfs_refine_beam_width = 2;
    bool refine_explore_shuffle = false;
    int dfs_growth_restarts = 1;
    bool refine_lexicographic_area = true;
    int dfs_finalize_repair_passes = 8;
    int dfs_finalize_max_component = 18;
    bool dg_aware_refine = true;
    float motif_refine_fracture_penalty = 1.0f;
};

struct ApplyDfsResult {
    std::vector<Tvertex> pre_finalize;
    std::vector<Tvertex> final_sel;
    float score_sum = 0.f;
    double dfs_loose_ms = 0.0;
    double finalize_ms = 0.0;
};

std::vector<Tvertex> prune_selection_to_independent_set(
    const PoseGraph &g,
    const std::vector<Tvertex> &selected,
    const std::vector<Tscore> *scores = nullptr);

ApplyDfsResult apply_dfs_refinement(
    const PoseGraph &graph,
    const PlacementRuleSet &rule_set,
    std::vector<Tvertex> selected,
    const std::vector<Tscore> &scores,
    int dfs_passes,
    int dfs_max_tries,
    DfsMode mode,
    const DfsDispatcherConfig &cfg,
    const FinalizeSelectionOptions &finalize_opts,
    const std::vector<float> *node_areas = nullptr,
    int refine_seed = -1,
    const DecisionGraph *dg = nullptr);
