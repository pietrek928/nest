#include "refine/apply_dfs.h"

#include <algorithm>
#include <numeric>

#include "refine/finalize.h"

namespace {

float selection_score_sum(
    const std::vector<Tscore> &scores,
    const std::vector<Tvertex> &selected) {
    float total = 0.f;
    for (Tvertex v : selected) {
        if (v >= 0 && static_cast<std::size_t>(v) < scores.size()) {
            total += scores[static_cast<std::size_t>(v)];
        }
    }
    return total;
}

int dfs_refine_seed(int seed0, int pass_i) {
    if (seed0 < 0) {
        return -1;
    }
    return seed0 + pass_i * 17;
}

void apply_dg_refine_options(
    RefineSelectionOptions &opts,
    const DecisionGraph *dg,
    const DfsDispatcherConfig &cfg) {
    if (!cfg.dg_aware_refine || dg == nullptr) {
        return;
    }
    opts.dg_aware_refine = true;
    opts.motif_fracture_penalty = cfg.motif_refine_fracture_penalty;
    opts.motif_join_pairs.clear();
    for (const MotifJoin &m : dg->motifs()) {
        if (m.a >= 0 && m.b >= 0) {
            opts.motif_join_pairs.push_back({m.a, m.b});
        }
    }
}

RefineSelectionOptions base_refine_options(
    const DfsDispatcherConfig &cfg,
    bool loose,
    const std::vector<float> *node_areas,
    int seed,
    const DecisionGraph *dg) {
    RefineSelectionOptions opts;
    opts.max_tries = cfg.dfs_max_tries;
    opts.max_passes = cfg.dfs_refine_max_passes;
    opts.max_stagnant_passes = cfg.dfs_refine_max_stagnant_passes;
    opts.beam_width = cfg.dfs_refine_beam_width;
    opts.explore_shuffle = cfg.refine_explore_shuffle;
    opts.growth_restarts = std::max(1, cfg.dfs_growth_restarts);
    if (seed >= 0) {
        opts.seed = static_cast<std::uint32_t>(seed) & 0xFFFFFFFFu;
    }
    opts.lexicographic_area = cfg.refine_lexicographic_area;
    if (node_areas != nullptr && opts.lexicographic_area) {
        opts.node_areas = *node_areas;
    }
    if (loose) {
        opts.min_collisions = 2;
        opts.max_root_collisions = 2;
    } else {
        opts.min_collisions = 1;
        opts.max_root_collisions = 1;
    }
    apply_dg_refine_options(opts, dg, cfg);
    return opts;
}

RefineSelectionOptions strict_refine_options(
    const DfsDispatcherConfig &cfg,
    const std::vector<float> *node_areas,
    int seed,
    const DecisionGraph *dg) {
    RefineSelectionOptions opts = base_refine_options(cfg, false, node_areas, seed, dg);
    opts.min_collisions = 0;
    opts.max_root_collisions = 0;
    return opts;
}

std::vector<Tvertex> dfs_finalize_selection(
    const PoseGraph &graph,
    const std::vector<Tvertex> &selected,
    const std::vector<Tscore> &scores,
    const FinalizeSelectionOptions &finalize_opts) {
    return finalize_selection(graph, selected, scores, finalize_opts, nullptr);
}

}  // namespace

std::vector<Tvertex> prune_selection_to_independent_set(
    const PoseGraph &g,
    const std::vector<Tvertex> &selected,
    const std::vector<Tscore> *scores) {
    if (selected.empty()) {
        return {};
    }
    std::vector<Tvertex> order = selected;
    if (scores != nullptr && scores->size() == g.group_id.size()) {
        std::sort(order.begin(), order.end(), [&scores](Tvertex a, Tvertex b) {
            return (*scores)[static_cast<std::size_t>(a)]
                > (*scores)[static_cast<std::size_t>(b)];
        });
    }
    std::vector<Tvertex> kept;
    std::vector<bool> kept_set(g.size(), false);
    for (Tvertex v : order) {
        if (v < 0 || static_cast<std::size_t>(v) >= g.collisions.size()) {
            continue;
        }
        bool collides = false;
        for (Tvertex u : g.collisions[static_cast<std::size_t>(v)]) {
            if (u >= 0 && static_cast<std::size_t>(u) < kept_set.size() && kept_set[static_cast<std::size_t>(u)]) {
                collides = true;
                break;
            }
        }
        if (collides) {
            continue;
        }
        kept.push_back(v);
        kept_set[static_cast<std::size_t>(v)] = true;
    }
    return kept;
}

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
    const std::vector<float> *node_areas,
    int refine_seed,
    const DecisionGraph *dg) {
    ApplyDfsResult out;
    const PoseGraph graph_sorted = sort_graph(graph, rule_set, false);
    const PoseGraph graph_sorted_rev = sort_graph(graph, rule_set, true);
    std::vector<Tvertex> pre_finalize = selected;

    if (mode == DfsMode::NestOnly) {
        out.pre_finalize = selected;
        out.final_sel = selected;
        out.score_sum = selection_score_sum(scores, selected);
        return out;
    }

    if (mode == DfsMode::LegacyAlternating) {
        for (int i = 0; i < dfs_passes; ++i) {
            (void)i;
            selected = increase_selection_dfs(graph_sorted_rev, selected, dfs_max_tries);
            selected = increase_selection_dfs(graph, selected, dfs_max_tries);
            selected = increase_score_dfs(graph_sorted_rev, selected, scores);
            selected = increase_selection_dfs(graph_sorted, selected, dfs_max_tries);
            selected = increase_score_dfs(graph_sorted, selected, scores);
        }
        pre_finalize = selected;
        out.pre_finalize = pre_finalize;
        out.final_sel = dfs_finalize_selection(graph, selected, scores, finalize_opts);
        out.score_sum = selection_score_sum(scores, out.final_sel);
        return out;
    }

    if (mode == DfsMode::HeadPipeline) {
        RefineSelectionOptions loose = base_refine_options(cfg, true, node_areas, -1, dg);
        RefineSelectionOptions tight;
        tight.min_collisions = 1;
        tight.max_root_collisions = 2;
        tight.max_passes = cfg.dfs_refine_max_passes;
        tight.max_stagnant_passes = cfg.dfs_refine_max_stagnant_passes;
        tight.beam_width = cfg.dfs_refine_beam_width;
        apply_dg_refine_options(tight, dg, cfg);
        for (int i = 0; i < dfs_passes; ++i) {
            (void)i;
            selected = increase_selection_dfs(graph_sorted_rev, selected, dfs_max_tries);
            selected = increase_selection_dfs(graph, selected, dfs_max_tries);
            selected = increase_score_dfs(graph_sorted_rev, selected, scores, loose);
            selected = increase_selection_dfs(graph_sorted, selected, dfs_max_tries);
            selected = increase_score_dfs(graph_sorted, selected, scores, tight);
        }
        pre_finalize = selected;
        out.pre_finalize = pre_finalize;
        out.final_sel = pre_finalize;
        out.score_sum = selection_score_sum(scores, pre_finalize);
        return out;
    }

    if (mode == DfsMode::StrictNoPrune) {
        for (int pass_i = 0; pass_i < dfs_passes; ++pass_i) {
            RefineSelectionOptions strict = strict_refine_options(
                cfg, node_areas, dfs_refine_seed(refine_seed, pass_i), dg);
            selected = refine_selection(graph_sorted_rev, selected, scores, strict);
            selected = refine_selection(graph, selected, scores, strict);
        }
        pre_finalize = selected;
        out.pre_finalize = pre_finalize;
        out.final_sel = pre_finalize;
        out.score_sum = selection_score_sum(scores, pre_finalize);
        return out;
    }

    if (mode == DfsMode::StrictPrune) {
        for (int pass_i = 0; pass_i < dfs_passes; ++pass_i) {
            RefineSelectionOptions strict = strict_refine_options(
                cfg, node_areas, dfs_refine_seed(refine_seed, pass_i), dg);
            selected = refine_selection(graph_sorted_rev, selected, scores, strict);
            selected = refine_selection(graph, selected, scores, strict);
        }
        pre_finalize = selected;
        out.pre_finalize = pre_finalize;
        out.final_sel = prune_selection_to_independent_set(graph, selected, &scores);
        out.score_sum = selection_score_sum(scores, out.final_sel);
        return out;
    }

    if (mode == DfsMode::MergedSinglePass) {
        std::vector<Tvertex> final_sel = selected;
        for (int pass_i = 0; pass_i < dfs_passes; ++pass_i) {
            RefineSelectionOptions loose = base_refine_options(
                cfg, true, node_areas, dfs_refine_seed(refine_seed, pass_i), dg);
            selected = refine_selection(graph_sorted_rev, selected, scores, loose);
            pre_finalize = selected;
            final_sel = dfs_finalize_selection(graph, selected, scores, finalize_opts);
        }
        out.pre_finalize = pre_finalize;
        out.final_sel = final_sel;
        out.score_sum = selection_score_sum(scores, final_sel);
        return out;
    }

    if (mode == DfsMode::MergedLooseFinalizeEnd) {
        for (int pass_i = 0; pass_i < dfs_passes; ++pass_i) {
            RefineSelectionOptions loose = base_refine_options(
                cfg, true, node_areas, dfs_refine_seed(refine_seed, pass_i), dg);
            selected = refine_selection(graph_sorted_rev, selected, scores, loose);
        }
        pre_finalize = selected;
        out.pre_finalize = pre_finalize;
        out.final_sel = dfs_finalize_selection(graph, selected, scores, finalize_opts);
        out.score_sum = selection_score_sum(scores, out.final_sel);
        return out;
    }

    if (mode == DfsMode::MergedLooseTightFinalizeEnd
        || mode == DfsMode::HighPassLoose) {
        for (int pass_i = 0; pass_i < dfs_passes; ++pass_i) {
            RefineSelectionOptions loose = base_refine_options(
                cfg, true, node_areas, dfs_refine_seed(refine_seed, pass_i), dg);
            RefineSelectionOptions tight = base_refine_options(
                cfg, false, node_areas, dfs_refine_seed(refine_seed, pass_i + 1), dg);
            selected = refine_selection(graph_sorted_rev, selected, scores, loose);
            selected = refine_selection(graph, selected, scores, tight);
        }
        pre_finalize = selected;
        out.pre_finalize = pre_finalize;
        out.final_sel = dfs_finalize_selection(graph, selected, scores, finalize_opts);
        out.score_sum = selection_score_sum(scores, out.final_sel);
        return out;
    }

    // merged_loose_tight (default): finalize after each outer pass + growth
    for (int pass_i = 0; pass_i < dfs_passes; ++pass_i) {
        RefineSelectionOptions loose = base_refine_options(
            cfg, true, node_areas, dfs_refine_seed(refine_seed, pass_i), dg);
        RefineSelectionOptions tight = base_refine_options(
            cfg, false, node_areas, dfs_refine_seed(refine_seed, pass_i + 1), dg);
        selected = refine_selection(graph_sorted_rev, selected, scores, loose);
        selected = refine_selection(graph, selected, scores, tight);
    }
    pre_finalize = selected;
    std::vector<Tvertex> final_sel = dfs_finalize_selection(
        graph, selected, scores, finalize_opts);
    std::vector<Tvertex> grown = increase_selection_dfs(
        graph_sorted_rev, final_sel, dfs_max_tries);
    grown = increase_selection_dfs(graph, grown, dfs_max_tries);
    if (grown.size() > final_sel.size()) {
        final_sel = dfs_finalize_selection(graph, grown, scores, finalize_opts);
    }
    out.pre_finalize = pre_finalize;
    out.final_sel = final_sel;
    out.score_sum = selection_score_sum(scores, final_sel);
    return out;
}
