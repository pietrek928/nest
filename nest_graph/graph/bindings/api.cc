#include <optional>

#include <nanobind/nanobind.h>
namespace nb = nanobind;
#include <nanobind/stl/optional.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/tuple.h>
#include <nanobind/stl/vector.h>

#include "bindings.h"
#include "decision_graph.h"
#include "pose/pose_graph.h"
#include "refine/apply_dfs.h"
#include "rules/rules.h"
#include "scoring/scoring.h"

void bind_graph_api(nb::module_ &m) {
    nb::enum_<DfsMode>(m, "DfsMode")
        .value("NestOnly", DfsMode::NestOnly)
        .value("HeadPipeline", DfsMode::HeadPipeline)
        .value("StrictNoPrune", DfsMode::StrictNoPrune)
        .value("StrictPrune", DfsMode::StrictPrune)
        .value("LegacyAlternating", DfsMode::LegacyAlternating)
        .value("MergedLooseTight", DfsMode::MergedLooseTight)
        .value("MergedLooseFinalizeEnd", DfsMode::MergedLooseFinalizeEnd)
        .value("MergedLooseTightFinalizeEnd", DfsMode::MergedLooseTightFinalizeEnd)
        .value("MergedSinglePass", DfsMode::MergedSinglePass)
        .value("HighPassLoose", DfsMode::HighPassLoose);

    nb::class_<DfsDispatcherConfig>(m, "DfsDispatcherConfig")
        .def(nb::init<>())
        .def_rw("dfs_max_tries", &DfsDispatcherConfig::dfs_max_tries)
        .def_rw("dfs_refine_max_passes", &DfsDispatcherConfig::dfs_refine_max_passes)
        .def_rw("dfs_refine_max_stagnant_passes", &DfsDispatcherConfig::dfs_refine_max_stagnant_passes)
        .def_rw("dfs_refine_beam_width", &DfsDispatcherConfig::dfs_refine_beam_width)
        .def_rw("refine_explore_shuffle", &DfsDispatcherConfig::refine_explore_shuffle)
        .def_rw("dfs_growth_restarts", &DfsDispatcherConfig::dfs_growth_restarts)
        .def_rw("refine_lexicographic_area", &DfsDispatcherConfig::refine_lexicographic_area)
        .def_rw("dfs_finalize_repair_passes", &DfsDispatcherConfig::dfs_finalize_repair_passes)
        .def_rw("dfs_finalize_max_component", &DfsDispatcherConfig::dfs_finalize_max_component)
        .def_rw("dg_aware_refine", &DfsDispatcherConfig::dg_aware_refine)
        .def_rw("motif_refine_fracture_penalty", &DfsDispatcherConfig::motif_refine_fracture_penalty);

    m.def("dfs_mode_from_string", &dfs_mode_from_string, nb::arg("mode"));

    m.def(
        "prune_selection_to_independent_set",
        [](const PoseGraph &g,
           const std::vector<Tvertex> &selected,
           const std::vector<Tscore> *scores) {
            return prune_selection_to_independent_set(g, selected, scores);
        },
        nb::arg("g"),
        nb::arg("selected"),
        nb::arg("scores") = nullptr);

    m.def(
        "apply_dfs_refinement",
        [](const PoseGraph &graph,
           const PlacementRuleSet &rule_set,
           std::vector<Tvertex> selected,
           const std::vector<Tscore> &scores,
           int dfs_passes,
           int dfs_max_tries,
           DfsMode mode,
           const DfsDispatcherConfig &cfg,
           const FinalizeSelectionOptions &finalize_opts,
           std::optional<std::vector<float>> node_areas,
           int refine_seed,
           const DecisionGraph *dg) {
            const std::vector<float> *areas_ptr =
                node_areas.has_value() ? &node_areas.value() : nullptr;
            const ApplyDfsResult result = apply_dfs_refinement(
                graph,
                rule_set,
                std::move(selected),
                scores,
                dfs_passes,
                dfs_max_tries,
                mode,
                cfg,
                finalize_opts,
                areas_ptr,
                refine_seed,
                dg);
            return std::make_tuple(
                result.pre_finalize, result.final_sel, result.score_sum);
        },
        nb::arg("graph"),
        nb::arg("rule_set"),
        nb::arg("selected"),
        nb::arg("scores"),
        nb::arg("dfs_passes"),
        nb::arg("dfs_max_tries"),
        nb::arg("mode"),
        nb::arg("cfg"),
        nb::arg("finalize_opts"),
        nb::arg("node_areas") = nullptr,
        nb::arg("refine_seed") = -1,
        nb::arg("dg") = nullptr);
    m.def(
        "augment_rules",
        [](const std::vector<PlacementRuleSet> &rules,
           const RuleMutationSettings &settings,
           std::uint32_t seed) {
            return ::augment_rules(rules, settings, seed);
        },
        nb::arg("rules"),
        nb::arg("settings"),
        nb::arg("seed") = 0);

    m.def(
        "nest_by_graph",
        [](const PoseGraph &g,
           const std::vector<PlacementRuleSet> &cases,
           const SelectOptions &select) {
            return ::nest_by_graph(g, cases, select);
        },
        nb::arg("g"),
        nb::arg("cases"),
        nb::arg("select") = SelectOptions{});

    m.def(
        "nest_by_scores",
        [](const PoseGraph &g,
           const std::vector<Tscore> &scores,
           const SelectOptions &select) {
            return ::nest_by_scores(g, scores, select);
        },
        nb::arg("g"),
        nb::arg("scores"),
        nb::arg("select") = SelectOptions{});

    m.def(
        "nest_by_scores",
        [](const DecisionGraph &dg,
           const std::vector<Tscore> &scores,
           const SelectOptions &select) {
            return ::nest_by_scores(dg, scores, select);
        },
        nb::arg("dg"),
        nb::arg("scores"),
        nb::arg("select") = SelectOptions{});

    m.def(
        "sort_graph",
        &sort_graph,
        nb::arg("g"),
        nb::arg("rules"),
        nb::arg("reverse") = false);

    m.def(
        "score_rules",
        [](const std::vector<PoseGraph> &graphs,
           const std::vector<PlacementRuleSet> &rules,
           const ScoreRulesOptions &options) {
            return ::score_rules(graphs, rules, options);
        },
        nb::arg("graphs"),
        nb::arg("rules"),
        nb::arg("options") = ScoreRulesOptions{});

    m.def(
        "score_elems",
        [](const PoseGraph &g, const PlacementRuleSet &rules, ScoreAggregation agg) {
            return ::score_elems(g, rules, agg);
        },
        nb::arg("g"),
        nb::arg("rules"),
        nb::arg("aggregation") = ScoreAggregation::Sum);

    m.def(
        "score_transform",
        [](const PlacementRuleSet &rules,
           Tvertex group,
           float x,
           float y,
           float angle,
           ScoreAggregation agg,
           float radius) {
            return ::score_transform(
                rules, group, Vec2f({x, y}), angle, agg, radius);
        },
        nb::arg("rules"),
        nb::arg("group"),
        nb::arg("x"),
        nb::arg("y"),
        nb::arg("angle"),
        nb::arg("aggregation") = ScoreAggregation::Sum,
        nb::arg("radius") = 0.5f);

    m.def(
        "refine_selection_dfs",
        &refine_selection_dfs,
        nb::arg("g"),
        nb::arg("selected_nodes"),
        nb::arg("scores"),
        nb::arg("options") = RefineSelectionOptions{});

    m.def(
        "increase_selection_dfs",
        &increase_selection_dfs,
        nb::arg("g"),
        nb::arg("selected_nodes"),
        nb::arg("max_tries"));

    m.def(
        "increase_score_dfs",
        [](const PoseGraph &g,
           const std::vector<Tvertex> &selected,
           const std::vector<Tscore> &scores,
           const RefineSelectionOptions &options) {
            return ::increase_score_dfs(g, selected, scores, options);
        },
        nb::arg("g"),
        nb::arg("selected_nodes"),
        nb::arg("scores"),
        nb::arg("options") = RefineSelectionOptions{});

    m.def(
        "refine_selection",
        &refine_selection,
        nb::arg("g"),
        nb::arg("selected_nodes"),
        nb::arg("scores"),
        nb::arg("options") = RefineSelectionOptions{});

    m.def(
        "finalize_selection",
        [](const PoseGraph &g,
           const std::vector<Tvertex> &selected,
           const std::vector<Tscore> &scores,
           const FinalizeSelectionOptions &options) {
            return ::finalize_selection(g, selected, scores, options, nullptr);
        },
        nb::arg("g"),
        nb::arg("selected_nodes"),
        nb::arg("scores"),
        nb::arg("options") = FinalizeSelectionOptions{});

    m.def(
        "greedy_weighted_mis",
        [](const std::vector<Tvertex> &verts,
           const std::vector<Tscore> &scores,
           const PoseGraph &g,
           const std::vector<Tvertex> &locked) {
            return ::greedy_weighted_mis(verts, scores, g, locked);
        },
        nb::arg("verts"),
        nb::arg("scores"),
        nb::arg("g"),
        nb::arg("locked_indices") = std::vector<Tvertex>{});

    m.def(
        "selection_is_independent",
        &selection_is_independent,
        nb::arg("g"),
        nb::arg("selected_nodes"));
}
