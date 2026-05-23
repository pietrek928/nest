#include <nanobind/nanobind.h>
#include <nanobind/stl/vector.h>

#include "bindings.h"
#include "graph/graph.h"
#include "rules/rules.h"

void bind_elem_graph_api(nb::module_ &m) {
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
        [](const ElemGraph &g,
           const std::vector<PlacementRuleSet> &cases,
           const SelectOptions &select) {
            return ::nest_by_graph(g, cases, select);
        },
        nb::arg("g"),
        nb::arg("cases"),
        nb::arg("select") = SelectOptions{});

    m.def(
        "sort_graph",
        &sort_graph,
        nb::arg("g"),
        nb::arg("rules"),
        nb::arg("reverse") = false);

    m.def(
        "score_rules",
        [](const std::vector<ElemGraph> &graphs,
           const std::vector<PlacementRuleSet> &rules,
           const ScoreRulesOptions &options) {
            return ::score_rules(graphs, rules, options);
        },
        nb::arg("graphs"),
        nb::arg("rules"),
        nb::arg("options") = ScoreRulesOptions{});

    m.def(
        "score_elems",
        [](const ElemGraph &g, const PlacementRuleSet &rules, ScoreAggregation agg) {
            return ::score_elems(g, rules, agg);
        },
        nb::arg("g"),
        nb::arg("rules"),
        nb::arg("aggregation") = ScoreAggregation::Sum);

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
        nb::arg("max_tries"),
        nb::arg("min_collisions"));

    m.def(
        "increase_score_dfs",
        [](const ElemGraph &g,
           const std::vector<Tvertex> &selected,
           const std::vector<Tscore> &scores,
           const RefineSelectionOptions &options) {
            return ::increase_score_dfs(g, selected, scores, options);
        },
        nb::arg("g"),
        nb::arg("selected_nodes"),
        nb::arg("scores"),
        nb::arg("options") = RefineSelectionOptions{});
}
