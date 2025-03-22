#pragma once

#include <vector>

#include "rules.h"


typedef struct ElemPlace {
    float x, y, a;
} ElemPlace;

typedef struct ElemGroup {
    int count_limit;
    float priority;
} ElemGroup;

typedef struct ElemGraph {
    std::vector<Tvertex> group_id;
    std::vector<ElemPlace> elems;
    std::vector<BBox> coords;
    std::vector<std::vector<Tvertex>> collisions;

    auto size() const { return group_id.size(); }
} ElemGraph;


std::vector<std::vector<Tvertex>> nest_by_graph(
    const ElemGraph &g, const std::vector<PlacementRuleSet> &cases
);

ElemGraph sort_graph(const ElemGraph &g, const PlacementRuleSet &rules, bool reverse);

std::vector<Tscore> score_rules(
    const std::vector<ElemGraph> &graphs,
    const std::vector<PlacementRuleSet> &rule_sets
);

std::vector<Tvertex> increase_selection_dfs(
    const ElemGraph& g,
    const std::vector<Tvertex>& selected_nodes,
    int max_tries,
    int min_collisions);
std::vector<Tvertex> increase_score_dfs(
    const ElemGraph& g,
    const std::vector<Tvertex>& selected_nodes,
    const std::vector<Tscore>& scores);
