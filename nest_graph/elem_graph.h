#pragma once

#include <vector>

using Tvertex = int;
using Tscore = float;

typedef struct BBox {
    float xstart, xend, ystart, yend;
} BBox;

typedef struct Point {
    float x, y;
} Point;


typedef struct ElemGroup {
    int count_limit;
    float priority;
} ElemGroup;


typedef struct PointPlaceRule {
    float x, y, r, w;
    Tvertex group;
} PointPlaceRule;


typedef struct BBoxPlaceRule {
    BBox bbox;
    float r, w;
    Tvertex group;
} BBoxPlaceRule;


typedef struct PlacementRuleSet {
    std::vector<PointPlaceRule> point_rules;
    std::vector<BBoxPlaceRule> bbox_rules;
} PlacementRuleSet;


typedef struct ElemGraph {
    std::vector<Tvertex> group_id;
    std::vector<Point> centers;
    std::vector<BBox> coords;
    std::vector<std::vector<Tvertex>> collisions;

    auto size() const { return group_id.size(); }
} ElemGraph;


std::vector<std::vector<Tvertex>> nest_by_graph(
    const ElemGraph &g, const std::vector<PlacementRuleSet> &cases
);

ElemGraph sort_graph(const ElemGraph &g, const PlacementRuleSet &rules, bool reverse);

std::vector<Tvertex> increase_selection_dfs(
    const ElemGraph& g,
    const std::vector<Tvertex>& selected_nodes,
    int max_tries,
    int min_collisions);
std::vector<Tvertex> increase_score_dfs(
    const ElemGraph& g,
    const std::vector<Tvertex>& selected_nodes,
    const std::vector<Tscore>& scores);