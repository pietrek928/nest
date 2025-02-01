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


typedef struct ElemGraph {
    std::vector<Tvertex> group_id;
    std::vector<Point> centers;
    std::vector<BBox> coords;
    std::vector<std::vector<Tvertex>> collisions;

    auto size() const { return group_id.size(); }
} ElemGraph;


typedef struct PlacementRules {
    std::vector<PointPlaceRule> point_rules;
    std::vector<BBoxPlaceRule> bbox_rules;
} PlacementRules;


std::vector<std::vector<Tvertex>> nest_by_graph(
    const ElemGraph &g, const std::vector<PlacementRules> &cases
);
