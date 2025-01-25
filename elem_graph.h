#pragma once

#include <vector>

using Tvertex = int;

typedef struct BBox {
    float xstart, xend, ystart, yend;
} BBox;


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
    std::vector<BBox> coords;
    std::vector<std::vector<Tvertex>> collisions;
} ElemGraph;
