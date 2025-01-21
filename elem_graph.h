#pragma once

#include <vector>

using Tvertex = int;

typedef struct ElemGroup {
    int count_limit;
    float priority;
} ElemGroup;

typedef struct ElemGraph {
    std::vector<Tvertex> group_id;
    std::vector<std::pair<float, float>> coords;
    std::vector<std::vector<Tvertex>> collisions;
} ElemGraph;
