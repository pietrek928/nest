#pragma once

#include <vector>

#include "graph/graph.h"

std::vector<std::vector<Tvertex>> get_elems_by_group(const PoseGraph &g);

void sort_collisions(
    std::vector<std::vector<Tvertex>> &collisions,
    const Tscore *scores,
    bool reverse);

PoseGraph sort_graph(
    const PoseGraph &g, const PlacementRuleSet &rules, bool reverse);
