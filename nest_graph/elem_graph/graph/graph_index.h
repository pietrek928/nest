#pragma once

#include <vector>

#include "graph/graph.h"

std::vector<std::vector<Tvertex>> get_elems_by_group(const ElemGraph &g);

void sort_collisions(
    std::vector<std::vector<Tvertex>> &collisions,
    const Tscore *scores,
    bool reverse);

ElemGraph sort_graph(
    const ElemGraph &g, const PlacementRuleSet &rules, bool reverse);
