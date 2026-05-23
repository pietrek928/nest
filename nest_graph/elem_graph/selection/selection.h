#pragma once

#include <vector>

#include "graph/graph.h"

void select_elems(
    const ElemGraph &g,
    const std::vector<std::vector<Tvertex>> &elems_by_group,
    const PlacementRuleSet &rules,
    const std::vector<Tscore> &scores,
    std::vector<bool> &marked,
    std::vector<Tvertex> &selected,
    std::vector<Tvertex> &order_buf,
    const SelectOptions &options);
