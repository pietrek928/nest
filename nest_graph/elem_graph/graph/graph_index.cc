#include "graph/graph_index.h"

#include <algorithm>

#include "scoring/scoring.h"

std::vector<std::vector<Tvertex>> get_elems_by_group(const ElemGraph &g) {
    std::vector<std::vector<Tvertex>> elems_by_group;
    for (Tvertex i = 0; i < static_cast<Tvertex>(g.group_id.size()); i++) {
        Tvertex group = g.group_id[i];
        if (group >= static_cast<Tvertex>(elems_by_group.size())) {
            elems_by_group.resize(group + 1);
        }
        elems_by_group[group].push_back(i);
    }
    return elems_by_group;
}

void sort_collisions(
    std::vector<std::vector<Tvertex>> &collisions, const Tscore *scores, bool reverse
) {
    if (!reverse) {
        for (auto &cs : collisions) {
            std::sort(
                cs.begin(), cs.end(),
                [scores](Tvertex a, Tvertex b) { return scores[a] < scores[b]; });
        }
    } else {
        for (auto &cs : collisions) {
            std::sort(
                cs.begin(), cs.end(),
                [scores](Tvertex a, Tvertex b) { return scores[a] > scores[b]; });
        }
    }
}

ElemGraph sort_graph(const ElemGraph &g, const PlacementRuleSet &rules, bool reverse) {
    ElemGraph r = g;
    std::vector<Tscore> scores;
    const auto elems_by_group = get_elems_by_group(g);

    compute_scores(g, elems_by_group, scores, rules, ScoreAggregation::Sum);
    sort_collisions(r.collisions, scores.data(), reverse);
    return r;
}
