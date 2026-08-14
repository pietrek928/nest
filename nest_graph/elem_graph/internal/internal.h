#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

#include "graph/graph.h"

inline float normalized_angle_delta(float a, float b) {
    float adist = std::abs(a - b);
    if (adist > 2 * static_cast<float>(M_PI)) {
        adist -= 2 * static_cast<float>(M_PI);
    }
    return std::min(adist, 2 * static_cast<float>(M_PI) - adist) + 1.0f;
}

inline void accumulate_score(Tscore &dst, Tscore add, ScoreAggregation agg) {
    if (agg == ScoreAggregation::Max) {
        dst = std::max(dst, add);
    } else {
        dst += add;
    }
}

inline bool vertex_in_graph(Tvertex v, int n) {
    return v >= 0 && v < n;
}

bool group_has_elems(
    const std::vector<std::vector<Tvertex>> &elems_by_group, Tvertex group);

float sum_selected_scores(
    const Tscore *scores, const unsigned char *selected, int n);

void recompute_selected_collisions(
    const std::vector<std::vector<Tvertex>> &collisions,
    const unsigned char *selected,
    int *selected_collisions,
    int n);

float sum_selected_scores_vec(
    const std::vector<Tscore> &elem_scores, const std::vector<Tvertex> &selected);

inline float attract_to_mask(
    const ElemGraph &g,
    int n,
    Tvertex u,
    const unsigned char *selected,
    Tvertex except = -1
) {
    if (static_cast<int>(g.attract.size()) != n) {
        return 0.0f;
    }
    float sum = 0.0f;
    for (const AttractEdge &e : g.attract[static_cast<std::size_t>(u)]) {
        if (!vertex_in_graph(e.target, n) || e.target == except) {
            continue;
        }
        if (selected[static_cast<std::size_t>(e.target)]) {
            sum += e.w;
        }
    }
    return sum;
}

inline float attract_weight_uv(const ElemGraph &g, int n, Tvertex u, Tvertex v) {
    if (static_cast<int>(g.attract.size()) != n) {
        return 0.0f;
    }
    for (const AttractEdge &e : g.attract[static_cast<std::size_t>(u)]) {
        if (e.target == v) {
            return e.w;
        }
    }
    return 0.0f;
}

inline float selected_attract_pairs(
    const ElemGraph &g, const unsigned char *selected, int n
) {
    if (static_cast<int>(g.attract.size()) != n) {
        return 0.0f;
    }
    float pair = 0.0f;
    for (int i = 0; i < n; ++i) {
        if (!selected[i]) {
            continue;
        }
        for (const AttractEdge &e : g.attract[static_cast<std::size_t>(i)]) {
            if (e.target > i && vertex_in_graph(e.target, n)
                && selected[static_cast<std::size_t>(e.target)]) {
                pair += e.w;
            }
        }
    }
    return pair;
}

inline float sum_selected_objective(
    const ElemGraph &g,
    const Tscore *scores,
    const unsigned char *selected,
    int n
) {
    return sum_selected_scores(scores, selected, n)
        + selected_attract_pairs(g, selected, n);
}
