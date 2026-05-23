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
