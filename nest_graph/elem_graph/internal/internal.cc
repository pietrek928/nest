#include "internal/internal.h"

#include <cmath>

bool group_has_elems(
    const std::vector<std::vector<Tvertex>> &elems_by_group, Tvertex group
) {
    return group >= 0
        && group < static_cast<Tvertex>(elems_by_group.size())
        && !elems_by_group[group].empty();
}

float sum_selected_scores(
    const Tscore *scores, const unsigned char *selected, int n
) {
    float total = 0.0f;
    for (int i = 0; i < n; ++i) {
        if (selected[i]) {
            total += scores[i];
        }
    }
    return total;
}

void recompute_selected_collisions(
    const std::vector<std::vector<Tvertex>> &collisions,
    const unsigned char *selected,
    int *selected_collisions,
    int n
) {
    for (int i = 0; i < n; ++i) {
        selected_collisions[i] = 0;
        for (Tvertex v : collisions[i]) {
            if (v >= 0 && v < n && selected[v]) {
                selected_collisions[i]++;
            }
        }
    }
}

float sum_selected_scores_vec(
    const std::vector<Tscore> &elem_scores, const std::vector<Tvertex> &selected
) {
    float sum = 0.0f;
    for (Tvertex v : selected) {
        if (v >= 0 && v < static_cast<Tvertex>(elem_scores.size())) {
            sum += elem_scores[v];
        }
    }
    return sum;
}
