#include "selection/selection.h"

#include <algorithm>

#include "graph/graph_index.h"
#include "scoring/scoring.h"

void sort_collision_lists_by_score(
    const ElemGraph &g,
    const std::vector<Tscore> &scores,
    std::vector<std::vector<Tvertex>> &collisions
) {
    collisions = g.collisions;
    sort_collisions(collisions, scores.data(), false);
}

void select_elems_greedy(
    const std::vector<std::vector<Tvertex>> &collisions,
    int n,
    const std::vector<Tscore> &scores,
    std::vector<bool> &marked,
    std::vector<Tvertex> &selected,
    std::vector<Tvertex> &order_buf,
    SelectMode mode
) {
    marked.resize(n);
    std::fill(marked.begin(), marked.end(), false);
    selected.clear();

    order_buf.resize(n);
    for (Tvertex i = 0; i < static_cast<Tvertex>(n); i++) {
        order_buf[i] = i;
    }

    if (mode == SelectMode::WeightedGreedy) {
        std::sort(
            order_buf.begin(), order_buf.end(),
            [&](Tvertex a, Tvertex b) {
                const float da = 1.0f + static_cast<float>(collisions[a].size());
                const float db = 1.0f + static_cast<float>(collisions[b].size());
                return (scores[a] / da) > (scores[b] / db);
            });
    } else {
        std::sort(
            order_buf.begin(), order_buf.end(),
            [&](Tvertex a, Tvertex b) { return scores[a] > scores[b]; });
    }

    for (Tvertex i : order_buf) {
        if (marked[i]) {
            continue;
        }
        marked[i] = true;
        selected.push_back(i);
        for (Tvertex j : collisions[i]) {
            if (j >= 0 && j < static_cast<Tvertex>(n)) {
                marked[j] = true;
            }
        }
    }
}

void select_elems_local_swap(
    const std::vector<std::vector<Tvertex>> &collisions,
    int n,
    const std::vector<Tscore> &scores,
    std::vector<bool> &marked,
    std::vector<Tvertex> &selected
) {
    std::vector<bool> is_selected(n, false);
    for (Tvertex v : selected) {
        if (v >= 0 && v < static_cast<Tvertex>(n)) {
            is_selected[v] = true;
        }
    }

    bool improved = true;
    while (improved) {
        improved = false;
        for (Tvertex u = 0; u < static_cast<Tvertex>(n); ++u) {
            if (is_selected[u] || marked[u]) {
                continue;
            }
            Tvertex blocker = -1;
            int n_blockers = 0;
            for (Tvertex j : collisions[u]) {
                if (j >= 0 && j < static_cast<Tvertex>(n) && is_selected[j]) {
                    blocker = j;
                    n_blockers++;
                }
            }
            if (n_blockers != 1 || blocker < 0) {
                continue;
            }
            if (scores[u] <= scores[blocker]) {
                continue;
            }
            bool independent = true;
            for (Tvertex j = 0; j < static_cast<Tvertex>(n); ++j) {
                if (!is_selected[j] || j == blocker) {
                    continue;
                }
                for (Tvertex c : collisions[u]) {
                    if (c == j) {
                        independent = false;
                        break;
                    }
                }
                if (!independent) {
                    break;
                }
            }
            if (!independent) {
                continue;
            }
            is_selected[blocker] = false;
            marked[blocker] = false;
            for (Tvertex c : collisions[blocker]) {
                if (c >= 0 && c < static_cast<Tvertex>(n) && c != u) {
                    marked[c] = false;
                }
            }
            is_selected[u] = true;
            marked[u] = true;
            for (Tvertex c : collisions[u]) {
                if (c >= 0 && c < static_cast<Tvertex>(n)) {
                    marked[c] = true;
                }
            }
            auto it = std::find(selected.begin(), selected.end(), blocker);
            if (it != selected.end()) {
                *it = u;
            }
            improved = true;
        }
    }
}

bool nodes_independent(
    const std::vector<std::vector<Tvertex>> &collisions,
    int n,
    Tvertex a,
    Tvertex b
) {
    if (a == b) {
        return false;
    }
    for (Tvertex c : collisions[a]) {
        if (c == b) {
            return false;
        }
    }
    (void)n;
    return true;
}

bool node_independent_of_set(
    const std::vector<std::vector<Tvertex>> &collisions,
    int n,
    Tvertex u,
    const std::vector<bool> &is_selected
) {
    for (Tvertex j = 0; j < static_cast<Tvertex>(n); ++j) {
        if (!is_selected[j]) {
            continue;
        }
        for (Tvertex c : collisions[u]) {
            if (c == j) {
                return false;
            }
        }
    }
    return true;
}

void select_elems_two_swap(
    const std::vector<std::vector<Tvertex>> &collisions,
    int n,
    const std::vector<Tscore> &scores,
    std::vector<bool> &marked,
    std::vector<Tvertex> &selected,
    int max_tries
) {
    std::vector<bool> is_selected(n, false);
    for (Tvertex v : selected) {
        if (v >= 0 && v < static_cast<Tvertex>(n)) {
            is_selected[v] = true;
        }
    }

    int tries = 0;
    bool improved = true;
    while (improved && tries < max_tries) {
        improved = false;
        ++tries;

        std::vector<Tvertex> sel_copy = selected;
        std::sort(
            sel_copy.begin(), sel_copy.end(),
            [&](Tvertex a, Tvertex b) { return scores[a] < scores[b]; });

        for (Tvertex blocker : sel_copy) {
            std::vector<Tvertex> candidates;
            for (Tvertex u = 0; u < static_cast<Tvertex>(n); ++u) {
                if (is_selected[u] || marked[u]) {
                    continue;
                }
                bool blocks = false;
                for (Tvertex c : collisions[u]) {
                    if (c == blocker) {
                        blocks = true;
                        break;
                    }
                }
                if (!blocks) {
                    continue;
                }
                if (!node_independent_of_set(collisions, n, u, is_selected)) {
                    continue;
                }
                candidates.push_back(u);
            }
            std::sort(
                candidates.begin(), candidates.end(),
                [&](Tvertex a, Tvertex b) { return scores[a] > scores[b]; });

            for (std::size_t i = 0; i < candidates.size(); ++i) {
                for (std::size_t j = i + 1; j < candidates.size(); ++j) {
                    const Tvertex u = candidates[i];
                    const Tvertex v = candidates[j];
                    if (!nodes_independent(collisions, n, u, v)) {
                        continue;
                    }
                    const float gain =
                        scores[u] + scores[v] - scores[blocker];
                    if (gain <= 0.0f) {
                        continue;
                    }

                    is_selected[blocker] = false;
                    marked[blocker] = false;
                    for (Tvertex c : collisions[blocker]) {
                        if (c >= 0 && c < static_cast<Tvertex>(n)) {
                            marked[c] = false;
                        }
                    }
                    auto it = std::find(selected.begin(), selected.end(), blocker);
                    if (it != selected.end()) {
                        selected.erase(it);
                    }

                    is_selected[u] = true;
                    is_selected[v] = true;
                    marked[u] = true;
                    marked[v] = true;
                    for (Tvertex c : collisions[u]) {
                        if (c >= 0 && c < static_cast<Tvertex>(n)) {
                            marked[c] = true;
                        }
                    }
                    for (Tvertex c : collisions[v]) {
                        if (c >= 0 && c < static_cast<Tvertex>(n)) {
                            marked[c] = true;
                        }
                    }
                    selected.push_back(u);
                    selected.push_back(v);
                    improved = true;
                    break;
                }
                if (improved) {
                    break;
                }
            }
            if (improved) {
                break;
            }
        }
    }
}

void select_elems(
    const ElemGraph &g,
    const std::vector<std::vector<Tvertex>> &elems_by_group,
    const PlacementRuleSet &rules,
    const std::vector<Tscore> &scores,
    std::vector<bool> &marked,
    std::vector<Tvertex> &selected,
    std::vector<Tvertex> &order_buf,
    const SelectOptions &options
) {
    (void)elems_by_group;
    (void)rules;

    std::vector<std::vector<Tvertex>> sorted_collisions;
    sort_collision_lists_by_score(g, scores, sorted_collisions);
    const int n = static_cast<int>(g.size());

    select_elems_greedy(
        sorted_collisions, n, scores, marked, selected, order_buf, options.mode);
    if (options.local_swap) {
        select_elems_local_swap(sorted_collisions, n, scores, marked, selected);
        select_elems_two_swap(sorted_collisions, n, scores, marked, selected, 64);
    }
}

std::vector<std::vector<Tvertex>> nest_by_graph(
    const ElemGraph &g,
    const std::vector<PlacementRuleSet> &cases,
    const SelectOptions &select
) {
    std::vector<Tscore> scores;
    std::vector<bool> marked;
    std::vector<Tvertex> selected;
    std::vector<Tvertex> order_buf;
    const auto elems_by_group = get_elems_by_group(g);

    std::vector<std::vector<Tvertex>> result;
    for (const PlacementRuleSet &rules : cases) {
        compute_scores(g, elems_by_group, scores, rules, select.aggregation);
        select_elems(
            g, elems_by_group, rules, scores, marked, selected, order_buf, select);
        result.push_back(selected);
    }
    return result;
}
