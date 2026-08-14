#include "selection/selection.h"

#include <algorithm>
#include <stdexcept>

#include "graph/graph_index.h"
#include "internal/internal.h"
#include "scoring/scoring.h"

void sort_collision_lists_by_score(
    const ElemGraph &g,
    const std::vector<Tscore> &scores,
    std::vector<std::vector<Tvertex>> &collisions
) {
    collisions = g.collisions;
    sort_collisions(collisions, scores.data(), false);
}

void rebuild_marked_from_selected(
    const std::vector<std::vector<Tvertex>> &collisions,
    int n,
    const std::vector<bool> &is_selected,
    std::vector<bool> &marked
) {
    marked.assign(static_cast<std::size_t>(n), false);
    for (Tvertex v = 0; v < static_cast<Tvertex>(n); ++v) {
        if (!is_selected[static_cast<std::size_t>(v)]) {
            continue;
        }
        marked[static_cast<std::size_t>(v)] = true;
        for (Tvertex c : collisions[static_cast<std::size_t>(v)]) {
            if (vertex_in_graph(c, n)) {
                marked[static_cast<std::size_t>(c)] = true;
            }
        }
    }
}

bool collides_with(
    const std::vector<std::vector<Tvertex>> &collisions,
    Tvertex u,
    Tvertex v
) {
    if (u == v) {
        return true;
    }
    for (Tvertex c : collisions[static_cast<std::size_t>(u)]) {
        if (c == v) {
            return true;
        }
    }
    return false;
}

bool node_independent_of_selected_except(
    const std::vector<std::vector<Tvertex>> &collisions,
    int n,
    Tvertex u,
    const std::vector<bool> &is_selected,
    Tvertex except
) {
    for (Tvertex c : collisions[static_cast<std::size_t>(u)]) {
        if (!vertex_in_graph(c, n) || c == except) {
            continue;
        }
        if (is_selected[static_cast<std::size_t>(c)]) {
            return false;
        }
    }
    return true;
}

void install_locked_indices(
    const std::vector<std::vector<Tvertex>> &collisions,
    int n,
    const std::vector<Tvertex> &locked_indices,
    std::vector<bool> &marked,
    std::vector<Tvertex> &selected,
    std::vector<bool> &is_locked
) {
    is_locked.assign(static_cast<std::size_t>(n), false);
    for (Tvertex vi : locked_indices) {
        if (!vertex_in_graph(vi, n)) {
            continue;
        }
        if (marked[static_cast<std::size_t>(vi)]) {
            continue;
        }
        selected.push_back(vi);
        is_locked[static_cast<std::size_t>(vi)] = true;
        marked[static_cast<std::size_t>(vi)] = true;
        for (Tvertex j : collisions[static_cast<std::size_t>(vi)]) {
            if (vertex_in_graph(j, n)) {
                marked[static_cast<std::size_t>(j)] = true;
            }
        }
    }
}

void select_elems_greedy(
    const std::vector<std::vector<Tvertex>> &collisions,
    int n,
    const std::vector<Tscore> &scores,
    std::vector<bool> &marked,
    std::vector<Tvertex> &selected,
    std::vector<Tvertex> &order_buf,
    SelectMode mode,
    const std::vector<Tvertex> &locked_indices,
    std::vector<bool> &is_locked
) {
    marked.assign(static_cast<std::size_t>(n), false);
    selected.clear();
    install_locked_indices(
        collisions, n, locked_indices, marked, selected, is_locked);

    order_buf.resize(static_cast<std::size_t>(n));
    for (Tvertex i = 0; i < static_cast<Tvertex>(n); i++) {
        order_buf[static_cast<std::size_t>(i)] = i;
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
        if (marked[static_cast<std::size_t>(i)]) {
            continue;
        }
        marked[static_cast<std::size_t>(i)] = true;
        selected.push_back(i);
        for (Tvertex j : collisions[static_cast<std::size_t>(i)]) {
            if (vertex_in_graph(j, n)) {
                marked[static_cast<std::size_t>(j)] = true;
            }
        }
    }
}

float attract_to_selected(
    const ElemGraph &g,
    int n,
    Tvertex u,
    const std::vector<bool> &is_selected,
    Tvertex except
) {
    if (static_cast<int>(g.attract.size()) != n) {
        return 0.0f;
    }
    float sum = 0.0f;
    for (const AttractEdge &e : g.attract[static_cast<std::size_t>(u)]) {
        if (!vertex_in_graph(e.target, n) || e.target == except) {
            continue;
        }
        if (is_selected[static_cast<std::size_t>(e.target)]) {
            sum += e.w;
        }
    }
    return sum;
}

float attract_pair_weight(const ElemGraph &g, int n, Tvertex u, Tvertex v) {
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

void select_elems_local_swap(
    const ElemGraph &g,
    const std::vector<std::vector<Tvertex>> &collisions,
    int n,
    const std::vector<Tscore> &scores,
    std::vector<bool> &marked,
    std::vector<Tvertex> &selected,
    const std::vector<bool> &is_locked
) {
    std::vector<bool> is_selected(static_cast<std::size_t>(n), false);
    for (Tvertex v : selected) {
        if (vertex_in_graph(v, n)) {
            is_selected[static_cast<std::size_t>(v)] = true;
        }
    }

    bool improved = true;
    while (improved) {
        improved = false;
        for (Tvertex u = 0; u < static_cast<Tvertex>(n); ++u) {
            if (is_selected[static_cast<std::size_t>(u)]) {
                continue;
            }
            Tvertex blocker = -1;
            int n_blockers = 0;
            for (Tvertex j : collisions[static_cast<std::size_t>(u)]) {
                if (vertex_in_graph(j, n) && is_selected[static_cast<std::size_t>(j)]) {
                    blocker = j;
                    n_blockers++;
                    if (n_blockers > 1) {
                        break;
                    }
                }
            }
            if (n_blockers != 1 || blocker < 0) {
                continue;
            }
            if (is_locked[static_cast<std::size_t>(blocker)]) {
                continue;
            }
            const float gain =
                (scores[static_cast<std::size_t>(u)]
                 + attract_to_selected(g, n, u, is_selected, blocker))
                - (scores[static_cast<std::size_t>(blocker)]
                   + attract_to_selected(g, n, blocker, is_selected, blocker));
            if (gain <= 0.0f) {
                continue;
            }
            if (!node_independent_of_selected_except(
                    collisions, n, u, is_selected, blocker)) {
                continue;
            }
            is_selected[static_cast<std::size_t>(blocker)] = false;
            is_selected[static_cast<std::size_t>(u)] = true;
            auto it = std::find(selected.begin(), selected.end(), blocker);
            if (it != selected.end()) {
                *it = u;
            }
            improved = true;
        }
    }
    rebuild_marked_from_selected(collisions, n, is_selected, marked);
}

bool nodes_independent(
    const std::vector<std::vector<Tvertex>> &collisions,
    int n,
    Tvertex a,
    Tvertex b
) {
    (void)n;
    return !collides_with(collisions, a, b);
}

void select_elems_two_swap(
    const ElemGraph &g,
    const std::vector<std::vector<Tvertex>> &collisions,
    int n,
    const std::vector<Tscore> &scores,
    std::vector<bool> &marked,
    std::vector<Tvertex> &selected,
    int max_tries,
    const std::vector<bool> &is_locked
) {
    std::vector<bool> is_selected(static_cast<std::size_t>(n), false);
    for (Tvertex v : selected) {
        if (vertex_in_graph(v, n)) {
            is_selected[static_cast<std::size_t>(v)] = true;
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
            if (vertex_in_graph(blocker, n)
                && is_locked[static_cast<std::size_t>(blocker)]) {
                continue;
            }
            std::vector<Tvertex> candidates;
            for (Tvertex u = 0; u < static_cast<Tvertex>(n); ++u) {
                if (is_selected[static_cast<std::size_t>(u)]) {
                    continue;
                }
                if (!collides_with(collisions, u, blocker)) {
                    continue;
                }
                if (!node_independent_of_selected_except(
                        collisions, n, u, is_selected, blocker)) {
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
                        scores[static_cast<std::size_t>(u)]
                        + scores[static_cast<std::size_t>(v)]
                        - scores[static_cast<std::size_t>(blocker)]
                        + attract_to_selected(g, n, u, is_selected, blocker)
                        + attract_to_selected(g, n, v, is_selected, blocker)
                        + attract_pair_weight(g, n, u, v)
                        - attract_to_selected(g, n, blocker, is_selected, blocker);
                    if (gain <= 0.0f) {
                        continue;
                    }

                    is_selected[static_cast<std::size_t>(blocker)] = false;
                    auto it = std::find(selected.begin(), selected.end(), blocker);
                    if (it != selected.end()) {
                        selected.erase(it);
                    }

                    is_selected[static_cast<std::size_t>(u)] = true;
                    is_selected[static_cast<std::size_t>(v)] = true;
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
    rebuild_marked_from_selected(collisions, n, is_selected, marked);
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
    std::vector<bool> is_locked;

    select_elems_greedy(
        sorted_collisions, n, scores, marked, selected, order_buf, options.mode,
        options.locked_indices, is_locked);
    if (options.local_swap) {
        select_elems_local_swap(
            g, sorted_collisions, n, scores, marked, selected, is_locked);
        select_elems_two_swap(
            g, sorted_collisions, n, scores, marked, selected, 64, is_locked);
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

std::vector<Tvertex> nest_by_scores(
    const ElemGraph &g,
    const std::vector<Tscore> &scores,
    const SelectOptions &select
) {
    if (scores.size() != g.size()) {
        throw std::invalid_argument("nest_by_scores: scores size != graph size");
    }
    std::vector<bool> marked;
    std::vector<Tvertex> selected;
    std::vector<Tvertex> order_buf;
    const auto elems_by_group = get_elems_by_group(g);
    PlacementRuleSet empty_rules;
    select_elems(
        g, elems_by_group, empty_rules, scores, marked, selected, order_buf, select);
    return selected;
}
