#include "graph/graph.h"

#include <algorithm>
#include <random>
#include <stdexcept>
#include <vector>

#include "internal/internal.h"

namespace {

bool vertex_in_graph(Tvertex v, int n) {
    return v >= 0 && v < n;
}

void select_node(
    Tvertex node,
    const std::vector<Tvertex> *collisions,
    unsigned char *selected,
    int *selected_collisions,
    int n
) {
    if (!selected[node]) {
        selected[node] = 1;
        for (Tvertex v : collisions[node]) {
            if (vertex_in_graph(v, n)) {
                selected_collisions[v]++;
            }
        }
    }
}

void unselect_node(
    Tvertex node,
    const std::vector<Tvertex> *collisions,
    unsigned char *selected,
    int *selected_collisions,
    int n
) {
    if (selected[node]) {
        selected[node] = 0;
        for (Tvertex v : collisions[node]) {
            if (vertex_in_graph(v, n)) {
                selected_collisions[v]--;
            }
        }
    }
}

void mark_collisions(
    Tvertex node, const std::vector<Tvertex> *collisions, int *mark, int n
) {
    for (Tvertex v : collisions[node]) {
        if (vertex_in_graph(v, n)) {
            mark[v]++;
        }
    }
}

void unmark_collisions(
    Tvertex node, const std::vector<Tvertex> *collisions, int *mark, int n
) {
    for (Tvertex v : collisions[node]) {
        if (vertex_in_graph(v, n)) {
            mark[v]--;
        }
    }
}

bool increase_path_dfs(
    Tvertex node,
    const std::vector<Tvertex> *collisions,
    int *mark,
    unsigned char *selected,
    int *selected_collisions,
    int n
) {
    mark[node]++;
    mark_collisions(node, collisions, mark, n);
    select_node(node, collisions, selected, selected_collisions, n);
    if (!selected_collisions[node]) {
        return true;
    }

    for (Tvertex v : collisions[node]) {
        if (!vertex_in_graph(v, n) || !selected[v]) {
            continue;
        }
        unselect_node(v, collisions, selected, selected_collisions, n);
        bool path_found = false;
        for (Tvertex v2 : collisions[v]) {
            if (!vertex_in_graph(v2, n)) {
                continue;
            }
            if (!mark[v2] && !selected[v2] && selected_collisions[v2] <= 1) {
                if (increase_path_dfs(
                        v2, collisions, mark, selected, selected_collisions, n)) {
                    path_found = true;
                    break;
                }
            }
        }
        if (!path_found) {
            select_node(v, collisions, selected, selected_collisions, n);
            unmark_collisions(node, collisions, mark, n);
            unselect_node(node, collisions, selected, selected_collisions, n);
            return false;
        }
    }

    unmark_collisions(node, collisions, mark, n);
    return true;
}

float score_path_dfs(
    Tvertex node,
    const std::vector<Tvertex> *collisions,
    const Tscore *scores,
    float path_delta,
    int depth,
    int max_depth,
    int beam_width,
    float &best_delta,
    std::vector<unsigned char> &best_selected,
    unsigned char *selected,
    int *mark,
    int *selected_collisions,
    int n
) {
    if (depth > max_depth || mark[node]) {
        return best_delta;
    }

    mark[node]++;
    mark_collisions(node, collisions, mark, n);
    select_node(node, collisions, selected, selected_collisions, n);
    path_delta += scores[node];

    if (!selected_collisions[node]) {
        if (path_delta > best_delta) {
            best_delta = path_delta;
            best_selected.assign(selected, selected + n);
        }
        unmark_collisions(node, collisions, mark, n);
        unselect_node(node, collisions, selected, selected_collisions, n);
        mark[node]--;
        return best_delta;
    }

    std::vector<Tvertex> conflicts;
    for (Tvertex v : collisions[node]) {
        if (vertex_in_graph(v, n) && selected[v]) {
            conflicts.push_back(v);
        }
    }
    std::sort(
        conflicts.begin(), conflicts.end(),
        [scores](Tvertex a, Tvertex b) { return scores[a] < scores[b]; });

    for (Tvertex v : conflicts) {
        unselect_node(v, collisions, selected, selected_collisions, n);
        const float delta_after_remove = path_delta - scores[v];

        std::vector<Tvertex> candidates;
        for (Tvertex v2 : collisions[v]) {
            if (!vertex_in_graph(v2, n)) {
                continue;
            }
            if (!mark[v2] && !selected[v2] && selected_collisions[v2] <= 1) {
                candidates.push_back(v2);
            }
        }
        std::sort(
            candidates.begin(), candidates.end(),
            [scores](Tvertex a, Tvertex b) { return scores[a] > scores[b]; });
        const int limit = std::min(
            beam_width, static_cast<int>(candidates.size()));
        for (int ci = 0; ci < limit; ++ci) {
            score_path_dfs(
                candidates[static_cast<std::size_t>(ci)], collisions, scores,
                delta_after_remove, depth + 1, max_depth, beam_width, best_delta,
                best_selected, selected, mark, selected_collisions, n);
        }

        select_node(v, collisions, selected, selected_collisions, n);
        path_delta = delta_after_remove + scores[v];
    }

    unmark_collisions(node, collisions, mark, n);
    unselect_node(node, collisions, selected, selected_collisions, n);
    mark[node]--;
    return best_delta;
}

bool try_refine_root(
    Tvertex v,
    const ElemGraph &g,
    const std::vector<Tscore> &scores,
    const RefineSelectionOptions &options,
    float baseline_sum,
    std::vector<unsigned char> &selected,
    std::vector<unsigned char> &backup,
    std::vector<unsigned char> &best_selected,
    std::vector<int> &mark,
    std::vector<int> &selected_collisions,
    float &selected_sum
) {
    const int n = static_cast<int>(g.size());
    const int beam = std::max(1, options.beam_width);

    backup = selected;
    float best_delta = -1e30f;
    score_path_dfs(
        v, &g.collisions[0], scores.data(), 0.0f, 0, options.max_depth, beam,
        best_delta, best_selected, selected.data(), mark.data(),
        selected_collisions.data(), n);

    const float new_sum = sum_selected_scores(scores.data(), best_selected.data(), n);
    if (new_sum > baseline_sum + options.min_score_delta) {
        selected = best_selected;
        recompute_selected_collisions(
            g.collisions, selected.data(), selected_collisions.data(), n);
        selected_sum = new_sum;
        return true;
    }

    selected = backup;
    recompute_selected_collisions(
        g.collisions, selected.data(), selected_collisions.data(), n);
    selected_sum = baseline_sum;
    return false;
}

}  // namespace

std::vector<Tvertex> refine_selection_dfs(
    const ElemGraph &g,
    const std::vector<Tvertex> &selected_nodes,
    const std::vector<Tscore> &scores,
    const RefineSelectionOptions &options
) {
    if (g.size() != scores.size()) {
        throw std::invalid_argument("Graph size different than scores size");
    }

    const int n = static_cast<int>(g.size());
    std::vector<unsigned char> selected(n, 0);
    std::vector<unsigned char> backup(n, 0);
    std::vector<unsigned char> best_selected(n, 0);
    std::vector<int> mark(n, 0);
    std::vector<int> selected_collisions(n, 0);
    std::vector<Tvertex> nodes(n);

    for (Tvertex v : selected_nodes) {
        if (vertex_in_graph(v, n)) {
            selected[v] = 1;
        }
    }
    recompute_selected_collisions(
        g.collisions, selected.data(), selected_collisions.data(), n);

    float selected_sum =
        sum_selected_scores(scores.data(), selected.data(), n);

    std::mt19937 rng(
        options.seed != 0 ? options.seed : std::random_device{}());

    const int max_root_coll = std::max(1, options.max_root_collisions);

    for (int i = 0; i < n; ++i) {
        nodes[i] = i;
    }

    int passes = 0;
    bool improved = true;
    while (improved && passes < options.max_passes) {
        improved = false;
        ++passes;

        std::sort(
            nodes.begin(), nodes.end(),
            [&](Tvertex a, Tvertex b) {
                const float da = 1.0f + static_cast<float>(selected_collisions[a]);
                const float db = 1.0f + static_cast<float>(selected_collisions[b]);
                return (scores[a] / da) > (scores[b] / db);
            });

        std::fill(mark.begin(), mark.end(), 0);
        for (Tvertex v : nodes) {
            if (selected[v] || selected_collisions[v] > max_root_coll) {
                continue;
            }
            const float baseline_sum = selected_sum;
            if (try_refine_root(
                    v, g, scores, options, baseline_sum, selected, backup,
                    best_selected, mark, selected_collisions, selected_sum)) {
                improved = true;
            }
            std::fill(mark.begin(), mark.end(), 0);
        }

        if (options.explore_shuffle) {
            std::shuffle(nodes.begin(), nodes.end(), rng);
            std::fill(mark.begin(), mark.end(), 0);
            for (Tvertex v : nodes) {
                if (selected[v] || selected_collisions[v] > max_root_coll) {
                    continue;
                }
                const float baseline_sum = selected_sum;
                if (try_refine_root(
                        v, g, scores, options, baseline_sum, selected, backup,
                        best_selected, mark, selected_collisions, selected_sum)) {
                    improved = true;
                }
                std::fill(mark.begin(), mark.end(), 0);
            }
        }
    }

    std::vector<Tvertex> result;
    for (int i = 0; i < n; ++i) {
        if (selected[i]) {
            result.push_back(i);
        }
    }
    return result;
}

std::vector<Tvertex> increase_selection_dfs(
    const ElemGraph &g,
    const std::vector<Tvertex> &selected_nodes,
    int max_tries,
    int min_collisions
) {
    std::random_device rd;
    std::mt19937 rand_gen(rd());

    const auto n = static_cast<int>(g.size());
    std::vector<unsigned char> selected(n);
    std::vector<int> mark(n), selected_collisions(n);
    std::vector<Tvertex> nodes(n);

    std::fill(selected.begin(), selected.end(), 0);
    for (Tvertex v : selected_nodes) {
        if (vertex_in_graph(v, n)) {
            selected[v] = 1;
        }
    }

    for (Tvertex i = 0; i < n; i++) {
        nodes[i] = i;
        selected_collisions[i] = 0;
        for (Tvertex v : g.collisions[i]) {
            if (vertex_in_graph(v, n) && selected[v]) {
                selected_collisions[i]++;
            }
        }
    }

    int tries = max_tries;
    do {
        std::shuffle(nodes.begin(), nodes.end(), rand_gen);
        std::fill(mark.begin(), mark.end(), 0);
        for (Tvertex v : nodes) {
            if (!selected[v] && selected_collisions[v] <= min_collisions) {
                if (increase_path_dfs(
                        v, &g.collisions[0], mark.data(), selected.data(),
                        selected_collisions.data(), n)) {
                    tries = max_tries;
                }
            }
        }
    } while (--tries);

    std::vector<Tvertex> r;
    for (Tvertex i = 0; i < n; i++) {
        if (selected[i]) {
            r.push_back(i);
        }
    }
    return r;
}

std::vector<Tvertex> increase_score_dfs(
    const ElemGraph &g,
    const std::vector<Tvertex> &selected_nodes,
    const std::vector<Tscore> &scores,
    const RefineSelectionOptions &options
) {
    return refine_selection_dfs(g, selected_nodes, scores, options);
}
