#include "graph/graph.h"

#include <algorithm>
#include <random>
#include <stdexcept>
#include <vector>

#include "internal/internal.h"

constexpr int kDefaultMinCollisionsLoose = 2;
constexpr int kDefaultMinCollisionsTight = 1;

bool selection_independent(
    const ElemGraph &g, const std::vector<unsigned char> &selected
) {
    const int n = static_cast<int>(g.size());
    for (int v = 0; v < n; ++v) {
        if (!selected[static_cast<std::size_t>(v)]) {
            continue;
        }
        for (Tvertex u : g.collisions[static_cast<std::size_t>(v)]) {
            if (u > v && vertex_in_graph(u, n) && selected[static_cast<std::size_t>(u)]) {
                return false;
            }
        }
    }
    return true;
}

void update_best_independent(
    const ElemGraph &g,
    const std::vector<unsigned char> &selected,
    const std::vector<Tscore> &scores,
    std::vector<unsigned char> &best_selected,
    float &best_sum,
    int &best_count
) {
    if (!selection_independent(g, selected)) {
        return;
    }
    const int n = static_cast<int>(g.size());
    const float sum = sum_selected_scores(scores.data(), selected.data(), n);
    int count = 0;
    for (int i = 0; i < n; ++i) {
        if (selected[static_cast<std::size_t>(i)]) {
            count++;
        }
    }
    if (count > best_count || (count == best_count && sum > best_sum + 1e-6f)) {
        best_selected = selected;
        best_sum = sum;
        best_count = count;
    }
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

void run_growth_pass(
    const ElemGraph &g,
    const std::vector<Tscore> &scores,
    int max_tries,
    int min_collisions,
    std::vector<unsigned char> &selected,
    std::vector<int> &selected_collisions,
    std::vector<unsigned char> &best_selected,
    float &best_sum,
    int &best_count
) {
    if (max_tries <= 0) {
        return;
    }
    const int n = static_cast<int>(g.size());
    std::vector<int> mark(n, 0);
    std::vector<Tvertex> nodes(n);
    for (int i = 0; i < n; ++i) {
        nodes[i] = i;
    }

    std::mt19937 rng(std::random_device{}());
    int tries = max_tries;
    do {
        std::shuffle(nodes.begin(), nodes.end(), rng);
        std::fill(mark.begin(), mark.end(), 0);
        for (Tvertex v : nodes) {
            if (selected[static_cast<std::size_t>(v)]) {
                continue;
            }
            if (selected_collisions[v] > min_collisions) {
                continue;
            }
            if (increase_path_dfs(
                    v, &g.collisions[0], mark.data(), selected.data(),
                    selected_collisions.data(), n)) {
                tries = max_tries;
            }
        }
        update_best_independent(g, selected, scores, best_selected, best_sum, best_count);
    } while (--tries);
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
        bool independent = true;
        for (int v = 0; v < n && independent; ++v) {
            if (!selected[v]) {
                continue;
            }
            for (Tvertex u : collisions[v]) {
                if (u > v && vertex_in_graph(u, n) && selected[u]) {
                    independent = false;
                    break;
                }
            }
        }
        if (independent && path_delta > best_delta) {
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

    std::vector<unsigned char> best_independent(n, 0);
    float best_sum = -1e30f;
    int best_count = -1;
    update_best_independent(
        g, selected, scores, best_independent, best_sum, best_count);

    run_growth_pass(
        g, scores, options.max_tries, options.min_collisions, selected,
        selected_collisions, best_independent, best_sum, best_count);

    float selected_sum =
        sum_selected_scores(scores.data(), selected.data(), n);

    std::mt19937 rng(
        options.seed != 0 ? options.seed : std::random_device{}());

    const int max_root_coll = std::max(0, options.max_root_collisions);

    for (int i = 0; i < n; ++i) {
        nodes[i] = i;
    }

    const int stagnant_limit = std::max(1, options.max_stagnant_passes);
    int stagnant_passes = 0;
    for (int pass = 0; pass < options.max_passes; ++pass) {
        bool pass_improved = false;

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
                pass_improved = true;
                update_best_independent(
                    g, selected, scores, best_independent, best_sum, best_count);
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
                    pass_improved = true;
                    update_best_independent(
                        g, selected, scores, best_independent, best_sum, best_count);
                }
                std::fill(mark.begin(), mark.end(), 0);
            }
        }

        if (pass_improved) {
            stagnant_passes = 0;
        } else {
            stagnant_passes++;
            if (stagnant_passes >= stagnant_limit) {
                break;
            }
        }
    }

    if (!selection_independent(g, selected) && best_count >= 0) {
        selected = best_independent;
    } else {
        update_best_independent(
            g, selected, scores, best_independent, best_sum, best_count);
        if (best_count >= 0) {
            selected = best_independent;
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

std::vector<Tvertex> refine_selection(
    const ElemGraph &g,
    const std::vector<Tvertex> &selected_nodes,
    const std::vector<Tscore> &scores,
    const RefineSelectionOptions &options
) {
    return refine_selection_dfs(g, selected_nodes, scores, options);
}

std::vector<Tvertex> increase_selection_dfs(
    const ElemGraph &g,
    const std::vector<Tvertex> &selected_nodes,
    int max_tries
) {
    RefineSelectionOptions opts;
    opts.max_tries = max_tries;
    opts.min_collisions = kDefaultMinCollisionsLoose;
    opts.max_passes = 0;
    opts.max_root_collisions = kDefaultMinCollisionsLoose;
    return refine_selection_dfs(g, selected_nodes, std::vector<Tscore>(g.size(), 1.0f), opts);
}

std::vector<Tvertex> increase_score_dfs(
    const ElemGraph &g,
    const std::vector<Tvertex> &selected_nodes,
    const std::vector<Tscore> &scores,
    const RefineSelectionOptions &options
) {
    return refine_selection_dfs(g, selected_nodes, scores, options);
}
