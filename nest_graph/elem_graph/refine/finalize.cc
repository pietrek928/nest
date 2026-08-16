#include "refine/finalize.h"

#include <algorithm>
#include <cstdint>
#include <cmath>
#include <unordered_set>
#include <vector>

#include "internal/internal.h"

bool mask_is_independent(int mask, const std::vector<int> &adj) {
    for (std::size_t i = 0; i < adj.size(); ++i) {
        if (!(mask & (1 << static_cast<int>(i)))) {
            continue;
        }
        if (mask & adj[i]) {
            return false;
        }
    }
    return true;
}

float mask_weight(const std::vector<Tscore> &w, int mask) {
    float sum = 0.0f;
    for (std::size_t i = 0; i < w.size(); ++i) {
        if (mask & (1 << static_cast<int>(i))) {
            sum += w[i];
        }
    }
    return sum;
}

float mask_objective(
    const PoseGraph &g,
    const std::vector<Tvertex> &verts,
    const std::vector<Tscore> &w,
    int mask,
    const unsigned char *kept_outside
) {
    float tw = mask_weight(w, mask);
    const int n = static_cast<int>(g.size());
    const int k = static_cast<int>(verts.size());
    for (int i = 0; i < k; ++i) {
        if (!(mask & (1 << i))) {
            continue;
        }
        const Tvertex vi = verts[static_cast<std::size_t>(i)];
        if (kept_outside != nullptr) {
            tw += attract_to_mask(g, n, vi, kept_outside);
        }
        for (int j = i + 1; j < k; ++j) {
            if (!(mask & (1 << j))) {
                continue;
            }
            tw += attract_weight_uv(
                g, n, vi, verts[static_cast<std::size_t>(j)]);
        }
    }
    return tw;
}

std::vector<int> brute_force_weighted_mis(
    const std::vector<Tvertex> &verts,
    const std::vector<Tscore> &scores,
    const PoseGraph &g,
    const std::vector<Tvertex> &locked_indices,
    const unsigned char *kept_outside = nullptr
) {
    const int k = static_cast<int>(verts.size());
    std::vector<int> local_idx(static_cast<std::size_t>(k));
    std::vector<Tscore> w(static_cast<std::size_t>(k));
    for (int i = 0; i < k; ++i) {
        local_idx[static_cast<std::size_t>(i)] = verts[static_cast<std::size_t>(i)];
        w[static_cast<std::size_t>(i)] = scores[verts[static_cast<std::size_t>(i)]];
    }

    std::vector<int> adj(static_cast<std::size_t>(k), 0);
    for (int i = 0; i < k; ++i) {
        for (int j = i + 1; j < k; ++j) {
            const Tvertex a = verts[static_cast<std::size_t>(i)];
            const Tvertex b = verts[static_cast<std::size_t>(j)];
            bool edge = false;
            for (Tvertex u : g.collisions[static_cast<std::size_t>(a)]) {
                if (u == b) {
                    edge = true;
                    break;
                }
            }
            if (edge) {
                adj[static_cast<std::size_t>(i)] |= 1 << j;
                adj[static_cast<std::size_t>(j)] |= 1 << i;
            }
        }
    }

    int forced_mask = 0;
    std::unordered_set<Tvertex> in_comp(verts.begin(), verts.end());
    for (Tvertex vi : locked_indices) {
        if (!in_comp.count(vi)) {
            continue;
        }
        int li = -1;
        for (int i = 0; i < k; ++i) {
            if (verts[static_cast<std::size_t>(i)] == vi) {
                li = i;
                break;
            }
        }
        if (li < 0) {
            continue;
        }
        if (forced_mask & adj[static_cast<std::size_t>(li)]) {
            continue;
        }
        forced_mask |= 1 << li;
    }

    int best_mask = forced_mask;
    float best_w = -1.0f;
    int best_count = -1;
    const int limit = 1 << k;
    for (int mask = 0; mask < limit; ++mask) {
        if ((mask & forced_mask) != forced_mask) {
            continue;
        }
        if (!mask_is_independent(mask, adj)) {
            continue;
        }
        int pop = 0;
        for (int i = 0; i < k; ++i) {
            if (mask & (1 << i)) {
                pop++;
            }
        }
        const float tw = mask_objective(g, verts, w, mask, kept_outside);
        if (pop > best_count || (pop == best_count && tw > best_w)) {
            best_count = pop;
            best_w = tw;
            best_mask = mask;
        }
    }

    std::vector<int> keep;
    for (int i = 0; i < k; ++i) {
        if (best_mask & (1 << i)) {
            keep.push_back(verts[static_cast<std::size_t>(i)]);
        }
    }
    return keep;
}

std::vector<int> greedy_weighted_mis(
    const std::vector<Tvertex> &verts,
    const std::vector<Tscore> &scores,
    const PoseGraph &g,
    const std::vector<Tvertex> &locked_indices,
    const unsigned char *kept_outside
) {
    (void)kept_outside;
    std::unordered_set<Tvertex> in_verts(verts.begin(), verts.end());
    std::unordered_set<Tvertex> kept;
    for (Tvertex vi : locked_indices) {
        if (!in_verts.count(vi) || kept.count(vi)) {
            continue;
        }
        bool ok = true;
        for (Tvertex u : g.collisions[static_cast<std::size_t>(vi)]) {
            if (kept.count(u)) {
                ok = false;
                break;
            }
        }
        if (ok) {
            kept.insert(vi);
        }
    }
    std::vector<Tvertex> order = verts;
    std::sort(
        order.begin(), order.end(),
        [&scores](Tvertex a, Tvertex b) { return scores[a] > scores[b]; });
    for (Tvertex v : order) {
        if (kept.count(v)) {
            continue;
        }
        bool ok = true;
        for (Tvertex u : g.collisions[static_cast<std::size_t>(v)]) {
            if (kept.count(u)) {
                ok = false;
                break;
            }
        }
        if (ok) {
            kept.insert(v);
        }
    }
    std::vector<int> result(kept.begin(), kept.end());
    return result;
}

void build_overlap_components(
    const PoseGraph &g,
    const std::vector<Tvertex> &selected,
    std::vector<std::vector<Tvertex>> &components
) {
    std::unordered_set<Tvertex> sel(selected.begin(), selected.end());
    std::unordered_set<Tvertex> visited;
    for (Tvertex v : selected) {
        if (visited.count(v)) {
            continue;
        }
        std::vector<Tvertex> stack = {v};
        std::vector<Tvertex> comp;
        visited.insert(v);
        while (!stack.empty()) {
            const Tvertex u = stack.back();
            stack.pop_back();
            comp.push_back(u);
            for (Tvertex w : g.collisions[static_cast<std::size_t>(u)]) {
                if (!sel.count(w) || visited.count(w)) {
                    continue;
                }
                visited.insert(w);
                stack.push_back(w);
            }
        }
        if (comp.size() > 1) {
            components.push_back(std::move(comp));
        }
    }
}

std::vector<unsigned char> selected_to_mask(
    const PoseGraph &g, const std::vector<Tvertex> &selected
) {
    const int n = static_cast<int>(g.size());
    std::vector<unsigned char> mask(static_cast<std::size_t>(n), 0);
    for (Tvertex v : selected) {
        if (v >= 0 && v < n) {
            mask[static_cast<std::size_t>(v)] = 1;
        }
    }
    return mask;
}

std::vector<Tvertex> mask_to_list(const std::vector<unsigned char> &mask) {
    std::vector<Tvertex> out;
    for (int i = 0; i < static_cast<int>(mask.size()); ++i) {
        if (mask[static_cast<std::size_t>(i)]) {
            out.push_back(i);
        }
    }
    return out;
}

bool selection_is_independent_mask(
    const PoseGraph &g, const std::vector<unsigned char> &selected
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

bool selection_is_independent(
    const PoseGraph &g, const std::vector<Tvertex> &selected_nodes
) {
    return selection_is_independent_mask(g, selected_to_mask(g, selected_nodes));
}

void insert_clear_locks(
    const PoseGraph &g,
    std::vector<unsigned char> &selected,
    const std::vector<Tvertex> &locked_indices
) {
    const int n = static_cast<int>(g.size());
    for (Tvertex vi : locked_indices) {
        if (!vertex_in_graph(vi, n) || selected[static_cast<std::size_t>(vi)]) {
            continue;
        }
        bool hits = false;
        for (Tvertex u : g.collisions[static_cast<std::size_t>(vi)]) {
            if (vertex_in_graph(u, n) && selected[static_cast<std::size_t>(u)]) {
                hits = true;
                break;
            }
        }
        if (hits) {
            continue;
        }
        selected[static_cast<std::size_t>(vi)] = 1;
    }
}

std::vector<Tvertex> finalize_selection(
    const PoseGraph &g,
    const std::vector<Tvertex> &selected_nodes,
    const std::vector<Tscore> &scores,
    const FinalizeSelectionOptions &options,
    FinalizeSelectionStats *stats
) {
    if (g.size() != scores.size()) {
        throw std::invalid_argument("Graph size different than scores size");
    }

    FinalizeSelectionStats local_stats;
    FinalizeSelectionStats *st = stats ? stats : &local_stats;

    std::vector<unsigned char> selected = selected_to_mask(g, selected_nodes);
    if (selection_is_independent_mask(g, selected)) {
        insert_clear_locks(g, selected, options.locked_indices);
        return mask_to_list(selected);
    }

    RefineSelectionOptions repair_opts;
    repair_opts.max_passes = 1;
    repair_opts.max_depth = 32;
    repair_opts.beam_width = 2;
    repair_opts.max_root_collisions = 0;
    repair_opts.min_score_delta = 0.0f;
    repair_opts.locked_indices = options.locked_indices;

    for (int pass = 0; pass < options.repair_passes; ++pass) {
        if (selection_is_independent_mask(g, selected)) {
            break;
        }
        st->repair_passes_used++;
        const auto repaired = refine_selection_dfs(
            g, mask_to_list(selected), scores, repair_opts);
        selected = selected_to_mask(g, repaired);
    }

    if (selection_is_independent_mask(g, selected)) {
        insert_clear_locks(g, selected, options.locked_indices);
        return mask_to_list(selected);
    }

    const std::vector<Tvertex> before = mask_to_list(selected);
    std::vector<std::vector<Tvertex>> components;
    build_overlap_components(g, before, components);

    std::unordered_set<Tvertex> keep_set;
    const int n = static_cast<int>(g.size());
    for (const auto &comp : components) {
        std::unordered_set<Tvertex> in_comp(comp.begin(), comp.end());
        std::vector<unsigned char> kept_k(static_cast<std::size_t>(n), 0);
        for (Tvertex v : before) {
            if (!in_comp.count(v) && vertex_in_graph(v, n)) {
                kept_k[static_cast<std::size_t>(v)] = 1;
            }
        }
        std::vector<int> kept;
        if (static_cast<int>(comp.size()) <= options.max_exact_component_size) {
            kept = brute_force_weighted_mis(
                comp, scores, g, options.locked_indices, kept_k.data());
            st->optimal_components++;
        } else {
            kept = greedy_weighted_mis(
                comp, scores, g, options.locked_indices, kept_k.data());
            st->greedy_fallback_components++;
        }
        for (Tvertex v : kept) {
            keep_set.insert(v);
        }
    }

    for (Tvertex v : before) {
        bool in_overlap_comp = false;
        for (const auto &comp : components) {
            if (std::find(comp.begin(), comp.end(), v) != comp.end()) {
                in_overlap_comp = true;
                break;
            }
        }
        if (!in_overlap_comp) {
            keep_set.insert(v);
        }
    }

    std::vector<Tvertex> result(keep_set.begin(), keep_set.end());
    std::sort(result.begin(), result.end());
    st->nodes_dropped = static_cast<int>(before.size()) - static_cast<int>(result.size());

    if (!selection_is_independent_mask(g, selected_to_mask(g, result))) {
        result = greedy_weighted_mis(
            before, scores, g, options.locked_indices);
    }

    selected = selected_to_mask(g, result);
    insert_clear_locks(g, selected, options.locked_indices);
    return mask_to_list(selected);
}
