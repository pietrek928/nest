#pragma once

#include <algorithm>
#include <cstdint>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "decision/action_gen.h"
#include "decision_graph.h"

inline std::tuple<int32_t, int32_t, int32_t, int32_t> macro_action_key(
    const MacroAction &action
) {
    int32_t motif_id = action.motif_id;
    if (motif_id < 0) {
        motif_id = -1;
    }
    return {
        static_cast<int32_t>(action.region),
        action.rule_id,
        motif_id,
        action.preset_id,
    };
}

inline std::vector<int32_t> decision_arena_ancestors(const DecisionArena &arena, int32_t node_id) {
    std::vector<int32_t> path;
    int32_t cur = node_id;
    while (cur >= 0) {
        path.push_back(cur);
        if (cur == 0) {
            break;
        }
        cur = arena.node(cur).parent_id;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

struct MacroPathRealized {
    std::unordered_map<int32_t, int32_t> survive_by_motif;
    int32_t survive_motif_n = 0;
    /** Same four bins as MctsRealized.kind (Rim/Void/Sheet/Motif). */
    int32_t kind[4] = {0, 0, 0, 0};
};

/** Soft prior under H3 path AMAF — uncapped kind counts swamp Motif siblings. */
constexpr int32_t kSurviveRankKindCap = 3;

inline float survive_rank_motif_id(int32_t motif_id, const MacroPathRealized &realized) {
    float base = 0.f;
    if (motif_id >= 0) {
        const auto it = realized.survive_by_motif.find(motif_id);
        if (it != realized.survive_by_motif.end()) {
            base = static_cast<float>(it->second);
        }
    }
    base += 0.01f * static_cast<float>(realized.survive_motif_n);
    return base;
}

inline float survive_rank(const MacroAction &action, const MacroPathRealized &realized) {
    if (action.motif_id >= 0) {
        return survive_rank_motif_id(action.motif_id, realized);
    }
    const int32_t ri = static_cast<int32_t>(action.region);
    float base = 0.f;
    if (ri >= 0 && ri < 4) {
        base = static_cast<float>(
            std::min(realized.kind[static_cast<std::size_t>(ri)], kSurviveRankKindCap)
        );
    }
    base += 0.01f * static_cast<float>(realized.survive_motif_n);
    return base;
}

/** L2: MotifRecord edge/flush fuel for survive-count tie-break (never primary). */
inline float motif_join_edge_fuel(const MotifBase *mb, int32_t mid) {
    if (mb == nullptr || mid < 0 || mid >= mb->size()) {
        return 0.f;
    }
    const MotifRecord &rec = mb->at(mid);
    int32_t ea = rec.edge_a;
    int32_t eb = rec.edge_b;
    if (ea < 0 && eb < 0) {
        wang_edge_pair_from_relative(rec.relative, ea, eb);
    }
    float fuel = 0.f;
    if (ea >= 0 && eb >= 0) {
        fuel += 1.f;
    }
    if (rec.flush) {
        fuel += 1.f;
    }
    return fuel;
}

inline std::vector<PathNode> rank_motif_join_neighbors(
    const DecisionGraph &dg,
    int32_t macro_node_id,
    const std::unordered_map<int32_t, int32_t> &survive_by_motif,
    const MotifBase *motif_base = nullptr,
    int32_t *wang_rank_flip_n = nullptr) {
    const PathNode step = node_macro(macro_node_id);
    std::vector<PathNode> join_steps;
    for (const PathNode &nbr : dg.neighbors(step)) {
        if (nbr.kind == PathKind::MotifJoin || nbr.kind == PathKind::Attach) {
            join_steps.push_back(nbr);
        }
    }
    const auto survive_count = [&survive_by_motif](int32_t mid) -> int32_t {
        if (mid < 0) {
            return 0;
        }
        const auto it = survive_by_motif.find(mid);
        return it == survive_by_motif.end() ? 0 : it->second;
    };
    // Survive-only order for flip telem (L2).
    std::vector<PathNode> survive_order = join_steps;
    std::sort(
        survive_order.begin(),
        survive_order.end(),
        [&survive_count](const PathNode &a, const PathNode &b) {
            const int32_t ca = survive_count(a.motif_id);
            const int32_t cb = survive_count(b.motif_id);
            if (ca != cb) {
                return ca > cb;
            }
            const int32_t ma = a.motif_id >= 0 ? a.motif_id : -1;
            const int32_t mb = b.motif_id >= 0 ? b.motif_id : -1;
            return ma < mb;
        });
    std::sort(
        join_steps.begin(),
        join_steps.end(),
        [&survive_count, motif_base](const PathNode &a, const PathNode &b) {
            const int32_t ca = survive_count(a.motif_id);
            const int32_t cb = survive_count(b.motif_id);
            if (ca != cb) {
                return ca > cb;
            }
            // L2: Wang edge_fuel tie-break before id ascending — never primary.
            const float fa = motif_join_edge_fuel(motif_base, a.motif_id);
            const float fb = motif_join_edge_fuel(motif_base, b.motif_id);
            if (fa != fb) {
                return fa > fb;
            }
            const int32_t ma = a.motif_id >= 0 ? a.motif_id : -1;
            const int32_t mb = b.motif_id >= 0 ? b.motif_id : -1;
            return ma < mb;
        });
    if (wang_rank_flip_n != nullptr) {
        int32_t flips = 0;
        const std::size_t n = std::min(survive_order.size(), join_steps.size());
        for (std::size_t i = 0; i < n; ++i) {
            if (survive_order[i].id != join_steps[i].id
                || survive_order[i].motif_id != join_steps[i].motif_id) {
                flips += 1;
            }
        }
        *wang_rank_flip_n = flips;
    }
    return join_steps;
}

inline std::vector<MacroAction> sibling_macro_actions(
    const std::vector<int32_t> &remaining_gids,
    const std::vector<int32_t> &rule_ids,
    const MotifBase *motif_base,
    bool prefer_motifs,
    const std::vector<int32_t> &warm_motif_ids,
    FreeKind free_kind,
    const std::vector<MotifCohortSpec> &motif_cohorts,
    const MacroAction &blocked_action,
    const std::vector<int32_t> &preset_ids = {},
    float void_fit_r = 0.f) {
    const auto blocked_key = macro_action_key(blocked_action);
    std::vector<MacroAction> actions = generate_macros(
        remaining_gids,
        rule_ids,
        motif_base,
        prefer_motifs,
        warm_motif_ids,
        free_kind,
        motif_cohorts,
        preset_ids,
        void_fit_r);
    std::vector<MacroAction> out;
    out.reserve(actions.size());
    for (const MacroAction &a : actions) {
        if (macro_action_key(a) != blocked_key) {
            out.push_back(a);
        }
    }
    return out;
}
