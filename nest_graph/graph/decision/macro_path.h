#pragma once

#include <algorithm>
#include <cstdint>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "decision/action_gen.h"
#include "decision_graph.h"

inline std::tuple<int32_t, int32_t, int32_t> macro_action_key(const MacroAction &action) {
    int32_t motif_id = action.motif_id;
    if (motif_id < 0) {
        motif_id = -1;
    }
    return {static_cast<int32_t>(action.region), action.rule_id, motif_id};
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
};

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
    return survive_rank_motif_id(action.motif_id, realized);
}

inline std::vector<PathNode> rank_motif_join_neighbors(
    const DecisionGraph &dg,
    int32_t macro_node_id,
    const std::unordered_map<int32_t, int32_t> &survive_by_motif) {
    const PathNode step = node_macro(macro_node_id);
    std::vector<PathNode> join_steps;
    for (const PathNode &nbr : dg.neighbors(step)) {
        if (nbr.kind == PathKind::MotifJoin) {
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
    std::sort(
        join_steps.begin(),
        join_steps.end(),
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
    return join_steps;
}

inline std::vector<MacroAction> sibling_macro_actions(
    const std::vector<int32_t> &remaining_gids,
    const std::vector<int32_t> &rule_ids,
    const MotifBase *motif_base,
    bool prefer_motifs,
    const std::vector<int32_t> &warm_motif_ids,
    const std::string &free_kind,
    const std::vector<MotifCohortSpec> &motif_cohorts,
    const MacroAction &blocked_action) {
    const auto blocked_key = macro_action_key(blocked_action);
    std::vector<MacroAction> actions = generate_macros(
        remaining_gids,
        rule_ids,
        motif_base,
        prefer_motifs,
        warm_motif_ids,
        free_kind,
        motif_cohorts);
    std::vector<MacroAction> out;
    out.reserve(actions.size());
    for (const MacroAction &a : actions) {
        if (macro_action_key(a) != blocked_key) {
            out.push_back(a);
        }
    }
    return out;
}
