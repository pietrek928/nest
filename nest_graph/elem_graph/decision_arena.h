#pragma once

#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include "se2.h"

/** Macro action that produced a DecisionArena child (policy edge, not a pose). */

enum class MacroRegion : int32_t {
    Rim = 0,
    Void = 1,
    Sheet = 2,
    Motif = 3,
};

struct MacroAction {
    int32_t part_gid = -1;
    MacroRegion region = MacroRegion::Sheet;
    int32_t rule_id = -1;
    int32_t motif_id = -1;
};

struct DecisionNode {
    int32_t parent_id = -1;
    int32_t first_child_id = -1;
    int32_t next_sibling_id = -1;
    int32_t visits = 0;
    float total_reward = 0.f;
    MacroAction action{};
};

class DecisionArena {
public:
    DecisionArena() {
        arena_.reserve(1024);
        arena_.push_back(DecisionNode{});  // root id 0
    }

    int32_t root_id() const { return 0; }

    int32_t add_node(int32_t parent_id, const MacroAction &action) {
        const int32_t new_id = static_cast<int32_t>(arena_.size());
        DecisionNode node;
        node.parent_id = parent_id;
        node.action = action;
        arena_.push_back(node);
        if (parent_id >= 0 && parent_id < static_cast<int32_t>(arena_.size() - 1)) {
            int32_t &child = arena_[static_cast<std::size_t>(parent_id)].first_child_id;
            if (child < 0) {
                child = new_id;
            } else {
                int32_t curr = child;
                while (arena_[static_cast<std::size_t>(curr)].next_sibling_id >= 0) {
                    curr = arena_[static_cast<std::size_t>(curr)].next_sibling_id;
                }
                arena_[static_cast<std::size_t>(curr)].next_sibling_id = new_id;
            }
        }
        return new_id;
    }

    DecisionNode &node(int32_t id) { return arena_.at(static_cast<std::size_t>(id)); }
    const DecisionNode &node(int32_t id) const { return arena_.at(static_cast<std::size_t>(id)); }

    void record_visit(int32_t id, float reward) {
        DecisionNode &n = node(id);
        n.visits += 1;
        n.total_reward += reward;
    }

    int32_t size() const { return static_cast<int32_t>(arena_.size()); }

    /** Progressive widening: allow new child when children < c * visits^α (Q80). */
    bool may_expand(int32_t id, float c = 1.5f, float alpha = 0.5f) const {
        const DecisionNode &n = node(id);
        int children = 0;
        for (int32_t ch = n.first_child_id; ch >= 0; ch = arena_[static_cast<std::size_t>(ch)].next_sibling_id) {
            ++children;
        }
        const float visits = static_cast<float>(std::max(n.visits, 1));
        const float cap = c * std::pow(visits, alpha);
        return static_cast<float>(children) < cap;
    }

private:
    std::vector<DecisionNode> arena_;
};
