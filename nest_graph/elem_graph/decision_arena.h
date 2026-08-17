#pragma once

#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <tuple>
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

    /** C0: AMAF ledger key = (region, rule_id, motif_id). */
    void amaf_record(int32_t region, int32_t rule_id, int32_t motif_id, float reward, bool miss) {
        const auto key = std::make_tuple(region, rule_id, motif_id);
        AmafEntry &e = amaf_[key];
        e.visits += 1;
        e.total_reward += reward;
        if (miss) {
            e.misses += 1;
        }
    }

    float amaf_mean(int32_t region, int32_t rule_id, int32_t motif_id) const {
        const auto key = std::make_tuple(region, rule_id, motif_id);
        auto it = amaf_.find(key);
        if (it == amaf_.end() || it->second.visits <= 0) {
            return 0.f;
        }
        return it->second.total_reward / static_cast<float>(it->second.visits);
    }

    int32_t amaf_visits(int32_t region, int32_t rule_id, int32_t motif_id) const {
        const auto key = std::make_tuple(region, rule_id, motif_id);
        auto it = amaf_.find(key);
        if (it == amaf_.end()) {
            return 0;
        }
        return it->second.visits;
    }

    /** UCB1 + optional AMAF blend (C0). Returns -1e30 if visits==0 and not unvisited-inf. */
    float ucb_score(int32_t node_id, int32_t parent_visits, float ucb_c = 1.4f) const {
        const DecisionNode &n = node(node_id);
        if (n.visits <= 0) {
            return std::numeric_limits<float>::infinity();
        }
        float mean = n.total_reward / static_cast<float>(n.visits);
        const float explore = ucb_c * std::sqrt(
            std::log(static_cast<float>(std::max(parent_visits, 1)) + 1.f)
            / static_cast<float>(n.visits)
        );
        const int32_t region = static_cast<int32_t>(n.action.region);
        const int32_t av = amaf_visits(region, n.action.rule_id, n.action.motif_id);
        if (av > 0) {
            const float beta = static_cast<float>(av)
                / (static_cast<float>(av) + static_cast<float>(n.visits) + 1e-9f);
            mean = beta * amaf_mean(region, n.action.rule_id, n.action.motif_id)
                + (1.f - beta) * mean;
        }
        return mean + explore;
    }

private:
    struct AmafEntry {
        int32_t visits = 0;
        float total_reward = 0.f;
        int32_t misses = 0;
    };

    std::vector<DecisionNode> arena_;
    std::map<std::tuple<int32_t, int32_t, int32_t>, AmafEntry> amaf_;
};
