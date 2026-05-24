#pragma once

#include <cassert>
#include <set>
#include <stdexcept>
#include <vector>

#include "graph/graph.h"
#include "rules/rules_util.h"
#include "types/types.h"

inline void append_elem_at(
    ElemGraph &g, int group_id, float x, float y, float r_sq = 0.25f) {
    g.group_id.push_back(group_id);
    g.elems.push_back(ElemPlace{Vec2f({x, y}), 0.0f});
    g.coords.push_back(Circle2f(Vec2f({x, y}), r_sq));
    g.collisions.emplace_back();
}

inline void add_collision_pair(ElemGraph &g, int i, int j) {
    g.collisions[static_cast<std::size_t>(i)].push_back(j);
    g.collisions[static_cast<std::size_t>(j)].push_back(i);
}

inline ElemGraph star_graph(int spokes) {
    ElemGraph g;
    append_elem_at(g, 0, 0.0f, 0.0f);
    for (int i = 1; i <= spokes; ++i) {
        append_elem_at(g, 0, static_cast<float>(i), 1.0f);
        add_collision_pair(g, 0, i);
    }
    return g;
}

inline ElemGraph path_graph(int n, const std::vector<float> &scores) {
    if (static_cast<int>(scores.size()) != n) {
        throw std::invalid_argument("path_graph: scores length must match n");
    }
    ElemGraph g;
    for (int i = 0; i < n; ++i) {
        append_elem_at(g, 0, static_cast<float>(i), 0.0f);
    }
    for (int i = 0; i < n - 1; ++i) {
        add_collision_pair(g, i, i + 1);
    }
    (void)scores;
    return g;
}

inline bool is_independent_set(const ElemGraph &g, const std::vector<int> &selected) {
    std::set<int> sel(selected.begin(), selected.end());
    const int n = static_cast<int>(g.size());
    for (int v : sel) {
        if (v < 0 || v >= n) {
            return false;
        }
        for (int u : g.collisions[static_cast<std::size_t>(v)]) {
            if (u != v && sel.count(u) != 0) {
                return false;
            }
        }
    }
    return true;
}

inline PlacementRuleSet point_rule_at(float x, float y, float r = 0.5f, float w = 1.0f) {
    PlacementRuleSet rules;
    append_point_place_rule_at(rules, Vec2f({x, y}), r, w, 0);
    return rules;
}
