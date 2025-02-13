#include "elem_graph.h"

#include <cmath>
#include <algorithm>


float bbox_qdist(const BBox &a, const BBox &b) {
    float dx = std::max(0.0f, std::max(a.xstart - b.xend, b.xstart - a.xend));
    float dy = std::max(0.0f, std::max(a.ystart - b.yend, b.ystart - a.yend));
    return dx * dx + dy * dy;
}

float bbox_qdist(const BBox &a, float x, float y) {
    float dx = std::max(0.0f, std::max(a.xstart - x, x - a.xend));
    float dy = std::max(0.0f, std::max(a.ystart - y, y - a.yend));
    return dx * dx + dy * dy;
}

Tscore compute_score(const PointPlaceRule &p, float x, float y) {
    float qdist = (p.x - x) * (p.x - x) + (p.y - y) * (p.y - y) + 1.0;
    return p.w * std::exp(-qdist / p.r);
}

Tscore compute_score(const BBoxPlaceRule &p, const BBox &bbox) {
    float qdist = bbox_qdist(p.bbox, bbox) + 1.0;
    return p.w * std::exp(-qdist / p.r);
}

std::vector<std::vector<Tvertex>> get_elems_by_group(const ElemGraph &g) {
    std::vector<std::vector<Tvertex>> elems_by_group;
    for (Tvertex i = 0; i < g.group_id.size(); i++) {
        Tvertex group = g.group_id[i];
        if (group >= elems_by_group.size()) {
            elems_by_group.resize(group + 1);
        }
        elems_by_group[group].push_back(i);
    }
    return elems_by_group;
}

void compute_scores(
    const ElemGraph &g, const std::vector<std::vector<Tvertex>> &elems_by_group,
    std::vector<Tscore> &scores_out, const PlacementRuleSet rules
) {
    int n = g.size();
    scores_out.resize(n);
    std::fill(scores_out.begin(), scores_out.end(), 0.0);

    for (const PointPlaceRule &p : rules.point_rules) {
        for (Tvertex elem : elems_by_group[p.group]) {
            scores_out[elem] += compute_score(p, g.centers[elem].x, g.centers[elem].y);
        }
    }
    for (const BBoxPlaceRule &p : rules.bbox_rules) {
        for (Tvertex elem : elems_by_group[p.group]) {
            scores_out[elem] += compute_score(p, g.coords[elem]);
        }
    }
}

void select_elems(
    const ElemGraph &g, const std::vector<std::vector<Tvertex>> &elems_by_group,
    const PlacementRuleSet rules, const std::vector<Tscore> scores, std::vector<bool> &marked,
    std::vector<Tvertex> &selected, std::vector<Tvertex> &points_sort_buf
) {
    marked.resize(g.size());
    std::fill(marked.begin(), marked.end(), false);
    selected.resize(0);

    // sort points by score
    points_sort_buf.resize(g.size());
    for (Tvertex i = 0; i < g.size(); i++) {
        points_sort_buf[i] = i;
    }
    std::sort(points_sort_buf.begin(), points_sort_buf.end(),
        [&scores](Tvertex a, Tvertex b) {return scores[a] > scores[b];}
    );

    for (Tvertex i : points_sort_buf) {
        if (marked[i]) {
            continue;
        }
        marked[i] = true;
        selected.push_back(i);

        for (Tvertex j : g.collisions[i]) {
            marked[j] = true;
        }
    }
}

std::vector<std::vector<Tvertex>> nest_by_graph(
    const ElemGraph &g, const std::vector<PlacementRuleSet> &cases
) {
    std::vector<Tscore> scores;
    std::vector<bool> marked;
    std::vector<Tvertex> selected;
    std::vector<Tvertex> points_sort_buf;
    auto elems_by_group = get_elems_by_group(g);

    std::vector<std::vector<Tvertex>> result;
    for (const PlacementRuleSet &rules : cases) {
        compute_scores(g, elems_by_group, scores, rules);
        select_elems(g, elems_by_group, rules, scores, marked, selected, points_sort_buf);

        result.push_back(selected);
    }
    return result;
}


void select_node(Tvertex node, const ElemGraph &g, bool *selected, int *selected_collisions) {
    selected[node] = true;
    for (auto &v : g.collisions[node]) {
        selected_collisions[v] ++;
    }
}

void unselect_node(Tvertex node, const ElemGraph &g, bool *selected, int *selected_collisions) {
    selected[node] = false;
    for (auto &v : g.collisions[node]) {
        selected_collisions[v] --;
    }
}

// node MUST NOT be selected
bool increase_path_dfs(
    Tvertex node, const ElemGraph &g,
    bool *tried_select, bool *selected, int *selected_collisions
) {
    tried_select[node] = true;
    select_node(node, g, selected, selected_collisions);
    if (!selected_collisions[node]) {
        return true;
    }

    for (auto &v : g.collisions[node]) {
        if (selected[v]) {
            unselect_node(v, g, selected, selected_collisions);
            bool path_found = false;
            for (auto &v2 : g.collisions[node]) {
                if (!tried_select[v2] && !selected[v2] && selected_collisions[v2] <= 1) {
                    if (increase_path_dfs(v2, g, tried_select, selected, selected_collisions)) {
                        path_found = true;
                        break;
                    }
                }
            }
            if (!path_found) {
                select_node(v, g, selected, selected_collisions);
                unselect_node(node, g, selected, selected_collisions);
                return false;
            }
        }
    }

    return true;
}
