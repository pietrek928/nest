#include "elem_graph.h"

#include <cmath>
#include <algorithm>
#include <random>
#include <stdexcept>
#include <vector>


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

void sort_collisions(std::vector<std::vector<Tvertex>> collisions, Tscore *scores, bool reverse) {
    if (!reverse) {
        for (auto &cs : collisions) {
            std::sort(
                cs.begin(), cs.end(),
                [&scores](Tvertex a, Tvertex b) {return scores[a] < scores[b];}
            );
        }
    } else {
        for (auto &cs : collisions) {
            std::sort(
                cs.begin(), cs.end(),
                [&scores](Tvertex a, Tvertex b) {return scores[a] > scores[b];}
            );
        }
    }
}

ElemGraph sort_graph(const ElemGraph &g, const PlacementRuleSet &rules, bool reverse) {
    ElemGraph r = g;
    std::vector<Tscore> scores;
    auto elems_by_group = get_elems_by_group(g);

    compute_scores(g, elems_by_group, scores, rules);
    sort_collisions(r.collisions, &scores[0], reverse);
    return r;
}

std::vector<Tscore> score_elems(
    const ElemGraph& g, const PlacementRuleSet& rules) {
    std::vector<Tscore> scores;
    auto elems_by_group = get_elems_by_group(g);

    compute_scores(g, elems_by_group, scores, rules);
    return scores;
}

void select_node(Tvertex node, const std::vector<Tvertex> *collisions, unsigned char *selected, int *selected_collisions) {
    if (!selected[node]) {
        selected[node] = true;
        for (auto &v : collisions[node]) {
            selected_collisions[v] ++;
        }
    }
}

void unselect_node(Tvertex node, const std::vector<Tvertex> *collisions, unsigned char *selected, int *selected_collisions) {
    if (selected[node]) {
        selected[node] = false;
        for (auto &v : collisions[node]) {
            selected_collisions[v] --;
        }
    }
}

void mark_collisions(Tvertex node, const std::vector<Tvertex> *collisions, int *mark) {
    for (auto &v: collisions[node]) {
        mark[v] ++;
    }
}

void unmark_collisions(Tvertex node, const std::vector<Tvertex> *collisions, int *mark) {
    for (auto &v: collisions[node]) {
        mark[v] ++;
    }
}

// node MUST NOT be selected
bool increase_path_dfs(
    Tvertex node, const std::vector<Tvertex> *collisions,
    int *mark, unsigned char *selected, int *selected_collisions
) {
    mark[node] ++;
    mark_collisions(node, collisions, mark);
    select_node(node, collisions, selected, selected_collisions);
    if (!selected_collisions[node]) {
        return true;
    }

    for (auto &v : collisions[node]) {
        if (selected[v]) {
            unselect_node(v, collisions, selected, selected_collisions);
            bool path_found = false;
            for (auto &v2 : collisions[v]) {
                if (!mark[v2] && !selected[v2] && selected_collisions[v2] <= 1) {
                    if (increase_path_dfs(v2, collisions, mark, selected, selected_collisions)) {
                        path_found = true;
                        break;
                    }
                }
            }
            if (!path_found) {
                select_node(v, collisions, selected, selected_collisions);
                unmark_collisions(node, collisions, mark);
                unselect_node(node, collisions, selected, selected_collisions);
                return false;
            }
        }
    }

    unmark_collisions(node, collisions, mark);
    return true;
}

std::vector<Tvertex> increase_selection_dfs(
    const ElemGraph &g, const std::vector<Tvertex> &selected_nodes,
    int max_tries, int min_collisions
) {
    std::random_device rd;
    std::mt19937 rand_gen(rd());

    auto n = g.size();
    std::vector<unsigned char> selected(n);
    std::vector<int> mark(n), selected_collisions(n);
    std::vector<Tvertex> nodes(n);

    std::fill(selected.begin(), selected.end(), false);
    for (auto &v : selected_nodes) {
        selected[v] = true;
    }

    for (Tvertex i=0; i<n; i++) {
        nodes[i] = i;
        selected_collisions[i] = 0;
        for (auto &v : g.collisions[i]) {
            if (selected[v]) {
                selected_collisions[i] ++;
            }
        }
    }

    int tries = max_tries;
    do {
        std::shuffle(nodes.begin(), nodes.end(), rand_gen);
        std::fill(mark.begin(), mark.end(), 0);
        for (auto &v : nodes) {
            if (!selected[v] && selected_collisions[v] <= min_collisions) {
                if (increase_path_dfs(v, &g.collisions[0], &mark[0], &selected[0], &selected_collisions[0])) {
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


// node MUST NOT be selected
// must have 1 collision selected
Tscore increase_score_path_dfs(
    Tvertex node, const std::vector<Tvertex> *collisions, const Tscore *scores,
    Tscore score, int *mark, unsigned char *selected, int *selected_collisions
) {
    auto start_score = score;

    mark[node] ++;
    mark_collisions(node, collisions, mark);
    select_node(node, collisions, selected, selected_collisions);
    score += scores[node];
    if (!selected_collisions[node]) {
        if (score > 0) {
            return score;
        } else {
            unselect_node(node, collisions, selected, selected_collisions);
            return start_score;
        }
    }

    for (auto &v : collisions[node]) {
        if (selected[v]) {
            unselect_node(v, collisions, selected, selected_collisions);
            score -= scores[v];
            for (auto &v2 : collisions[v]) {
                if (!mark[v2] && !selected[v2] && selected_collisions[v2] <= 1) {
                    score = increase_score_path_dfs(
                        v2, collisions, scores, score, mark, selected,
                        selected_collisions);
                    if (score > 0) {
                        break;
                    }
                }
            }
            if (!(score > 0)) {
                select_node(v, collisions, selected, selected_collisions);
                unmark_collisions(node, collisions, mark);
                unselect_node(node, collisions, selected, selected_collisions);
                return start_score;
            }
        }
    }

    unmark_collisions(node, collisions, mark);
    return score;
}

std::vector<Tvertex> increase_score_dfs(
    const ElemGraph& g,
    const std::vector<Tvertex>& selected_nodes,
    const std::vector<Tscore>& scores) {
    std::random_device rd;
    std::mt19937 rand_gen(rd());

    if (g.size() != scores.size()) {
        throw std::invalid_argument("Graph size different than scores size");
    }

    auto n = g.size();
    std::vector<unsigned char> selected(n);
    std::vector<int> mark(n), selected_collisions(n);
    std::vector<Tvertex> nodes(n);

    std::fill(selected.begin(), selected.end(), false);
    for (auto& v : selected_nodes) {
        selected[v] = true;
    }

    for (Tvertex i = 0; i < n; i++) {
        nodes[i] = i;
        selected_collisions[i] = 0;
        for (auto& v : g.collisions[i]) {
            if (selected[v]) {
                selected_collisions[i]++;
            }
        }
    }

    bool stop = false;
    while (!stop) {
        stop = true;
        std::sort(nodes.begin(), nodes.end(), [&scores](Tvertex a, Tvertex b) {
            return scores[a] > scores[b];
        });
        std::fill(mark.begin(), mark.end(), 0);
        for (auto& v : nodes) {
            if (!selected[v] && selected_collisions[v] <= 1) {
                if (increase_score_path_dfs(
                        v, &g.collisions[0], &scores[0], 0, &mark[0],
                        &selected[0], &selected_collisions[0]) > 0) {
                    stop = false;
                }
            }
        }

        std::shuffle(nodes.begin(), nodes.end(), rand_gen);
        std::fill(mark.begin(), mark.end(), 0);
        for (auto& v : nodes) {
            if (!selected[v] && selected_collisions[v] <= 1) {
                if (increase_score_path_dfs(
                        v, &g.collisions[0], &scores[0], 0,
                        &mark[0], &selected[0], &selected_collisions[0]) > 0) {
                    stop = false;
                }
            }
        }
    }

    std::vector<Tvertex> r;
    for (Tvertex i = 0; i < n; i++) {
        if (selected[i]) {
            r.push_back(i);
        }
    }
    return r;
}
