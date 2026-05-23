#include "scoring/scoring.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include "graph/graph_index.h"
#include "internal/internal.h"

float circle_gap_sq(const Circle2f &a, const Circle2f &b) {
    const float d = std::sqrt((a.c - b.c).len_sq());
    const float gap =
        std::max(0.0f, d - std::sqrt(a.r_sq) - std::sqrt(b.r_sq));
    return gap * gap;
}

Tscore compute_score(const PointPlaceRule &p, const Vec2f &v) {
    const float qdist = p.pos.qdist(v) + 1.0f;
    return p.w * std::exp(-qdist / p.r);
}

Tscore compute_score(const CirclePlaceRule &p, const Circle2f &circle) {
    const float qdist = circle_gap_sq(p.circle, circle) + 1.0f;
    return p.w * std::exp(-qdist / p.r);
}

Tscore compute_score(const PointAngleRule &p, const Vec2f &v, float a) {
    const float adist = normalized_angle_delta(p.a, a);
    const float qdist = p.pos.qdist(v) + 1.0f;
    return p.w * std::exp(-qdist / p.r) * std::exp(-adist / p.r);
}

Tscore compute_score(const CircleAngleRule &p, const Circle2f &circle, float a) {
    const float adist = normalized_angle_delta(p.a, a);
    const float qdist = circle_gap_sq(p.circle, circle) + 1.0f;
    return p.w * std::exp(-qdist / p.r) * std::exp(-adist / p.r);
}

void compute_scores(
    const ElemGraph &g,
    const std::vector<std::vector<Tvertex>> &elems_by_group,
    std::vector<Tscore> &scores_out,
    const PlacementRuleSet &rules,
    ScoreAggregation aggregation
) {
    const int n = static_cast<int>(g.size());
    scores_out.resize(n);
    std::fill(scores_out.begin(), scores_out.end(), 0.0f);

    for (const PointPlaceRule &p : rules.point_rules) {
        if (!group_has_elems(elems_by_group, p.group)) {
            continue;
        }
        for (Tvertex elem : elems_by_group[p.group]) {
            accumulate_score(
                scores_out[elem], compute_score(p, g.elems[elem].pos), aggregation);
        }
    }
    for (const CirclePlaceRule &p : rules.circle_rules) {
        if (!group_has_elems(elems_by_group, p.group)) {
            continue;
        }
        for (Tvertex elem : elems_by_group[p.group]) {
            accumulate_score(
                scores_out[elem], compute_score(p, g.coords[elem]), aggregation);
        }
    }
    for (const PointAngleRule &p : rules.point_angle_rules) {
        if (!group_has_elems(elems_by_group, p.group)) {
            continue;
        }
        for (Tvertex elem : elems_by_group[p.group]) {
            accumulate_score(
                scores_out[elem],
                compute_score(p, g.elems[elem].pos, g.elems[elem].a),
                aggregation);
        }
    }
    for (const CircleAngleRule &p : rules.circle_angle_rules) {
        if (!group_has_elems(elems_by_group, p.group)) {
            continue;
        }
        for (Tvertex elem : elems_by_group[p.group]) {
            accumulate_score(
                scores_out[elem],
                compute_score(p, g.coords[elem], g.elems[elem].a),
                aggregation);
        }
    }
}

std::vector<Tscore> score_elems(
    const ElemGraph &g,
    const PlacementRuleSet &rules,
    ScoreAggregation aggregation
) {
    std::vector<Tscore> scores;
    const auto elems_by_group = get_elems_by_group(g);
    compute_scores(g, elems_by_group, scores, rules, aggregation);
    return scores;
}
