#pragma once

#include <vector>

#include "graph/graph.h"

float circle_gap_sq(const Circle2f &a, const Circle2f &b);

Tscore compute_score(const PointPlaceRule &p, const Vec2f &v);
Tscore compute_score(const CirclePlaceRule &p, const Circle2f &circle);
Tscore compute_score(const PointAngleRule &p, const Vec2f &v, float a);
Tscore compute_score(const CircleAngleRule &p, const Circle2f &circle, float a);

void compute_scores(
    const ElemGraph &g,
    const std::vector<std::vector<Tvertex>> &elems_by_group,
    std::vector<Tscore> &scores_out,
    const PlacementRuleSet &rules,
    ScoreAggregation aggregation);

std::vector<Tscore> score_elems(
    const ElemGraph &g,
    const PlacementRuleSet &rules,
    ScoreAggregation aggregation);
