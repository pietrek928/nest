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

void mark_element(ElemGraph &graph, Tvertex elem, Tvertex *marked) {
    if (marked[elem]) return;
    marked[elem] = 1;
    for (Tvertex coll : graph.collisions[elem]) {
        marked[coll] = 1;
    }
}

float compute_score(const PointPlaceRule &p, float x, float y) {
    float qdist = (p.x - x) * (p.x - x) + (p.y - y) * (p.y - y) + 1.0;
    return p.w * std::exp(-qdist / p.r);
}

float compute_score(const BBoxPlaceRule &p, const BBox &bbox) {
    float qdist = bbox_qdist(p.bbox, bbox) + 1.0;
    return p.w * std::exp(-qdist / p.r);
}

void compute_scores(
    const BBox *elems, float *scores_out, int n,
    const PointPlaceRule *point_rules, int npoint_rules,
    const BBoxPlaceRule *bbox_rules, int nbbox_rules
) {
    for (int i = 0; i < n; i++) {
        scores_out[i] = 0.0;
        // TOGO: weight center ?
        float xcenter = (elems[i].xstart + elems[i].xend) * 0.5;
        float ycenter = (elems[i].ystart + elems[i].yend) * 0.5;
        for (int j = 0; j < npoint_rules; j++) {
            scores_out[i] += compute_score(point_rules[j], xcenter, ycenter);
        }
        for (int j = 0; j < nbbox_rules; j++) {
            scores_out[i] += compute_score(bbox_rules[j], elems[i]);
        }
    }
}

void select_elems(
    const ElemGraph &g,
    const PointPlaceRule *point_rules, int npoint_rules,
    const BBoxPlaceRule *bbox_rules, int nbbox_rules
) {
    //
}
