#pragma once

#include <circle.h>
#include <vec.h>


using Tvertex = int;
using Tscore = float;
using Vec2f = Vec<2, float>;
using Circle2f = Circle<Vec2f>;

inline Circle2f circle_from_bounds(
    float xmin, float ymin, float xmax, float ymax) {
    const float cx = 0.5f * (xmin + xmax);
    const float cy = 0.5f * (ymin + ymax);
    const float hx = 0.5f * (xmax - xmin);
    const float hy = 0.5f * (ymax - ymin);
    return Circle2f(Vec2f({cx, cy}), hx * hx + hy * hy);
}
