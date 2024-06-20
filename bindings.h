#pragma once

#include "algo.h"


using Vec2f = Vec<2, float>;
using Vec2d = Vec<2, double>;
using Vec3f = Vec<3, float>;
using Vec3d = Vec<3, double>;

constexpr auto v2f = v<float, float, float>;
constexpr auto v2d = v<double, double, double>;
constexpr auto v3f = v<float, float, float, float>;
constexpr auto v3d = v<double, double, double, double>;

unsigned int g3_size() {
    return Diff2<3, float>::size();
}
using f_g6 = Diff2<6, float>;
using d_g6 = Diff2<6, double>;
unsigned int g6_size() {
    return Diff2<6, float>::size();
}

using Vec2f_g3 = Vec<2, Diff2<3, float>>;
using Vec2d_g3 = Vec<2, Diff2<3, double>>;
unsigned int Vec2_g3_size() {
    return Diff2<3, float>::size() * 2;
}

using Vec2f_g6 = Vec<2, Diff2<6, float>>;
using Vec2d_g6 = Vec<2, Diff2<6, double>>;
unsigned int Vec2_g6_size() {
    return Diff2<6, float>::size() * 2;
}

constexpr auto transform_points_2f = transform_points<float>;
constexpr auto transform_points_2d = transform_points<double>;

constexpr auto transform_points_g3_2f = transform_points_g3<float>;
constexpr auto transform_points_g3_2d = transform_points_g3<double>;

f_g6 points_line_string_distance_2f_g3(
    const Vec2f_g3 *points, int npoints,
    const Vec2f_g3 *line_string, int nline
) {
    return points_line_string_distance_g3(
        points, npoints, line_string, nline,
        [](const auto &v) { return v ^ .5; }
    );
}

d_g6 points_line_string_distance_2d_g3(
    const Vec2d_g3 *points, int npoints,
    const Vec2d_g3 *line_string, int nline
) {
    return points_line_string_distance_g3(
        points, npoints, line_string, nline,
        [](const auto &v) { return v ^ .5; }
    );
}

f_g6 points_line_ring_distance_2f_g3(
    const Vec2f_g3 *points, int npoints,
    const Vec2f_g3 *line_ring, int nline
) {
    return points_line_ring_distance_g3(
        points, npoints, line_ring, nline,
        [](const auto &v) { return v ^ .5; }
    );
}

d_g6 points_line_ring_distance_2d_g3(
    const Vec2d_g3 *points, int npoints,
    const Vec2d_g3 *line_ring, int nline
) {
    return points_line_ring_distance_g3(
        points, npoints, line_ring, nline,
        [](const auto &v) { return v ^ .5; }
    );
}
