#pragma once

#include "algo.h"


using Vec2f = Vec<2, float>;
using Vec2d = Vec<2, double>;
using Vec3f = Vec<3, float>;
using Vec3d = Vec<3, double>;

using Vec2f_g3 = Vec<2, Diff2<3, float>>;
using Vec2d_g3 = Vec<2, Diff2<3, double>>;
unsigned int Vec2_g3_size() {
    return Diff2<3, float>::size() * 2;
}

constexpr auto transform_points_2f = transform_points<float>;
constexpr auto transform_points_2d = transform_points<double>;

constexpr auto transform_points_g3_2f = transform_points_g3<float>;
constexpr auto transform_points_g3_2d = transform_points_g3<double>;
