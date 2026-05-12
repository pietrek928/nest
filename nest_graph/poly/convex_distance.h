#pragma once

#include <cmath>
#include <limits>
#include "gjk.h"


template <class T, class VecType>
struct SimplexResult {
    VecType closest_point;
    int new_size;
    int indices[2];
};

template <class T>
struct DistanceResult {
    T distance_sq;
    bool intersect;
    int it1, it2;
};

template <class T, class VecType>
inline SimplexResult<T, VecType> closest_point_on_segment(
    const VecType& A, const VecType& B, T epsilon_sq
) {
    VecType AB = B - A;
    T ab_len_sq = AB.qlen();

    if (ab_len_sq < epsilon_sq) {
        return { A, 1, {0, 0} };
    }

    T t = (-A).dp(AB) / ab_len_sq;

    if (t <= 0)      return { A, 1, {0, 0} };
    else if (t >= 1) return { B, 1, {1, 0} };
    else             return { A + AB * t, 2, {0, 1} };
}

// -------------------------------------------------------------------------
// THE CORE IMPLEMENTATION
// -------------------------------------------------------------------------
template <bool UseGradient, class T, class VecType>
inline DistanceResult<T> convex_polygons_distance_gjk_impl(
    const VecType* poly1, int n1,
    const VecType* poly2, int n2,
    int it1, int it2,
    T epsilon
) {
    const T epsilon_sq = epsilon * epsilon;
    VecType simplex[3];
    int simplex_size = 0;

    auto dir = poly1[it1] - poly2[it2];
    if (dir.qlen() < epsilon_sq) return { 0, true, it1, it2 };

    dir = -dir;
    VecType closest_point = -dir;
    T last_dist_sq = std::numeric_limits<T>::max();

    int op_limit = 32;
    while (op_limit-- > 0) {
        if constexpr (UseGradient) {
            it1 = get_extreme_index_gradient<T, VecType>(poly1, n1, dir, it1);
            it2 = get_extreme_index_gradient<T, VecType>(poly2, n2, -dir, it2);
        } else {
            it1 = get_extreme_index<T, VecType>(poly1, n1, dir, it1);
            it2 = get_extreme_index<T, VecType>(poly2, n2, -dir, it2);
        }

        const auto support = poly1[it1] - poly2[it2];
        const T dist_sq = closest_point.qlen();

        // Convergence: Is the new point far enough "past" our current closest point?
        // This projection check is the most numerically stable way to end GJK.
        T proj = (-closest_point).dp(support - closest_point);
        if (proj < epsilon_sq || dist_sq >= last_dist_sq) {
            return { dist_sq, dist_sq < epsilon_sq, it1, it2 };
        }
        last_dist_sq = dist_sq;

        // We check '== 2' first because most GJK iterations occur
        // in this state while "sliding" along the Minkowski boundary.
        if (simplex_size == 2) {
            // Triangle logic: Support + current Segment
            auto res01 = closest_point_on_segment<T, VecType>(support, simplex[0], epsilon_sq);
            auto res02 = closest_point_on_segment<T, VecType>(support, simplex[1], epsilon_sq);

            if (res01.closest_point.qlen() < res02.closest_point.qlen()) {
                closest_point = res01.closest_point;
                VecType next_s[2] = { support, simplex[0] };
                simplex[0] = next_s[res01.indices[0]];
                simplex[1] = next_s[res01.indices[1]];
                simplex_size = res01.new_size;
            } else {
                closest_point = res02.closest_point;
                VecType next_s[2] = { support, simplex[1] };
                simplex[0] = next_s[res02.indices[0]];
                simplex[1] = next_s[res02.indices[1]];
                simplex_size = res02.new_size;
            }
        }
        else if (simplex_size == 1) {
            // Segment logic: Support + current Point
            simplex[1] = simplex[0];
            simplex[0] = support;
            auto res = closest_point_on_segment<T, VecType>(simplex[0], simplex[1], epsilon_sq);

            VecType next_s[2] = { simplex[res.indices[0]], simplex[res.indices[1]] };
            simplex[0] = next_s[0];
            simplex[1] = next_s[1];
            simplex_size = res.new_size;
            closest_point = res.closest_point;
        }
        else {
            // Initial state: size == 0
            simplex[0] = support;
            simplex_size = 1;
            closest_point = support;
        }

        if (closest_point.qlen() < epsilon_sq) return { 0, true, it1, it2 };
        dir = -closest_point;
    }

    return { closest_point.qlen(), false, it1, it2 };
}

// -------------------------------------------------------------------------
// PUBLIC APIs
// -------------------------------------------------------------------------
// Separation distance (squared); when polygons have positive gap,
// DistanceResult::distance_sq is that gap squared (up to epsilon) and
// intersect is usually false unless numerically grazing zero.
//
// Not for penetration, overlap depth, containment, or "one hull inside another":
// once the convex hulls touch or overlap in R^d, vanilla GJK on the difference
// no longer minimizes to a penetration metric; callers must use intersect / EPA
// (or a 2D-specific routine). Do not infer overlap from DistanceResult alone.
template <class T, class VecType>
DistanceResult<T> convex_polygons_distance_gjk(
    const VecType* poly1, int n1, const VecType* poly2, int n2,
    int it1 = 0, int it2 = 0, T epsilon = static_cast<T>(1e-6)
) {
    return convex_polygons_distance_gjk_impl<false, T, VecType>(poly1, n1, poly2, n2, it1, it2, epsilon);
}

// Same contract as convex_polygons_distance_gjk (gradient-based support indices).
template <class T, class VecType>
DistanceResult<T> convex_polygons_distance_gjk_gradient(
    const VecType* poly1, int n1, const VecType* poly2, int n2,
    int it1 = 0, int it2 = 0, T epsilon = static_cast<T>(1e-6)
) {
    return convex_polygons_distance_gjk_impl<true, T, VecType>(poly1, n1, poly2, n2, it1, it2, epsilon);
}
