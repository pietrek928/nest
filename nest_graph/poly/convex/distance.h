#pragma once

#include <cmath>
#include <limits>
#include "../../vec.h"
#include "../gjk.h"

template <class VecType>
struct SimplexResult {
    VecType closest_point;
    int new_size;
    int indices[2];
};

template <class VecType>
struct DistanceResult {
    typename VecType::Scalar distance_sq;
    bool intersect;
    int it1, it2;
};

template <class VecType>
inline SimplexResult<VecType> closest_point_on_segment(
    const VecType& A, const VecType& B, typename VecType::Scalar epsilon_sq
) {
    using Scalar = typename VecType::Scalar;
    VecType AB = B - A;
    Scalar ab_len_sq = AB.qlen();

    if (ab_len_sq < epsilon_sq) {
        return { A, 1, {0, 0} };
    }

    Scalar t = (-A).dp(AB) / ab_len_sq;

    if (t <= 0)      return { A, 1, {0, 0} };
    else if (t >= 1) return { B, 1, {1, 0} };
    else             return { A + AB * t, 2, {0, 1} };
}

// -------------------------------------------------------------------------
// THE CORE IMPLEMENTATION
// -------------------------------------------------------------------------
template <bool UseGradient, class VecType>
inline DistanceResult<VecType> convex_polygons_distance_gjk_impl(
    const VecType* poly1, int n1,
    const VecType* poly2, int n2,
    int it1, int it2,
    typename VecType::Scalar epsilon
) {
    using Scalar = typename VecType::Scalar;
    const Scalar epsilon_sq = epsilon * epsilon;
    VecType simplex[3];
    int simplex_size = 0;

    auto dir = poly1[it1] - poly2[it2];
    if (dir.qlen() < epsilon_sq) return { static_cast<Scalar>(0), true, it1, it2 };

    dir = -dir;
    VecType closest_point = -dir;
    Scalar last_dist_sq = std::numeric_limits<Scalar>::max();

    int op_limit = 32;
    while (op_limit-- > 0) {
        if constexpr (UseGradient) {
            it1 = get_extreme_index_gradient<VecType>(poly1, n1, dir, it1);
            it2 = get_extreme_index_gradient<VecType>(poly2, n2, -dir, it2);
        } else {
            it1 = get_extreme_index<VecType>(poly1, n1, dir, it1);
            it2 = get_extreme_index<VecType>(poly2, n2, -dir, it2);
        }

        const auto support = poly1[it1] - poly2[it2];
        const Scalar dist_sq = closest_point.qlen();

        // Convergence: Is the new point far enough "past" our current closest point?
        Scalar proj = (-closest_point).dp(support - closest_point);

        // If the support point doesn't meaningfully push us closer to the origin,
        // or we stalled, the current closest_point is the absolute minimum.
        if (proj < epsilon_sq || dist_sq >= last_dist_sq) {
            return { dist_sq, dist_sq < epsilon_sq, it1, it2 };
        }

        last_dist_sq = dist_sq;

        // --- SIMPLEX EVOLUTION ---
        if (simplex_size == 2) {
            // Forward-Progression Triangle Logic:
            // Because we passed the convergence check above, the new support point is GUARANTEED
            // to form a boundary closer to the origin than the old segment.
            // We strictly evaluate the two NEW edges. Do not evaluate the old edge.
            auto res01 = closest_point_on_segment<VecType>(support, simplex[0], epsilon_sq);
            auto res02 = closest_point_on_segment<VecType>(support, simplex[1], epsilon_sq);

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
            auto res = closest_point_on_segment<VecType>(simplex[0], simplex[1], epsilon_sq);

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

        if (closest_point.qlen() < epsilon_sq) {
            return { static_cast<Scalar>(0), true, it1, it2 };
        }

        // Seek origin from current closest feature
        dir = -closest_point;
    }

    // Fallback if max iterations reached
    return { closest_point.qlen(), closest_point.qlen() < epsilon_sq, it1, it2 };
}

// -------------------------------------------------------------------------
// PUBLIC APIs
// -------------------------------------------------------------------------
template <class VecType>
DistanceResult<VecType> convex_polygons_distance_gjk(
    const VecType* poly1, int n1, const VecType* poly2, int n2,
    int it1 = 0, int it2 = 0,
    typename VecType::Scalar epsilon = static_cast<typename VecType::Scalar>(1e-6)
) {
    return convex_polygons_distance_gjk_impl<false, VecType>(poly1, n1, poly2, n2, it1, it2, epsilon);
}

template <class VecType>
DistanceResult<VecType> convex_polygons_distance_gjk_gradient(
    const VecType* poly1, int n1, const VecType* poly2, int n2,
    int it1 = 0, int it2 = 0,
    typename VecType::Scalar epsilon = static_cast<typename VecType::Scalar>(1e-6)
) {
    return convex_polygons_distance_gjk_impl<true, VecType>(poly1, n1, poly2, n2, it1, it2, epsilon);
}
