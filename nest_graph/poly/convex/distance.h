#pragma once

#include <cmath>
#include <limits>
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
// THE CORE IMPLEMENTATION (Unified & Robust)
// -------------------------------------------------------------------------
template <bool UseGradient, class VecType>
inline DistanceResult<VecType> convex_polygons_distance_gjk_impl(
    const VecType* poly1, int n1,
    const VecType* poly2, int n2,
    bool known_overlap,
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

        Scalar proj = (-closest_point).dp(support - closest_point);

        // Stall detection is strictly enabled for overlapping shapes (Minkowski slack).
        // Separated shapes disable it to allow proper Euclidean boundary sliding.
        const bool stall = known_overlap && (dist_sq >= last_dist_sq);
        if (proj < epsilon_sq || stall) {
            return { dist_sq, dist_sq < epsilon_sq, it1, it2 };
        }

        last_dist_sq = dist_sq;

        // --- SIMPLEX EVOLUTION ---
        if (simplex_size == 2) {
            auto res01 = closest_point_on_segment<VecType>(support, simplex[0], epsilon_sq);
            auto res02 = closest_point_on_segment<VecType>(support, simplex[1], epsilon_sq);

            if (!known_overlap) {
                // Disjoint: Evaluate all 3 edges to allow sliding past planar seams (Fixes TC2/TC6)
                auto res12 = closest_point_on_segment<VecType>(simplex[0], simplex[1], epsilon_sq);

                const Scalar d01 = res01.closest_point.qlen();
                const Scalar d02 = res02.closest_point.qlen();
                const Scalar d12 = res12.closest_point.qlen();

                if (d01 <= d02 && d01 <= d12) {
                    closest_point = res01.closest_point;
                    VecType next_s[2] = { support, simplex[0] };
                    simplex[0] = next_s[res01.indices[0]];
                    simplex[1] = next_s[res01.indices[1]];
                    simplex_size = res01.new_size;
                } else if (d02 <= d12) {
                    closest_point = res02.closest_point;
                    VecType next_s[2] = { support, simplex[1] };
                    simplex[0] = next_s[res02.indices[0]];
                    simplex[1] = next_s[res02.indices[1]];
                    simplex_size = res02.new_size;
                } else {
                    closest_point = res12.closest_point;
                    VecType next_s[2] = { simplex[0], simplex[1] };
                    simplex[0] = next_s[res12.indices[0]];
                    simplex[1] = next_s[res12.indices[1]];
                    simplex_size = res12.new_size;
                }
            } else {
                // Overlap: Strictly evaluate new edges to march inward and rely on stall check
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
        } else if (simplex_size == 1) {
            simplex[1] = simplex[0];
            simplex[0] = support;
            auto res = closest_point_on_segment<VecType>(simplex[0], simplex[1], epsilon_sq);

            VecType next_s[2] = { simplex[res.indices[0]], simplex[res.indices[1]] };
            simplex[0] = next_s[0];
            simplex[1] = next_s[1];
            simplex_size = res.new_size;
            closest_point = res.closest_point;
        } else {
            simplex[0] = support;
            simplex_size = 1;
            closest_point = support;
        }

        if (closest_point.qlen() < epsilon_sq) {
            return { static_cast<Scalar>(0), true, it1, it2 };
        }

        dir = -closest_point;
    }

    return { closest_point.qlen(), closest_point.qlen() < epsilon_sq, it1, it2 };
}

// -------------------------------------------------------------------------
// PUBLIC APIs
// -------------------------------------------------------------------------

template <class VecType>
DistanceResult<VecType> convex_polygons_distance_gjk(
    const VecType* poly1, int n1, const VecType* poly2, int n2,
    bool known_overlap = false,
    int it1 = 0, int it2 = 0,
    typename VecType::Scalar epsilon = static_cast<typename VecType::Scalar>(1e-6)
) {
    return convex_polygons_distance_gjk_impl<false, VecType>(poly1, n1, poly2, n2, known_overlap, it1, it2, epsilon);
}

template <class VecType>
DistanceResult<VecType> convex_polygons_distance_gjk_gradient(
    const VecType* poly1, int n1, const VecType* poly2, int n2,
    bool known_overlap = false,
    int it1 = 0, int it2 = 0,
    typename VecType::Scalar epsilon = static_cast<typename VecType::Scalar>(1e-6)
) {
    return convex_polygons_distance_gjk_impl<true, VecType>(poly1, n1, poly2, n2, known_overlap, it1, it2, epsilon);
}
