#pragma once

#include <cmath>
#include <limits>
#include <vector>

#include "intersect.h" // Updated to use the Line String helpers

template <class VecType>
struct PenetrationResult {
    typename VecType::Scalar penetration_sq;
    bool intersect;
    VecType mtv;  // Minimum Translation Vector
    int it1, it2; // Cached indices for coherence
};

// -------------------------------------------------------------------------
// EPA CORE IMPLEMENTATION (Adapted for Convex Line Strings)
// -------------------------------------------------------------------------
template <bool UseGradient, class VecType>
inline PenetrationResult<VecType> convex_linestrings_penetration_gjk_epa_impl(
    const VecType* ls1, int n1,
    const VecType* ls2, int n2,
    int it1, int it2,
    typename VecType::Scalar epsilon
) {
    using Scalar = typename VecType::Scalar;
    const Scalar epsilon_sq = epsilon * epsilon;
    VecType simplex[3];
    int simplex_size = 0;

    // --- PHASE 1: GJK (Find the Origin-Containing Simplex) ---
    auto dir = ls2[it2] - ls1[it1];

    if (dir.qlen() < epsilon_sq) {
        dir = ls1[0] - ls1[n1 / 2];
    }

    int op_limit = 32;
    bool intersected = false;

    while (op_limit-- > 0) {
        if constexpr (UseGradient) {
            it1 = get_extreme_index_linestring_gradient<VecType>(ls1, n1, dir, it1);
            it2 = get_extreme_index_linestring_gradient<VecType>(ls2, n2, -dir, it2);
        } else {
            it1 = get_extreme_index_linestring<VecType>(ls1, n1, dir, it1);
            it2 = get_extreme_index_linestring<VecType>(ls2, n2, -dir, it2);
        }

        const auto support = ls1[it1] - ls2[it2];

        if (support.dp(dir) < 0) {
            return { static_cast<Scalar>(0), false, VecType(), it1, it2 };
        }

        for (int i = 0; i < simplex_size; ++i) {
            if ((support - simplex[i]).qlen() < epsilon_sq) {
                return { static_cast<Scalar>(0), false, VecType(), it1, it2 };
            }
        }

        simplex[2] = simplex[1];
        simplex[1] = simplex[0];
        simplex[0] = support;
        simplex_size++;

        if (simplex_size == 3) {
            auto AB = simplex[1] - simplex[0];
            auto AC = simplex[2] - simplex[0];
            auto AO = -simplex[0];

            auto AB_perp = triple_product(AC, AB, AB);
            auto AC_perp = triple_product(AB, AC, AC);

            if (AB_perp.dp(AO) > 0) {
                simplex_size = 2;
                dir = AB_perp;
            } else if (AC_perp.dp(AO) > 0) {
                simplex[1] = simplex[2];
                simplex_size = 2;
                dir = AC_perp;
            } else {
                intersected = true;
                break;
            }
        } else if (simplex_size == 2) {
            auto AB = simplex[1] - simplex[0];
            auto AO = -simplex[0];
            dir = triple_product(AB, AO, AB);

            if (dir.qlen() < epsilon_sq) {
                intersected = true;
                break;
            }
        } else {
            dir = -simplex[0];
        }
    }

    if (!intersected) return { static_cast<Scalar>(0), false, VecType(), it1, it2 };
    if (simplex_size < 3) return { static_cast<Scalar>(0), true, VecType(), it1, it2 };

    // --- PHASE 2: EPA (Expand the Polytope to find Penetration Vector) ---
    std::vector<VecType> polytope;
    polytope.reserve(32);
    polytope.push_back(simplex[0]);
    polytope.push_back(simplex[1]);
    polytope.push_back(simplex[2]);

    // Robustness Fix: Calculate the true geometric centroid.
    // The origin might be right on the edge, but the centroid is strictly trapped deep inside.
    VecType centroid = (simplex[0] + simplex[1] + simplex[2]) * static_cast<Scalar>(1.0 / 3.0);

    int epa_limit = 32;
    while (epa_limit-- > 0) {
        Scalar min_dist_sq = std::numeric_limits<Scalar>::max();
        int min_index = 0;
        VecType min_normal;
        Scalar min_n_len_sq = 1;
        Scalar min_dot = 0;

        // Optimization: Find the closest edge without using a single square root
        for (size_t i = 0; i < polytope.size(); ++i) {
            size_t j = (i + 1 == polytope.size()) ? 0 : i + 1; // Faster wrap-around than %
            VecType A = polytope[i];
            VecType B = polytope[j];
            VecType AB = B - A;

            // Point towards the centroid to guarantee an "inward" vector
            VecType ACentroid = centroid - A;

            // Triple product yields normal pointing AWAY from centroid (Outward)
            VecType N = -triple_product(AB, ACentroid, AB);
            Scalar n_len_sq = N.qlen();

            if (n_len_sq < epsilon_sq) {
                return { static_cast<Scalar>(0), true, VecType(), it1, it2 };
            }

            // A.N gives the unnormalized projection. Force positive to counter float-drift.
            Scalar dot = A.dp(N);
            if (dot < 0) dot = -dot;

            // Calculate squared distance: dist^2 = dot^2 / length^2
            Scalar current_dist_sq = (dot * dot) / n_len_sq;

            if (current_dist_sq < min_dist_sq) {
                min_dist_sq = current_dist_sq;
                min_index = static_cast<int>(i);
                min_normal = N;
                min_n_len_sq = n_len_sq;
                min_dot = dot;
            }
        }

        // Only compute the expensive sqrt() ONCE per loop, for the winner
        Scalar inv_len = static_cast<Scalar>(1.0) / std::sqrt(static_cast<double>(min_n_len_sq));
        VecType normalized_n = min_normal * inv_len;
        Scalar min_dist = min_dot * inv_len;

        // Request support point
        int tmp_it1, tmp_it2;
        if constexpr (UseGradient) {
            tmp_it1 = get_extreme_index_linestring_gradient<VecType>(ls1, n1, normalized_n, it1);
            tmp_it2 = get_extreme_index_linestring_gradient<VecType>(ls2, n2, -normalized_n, it2);
        } else {
            tmp_it1 = get_extreme_index_linestring<VecType>(ls1, n1, normalized_n, it1);
            tmp_it2 = get_extreme_index_linestring<VecType>(ls2, n2, -normalized_n, it2);
        }
        VecType support = ls1[tmp_it1] - ls2[tmp_it2];

        // Degenerate Edge Guard: If the support point is identical to an existing edge vertex,
        // inserting it creates a 0-length edge and crashes the next loop iteration.
        if ((support - polytope[min_index]).qlen() < epsilon_sq) {
            VecType mtv = -normalized_n * min_dist;
            return { min_dist * min_dist, true, mtv, tmp_it1, tmp_it2 };
        }

        Scalar proj_dist = support.dp(normalized_n);

        if (proj_dist - min_dist < epsilon || epa_limit == 0) {
            VecType mtv = -normalized_n * min_dist;
            return { min_dist * min_dist, true, mtv, tmp_it1, tmp_it2 };
        }

        polytope.insert(polytope.begin() + min_index + 1, support);
    }

    return { static_cast<Scalar>(0), true, VecType(), it1, it2 };
}

// -------------------------------------------------------------------------
// PUBLIC APIs
// -------------------------------------------------------------------------

template <class VecType>
inline PenetrationResult<VecType> convex_linestrings_penetration(
    const VecType* ls1, int n1, const VecType* ls2, int n2,
    int it1 = 0, int it2 = 0,
    typename VecType::Scalar epsilon = static_cast<typename VecType::Scalar>(1e-6)
) {
    return convex_linestrings_penetration_gjk_epa_impl<false, VecType>(ls1, n1, ls2, n2, it1, it2, epsilon);
}

template <class VecType>
inline PenetrationResult<VecType> convex_linestrings_penetration_gradient(
    const VecType* ls1, int n1, const VecType* ls2, int n2,
    int it1 = 0, int it2 = 0,
    typename VecType::Scalar epsilon = static_cast<typename VecType::Scalar>(1e-6)
) {
    return convex_linestrings_penetration_gjk_epa_impl<true, VecType>(ls1, n1, ls2, n2, it1, it2, epsilon);
}