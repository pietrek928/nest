#pragma once

#include <cmath>
#include <algorithm>
#include "intersect.h"

// -------------------------------------------------------------------------
// CORE IMPLEMENTATION: Point vs Convex Polygon
// -------------------------------------------------------------------------
template <bool UseGradient, class VecType>
inline bool point_in_convex_poly_gjk_impl(
    const VecType& P,
    const VecType* polyB, int nB,
    int& cached_it2,
    typename VecType::Scalar epsilon
) {
    using Scalar = typename VecType::Scalar;
    const Scalar epsilon_sq = epsilon * epsilon;
    VecType simplex[3];
    int simplex_size = 0;

    // 1. Initial direction
    auto dir = P - polyB[cached_it2];

    if (dir.qlen() < epsilon_sq) {
        return true;
    }

    // 2. Seek the origin!
    dir = -dir;

    int op_limit = 32;

    while (op_limit-- > 0) {
        // 3. Support Search (Only searching Polygon B)
        if constexpr (UseGradient) {
            cached_it2 = get_extreme_index_gradient<VecType>(polyB, nB, -dir, cached_it2);
        } else {
            cached_it2 = get_extreme_index<VecType>(polyB, nB, -dir, cached_it2);
        }

        auto support = P - polyB[cached_it2];

        // 4. Convergence Check
        if (support.dp(dir) < 0) {
            return false;
        }

        // 5. Cycle Detection
        for (int i = 0; i < simplex_size; ++i) {
            if ((support - simplex[i]).qlen() < epsilon_sq) {
                return false;
            }
        }

        simplex[2] = simplex[1];
        simplex[1] = simplex[0];
        simplex[0] = support;
        simplex_size++;

        // 6. SIMPLEX STATE MACHINE (Hot-Path Optimized)
        if (simplex_size == 3) {
            auto AB = simplex[1] - simplex[0];
            auto AC = simplex[2] - simplex[0];
            auto AO = -simplex[0];

            auto AB_perp = triple_product(AC, AB, AB);
            auto AC_perp = triple_product(AB, AC, AC);

            if (AB_perp.dp(AO) > 0) {
                simplex_size = 2;
                dir = AB_perp;
            }
            else if (AC_perp.dp(AO) > 0) {
                simplex[1] = simplex[2];
                simplex_size = 2;
                dir = AC_perp;
            }
            else {
                return true; // Origin encapsulated
            }
        }
        else if (simplex_size == 2) {
            auto AB = simplex[1] - simplex[0];
            auto AO = -simplex[0];

            dir = triple_product(AB, AO, AB);
            if (dir.qlen() < epsilon_sq) return true;
        }
        else { // simplex_size == 1
            // FIXED FATAL BUG: Update direction toward origin!
            dir = -simplex[0];
        }
    }

    return true; // Fallback safety
}

// -------------------------------------------------------------------------
// PUBLIC APIs
// -------------------------------------------------------------------------
template <class VecType>
bool is_polygon_fully_inside(
    const VecType* polyA, int nA,
    const VecType* polyB, int nB,
    typename VecType::Scalar epsilon = static_cast<typename VecType::Scalar>(1e-8)
) {
    if (nA == 0 || nB == 0) return false;

    int cached_it2 = 0;

    for (int i = 0; i < nA; ++i) {
        if (!point_in_convex_poly_gjk_impl<false, VecType>(
            polyA[i], polyB, nB, cached_it2, epsilon
        )) {
            return false;
        }
    }

    return true;
}

template <class VecType>
bool is_polygon_fully_inside_gradient(
    const VecType* polyA, int nA,
    const VecType* polyB, int nB,
    typename VecType::Scalar epsilon = static_cast<typename VecType::Scalar>(1e-8)
) {
    if (nA == 0 || nB == 0) return false;

    int cached_it2 = 0;

    for (int i = 0; i < nA; ++i) {
        if (!point_in_convex_poly_gjk_impl<true, VecType>(
            polyA[i], polyB, nB, cached_it2, epsilon
        )) {
            return false;
        }
    }

    return true;
}
