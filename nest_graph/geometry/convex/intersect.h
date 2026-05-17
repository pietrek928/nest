#pragma once

#include <cmath>

#include "lookup.h"

struct IntersectResult {
    bool intersect;
    int it1;
    int it2;
};

// Helper: N-Dimensional Vector Triple Product (a x b) x c
template <class VecType>
inline VecType triple_product(const VecType& a, const VecType& b, const VecType& c) {
    return b * a.dp(c) - a * b.dp(c);
}

// -------------------------------------------------------------------------
// CORE IMPLEMENTATION
// -------------------------------------------------------------------------
template <bool UseGradient, class VecType>
inline IntersectResult convex_linestrings_intersect_gjk_impl(
    const VecType* ls1, int n1,
    const VecType* ls2, int n2,
    int it1, int it2,
    typename VecType::Scalar epsilon
) {
    using Scalar = typename VecType::Scalar;
    const Scalar epsilon_sq = epsilon * epsilon;
    VecType simplex[3];
    int simplex_size = 0;

    auto dir = ls1[it1] - ls2[it2];

    if (dir.len_sq() < epsilon_sq) {
        return {true, it1, it2};
    }

    dir = -dir;
    int op_limit = 32;

    while (op_limit-- > 0) {
        // Support Search using Line String specific boundaries
        if constexpr (UseGradient) {
            it1 = get_extreme_index_linestring_gradient<VecType>(ls1, n1, dir, it1);
            it2 = get_extreme_index_linestring_gradient<VecType>(ls2, n2, -dir, it2);
        } else {
            it1 = get_extreme_index_linestring<VecType>(ls1, n1, dir, it1);
            it2 = get_extreme_index_linestring<VecType>(ls2, n2, -dir, it2);
        }

        const auto support = ls1[it1] - ls2[it2];

        if (support.dp(dir) < 0) {
            return {false, it1, it2};
        }
        
        if (support.len_sq() < epsilon_sq) {
            return {true, it1, it2};
        }

        simplex[2] = simplex[1];
        simplex[1] = simplex[0];
        simplex[0] = support;
        simplex_size++;

        // SIMPLEX STATE MACHINE (Identical to Polygon Logic)
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
                return {true, it1, it2};
            }
        }
        else if (simplex_size == 2) {
            auto AB = simplex[1] - simplex[0];
            auto AO = -simplex[0];

            dir = triple_product(AB, AO, AB);

            if (dir.len_sq() < epsilon_sq) {
                return {true, it1, it2};
            }
        }
        else {
            dir = -simplex[0];
        }
    }

    return {false, it1, it2};
}

// -------------------------------------------------------------------------
// PUBLIC APIs
// -------------------------------------------------------------------------
template <class VecType>
IntersectResult convex_linestrings_intersect_gjk(
    const VecType* ls1, int n1,
    const VecType* ls2, int n2,
    int it1 = 0, int it2 = 0,
    typename VecType::Scalar epsilon = static_cast<typename VecType::Scalar>(1e-8)
) {
    return convex_linestrings_intersect_gjk_impl<false, VecType>(ls1, n1, ls2, n2, it1, it2, epsilon);
}

template <class VecType>
IntersectResult convex_linestrings_intersect_gjk_gradient(
    const VecType* ls1, int n1,
    const VecType* ls2, int n2,
    int it1 = 0, int it2 = 0,
    typename VecType::Scalar epsilon = static_cast<typename VecType::Scalar>(1e-8)
) {
    return convex_linestrings_intersect_gjk_impl<true, VecType>(ls1, n1, ls2, n2, it1, it2, epsilon);
}