#pragma once

#include <cmath>
#include "gjk.h"


template <class T>
struct IntersectResult {
    bool intersect;
    int it1;
    int it2;
};

// Helper: N-Dimensional Vector Triple Product (a x b) x c
// Using the geometric identity: (a x b) x c = b(a . c) - a(b . c)
template <class VecType>
inline VecType triple_product(const VecType& a, const VecType& b, const VecType& c) {
    return b * a.dp(c) - a * b.dp(c);
}

// -------------------------------------------------------------------------
// CORE IMPLEMENTATION
// -------------------------------------------------------------------------
template <bool UseGradient, class T, class VecType>
inline IntersectResult<T> convex_polygons_intersect_gjk_impl(
    const VecType* poly1, int n1,
    const VecType* poly2, int n2,
    int it1, int it2,
    T epsilon
) {
    const T epsilon_sq = epsilon * epsilon;
    VecType simplex[3];
    int simplex_size = 0;

    // 1. Initial vector between cached points
    auto dir = poly1[it1] - poly2[it2];

    if (dir.len_sq() < epsilon_sq) {
        return {true, it1, it2};
    }

    // 2. LOGICAL FIX: Point towards the origin!
    // GJK searches Minkowski space. We must actively seek the origin (0,0).
    dir = -dir;

    int op_limit = 32;

    while (op_limit-- > 0) {
        // Support Search
        if constexpr (UseGradient) {
            it1 = get_extreme_index_gradient<T, VecType>(poly1, n1, dir, it1);
            it2 = get_extreme_index_gradient<T, VecType>(poly2, n2, -dir, it2);
        } else {
            it1 = get_extreme_index<T, VecType>(poly1, n1, dir, it1);
            it2 = get_extreme_index<T, VecType>(poly2, n2, -dir, it2);
        }

        const auto support = poly1[it1] - poly2[it2];

        // 3. Early Exit: If the furthest point in the direction of the origin
        // hasn't actually crossed the origin, containment is impossible.
        if (support.dp(dir) < 0) {
            return {false, it1, it2};
        }

        // Shift existing points and insert newest point at index 0
        simplex[2] = simplex[1];
        simplex[1] = simplex[0];
        simplex[0] = support;
        simplex_size++;

        // 5. SIMPLEX STATE MACHINE (Hot-Path Optimized)
        // We prioritize size == 3 because after 2 iterations, the algorithm
        // spends its entire life evaluating triangles.
        if (simplex_size == 3) {
            auto AB = simplex[1] - simplex[0];
            auto AC = simplex[2] - simplex[0];
            auto AO = -simplex[0];

            auto AB_perp = triple_product(AC, AB, AB);
            auto AC_perp = triple_product(AB, AC, AC);

            if (AB_perp.dp(AO) > 0) {
                // Drop C, keep A and B
                simplex_size = 2;
                dir = AB_perp;
            }
            else if (AC_perp.dp(AO) > 0) {
                // Drop B, keep A and C
                simplex[1] = simplex[2];
                simplex_size = 2;
                dir = AC_perp;
            }
            else {
                // The origin is strictly bounded by the triangle. Collision!
                return {true, it1, it2};
            }
        }
        else if (simplex_size == 2) {
            auto AB = simplex[1] - simplex[0];
            auto AO = -simplex[0];

            dir = triple_product(AB, AO, AB);

            // If the perpendicular vector is 0, the origin lies exactly on the segment
            if (dir.len_sq() < epsilon_sq) {
                return {true, it1, it2};
            }
        }
        else { // simplex_size == 1
            dir = -simplex[0];
        }
    }

    // Fallback: If op_limit triggers, assume true to prevent object tunneling
    return {true, it1, it2};
}

// -------------------------------------------------------------------------
// PUBLIC APIs
// -------------------------------------------------------------------------
template <class T, class VecType>
IntersectResult<T> convex_polygons_intersect_gjk(
    const VecType* poly1, int n1,
    const VecType* poly2, int n2,
    int it1 = 0, int it2 = 0,
    T epsilon = static_cast<T>(1e-8)
) {
    return convex_polygons_intersect_gjk_impl<false, T, VecType>(poly1, n1, poly2, n2, it1, it2, epsilon);
}

template <class T, class VecType>
IntersectResult<T> convex_polygons_intersect_gjk_gradient(
    const VecType* poly1, int n1,
    const VecType* poly2, int n2,
    int it1 = 0, int it2 = 0,
    T epsilon = static_cast<T>(1e-8)
) {
    return convex_polygons_intersect_gjk_impl<true, T, VecType>(poly1, n1, poly2, n2, it1, it2, epsilon);
}
