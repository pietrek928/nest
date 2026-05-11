#pragma once

#include <cmath>
#include <algorithm>

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
// STANDARD GJK INTERSECTION FUNCTION (Hill-Climbing)
// -------------------------------------------------------------------------
template <class T, class VecType>
inline IntersectResult<T> convex_polygons_intersect_gjk(
    const VecType* poly1, int n1,
    const VecType* poly2, int n2,
    int it1 = 0, int it2 = 0,
    T epsilon = static_cast<T>(1e-8)
) {
    T epsilon_sq = epsilon * epsilon;
    VecType simplex[3];
    int simplex_size = 0;

    // Initial search direction based on cached points
    auto dir = poly1[it1] - poly2[it2];

    // If starting points are perfectly overlapping, they intersect
    if (dir.len_sq() < epsilon_sq) {
        return {true, it1, it2};
    }

    int op_limit = 32; // Silent safety net for catastrophic float failures

    while (op_limit-- > 0) {
        // 1. Get Support Point on Minkowski Difference
        it1 = get_extreme_index<T>(poly1, n1, dir, it1);
        it2 = get_extreme_index<T>(poly2, n2, -dir, it2);

        auto support = poly1[it1] - poly2[it2];

        // 2. Terminate early if the furthest point doesn't cross the origin
        if (support.dp(dir) < 0) {
            return {false, it1, it2};
        }

        // 3. Duplicate Point Check (Cycle Detection)
        for (int i = 0; i < simplex_size; ++i) {
            auto diff = support - simplex[i];
            if (diff.len_sq() < epsilon_sq) {
                return {false, it1, it2};
            }
        }

        // 4. Update Simplex
        simplex[2] = simplex[1];
        simplex[1] = simplex[0];
        simplex[0] = support;
        simplex_size++;

        // 5. Evolve Simplex State Machine
        switch (simplex_size) {
            case 2: { // Line Segment State
                auto AB = simplex[1] - simplex[0];
                auto AO = -simplex[0];

                dir = triple_product(AB, AO, AB);
                if (dir.len_sq() < epsilon_sq) {
                    return {true, it1, it2};
                }
                break;
            }
            case 3: { // Triangle State
                auto AB = simplex[1] - simplex[0];
                auto AC = simplex[2] - simplex[0];
                auto AO = -simplex[0];

                auto AB_perp = triple_product(AC, AB, AB);
                auto AC_perp = triple_product(AB, AC, AC);

                if (AB_perp.dp(AO) > 0) {
                    simplex_size = 2; // Downgrade to Line
                    dir = AB_perp;
                }
                else if (AC_perp.dp(AO) > 0) {
                    simplex[1] = simplex[2]; // Keep AC, drop B
                    simplex_size = 2; // Downgrade to Line
                    dir = AC_perp;
                }
                else {
                    // Origin safely encapsulated
                    return {true, it1, it2};
                }
                break;
            }
        }
    }

    // Safety net fallback (assume collision if math completely breaks down)
    return {true, it1, it2};
}

// -------------------------------------------------------------------------
// GRADIENT-BOOSTED GJK INTERSECTION (Galloping Search)
// -------------------------------------------------------------------------
template <class T, class VecType>
inline IntersectResult<T> convex_polygons_intersect_gjk_gradient(
    const VecType* poly1, int n1,
    const VecType* poly2, int n2,
    int it1 = 0, int it2 = 0,
    T epsilon = static_cast<T>(1e-8)
) {
    T epsilon_sq = epsilon * epsilon;
    VecType simplex[3];
    int simplex_size = 0;

    auto dir = poly1[it1] - poly2[it2];

    if (dir.len_sq() < epsilon_sq) {
        return {true, it1, it2};
    }

    int op_limit = 32;

    while (op_limit-- > 0) {
        // Use the Gradient-Boosted Exponential Search
        it1 = get_extreme_index_gradient<T, VecType>(poly1, n1, dir, it1);
        it2 = get_extreme_index_gradient<T, VecType>(poly2, n2, -dir, it2);

        auto support = poly1[it1] - poly2[it2];

        // Terminate early if the furthest point doesn't cross the origin
        if (support.dp(dir) < 0) {
            return {false, it1, it2};
        }

        // Duplicate Point Check (Cycle Detection)
        for (int i = 0; i < simplex_size; ++i) {
            auto diff = support - simplex[i];
            if (diff.len_sq() < epsilon_sq) {
                return {false, it1, it2};
            }
        }

        simplex[2] = simplex[1];
        simplex[1] = simplex[0];
        simplex[0] = support;
        simplex_size++;

        // Evolve Simplex State Machine
        switch (simplex_size) {
            case 2: { // Line Segment State
                auto AB = simplex[1] - simplex[0];
                auto AO = -simplex[0];

                dir = triple_product(AB, AO, AB);
                if (dir.len_sq() < epsilon_sq) {
                    return {true, it1, it2};
                }
                break;
            }
            case 3: { // Triangle State
                auto AB = simplex[1] - simplex[0];
                auto AC = simplex[2] - simplex[0];
                auto AO = -simplex[0];

                auto AB_perp = triple_product(AC, AB, AB);
                auto AC_perp = triple_product(AB, AC, AC);

                if (AB_perp.dp(AO) > 0) {
                    simplex_size = 2; // Downgrade to Line
                    dir = AB_perp;
                }
                else if (AC_perp.dp(AO) > 0) {
                    simplex[1] = simplex[2]; // Keep AC, drop B
                    simplex_size = 2; // Downgrade to Line
                    dir = AC_perp;
                }
                else {
                    // Origin safely encapsulated
                    return {true, it1, it2};
                }
                break;
            }
        }
    }

    return {true, it1, it2};
}
