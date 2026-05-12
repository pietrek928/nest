#pragma once

#include "gjk.h"


// -------------------------------------------------------------------------
// SPECIALIZED GJK: Point vs Convex Polygon
// -------------------------------------------------------------------------
template <class T, class VecType>
inline bool point_in_convex_poly_gjk(
    const VecType& P,
    const VecType* polyB, int nB,
    int& cached_it2,
    T epsilon
) {
    T epsilon_sq = epsilon * epsilon;
    VecType simplex[3];
    int simplex_size = 0;

    // Initial direction: from a vertex on B towards our point P
    auto dir = P - polyB[cached_it2];

    if (dir.qlen() < epsilon_sq) {
        return true; // Point is exactly on the cached vertex
    }

    int op_limit = 32;

    while (op_limit-- > 0) {
        // 1. MASSIVE OPTIMIZATION: We only search Polygon B!
        // P is always the extreme point of Polygon A.
        cached_it2 = get_extreme_index<T, VecType>(polyB, nB, -dir, cached_it2);

        auto support = P - polyB[cached_it2];

        // Terminate early if the furthest point doesn't cross the origin
        if (support.dp(dir) < 0) {
            return false;
        }

        // Duplicate Point Check (Cycle Detection)
        for (int i = 0; i < simplex_size; ++i) {
            if ((support - simplex[i]).qlen() < epsilon_sq) {
                return false;
            }
        }

        // Update Simplex
        simplex[2] = simplex[1];
        simplex[1] = simplex[0];
        simplex[0] = support;
        simplex_size++;

        // Evolve Simplex State Machine (Exactly as we optimized earlier!)
        switch (simplex_size) {
            case 2: {
                auto AB = simplex[1] - simplex[0];
                auto AO = -simplex[0];

                dir = triple_product(AB, AO, AB);
                if (dir.qlen() < epsilon_sq) return true;
                break;
            }
            case 3: {
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
                    return true;
                }
                break;
            }
        }
    }

    return true;
}

// -------------------------------------------------------------------------
// ULTIMATE N-DIMENSIONAL CONTAINMENT CHECK
// -------------------------------------------------------------------------
template <class T, class VecType>
inline bool is_polygon_fully_inside(
    const VecType* polyA, int nA,
    const VecType* polyB, int nB,
    T epsilon = static_cast<T>(1e-8)
) {
    if (nA == 0 || nB == 0) return false;

    int cached_it2 = 0;

    for (int i = 0; i < nA; ++i) {
        // Use our new ultra-lean specialized GJK
        if (!point_in_convex_poly_gjk<T, VecType>(
            polyA[i], polyB, nB, cached_it2, epsilon
        )) {
            return false;
        }
    }

    return true;
}
