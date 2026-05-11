#pragma once

#include <cmath>
#include <algorithm>

#include "gjk.h"


template <class T>
struct DistanceResult {
    T distance_sq;      // Squared distance between polygons (0 if intersecting)
    bool intersect;     // True if distance is 0
    int it1, it2;       // Cached indices for temporal coherence
};

// Helper: Find the closest point on line segment AB to the origin
template <class T, class VecType>
inline VecType closest_point_to_origin(
    const VecType& A, const VecType& B, int& out_simplex_size,
    T epsilon = static_cast<T>(1e-8)
) {
    T epsilon_sq = epsilon * epsilon;
    VecType AB = B - A;
    VecType AO = -A;

    T ab_len_sq = AB.len_sq();

    // If A and B are essentially the same point
    if (ab_len_sq < epsilon_sq) {
        out_simplex_size = 1;
        return A;
    }

    // Project AO onto AB to find the interpolation factor 't'
    T t = AO.dp(AB) / ab_len_sq;

    if (t <= 0) {
        // Closest point is A. We drop B from the simplex.
        out_simplex_size = 1;
        return A;
    }
    else if (t >= 1) {
        // Closest point is B. We drop A from the simplex.
        out_simplex_size = 1;
        return B; // Note: the caller must shift B into simplex[0]
    }
    else {
        // Closest point is strictly on the segment between A and B
        out_simplex_size = 2;
        return A + AB * t;
    }
}

// -------------------------------------------------------------------------
// STANDARD DISTANCE ALGORITHM (Hill-Climbing)
// -------------------------------------------------------------------------
template <class T, class VecType>
inline DistanceResult<T> convex_polygons_distance_gjk(
    const VecType* poly1, int n1,
    const VecType* poly2, int n2,
    int it1 = 0, int it2 = 0,
    T epsilon = static_cast<T>(1e-6)
) {
    T epsilon_sq = epsilon * epsilon;
    VecType simplex[3];
    int simplex_size = 0;

    // Initial search direction
    auto dir = poly1[it1] - poly2[it2];

    // If we start exactly on top of each other, distance is 0
    if (dir.len_sq() < epsilon_sq) {
        return { 0, true, it1, it2 };
    }

    // Reverse direction to point TOWARDS the origin
    dir = -dir;

    // Initialize with dir to guarantee dimension matching without {0,0}
    VecType closest_point = dir;
    int op_limit = 32;

    while (op_limit-- > 0) {
        // 1. Get Support Point
        it1 = get_extreme_index<T>(poly1, n1, dir, it1);
        it2 = get_extreme_index<T>(poly2, n2, -dir, it2);

        auto support = poly1[it1] - poly2[it2];

        // 2. Add to Simplex State Machine
        switch (simplex_size) {
            case 0:
                simplex[0] = support;
                simplex_size = 1;
                closest_point = support;
                break;

            case 1:
                simplex[1] = simplex[0];
                simplex[0] = support;
                simplex_size = 2;
                break;

            case 2:
                // Shift oldest point out, keep the newest
                simplex[1] = simplex[0];
                simplex[0] = support;
                break;
        }

        // 3. Find the closest point on our current simplex to the origin
        if (simplex_size == 2) {
            int new_size;
            closest_point = closest_point_to_origin<T>(
                simplex[0], simplex[1], new_size, epsilon
            );

            // If the closest point was B (simplex[1]), shift it to index 0
            if (new_size == 1 && (closest_point - simplex[1]).len_sq() < epsilon_sq) {
                simplex[0] = simplex[1];
            }
            simplex_size = new_size;
        }

        // 4. Check for Convergence
        T dist_sq = closest_point.len_sq();

        if (dist_sq < epsilon_sq) {
            return { 0, true, it1, it2 };
        }

        // Project our new support point onto the direction vector
        T v_dot_w = -closest_point.dp(support);
        if (v_dot_w - dist_sq <= epsilon_sq) {
            return { dist_sq, false, it1, it2 };
        }

        // 5. Update search direction to point from the closest point to the origin
        dir = -closest_point;
    }

    T fallback_dist_sq = closest_point.len_sq();
    return { fallback_dist_sq, fallback_dist_sq < epsilon_sq, it1, it2 };
}

// -------------------------------------------------------------------------
// GRADIENT-BOOSTED DISTANCE ALGORITHM (Galloping Search)
// -------------------------------------------------------------------------
template <class T, class VecType>
inline DistanceResult<T> convex_polygons_distance_gjk_gradient(
    const VecType* poly1, int n1,
    const VecType* poly2, int n2,
    int it1 = 0, int it2 = 0,
    T epsilon = static_cast<T>(1e-6)
) {
    T epsilon_sq = epsilon * epsilon;
    VecType simplex[3];
    int simplex_size = 0;

    auto dir = poly1[it1] - poly2[it2];

    if (dir.len_sq() < epsilon_sq) {
        return { 0, true, it1, it2 };
    }

    dir = -dir;

    VecType closest_point = dir;
    int op_limit = 32;

    while (op_limit-- > 0) {
        // Gradient Support Functions
        it1 = get_extreme_index_gradient<T, VecType>(poly1, n1, dir, it1);
        it2 = get_extreme_index_gradient<T, VecType>(poly2, n2, -dir, it2);

        auto support = poly1[it1] - poly2[it2];

        // Add to Simplex State Machine
        switch (simplex_size) {
            case 0:
                simplex[0] = support;
                simplex_size = 1;
                closest_point = support;
                break;

            case 1:
                simplex[1] = simplex[0];
                simplex[0] = support;
                simplex_size = 2;
                break;

            case 2:
                // Shift oldest point out, keep the newest
                simplex[1] = simplex[0];
                simplex[0] = support;
                break;
        }

        // Find the closest point
        if (simplex_size == 2) {
            int new_size;
            closest_point = closest_point_to_origin<T, VecType>(
                simplex[0], simplex[1], new_size, epsilon
            );

            if (new_size == 1 && (closest_point - simplex[1]).len_sq() < epsilon_sq) {
                simplex[0] = simplex[1];
            }
            simplex_size = new_size;
        }

        T dist_sq = closest_point.len_sq();

        if (dist_sq < epsilon_sq) {
            return { 0, true, it1, it2 };
        }

        // Convergence Check
        T v_dot_w = -closest_point.dp(support);
        if (v_dot_w - dist_sq <= epsilon_sq) {
            return { dist_sq, false, it1, it2 };
        }

        dir = -closest_point;
    }

    T fallback_dist_sq = closest_point.len_sq();
    return { fallback_dist_sq, fallback_dist_sq < epsilon_sq, it1, it2 };
}
