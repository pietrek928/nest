#pragma once

#include <cmath>
#include <limits>
#include <vector>

#include "intersect.h"


template <class VecType>
struct PenetrationResult {
    typename VecType::Scalar penetration_sq;
    bool intersect;
    VecType mtv;  // Minimum Translation Vector: Move poly1 by this vector to resolve the collision.
    int it1, it2; // Cached indices for coherence
};

// -------------------------------------------------------------------------
// EPA CORE IMPLEMENTATION (Expanding Polytope Algorithm)
// -------------------------------------------------------------------------
template <bool UseGradient, class VecType>
inline PenetrationResult<VecType> convex_polygons_penetration_gjk_epa_impl(
    const VecType* poly1, int n1,
    const VecType* poly2, int n2,
    int it1, int it2,
    typename VecType::Scalar epsilon
) {
    using Scalar = typename VecType::Scalar;
    const Scalar epsilon_sq = epsilon * epsilon;
    VecType simplex[3];
    int simplex_size = 0;

    // --- PHASE 1: GJK (Find the Origin-Containing Simplex) ---
    auto dir = poly2[it2] - poly1[it1];

    // Fallback if cached points perfectly overlap
    if (dir.qlen() < epsilon_sq) {
        dir = poly1[0] - poly1[n1 / 2];
    }

    int op_limit = 32;
    bool intersected = false;

    while (op_limit-- > 0) {
        if constexpr (UseGradient) {
            it1 = get_extreme_index_gradient<VecType>(poly1, n1, dir, it1);
            it2 = get_extreme_index_gradient<VecType>(poly2, n2, -dir, it2);
        } else {
            it1 = get_extreme_index<VecType>(poly1, n1, dir, it1);
            it2 = get_extreme_index<VecType>(poly2, n2, -dir, it2);
        }

        const auto support = poly1[it1] - poly2[it2];

        // If the furthest point towards the origin doesn't cross it, they don't intersect
        if (support.dp(dir) < 0) {
            return { static_cast<Scalar>(0), false, VecType(), it1, it2 };
        }

        // Duplicate point check
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
                intersected = true; // Origin is safely trapped!
                break;
            }
        } else if (simplex_size == 2) {
            auto AB = simplex[1] - simplex[0];
            auto AO = -simplex[0];
            dir = triple_product(AB, AO, AB);

            if (dir.qlen() < epsilon_sq) {
                intersected = true; // Origin lies EXACTLY on the segment
                break;
            }
        } else {
            dir = -simplex[0];
        }
    }

    if (!intersected) {
        return { static_cast<Scalar>(0), false, VecType(), it1, it2 };
    }

    // If intersection happened exactly on the boundary, penetration is 0
    if (simplex_size < 3) {
        return { static_cast<Scalar>(0), true, VecType(), it1, it2 };
    }

    // --- PHASE 2: EPA (Expand the Polytope to find Penetration Vector) ---
    std::vector<VecType> polytope;
    polytope.reserve(32); // Pre-allocate to avoid runtime allocations
    polytope.push_back(simplex[0]);
    polytope.push_back(simplex[1]);
    polytope.push_back(simplex[2]);

    int epa_limit = 32;
    while (epa_limit-- > 0) {
        Scalar min_dist = std::numeric_limits<Scalar>::max();
        int min_index = 0;
        VecType min_normal;

        // 1. Find the edge of our polytope closest to the origin
        for (size_t i = 0; i < polytope.size(); ++i) {
            int j = (static_cast<int>(i) + 1) % static_cast<int>(polytope.size());
            VecType A = polytope[i];
            VecType B = polytope[static_cast<size_t>(j)];
            VecType AB = B - A;
            VecType AO = -A;

            // Compute outward normal using the N-dimensional triple product trick.
            // This guarantees the normal points AWAY from the trapped origin.
            VecType N = -triple_product(AB, AO, AB);
            Scalar n_len_sq = N.qlen();

            if (n_len_sq < epsilon_sq) {
                return { static_cast<Scalar>(0), true, VecType(), it1, it2 }; // Origin is exactly on the edge
            }

            // Project A onto normalized N to find the absolute distance to the edge
            Scalar dist = A.dp(N) / std::sqrt(static_cast<double>(n_len_sq));
            if (dist < min_dist) {
                min_dist = dist;
                min_index = static_cast<int>(i);
                min_normal = N;
            }
        }

        // 2. Request a support point from the Minkowski boundary in the direction of the outward normal
        int tmp_it1, tmp_it2;
        if constexpr (UseGradient) {
            tmp_it1 = get_extreme_index_gradient<VecType>(poly1, n1, min_normal, it1);
            tmp_it2 = get_extreme_index_gradient<VecType>(poly2, n2, -min_normal, it2);
        } else {
            tmp_it1 = get_extreme_index<VecType>(poly1, n1, min_normal, it1);
            tmp_it2 = get_extreme_index<VecType>(poly2, n2, -min_normal, it2);
        }
        VecType support = poly1[tmp_it1] - poly2[tmp_it2];

        // 3. Convergence Check: Is the support point significantly further out than our current edge?
        Scalar proj_dist = support.dp(min_normal) / std::sqrt(static_cast<double>(min_normal.qlen()));

        if (proj_dist - min_dist < epsilon || epa_limit == 0) {
            // WE CONVERGED! The closest edge is the true boundary of the Minkowski difference.
            Scalar inv_len = static_cast<Scalar>(1.0) / static_cast<Scalar>(std::sqrt(static_cast<double>(min_normal.qlen())));
            VecType normalized_n = min_normal * inv_len;

            // To push poly1 out of poly2, we move it by exactly the negative distance of the outward normal.
            VecType mtv = -normalized_n * min_dist;

            return { min_dist * min_dist, true, mtv, tmp_it1, tmp_it2 };
        }

        // 4. Expand Polytope: "Tent" the polygon outward by inserting the support point between the edge vertices
        polytope.insert(polytope.begin() + min_index + 1, support);
    }

    return { static_cast<Scalar>(0), true, VecType(), it1, it2 }; // Safety fallback
}

// -------------------------------------------------------------------------
// PUBLIC APIs
// -------------------------------------------------------------------------

// Computes exact penetration depth and the Minimum Translation Vector (MTV)
// required to push poly1 out of poly2. If the polygons are separated,
// intersect will return false and penetration_sq will be 0.
template <class VecType>
inline PenetrationResult<VecType> convex_polygons_penetration(
    const VecType* poly1, int n1, const VecType* poly2, int n2,
    int it1 = 0, int it2 = 0,
    typename VecType::Scalar epsilon = static_cast<typename VecType::Scalar>(1e-6)
) {
    return convex_polygons_penetration_gjk_epa_impl<false, VecType>(poly1, n1, poly2, n2, it1, it2, epsilon);
}

template <class VecType>
inline PenetrationResult<VecType> convex_polygons_penetration_gradient(
    const VecType* poly1, int n1, const VecType* poly2, int n2,
    int it1 = 0, int it2 = 0,
    typename VecType::Scalar epsilon = static_cast<typename VecType::Scalar>(1e-6)
) {
    return convex_polygons_penetration_gjk_epa_impl<true, VecType>(poly1, n1, poly2, n2, it1, it2, epsilon);
}
