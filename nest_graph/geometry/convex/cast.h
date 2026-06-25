#pragma once

#include <cmath>
#include <algorithm>
#include "lookup.h"


template <class VecType>
struct ShapeCastResult {
    bool intersects_path;             // True if the shape will ever hit along this vector
    typename VecType::Scalar t_entry; // Absolute distance to move to JUST touch (kiss) the shape
    typename VecType::Scalar t_exit;  // Absolute distance to move to completely clear the shape
    int it1, it2;                     // Cached indices for coherence
};

// -------------------------------------------------------------------------
// SHAPE CAST CORE IMPLEMENTATION (O(1) Loop Optimization)
// -------------------------------------------------------------------------
template <bool UseGradient, class VecType>
inline ShapeCastResult<VecType> convex_linestrings_cast_impl(
    const VecType* ls1, int n1,
    const VecType* ls2, int n2,
    const VecType& slide_vector,
    int it1, int it2,
    typename VecType::Scalar epsilon
) {
    using Scalar = typename VecType::Scalar;
    const Scalar epsilon_sq = epsilon * epsilon;
    Scalar v_len_sq = slide_vector.qlen();

    if (v_len_sq < epsilon_sq) {
        return {false, static_cast<Scalar>(0), static_cast<Scalar>(0), it1, it2};
    }

    // This is the ONLY sqrt() in the entire function. It happens once outside the loops
    // to ensure our returned t_entry and t_exit values are exact absolute distances.
    Scalar v_len = std::sqrt(v_len_sq);
    VecType d = slide_vector * (static_cast<Scalar>(-1.0) / v_len);
    VecType n = {-d[1], d[0]};

    auto get_support = [&](const VecType& dir_vec, int& i1, int& i2) -> VecType {
        if constexpr (UseGradient) {
            i1 = get_extreme_index_linestring_gradient<VecType>(ls1, n1, dir_vec, i1);
            i2 = get_extreme_index_linestring_gradient<VecType>(ls2, n2, -dir_vec, i2);
        } else {
            i1 = get_extreme_index_linestring<VecType>(ls1, n1, dir_vec, i1);
            i2 = get_extreme_index_linestring<VecType>(ls2, n2, -dir_vec, i2);
        }
        return ls1[i1] - ls2[i2];
    };

    VecType S_left = get_support(n, it1, it2);
    VecType S_right = get_support(-n, it1, it2);

    Scalar dist_left = S_left.dp(n);
    Scalar dist_right = S_right.dp(-n);

    // If the absolute lateral extremes don't straddle the ray, the path completely misses
    if (dist_left < -epsilon || dist_right < -epsilon) {
        return {false, static_cast<Scalar>(0), static_cast<Scalar>(0), it1, it2};
    }

    // Highly optimized exact ray intersection calculation
    auto get_t = [&](const VecType& P_A, const VecType& P_B) -> Scalar {
        Scalar dot_A = P_A.dp(n);
        Scalar dot_B = P_B.dp(n);
        Scalar denom = dot_B - dot_A;
        if (std::abs(denom) < epsilon) {
            return std::max(P_A.dp(d), P_B.dp(d));
        }
        Scalar u = -dot_A / denom;
        return P_A.dp(d) + u * (P_B - P_A).dp(d);
    };

    // =========================================================================
    // PHASE 1: Find t_max (The Escape Point)
    // =========================================================================
    VecType A = S_left;
    VecType B = S_right;
    Scalar t_max = 0;
    int limit = 32;

    while (limit-- > 0) {
        VecType AB = B - A;
        Scalar L_sq = AB.qlen();

        if (L_sq < epsilon_sq) {
            t_max = A.dp(d);
            break;
        }

        // Unnormalized normal (saves sqrt and division)
        VecType m = {-AB[1], AB[0]};
        VecType S = get_support(m, it1, it2);

        // Check if the new point significantly extends the boundary
        Scalar diff = (S - A).dp(m);

        // Mathematically identical to true_dist <= epsilon, but incredibly fast
        if (diff <= 0 || (diff * diff) <= epsilon_sq * L_sq) {
            t_max = get_t(A, B);
            break;
        }

        Scalar S_n = S.dp(n);
        if (S_n > epsilon) A = S;
        else if (S_n < -epsilon) B = S;
        else { t_max = S.dp(d); break; }
    }

    // =========================================================================
    // PHASE 2: Find t_min (The Entry/Kiss Point)
    // =========================================================================
    A = S_right;
    B = S_left;
    Scalar t_min = 0;
    limit = 32;

    while (limit-- > 0) {
        VecType AB = B - A;
        Scalar L_sq = AB.qlen();

        if (L_sq < epsilon_sq) {
            t_min = A.dp(d);
            break;
        }

        VecType m = {-AB[1], AB[0]};
        VecType S = get_support(m, it1, it2);

        Scalar diff = (S - A).dp(m);

        if (diff <= 0 || (diff * diff) <= epsilon_sq * L_sq) {
            t_min = get_t(A, B);
            break;
        }

        Scalar S_n = S.dp(n);
        if (S_n < -epsilon) A = S;
        else if (S_n > epsilon) B = S;
        else { t_min = S.dp(d); break; }
    }

    // Obstacle entirely behind the cast direction: not a forward impact.
    if (t_max < -epsilon) {
        return {false, static_cast<Scalar>(0), static_cast<Scalar>(0), it1, it2};
    }

    return {true, t_min, t_max, it1, it2};
}

// -------------------------------------------------------------------------
// PUBLIC APIs
// -------------------------------------------------------------------------

template <class VecType>
inline ShapeCastResult<VecType> convex_linestrings_cast(
    const VecType* ls1, int n1,
    const VecType* ls2, int n2,
    const VecType& slide_vector,
    int it1 = 0, int it2 = 0,
    typename VecType::Scalar epsilon = static_cast<typename VecType::Scalar>(1e-6)
) {
    return convex_linestrings_cast_impl<false, VecType>(ls1, n1, ls2, n2, slide_vector, it1, it2, epsilon);
}

template <class VecType>
inline ShapeCastResult<VecType> convex_linestrings_cast_gradient(
    const VecType* ls1, int n1,
    const VecType* ls2, int n2,
    const VecType& slide_vector,
    int it1 = 0, int it2 = 0,
    typename VecType::Scalar epsilon = static_cast<typename VecType::Scalar>(1e-6)
) {
    return convex_linestrings_cast_impl<true, VecType>(ls1, n1, ls2, n2, slide_vector, it1, it2, epsilon);
}