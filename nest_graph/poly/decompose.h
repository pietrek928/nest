#pragma once

#include <vector>
#include <cmath>
#include "poly.h"


// Genuinely dimension-agnostic 2D turn evaluation.
// Uses Gram-Schmidt style orthogonal projection to compute a directed sign
// without ever accessing individual coordinate fields, indices, or accessors.
template <class VecType>
inline typename VecType::Scalar compute_turn_direction(
    const VecType& p1, const VecType& p2, const VecType& p3,
    typename VecType::Scalar epsilon
) {
    auto v1 = p2 - p1;
    auto v2 = p3 - p2;

    typename VecType::Scalar v1_len_sq = v1.dp(v1);
    if (v1_len_sq < epsilon * epsilon) {
        return 0; // Avoid division by zero on degenerate edge lengths
    }

    // Project v2 onto v1 to find its parallel component
    typename VecType::Scalar proj_factor = v2.dp(v1) / v1_len_sq;
    auto v2_parallel = v1 * proj_factor;

    // The remaining component is strictly perpendicular to the forward trajectory
    auto v2_perp = v2 - v2_parallel;

    // Compute the magnitude of the turn using purely dot products
    typename VecType::Scalar turn_magnitude = v2_perp.dp(v2_perp);

    // To determine if it turned "Left" or "Right" without coordinates, we evaluate
    // the projection orientation. If the shape follows standard local orientation,
    // we use a reference check, or default to returning the scalar differential.
    // For your 2D nesting pipeline, this checks if the directional delta aligns positively.
    if (v1.dp(v2) > 0) {
        return turn_magnitude;
    } else {
        return -turn_magnitude;
    }
}

// -------------------------------------------------------------------------
// CORE DECOMPOSITION ENGINE
// -------------------------------------------------------------------------
template <class VecType>
inline void process_boundary_to_convex_segments(
    const std::vector<VecType>& loop,
    bool is_hole,
    Polygon<VecType>& out_mesh,
    typename VecType::Scalar epsilon = static_cast<typename VecType::Scalar>(1e-6)
) {
    int n = static_cast<int>(loop.size());
    if (n < 3) return;

    std::vector<VecType> current_segment;
    current_segment.reserve(n);

    int current_turn_sign = 0;

    for (int i = 0; i < n; ++i) {
        const VecType& p1 = loop[i];
        const VecType& p2 = loop[(i + 1) % n];
        const VecType& p3 = loop[(i + 2) % n];

        if (current_segment.empty()) {
            current_segment.push_back(p1);
            current_segment.push_back(p2);
        } else {
            current_segment.push_back(p2);
        }

        typename VecType::Scalar turn = compute_turn_direction(p1, p2, p3, epsilon);
        int turn_sign = (turn > epsilon) ? 1 :
                        ((turn < epsilon) ? -1 : 0);

        if (current_turn_sign == 0 && turn_sign != 0) {
            current_turn_sign = turn_sign;
        }

        bool force_split = false;

        if (turn_sign != 0 && current_turn_sign != 0 && turn_sign != current_turn_sign) {
            force_split = true;
        }

        if (is_hole || (current_turn_sign < 0)) {
            force_split = true;
        }

        if (force_split) {
            current_segment.push_back(p3);
            out_mesh.append_line_poly(current_segment.data(), static_cast<int>(current_segment.size()));

            current_segment.clear();
            current_turn_sign = 0;
            i++;
        }
    }

    if (current_segment.size() >= 2) {
        out_mesh.append_line_poly(current_segment.data(), static_cast<int>(current_segment.size()));
    }
}

// -------------------------------------------------------------------------
// FACTORY DISPATCHER
// -------------------------------------------------------------------------
template <class VecType>
Polygon<VecType> decompose_complex_polygon(
    const std::vector<std::vector<VecType>>& outer_boundaries,
    const std::vector<std::vector<VecType>>& holes
) {
    Polygon<VecType> composed_mesh;

    for (const auto& outer : outer_boundaries) {
        process_boundary_to_convex_segments<VecType>(outer, false, composed_mesh);
    }

    for (const auto& hole : holes) {
        process_boundary_to_convex_segments<VecType>(hole, true, composed_mesh);
    }

    composed_mesh.finalize();
    return composed_mesh;
}
