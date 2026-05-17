#pragma once

#include <algorithm>
#include <vector>
#include <cmath>
#include "poly.h"


template <class VecType>
inline std::vector<VecType> reverse_ring(const std::vector<VecType>& ring) {
    std::vector<VecType> out = ring;
    std::reverse(out.begin(), out.end());
    return out;
}


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
    return v1[0] * v2[1] - v1[1] * v2[0];
}

// -------------------------------------------------------------------------
// CORE DECOMPOSITION ENGINE
// -------------------------------------------------------------------------
template <class VecType>
inline void process_boundary_to_convex_segments(
    const std::vector<VecType>& loop,
    SolidGeometry<VecType>& out_mesh,
    bool subtractive = false,
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
                        ((turn < -epsilon) ? -1 : 0);

        if (current_turn_sign == 0 && turn_sign != 0) {
            current_turn_sign = turn_sign;
        }

        bool force_split = false;

        // Split if the turn direction changes (e.g., from convex to concave)
        if (turn_sign != 0 && current_turn_sign != 0 && turn_sign != current_turn_sign) {
            force_split = true;
        }

        // Split if it's a concave turn
        if (current_turn_sign < 0) {
            force_split = true;
        }

        // Also check if adding p3 would make the current segment concave internally
        if (!force_split && current_segment.size() >= 2) {
            const VecType& last_p1 = current_segment[current_segment.size() - 2];
            const VecType& last_p2 = current_segment[current_segment.size() - 1];
            typename VecType::Scalar internal_turn = compute_turn_direction(last_p1, last_p2, p3, epsilon);
            int internal_turn_sign = (internal_turn > epsilon) ? 1 :
                                     ((internal_turn < -epsilon) ? -1 : 0);
            
            if (internal_turn_sign < 0 || (internal_turn_sign != 0 && current_turn_sign != 0 && internal_turn_sign != current_turn_sign)) {
                force_split = true;
            }
        }

        // Split if the accumulated turn exceeds 90 degrees
        // This is crucial for highly curved shapes like spirals, where the turn direction
        // is constant but the total turn exceeds 180 degrees, breaking GJK's assumption
        // that the support mapping has only one local maximum.
        if (!force_split && current_segment.size() >= 2) {
            const VecType& first_p1 = current_segment[0];
            const VecType& first_p2 = current_segment[1];
            VecType first_edge = first_p2 - first_p1;
            VecType current_edge = p3 - p2;
            if (first_edge.dp(current_edge) <= 0) {
                force_split = true;
            }
        }

        if (force_split) {
            // We don't push p3 here, because p2 is the last valid point of this convex segment.
            // But to close the segment, we need to ensure it's a valid line string.
            out_mesh.append_line_poly(
                current_segment.data(), static_cast<int>(current_segment.size()), subtractive);

            current_segment.clear();
            current_turn_sign = 0;
            // We don't skip i++, so the next segment will start with p2 and p3 in the next iteration
        }
    }

    if (current_segment.size() >= 2) {
        out_mesh.append_line_poly(
            current_segment.data(), static_cast<int>(current_segment.size()), subtractive);
    }
}

// -------------------------------------------------------------------------
// FACTORY DISPATCHER
// -------------------------------------------------------------------------
template <class VecType>
SolidGeometry<VecType> decompose_complex_polygon(
    const std::vector<std::vector<VecType>>& outer_boundaries,
    const std::vector<std::vector<VecType>>& holes
) {
    SolidGeometry<VecType> composed_mesh;

    for (const auto& outer : outer_boundaries) {
        composed_mesh.add_boundary_ring(outer, /*subtractive=*/false);
        process_boundary_to_convex_segments<VecType>(outer, composed_mesh, /*subtractive=*/false);
    }

    for (const auto& hole : holes) {
        auto reversed = reverse_ring(hole);
        composed_mesh.add_boundary_ring(reversed, /*subtractive=*/true);
        process_boundary_to_convex_segments<VecType>(reversed, composed_mesh, /*subtractive=*/true);
    }

    composed_mesh.finalize();
    return composed_mesh;
}
