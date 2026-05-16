#pragma once

#include <cmath>
#include <vector>

#include "poly.h"


enum class SweepMode {
    Monopartite, // All vs All (Skips self-collisions)
    Bipartite,   // Set A vs Set B (Skips intra-set collisions)
    Subset       // Active vs All (Skips Static vs Static)
};

template<class VecType>
struct PartSweepElement {
    int poly_idx;          // Original index in the user's vector
    int part_idx;          // Which convex part of the polygon this is
    int group_id;          // 0 = Set A, 1 = Set B (used for bipartite checks)

    typename VecType::Scalar min_proj;
    typename VecType::Scalar max_proj;

    const SolidGeometry<VecType>* poly_ptr; // NEW: Decouples element from its source vector
    const Circle<VecType>* bounds;
};

template<class VecType>
inline VecType compute_optimal_sweep_axis(const std::vector<SolidGeometry<VecType>>& polygons) {
    using Scalar = typename VecType::Scalar;

    VecType optimal_axis{}; // Zero-initializes all N dimensions safely

    if (polygons.empty() || polygons[0].line_parts.empty()) {
        optimal_axis.get_(0) = static_cast<Scalar>(1); // Safe fallback to primary axis
        return optimal_axis;
    }

    // 1. Initialize bounds using the very first circle center
    VecType min_bounds = polygons[0].line_parts[0].bounding_circle.center();
    VecType max_bounds = min_bounds;

    // 2. Expand bounds in N-dimensions using your vector ops
    for (const auto& poly : polygons) {
        for (const auto& part : poly.line_parts) {
            const auto& c = part.bounding_circle.center();
            min_bounds = min_bounds.min(c);
            max_bounds = max_bounds.max(c);
        }
    }

    // 3. Find the axis of maximum variance
    VecType spread = max_bounds - min_bounds;

    int best_axis = 0;
    Scalar max_spread = spread[0];

    for (int i = 1; i < VecType::dim; ++i) {
        if (spread[i] > max_spread) {
            max_spread = spread[i];
            best_axis = i;
        }
    }

    // Construct the normalized sweep axis
    optimal_axis.get_(best_axis) = static_cast<Scalar>(1);
    return optimal_axis;
}

template<class VecType>
inline VecType compute_optimal_sweep_axis(
    const std::vector<SolidGeometry<VecType>>& set_a,
    const std::vector<SolidGeometry<VecType>>& set_b
) {
    std::vector<SolidGeometry<VecType>> combined;
    combined.reserve(set_a.size() + set_b.size());
    combined.insert(combined.end(), set_a.begin(), set_a.end());
    combined.insert(combined.end(), set_b.begin(), set_b.end());
    return compute_optimal_sweep_axis(combined);
}
