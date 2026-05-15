#pragma once

#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>

#include "decompose_simple.h"


// -------------------------------------------------------------------------
// MATH & VISIBILITY HELPERS
// -------------------------------------------------------------------------
template <class VecType>
inline typename VecType::Scalar ccw(const VecType& a, const VecType& b, const VecType& c) {
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
}

template <class VecType>
inline bool segments_intersect(const VecType& p1, const VecType& p2, const VecType& p3, const VecType& p4) {
    using Scalar = typename VecType::Scalar;
    // Standard segment intersection using cross products
    auto dir1 = ccw(p3, p4, p1);
    auto dir2 = ccw(p3, p4, p2);
    auto dir3 = ccw(p1, p2, p3);
    auto dir4 = ccw(p1, p2, p4);

    // If signs are different, they strictly intersect
    if (((dir1 > 0 && dir2 < 0) || (dir1 < 0 && dir2 > 0)) &&
        ((dir3 > 0 && dir4 < 0) || (dir3 < 0 && dir4 > 0))) {
        return true;
    }
    return false; // Ignoring collinear touch cases as bridges connect to vertices safely
}

template <class VecType>
inline bool is_visible(const VecType& p1, const VecType& p2, const std::vector<VecType>& poly, const std::vector<VecType>& hole) {
    // Check against all outer polygon edges
    for (size_t i = 0; i < poly.size(); ++i) {
        size_t next = (i + 1) % poly.size();
        if (poly[i] == p1 || poly[i] == p2 || poly[next] == p1 || poly[next] == p2) continue;
        if (segments_intersect(p1, p2, poly[i], poly[next])) return false;
    }
    // Check against all hole edges
    for (size_t i = 0; i < hole.size(); ++i) {
        size_t next = (i + 1) % hole.size();
        if (hole[i] == p1 || hole[i] == p2 || hole[next] == p1 || hole[next] == p2) continue;
        if (segments_intersect(p1, p2, hole[i], hole[next])) return false;
    }
    return true;
}

// -------------------------------------------------------------------------
// BRIDGE BUILDER
// -------------------------------------------------------------------------
template <class VecType>
std::vector<VecType> merge_hole_into_polygon(std::vector<VecType> outer, const std::vector<VecType>& hole) {
    using Scalar = typename VecType::Scalar;
    if (hole.empty()) return outer;

    // 1. Find the right-most vertex of the hole
    int h_idx = 0;
    Scalar max_x = std::numeric_limits<Scalar>::lowest();
    for (size_t i = 0; i < hole.size(); ++i) {
        if (hole[i][0] > max_x) {
            max_x = hole[i][0];
            h_idx = static_cast<int>(i);
        }
    }
    const VecType& H = hole[h_idx];

    // 2. Find the closest mutually visible vertex on the outer boundary
    int best_outer_idx = -1;
    Scalar min_dist_sq = std::numeric_limits<Scalar>::max();

    for (size_t i = 0; i < outer.size(); ++i) {
        const VecType& P = outer[i];

        // Quick heuristic: the bridge usually goes to the right, but we check all visible
        Scalar dist_sq = (P - H).len_sq();
        if (dist_sq < min_dist_sq) {
            if (is_visible(H, P, outer, hole)) {
                min_dist_sq = dist_sq;
                best_outer_idx = static_cast<int>(i);
            }
        }
    }

    // Fallback if visibility fails due to floating point edge cases (rare)
    if (best_outer_idx == -1) best_outer_idx = 0;

    // 3. Splice the arrays together
    std::vector<VecType> merged;
    merged.reserve(outer.size() + hole.size() + 2); // +2 for the back-and-forth bridge

    // Push outer up to the bridge point
    for (int i = 0; i <= best_outer_idx; ++i) merged.push_back(outer[i]);

    // Push the hole starting from the bridge connection, wrapping around
    for (size_t i = 0; i < hole.size(); ++i) {
        merged.push_back(hole[(h_idx + i) % hole.size()]);
    }

    // Close the hole by pushing the connection point again
    merged.push_back(hole[h_idx]);

    // Travel back across the bridge to the outer boundary
    merged.push_back(outer[best_outer_idx]);

    // Finish pushing the rest of the outer boundary
    for (size_t i = best_outer_idx + 1; i < outer.size(); ++i) {
        merged.push_back(outer[i]);
    }

    return merged;
}

// -------------------------------------------------------------------------
// MAIN PIPELINE
// -------------------------------------------------------------------------
template <class VecType>
std::vector<std::vector<VecType>> decompose_complex_polygon(
    std::vector<VecType> outer_boundary,
    std::vector<std::vector<VecType>> holes
) {
    // 1. Sort holes by their max X coordinate so we process from right to left.
    // This ensures that bridges built for earlier holes don't block later holes.
    std::sort(holes.begin(), holes.end(), [](const std::vector<VecType>& a, const std::vector<VecType>& b) {
        auto max_x_a = a[0][0], max_x_b = b[0][0];
        for(const auto& v : a) if(v[0] > max_x_a) max_x_a = v[0];
        for(const auto& v : b) if(v[0] > max_x_b) max_x_b = v[0];
        return max_x_a > max_x_b;
    });

    // 2. Iteratively merge all holes into the outer boundary
    for (const auto& hole : holes) {
        outer_boundary = merge_hole_into_polygon(outer_boundary, hole);
    }

    // 3. Now that it is a single simple polygon (with zero-width bridges),
    // run it through your existing Triangulation & Hertel-Mehlhorn optimizer.
    // (Assuming decompose_to_convex_optimal is included from convex_decomposition.h)
    return decompose_to_convex_optimal(outer_boundary);
}
