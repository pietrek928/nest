#pragma once

#include <cmath>
#include <utility>
#include <vector>

#include "solid_geometry.h"
#include "point_in_solid.h"
#include "geometry_common.h"

// Fallback when narrow-phase found no boundary crossings between any additive parts.
// Uses the first vertex of each additive line part as an inside witness (holes skipped).
// Sufficient for "A fully inside B, boundaries disjoint" after part-vs-part GJK misses.
template <class VecType>
inline bool check_mutual_containment(
    const SolidGeometry<VecType>& polyA,
    const SolidGeometry<VecType>& polyB
) {
    if (polyA.line_parts.empty() || polyB.line_parts.empty()) {
        return false;
    }

    // Check if any point of A is inside B (additive parts only)
    for (size_t p_idx = 0; p_idx < polyA.line_parts.size(); ++p_idx) {
        if (polyA.line_parts[p_idx].is_subtractive) {
            continue;
        }
        const VecType& ptA = polyA.get_part_points(p_idx)[0];
        if (is_point_inside_solid_space(ptA, polyB)) {
            return true;
        }
    }

    // Check if any point of B is inside A (additive parts only)
    for (size_t p_idx = 0; p_idx < polyB.line_parts.size(); ++p_idx) {
        if (polyB.line_parts[p_idx].is_subtractive) {
            continue;
        }
        const VecType& ptB = polyB.get_part_points(p_idx)[0];
        if (is_point_inside_solid_space(ptB, polyA)) {
            return true;
        }
    }

    return false;
}

template <class VecType>
inline bool global_bounding_circles_overlap(
    const SolidGeometry<VecType>& polyA,
    const SolidGeometry<VecType>& polyB
) {
    return circles_overlap(polyA.get_bounding_circle(), polyB.get_bounding_circle());
}

template <class VecType>
inline bool try_add_containment_collision(
    const SolidGeometry<VecType>& polyA,
    const SolidGeometry<VecType>& polyB
) {
    if (!global_bounding_circles_overlap(polyA, polyB)) {
        return false;
    }
    return check_mutual_containment(polyA, polyB);
}

template <class VecType>
inline void promote_containment_pairs(
    const std::vector<SolidGeometry<VecType>>& polygons,
    const std::vector<std::pair<int, int>>& potential_containments,
    std::vector<std::pair<int, int>>& collisions
) {
    for (const auto& pair_id : potential_containments) {
        const int i = pair_id.first;
        const int j = pair_id.second;
        if (try_add_containment_collision(polygons[i], polygons[j])) {
            collisions.push_back(pair_id);
        }
    }
}

template <class VecType>
inline void promote_containment_pairs_bipartite(
    const std::vector<SolidGeometry<VecType>>& setA,
    const std::vector<SolidGeometry<VecType>>& setB,
    const std::vector<std::pair<int, int>>& potential_containments,
    std::vector<std::pair<int, int>>& collisions
) {
    for (const auto& pair_id : potential_containments) {
        const int i = pair_id.first;
        const int j = pair_id.second;
        if (try_add_containment_collision(setA[i], setB[j])) {
            collisions.push_back(pair_id);
        }
    }
}
