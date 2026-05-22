#pragma once

#include <cmath>
#include <utility>
#include <vector>

#include "solid_geometry.h"
#include "point_in_solid.h"
#include "geometry_common.h"

// PRECONDITION: convex inner ring and outer solid; inner/outer boundary segments do not cross.
// Caller must establish this (e.g. check_part_vs_part_intersect == false on this part pair).
// Do not infer from nesting alone: fully contained rings can still report GJK intersect=true.
// When the precondition holds and the inner ring is inside the outer solid, inner_pts[0]
// is inside; one point-in-solid test suffices.
template <class VecType>
inline bool narrow_phase_contain(
    const VecType* inner_pts,
    int inner_n,
    const SolidGeometry<VecType>& outer
) {
    if (inner_n <= 0) {
        return false;
    }
    return is_point_inside_solid_space(inner_pts[0], outer);
}

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
        if (narrow_phase_contain(
                polyA.get_part_points(p_idx),
                polyA.get_part_size(p_idx),
                polyB)) {
            return true;
        }
    }

    // Check if any point of B is inside A (additive parts only)
    for (size_t p_idx = 0; p_idx < polyB.line_parts.size(); ++p_idx) {
        if (polyB.line_parts[p_idx].is_subtractive) {
            continue;
        }
        if (narrow_phase_contain(
                polyB.get_part_points(p_idx),
                polyB.get_part_size(p_idx),
                polyA)) {
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
