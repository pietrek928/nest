#pragma once

#include <vector>
#include <algorithm>
#include <cmath>

#include "circle.h"
#include "poly.h"
#include "convex_intersect.h"
#include "convex_contain.h"


// -------------------------------------------------------------------------
// NARROW PHASE: Smart GJK Router
// -------------------------------------------------------------------------
template<class T, class VecType>
inline bool narrow_phase_intersect(
    const VecType* polyA, int nA,
    const VecType* polyB, int nB
) {
    const VecType* p1 = (nA <= nB) ? polyA : polyB;
    int s1 = (nA <= nB) ? nA : nB;

    const VecType* p2 = (nA <= nB) ? polyB : polyA;
    int s2 = (nA <= nB) ? nB : nA;

    const int GRADIENT_THRESHOLD = 24;

    if (s1 + s2 > GRADIENT_THRESHOLD) {
        return convex_polygons_intersect_gjk_gradient<T, VecType>(p1, s1, p2, s2).intersect;
    } else {
        return convex_polygons_intersect_gjk<T, VecType>(p1, s1, p2, s2).intersect;
    }
}


// -------------------------------------------------------------------------
// STRUCTURES
// -------------------------------------------------------------------------

template<class T, class VecType>
struct SweepElement {
    int poly_idx;
    T min_proj;
    T max_proj;
    const Circle<T, VecType>* bounds;
};

// Unified Element for both 2-Poly Sweep and N-Poly Cluster Sweep
template<class T, class VecType>
struct PartSweepElement {
    int poly_idx;
    int part_idx;
    T min_proj;
    T max_proj;
    const Circle<T, VecType>* bounds;
};

// Helper to generate a unique pair ID (smaller index first)
inline std::pair<int, int> make_sorted_pair(int a, int b) {
    return (a < b) ? std::make_pair(a, b) : std::make_pair(b, a);
}

// Helper 1: Verify if a confirmed collision is invalidated by a hole
template<class T, class VecType>
inline bool is_invalidated_by_hole(
    const VecType* ptsA, int nA, const Polygon<T, VecType>& polyA,
    const VecType* ptsB, int nB, const Polygon<T, VecType>& polyB
) {
    // Check if Part A is entirely swallowed by a hole in Poly B
    for (size_t hb = 0; hb < polyB.convex_holes.size(); ++hb) {
        if (is_polygon_fully_inside<T, VecType>(ptsA, nA, polyB.get_hole_points(hb), polyB.get_hole_size(hb))) {
            return true;
        }
    }

    // Check if Part B is entirely swallowed by a hole in Poly A
    for (size_t ha = 0; ha < polyA.convex_holes.size(); ++ha) {
        if (is_polygon_fully_inside<T, VecType>(ptsB, nB, polyA.get_hole_points(ha), polyA.get_hole_size(ha))) {
            return true;
        }
    }

    return false;
}

// -------------------------------------------------------------------------
// CORE LOGIC: Check a specific Convex Part vs another Convex Part
// -------------------------------------------------------------------------
template<class T, class VecType>
inline bool check_part_vs_part(
    const Polygon<T, VecType>& polyA, int a_idx,
    const Polygon<T, VecType>& polyB, int b_idx
) {
    // Tertiary Broad-Phase: Circle Overlap
    const auto& circleA = polyA.convex_parts[a_idx].bounding_circle;
    const auto& circleB = polyB.convex_parts[b_idx].bounding_circle;

    T dist_sq = (circleA.center() - circleB.center()).len_sq();
    T r_sum = std::sqrt(circleA.square_radius()) + std::sqrt(circleB.square_radius());

    if (dist_sq > r_sum * r_sum) return false;

    // Narrow Phase: GJK
    const VecType* ptsA = polyA.get_part_points(a_idx);
    int nA = polyA.get_part_size(a_idx);

    const VecType* ptsB = polyB.get_part_points(b_idx);
    int nB = polyB.get_part_size(b_idx);

    if (!narrow_phase_intersect<T, VecType>(ptsA, nA, ptsB, nB)) return false;

    // Hole Invalidation Check
    return !is_invalidated_by_hole<T, VecType>(ptsA, nA, polyA, ptsB, nB, polyB);
}

// -------------------------------------------------------------------------
// THE EXTRACTOR: Flattens a Polygon's parts into the global sweep array
// -------------------------------------------------------------------------
template<class T, class VecType>
inline void append_poly_parts_to_sweep(
    int poly_idx,
    const Polygon<T, VecType>& poly,
    const VecType& sweep_axis,
    T axis_len_sqrt,
    std::vector<PartSweepElement<T, VecType>>& out_elements
) {
    for (size_t part = 0; part < poly.convex_parts.size(); ++part) {
        const auto& bounds = poly.convex_parts[part].bounding_circle;
        T proj = bounds.center().dp(sweep_axis);
        T r = std::sqrt(bounds.square_radius()) * axis_len_sqrt;

        out_elements.push_back({poly_idx, static_cast<int>(part), proj - r, proj + r, &bounds});
    }
}

// -------------------------------------------------------------------------
// CORE SWEEP EXECUTION: The Pure Data Router
// -------------------------------------------------------------------------
template<class T, class VecType>
inline std::vector<std::pair<int, int>> execute_part_sweep(
    const std::vector<Polygon<T, VecType>>& polygons,
    std::vector<PartSweepElement<T, VecType>>& elements
) {
    std::vector<std::pair<int, int>> confirmed_collisions;
    if (elements.empty()) return confirmed_collisions;

    // 1. Sort globally by minimum projection
    std::sort(elements.begin(), elements.end(), [](const auto& a, const auto& b) {
        return a.min_proj < b.min_proj;
    });

    std::vector<std::pair<int, int>> known_collisions;

    // 2. Sweep all convex parts
    for (size_t i = 0; i < elements.size(); ++i) {
        for (size_t j = i + 1; j < elements.size(); ++j) {
            if (elements[j].min_proj > elements[i].max_proj) break; // Prune

            int polyA_idx = elements[i].poly_idx;
            int polyB_idx = elements[j].poly_idx;

            if (polyA_idx == polyB_idx) continue; // Ignore self-collisions

            auto pair_id = make_sorted_pair(polyA_idx, polyB_idx);

            // Skip if we already confirmed this Polygon-to-Polygon collision
            if (std::find(known_collisions.begin(), known_collisions.end(), pair_id) != known_collisions.end()) {
                continue;
            }

            // Narrow Phase: Circle -> GJK -> Holes
            if (check_part_vs_part(polygons[polyA_idx], elements[i].part_idx, polygons[polyB_idx], elements[j].part_idx)) {
                known_collisions.push_back(pair_id);
                confirmed_collisions.push_back(pair_id);
            }
        }
    }
    return confirmed_collisions;
}


// -------------------------------------------------------------------------
// MAIN ENGINE ENTRY POINT 1: Sweep ALL Polygons
// -------------------------------------------------------------------------
template<class T, class VecType>
std::vector<std::pair<int, int>> find_polygon_intersections(
    const std::vector<Polygon<T, VecType>>& polygons,
    const VecType& sweep_axis
) {
    if (polygons.size() < 2) return {};

    std::vector<PartSweepElement<T, VecType>> elements;
    elements.reserve(polygons.size() * 4); // Estimate 4 parts per polygon

    // Safety check for axis length to avoid division/scaling by zero later
    T axis_sq = sweep_axis.len_sq();
    if (axis_sq < static_cast<T>(1e-8)) return {};

    T axis_len_sqrt = std::sqrt(axis_sq);

    for (size_t i = 0; i < polygons.size(); ++i) {
        append_poly_parts_to_sweep(
            static_cast<int>(i), polygons[i], sweep_axis, axis_len_sqrt, elements
        );
    }

    return execute_part_sweep(polygons, elements);
}

// -------------------------------------------------------------------------
// MAIN ENGINE ENTRY POINT 2: Sweep ONLY Active Indices
// -------------------------------------------------------------------------
template<class T, class VecType>
std::vector<std::pair<int, int>> find_polygon_intersections(
    const std::vector<Polygon<T, VecType>>& polygons,
    const std::vector<int>& active_indices,
    const VecType& sweep_axis
) {
    if (active_indices.size() < 2) return {};

    std::vector<PartSweepElement<T, VecType>> elements;
    elements.reserve(active_indices.size() * 4);

    T axis_sq = sweep_axis.len_sq();
    if (axis_sq < static_cast<T>(1e-8)) return {};

    T axis_len_sqrt = std::sqrt(axis_sq);

    for (int p_idx : active_indices) {
        append_poly_parts_to_sweep(
            p_idx, polygons[p_idx], sweep_axis, axis_len_sqrt, elements
        );
    }

    return execute_part_sweep(polygons, elements);
}