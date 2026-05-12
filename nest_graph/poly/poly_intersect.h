#pragma once

#include <vector>
#include <algorithm>
#include <cmath>
#include <utility>

#include "circle.h"
#include "poly.h"
#include "convex/intersect.h"
#include "convex/contain.h"

// -------------------------------------------------------------------------
// NARROW PHASE ROUTERS (Smart GJK Dispatch)
// -------------------------------------------------------------------------
const int GRADIENT_THRESHOLD = 24;

template<class VecType>
inline bool narrow_phase_intersect(
    const VecType* polyA, int nA,
    const VecType* polyB, int nB
) {
    // Ensure smaller polygon is always first for cache efficiency
    const VecType* p1 = (nA <= nB) ? polyA : polyB;
    int s1 = (nA <= nB) ? nA : nB;

    const VecType* p2 = (nA <= nB) ? polyB : polyA;
    int s2 = (nA <= nB) ? nB : nA;

    if (s1 + s2 > GRADIENT_THRESHOLD) {
        return convex_polygons_intersect_gjk_gradient<VecType>(p1, s1, p2, s2).intersect;
    } else {
        return convex_polygons_intersect_gjk<VecType>(p1, s1, p2, s2).intersect;
    }
}

template<class VecType>
inline bool narrow_phase_contain(
    const VecType* polyA, int nA,
    const VecType* polyB, int nB
) {
    if (nA + nB > GRADIENT_THRESHOLD) {
        return is_polygon_fully_inside_gradient<VecType>(polyA, nA, polyB, nB);
    } else {
        return is_polygon_fully_inside<VecType>(polyA, nA, polyB, nB);
    }
}

// -------------------------------------------------------------------------
// STRUCTURES & HELPERS
// -------------------------------------------------------------------------
template<class VecType>
struct PartSweepElement {
    using Scalar = typename VecType::Scalar;

    int poly_idx;
    int part_idx;
    Scalar min_proj;
    Scalar max_proj;
    const Circle<VecType>* bounds;
};

inline std::pair<int, int> make_sorted_pair(int a, int b) {
    return (a < b) ? std::make_pair(a, b) : std::make_pair(b, a);
}

// -------------------------------------------------------------------------
// INVALIDATION & PAIR CHECKS
// -------------------------------------------------------------------------
template<class VecType>
inline bool is_invalidated_by_hole(
    const VecType* ptsA, int nA, const Polygon<VecType>& polyA,
    const VecType* ptsB, int nB, const Polygon<VecType>& polyB
) {
    // Check if Part A is entirely swallowed by a hole in Poly B
    for (size_t hb = 0; hb < polyB.convex_holes.size(); ++hb) {
        if (narrow_phase_contain<VecType>(ptsA, nA, polyB.get_hole_points(hb), polyB.get_hole_size(hb))) {
            return true;
        }
    }

    // Check if Part B is entirely swallowed by a hole in Poly A
    for (size_t ha = 0; ha < polyA.convex_holes.size(); ++ha) {
        if (narrow_phase_contain<VecType>(ptsB, nB, polyA.get_hole_points(ha), polyA.get_hole_size(ha))) {
            return true;
        }
    }

    return false;
}

template<class VecType>
inline bool check_part_vs_part(
    const Polygon<VecType>& polyA, int a_idx,
    const Polygon<VecType>& polyB, int b_idx
) {
    using Scalar = typename VecType::Scalar;
    // Tertiary Broad-Phase: Circle Overlap
    const auto& circleA = polyA.convex_parts[a_idx].bounding_circle;
    const auto& circleB = polyB.convex_parts[b_idx].bounding_circle;

    Scalar dist_sq = (circleA.center() - circleB.center()).len_sq();
    Scalar r_sum = static_cast<Scalar>(std::sqrt(static_cast<double>(circleA.square_radius())))
        + static_cast<Scalar>(std::sqrt(static_cast<double>(circleB.square_radius())));

    if (dist_sq > r_sum * r_sum) return false;

    // Narrow Phase: GJK
    const VecType* ptsA = polyA.get_part_points(a_idx);
    int nA = polyA.get_part_size(a_idx);

    const VecType* ptsB = polyB.get_part_points(b_idx);
    int nB = polyB.get_part_size(b_idx);

    if (!narrow_phase_intersect<VecType>(ptsA, nA, ptsB, nB)) return false;

    // Hole Invalidation Check
    return !is_invalidated_by_hole<VecType>(ptsA, nA, polyA, ptsB, nB, polyB);
}

// -------------------------------------------------------------------------
// SWEEP ENGINE
// -------------------------------------------------------------------------
template<class VecType>
inline void append_poly_parts_to_sweep(
    int poly_idx,
    const Polygon<VecType>& poly,
    const VecType& sweep_axis,
    typename VecType::Scalar axis_len_sqrt,
    std::vector<PartSweepElement<VecType>>& out_elements
) {
    using Scalar = typename VecType::Scalar;
    for (size_t part = 0; part < poly.convex_parts.size(); ++part) {
        const auto& bounds = poly.convex_parts[part].bounding_circle;
        Scalar proj = bounds.center().dp(sweep_axis);
        Scalar r = static_cast<Scalar>(std::sqrt(static_cast<double>(bounds.square_radius()))) * axis_len_sqrt;

        out_elements.push_back({poly_idx, static_cast<int>(part), proj - r, proj + r, &bounds});
    }
}

template<class VecType>
inline std::vector<std::pair<int, int>> execute_part_sweep(
    const std::vector<Polygon<VecType>>& polygons,
    std::vector<PartSweepElement<VecType>>& elements
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

            // OPTIMIZATION: O(log K) binary search instead of O(K) linear search
            auto it = std::lower_bound(known_collisions.begin(), known_collisions.end(), pair_id);
            if (it != known_collisions.end() && *it == pair_id) {
                continue; // Already confirmed collision
            }

            // Narrow Phase: Circle -> GJK -> Holes
            if (check_part_vs_part(polygons[polyA_idx], elements[i].part_idx, polygons[polyB_idx], elements[j].part_idx)) {
                // Insert maintaining sorted order
                known_collisions.insert(it, pair_id);
                confirmed_collisions.push_back(pair_id);
            }
        }
    }
    return confirmed_collisions;
}

// -------------------------------------------------------------------------
// MAIN ENGINE ENTRY POINTS
// -------------------------------------------------------------------------
template<class VecType>
std::vector<std::pair<int, int>> find_polygon_intersections(
    const std::vector<Polygon<VecType>>& polygons,
    const VecType& sweep_axis
) {
    using Scalar = typename VecType::Scalar;
    if (polygons.size() < 2) return {};

    std::vector<PartSweepElement<VecType>> elements;
    elements.reserve(polygons.size() * 4);

    Scalar axis_sq = sweep_axis.len_sq();
    if (axis_sq < static_cast<Scalar>(1e-8)) return {};

    Scalar axis_len_sqrt = static_cast<Scalar>(std::sqrt(static_cast<double>(axis_sq)));

    for (size_t i = 0; i < polygons.size(); ++i) {
        append_poly_parts_to_sweep(static_cast<int>(i), polygons[i], sweep_axis, axis_len_sqrt, elements);
    }

    return execute_part_sweep(polygons, elements);
}

template<class VecType>
std::vector<std::pair<int, int>> find_polygon_intersections(
    const std::vector<Polygon<VecType>>& polygons,
    const std::vector<int>& active_indices,
    const VecType& sweep_axis
) {
    using Scalar = typename VecType::Scalar;
    if (active_indices.size() < 2) return {};

    std::vector<PartSweepElement<VecType>> elements;
    elements.reserve(active_indices.size() * 4);

    Scalar axis_sq = sweep_axis.len_sq();
    if (axis_sq < static_cast<Scalar>(1e-8)) return {};

    Scalar axis_len_sqrt = static_cast<Scalar>(std::sqrt(static_cast<double>(axis_sq)));

    for (int p_idx : active_indices) {
        append_poly_parts_to_sweep(p_idx, polygons[p_idx], sweep_axis, axis_len_sqrt, elements);
    }

    return execute_part_sweep(polygons, elements);
}
