#pragma once

#include <vector>
#include <cmath>
#include <algorithm>
#include <type_traits>

#include "solid/solid_geometry.h"
#include "common/geometry_common.h"
#include "convex/cast.h"


// -------------------------------------------------------------------------
// RESULT STRUCTURE
// -------------------------------------------------------------------------
template <class VecType>
struct ComplexCastResult {
    using Scalar = typename VecType::Scalar;
    int polyA_idx;
    int partA_idx;
    int polyB_idx;
    int partB_idx;

    bool intersects_path;
    Scalar t_entry; // Exact distance to kiss the boundary
    Scalar t_exit;  // Exact distance to completely pass through

    // Sort so closest impacts appear first
    bool operator<(const ComplexCastResult& other) const {
        if (intersects_path != other.intersects_path) return intersects_path; // true comes first
        return t_entry < other.t_entry;
    }
};

// -------------------------------------------------------------------------
// NARROW PHASE ROUTER
// -------------------------------------------------------------------------
template <class VecType, class Tracer = DefaultTracer>
inline ShapeCastResult<VecType> narrow_phase_cast(
    const VecType* lsA, int nA,
    const VecType* lsB, int nB,
    const VecType& slide_vector,
    int GRADIENT_THRESHOLD = 24,
    Tracer* tracer = nullptr
) {
    if constexpr (!std::is_same_v<Tracer, NullTracer>) {
        if (tracer) tracer->record_cast(); // Track engine load if desired
    }

    // Sort arrays to favor gradient descent on the larger polygon
    const bool swapped = (nA > nB);
    const VecType* p1 = swapped ? lsB : lsA;
    const int s1 = swapped ? nB : nA;
    const VecType* p2 = swapped ? lsA : lsB;
    const int s2 = swapped ? nA : nB;

    // CRITICAL: If we swap the polygons, Shape A moving by V against Shape B
    // is mathematically identical to Shape B moving by -V against Shape A.
    VecType active_slide = swapped ? -slide_vector : slide_vector;

    if (s1 + s2 > GRADIENT_THRESHOLD) {
        return convex_linestrings_cast_gradient<VecType>(p1, s1, p2, s2, active_slide);
    }
    return convex_linestrings_cast<VecType>(p1, s1, p2, s2, active_slide);
}

// -------------------------------------------------------------------------
// BROAD PHASE CULLING (Moving Circle vs Static Circle)
// -------------------------------------------------------------------------
template <class VecType>
inline bool moving_circle_intersects_static_circle(
    const Circle<VecType>& moving_circ,
    const VecType& slide_vector,
    const Circle<VecType>& static_circ,
    typename VecType::Scalar max_t // Dynamic limit for culling
) {
    using Scalar = typename VecType::Scalar;

    VecType CA = moving_circ.center();
    VecType CB = static_circ.center();
    VecType AB = CB - CA;

    Scalar v_sq = slide_vector.len_sq();
    Scalar t = 0;

    // Find the closest point on the line segment [CA, CA + (slide_vector * max_t)] to CB
    if (v_sq > 1e-8) {
        t = std::max(static_cast<Scalar>(0.0), std::min(max_t, AB.dp(slide_vector) / v_sq));
    }

    VecType closest_pt = CA + (slide_vector * t);
    Scalar dist_sq = (CB - closest_pt).len_sq();

    Scalar rA = std::sqrt(moving_circ.square_radius());
    Scalar rB = std::sqrt(static_circ.square_radius());
    Scalar combined_rad = rA + rB;

    return dist_sq <= (combined_rad * combined_rad);
}

// -------------------------------------------------------------------------
// MAIN ENGINE ENTRY POINTS
// -------------------------------------------------------------------------

// API 1: THE GUIDER HELPER (Finds ONLY the closest impact point)
// Use this when sliding a piece into a corner to dock it perfectly.
template <class VecType, class Tracer = DefaultTracer>
ComplexCastResult<VecType> find_closest_polygon_cast(
    int active_poly_idx,
    const std::vector<SolidGeometry<VecType>>& polygons,
    const VecType& slide_vector,
    typename VecType::Scalar max_t_limit = std::numeric_limits<typename VecType::Scalar>::max(),
    Tracer* tracer = nullptr
) {
    using Scalar = typename VecType::Scalar;

    ComplexCastResult<VecType> closest_hit;
    closest_hit.intersects_path = false;
    closest_hit.t_entry = max_t_limit;

    if (active_poly_idx < 0 || active_poly_idx >= polygons.size()) return closest_hit;

    const auto& active_poly = polygons[active_poly_idx];
    const auto& active_bounds = active_poly.get_bounding_circle();

    for (size_t i = 0; i < polygons.size(); ++i) {
        if (i == static_cast<size_t>(active_poly_idx)) continue;
        const auto& obstacle = polygons[i];

        // 1. Broad-Phase Capsule Culling (Dynamically shrinks as we find closer hits)
        if (!moving_circle_intersects_static_circle(active_bounds, slide_vector, obstacle.get_bounding_circle(), closest_hit.t_entry)) {
            continue;
        }

        // 2. Part-by-Part Evaluation
        for (size_t pA = 0; pA < active_poly.line_parts.size(); ++pA) {
            for (size_t pB = 0; pB < obstacle.line_parts.size(); ++pB) {

                if (!moving_circle_intersects_static_circle(
                    active_poly.line_parts[pA].bounding_circle, slide_vector,
                    obstacle.line_parts[pB].bounding_circle, closest_hit.t_entry)) {
                    continue;
                }

                auto res = narrow_phase_cast<VecType, Tracer>(
                    active_poly.get_part_points(pA), active_poly.get_part_size(pA),
                    obstacle.get_part_points(pB), obstacle.get_part_size(pB),
                    slide_vector, 24, tracer
                );

                // If it hits, and it's CLOSER than our previous best record
                if (res.intersects_path && res.t_entry < closest_hit.t_entry) {
                    closest_hit.intersects_path = true;
                    closest_hit.t_entry = res.t_entry;
                    closest_hit.t_exit = res.t_exit;
                    closest_hit.polyA_idx = active_poly_idx;
                    closest_hit.partA_idx = pA;
                    closest_hit.polyB_idx = i;
                    closest_hit.partB_idx = pB;
                }
            }
        }
    }

    return closest_hit;
}

// API 2: THE FULL SWEEP (Returns ALL boundary intersections along the vector)
// Useful for analyzing "How many walls will I punch through if I fire this ray?"
template <class VecType, class Tracer = DefaultTracer>
std::vector<ComplexCastResult<VecType>> find_all_polygon_casts(
    int active_poly_idx,
    const std::vector<SolidGeometry<VecType>>& polygons,
    const VecType& slide_vector,
    typename VecType::Scalar max_t_limit = std::numeric_limits<typename VecType::Scalar>::max(),
    Tracer* tracer = nullptr
) {
    std::vector<ComplexCastResult<VecType>> results;
    if (active_poly_idx < 0 || active_poly_idx >= polygons.size()) return results;

    const auto& active_poly = polygons[active_poly_idx];
    const auto& active_bounds = active_poly.get_bounding_circle();

    for (size_t i = 0; i < polygons.size(); ++i) {
        if (i == static_cast<size_t>(active_poly_idx)) continue;
        const auto& obstacle = polygons[i];

        if (!moving_circle_intersects_static_circle(active_bounds, slide_vector, obstacle.get_bounding_circle(), max_t_limit)) {
            continue;
        }

        for (size_t pA = 0; pA < active_poly.line_parts.size(); ++pA) {
            for (size_t pB = 0; pB < obstacle.line_parts.size(); ++pB) {

                if (!moving_circle_intersects_static_circle(
                    active_poly.line_parts[pA].bounding_circle, slide_vector,
                    obstacle.line_parts[pB].bounding_circle, max_t_limit)) {
                    continue;
                }

                auto res = narrow_phase_cast<VecType, Tracer>(
                    active_poly.get_part_points(pA), active_poly.get_part_size(pA),
                    obstacle.get_part_points(pB), obstacle.get_part_size(pB),
                    slide_vector, 24, tracer
                );

                if (res.intersects_path && res.t_entry <= max_t_limit) {
                    results.push_back({
                        active_poly_idx, static_cast<int>(pA),
                        static_cast<int>(i), static_cast<int>(pB),
                        true, res.t_entry, res.t_exit
                    });
                }
            }
        }
    }

    std::sort(results.begin(), results.end());
    return results;
}
