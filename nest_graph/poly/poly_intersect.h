#pragma once

#include <vector>
#include <algorithm>
#include <cmath>
#include <utility>
#include <type_traits>

#include "poly.h"
#include "convex/intersect.h" // Updated to our new Line String GJK
#include "sweep.h"
#include "tracer.h"

// -------------------------------------------------------------------------
// DEFAULT TRACER ALIAS
// -------------------------------------------------------------------------
#ifndef NEST_GRAPH_DEFAULT_TRACER
#define NEST_GRAPH_DEFAULT_TRACER NullTracer
#endif
using DefaultTracer = NEST_GRAPH_DEFAULT_TRACER;

// -------------------------------------------------------------------------
// NARROW PHASE ROUTERS (Smart GJK Dispatch)
// -------------------------------------------------------------------------

template<class VecType>
inline bool narrow_phase_intersect(
    const VecType* lsA, int nA,
    const VecType* lsB, int nB,
    int GRADIENT_THRESHOLD = 24
) {
    // Sort to optimize cache utilization in GJK
    const VecType* p1 = (nA <= nB) ? lsA : lsB;
    int s1 = (nA <= nB) ? nA : nB;

    const VecType* p2 = (nA <= nB) ? lsB : lsA;
    int s2 = (nA <= nB) ? nB : nA;

    // Dispatch to the Line String gradient climbers
    if (s1 + s2 > GRADIENT_THRESHOLD) {
        return convex_linestrings_intersect_gjk_gradient<VecType>(p1, s1, p2, s2).intersect;
    } else {
        return convex_linestrings_intersect_gjk<VecType>(p1, s1, p2, s2).intersect;
    }
}

// -------------------------------------------------------------------------
// SOLID-SIDE CONTAINMENT CHECK (Raycast Method)
// -------------------------------------------------------------------------
// Since you know the orientation (e.g., solid mass is on the "Inside-Left"),
// a standard raycast perfectly handles both solid mass AND non-convex holes.
// Odd number of crossings = Point is inside the solid mass!
template<class VecType>
inline bool is_point_inside_solid_space(const VecType& pt, const Polygon<VecType>& poly) {
    using Scalar = typename VecType::Scalar;
    int crossings = 0;

    for (size_t part = 0; part < poly.line_parts.size(); ++part) {
        const VecType* pts = poly.get_part_points(part);
        int n = poly.get_part_size(part);

        for (int i = 0; i < n; ++i) {
            const VecType& v1 = pts[i];
            const VecType& v2 = pts[(i + 1) % n];

            // Horizontal raycast to the right (+x)
            if (((v1[1] > pt[1]) != (v2[1] > pt[1])) &&
                (pt[0] < (v2[0] - v1[0]) * (pt[1] - v1[1]) / (v2[1] - v1[1]) + v1[0])) {
                crossings++;
            }
        }
    }
    return (crossings % 2) != 0;
}

// -------------------------------------------------------------------------
// HELPERS & NARROW PHASE EVALUATION
// -------------------------------------------------------------------------

inline std::pair<int, int> make_sorted_pair(int a, int b) {
    return (a < b) ? std::make_pair(a, b) : std::make_pair(b, a);
}

template<class VecType, class Tracer = DefaultTracer>
inline bool check_part_vs_part(
    const Polygon<VecType>& polyA, int a_idx,
    const Polygon<VecType>& polyB, int b_idx,
    Tracer* tracer = nullptr
) {
    using Scalar = typename VecType::Scalar;
    // Updated to use line_parts
    const auto& circleA = polyA.line_parts[a_idx].bounding_circle;
    const auto& circleB = polyB.line_parts[b_idx].bounding_circle;

    Scalar dist_sq = (circleA.center() - circleB.center()).len_sq();
    Scalar r_sum = static_cast<Scalar>(std::sqrt(static_cast<double>(circleA.square_radius())))
        + static_cast<Scalar>(std::sqrt(static_cast<double>(circleB.square_radius())));

    if (dist_sq > r_sum * r_sum) {
        if constexpr (!std::is_same_v<Tracer, NullTracer>) {
            if (tracer) tracer->count_circle_prune();
        }
        return false;
    }

    const VecType* ptsA = polyA.get_part_points(a_idx);
    int nA = polyA.get_part_size(a_idx);

    const VecType* ptsB = polyB.get_part_points(b_idx);
    int nB = polyB.get_part_size(b_idx);

    if constexpr (!std::is_same_v<Tracer, NullTracer>) {
        if (tracer) tracer->count_gjk_eval();
    }

    // 1. Check if the line string boundaries cross
    if (narrow_phase_intersect<VecType>(ptsA, nA, ptsB, nB)) {
        if constexpr (!std::is_same_v<Tracer, NullTracer>) {
            if (tracer) tracer->record_penetration();
        }
        return true;
    }

    return false;
}

// -------------------------------------------------------------------------
// SWEEP ENGINE
// -------------------------------------------------------------------------
template<class VecType>
inline void append_poly_parts_to_sweep(
    int poly_idx,
    int group_id,
    const Polygon<VecType>& poly,
    const VecType& sweep_axis,
    typename VecType::Scalar axis_len_sqrt,
    std::vector<PartSweepElement<VecType>>& out_elements
) {
    using Scalar = typename VecType::Scalar;
    // Updated to use line_parts
    for (size_t part = 0; part < poly.line_parts.size(); ++part) {
        const auto& bounds = poly.line_parts[part].bounding_circle;
        Scalar proj = bounds.center().dp(sweep_axis);
        Scalar r = static_cast<Scalar>(std::sqrt(static_cast<double>(bounds.square_radius()))) * axis_len_sqrt;

        out_elements.push_back({poly_idx, static_cast<int>(part), group_id, proj - r, proj + r, &poly, &bounds});
    }
}

template<class VecType, class Tracer = DefaultTracer>
inline std::vector<std::pair<int, int>> execute_part_sweep(
    std::vector<PartSweepElement<VecType>>& elements,
    SweepMode mode = SweepMode::Monopartite,
    int bipartite_set_a_size = -1,
    Tracer* tracer = nullptr
) {
    std::vector<std::pair<int, int>> confirmed_collisions;
    if (elements.empty()) return confirmed_collisions;

    std::sort(elements.begin(), elements.end(), [](const auto& a, const auto& b) {
        return a.min_proj < b.min_proj;
    });

    std::vector<std::pair<int, int>> known_collisions;
    // We use this to track pairs that had overlapping bounding boxes,
    // but their boundaries didn't cross. We must check these for full containment later!
    std::vector<std::pair<int, int>> potential_containments;

    for (size_t i = 0; i < elements.size(); ++i) {
        for (size_t j = i + 1; j < elements.size(); ++j) {
            if (elements[j].min_proj > elements[i].max_proj) break;

            int group_i = elements[i].group_id;
            int group_j = elements[j].group_id;
            int pA_idx = elements[i].poly_idx;
            int pB_idx = elements[j].poly_idx;

            if (pA_idx == pB_idx) continue;

            std::pair<int, int> pair_id;

            if (mode == SweepMode::Bipartite) {
                if (group_i == group_j) continue;

                int idxA = (group_i == 0) ? pA_idx : pB_idx;
                int idxB = (group_i == 1) ? pA_idx : pB_idx;
                pair_id = std::make_pair(idxA, idxB);
                if (bipartite_set_a_size > 0) {
                    pair_id.second -= bipartite_set_a_size;
                }
            }
            else if (mode == SweepMode::Subset) {
                if (group_i == 0 && group_j == 0) continue;
                pair_id = make_sorted_pair(pA_idx, pB_idx);
            }
            else {
                pair_id = make_sorted_pair(pA_idx, pB_idx);
            }

            if constexpr (!std::is_same_v<Tracer, NullTracer>) {
                if (tracer) tracer->count_sweep_pair();
            }

            auto it = std::lower_bound(known_collisions.begin(), known_collisions.end(), pair_id);
            if (it != known_collisions.end() && *it == pair_id) {
                continue;
            }

            TracerScope<Tracer> scope(tracer, pair_id.first, pair_id.second);

            if (check_part_vs_part<VecType, Tracer>(*(elements[i].poly_ptr), elements[i].part_idx, *(elements[j].poly_ptr), elements[j].part_idx, tracer)) {
                known_collisions.insert(it, pair_id);
                confirmed_collisions.push_back(pair_id);
            } else {
                // If the bounding boxes overlapped in the sweep, but boundaries didn't cross,
                // we log them to check if one is fully swallowed by the other.
                potential_containments.push_back(pair_id);
            }
        }
    }

    // --- PHASE 2: RESOLVE CONTAINMENTS ---
    // Remove duplicates from potential containments
    std::sort(potential_containments.begin(), potential_containments.end());
    potential_containments.erase(std::unique(potential_containments.begin(), potential_containments.end()), potential_containments.end());

    for (const auto& pair : potential_containments) {
        // Skip if we already confirmed a collision via boundary crossing
        if (std::binary_search(known_collisions.begin(), known_collisions.end(), pair)) {
            continue;
        }

        // Re-acquire the polygons (You will need to pass the actual polygon array into
        // execute_part_sweep or handle this in the main engine entry points.
        // For simplicity, assuming we have access to polyA and polyB here).

        // Example check: Is the first point of Polygon A inside Polygon B?
        // if (is_point_inside_solid_space(polyA.get_part_points(0)[0], polyB) ||
        //     is_point_inside_solid_space(polyB.get_part_points(0)[0], polyA)) {
        //     confirmed_collisions.push_back(pair);
        // }
    }

    return confirmed_collisions;
}

// -------------------------------------------------------------------------
// MAIN ENGINE ENTRY POINTS (Identical Signature, uses updated sweep logic)
// -------------------------------------------------------------------------

template<class VecType, class Tracer = DefaultTracer>
std::vector<std::pair<int, int>> find_polygon_intersections(
    const std::vector<Polygon<VecType>>& polygons,
    Tracer* tracer = nullptr
) {
    using Scalar = typename VecType::Scalar;
    if (polygons.size() < 2) return {};

    VecType sweep_axis = compute_optimal_sweep_axis(polygons);

    std::vector<PartSweepElement<VecType>> elements;
    elements.reserve(polygons.size() * 4);

    Scalar axis_sq = sweep_axis.len_sq();
    if (axis_sq < static_cast<Scalar>(1e-8)) return {};
    Scalar axis_len_sqrt = static_cast<Scalar>(std::sqrt(static_cast<double>(axis_sq)));

    for (size_t i = 0; i < polygons.size(); ++i) {
        append_poly_parts_to_sweep(static_cast<int>(i), 0, polygons[i], sweep_axis, axis_len_sqrt, elements);
    }

    // Execute Sweep
    auto collisions = execute_part_sweep<VecType, Tracer>(elements, SweepMode::Monopartite, -1, tracer);

    // Apply the Final Containment Check
    // Because execute_part_sweep only tests boundary intersections, we must iterate through
    // the non-intersecting but overlapping bounding boxes.
    for (size_t i = 0; i < polygons.size(); ++i) {
        for (size_t j = i + 1; j < polygons.size(); ++j) {
            auto pair_id = make_sorted_pair(i, j);

            // Skip if they already collided
            if (std::find(collisions.begin(), collisions.end(), pair_id) != collisions.end()) continue;

            // Broad-phase containment check on the master bounding circles
            const auto& global_bcA = polygons[i].get_bounding_circle();
            const auto& global_bcB = polygons[j].get_bounding_circle();
            Scalar c_dist_sq = (global_bcA.center() - global_bcB.center()).len_sq();
            Scalar r_sum = static_cast<Scalar>(std::sqrt(global_bcA.square_radius())) +
                           static_cast<Scalar>(std::sqrt(global_bcB.square_radius()));

            if (c_dist_sq <= r_sum * r_sum) {
                // If bounding boxes overlap, check if A is inside B, or B is inside A
                if (polygons[i].line_parts.size() > 0 && polygons[j].line_parts.size() > 0) {
                    const VecType& ptA = polygons[i].get_part_points(0)[0];
                    const VecType& ptB = polygons[j].get_part_points(0)[0];

                    if (is_point_inside_solid_space(ptA, polygons[j]) ||
                        is_point_inside_solid_space(ptB, polygons[i])) {
                        collisions.push_back(pair_id);
                    }
                }
            }
        }
    }

    return collisions;
}