#pragma once

#include <vector>
#include <algorithm>
#include <cmath>
#include <utility>
#include <type_traits>

#include "poly.h"
#include "common_checks.h"
#include "convex/intersect.h"
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
    const VecType* polyA, int nA,
    const VecType* polyB, int nB,
    int GRADIENT_THRESHOLD = 24
) {
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
    const auto& circleA = polyA.convex_parts[a_idx].bounding_circle;
    const auto& circleB = polyB.convex_parts[b_idx].bounding_circle;

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

    if (!narrow_phase_intersect<VecType>(ptsA, nA, ptsB, nB)) return false;

    bool hole_invalid = is_invalidated_by_hole<VecType>(ptsA, nA, polyA, ptsB, nB, polyB);

    if (hole_invalid) {
        if constexpr (!std::is_same_v<Tracer, NullTracer>) {
            if (tracer) tracer->count_hole_invalidation();
        }
    }

    return !hole_invalid;
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
    for (size_t part = 0; part < poly.convex_parts.size(); ++part) {
        const auto& bounds = poly.convex_parts[part].bounding_circle;
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
            }
        }
    }
    return confirmed_collisions;
}

// -------------------------------------------------------------------------
// MAIN ENGINE ENTRY POINTS
// -------------------------------------------------------------------------

template<class VecType, class Tracer = DefaultTracer>
std::vector<std::pair<int, int>> find_polygon_intersections(
    const std::vector<Polygon<VecType>>& polygons,
    const VecType& sweep_axis,
    Tracer* tracer = nullptr
) {
    using Scalar = typename VecType::Scalar;
    if (polygons.size() < 2) return {};

    std::vector<PartSweepElement<VecType>> elements;
    elements.reserve(polygons.size() * 4);

    Scalar axis_sq = sweep_axis.len_sq();
    if (axis_sq < static_cast<Scalar>(1e-8)) return {};
    Scalar axis_len_sqrt = static_cast<Scalar>(std::sqrt(static_cast<double>(axis_sq)));

    for (size_t i = 0; i < polygons.size(); ++i) {
        append_poly_parts_to_sweep(static_cast<int>(i), 0, polygons[i], sweep_axis, axis_len_sqrt, elements);
    }

    return execute_part_sweep<VecType, Tracer>(elements, SweepMode::Monopartite, -1, tracer);
}

template<class VecType, class Tracer = DefaultTracer>
std::vector<std::pair<int, int>> find_polygon_intersections(
    const std::vector<Polygon<VecType>>& polygons,
    const std::vector<int>& active_indices,
    const VecType& sweep_axis,
    Tracer* tracer = nullptr
) {
    using Scalar = typename VecType::Scalar;

    if (active_indices.empty() || polygons.size() < 2) return {};

    std::vector<PartSweepElement<VecType>> elements;
    elements.reserve(polygons.size() * 4);

    Scalar axis_sq = sweep_axis.len_sq();
    if (axis_sq < static_cast<Scalar>(1e-8)) return {};
    Scalar axis_len_sqrt = static_cast<Scalar>(std::sqrt(static_cast<double>(axis_sq)));

    std::vector<int> group_ids(polygons.size(), 0);
    for (int idx : active_indices) {
        if (idx >= 0 && idx < static_cast<int>(polygons.size())) {
            group_ids[idx] = 1;
        }
    }

    for (size_t i = 0; i < polygons.size(); ++i) {
        append_poly_parts_to_sweep(static_cast<int>(i), group_ids[i], polygons[i], sweep_axis, axis_len_sqrt, elements);
    }

    return execute_part_sweep<VecType, Tracer>(elements, SweepMode::Subset, -1, tracer);
}

template<class VecType, class Tracer = DefaultTracer>
std::vector<std::pair<int, int>> find_polygon_intersections(
    const std::vector<Polygon<VecType>>& setA,
    const std::vector<Polygon<VecType>>& setB,
    const VecType& sweep_axis,
    Tracer* tracer = nullptr
) {
    using Scalar = typename VecType::Scalar;
    if (setA.empty() || setB.empty()) return {};

    std::vector<PartSweepElement<VecType>> elements;
    elements.reserve((setA.size() + setB.size()) * 4);

    Scalar axis_sq = sweep_axis.len_sq();
    if (axis_sq < static_cast<Scalar>(1e-8)) return {};
    Scalar axis_len_sqrt = static_cast<Scalar>(std::sqrt(static_cast<double>(axis_sq)));

    for (size_t i = 0; i < setA.size(); ++i) {
        append_poly_parts_to_sweep(static_cast<int>(i), 0, setA[i], sweep_axis, axis_len_sqrt, elements);
    }

    for (size_t i = 0; i < setB.size(); ++i) {
        append_poly_parts_to_sweep(
            static_cast<int>(setA.size() + i),
            1,
            setB[i],
            sweep_axis,
            axis_len_sqrt,
            elements);
    }

    return execute_part_sweep<VecType, Tracer>(elements, SweepMode::Bipartite, static_cast<int>(setA.size()), tracer);
}