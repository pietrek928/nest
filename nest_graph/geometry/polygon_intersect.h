#pragma once

#include <vector>
#include <cmath>
#include <utility>

#include "solid_geometry.h"
#include "point_in_solid.h"
#include "containment.h"
#include "geometry_common.h"
#include "sweep_engine.h"
#include "convex/intersect.h"

// -------------------------------------------------------------------------
// NARROW PHASE ROUTERS
// -------------------------------------------------------------------------

template <class VecType>
inline bool narrow_phase_intersect(
    const VecType* lsA, int nA,
    const VecType* lsB, int nB,
    int GRADIENT_THRESHOLD = 24
) {
    const VecType* p1 = (nA <= nB) ? lsA : lsB;
    const int s1 = (nA <= nB) ? nA : nB;
    const VecType* p2 = (nA <= nB) ? lsB : lsA;
    const int s2 = (nA <= nB) ? nB : nA;

    if (s1 + s2 > GRADIENT_THRESHOLD) {
        return convex_linestrings_intersect_gjk_gradient<VecType>(p1, s1, p2, s2).intersect;
    }
    return convex_linestrings_intersect_gjk<VecType>(p1, s1, p2, s2).intersect;
}

// narrow_phase_contain: see containment.h (witness; requires disjoint boundaries).

template <class VecType, class Tracer = DefaultTracer>
inline bool check_part_vs_part(
    const SolidGeometry<VecType>& polyA, int a_idx,
    const SolidGeometry<VecType>& polyB, int b_idx,
    Tracer* tracer = nullptr
) {
    return check_part_vs_part_intersect<VecType, Tracer>(polyA, a_idx, polyB, b_idx, tracer);
}

// -------------------------------------------------------------------------
// MAIN ENGINE ENTRY POINTS
// -------------------------------------------------------------------------

template <class VecType, class Tracer = DefaultTracer>
std::vector<std::pair<int, int>> find_polygon_intersections(
    const std::vector<SolidGeometry<VecType>>& polygons,
    Tracer* tracer = nullptr
) {
    if (polygons.size() < 2) return {};

    auto ctx = prepare_sweep_axis<VecType>(polygons);

    std::vector<PartSweepElement<VecType>> elements;
    elements.reserve(calculate_exact_sweep_capacity(polygons));

    for (size_t i = 0; i < polygons.size(); ++i) {
        append_poly_parts_to_sweep(
            static_cast<int>(i), 0, polygons[i], ctx.axis, ctx.axis_len_sqrt, elements);
    }

    auto sweep_res = execute_intersect_sweep<VecType, Tracer>(elements, SweepMode::Monopartite, -1, tracer);
    auto collisions = std::move(sweep_res.confirmed_collisions);
    promote_containment_pairs(polygons, sweep_res.potential_containments, collisions);

    return collisions;
}

template <class VecType, class Tracer = DefaultTracer>
std::vector<std::pair<int, int>> find_polygon_intersections(
    const std::vector<SolidGeometry<VecType>>& polygons,
    const std::vector<int>& active_indices,
    Tracer* tracer = nullptr
) {
    if (active_indices.empty() || polygons.size() < 2) return {};

    auto ctx = prepare_sweep_axis<VecType>(polygons);

    std::vector<PartSweepElement<VecType>> elements;
    elements.reserve(calculate_exact_sweep_capacity(polygons));

    std::vector<int> group_ids(polygons.size(), 0);
    for (int idx : active_indices) {
        if (idx >= 0 && idx < static_cast<int>(polygons.size())) {
            group_ids[idx] = 1;
        }
    }

    for (size_t i = 0; i < polygons.size(); ++i) {
        append_poly_parts_to_sweep(
            static_cast<int>(i), group_ids[i], polygons[i], ctx.axis, ctx.axis_len_sqrt, elements);
    }

    auto sweep_res = execute_intersect_sweep<VecType, Tracer>(elements, SweepMode::Subset, -1, tracer);
    auto collisions = std::move(sweep_res.confirmed_collisions);
    promote_containment_pairs(polygons, sweep_res.potential_containments, collisions);

    return collisions;
}

template <class VecType, class Tracer = DefaultTracer>
std::vector<std::pair<int, int>> find_polygon_intersections(
    const std::vector<SolidGeometry<VecType>>& setA,
    const std::vector<SolidGeometry<VecType>>& setB,
    Tracer* tracer = nullptr
) {
    if (setA.empty() || setB.empty()) return {};

    auto ctx = prepare_sweep_axis<VecType>(setA, setB);

    std::vector<PartSweepElement<VecType>> elements;
    elements.reserve(calculate_exact_sweep_capacity(setA) + calculate_exact_sweep_capacity(setB));

    for (size_t i = 0; i < setA.size(); ++i) {
        append_poly_parts_to_sweep(
            static_cast<int>(i), 0, setA[i], ctx.axis, ctx.axis_len_sqrt, elements);
    }
    for (size_t i = 0; i < setB.size(); ++i) {
        append_poly_parts_to_sweep(
            static_cast<int>(setA.size() + i), 1, setB[i], ctx.axis, ctx.axis_len_sqrt, elements);
    }

    auto sweep_res = execute_intersect_sweep<VecType, Tracer>(
        elements, SweepMode::Bipartite, static_cast<int>(setA.size()), tracer);

    auto collisions = std::move(sweep_res.confirmed_collisions);
    promote_containment_pairs_bipartite(setA, setB, sweep_res.potential_containments, collisions);

    return collisions;
}
