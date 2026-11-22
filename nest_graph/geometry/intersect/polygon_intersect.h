#pragma once

#include <vector>
#include <cmath>
#include <utility>

#include "plane/plane_geometry.h"
#include "plane/containment.h"
#include "common/geometry_common.h"
#include "sweep/sweep_engine.h"
#include "convex/intersect.h"

// Narrow-phase packing SoT: evaluate_narrow_phase / check_part_vs_part_intersect
// in sweep/sweep_engine.h (Penetrating only). Witness contain: containment.h.

// -------------------------------------------------------------------------
// MAIN ENGINE ENTRY POINTS
// -------------------------------------------------------------------------

template <class VecType, class Tracer = DefaultTracer>
std::vector<std::pair<int, int>> find_polygon_intersections(
    const std::vector<PlaneGeometry<VecType>>& polygons,
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
    const std::vector<PlaneGeometry<VecType>>& polygons,
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
    const std::vector<PlaneGeometry<VecType>>& setA,
    const std::vector<PlaneGeometry<VecType>>& setB,
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

template <class VecType, class Tracer = DefaultTracer>
std::vector<std::pair<int, int>> find_polygon_intersections(
    const std::vector<const PlaneGeometry<VecType>*>& polygons,
    Tracer* tracer = nullptr
) {
    if (polygons.size() < 2) return {};

    auto ctx = prepare_sweep_axis<VecType>(polygons);

    std::vector<PartSweepElement<VecType>> elements;
    elements.reserve(calculate_exact_sweep_capacity(polygons));

    for (size_t i = 0; i < polygons.size(); ++i) {
        if (polygons[i] == nullptr) {
            continue;
        }
        append_poly_parts_to_sweep(
            static_cast<int>(i), 0, *polygons[i], ctx.axis, ctx.axis_len_sqrt, elements);
    }

    auto sweep_res = execute_intersect_sweep<VecType, Tracer>(elements, SweepMode::Monopartite, -1, tracer);
    auto collisions = std::move(sweep_res.confirmed_collisions);
    promote_containment_pairs(polygons, sweep_res.potential_containments, collisions);

    return collisions;
}

template <class VecType, class Tracer = DefaultTracer>
std::vector<std::pair<int, int>> find_polygon_intersections(
    const std::vector<const PlaneGeometry<VecType>*>& polygons,
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
            group_ids[static_cast<std::size_t>(idx)] = 1;
        }
    }

    for (size_t i = 0; i < polygons.size(); ++i) {
        if (polygons[i] == nullptr) {
            continue;
        }
        append_poly_parts_to_sweep(
            static_cast<int>(i), group_ids[i], *polygons[i], ctx.axis, ctx.axis_len_sqrt, elements);
    }

    auto sweep_res = execute_intersect_sweep<VecType, Tracer>(elements, SweepMode::Subset, -1, tracer);
    auto collisions = std::move(sweep_res.confirmed_collisions);
    promote_containment_pairs(polygons, sweep_res.potential_containments, collisions);

    return collisions;
}

template <class VecType, class Tracer = DefaultTracer>
std::vector<std::pair<int, int>> find_polygon_intersections(
    const std::vector<const PlaneGeometry<VecType>*>& setA,
    const std::vector<const PlaneGeometry<VecType>*>& setB,
    Tracer* tracer = nullptr
) {
    if (setA.empty() || setB.empty()) return {};

    auto ctx = prepare_sweep_axis<VecType>(setA, setB);

    std::vector<PartSweepElement<VecType>> elements;
    elements.reserve(calculate_exact_sweep_capacity(setA) + calculate_exact_sweep_capacity(setB));

    for (size_t i = 0; i < setA.size(); ++i) {
        if (setA[i] == nullptr) {
            continue;
        }
        append_poly_parts_to_sweep(
            static_cast<int>(i), 0, *setA[i], ctx.axis, ctx.axis_len_sqrt, elements);
    }
    for (size_t i = 0; i < setB.size(); ++i) {
        if (setB[i] == nullptr) {
            continue;
        }
        append_poly_parts_to_sweep(
            static_cast<int>(setA.size() + i), 1, *setB[i], ctx.axis, ctx.axis_len_sqrt, elements);
    }

    auto sweep_res = execute_intersect_sweep<VecType, Tracer>(
        elements, SweepMode::Bipartite, static_cast<int>(setA.size()), tracer);

    auto collisions = std::move(sweep_res.confirmed_collisions);
    promote_containment_pairs_bipartite(setA, setB, sweep_res.potential_containments, collisions);

    return collisions;
}
