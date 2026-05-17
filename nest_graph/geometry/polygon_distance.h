#pragma once

#include <vector>
#include <cmath>
#include <type_traits>

#include "solid_geometry.h"
#include "containment.h"
#include "geometry_common.h"
#include "sweep_engine.h"
#include "convex/distance.h"
#include "convex/penetration.h"
#include "sweep.h"
#include "tracer.h"

// ComplexDistanceResult and execute_distance_sweep live in sweep_engine.h

template <class VecType, class Tracer = DefaultTracer>
inline DistanceResult<VecType> narrow_phase_distance(
    const VecType* lsA,
    int nA,
    const VecType* lsB,
    int nB,
    bool known_overlap,
    int GRADIENT_THRESHOLD = 24,
    Tracer* tracer = nullptr
) {
    if constexpr (!std::is_same_v<Tracer, NullTracer>) {
        if (tracer) {
            tracer->record_distance();
        }
    }

    const VecType* p1 = (nA <= nB) ? lsA : lsB;
    const int s1 = (nA <= nB) ? nA : nB;
    const VecType* p2 = (nA <= nB) ? lsB : lsA;
    const int s2 = (nA <= nB) ? nB : nA;

    if (s1 + s2 > GRADIENT_THRESHOLD) {
        return convex_linestrings_distance_gjk_gradient<VecType>(p1, s1, p2, s2, known_overlap);
    }
    return convex_linestrings_distance_gjk<VecType>(p1, s1, p2, s2, known_overlap);
}

template <class VecType, class Tracer = DefaultTracer>
inline PenetrationResult<VecType> narrow_phase_penetration(
    const VecType* lsA,
    int nA,
    const VecType* lsB,
    int nB,
    int GRADIENT_THRESHOLD = 24,
    Tracer* tracer = nullptr
) {
    if constexpr (!std::is_same_v<Tracer, NullTracer>) {
        if (tracer) {
            tracer->record_penetration();
        }
    }

    const bool swapped = (nA > nB);
    const VecType* p1 = swapped ? lsB : lsA;
    const int s1 = swapped ? nB : nA;
    const VecType* p2 = swapped ? lsA : lsB;
    const int s2 = swapped ? nA : nB;

    auto res = (s1 + s2 > GRADIENT_THRESHOLD)
        ? convex_linestrings_penetration_gradient<VecType>(p1, s1, p2, s2)
        : convex_linestrings_penetration<VecType>(p1, s1, p2, s2);

    if (swapped && res.intersect) {
        res.mtv = -res.mtv;
    }
    return res;
}

// -------------------------------------------------------------------------
// MAIN ENGINE ENTRY POINTS
// -------------------------------------------------------------------------

template <class VecType, class Tracer = DefaultTracer>
std::vector<ComplexDistanceResult<VecType>> find_polygon_distances(
    const std::vector<SolidGeometry<VecType>>& polygons,
    typename VecType::Scalar aura_multiplier = static_cast<typename VecType::Scalar>(0.5),
    Tracer* tracer = nullptr
) {
    using Scalar = typename VecType::Scalar;
    if (polygons.size() < 2) {
        return {};
    }

    const VecType sweep_axis = compute_optimal_sweep_axis(polygons);
    std::vector<PartSweepElement<VecType>> elements;
    elements.reserve(polygons.size() * 4);

    const Scalar axis_sq = sweep_axis.len_sq();
    if (axis_sq < static_cast<Scalar>(1e-8)) {
        return {};
    }
    const Scalar axis_len_sqrt = static_cast<Scalar>(std::sqrt(static_cast<double>(axis_sq)));

    for (size_t i = 0; i < polygons.size(); ++i) {
        append_poly_parts_to_sweep(
            static_cast<int>(i), 0, polygons[i], sweep_axis, axis_len_sqrt, elements, aura_multiplier);
    }

    return execute_distance_sweep<VecType, Tracer>(
        elements, aura_multiplier, SweepMode::Monopartite, -1, tracer);
}

template <class VecType, class Tracer = DefaultTracer>
std::vector<ComplexDistanceResult<VecType>> find_polygon_distances(
    const std::vector<SolidGeometry<VecType>>& polygons,
    const std::vector<int>& active_indices,
    typename VecType::Scalar aura_multiplier = static_cast<typename VecType::Scalar>(0.5),
    Tracer* tracer = nullptr
) {
    using Scalar = typename VecType::Scalar;
    if (active_indices.empty() || polygons.size() < 2) {
        return {};
    }

    const VecType sweep_axis = compute_optimal_sweep_axis(polygons);
    std::vector<PartSweepElement<VecType>> elements;
    elements.reserve(polygons.size() * 4);

    const Scalar axis_sq = sweep_axis.len_sq();
    if (axis_sq < static_cast<Scalar>(1e-8)) {
        return {};
    }
    const Scalar axis_len_sqrt = static_cast<Scalar>(std::sqrt(static_cast<double>(axis_sq)));

    std::vector<int> group_ids(polygons.size(), 0);
    for (int idx : active_indices) {
        if (idx >= 0 && idx < static_cast<int>(polygons.size())) {
            group_ids[idx] = 1;
        }
    }

    for (size_t i = 0; i < polygons.size(); ++i) {
        append_poly_parts_to_sweep(
            static_cast<int>(i),
            group_ids[i],
            polygons[i],
            sweep_axis,
            axis_len_sqrt,
            elements,
            aura_multiplier);
    }

    return execute_distance_sweep<VecType, Tracer>(
        elements, aura_multiplier, SweepMode::Subset, -1, tracer);
}

template <class VecType, class Tracer = DefaultTracer>
std::vector<ComplexDistanceResult<VecType>> find_polygon_distances(
    const std::vector<SolidGeometry<VecType>>& setA,
    const std::vector<SolidGeometry<VecType>>& setB,
    typename VecType::Scalar aura_multiplier = static_cast<typename VecType::Scalar>(0.5),
    Tracer* tracer = nullptr
) {
    using Scalar = typename VecType::Scalar;
    if (setA.empty() || setB.empty()) {
        return {};
    }

    const VecType sweep_axis = compute_optimal_sweep_axis(setA, setB);
    std::vector<PartSweepElement<VecType>> elements;
    elements.reserve((setA.size() + setB.size()) * 4);

    const Scalar axis_sq = sweep_axis.len_sq();
    if (axis_sq < static_cast<Scalar>(1e-8)) {
        return {};
    }
    const Scalar axis_len_sqrt = static_cast<Scalar>(std::sqrt(static_cast<double>(axis_sq)));

    for (size_t i = 0; i < setA.size(); ++i) {
        append_poly_parts_to_sweep(
            static_cast<int>(i), 0, setA[i], sweep_axis, axis_len_sqrt, elements, aura_multiplier);
    }
    for (size_t i = 0; i < setB.size(); ++i) {
        append_poly_parts_to_sweep(
            static_cast<int>(setA.size() + i),
            1,
            setB[i],
            sweep_axis,
            axis_len_sqrt,
            elements,
            aura_multiplier);
    }

    return execute_distance_sweep<VecType, Tracer>(
        elements, aura_multiplier, SweepMode::Bipartite, static_cast<int>(setA.size()), tracer);
}
