#pragma once

#include <cmath>
#include <vector>

#include "distance/polygon_distance.h"
#include "solid/solid_geometry.h"
#include "sweep/sweep_engine.h"

template <class VecType>
struct StaticCollisionScene {
    using Scalar = typename VecType::Scalar;

    std::vector<SolidGeometry<VecType>> obstacles;
    SweepContext<VecType> ctx{};
    std::vector<PartSweepElement<VecType>> obstacle_elements;
    Scalar aura_multiplier = static_cast<Scalar>(0.5);

    void build(
        const std::vector<SolidGeometry<VecType>>& obs,
        Scalar aura = static_cast<Scalar>(0.5)
    ) {
        obstacles = obs;
        aura_multiplier = aura;
        obstacle_elements.clear();
        if (obstacles.empty()) {
            return;
        }
        ctx = prepare_sweep_axis<VecType>(obstacles);
        obstacle_elements.reserve(obstacles.size() * 4);
        for (size_t i = 0; i < obstacles.size(); ++i) {
            append_poly_parts_to_sweep(
                static_cast<int>(1 + i),
                1,
                obstacles[i],
                ctx.axis,
                ctx.axis_len_sqrt,
                obstacle_elements,
                aura_multiplier);
        }
    }

    std::vector<ComplexDistanceResult<VecType>> query_placed(
        const SolidGeometry<VecType>& placed,
        Scalar distance_margin = static_cast<Scalar>(0)
    ) const {
        if (obstacles.empty()) {
            return {};
        }
        std::vector<PartSweepElement<VecType>> elements;
        elements.reserve(obstacle_elements.size() + 4);
        append_poly_parts_to_sweep(
            0, 0, placed, ctx.axis, ctx.axis_len_sqrt, elements,
            aura_multiplier, distance_margin);
        elements.insert(
            elements.end(), obstacle_elements.begin(), obstacle_elements.end());
        return execute_distance_sweep<VecType>(
            elements, aura_multiplier, distance_margin, SweepMode::Bipartite, 1);
    }

    bool is_valid_placement(
        const SolidGeometry<VecType>& placed,
        Scalar margin_sq
    ) const {
        if (obstacles.empty()) {
            return true;
        }
        const Scalar distance_margin = margin_sq > static_cast<Scalar>(0)
            ? static_cast<Scalar>(std::sqrt(static_cast<double>(margin_sq)))
            : static_cast<Scalar>(0);
        return !query_any_violation(placed, margin_sq, distance_margin);
    }

    // Returns true if any obstacle intersects or is closer than margin_sq.
    // Uses distance_margin in the broad-phase prune and stops on first violation.
    bool query_any_violation(
        const SolidGeometry<VecType>& placed,
        Scalar margin_sq,
        Scalar distance_margin
    ) const {
        if (obstacles.empty()) {
            return false;
        }
        std::vector<PartSweepElement<VecType>> elements;
        elements.reserve(obstacle_elements.size() + 4);
        append_poly_parts_to_sweep(
            0, 0, placed, ctx.axis, ctx.axis_len_sqrt, elements,
            aura_multiplier, distance_margin);
        elements.insert(
            elements.end(), obstacle_elements.begin(), obstacle_elements.end());
        return execute_distance_sweep_any_violation<VecType>(
            elements, aura_multiplier, distance_margin, margin_sq,
            SweepMode::Bipartite, 1);
    }
};
