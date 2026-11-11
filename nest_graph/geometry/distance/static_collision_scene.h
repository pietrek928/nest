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

    // Reused per query: placed parts then obstacles (bipartite set A = placed count).
    // Buffer contract: capacity >= obstacle_elements.size() + placed_parts (~+4).
    // Mutable so const query_* can reuse without allocating; not thread-safe across
    // concurrent queries on the same scene instance.
    mutable std::vector<PartSweepElement<VecType>> query_elements;
    mutable SweepScratch<VecType> sweep_scratch;

    void build(
        std::vector<SolidGeometry<VecType>> obs,
        Scalar aura = static_cast<Scalar>(0.5)
    ) {
        obstacles = std::move(obs);
        aura_multiplier = aura;
        obstacle_elements.clear();
        query_elements.clear();
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

    // Assemble query_elements = placed parts + obstacle_elements (order fixed).
    //
    // Buffer contract:
    //   query_elements: cleared then reserved to obstacle_elements.size()+4;
    //   placed is poly_idx 0 (one polygon in bipartite set A); obstacles keep
    //   their poly_idx from build. Caller passes bipartite_set_a_size=1.
    void fill_query_elements(
        const SolidGeometry<VecType> &placed,
        Scalar distance_margin
    ) const {
        query_elements.clear();
        query_elements.reserve(obstacle_elements.size() + 4);
        append_poly_parts_to_sweep(
            0, 0, placed, ctx.axis, ctx.axis_len_sqrt, query_elements,
            aura_multiplier, distance_margin);
        query_elements.insert(
            query_elements.end(),
            obstacle_elements.begin(),
            obstacle_elements.end());
    }

    // Distance hits of placed vs obstacles.
    //
    // Buffer contract:
    //   Uses query_elements scratch (see fill_query_elements).
    //   Returns a new result vector (M2 pulls sweep result scratch).
    std::vector<ComplexDistanceResult<VecType>> query_placed(
        const SolidGeometry<VecType>& placed,
        Scalar distance_margin = static_cast<Scalar>(0)
    ) const {
        if (obstacles.empty()) {
            return {};
        }
        fill_query_elements(placed, distance_margin);
        return execute_distance_sweep<VecType>(
            query_elements, sweep_scratch, aura_multiplier, distance_margin,
            SweepMode::Bipartite, 1);
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
    //
    // Buffer contract:
    //   Uses query_elements scratch (capacity >= obstacle_elements.size()+placed_parts).
    bool query_any_violation(
        const SolidGeometry<VecType>& placed,
        Scalar margin_sq,
        Scalar distance_margin
    ) const {
        if (obstacles.empty()) {
            return false;
        }
        fill_query_elements(placed, distance_margin);
        return execute_distance_sweep_any_violation<VecType>(
            query_elements, sweep_scratch, aura_multiplier, distance_margin,
            margin_sq, SweepMode::Bipartite, 1);
    }
};
