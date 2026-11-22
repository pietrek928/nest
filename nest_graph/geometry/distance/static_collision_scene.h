#pragma once

#include <cmath>
#include <vector>

#include "common/aabb_tree.h"
#include "common/geometry_common.h"
#include "distance/polygon_distance.h"
#include "guide/polygon_cast.h"
#include "plane/plane_geometry.h"
#include "sweep/sweep_engine.h"

template <class VecType>
inline void plane_bounding_circle_ltrb(
    const PlaneGeometry<VecType>& solid,
    double* out4
) {
    circle_ltrb(solid.get_bounding_circle(), out4);
}

template <class VecType>
struct StaticCollisionScene {
    using Scalar = typename VecType::Scalar;

    std::vector<PlaneGeometry<VecType>> obstacles;
    SweepContext<VecType> ctx{};
    std::vector<PartSweepElement<VecType>> obstacle_elements;
    Scalar aura_multiplier = static_cast<Scalar>(0.5);

    // AABB BVH of obstacle circle-boxes. query_any_violation prunes via
    // aabb_tree_query_box (inflate by margin + pad); fail-open to full set.
    AabbTree aabb_tree;
    bool aabb_ok = false;

    // Reused per query: placed parts then obstacles (bipartite set A = placed count).
    // Buffer contract: capacity >= obstacle_elements.size() + placed_parts (~+4).
    // Mutable so const query_* can reuse without allocating; not thread-safe across
    // concurrent queries on the same scene instance.
    mutable std::vector<PartSweepElement<VecType>> query_elements;
    mutable SweepScratch<VecType> sweep_scratch;
    mutable AabbTreeScratch aabb_scratch;
    mutable std::vector<int> aabb_hits;
    mutable std::vector<char> aabb_hit_mask;

    void build(
        std::vector<PlaneGeometry<VecType>> obs,
        Scalar aura = static_cast<Scalar>(0.5)
    ) {
        obstacles = std::move(obs);
        aura_multiplier = aura;
        obstacle_elements.clear();
        query_elements.clear();
        aabb_ok = false;
        if (obstacles.empty()) {
            aabb_ok = aabb_tree_rebuild_boxes(nullptr, 0, aabb_tree);
            return;
        }
        ctx = prepare_sweep_axis<VecType>(obstacles);
        obstacle_elements.reserve(obstacles.size() * 4);
        std::vector<double> ltrb(obstacles.size() * 4);
        for (size_t i = 0; i < obstacles.size(); ++i) {
            append_poly_parts_to_sweep(
                static_cast<int>(1 + i),
                1,
                obstacles[i],
                ctx.axis,
                ctx.axis_len_sqrt,
                obstacle_elements,
                aura_multiplier);
            plane_bounding_circle_ltrb(obstacles[i], ltrb.data() + 4 * i);
        }
        aabb_ok = aabb_tree_rebuild_boxes(ltrb.data(), obstacles.size(), aabb_tree);
    }

    // Assemble query_elements = placed parts + obstacle_elements (order fixed).
    //
    // Buffer contract:
    //   query_elements: cleared then reserved to obstacle_elements.size()+4;
    //   placed is poly_idx 0 (one polygon in bipartite set A); obstacles keep
    //   their poly_idx from build. Caller passes bipartite_set_a_size=1.
    void fill_query_elements(
        const PlaneGeometry<VecType>& placed,
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

    void fill_query_elements_pruned(
        const PlaneGeometry<VecType>& placed,
        Scalar distance_margin,
        const int* obs_ids,
        std::size_t n_obs
    ) const {
        query_elements.clear();
        query_elements.reserve(obstacle_elements.size() + 4);
        append_poly_parts_to_sweep(
            0, 0, placed, ctx.axis, ctx.axis_len_sqrt, query_elements,
            aura_multiplier, distance_margin);
        if (n_obs == 0) {
            return;
        }
        aabb_hit_mask.assign(obstacles.size(), 0);
        for (std::size_t h = 0; h < n_obs; ++h) {
            const int oi = obs_ids[h];
            if (oi >= 0 && static_cast<std::size_t>(oi) < obstacles.size()) {
                aabb_hit_mask[static_cast<std::size_t>(oi)] = 1;
            }
        }
        for (const auto& el : obstacle_elements) {
            const int oi = el.poly_idx - 1;
            if (oi >= 0
                && static_cast<std::size_t>(oi) < aabb_hit_mask.size()
                && aabb_hit_mask[static_cast<std::size_t>(oi)]) {
                query_elements.push_back(el);
            }
        }
    }

    std::vector<ComplexDistanceResult<VecType>> query_placed(
        const PlaneGeometry<VecType>& placed,
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
        const PlaneGeometry<VecType>& placed,
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
        const PlaneGeometry<VecType>& placed,
        Scalar margin_sq,
        Scalar distance_margin
    ) const {
        if (obstacles.empty()) {
            return false;
        }

        bool pruned = false;
        std::size_t n_hit = 0;
        if (aabb_ok) {
            double pl[4];
            plane_bounding_circle_ltrb(placed, pl);
            // Inflate by Scene margin + conservative pad (touch scale / graze).
            const double inflate =
                static_cast<double>(distance_margin) + 2e-5;
            pl[0] -= inflate;
            pl[1] -= inflate;
            pl[2] += inflate;
            pl[3] += inflate;
            if (aabb2_ltrb_valid(pl[0], pl[1], pl[2], pl[3])) {
                aabb_hits.resize(obstacles.size());
                n_hit = aabb_tree_query_box(
                    aabb_tree,
                    aabb_scratch,
                    pl[0],
                    pl[1],
                    pl[2],
                    pl[3],
                    aabb_hits.data(),
                    aabb_hits.size());
                pruned = true;
            }
        }
        if (pruned && n_hit == 0) {
            return false;
        }
        if (pruned) {
            fill_query_elements_pruned(
                placed, distance_margin, aabb_hits.data(), n_hit);
        } else {
            fill_query_elements(placed, distance_margin);
        }
        return execute_distance_sweep_any_violation<VecType>(
            query_elements, sweep_scratch, aura_multiplier, distance_margin,
            margin_sq, SweepMode::Bipartite, 1);
    }

    // G4 cast: reuse scene AABB tree (no per-call rebuild). Fail-open to all.
    ComplexCastResult<VecType> cast_closest(
        const PlaneGeometry<VecType>& active,
        const VecType& slide_vector,
        Scalar max_t_limit
    ) const {
        ComplexCastResult<VecType> closest_hit;
        closest_hit.intersects_path = false;
        closest_hit.t_entry = max_t_limit;
        if (obstacles.empty()) {
            return closest_hit;
        }

        const auto& active_bounds = active.get_bounding_circle();
        bool use_full = !aabb_ok;
        std::size_t n_hit = 0;
        if (!use_full) {
            double active_box[4];
            circle_ltrb(active_bounds, active_box);
            const double acx = 0.5 * (active_box[0] + active_box[2]);
            const double acy = 0.5 * (active_box[1] + active_box[3]);
            const double ar = 0.5 * (active_box[2] - active_box[0]);
            const double sx = static_cast<double>(slide_vector[0]);
            const double sy = static_cast<double>(slide_vector[1]);
            const double tmax = static_cast<double>(max_t_limit);
            if (!std::isfinite(tmax)) {
                use_full = true;
            } else {
                const double t_use = std::max(0.0, tmax);
                const double ex = acx + sx * t_use;
                const double ey = acy + sy * t_use;
                const double inflate = 2e-5;
                const double qminx = std::min(acx, ex) - ar - inflate;
                const double qminy = std::min(acy, ey) - ar - inflate;
                const double qmaxx = std::max(acx, ex) + ar + inflate;
                const double qmaxy = std::max(acy, ey) + ar + inflate;
                if (!aabb2_ltrb_valid(qminx, qminy, qmaxx, qmaxy)) {
                    use_full = true;
                } else {
                    aabb_hits.resize(obstacles.size());
                    n_hit = aabb_tree_query_box(
                        aabb_tree,
                        aabb_scratch,
                        qminx,
                        qminy,
                        qmaxx,
                        qmaxy,
                        aabb_hits.data(),
                        aabb_hits.size());
                }
            }
        }

        auto consider = [&](std::size_t i) {
            const auto& obstacle = obstacles[i];
            if (!moving_circle_intersects_static_circle(
                    active_bounds, slide_vector, obstacle.get_bounding_circle(),
                    closest_hit.t_entry)) {
                return;
            }
            for (size_t pA = 0; pA < active.line_parts.size(); ++pA) {
                if (active.line_parts[pA].is_subtractive) {
                    continue;
                }
                for (size_t pB = 0; pB < obstacle.line_parts.size(); ++pB) {
                    if (obstacle.line_parts[pB].is_subtractive) {
                        continue;
                    }
                    if (!moving_circle_intersects_static_circle(
                            active.line_parts[pA].bounding_circle, slide_vector,
                            obstacle.line_parts[pB].bounding_circle,
                            closest_hit.t_entry)) {
                        continue;
                    }
                    auto res = narrow_phase_cast<VecType>(
                        active.get_part_points(pA), active.get_part_size(pA),
                        obstacle.get_part_points(pB), obstacle.get_part_size(pB),
                        slide_vector);
                    if (res.intersects_path && res.t_exit >= static_cast<Scalar>(0)) {
                        Scalar entry = std::max(static_cast<Scalar>(0), res.t_entry);
                        if (entry < closest_hit.t_entry) {
                            closest_hit.intersects_path = true;
                            closest_hit.t_entry = entry;
                            closest_hit.t_exit = res.t_exit;
                            closest_hit.polyA_idx = 0;
                            closest_hit.partA_idx = static_cast<int>(pA);
                            closest_hit.polyB_idx = static_cast<int>(i + 1);
                            closest_hit.partB_idx = static_cast<int>(pB);
                        }
                    }
                }
            }
        };

        if (use_full) {
            for (std::size_t i = 0; i < obstacles.size(); ++i) {
                consider(i);
            }
        } else {
            for (std::size_t h = 0; h < n_hit; ++h) {
                const int oi = aabb_hits[h];
                if (oi >= 0 && static_cast<std::size_t>(oi) < obstacles.size()) {
                    consider(static_cast<std::size_t>(oi));
                }
            }
        }
        return closest_hit;
    }
};
