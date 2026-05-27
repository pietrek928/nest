#pragma once

#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>
#include <string>

#include "solid/solid_geometry.h"
#include "distance/polygon_distance.h"

// -------------------------------------------------------------------------
// CONFIGURATION & RESULT STRUCTURES
// -------------------------------------------------------------------------
template <class VecType>
struct GuidanceConfig {
    using Scalar = typename VecType::Scalar;

    // --- Physics & Spacing ---
    Scalar minimum_placing_distance = static_cast<Scalar>(1e-6);

    // --- Tight Packing (Squeeze) ---
    bool use_tight_packing = true;
    Scalar squeeze_weight = static_cast<Scalar>(0.4); // Actively pulls shapes to close empty gaps

    // --- Hole Seeking & Behavior ---
    bool use_hole_seeking = true;
    Scalar hole_seeking_weight = static_cast<Scalar>(0.3);

    // --- Attractors & Gravity ---
    bool use_gravity = true;
    VecType gravity_vector{static_cast<Scalar>(-1.0), static_cast<Scalar>(-1.0)};
    bool use_target_attractor = false;
    VecType target_position{};
    Scalar target_angle_rad = 0.0;

    // --- PROPOSITION EXPLORATION (The "Wider Space") ---
    int max_propositions = 5; // How many distinct choices to return to the solver

    // Spatial Diversity (Prevents returning multiple hints that do the exact same thing)
    Scalar diversity_distance_threshold = static_cast<Scalar>(2.0);
    Scalar diversity_angle_rad_threshold = static_cast<Scalar>(M_PI / 8.0); // 22.5 degrees

    bool enable_grid_exploration = true; // Synthesize extra local micro-moves
    Scalar grid_exploration_step = static_cast<Scalar>(2.0);
    int max_alternative_angles = 3;
    Scalar slide_escape_multiplier = static_cast<Scalar>(0.8);

    // --- Weights & Limits ---
    Scalar attraction_weight = static_cast<Scalar>(0.1);
    Scalar alignment_weight = static_cast<Scalar>(0.5);
    Scalar escape_radius_multiplier = static_cast<Scalar>(1.5);
    Scalar search_radius = static_cast<Scalar>(5.0);
};

// A single curated move for the solver to attempt
template <class VecType>
struct PlacementProposition {
    using Scalar = typename VecType::Scalar;
    VecType translation{};
    Scalar rotation_rad = 0.0;
    Scalar heuristic_score = 0.0; // Higher is better
    std::string move_type = "";   // "Ejection", "Slide", "Pack", "Grid", etc.

    bool operator<(const PlacementProposition& o) const {
        return heuristic_score > o.heuristic_score; // Sort descending
    }
};

template <class VecType>
struct PlacementGuidance {
    using Scalar = typename VecType::Scalar;

    bool is_penetrating = false;
    Scalar clearance = std::numeric_limits<Scalar>::max();

    // The curated menu of distinct, high-quality moves for the solver
    std::vector<PlacementProposition<VecType>> propositions;
};

// Internal Context to pass data smoothly between phases
template <class VecType>
struct PhysicsContext {
    using Scalar = typename VecType::Scalar;

    VecType total_ejection{};
    VecType max_mtv{};
    Scalar max_mtv_sq = 0;
    VecType alignment_normal{};
    int penetration_count = 0;

    bool is_penetrating = false;
    Scalar min_clearance = std::numeric_limits<Scalar>::max();
    VecType closest_obstacle_center{};

    // Raw vectors generated before filtering
    std::vector<VecType> raw_translations;
    std::vector<Scalar> raw_rotations;
};

// -------------------------------------------------------------------------
// PHASE 1: AGGREGATION (Handles Deep Swallow & Hole Physics)
// -------------------------------------------------------------------------
template <class VecType>
inline void aggregate_physics_feedback(
    int placed_poly_idx,
    const std::vector<SolidGeometry<VecType>>& all_polygons,
    const std::vector<ComplexDistanceResult<VecType>>& results,
    const GuidanceConfig<VecType>& config,
    PhysicsContext<VecType>& ctx
) {
    using Scalar = typename VecType::Scalar;
    const auto& placed_bounds = all_polygons[placed_poly_idx].get_bounding_circle();
    const Scalar placed_radius = std::sqrt(placed_bounds.square_radius());

    for (const auto& res : results) {
        if (res.intersect) {
            ctx.is_penetrating = true;
            ctx.penetration_count++;

            const auto& obstacle = all_polygons[res.polyB_idx];
            const auto& obstacle_part = obstacle.line_parts[res.partB_idx];

            if (res.penetration_sq == std::numeric_limits<Scalar>::max()) {
                // DEEP SWALLOW & HOLE FIX
                VecType push_dir = placed_bounds.center() - obstacle_part.bounding_circle.center();

                // If completely swallowed by a hole's solid boundary, push BACK to the hole's center
                if (obstacle_part.is_subtractive) push_dir = -push_dir;

                Scalar dir_len_sq = push_dir.len_sq();
                if (dir_len_sq < static_cast<Scalar>(1e-8)) {
                    push_dir = { static_cast<Scalar>(1.0), static_cast<Scalar>(0.0) };
                    dir_len_sq = static_cast<Scalar>(1.0);
                }

                Scalar push_mag = (placed_radius * config.escape_radius_multiplier) + config.minimum_placing_distance;
                VecType fake_mtv = push_dir * (push_mag / std::sqrt(dir_len_sq));

                ctx.total_ejection = ctx.total_ejection + fake_mtv;
                if (fake_mtv.len_sq() > ctx.max_mtv_sq) {
                    ctx.max_mtv_sq = fake_mtv.len_sq();
                    ctx.max_mtv = fake_mtv;
                    ctx.alignment_normal = fake_mtv;
                }
            } else {
                VecType localized_mtv = res.mtv;
                if (res.polyB_idx == placed_poly_idx) localized_mtv = -localized_mtv;

                // INFLATE MTV BY MINIMUM PLACING DISTANCE
                Scalar mtv_len_sq = localized_mtv.len_sq();
                if (mtv_len_sq > 1e-8 && config.minimum_placing_distance > 0) {
                    Scalar mtv_len = std::sqrt(mtv_len_sq);
                    VecType n = localized_mtv * (static_cast<Scalar>(1.0) / mtv_len);
                    localized_mtv = localized_mtv + (n * config.minimum_placing_distance);
                    mtv_len_sq = localized_mtv.len_sq();
                }

                ctx.total_ejection = ctx.total_ejection + localized_mtv;
                if (mtv_len_sq > ctx.max_mtv_sq) {
                    ctx.max_mtv_sq = mtv_len_sq;
                    ctx.max_mtv = localized_mtv;
                    ctx.alignment_normal = localized_mtv;
                }
            }
        } else if (!ctx.is_penetrating) {
            if (res.distance_sq < ctx.min_clearance * ctx.min_clearance) {
                ctx.min_clearance = std::sqrt(static_cast<double>(res.distance_sq));
                ctx.closest_obstacle_center = all_polygons[res.polyB_idx].line_parts[res.partB_idx].bounding_circle.center();
            }
        }
    }
}

// -------------------------------------------------------------------------
// PHASE 2: MANDATORY EJECTION (Physics Resolution)
// -------------------------------------------------------------------------
template <class VecType>
inline void formulate_mandatory_ejection(
    const PhysicsContext<VecType>& ctx,
    const SolidGeometry<VecType>& placed_poly,
    const GuidanceConfig<VecType>& config,
    std::vector<PlacementProposition<VecType>>& raw_propositions
) {
    using Scalar = typename VecType::Scalar;

    VecType average_mtv = ctx.total_ejection * (static_cast<Scalar>(1.0) / ctx.penetration_count);
    VecType primary_ejection = (average_mtv + ctx.max_mtv) * static_cast<Scalar>(0.5);

    // Score: 100+ (Absolute priority to escape collision)
    raw_propositions.push_back({primary_ejection, 0.0, 100.0, "Primary Ejection"});

    if (ctx.max_mtv_sq > 1e-8) {
        VecType n = ctx.max_mtv * (static_cast<Scalar>(1.0) / std::sqrt(ctx.max_mtv_sq));
        VecType tangent = { -n[1], n[0] };
        Scalar placed_radius = std::sqrt(placed_poly.get_bounding_circle().square_radius());
        Scalar slide_mag = placed_radius * config.slide_escape_multiplier;

        // Score: 80 (Good fallback if primary ejection hits a bottleneck)
        raw_propositions.push_back({primary_ejection + (tangent * slide_mag), 0.0, 80.0, "Slide Ejection L"});
        raw_propositions.push_back({primary_ejection - (tangent * slide_mag), 0.0, 80.0, "Slide Ejection R"});
    }
}

// -------------------------------------------------------------------------
// PHASE 3: SOFT TRANSLATION (Packing, Squeeze, Seeking)
// -------------------------------------------------------------------------
template <class VecType>
inline void formulate_soft_translation(
    int placed_poly_idx,
    const std::vector<SolidGeometry<VecType>>& all_polygons,
    const VecType& current_position,
    const PhysicsContext<VecType>& ctx,
    const GuidanceConfig<VecType>& config,
    std::vector<PlacementProposition<VecType>>& raw_propositions
) {
    using Scalar = typename VecType::Scalar;
    VecType soft_translation{};
    bool seeking_active = false;

    const auto& placed_bounds = all_polygons[placed_poly_idx].get_bounding_circle();
    Scalar placed_radius = std::sqrt(placed_bounds.square_radius());

    // 1. Hole Seeking
    if (config.use_hole_seeking) {
        Scalar closest_hole_dist_sq = std::numeric_limits<Scalar>::max();
        VecType best_hole_center{};

        for (size_t i = 0; i < all_polygons.size(); ++i) {
            if (i == static_cast<size_t>(placed_poly_idx)) continue;

            for (size_t part = 0; part < all_polygons[i].line_parts.size(); ++part) {
                if (!all_polygons[i].line_parts[part].is_subtractive) continue;

                const auto& hole_bounds = all_polygons[i].line_parts[part].bounding_circle;
                Scalar hole_radius = std::sqrt(hole_bounds.square_radius());

                if (hole_radius >= (placed_radius + config.minimum_placing_distance) * static_cast<Scalar>(0.9)) {
                    Scalar dist_sq = (hole_bounds.center() - placed_bounds.center()).len_sq();
                    // HOLE FIX: Only attract if not already deeply inside the hole's center zone
                    if (dist_sq > (hole_radius * hole_radius) * static_cast<Scalar>(0.25)) {
                        if (dist_sq < closest_hole_dist_sq) {
                            closest_hole_dist_sq = dist_sq;
                            best_hole_center = hole_bounds.center();
                            seeking_active = true;
                        }
                    }
                }
            }
        }

        if (seeking_active) {
            soft_translation = soft_translation + ((best_hole_center - placed_bounds.center()) * config.hole_seeking_weight);
        }
    }

    // 2. Attractors, Gravity, and Tight Packing (Squeeze)
    if (!seeking_active) {
        if (config.use_target_attractor) {
            soft_translation = soft_translation + ((config.target_position - current_position) * config.attraction_weight);
        } else {
            if (config.use_gravity) {
                Scalar gravity_scale = std::min(static_cast<Scalar>(1.0), std::max(static_cast<Scalar>(0.0), ctx.min_clearance - config.minimum_placing_distance));
                soft_translation = soft_translation + (config.gravity_vector * gravity_scale);
            }
            // TIGHT PACKING FIX
            if (config.use_tight_packing && ctx.min_clearance > config.minimum_placing_distance && ctx.min_clearance < config.search_radius) {
                VecType squeeze_dir = ctx.closest_obstacle_center - placed_bounds.center();
                Scalar squeeze_len = std::sqrt(squeeze_dir.len_sq());
                if (squeeze_len > 1e-6) {
                    VecType n = squeeze_dir * (static_cast<Scalar>(1.0) / squeeze_len);
                    soft_translation = soft_translation + (n * config.squeeze_weight);
                }
            }
        }
    }

    // Score: 60 (Optimal packing, but lower priority than collision escape)
    raw_propositions.push_back({soft_translation, 0.0, 60.0, "Soft Packing/Seek"});
}

// -------------------------------------------------------------------------
// PHASE 4: SOFT ROTATION (Edge Mating)
// -------------------------------------------------------------------------
template <class VecType>
inline void formulate_rotations(
    const PhysicsContext<VecType>& ctx,
    const SolidGeometry<VecType>& placed_poly,
    const GuidanceConfig<VecType>& config,
    PhysicsContext<VecType>& out_ctx
) {
    using Scalar = typename VecType::Scalar;

    if (ctx.alignment_normal.len_sq() <= 1e-6 || config.max_alternative_angles <= 0) {
        if (config.use_target_attractor) out_ctx.raw_rotations.push_back(config.target_angle_rad);
        else out_ctx.raw_rotations.push_back(0.0);
        return;
    }

    Scalar inv_len = static_cast<Scalar>(1.0) / std::sqrt(static_cast<double>(ctx.alignment_normal.len_sq()));
    VecType n = ctx.alignment_normal * inv_len;
    VecType tangent = { -n[1], n[0] };

    struct EdgeData { Scalar len_sq; VecType vec; };
    std::vector<EdgeData> edges;

    for (size_t part = 0; part < placed_poly.line_parts.size(); ++part) {
        const VecType* pts = placed_poly.get_part_points(part);
        int n_pts = placed_poly.get_part_size(part);
        for (int i = 0; i < n_pts; ++i) {
            VecType edge = pts[(i + 1) % n_pts] - pts[i];
            edges.push_back({ edge.len_sq(), edge });
        }
    }

    std::sort(edges.begin(), edges.end(), [](const EdgeData& a, const EdgeData& b) { return a.len_sq > b.len_sq; });

    int angles_generated = 0;
    for (const auto& edge : edges) {
        if (angles_generated >= config.max_alternative_angles) break;
        if (edge.len_sq < 1e-6) continue;

        Scalar dot = edge.vec.dp(tangent);
        Scalar cross = (edge.vec[0] * tangent[1]) - (edge.vec[1] * tangent[0]);

        double base_angle = std::atan2(cross, dot);
        while (base_angle > M_PI / 2.0) base_angle -= M_PI;
        while (base_angle < -M_PI / 2.0) base_angle += M_PI;

        if (angles_generated == 0 && config.use_target_attractor) {
            base_angle = (base_angle * config.alignment_weight) + (config.target_angle_rad * (1.0 - config.alignment_weight));
        }

        out_ctx.raw_rotations.push_back(base_angle);
        angles_generated++;
    }
}

// -------------------------------------------------------------------------
// PHASE 5: PROPOSITION SYNTHESIS & SPATIAL FILTERING
// -------------------------------------------------------------------------
template <class VecType>
inline std::vector<PlacementProposition<VecType>> synthesize_propositions(
    std::vector<PlacementProposition<VecType>>& raw_props,
    const PhysicsContext<VecType>& ctx,
    const GuidanceConfig<VecType>& config
) {
    using Scalar = typename VecType::Scalar;
    std::vector<PlacementProposition<VecType>> cross_product_props;

    // 1. Cross-Multiply Translations with Rotations
    for (const auto& prop : raw_props) {
        if (ctx.raw_rotations.empty()) {
            cross_product_props.push_back(prop);
        } else {
            // Primary Angle
            PlacementProposition<VecType> p_primary = prop;
            p_primary.rotation_rad = ctx.raw_rotations[0];
            cross_product_props.push_back(p_primary);

            // Alternative Angles (Penalize score slightly so solver favors primary edge)
            for (size_t i = 1; i < ctx.raw_rotations.size(); ++i) {
                PlacementProposition<VecType> p_alt = prop;
                p_alt.rotation_rad = ctx.raw_rotations[i];
                p_alt.heuristic_score -= static_cast<Scalar>(5.0 * i);
                p_alt.move_type += " (Alt Angle)";
                cross_product_props.push_back(p_alt);
            }
        }
    }

    // 2. Synthesize Local Grid Exploration (Wider Space Search)
    if (!ctx.is_penetrating && config.enable_grid_exploration) {
        Scalar step = config.grid_exploration_step;
        VecType grid_nudges[4] = {{step, 0}, {-step, 0}, {0, step}, {0, -step}};
        Scalar primary_rot = ctx.raw_rotations.empty() ? 0.0 : ctx.raw_rotations[0];

        for (const auto& nudge : grid_nudges) {
            cross_product_props.push_back({
                raw_props.empty() ? nudge : raw_props[0].translation + nudge,
                primary_rot,
                40.0, // Low score, used only if main heuristics hit local minimum
                "Grid Exploration"
            });
        }
    }

    // 3. Spatial Diversity Filter (Non-Maximum Suppression)
    std::sort(cross_product_props.begin(), cross_product_props.end()); // Highest scores first

    std::vector<PlacementProposition<VecType>> final_propositions;
    Scalar dist_thresh_sq = config.diversity_distance_threshold * config.diversity_distance_threshold;

    for (const auto& prop : cross_product_props) {
        bool too_similar = false;

        for (const auto& accepted : final_propositions) {
            Scalar dist_sq = (prop.translation - accepted.translation).len_sq();

            // Normalize angle difference to [0, PI]
            Scalar ang_diff = std::abs(prop.rotation_rad - accepted.rotation_rad);
            while (ang_diff > M_PI) ang_diff -= 2.0 * M_PI;
            ang_diff = std::abs(ang_diff);

            if (dist_sq < dist_thresh_sq && ang_diff < config.diversity_angle_rad_threshold) {
                too_similar = true;
                break;
            }
        }

        if (!too_similar) {
            final_propositions.push_back(prop);
            if (static_cast<int>(final_propositions.size()) >= config.max_propositions) break;
        }
    }

    return final_propositions;
}

// -------------------------------------------------------------------------
// MAIN ENGINE ENTRY POINT
// -------------------------------------------------------------------------
template <class VecType>
inline PlacementGuidance<VecType> evaluate_local_placement(
    int placed_poly_idx,
    const std::vector<SolidGeometry<VecType>>& all_polygons,
    const VecType& current_position,
    const GuidanceConfig<VecType>& config = GuidanceConfig<VecType>{}
) {
    PlacementGuidance<VecType> guidance;

    std::vector<int> active_indices = { placed_poly_idx };
    auto results = find_polygon_distances<VecType>(all_polygons, active_indices, config.search_radius);

    if (results.empty()) {
        PlacementProposition<VecType> default_prop;
        if (config.use_target_attractor) {
            default_prop.translation = (config.target_position - current_position) * config.attraction_weight;
            default_prop.rotation_rad = config.target_angle_rad;
        } else if (config.use_gravity) {
            default_prop.translation = config.gravity_vector;
        }
        default_prop.heuristic_score = 100.0;
        default_prop.move_type = "Free Space Default";
        guidance.propositions.push_back(default_prop);
        return guidance;
    }

    PhysicsContext<VecType> ctx;
    std::vector<PlacementProposition<VecType>> raw_propositions;

    aggregate_physics_feedback(placed_poly_idx, all_polygons, results, config, ctx);

    guidance.is_penetrating = ctx.is_penetrating;
    guidance.clearance = ctx.min_clearance;

    if (ctx.is_penetrating) {
        formulate_mandatory_ejection(ctx, all_polygons[placed_poly_idx], config, raw_propositions);
    } else {
        formulate_soft_translation(placed_poly_idx, all_polygons, current_position, ctx, config, raw_propositions);
    }

    formulate_rotations(ctx, all_polygons[placed_poly_idx], config, ctx);

    guidance.propositions = synthesize_propositions(raw_propositions, ctx, config);

    return guidance;
}
