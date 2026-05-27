#pragma once

#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>

#include "solid/solid_geometry.h"
#include "distance/polygon_distance.h"

// -------------------------------------------------------------------------
// CONFIGURATION & RESULT STRUCTURES
// -------------------------------------------------------------------------
template <class VecType>
struct GuidanceConfig {
    using Scalar = typename VecType::Scalar;

    // --- NEW: Physics & Spacing ---
    // Default: geometric epsilon only; app code sets this from board_min_dist + margin.
    Scalar minimum_placing_distance = static_cast<Scalar>(1e-6);

    // --- NEW: Wider Space Exploration ---
    int max_alternative_angles = 3; // How many different edges to try mating against the wall
    Scalar slide_escape_multiplier = static_cast<Scalar>(0.8); // How far to slide sideways when trapped

    // Optional user-defined targets (Attractors)
    bool use_target_attractor = false;
    VecType target_position{};
    Scalar target_angle_rad = 0.0;

    // Gravity / Packing heuristics
    bool use_gravity = true;
    VecType gravity_vector{static_cast<Scalar>(-1.0), static_cast<Scalar>(-1.0)};

    // Hole Seeking
    bool use_hole_seeking = true;
    Scalar hole_seeking_weight = static_cast<Scalar>(0.3);

    // Weights & Constraints
    Scalar attraction_weight = static_cast<Scalar>(0.1);
    Scalar alignment_weight = static_cast<Scalar>(0.5);
    Scalar escape_radius_multiplier = static_cast<Scalar>(1.5);
    Scalar search_radius = static_cast<Scalar>(5.0);
};

template <class VecType>
struct PlacementGuidance {
    using Scalar = typename VecType::Scalar;

    // 1. MANDATORY PHYSICS (Collision Resolution)
    bool is_penetrating = false;
    VecType ejection_vector{};

    // "Wider Space": Alternative translations (e.g. sliding along the wall) to test if ejection fails
    std::vector<VecType> alternative_translations;

    // 2. SOFT HEURISTICS (Packing & User Preferences)
    VecType suggested_translation{};
    Scalar suggested_rotation_rad = 0.0;

    // "Wider Space": Mating angles for the 2nd, 3rd, etc. longest edges
    std::vector<Scalar> alternative_rotations;

    Scalar clearance = std::numeric_limits<Scalar>::max();
};

// -------------------------------------------------------------------------
// LOCAL GUIDANCE ENGINE
// -------------------------------------------------------------------------
template <class VecType>
inline PlacementGuidance<VecType> evaluate_local_placement(
    int placed_poly_idx,
    const std::vector<SolidGeometry<VecType>>& all_polygons,
    const VecType& current_position,
    const GuidanceConfig<VecType>& config = GuidanceConfig<VecType>{}
) {
    using Scalar = typename VecType::Scalar;
    PlacementGuidance<VecType> guidance;

    const auto& placed_poly = all_polygons[placed_poly_idx];
    const auto& placed_bounds = placed_poly.get_bounding_circle();
    Scalar placed_radius = std::sqrt(placed_bounds.square_radius());

    std::vector<int> active_indices = { placed_poly_idx };
    auto results = find_polygon_distances<VecType>(all_polygons, active_indices, config.search_radius);

    if (results.empty()) {
        if (config.use_target_attractor) {
            guidance.suggested_translation = (config.target_position - current_position) * config.attraction_weight;
            guidance.suggested_rotation_rad = config.target_angle_rad;
        } else if (config.use_gravity) {
            guidance.suggested_translation = config.gravity_vector;
        }
        return guidance;
    }

    VecType total_ejection{};
    VecType max_mtv{};
    Scalar max_mtv_sq = 0;
    VecType alignment_normal{};
    int penetration_count = 0;
    bool is_deep_swallowed = false;

    // 1. Aggregate Physics Feedback with CLEARANCE GAP
    for (const auto& res : results) {
        if (res.intersect) {
            guidance.is_penetrating = true;
            penetration_count++;

            // Handle Deep Swallow
            if (res.penetration_sq == std::numeric_limits<Scalar>::max()) {
                is_deep_swallowed = true;
                const auto& obstacle_bounds = all_polygons[res.polyB_idx].get_bounding_circle();
                VecType push_dir = placed_bounds.center() - obstacle_bounds.center();

                Scalar dir_len_sq = push_dir.len_sq();
                if (dir_len_sq < static_cast<Scalar>(1e-8)) {
                    push_dir = { static_cast<Scalar>(1.0), static_cast<Scalar>(0.0) };
                    dir_len_sq = static_cast<Scalar>(1.0);
                }

                // Add Minimum Placing Distance to the escape radius!
                Scalar push_mag = (placed_radius * config.escape_radius_multiplier) + config.minimum_placing_distance;
                VecType fake_mtv = push_dir * (push_mag / std::sqrt(dir_len_sq));
                total_ejection = total_ejection + fake_mtv;

                guidance.alternative_translations.push_back({ fake_mtv[0], -fake_mtv[1] });
                guidance.alternative_translations.push_back({ -fake_mtv[0], fake_mtv[1] });
                guidance.alternative_translations.push_back({ -fake_mtv[0], -fake_mtv[1] });

            } else {
                VecType localized_mtv = res.mtv;
                if (res.polyB_idx == placed_poly_idx) {
                    localized_mtv = -localized_mtv;
                }

                // INFLATE MTV BY MINIMUM PLACING DISTANCE
                Scalar mtv_len_sq = localized_mtv.len_sq();
                if (mtv_len_sq > 1e-8 && config.minimum_placing_distance > 0) {
                    Scalar mtv_len = std::sqrt(mtv_len_sq);
                    VecType n = localized_mtv * (static_cast<Scalar>(1.0) / mtv_len);
                    localized_mtv = localized_mtv + (n * config.minimum_placing_distance);
                    mtv_len_sq = localized_mtv.len_sq(); // Update length tracking
                }

                total_ejection = total_ejection + localized_mtv;

                if (mtv_len_sq > max_mtv_sq) {
                    max_mtv_sq = mtv_len_sq;
                    max_mtv = localized_mtv;
                    alignment_normal = localized_mtv;
                }
            }
        } else if (!guidance.is_penetrating) {
            if (res.distance_sq < guidance.clearance * guidance.clearance) {
                guidance.clearance = std::sqrt(static_cast<double>(res.distance_sq));
            }
        }
    }

    // 2. Formulate Mandatory Ejection & Smart Sliding (Wider Space)
    if (guidance.is_penetrating) {
        VecType average_mtv = total_ejection * (static_cast<Scalar>(1.0) / penetration_count);
        guidance.ejection_vector = (average_mtv + max_mtv) * static_cast<Scalar>(0.5);

        // WIDER SPACE: Tangential Sliding
        // If pushing straight out fails, generate escapes that slide along the wall
        if (max_mtv_sq > 1e-8) {
            VecType n = max_mtv * (static_cast<Scalar>(1.0) / std::sqrt(max_mtv_sq));
            VecType tangent = { -n[1], n[0] };

            Scalar slide_mag = placed_radius * config.slide_escape_multiplier;

            // Try sliding left along the wall
            guidance.alternative_translations.push_back(guidance.ejection_vector + (tangent * slide_mag));
            // Try sliding right along the wall
            guidance.alternative_translations.push_back(guidance.ejection_vector - (tangent * slide_mag));
        }
    }

    // 3. Formulate Soft Translation & Hole Seeking
    if (!guidance.is_penetrating) {
        VecType soft_translation{};
        bool hole_found = false;

        if (config.use_hole_seeking) {
            Scalar closest_hole_dist_sq = std::numeric_limits<Scalar>::max();
            VecType best_hole_center{};

            for (size_t i = 0; i < all_polygons.size(); ++i) {
                if (i == static_cast<size_t>(placed_poly_idx)) continue;

                const auto& poly = all_polygons[i];
                for (size_t part = 0; part < poly.line_parts.size(); ++part) {
                    if (!poly.line_parts[part].is_subtractive) continue;

                    const auto& hole_bounds = poly.line_parts[part].bounding_circle;
                    Scalar hole_radius = std::sqrt(hole_bounds.square_radius());

                    // Hole must be big enough to fit the shape + the minimum clearance gap
                    if (hole_radius >= (placed_radius + config.minimum_placing_distance) * static_cast<Scalar>(0.9)) {
                        Scalar dist_sq = (hole_bounds.center() - placed_bounds.center()).len_sq();
                        if (dist_sq < closest_hole_dist_sq) {
                            closest_hole_dist_sq = dist_sq;
                            best_hole_center = hole_bounds.center();
                            hole_found = true;
                        }
                    }
                }
            }

            if (hole_found) {
                VecType hole_pull = (best_hole_center - placed_bounds.center()) * config.hole_seeking_weight;
                soft_translation = soft_translation + hole_pull;
            }
        }

        if (config.use_target_attractor && !hole_found) {
            soft_translation = soft_translation + ((config.target_position - current_position) * config.attraction_weight);
        } else if (config.use_gravity) {
            // Apply clearance buffer to gravity so it stops BEFORE it hits the wall
            Scalar gravity_scale = std::min(static_cast<Scalar>(1.0), std::max(static_cast<Scalar>(0.0), guidance.clearance - config.minimum_placing_distance));
            soft_translation = soft_translation + (config.gravity_vector * gravity_scale);
        }

        guidance.suggested_translation = soft_translation;
    }

    // 4. WIDER SPACE: Smart Multi-Edge Mating
    if (guidance.is_penetrating && alignment_normal.len_sq() > 1e-6 && config.max_alternative_angles > 0) {
        Scalar inv_len = static_cast<Scalar>(1.0) / std::sqrt(static_cast<double>(alignment_normal.len_sq()));
        VecType n = alignment_normal * inv_len;
        VecType tangent = { -n[1], n[0] };

        // Collect ALL edges and sort them by length
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

        std::sort(edges.begin(), edges.end(), [](const EdgeData& a, const EdgeData& b) {
            return a.len_sq > b.len_sq; // Sort descending
        });

        // Generate a mathematically perfect mating angle for the top N longest edges
        int angles_generated = 0;
        for (const auto& edge : edges) {
            if (angles_generated >= config.max_alternative_angles) break;
            if (edge.len_sq < 1e-6) continue;

            Scalar dot = edge.vec.dp(tangent);
            Scalar cross = (edge.vec[0] * tangent[1]) - (edge.vec[1] * tangent[0]);

            double base_angle = std::atan2(cross, dot);
            while (base_angle > M_PI / 2.0) base_angle -= M_PI;
            while (base_angle < -M_PI / 2.0) base_angle += M_PI;

            if (angles_generated == 0) {
                // The absolute longest edge goes into the primary suggestion
                if (config.use_target_attractor) {
                    guidance.suggested_rotation_rad = (base_angle * config.alignment_weight) +
                                                      (config.target_angle_rad * (1.0 - config.alignment_weight));
                } else {
                    guidance.suggested_rotation_rad = base_angle;
                }
            } else {
                // The 2nd, 3rd, etc. longest edges go into the alternatives vector
                guidance.alternative_rotations.push_back(base_angle);
            }
            angles_generated++;
        }
    } else if (config.use_target_attractor) {
        guidance.suggested_rotation_rad = config.target_angle_rad;
    }

    return guidance;
}
