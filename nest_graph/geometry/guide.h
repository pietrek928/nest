#pragma once

#include <vector>
#include <cmath>
#include <algorithm>

#include "solid_geometry.h"
#include "polygon_distance.h"

// -------------------------------------------------------------------------
// CONFIGURATION & RESULT STRUCTURES
// -------------------------------------------------------------------------
template <class VecType>
struct GuidanceConfig {
    using Scalar = typename VecType::Scalar;

    // Optional user-defined targets (Attractors)
    bool use_target_attractor = false;
    VecType target_position{};
    Scalar target_angle_rad = 0.0;

    // Gravity / Packing heuristics
    bool use_gravity = true;
    VecType gravity_vector{static_cast<Scalar>(-1.0), static_cast<Scalar>(-1.0)}; // Bottom-Left bias

    // Weights to help the solver blend the hints
    Scalar attraction_weight = static_cast<Scalar>(0.1);
    Scalar alignment_weight = static_cast<Scalar>(0.5);

    Scalar search_radius = static_cast<Scalar>(5.0);
};

template <class VecType>
struct PlacementGuidance {
    using Scalar = typename VecType::Scalar;

    // 1. MANDATORY PHYSICS (Collision Resolution)
    bool is_penetrating = false;
    VecType ejection_vector{};

    // 2. SOFT HEURISTICS (Packing & User Preferences)
    VecType suggested_translation{};      // Combination of gravity and target attraction
    Scalar suggested_rotation_rad = 0.0;  // Twist to mate edges or match target

    Scalar clearance = std::numeric_limits<Scalar>::max(); // Distance to nearest object
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

    // 1. Broad-Phase + Narrow-Phase Sweep
    std::vector<int> active_indices = { placed_poly_idx };
    auto results = find_polygon_distances<VecType>(all_polygons, active_indices, config.search_radius);

    if (results.empty()) {
        // If floating in empty space, just return the soft packing heuristics
        if (config.use_target_attractor) {
            guidance.suggested_translation = (config.target_position - current_position) * config.attraction_weight;
            guidance.suggested_rotation_rad = config.target_angle_rad;
        } else if (config.use_gravity) {
            guidance.suggested_translation = config.gravity_vector;
        }
        return guidance;
    }

    VecType total_ejection{};
    VecType alignment_normal{};
    int penetration_count = 0;

    // 2. Aggregate Physics Feedback
    for (const auto& res : results) {
        if (res.intersect) {
            guidance.is_penetrating = true;
            penetration_count++;

            // Extract MTV (Ensure it points AWAY from the obstacle)
            VecType localized_mtv = res.mtv;
            if (res.polyB_idx == placed_poly_idx) {
                localized_mtv = -localized_mtv;
            }
            total_ejection = total_ejection + localized_mtv;

            // Keep the largest MTV normal for edge mating
            if (localized_mtv.len_sq() > alignment_normal.len_sq()) {
                alignment_normal = localized_mtv;
            }
        } else if (!guidance.is_penetrating) {
            // Track the tightest clearance for snapping
            if (res.distance_sq < guidance.clearance * guidance.clearance) {
                guidance.clearance = std::sqrt(static_cast<double>(res.distance_sq));
            }
        }
    }

    // 3. Formulate the Mandatory Ejection
    if (guidance.is_penetrating) {
        // Average the MTVs to "walk" out of tight corners smoothly
        guidance.ejection_vector = total_ejection * (static_cast<Scalar>(1.0) / penetration_count);
    }

    // 4. Formulate the Soft Translation (Attraction / Gravity)
    if (!guidance.is_penetrating) {
        if (config.use_target_attractor) {
            guidance.suggested_translation = (config.target_position - current_position) * config.attraction_weight;
        } else if (config.use_gravity) {
            // Apply gravity, but scale it down as we get closer to objects to avoid ramming them
            Scalar gravity_scale = std::min(static_cast<Scalar>(1.0), guidance.clearance);
            guidance.suggested_translation = config.gravity_vector * gravity_scale;
        }
    }

    // 5. Formulate the Soft Rotation (Edge Mating vs Target Angle)
    if (guidance.is_penetrating && alignment_normal.len_sq() > 1e-6) {
        // --- Edge Mating Heuristic ---
        // Twist the shape so its longest edge aligns flat against the obstacle
        Scalar inv_len = static_cast<Scalar>(1.0) / std::sqrt(static_cast<double>(alignment_normal.len_sq()));
        VecType n = alignment_normal * inv_len;

        const auto& placed_poly = all_polygons[placed_poly_idx];
        VecType longest_edge{};
        Scalar max_len_sq = 0;

        for (size_t part = 0; part < placed_poly.line_parts.size(); ++part) {
            const VecType* pts = placed_poly.get_part_points(part);
            int n_pts = placed_poly.get_part_size(part);
            for (int i = 0; i < n_pts; ++i) {
                VecType edge = pts[(i + 1) % n_pts] - pts[i];
                Scalar len_sq = edge.len_sq();
                if (len_sq > max_len_sq) {
                    max_len_sq = len_sq;
                    longest_edge = edge;
                }
            }
        }

        if (max_len_sq > 0) {
            // Assuming 2D for rotation math
            VecType tangent = { -n[1], n[0] };

            Scalar dot = longest_edge.dp(tangent);
            Scalar cross = (longest_edge[0] * tangent[1]) - (longest_edge[1] * tangent[0]);

            double angle = std::atan2(cross, dot);
            if (angle > M_PI / 2.0) angle -= M_PI;
            if (angle < -M_PI / 2.0) angle += M_PI;

            // Blend edge-mating with the user's preferred target angle
            if (config.use_target_attractor) {
                guidance.suggested_rotation_rad = (angle * config.alignment_weight) +
                                                  (config.target_angle_rad * (1.0 - config.alignment_weight));
            } else {
                guidance.suggested_rotation_rad = angle;
            }
        }
    } else if (config.use_target_attractor) {
        // If not colliding, simply twist towards the user's target orientation
        guidance.suggested_rotation_rad = config.target_angle_rad;
    }

    return guidance;
}
