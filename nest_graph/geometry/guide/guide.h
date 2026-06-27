#pragma once

#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>
#include <string>

#include "solid/solid_geometry.h"
#include "distance/polygon_distance.h"
#include "polygon_cast.h"

// -------------------------------------------------------------------------
// CONFIGURATION & RESULT STRUCTURES
// -------------------------------------------------------------------------
template <class VecType>
struct GuidanceConfig {
    using Scalar = typename VecType::Scalar;

    // --- Physics & Spacing ---
    Scalar minimum_placing_distance = static_cast<Scalar>(1e-6);

    // --- Tight Packing (Squeeze & Snap) ---
    bool use_tight_packing = true;

    // --- NEW: Corner Alignment ---
    bool use_corner_alignment = true;

    // --- Hole Seeking & Behavior ---
    bool use_hole_seeking = true;
    Scalar max_hole_size_ratio = static_cast<Scalar>(4.0);

    // --- Attractors & Gravity ---
    bool use_gravity = true;
    VecType gravity_vector{static_cast<Scalar>(-1.0), static_cast<Scalar>(-1.0)};
    bool use_target_attractor = false;
    VecType target_position{};
    Scalar target_angle_rad = 0.0;

    // --- PROPOSITION EXPLORATION ---
    int max_propositions = 8;
    Scalar diversity_distance_threshold = static_cast<Scalar>(2.0);
    Scalar diversity_angle_rad_threshold = static_cast<Scalar>(M_PI / 8.0);

    bool enable_grid_exploration = true;
    int max_alternative_angles = 3;
    Scalar slide_escape_multiplier = static_cast<Scalar>(0.8);

    // --- Weights & Limits ---
    Scalar attraction_weight = static_cast<Scalar>(0.1);
    Scalar alignment_weight = static_cast<Scalar>(0.5);
    Scalar escape_radius_multiplier = static_cast<Scalar>(1.5);

    // Absolute cast horizon and neighbor-distance sweep margin (NOT the sweep aura multiplier).
    Scalar search_radius = static_cast<Scalar>(25.0);

    bool use_board_edge_cast = false;
    int max_hole_seeks = 2;
    int max_void_polygons_for_holes = 8;

    bool is_post_rotation_pass = false;
};

template <class VecType>
struct PlacementProposition {
    using Scalar = typename VecType::Scalar;
    VecType translation{};
    Scalar rotation_rad = 0.0;
    Scalar heuristic_score = 0.0;
    std::string move_type = "";
    int closest_poly_idx = -1;

    bool operator<(const PlacementProposition& o) const {
        return heuristic_score > o.heuristic_score;
    }
};

template <class VecType>
struct PlacementGuidance {
    using Scalar = typename VecType::Scalar;
    bool is_penetrating = false;
    Scalar clearance = std::numeric_limits<Scalar>::max();
    std::vector<PlacementProposition<VecType>> propositions;
};

template <class VecType>
struct PhysicsContext {
    using Scalar = typename VecType::Scalar;

    VecType total_ejection{};
    VecType max_mtv{};
    Scalar max_mtv_sq = 0;
    VecType alignment_normal{};
    VecType closest_feature_normal{};
    int penetration_count = 0;

    bool is_penetrating = false;
    Scalar min_clearance = std::numeric_limits<Scalar>::max();

    // Track the absolute closest geometry for precision docking
    VecType closest_obstacle_center{};
    int closest_poly_idx = -1;

    // Second-closest neighbor for dual snap / corner alignment
    VecType second_closest_obstacle_center{};
    int second_closest_poly_idx = -1;
    Scalar second_min_clearance = std::numeric_limits<Scalar>::max();

    std::vector<Scalar> raw_rotations;
};

// -------------------------------------------------------------------------
// PHASE 1: AGGREGATION
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
                VecType push_dir = placed_bounds.center() - obstacle_part.bounding_circle.center();
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
                    ctx.closest_poly_idx = res.polyB_idx;
                    ctx.closest_obstacle_center = obstacle_part.bounding_circle.center();
                }
            } else {
                VecType localized_mtv = res.mtv;
                if (res.polyB_idx == placed_poly_idx) localized_mtv = -localized_mtv;

                Scalar mtv_len_sq = localized_mtv.len_sq();
                if (mtv_len_sq < static_cast<Scalar>(1e-8)) {
                    VecType push_dir = placed_bounds.center() - obstacle_part.bounding_circle.center();
                    if (obstacle_part.is_subtractive) push_dir = -push_dir;

                    if (push_dir.len_sq() < static_cast<Scalar>(1e-8)) {
                        push_dir = { static_cast<Scalar>(1.0), static_cast<Scalar>(0.0) };
                    }
                    Scalar push_len = std::sqrt(push_dir.len_sq());
                    Scalar push_mag = config.minimum_placing_distance + config.escape_radius_multiplier * placed_radius;
                    localized_mtv = push_dir * (push_mag / push_len);
                    mtv_len_sq = localized_mtv.len_sq();
                } else if (config.minimum_placing_distance > 0) {
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
                    ctx.closest_poly_idx = res.polyB_idx;
                    ctx.closest_obstacle_center =
                        all_polygons[res.polyB_idx].line_parts[res.partB_idx].bounding_circle.center();
                }
            }
        } else if (!ctx.is_penetrating) {
            const Scalar dist_sq = res.distance_sq;
            if (dist_sq < ctx.min_clearance * ctx.min_clearance) {
                if (ctx.closest_poly_idx >= 0 && res.polyB_idx != ctx.closest_poly_idx) {
                    ctx.second_min_clearance = ctx.min_clearance;
                    ctx.second_closest_obstacle_center = ctx.closest_obstacle_center;
                    ctx.second_closest_poly_idx = ctx.closest_poly_idx;
                }

                ctx.min_clearance = std::sqrt(static_cast<double>(dist_sq));
                ctx.closest_obstacle_center = all_polygons[res.polyB_idx].line_parts[res.partB_idx].bounding_circle.center();
                ctx.closest_poly_idx = res.polyB_idx;
                if (res.closest_normal.len_sq() > static_cast<Scalar>(1e-8)) {
                    ctx.closest_feature_normal = res.closest_normal;
                }
            } else if (res.polyB_idx != ctx.closest_poly_idx &&
                       dist_sq < ctx.second_min_clearance * ctx.second_min_clearance) {
                ctx.second_min_clearance = std::sqrt(static_cast<double>(dist_sq));
                ctx.second_closest_obstacle_center = all_polygons[res.polyB_idx].line_parts[res.partB_idx].bounding_circle.center();
                ctx.second_closest_poly_idx = res.polyB_idx;
            }
        }
    }

    if (!ctx.is_penetrating && ctx.closest_poly_idx >= 0) {
        if (ctx.closest_feature_normal.len_sq() > static_cast<Scalar>(1e-8)) {
            ctx.alignment_normal = ctx.closest_feature_normal;
        } else {
            VecType align_dir = ctx.closest_obstacle_center - placed_bounds.center();
            if (align_dir.len_sq() > static_cast<Scalar>(1e-8)) {
                ctx.alignment_normal = align_dir;
            }
        }
    }
}

// -------------------------------------------------------------------------
// PHASE 2: MANDATORY EJECTION
// -------------------------------------------------------------------------
template <class VecType>
inline void formulate_mandatory_ejection(
    int placed_poly_idx,
    const std::vector<SolidGeometry<VecType>>& all_polygons,
    const PhysicsContext<VecType>& ctx,
    const SolidGeometry<VecType>& placed_poly,
    const GuidanceConfig<VecType>& config,
    std::vector<PlacementProposition<VecType>>& raw_propositions
) {
    using Scalar = typename VecType::Scalar;

    VecType average_mtv = ctx.total_ejection * (static_cast<Scalar>(1.0) / ctx.penetration_count);
    VecType primary_ejection = (average_mtv + ctx.max_mtv) * static_cast<Scalar>(0.5);

    raw_propositions.push_back({primary_ejection, 0.0, 100.0, "Primary Ejection", -1});

    if (ctx.max_mtv_sq > 1e-8) {
        VecType n = ctx.max_mtv * (static_cast<Scalar>(1.0) / std::sqrt(ctx.max_mtv_sq));
        VecType tangent = { -n[1], n[0] };
        Scalar placed_radius = std::sqrt(placed_poly.get_bounding_circle().square_radius());
        Scalar slide_mag = placed_radius * config.slide_escape_multiplier;

        raw_propositions.push_back({primary_ejection + (tangent * slide_mag), 0.0, 80.0, "Slide Escape L", -1});
        raw_propositions.push_back({primary_ejection - (tangent * slide_mag), 0.0, 80.0, "Slide Escape R", -1});

        Scalar eject_mag = std::sqrt(ctx.max_mtv_sq);
        std::vector<SolidGeometry<VecType>> ejected_polys = all_polygons;
        ejected_polys[placed_poly_idx] =
            ejected_polys[placed_poly_idx].translate(primary_ejection);

        auto try_post_ejection_cast = [&](const VecType& dir, const std::string& label) {
            Scalar dir_len = std::sqrt(dir.len_sq());
            if (dir_len < static_cast<Scalar>(1e-8)) return;
            VecType dir_norm = dir * (static_cast<Scalar>(1.0) / dir_len);
            auto cast_res = find_closest_polygon_cast<VecType>(
                placed_poly_idx, ejected_polys, dir_norm, config.search_radius
            );
            if (!cast_res.intersects_path) return;
            Scalar snap_dist = std::max(
                static_cast<Scalar>(0), cast_res.t_entry - config.minimum_placing_distance
            );
            if (snap_dist < static_cast<Scalar>(1e-5)) return;
            raw_propositions.push_back({
                primary_ejection + (dir_norm * snap_dist),
                0.0,
                88.0,
                label,
                ctx.closest_poly_idx,
            });
        };

        try_post_ejection_cast(n, "Post-Ejection Snap");
        for (size_t i = 0; i < all_polygons.size(); ++i) {
            if (static_cast<int>(i) == placed_poly_idx) continue;
            if (static_cast<int>(i) == ctx.closest_poly_idx) continue;
            const auto& obs_bounds = all_polygons[i].get_bounding_circle();
            VecType toward = obs_bounds.center() - placed_poly.get_bounding_circle().center();
            try_post_ejection_cast(toward, "Post-Ejection Re-seat");
        }
        (void)eject_mag;
    }
}

// -------------------------------------------------------------------------
// PHASE 3: EXACT CASTING TRANSLATION (Shape Cast Powered)
// -------------------------------------------------------------------------
template <class VecType>
inline void collect_edge_endpoints(
    const SolidGeometry<VecType>& poly,
    int max_edges,
    std::vector<VecType>& out_pts
) {
    using Scalar = typename VecType::Scalar;
    struct EdgeData { Scalar len_sq; int start_idx; const VecType* pts; int n_pts; };
    std::vector<EdgeData> edges;

    for (size_t part = 0; part < poly.line_parts.size(); ++part) {
        const VecType* pts = poly.get_part_points(part);
        int n_pts = poly.get_part_size(part);
        for (int i = 0; i < n_pts; ++i) {
            VecType edge = pts[(i + 1) % n_pts] - pts[i];
            edges.push_back({ edge.len_sq(), i, pts, n_pts });
        }
    }

    std::sort(edges.begin(), edges.end(), [](const EdgeData& a, const EdgeData& b) {
        return a.len_sq > b.len_sq;
    });

    auto add_unique = [&](const VecType& p) {
        for (const auto& existing : out_pts) {
            if ((existing - p).len_sq() < static_cast<Scalar>(1e-10)) return;
        }
        out_pts.push_back(p);
    };

    int count = 0;
    for (const auto& edge : edges) {
        if (count >= max_edges) break;
        if (edge.len_sq < static_cast<Scalar>(1e-6)) continue;
        add_unique(edge.pts[edge.start_idx]);
        add_unique(edge.pts[(edge.start_idx + 1) % edge.n_pts]);
        count++;
    }
}

template <class VecType>
inline bool try_cast_move(
    int placed_poly_idx,
    const std::vector<SolidGeometry<VecType>>& all_polygons,
    const VecType& dir_norm,
    typename VecType::Scalar max_t,
    typename VecType::Scalar min_dist,
    typename VecType::Scalar score,
    const std::string& label,
    int neighbor_idx,
    std::vector<PlacementProposition<VecType>>& raw_propositions
) {
    using Scalar = typename VecType::Scalar;
    auto cast_res = find_closest_polygon_cast<VecType>(
        placed_poly_idx, all_polygons, dir_norm, max_t
    );
    if (cast_res.intersects_path && cast_res.t_entry > min_dist) {
        Scalar exact_dist = cast_res.t_entry - min_dist;
        raw_propositions.push_back({dir_norm * exact_dist, 0.0, score, label, neighbor_idx});
        return true;
    }
    return false;
}

template <class VecType>
inline void formulate_exact_casting_translation(
    int placed_poly_idx,
    const std::vector<SolidGeometry<VecType>>& all_polygons,
    const VecType& current_position,
    const PhysicsContext<VecType>& ctx,
    const GuidanceConfig<VecType>& config,
    std::vector<PlacementProposition<VecType>>& raw_propositions
) {
    using Scalar = typename VecType::Scalar;
    const auto& placed_bounds = all_polygons[placed_poly_idx].get_bounding_circle();
    Scalar placed_radius = std::sqrt(placed_bounds.square_radius());

    // 1. EXACT GRAVITY DOCKING
    if (config.use_gravity) {
        Scalar grav_len = std::sqrt(config.gravity_vector.len_sq());
        if (grav_len > 1e-6) {
            VecType g_norm = config.gravity_vector * (static_cast<Scalar>(1.0) / grav_len);

            if (!try_cast_move(
                    placed_poly_idx, all_polygons, g_norm, config.search_radius,
                    config.minimum_placing_distance, 95.0, "Exact Gravity Dock",
                    ctx.closest_poly_idx, raw_propositions)) {
                raw_propositions.push_back({g_norm * config.search_radius, 0.0, 50.0, "Gravity Fall", -1});
            }
        }
    }

    // 1b. BOARD-EDGE CASTS (axis-aligned inward docks for border-focus configs)
    if (config.use_board_edge_cast && config.use_gravity) {
        Scalar grav_len = std::sqrt(config.gravity_vector.len_sq());
        if (grav_len > 1e-6) {
            VecType g_norm = config.gravity_vector * (static_cast<Scalar>(1.0) / grav_len);
            if (std::fabs(static_cast<double>(g_norm[0])) > 1e-6) {
                VecType x_norm{{g_norm[0] > 0 ? static_cast<Scalar>(1.0) : static_cast<Scalar>(-1.0), static_cast<Scalar>(0.0)}};
                try_cast_move(
                    placed_poly_idx, all_polygons, x_norm, config.search_radius,
                    config.minimum_placing_distance, 93.0, "Board Edge Cast X",
                    -1, raw_propositions);
            }
            if (std::fabs(static_cast<double>(g_norm[1])) > 1e-6) {
                VecType y_norm{{static_cast<Scalar>(0.0), g_norm[1] > 0 ? static_cast<Scalar>(1.0) : static_cast<Scalar>(-1.0)}};
                try_cast_move(
                    placed_poly_idx, all_polygons, y_norm, config.search_radius,
                    config.minimum_placing_distance, 93.0, "Board Edge Cast Y",
                    -1, raw_propositions);
            }
        }
    }

    // 2. EXACT NEIGHBOR SNAP (closest + second-closest)
    if (config.use_tight_packing) {
        struct NeighborTarget {
            VecType center;
            Scalar clearance;
            int poly_idx;
        };
        std::vector<NeighborTarget> neighbors;
        if (ctx.closest_poly_idx >= 0 && ctx.min_clearance < config.search_radius) {
            neighbors.push_back({ctx.closest_obstacle_center, ctx.min_clearance, ctx.closest_poly_idx});
        }
        if (ctx.second_closest_poly_idx >= 0 &&
            ctx.second_closest_poly_idx != ctx.closest_poly_idx &&
            ctx.second_min_clearance < config.search_radius) {
            neighbors.push_back({ctx.second_closest_obstacle_center, ctx.second_min_clearance, ctx.second_closest_poly_idx});
        }

        for (const auto& nb : neighbors) {
            VecType squeeze_dir = nb.center - placed_bounds.center();
            Scalar sq_len = std::sqrt(squeeze_dir.len_sq());

            if (sq_len > 1e-6) {
                VecType sq_norm = squeeze_dir * (static_cast<Scalar>(1.0) / sq_len);
                try_cast_move(
                    placed_poly_idx, all_polygons, sq_norm, config.search_radius,
                    config.minimum_placing_distance, 85.0, "Exact Neighbor Snap",
                    nb.poly_idx, raw_propositions);
            }
        }
    }

    // 3. EXACT CORNER ALIGNMENT (edge-endpoint match, top-K edges)
    if (config.use_corner_alignment) {
        std::vector<int> neighbor_idxs;
        if (ctx.closest_poly_idx >= 0 && ctx.min_clearance < config.search_radius) {
            neighbor_idxs.push_back(ctx.closest_poly_idx);
        }
        if (ctx.second_closest_poly_idx >= 0 &&
            ctx.second_closest_poly_idx != ctx.closest_poly_idx &&
            ctx.second_min_clearance < config.search_radius) {
            neighbor_idxs.push_back(ctx.second_closest_poly_idx);
        }

        const auto& placed = all_polygons[placed_poly_idx];
        constexpr int k_corner_edges = 6;

        for (int obstacle_idx : neighbor_idxs) {
            const auto& obstacle = all_polygons[obstacle_idx];

            std::vector<VecType> corner_pts_a;
            std::vector<VecType> corner_pts_b;
            collect_edge_endpoints(placed, k_corner_edges, corner_pts_a);
            collect_edge_endpoints(obstacle, k_corner_edges, corner_pts_b);

            Scalar best_dist_sq = std::numeric_limits<Scalar>::max();
            VecType best_v_norm{};
            Scalar best_v_len = 0;
            bool found_corner = false;

            for (const auto& ptA : corner_pts_a) {
                for (const auto& ptB : corner_pts_b) {
                    VecType diff = ptB - ptA;
                    Scalar d_sq = diff.len_sq();

                    if (d_sq > 1e-8 && d_sq < best_dist_sq && d_sq < (config.search_radius * config.search_radius)) {
                        best_dist_sq = d_sq;
                        Scalar len = std::sqrt(d_sq);
                        best_v_norm = diff * (static_cast<Scalar>(1.0) / len);
                        best_v_len = len;
                        found_corner = true;
                    }
                }
            }

            if (found_corner) {
                auto cast_res = find_closest_polygon_cast<VecType>(
                    placed_poly_idx, all_polygons, best_v_norm, best_v_len + config.minimum_placing_distance
                );

                if (cast_res.intersects_path) {
                    if (cast_res.t_entry >= (best_v_len - config.minimum_placing_distance - 1e-4)) {
                        Scalar exact_snap = std::max(static_cast<Scalar>(0.0), cast_res.t_entry - config.minimum_placing_distance);
                        raw_propositions.push_back({best_v_norm * exact_snap, 0.0, 92.0, "Vertex Corner Match", obstacle_idx});
                    } else if (cast_res.t_entry > config.minimum_placing_distance) {
                        Scalar exact_snap = cast_res.t_entry - config.minimum_placing_distance;
                        raw_propositions.push_back({best_v_norm * exact_snap, 0.0, 82.0, "Corner Match (Intercept)", obstacle_idx});
                    }
                }
            }
        }
    }

    // 4. TARGET ATTRACTOR
    if (config.use_target_attractor) {
        VecType pull = (config.target_position - current_position) * config.attraction_weight;
        raw_propositions.push_back({pull, 0.0, 60.0, "Target Attractor", -1});
    }

    // 5. SMART HOLE SEEKING (budgeted)
    if (config.use_hole_seeking) {
        int void_poly_count = 0;
        for (size_t i = 0; i < all_polygons.size(); ++i) {
            if (i == static_cast<size_t>(placed_poly_idx)) continue;
            for (size_t part = 0; part < all_polygons[i].line_parts.size(); ++part) {
                if (all_polygons[i].line_parts[part].is_subtractive) {
                    void_poly_count++;
                    break;
                }
            }
        }

        if (void_poly_count <= config.max_void_polygons_for_holes) {
            struct HoleCandidate {
                VecType pull_norm;
                Scalar pull_len;
                Scalar score;
                std::string label;
                VecType fallback_pull;
            };
            std::vector<HoleCandidate> hole_candidates;

            for (size_t i = 0; i < all_polygons.size(); ++i) {
                if (i == static_cast<size_t>(placed_poly_idx)) continue;

                for (size_t part = 0; part < all_polygons[i].line_parts.size(); ++part) {
                    if (!all_polygons[i].line_parts[part].is_subtractive) continue;

                    const auto& hole_bounds = all_polygons[i].line_parts[part].bounding_circle;
                    Scalar hole_radius = std::sqrt(hole_bounds.square_radius());

                    if (hole_radius >= placed_radius && hole_radius <= placed_radius * config.max_hole_size_ratio) {
                        Scalar dist_sq = (hole_bounds.center() - placed_bounds.center()).len_sq();

                        if (dist_sq > (hole_radius * hole_radius) * static_cast<Scalar>(0.25)) {
                            VecType pull_dir = hole_bounds.center() - placed_bounds.center();
                            Scalar pull_len = std::sqrt(pull_dir.len_sq());
                            if (pull_len > 1e-6) {
                                VecType pull_norm = pull_dir * (static_cast<Scalar>(1.0) / pull_len);
                                hole_candidates.push_back({
                                    pull_norm,
                                    pull_len,
                                    dist_sq,
                                    "Safe Hole Seek",
                                    pull_dir * config.attraction_weight,
                                });
                            }
                        }
                    }
                }
            }

            std::sort(hole_candidates.begin(), hole_candidates.end(),
                [](const HoleCandidate& a, const HoleCandidate& b) { return a.score < b.score; });

            int seeks = 0;
            for (const auto& hole : hole_candidates) {
                if (seeks >= config.max_hole_seeks) break;
                auto cast_res = find_closest_polygon_cast<VecType>(
                    placed_poly_idx, all_polygons, hole.pull_norm, hole.pull_len
                );

                if (cast_res.intersects_path) {
                    Scalar safe_pull = std::max(static_cast<Scalar>(0), cast_res.t_entry - config.minimum_placing_distance);
                    raw_propositions.push_back({hole.pull_norm * safe_pull, 0.0, 75.0, hole.label, -1});
                } else {
                    raw_propositions.push_back({hole.fallback_pull, 0.0, 65.0, "Hole Seek", -1});
                }
                seeks++;
            }
        }
    }
}

// -------------------------------------------------------------------------
// PHASE 4: SOFT ROTATION
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
// PHASE 5: PROPOSITION SYNTHESIS (Corner Finding)
// -------------------------------------------------------------------------
template <class VecType>
inline std::vector<PlacementProposition<VecType>> synthesize_propositions(
    std::vector<PlacementProposition<VecType>>& raw_props,
    const PhysicsContext<VecType>& ctx,
    const GuidanceConfig<VecType>& config,
    int placed_poly_idx,
    const std::vector<SolidGeometry<VecType>>& all_polygons
) {
    using Scalar = typename VecType::Scalar;
    std::vector<PlacementProposition<VecType>> cross_product_props;

    for (const auto& prop : raw_props) {
        if (ctx.raw_rotations.empty()) {
            cross_product_props.push_back(prop);
        } else {
            PlacementProposition<VecType> p_primary = prop;
            p_primary.rotation_rad = ctx.raw_rotations[0];
            cross_product_props.push_back(p_primary);

            for (size_t i = 1; i < ctx.raw_rotations.size(); ++i) {
                PlacementProposition<VecType> p_alt = prop;
                p_alt.rotation_rad = ctx.raw_rotations[i];
                p_alt.heuristic_score -= static_cast<Scalar>(5.0 * i);
                p_alt.move_type += " (Alt Angle)";
                cross_product_props.push_back(p_alt);
            }
        }
    }

    // EXACT FLOOR WALKING (Edge Casts)
    if (!ctx.is_penetrating && config.enable_grid_exploration && config.use_gravity) {
        Scalar primary_rot = ctx.raw_rotations.empty() ? 0.0 : ctx.raw_rotations[0];
        VecType g_tan = { -config.gravity_vector[1], config.gravity_vector[0] };
        Scalar g_len = std::sqrt(g_tan.len_sq());

        if (g_len > 1e-6) {
            VecType right_norm = g_tan * (static_cast<Scalar>(1.0) / g_len);
            VecType left_norm = -right_norm;

            // Cast perfectly perpendicular to gravity to slide into tight floor corners
            auto cast_right = find_closest_polygon_cast<VecType>(placed_poly_idx, all_polygons, right_norm, config.search_radius);
            if (cast_right.intersects_path && cast_right.t_entry > config.minimum_placing_distance) {
                cross_product_props.push_back({right_norm * (cast_right.t_entry - config.minimum_placing_distance), primary_rot, 90.0, "Floor Walk R", -1});
            }

            auto cast_left = find_closest_polygon_cast<VecType>(placed_poly_idx, all_polygons, left_norm, config.search_radius);
            if (cast_left.intersects_path && cast_left.t_entry > config.minimum_placing_distance) {
                cross_product_props.push_back({left_norm * (cast_left.t_entry - config.minimum_placing_distance), primary_rot, 90.0, "Floor Walk L", -1});
            }
        }
    }

    std::sort(cross_product_props.begin(), cross_product_props.end());

    std::vector<PlacementProposition<VecType>> final_propositions;
    Scalar dist_thresh_sq = config.diversity_distance_threshold * config.diversity_distance_threshold;

    for (const auto& prop : cross_product_props) {
        bool too_similar = false;

        for (const auto& accepted : final_propositions) {
            Scalar dist_sq = (prop.translation - accepted.translation).len_sq();
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
// FALLBACK + MAIN ENGINE ENTRY POINT
// -------------------------------------------------------------------------
template <class VecType>
inline PlacementProposition<VecType> formulate_gravity_fallback(
    int placed_poly_idx,
    const std::vector<SolidGeometry<VecType>>& all_polygons,
    const GuidanceConfig<VecType>& config
) {
    using Scalar = typename VecType::Scalar;
    PlacementProposition<VecType> default_prop;
    default_prop.heuristic_score = 100.0;
    if (config.use_gravity) {
        Scalar grav_len = std::sqrt(static_cast<double>(config.gravity_vector.len_sq()));
        if (grav_len > 1e-6) {
            VecType g_norm = config.gravity_vector * (static_cast<Scalar>(1.0) / grav_len);
            auto cast_floor = find_closest_polygon_cast<VecType>(
                placed_poly_idx, all_polygons, g_norm, config.search_radius * 2.0
            );

            if (cast_floor.intersects_path) {
                default_prop.translation = g_norm * (cast_floor.t_entry - config.minimum_placing_distance);
                default_prop.move_type = "Long Range Gravity Dock";
            } else {
                default_prop.translation = config.gravity_vector * config.search_radius;
                default_prop.move_type = "Free Space Default";
            }
        }
    }
    return default_prop;
}

template <class VecType, class Tracer = DefaultTracer>
inline PlacementGuidance<VecType> evaluate_local_placement(
    int placed_poly_idx,
    const std::vector<SolidGeometry<VecType>>& all_polygons,
    const VecType& current_position,
    const GuidanceConfig<VecType>& config = GuidanceConfig<VecType>{},
    Tracer* tracer = nullptr
) {
    using Scalar = typename VecType::Scalar;
    PlacementGuidance<VecType> guidance;

    std::vector<int> active_indices = { placed_poly_idx };
    auto results = find_polygon_distances<VecType, Tracer>(
        all_polygons,
        active_indices,
        static_cast<Scalar>(0),
        config.search_radius,
        tracer
    );

    if (results.empty()) {
        guidance.propositions.push_back(
            formulate_gravity_fallback<VecType>(placed_poly_idx, all_polygons, config)
        );
        return guidance;
    }

    PhysicsContext<VecType> ctx;
    std::vector<PlacementProposition<VecType>> raw_propositions;

    aggregate_physics_feedback(placed_poly_idx, all_polygons, results, config, ctx);

    guidance.is_penetrating = ctx.is_penetrating;
    guidance.clearance = ctx.min_clearance;

    if (ctx.is_penetrating) {
        formulate_mandatory_ejection(
            placed_poly_idx, all_polygons, ctx, all_polygons[placed_poly_idx], config, raw_propositions
        );
    } else {
        formulate_exact_casting_translation(
            placed_poly_idx, all_polygons, current_position, ctx, config, raw_propositions
        );
    }

    formulate_rotations(ctx, all_polygons[placed_poly_idx], config, ctx);

    if (!config.is_post_rotation_pass && !ctx.is_penetrating) {
        constexpr Scalar rot_decay = static_cast<Scalar>(0.95);
        for (Scalar rot : ctx.raw_rotations) {
            if (std::fabs(static_cast<double>(rot)) <= 1e-6) {
                continue;
            }

            SolidGeometry<VecType> rotated_part = all_polygons[placed_poly_idx];
            rotated_part.rotate(rot, current_position);

            std::vector<SolidGeometry<VecType>> rotated_polys = all_polygons;
            rotated_polys[placed_poly_idx] = rotated_part;

            auto rot_results = find_polygon_distances<VecType, Tracer>(
                rotated_polys,
                active_indices,
                static_cast<Scalar>(0),
                config.search_radius,
                tracer
            );

            PhysicsContext<VecType> rot_ctx;
            aggregate_physics_feedback(placed_poly_idx, rotated_polys, rot_results, config, rot_ctx);

            if (!rot_ctx.is_penetrating) {
                std::vector<PlacementProposition<VecType>> rot_props;
                formulate_exact_casting_translation(
                    placed_poly_idx, rotated_polys, current_position, rot_ctx, config, rot_props
                );
                for (auto prop : rot_props) {
                    prop.rotation_rad += rot;
                    prop.heuristic_score *= rot_decay;
                    if (prop.move_type.find("(Alt Angle)") == std::string::npos) {
                        prop.move_type += " (Alt Angle)";
                    }
                    raw_propositions.push_back(std::move(prop));
                }
            }
        }
    }

    guidance.propositions = synthesize_propositions(
        raw_propositions, ctx, config, placed_poly_idx, all_polygons
    );

    if (guidance.propositions.empty()) {
        guidance.propositions.push_back(
            formulate_gravity_fallback<VecType>(placed_poly_idx, all_polygons, config)
        );
    }

    return guidance;
}
