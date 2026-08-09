#pragma once

#include <cmath>
#include <limits>
#include <optional>
#include <string_view>
#include <tuple>
#include <vector>

#include "../bindings/geometry_factory.h"
#include "distance/static_collision_scene.h"
#include "solid/containment.h"

// Thin SE(2) polish: cast_slide along dirs (backoff min_dist), board footprint,
// StaticCollisionScene clearance; score by pole distance or slide length.
template <class VecType>
inline std::optional<std::tuple<
    typename VecType::Scalar,
    typename VecType::Scalar,
    typename VecType::Scalar>>
polish_se2_part(
    const SolidGeometry<VecType> &part,
    typename VecType::Scalar pose_x,
    typename VecType::Scalar pose_y,
    typename VecType::Scalar pose_theta,
    const std::vector<SolidGeometry<VecType>> &obstacles,
    const SolidGeometry<VecType> *board,
    const std::vector<VecType> &dirs,
    int n_angles,
    typename VecType::Scalar max_t,
    typename VecType::Scalar min_dist,
    std::string_view mode,
    const VecType *pole
) {
    using Scalar = typename VecType::Scalar;
    constexpr Scalar kEps = static_cast<Scalar>(1e-9);
    constexpr Scalar kImproveEps = static_cast<Scalar>(1e-9);

    if (n_angles < 1) {
        n_angles = 1;
    }
    if (!(max_t > kEps)) {
        return std::nullopt;
    }

    const bool pole_mode = (mode == "pole");
    if (pole_mode && pole == nullptr) {
        return std::nullopt;
    }

    StaticCollisionScene<VecType> scene;
    scene.build(obstacles, static_cast<Scalar>(0.5));
    const Scalar clearance = min_dist > static_cast<Scalar>(0) ? min_dist : static_cast<Scalar>(0);
    const Scalar margin_sq = clearance * clearance;
    const Scalar distance_margin = clearance;

    auto pose_ok = [&](const SolidGeometry<VecType> &placed) -> bool {
        if (board != nullptr && !solid_footprint_inside(placed, *board)) {
            return false;
        }
        if (obstacles.empty()) {
            return true;
        }
        return !scene.query_any_violation(placed, margin_sq, distance_margin);
    };

    SolidGeometry<VecType> start =
        part.rotate(pose_theta).translate(VecType({pose_x, pose_y}));
    if (!pose_ok(start)) {
        // Still allow searching for a clear nearby pose from this seed.
    }

    const VecType start_c = solid_centroid(start);
    Scalar best_pole_dist = std::numeric_limits<Scalar>::infinity();
    if (pole_mode) {
        const Scalar dx0 = start_c[0] - (*pole)[0];
        const Scalar dy0 = start_c[1] - (*pole)[1];
        best_pole_dist = static_cast<Scalar>(
            std::sqrt(static_cast<double>(dx0 * dx0 + dy0 * dy0)));
    }
    Scalar best_slide = static_cast<Scalar>(0);
    bool have_best = false;
    Scalar best_x = pose_x;
    Scalar best_y = pose_y;
    Scalar best_theta = pose_theta;

    const Scalar two_pi =
        static_cast<Scalar>(2) * static_cast<Scalar>(std::acos(static_cast<Scalar>(-1)));

    for (const VecType &raw_dir : dirs) {
        const Scalar dn = static_cast<Scalar>(
            std::sqrt(static_cast<double>(
                raw_dir[0] * raw_dir[0] + raw_dir[1] * raw_dir[1])));
        if (dn < kEps) {
            continue;
        }
        const VecType dir({raw_dir[0] / dn, raw_dir[1] / dn});

        for (int k = 0; k < n_angles; ++k) {
            const Scalar dtheta =
                two_pi * static_cast<Scalar>(k) / static_cast<Scalar>(n_angles);
            const Scalar theta = pose_theta + dtheta;
            SolidGeometry<VecType> rotated = part.rotate(theta);
            SolidGeometry<VecType> seeded =
                rotated.translate(VecType({pose_x, pose_y}));

            auto try_slide_dir = [&](const VecType &slide_dir) {
                const auto cast = cast_slide(
                    seeded, obstacles, slide_dir, max_t);
                Scalar travel = max_t;
                if (cast.intersects_path) {
                    travel = std::max(
                        static_cast<Scalar>(0),
                        cast.t_entry - clearance);
                }
                if (travel <= kEps) {
                    return;
                }
                const int n_grid = 8;
                for (int gi = n_grid; gi >= 1; --gi) {
                    const Scalar t =
                        travel * static_cast<Scalar>(gi) / static_cast<Scalar>(n_grid);
                    SolidGeometry<VecType> placed = rotated.translate(
                        VecType({
                            pose_x + slide_dir[0] * t,
                            pose_y + slide_dir[1] * t}));
                    if (!pose_ok(placed)) {
                        continue;
                    }
                    if (pole_mode) {
                        const VecType pc = solid_centroid(placed);
                        const Scalar dx = pc[0] - (*pole)[0];
                        const Scalar dy = pc[1] - (*pole)[1];
                        const Scalar d = static_cast<Scalar>(
                            std::sqrt(static_cast<double>(dx * dx + dy * dy)));
                        if (d + kImproveEps >= best_pole_dist) {
                            continue;
                        }
                        best_pole_dist = d;
                        best_x = pose_x + slide_dir[0] * t;
                        best_y = pose_y + slide_dir[1] * t;
                        best_theta = theta;
                        have_best = true;
                    } else {
                        if (t + kImproveEps <= best_slide) {
                            continue;
                        }
                        best_slide = t;
                        best_x = pose_x + slide_dir[0] * t;
                        best_y = pose_y + slide_dir[1] * t;
                        best_theta = theta;
                        have_best = true;
                    }
                    break;
                }
            };

            const auto cast = cast_slide(seeded, obstacles, dir, max_t);
            Scalar travel = max_t;
            if (cast.intersects_path) {
                travel = std::max(
                    static_cast<Scalar>(0),
                    cast.t_entry - clearance);
            }
            if (travel > kEps) {
                try_slide_dir(dir);
            } else if (!obstacles.empty()) {
                // Primary ray blocked at contact: slide along ±tangent from
                // nearest obstacle normal (guide Slide Escape pattern).
                const auto hits = scene.query_placed(seeded, clearance);
                Scalar best_d = std::numeric_limits<Scalar>::infinity();
                VecType n{};
                bool have_n = false;
                for (const auto &h : hits) {
                    if (h.intersect) {
                        if (h.mtv.len_sq() > kEps) {
                            n = h.mtv;
                            have_n = true;
                            break;
                        }
                        continue;
                    }
                    if (h.distance_sq < best_d && h.closest_normal.len_sq() > kEps) {
                        best_d = h.distance_sq;
                        n = h.closest_normal;
                        have_n = true;
                    }
                }
                if (have_n) {
                    const Scalar nn = static_cast<Scalar>(
                        std::sqrt(static_cast<double>(n[0] * n[0] + n[1] * n[1])));
                    if (nn > kEps) {
                        const VecType nu({n[0] / nn, n[1] / nn});
                        const VecType t1({-nu[1], nu[0]});
                        const VecType t2({nu[1], -nu[0]});
                        try_slide_dir(t1);
                        try_slide_dir(t2);
                    }
                }
            }
        }
    }

    if (!have_best) {
        return std::nullopt;
    }
    // Normalize angle into [0, 2π).
    Scalar out_theta = best_theta;
    if (std::isfinite(static_cast<double>(out_theta))) {
        double th = std::fmod(
            static_cast<double>(out_theta), static_cast<double>(two_pi));
        if (th < 0.0) {
            th += static_cast<double>(two_pi);
        }
        out_theta = static_cast<Scalar>(th);
    }
    return std::make_tuple(best_x, best_y, out_theta);
}
