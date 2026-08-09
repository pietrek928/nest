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

// Thin SE(2) polish: one cast_slide for TOI/normal, tangent project, Scene authority.
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
    constexpr Scalar kSceneBackoff = static_cast<Scalar>(1e-6);

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
        if (board != nullptr && !is_solid_fully_contained(placed, *board)) {
            return false;
        }
        if (obstacles.empty()) {
            return true;
        }
        return !scene.query_any_violation(placed, margin_sq, distance_margin);
    };

    SolidGeometry<VecType> start =
        part.rotate(pose_theta).translate(VecType({pose_x, pose_y}));
    (void)start;

    const VecType start_c = solid_centroid(
        part.rotate(pose_theta).translate(VecType({pose_x, pose_y})));
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

    auto try_accept = [&](
        Scalar x, Scalar y, Scalar theta, Scalar travel_score
    ) -> bool {
        SolidGeometry<VecType> placed =
            part.rotate(theta).translate(VecType({x, y}));
        if (!pose_ok(placed)) {
            return false;
        }
        if (pole_mode) {
            const VecType pc = solid_centroid(placed);
            const Scalar dx = pc[0] - (*pole)[0];
            const Scalar dy = pc[1] - (*pole)[1];
            const Scalar d = static_cast<Scalar>(
                std::sqrt(static_cast<double>(dx * dx + dy * dy)));
            if (d + kImproveEps >= best_pole_dist) {
                return false;
            }
            best_pole_dist = d;
        } else {
            if (travel_score + kImproveEps <= best_slide) {
                return false;
            }
            best_slide = travel_score;
        }
        best_x = x;
        best_y = y;
        best_theta = theta;
        have_best = true;
        return true;
    };

    // Place at TOI-backoff along slide_dir; Scene is authority with one backoff retry.
    auto try_slide_from_cast = [&](
        const SolidGeometry<VecType> &seeded,
        Scalar theta,
        const VecType &slide_dir,
        const auto &cast
    ) {
        Scalar travel = max_t;
        if (cast.intersects_path) {
            travel = std::max(static_cast<Scalar>(0), cast.t_entry - clearance);
        }

        auto place_at = [&](Scalar t, const VecType &dir) -> bool {
            const Scalar x = pose_x + dir[0] * t;
            const Scalar y = pose_y + dir[1] * t;
            SolidGeometry<VecType> placed =
                part.rotate(theta).translate(VecType({x, y}));
            if (pose_ok(placed)) {
                return try_accept(x, y, theta, t);
            }
            // Scene reject: back off along inverse slide direction once.
            const Scalar bx = x - dir[0] * kSceneBackoff;
            const Scalar by = y - dir[1] * kSceneBackoff;
            SolidGeometry<VecType> backed =
                part.rotate(theta).translate(VecType({bx, by}));
            if (!pose_ok(backed)) {
                return false;
            }
            return try_accept(bx, by, theta, t);
        };

        if (travel > kEps) {
            // Shrink travel until board/Scene accepts (Scene is authority).
            Scalar t = travel;
            for (int iter = 0; iter < 12 && t > kEps; ++iter) {
                if (place_at(t, slide_dir)) {
                    return;
                }
                t *= static_cast<Scalar>(0.5);
            }
            return;
        }

        // Primary ray jammed at contact: ±tangent from scene normal.
        if (obstacles.empty()) {
            return;
        }
        const auto hits = scene.query_placed(seeded, clearance);
        Scalar best_d = std::numeric_limits<Scalar>::infinity();
        VecType normal{};
        bool have_n = false;
        for (const auto &h : hits) {
            if (h.intersect) {
                if (h.mtv.len_sq() > kEps) {
                    normal = h.mtv;
                    have_n = true;
                    break;
                }
                continue;
            }
            if (h.distance_sq < best_d && h.closest_normal.len_sq() > kEps) {
                best_d = h.distance_sq;
                normal = h.closest_normal;
                have_n = true;
            }
        }
        if (!have_n) {
            return;
        }
        const Scalar nn = static_cast<Scalar>(
            std::sqrt(static_cast<double>(
                normal[0] * normal[0] + normal[1] * normal[1])));
        if (nn <= kEps) {
            return;
        }
        const VecType nu({normal[0] / nn, normal[1] / nn});
        const VecType t1({-nu[1], nu[0]});
        const VecType t2({nu[1], -nu[0]});
        for (const VecType &td : {t1, t2}) {
            const auto tcast = cast_slide(seeded, obstacles, td, max_t);
            Scalar tt = max_t;
            if (tcast.intersects_path) {
                tt = std::max(static_cast<Scalar>(0), tcast.t_entry - clearance);
            }
            if (tt <= kEps) {
                continue;
            }
            place_at(tt, td);
        }
    };

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
            SolidGeometry<VecType> seeded =
                part.rotate(theta).translate(VecType({pose_x, pose_y}));

            const auto cast = cast_slide(seeded, obstacles, dir, max_t);
            try_slide_from_cast(seeded, theta, dir, cast);
        }
    }

    if (!have_best) {
        return std::nullopt;
    }
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
