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

// A3a: optional horizon + winner legs. Defaults off so nfp_lite / Motif stay bit-stable.
struct PolishSe2Opts {
    bool exact_horizon = false;
    bool winner_legs = false;
    int max_legs = 2;
    // Optional telem counter (Python binding may pass nullptr).
    int *se2_legs_out = nullptr;
};

// Core SE(2) polish against a prebuilt Scene (A2a reuse). Obstacles must already
// exclude the moving part (caller responsibility).
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
    const StaticCollisionScene<VecType> &scene,
    const SolidGeometry<VecType> *board,
    const std::vector<VecType> &dirs,
    int n_angles,
    typename VecType::Scalar max_t,
    typename VecType::Scalar min_dist,
    std::string_view mode,
    const VecType *pole,
    PolishSe2Opts opts = {}
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

    const Scalar clearance = min_dist > static_cast<Scalar>(0) ? min_dist : static_cast<Scalar>(0);
    const Scalar margin_sq = clearance * clearance;
    const Scalar distance_margin = clearance;

    auto pose_ok = [&](const SolidGeometry<VecType> &placed) -> bool {
        (void)board;  // board membership via void solids in obstacles
        if (scene.obstacles.empty()) {
            return true;
        }
        return !scene.query_any_violation(placed, margin_sq, distance_margin);
    };

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

    // Bisect largest Scene-clear t in [0, travel_hi]; optionally test hi first.
    auto bisect_clear_t = [&](
        Scalar travel_hi,
        const VecType &slide_dir,
        Scalar theta,
        bool hi_first
    ) -> Scalar {
        if (!(travel_hi > kEps)) {
            return static_cast<Scalar>(0);
        }
        if (hi_first) {
            const Scalar x = pose_x + slide_dir[0] * travel_hi;
            const Scalar y = pose_y + slide_dir[1] * travel_hi;
            SolidGeometry<VecType> placed =
                part.rotate(theta).translate(VecType({x, y}));
            if (pose_ok(placed)) {
                return travel_hi;
            }
        }
        Scalar lo = static_cast<Scalar>(0);
        Scalar hi = travel_hi;
        Scalar best_t = static_cast<Scalar>(0);
        bool any_ok = false;
        for (int iter = 0; iter < 12; ++iter) {
            const Scalar mid = (lo + hi) * static_cast<Scalar>(0.5);
            const Scalar x = pose_x + slide_dir[0] * mid;
            const Scalar y = pose_y + slide_dir[1] * mid;
            SolidGeometry<VecType> placed =
                part.rotate(theta).translate(VecType({x, y}));
            if (pose_ok(placed)) {
                lo = mid;
                best_t = mid;
                any_ok = true;
            } else {
                hi = mid;
            }
        }
        return any_ok ? best_t : static_cast<Scalar>(0);
    };

    // Place at TOI-backoff along slide_dir; Scene is authority with one backoff retry.
    auto try_slide_from_cast = [&](
        const SolidGeometry<VecType> &seeded,
        Scalar theta,
        const VecType &slide_dir,
        const auto &cast,
        Scalar cast_max_t
    ) {
        Scalar travel = cast_max_t;
        if (cast.intersects_path) {
            travel = std::max(static_cast<Scalar>(0), cast.t_entry - clearance);
        }
        if (travel > cast_max_t) {
            travel = cast_max_t;
        }

        auto place_at = [&](Scalar t, const VecType &dir) -> bool {
            const Scalar x = pose_x + dir[0] * t;
            const Scalar y = pose_y + dir[1] * t;
            SolidGeometry<VecType> placed =
                part.rotate(theta).translate(VecType({x, y}));
            if (pose_ok(placed)) {
                return try_accept(x, y, theta, t);
            }
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
            const Scalar best_t = bisect_clear_t(
                travel, slide_dir, theta, /*hi_first=*/opts.exact_horizon);
            if (best_t > kEps) {
                place_at(best_t, slide_dir);
            }
            return;
        }

        // Primary ray jammed at contact: ±tangent from scene normal.
        // (Legacy path; A3a winner_legs runs after the angle loop instead.)
        if (opts.winner_legs && pole_mode) {
            return;
        }
        if (scene.obstacles.empty()) {
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
            const auto tcast = cast_slide(seeded, scene.obstacles, td, cast_max_t);
            Scalar tt = cast_max_t;
            if (tcast.intersects_path) {
                tt = std::max(static_cast<Scalar>(0), tcast.t_entry - clearance);
            }
            if (tt <= kEps) {
                continue;
            }
            const Scalar bt = bisect_clear_t(tt, td, theta, /*hi_first=*/true);
            if (bt > kEps) {
                place_at(bt, td);
            }
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

            Scalar cast_max = max_t;
            if (opts.exact_horizon && pole_mode) {
                const VecType c_seed = solid_centroid(seeded);
                const Scalar hx = (*pole)[0] - c_seed[0];
                const Scalar hy = (*pole)[1] - c_seed[1];
                const Scalar h = hx * dir[0] + hy * dir[1];
                if (h <= kEps) {
                    continue;
                }
                cast_max = std::min(max_t, h);
            }

            const auto cast = cast_slide(seeded, scene.obstacles, dir, cast_max);
            try_slide_from_cast(seeded, theta, dir, cast, cast_max);
        }
    }

    // A3a winner legs (pole only): tangent along contact then re-cast to pole.
    if (opts.winner_legs && pole_mode && !scene.obstacles.empty()) {
        Scalar cur_x = have_best ? best_x : pose_x;
        Scalar cur_y = have_best ? best_y : pose_y;
        Scalar cur_th = have_best ? best_theta : pose_theta;
        int legs = 0;
        const int max_legs = opts.max_legs > 0 ? opts.max_legs : 2;
        while (legs < max_legs) {
            SolidGeometry<VecType> seeded =
                part.rotate(cur_th).translate(VecType({cur_x, cur_y}));
            const auto hits = scene.query_placed(seeded, clearance * static_cast<Scalar>(2));
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
                break;
            }
            const Scalar nn = static_cast<Scalar>(
                std::sqrt(static_cast<double>(
                    normal[0] * normal[0] + normal[1] * normal[1])));
            if (nn <= kEps) {
                break;
            }
            const VecType nu({normal[0] / nn, normal[1] / nn});
            VecType t1({-nu[1], nu[0]});
            VecType t2({nu[1], -nu[0]});
            const VecType pc = solid_centroid(seeded);
            const Scalar to_px = (*pole)[0] - pc[0];
            const Scalar to_py = (*pole)[1] - pc[1];
            // Prefer tangent toward the pole.
            VecType td = t1;
            if (t1[0] * to_px + t1[1] * to_py < t2[0] * to_px + t2[1] * to_py) {
                td = t2;
            }
            Scalar h_tan = to_px * td[0] + to_py * td[1];
            if (h_tan <= kEps) {
                // Near-orthogonal: try both signs with sheet-scale budget.
                h_tan = max_t;
            }
            const Scalar cast_max = opts.exact_horizon ? std::min(max_t, h_tan) : max_t;
            const auto tcast = cast_slide(seeded, scene.obstacles, td, cast_max);
            Scalar tt = cast_max;
            if (tcast.intersects_path) {
                tt = std::max(static_cast<Scalar>(0), tcast.t_entry - clearance);
            }
            if (tt > cast_max) {
                tt = cast_max;
            }
            bool moved = false;
            if (tt > kEps) {
                // Temporarily set pose origin for bisect helper via local place.
                const Scalar save_x = pose_x;
                const Scalar save_y = pose_y;
                pose_x = cur_x;
                pose_y = cur_y;
                const Scalar bt = bisect_clear_t(tt, td, cur_th, /*hi_first=*/true);
                pose_x = save_x;
                pose_y = save_y;
                if (bt > kEps) {
                    const Scalar nx = cur_x + td[0] * bt;
                    const Scalar ny = cur_y + td[1] * bt;
                    const Scalar prev = best_pole_dist;
                    if (try_accept(nx, ny, cur_th, bt)) {
                        moved = true;
                        cur_x = best_x;
                        cur_y = best_y;
                        cur_th = best_theta;
                        (void)prev;
                    }
                }
            }
            // Re-cast toward pole from current best.
            {
                SolidGeometry<VecType> s2 =
                    part.rotate(cur_th).translate(VecType({cur_x, cur_y}));
                const VecType c2 = solid_centroid(s2);
                Scalar pdx = (*pole)[0] - c2[0];
                Scalar pdy = (*pole)[1] - c2[1];
                const Scalar plen = static_cast<Scalar>(
                    std::sqrt(static_cast<double>(pdx * pdx + pdy * pdy)));
                if (plen > kEps) {
                    const VecType pdir({pdx / plen, pdy / plen});
                    Scalar cast_max_p = max_t;
                    if (opts.exact_horizon) {
                        cast_max_p = std::min(max_t, plen);
                    }
                    const auto pcast =
                        cast_slide(s2, scene.obstacles, pdir, cast_max_p);
                    Scalar pt = cast_max_p;
                    if (pcast.intersects_path) {
                        pt = std::max(
                            static_cast<Scalar>(0), pcast.t_entry - clearance);
                    }
                    if (pt > cast_max_p) {
                        pt = cast_max_p;
                    }
                    if (pt > kEps) {
                        const Scalar save_x = pose_x;
                        const Scalar save_y = pose_y;
                        pose_x = cur_x;
                        pose_y = cur_y;
                        const Scalar bt =
                            bisect_clear_t(pt, pdir, cur_th, /*hi_first=*/true);
                        pose_x = save_x;
                        pose_y = save_y;
                        if (bt > kEps) {
                            const Scalar nx = cur_x + pdir[0] * bt;
                            const Scalar ny = cur_y + pdir[1] * bt;
                            if (try_accept(nx, ny, cur_th, bt)) {
                                moved = true;
                                cur_x = best_x;
                                cur_y = best_y;
                                cur_th = best_theta;
                            }
                        }
                    }
                }
            }
            ++legs;
            if (opts.se2_legs_out != nullptr) {
                *opts.se2_legs_out += 1;
            }
            if (!moved) {
                break;
            }
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

// Thin SE(2) polish: builds a Scene from owned obstacles (legacy / Motif default).
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
    std::vector<SolidGeometry<VecType>> obstacles,
    const SolidGeometry<VecType> *board,
    const std::vector<VecType> &dirs,
    int n_angles,
    typename VecType::Scalar max_t,
    typename VecType::Scalar min_dist,
    std::string_view mode,
    const VecType *pole,
    PolishSe2Opts opts = {}
) {
    StaticCollisionScene<VecType> scene;
    scene.build(std::move(obstacles), static_cast<typename VecType::Scalar>(0.5));
    return polish_se2_part(
        part,
        pose_x,
        pose_y,
        pose_theta,
        scene,
        board,
        dirs,
        n_angles,
        max_t,
        min_dist,
        mode,
        pole,
        opts);
}
