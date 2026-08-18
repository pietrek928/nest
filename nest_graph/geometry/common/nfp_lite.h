#pragma once

#include <cmath>
#include <tuple>
#include <vector>

#include "../bindings/geometry_factory.h"
#include "distance/static_collision_scene.h"
#include "polish_se2.h"
#include "se2.h"

/** NFP-lite inject polish: cast toward anchor + polish_se2. Scene fail-closed (Q138). */
template <class VecType>
inline std::tuple<
    typename VecType::Scalar,
    typename VecType::Scalar,
    typename VecType::Scalar,
    bool>
nfp_lite_relative(
    const SolidGeometry<VecType> &follow,
    typename VecType::Scalar pose_x,
    typename VecType::Scalar pose_y,
    typename VecType::Scalar pose_theta,
    const SolidGeometry<VecType> &anchor,
    const std::vector<SolidGeometry<VecType>> &other_obstacles,
    typename VecType::Scalar min_dist,
    typename VecType::Scalar max_t
) {
    using Scalar = typename VecType::Scalar;
    constexpr Scalar kEps = static_cast<Scalar>(1e-9);

    const Scalar orig_x = pose_x;
    const Scalar orig_y = pose_y;
    const Scalar orig_th = pose_theta;
    bool kept_original = true;

    const VecType ac = solid_centroid(anchor);
    const Scalar ax = ac[0];
    const Scalar ay = ac[1];
    const Scalar dx = ax - pose_x;
    const Scalar dy = ay - pose_y;
    const Scalar dist0 = static_cast<Scalar>(
        std::sqrt(static_cast<double>(dx * dx + dy * dy)));
    if (dist0 < kEps) {
        return {orig_x, orig_y, orig_th, true};
    }
    const Scalar ux = dx / dist0;
    const Scalar uy = dy / dist0;

    std::vector<SolidGeometry<VecType>> obs;
    obs.reserve(other_obstacles.size() + 1);
    obs.push_back(anchor);
    obs.insert(obs.end(), other_obstacles.begin(), other_obstacles.end());

    StaticCollisionScene<VecType> scene;
    scene.build(obs, static_cast<Scalar>(0.5));
    const Scalar clearance = min_dist > static_cast<Scalar>(0) ? min_dist : static_cast<Scalar>(0);
    const Scalar margin_sq = clearance * clearance;
    const Scalar distance_margin = clearance;

    auto pose_ok = [&](const SolidGeometry<VecType> &placed) -> bool {
        if (obs.empty()) {
            return true;
        }
        return !scene.query_any_violation(placed, margin_sq, distance_margin);
    };

    SolidGeometry<VecType> placed0 =
        follow.rotate(pose_theta).translate(VecType({pose_x, pose_y}));
    const VecType slide({ux, uy});
    const auto cast = cast_slide(placed0, obs, slide, max_t);
    if (cast.intersects_path) {
        const Scalar t_hit = cast.t_entry;
        if (std::isfinite(static_cast<double>(t_hit))
            && t_hit > static_cast<Scalar>(0)
            && t_hit < dist0 + static_cast<Scalar>(1)) {
            const Scalar step = std::max(
                static_cast<Scalar>(0),
                t_hit - std::max(clearance, static_cast<Scalar>(1e-4)));
            const Scalar fx2 = pose_x + ux * step;
            const Scalar fy2 = pose_y + uy * step;
            const Scalar d2 = static_cast<Scalar>(
                std::sqrt(static_cast<double>(
                    (ax - fx2) * (ax - fx2) + (ay - fy2) * (ay - fy2))));
            if (d2 <= dist0 + kEps) {
                SolidGeometry<VecType> cast_placed =
                    follow.rotate(pose_theta).translate(VecType({fx2, fy2}));
                if (pose_ok(cast_placed)) {
                    pose_x = fx2;
                    pose_y = fy2;
                    kept_original = false;
                }
            }
        }
    }

    std::vector<VecType> dirs;
    dirs.push_back(VecType({ux, uy}));
    dirs.push_back(VecType({-uy, ux}));
    dirs.push_back(VecType({uy, -ux}));
    const Scalar polish_max = std::min(max_t, std::max(dist0, static_cast<Scalar>(1)));
    const VecType pole({ax, ay});
    const auto polished = polish_se2_part(
        follow,
        pose_x,
        pose_y,
        pose_theta,
        obs,
        static_cast<const SolidGeometry<VecType> *>(nullptr),
        dirs,
        2,
        polish_max,
        min_dist,
        "pole",
        &pole);
    if (polished.has_value()) {
        const Scalar px = std::get<0>(*polished);
        const Scalar py = std::get<1>(*polished);
        const Scalar pth = std::get<2>(*polished);
        const Scalar d1 = static_cast<Scalar>(
            std::sqrt(static_cast<double>(
                (ax - px) * (ax - px) + (ay - py) * (ay - py))));
        if (d1 <= dist0 + static_cast<Scalar>(1e-6)) {
            const bool same =
                px == orig_x && py == orig_y && pth == orig_th;
            return {px, py, pth, same};
        }
    }
    const bool same =
        pose_x == orig_x && pose_y == orig_y && pose_theta == orig_th;
    return {pose_x, pose_y, pose_theta, same || kept_original};
}

template <class VecType>
inline Se2 nfp_lite_pair_relative(
    const SolidGeometry<VecType> &follow,
    Se2 t_a,
    Se2 t_b,
    const SolidGeometry<VecType> &anchor,
    typename VecType::Scalar min_dist
) {
    using Scalar = typename VecType::Scalar;
    const auto world = nfp_lite_relative(
        follow,
        static_cast<Scalar>(t_b.x),
        static_cast<Scalar>(t_b.y),
        static_cast<Scalar>(t_b.a),
        anchor,
        std::vector<SolidGeometry<VecType>>{},
        min_dist,
        static_cast<Scalar>(10));
    const Se2 t_b2{
        static_cast<float>(std::get<0>(world)),
        static_cast<float>(std::get<1>(world)),
        static_cast<float>(std::get<2>(world)),
    };
    return se2_relative(t_a, t_b2);
}
