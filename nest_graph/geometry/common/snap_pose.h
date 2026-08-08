#pragma once

#include <cmath>
#include <limits>
#include <optional>
#include <tuple>
#include <vector>

#include "../bindings/geometry_factory.h"
#include "solid/containment.h"

// Snap rotated part so nearest outline point sits at min_dist along inward.
// Matches ProposeGeometry Python `_snap_coords_along_exterior_geom` (24-step loop).
template <class VecType>
inline std::optional<std::tuple<
    typename VecType::Scalar,
    typename VecType::Scalar,
    typename VecType::Scalar>>
snap_pose_to_ring(
    const SolidGeometry<VecType> &part,
    const SolidGeometry<VecType> &boundary_ring,
    const VecType &contact,
    const VecType &inward,
    typename VecType::Scalar angle,
    typename VecType::Scalar min_dist,
    const SolidGeometry<VecType> *board = nullptr,
    std::optional<typename VecType::Scalar> standoff_pad = std::nullopt
) {
    using Scalar = typename VecType::Scalar;
    constexpr int kAxisPushSteps = 24;
    constexpr Scalar kGapTol = static_cast<Scalar>(1e-6);
    constexpr Scalar kInLenTol = static_cast<Scalar>(1e-9);

    auto aabb_extent = [](const SolidGeometry<VecType> &g, Scalar &minx, Scalar &miny,
                          Scalar &maxx, Scalar &maxy) -> bool {
        const auto &pts = g.line_points;
        if (pts.empty()) {
            return false;
        }
        minx = pts[0][0];
        miny = pts[0][1];
        maxx = minx;
        maxy = miny;
        for (std::size_t i = 1; i < pts.size(); ++i) {
            const Scalar x = pts[i][0];
            const Scalar y = pts[i][1];
            if (x < minx) {
                minx = x;
            }
            if (y < miny) {
                miny = y;
            }
            if (x > maxx) {
                maxx = x;
            }
            if (y > maxy) {
                maxy = y;
            }
        }
        return true;
    };

    SolidGeometry<VecType> rotated = part.rotate(angle);
    const VecType rcx_vec = solid_centroid(rotated);
    const Scalar rcx = rcx_vec[0];
    const Scalar rcy = rcx_vec[1];

    Scalar pad;
    if (standoff_pad.has_value()) {
        pad = *standoff_pad;
    } else {
        Scalar minx{}, miny{}, maxx{}, maxy{};
        if (!aabb_extent(rotated, minx, miny, maxx, maxy)) {
            return std::nullopt;
        }
        pad = std::max(maxx - minx, maxy - miny) / static_cast<Scalar>(2);
    }

    const auto md = standoff_distance_pair(rotated, boundary_ring);
    const Scalar p_part_x = md.closest_a[0];
    const Scalar p_part_y = md.closest_a[1];
    const Scalar target_x = contact[0] + inward[0] * (min_dist + pad);
    const Scalar target_y = contact[1] + inward[1] * (min_dist + pad);
    SolidGeometry<VecType> placed =
        rotated.translate(VecType({target_x - p_part_x, target_y - p_part_y}));

    const Scalar ilen = static_cast<Scalar>(
        std::sqrt(static_cast<double>(inward[0] * inward[0] + inward[1] * inward[1])));
    if (ilen < kInLenTol) {
        return std::nullopt;
    }
    const Scalar in_x = inward[0] / ilen;
    const Scalar in_y = inward[1] / ilen;

    Scalar minx{}, miny{}, maxx{}, maxy{};
    if (!aabb_extent(rotated, minx, miny, maxx, maxy)) {
        return std::nullopt;
    }
    const Scalar max_cast =
        std::max(maxx - minx, maxy - miny) * static_cast<Scalar>(2)
        + min_dist * static_cast<Scalar>(4);

    std::vector<SolidGeometry<VecType>> ring_obs = {boundary_ring};
    for (int step_i = 0; step_i < kAxisPushSteps; ++step_i) {
        const auto gap_pair = standoff_distance_pair(placed, boundary_ring);
        Scalar gap = static_cast<Scalar>(0);
        if (!gap_pair.core.intersect) {
            gap = static_cast<Scalar>(
                std::sqrt(static_cast<double>(gap_pair.core.distance_sq)));
        }
        const bool inside =
            board == nullptr || solid_footprint_inside(placed, *board);
        if (gap >= min_dist - kGapTol && inside) {
            break;
        }
        const Scalar deficit = min_dist - gap;
        if (deficit > kInLenTol) {
            placed = placed.translate(VecType({in_x * deficit, in_y * deficit}));
        } else {
            const auto cast = cast_slide(
                placed,
                ring_obs,
                VecType({-in_x, -in_y}),
                max_cast);
            Scalar step = min_dist * static_cast<Scalar>(0.2);
            if (cast.intersects_path && cast.t_entry < step) {
                step = std::max(
                    cast.t_entry * static_cast<Scalar>(0.5),
                    min_dist * static_cast<Scalar>(0.05));
            }
            placed = placed.translate(VecType({in_x * step, in_y * step}));
        }
    }

    {
        const auto gap_pair = standoff_distance_pair(placed, boundary_ring);
        Scalar gap = static_cast<Scalar>(0);
        if (!gap_pair.core.intersect) {
            gap = static_cast<Scalar>(
                std::sqrt(static_cast<double>(gap_pair.core.distance_sq)));
        }
        if (gap < min_dist - kGapTol) {
            return std::nullopt;
        }
    }
    if (board != nullptr && !solid_footprint_inside(placed, *board)) {
        return std::nullopt;
    }

    const VecType pc = solid_centroid(placed);
    return std::make_tuple(pc[0] - rcx, pc[1] - rcy, angle);
}
