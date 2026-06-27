#pragma once

#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

#include "common/geometry_common.h"
#include "distance/polygon_distance.h"
#include "guide/polygon_cast.h"
#include "solid/decompose.h"
#include "solid/containment.h"
#include "solid/solid_geometry.h"

template <class VecType>
inline VecType solid_centroid(const SolidGeometry<VecType> &solid) {
    using Scalar = typename VecType::Scalar;
    Scalar sx = static_cast<Scalar>(0);
    Scalar sy = static_cast<Scalar>(0);
    std::size_t count = 0;
    for (std::size_t i = 0; i < solid.line_parts.size(); ++i) {
        if (solid.line_parts[i].is_subtractive) {
            continue;
        }
        const VecType *pts = solid.get_part_points(i);
        const int n = solid.get_part_size(i);
        for (int j = 0; j < n; ++j) {
            sx += pts[j][0];
            sy += pts[j][1];
            ++count;
        }
    }
    if (count == 0) {
        const auto &c = solid.get_bounding_circle().center();
        return VecType({c[0], c[1]});
    }
    const Scalar inv = static_cast<Scalar>(1) / static_cast<Scalar>(count);
    return VecType({sx * inv, sy * inv});
}

template <class VecType>
inline SolidGeometry<VecType> solid_from_rings_coords(
    const std::vector<std::vector<VecType>> &rings,
    std::mt19937 &rng
) {
    SolidGeometry<VecType> solid;
    std::size_t added = 0;
    for (const auto &ring_pts : rings) {
        if (ring_pts.size() < 2) {
            continue;
        }
        solid.add_boundary_ring(ring_pts, false);
        std::vector<VecType> closed = ring_pts;
        const VecType &first = closed.front();
        const VecType &last = closed.back();
        if (first[0] != last[0] || first[1] != last[1]) {
            closed.push_back(first);
        }
        process_boundary_to_convex_segments<VecType>(closed, solid, rng, false);
        ++added;
    }
    if (added == 0) {
        throw std::invalid_argument(
            "from_rings: need at least one ring with 2+ points");
    }
    solid.finalize(rng);
    return solid;
}

template <class VecType>
inline SolidGeometry<VecType> solid_from_ring_coords(
    const std::vector<VecType> &ring_pts,
    std::mt19937 &rng
) {
    if (ring_pts.size() < 2) {
        throw std::invalid_argument("from_ring: need at least 2 distinct points");
    }
    return solid_from_rings_coords<VecType>({{ring_pts}}, rng);
}

template <class VecType>
inline std::pair<VecType, VecType> closest_points_between_parts(
    const SolidGeometry<VecType> &poly_a,
    int part_a,
    const SolidGeometry<VecType> &poly_b,
    int part_b
) {
    using Scalar = typename VecType::Scalar;
    const VecType *pts_a = poly_a.get_part_points(part_a);
    const int n_a = poly_a.get_part_size(part_a);
    const VecType *pts_b = poly_b.get_part_points(part_b);
    const int n_b = poly_b.get_part_size(part_b);

    Scalar best_sq = std::numeric_limits<Scalar>::max();
    VecType best_a = pts_a[0];
    VecType best_b = pts_b[0];

    auto consider = [&](const VecType &pa, const VecType &pb) {
        const VecType d = pa - pb;
        const Scalar sq = d.dp(d);
        if (sq < best_sq) {
            best_sq = sq;
            best_a = pa;
            best_b = pb;
        }
    };

    if (n_a == 1 && n_b == 1) {
        consider(pts_a[0], pts_b[0]);
        return {best_a, best_b};
    }

    for (int i = 0; i < std::max(1, n_a - 1); ++i) {
        const VecType a0 = pts_a[i];
        const VecType a1 = pts_a[std::min(i + 1, n_a - 1)];
        for (int j = 0; j < std::max(1, n_b - 1); ++j) {
            const VecType b0 = pts_b[j];
            const VecType b1 = pts_b[std::min(j + 1, n_b - 1)];
            VecType pa;
            VecType pb;
            Scalar dsq = static_cast<Scalar>(0);
            closest_points_between_segments(a0, a1, b0, b1, pa, pb, dsq);
            if (dsq < best_sq) {
                best_sq = dsq;
                best_a = pa;
                best_b = pb;
            }
        }
    }

    return {best_a, best_b};
}

template <class VecType>
struct PairDistanceResult {
    ComplexDistanceResult<VecType> core;
    VecType closest_a;
    VecType closest_b;
};

// GEOS-compatible polygon-outline to open-ring distance (segment-only, no fill).
template <class VecType>
inline PairDistanceResult<VecType> standoff_distance_pair(
    const SolidGeometry<VecType> &part,
    const SolidGeometry<VecType> &ring
) {
    using Scalar = typename VecType::Scalar;
    PairDistanceResult<VecType> out{};
    out.core.polyA_idx = 0;
    out.core.polyB_idx = 1;
    out.core.intersect = false;
    out.core.distance_sq = std::numeric_limits<Scalar>::max();
    out.core.penetration_sq = static_cast<Scalar>(0);
    out.core.mtv = VecType();
    out.closest_a = part.get_bounding_circle().center();
    out.closest_b = ring.get_bounding_circle().center();

    for (std::size_t i = 0; i < part.line_parts.size(); ++i) {
        if (part.line_parts[i].is_subtractive) {
            continue;
        }
        for (std::size_t j = 0; j < ring.line_parts.size(); ++j) {
            if (ring.line_parts[j].is_subtractive) {
                continue;
            }
            auto pts = closest_points_between_parts(
                part, static_cast<int>(i), ring, static_cast<int>(j));
            const VecType d = pts.first - pts.second;
            const Scalar sq = d.dp(d);
            if (sq < out.core.distance_sq) {
                out.core.distance_sq = sq;
                out.closest_a = pts.first;
                out.closest_b = pts.second;
            }
        }
    }

    const Scalar touch_eps_sq = static_cast<Scalar>(1e-12);
    if (out.core.distance_sq <= touch_eps_sq) {
        out.core.intersect = true;
        out.core.distance_sq = static_cast<Scalar>(0);
    }
    return out;
}

template <class VecType>
inline PairDistanceResult<VecType> min_distance_pair(
    const SolidGeometry<VecType> &a,
    const SolidGeometry<VecType> &b,
    typename VecType::Scalar aura = static_cast<typename VecType::Scalar>(0.5)
) {
    using Scalar = typename VecType::Scalar;
    PairDistanceResult<VecType> out{};
    out.core.polyA_idx = 0;
    out.core.polyB_idx = 1;
    out.core.intersect = false;
    out.core.distance_sq = std::numeric_limits<Scalar>::max();
    out.core.penetration_sq = static_cast<Scalar>(0);
    out.core.mtv = VecType();
    out.closest_a = a.get_bounding_circle().center();
    out.closest_b = b.get_bounding_circle().center();

    const auto results = find_polygon_distances<VecType>(
        std::vector<SolidGeometry<VecType>>{a},
        std::vector<SolidGeometry<VecType>>{b},
        aura);

    for (const auto &r : results) {
        if (r.intersect) {
            out.core = r;
            out.core.polyA_idx = 0;
            out.core.polyB_idx = 1;
            auto pts = closest_points_between_parts(a, r.partA_idx, b, r.partB_idx);
            out.closest_a = pts.first;
            out.closest_b = pts.second;
            return out;
        }
        if (r.distance_sq < out.core.distance_sq) {
            out.core = r;
            out.core.polyA_idx = 0;
            out.core.polyB_idx = 1;
            auto pts = closest_points_between_parts(a, r.partA_idx, b, r.partB_idx);
            out.closest_a = pts.first;
            out.closest_b = pts.second;
        }
    }
    return out;
}

template <class VecType>
inline ComplexCastResult<VecType> cast_slide(
    const SolidGeometry<VecType> &active,
    const std::vector<SolidGeometry<VecType>> &obstacles,
    const VecType &slide,
    typename VecType::Scalar max_t
) {
    std::vector<SolidGeometry<VecType>> polys;
    polys.reserve(1 + obstacles.size());
    polys.push_back(active);
    for (const auto &o : obstacles) {
        polys.push_back(o);
    }
    return find_closest_polygon_cast<VecType>(0, polys, slide, max_t);
}

template <class VecType>
inline std::vector<ComplexCastResult<VecType>> cast_slide_all(
    const SolidGeometry<VecType> &active,
    const std::vector<SolidGeometry<VecType>> &obstacles,
    const VecType &slide,
    typename VecType::Scalar max_t
) {
    std::vector<SolidGeometry<VecType>> polys;
    polys.reserve(1 + obstacles.size());
    polys.push_back(active);
    for (const auto &o : obstacles) {
        polys.push_back(o);
    }
    return find_all_polygon_casts<VecType>(0, polys, slide, max_t);
}
