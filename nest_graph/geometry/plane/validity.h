#pragma once

#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <vector>

#include "common/geometry_common.h"
#include "plane/decompose.h"
#include "plane/plane_geometry.h"
#include "plane/point_in_plane.h"
#include "intersect/polygon_intersect.h"

template <class VecType>
inline bool point_on_segment_inclusive(
    const VecType &a,
    const VecType &b,
    const VecType &p,
    typename VecType::Scalar eps
) {
    using Scalar = typename VecType::Scalar;
    const Scalar cross = (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0]);
    if (cross > eps || cross < -eps) {
        return false;
    }
    const Scalar dot = (p[0] - a[0]) * (b[0] - a[0]) + (p[1] - a[1]) * (b[1] - a[1]);
    if (dot < -eps) {
        return false;
    }
    const Scalar len_sq = (b[0] - a[0]) * (b[0] - a[0]) + (b[1] - a[1]) * (b[1] - a[1]);
    return dot <= len_sq + eps;
}

// True if non-adjacent edges properly cross or improperly touch (vertex-on-edge /
// collinear overlap). Adjacent edges sharing a vertex are ignored by the caller.
template <class VecType>
inline bool segments_have_improper_contact(
    const VecType &a,
    const VecType &b,
    const VecType &c,
    const VecType &d,
    typename VecType::Scalar eps
) {
    using Scalar = typename VecType::Scalar;
    const Scalar o1 = compute_turn_direction(a, b, c, eps);
    const Scalar o2 = compute_turn_direction(a, b, d, eps);
    const Scalar o3 = compute_turn_direction(c, d, a, eps);
    const Scalar o4 = compute_turn_direction(c, d, b, eps);

    if ((o1 > eps && o2 < -eps) || (o1 < -eps && o2 > eps)) {
        if ((o3 > eps && o4 < -eps) || (o3 < -eps && o4 > eps)) {
            return true;
        }
    }

    const Scalar z = static_cast<Scalar>(0);
    auto near0 = [&](Scalar v) { return v <= eps && v >= -eps; };
    if (near0(o1) && point_on_segment_inclusive(a, b, c, eps)) {
        return true;
    }
    if (near0(o2) && point_on_segment_inclusive(a, b, d, eps)) {
        return true;
    }
    if (near0(o3) && point_on_segment_inclusive(c, d, a, eps)) {
        return true;
    }
    if (near0(o4) && point_on_segment_inclusive(c, d, b, eps)) {
        return true;
    }
    (void)z;
    return false;
}

template <class VecType>
inline bool chain_has_proper_crossing(
    const std::vector<VecType> &pts,
    bool closed,
    typename VecType::Scalar eps = static_cast<typename VecType::Scalar>(1e-12)
) {
    const int n = static_cast<int>(pts.size());
    if (n < 2) {
        return false;
    }
    const int n_edges = closed ? n : (n - 1);
    if (n_edges < 2) {
        return false;
    }
    for (int i = 0; i < n_edges; ++i) {
        const VecType &a = pts[static_cast<std::size_t>(i)];
        const VecType &b = pts[static_cast<std::size_t>((i + 1) % n)];
        for (int j = i + 1; j < n_edges; ++j) {
            // Skip adjacent edges (including first/last when closed).
            if (j == i + 1) {
                continue;
            }
            if (closed && i == 0 && j == n_edges - 1) {
                continue;
            }
            const VecType &c = pts[static_cast<std::size_t>(j)];
            const VecType &d = pts[static_cast<std::size_t>((j + 1) % n)];
            // Shared endpoint between non-adjacent edges (repeated vertex).
            if ((a[0] == c[0] && a[1] == c[1]) || (a[0] == d[0] && a[1] == d[1])
                || (b[0] == c[0] && b[1] == c[1]) || (b[0] == d[0] && b[1] == d[1])) {
                return true;
            }
            if (segments_have_improper_contact(a, b, c, d, eps)) {
                return true;
            }
        }
    }
    return false;
}

template <class VecType>
inline bool coords_are_finite(const VecType &p) {
    return std::isfinite(static_cast<double>(p[0]))
        && std::isfinite(static_cast<double>(p[1]));
}

template <class VecType>
inline bool plane_is_valid(const PlaneGeometry<VecType> &mesh) {
    using Scalar = typename VecType::Scalar;
    const Scalar eps = static_cast<Scalar>(1e-12);

    for (const auto &p : mesh.line_points) {
        if (!coords_are_finite(p)) {
            return false;
        }
    }
    for (const auto &ring : mesh.boundary_rings) {
        for (const auto &p : ring.points) {
            if (!coords_are_finite(p)) {
                return false;
            }
        }
    }

    for (std::size_t i = 0; i < mesh.line_parts.size(); ++i) {
        const auto &part = mesh.line_parts[i];
        if (part.role == ChainRole::Point) {
            if (mesh.get_part_size(i) < 1) {
                return false;
            }
            continue;
        }
        if (part.role == ChainRole::Stroke) {
            const int n = mesh.get_part_size(i);
            if (n < 2) {
                return false;
            }
            // OGC: self-crossing LineString may still be is_valid; only finiteness /
            // length checked here.
            continue;
        }
    }

    const auto &rings = mesh.boundary_rings;
    if (rings.empty()) {
        return true;
    }

    for (const auto &ring : rings) {
        if (ring.points.size() < 3) {
            return false;
        }
        if (chain_has_proper_crossing(ring.points, /*closed=*/true, eps)) {
            return false;
        }
    }

    // Inter-ring improper contacts.
    for (std::size_t i = 0; i < rings.size(); ++i) {
        const auto &ri = rings[i].points;
        const int ni = static_cast<int>(ri.size());
        for (std::size_t j = i + 1; j < rings.size(); ++j) {
            const auto &rj = rings[j].points;
            const int nj = static_cast<int>(rj.size());
            for (int ei = 0; ei < ni; ++ei) {
                const VecType &a = ri[static_cast<std::size_t>(ei)];
                const VecType &b = ri[static_cast<std::size_t>((ei + 1) % ni)];
                for (int ej = 0; ej < nj; ++ej) {
                    const VecType &c = rj[static_cast<std::size_t>(ej)];
                    const VecType &d = rj[static_cast<std::size_t>((ej + 1) % nj)];
                    if (segments_have_improper_contact(a, b, c, d, eps)) {
                        return false;
                    }
                }
            }
        }
    }

    // Hole / shell containment: each subtractive ring needs a vertex inside some
    // additive outer; additive shells must not nest inside each other.
    std::vector<std::size_t> outers;
    std::vector<std::size_t> holes;
    for (std::size_t i = 0; i < rings.size(); ++i) {
        if (rings[i].is_subtractive) {
            holes.push_back(i);
        } else {
            outers.push_back(i);
        }
    }
    for (std::size_t hi : holes) {
        bool inside_some = false;
        for (std::size_t oi : outers) {
            PlaneGeometry<VecType> probe;
            probe.add_boundary_ring(rings[oi].points, false);
            if (is_point_inside_plane_space(rings[hi].points[0], probe)) {
                inside_some = true;
                break;
            }
        }
        if (!inside_some) {
            return false;
        }
    }
    for (std::size_t i = 0; i < outers.size(); ++i) {
        for (std::size_t j = 0; j < outers.size(); ++j) {
            if (i == j) {
                continue;
            }
            PlaneGeometry<VecType> probe;
            probe.add_boundary_ring(rings[outers[j]].points, false);
            if (is_point_inside_plane_space(rings[outers[i]].points[0], probe)) {
                return false;
            }
        }
    }
    return true;
}

template <class VecType>
inline bool planes_are_valid(
    const std::vector<const PlaneGeometry<VecType> *> &geoms
) {
    for (const auto *g : geoms) {
        if (g == nullptr) {
            return false;
        }
        if (g->has_non_filled_parts()) {
            throw std::invalid_argument(
                "planes_are_valid: Point/Stroke geometries are not supported");
        }
        if (!plane_is_valid(*g)) {
            return false;
        }
    }
    if (geoms.size() < 2) {
        return true;
    }
    return find_polygon_intersections<VecType>(geoms).empty();
}
