#pragma once

#include "solid/solid_geometry.h"
#include "common/geometry_common.h"

#include <cmath>
#include <limits>

template<class VecType>
inline bool is_point_inside_solid_space(const VecType& pt, const SolidGeometry<VecType>& poly) {
    if (!poly.boundary_rings.empty()) {
        int winding = 0;

        for (const auto& ring : poly.boundary_rings) {
            const int n = static_cast<int>(ring.points.size());
            if (n < 3) {
                continue;
            }
            const int delta = ring.is_subtractive ? -1 : 1;

            for (int i = 0; i < n; ++i) {
                const VecType& v1 = ring.points[i];
                const VecType& v2 = ring.points[(i + 1) % n];

                if (((v1[1] > pt[1]) != (v2[1] > pt[1])) &&
                    (pt[0] < (v2[0] - v1[0]) * (pt[1] - v1[1]) / (v2[1] - v1[1]) + v1[0])) {
                    winding += delta;
                }
            }
        }
        // Odd winding: even counts from parallel slab edges must not read as inside.
        return (winding & 1) != 0;
    }

    // Fallback for manually built geometry without stored rings
    int winding = 0;
    for (size_t part = 0; part < poly.line_parts.size(); ++part) {
        const VecType* pts = poly.get_part_points(part);
        int n = poly.get_part_size(part);
        const int delta = poly.line_parts[part].is_subtractive ? -1 : 1;

        for (int i = 0; i < n - 1; ++i) {
            const VecType& v1 = pts[i];
            const VecType& v2 = pts[i + 1];

            if (((v1[1] > pt[1]) != (v2[1] > pt[1])) &&
                (pt[0] < (v2[0] - v1[0]) * (pt[1] - v1[1]) / (v2[1] - v1[1]) + v1[0])) {
                winding += delta;
            }
        }
    }
    return (winding & 1) != 0;
}

template<class VecType>
inline typename VecType::Scalar point_boundary_clearance(
    const VecType& pt,
    const SolidGeometry<VecType>& poly
) {
    using Scalar = typename VecType::Scalar;
    if (!is_point_inside_solid_space(pt, poly)) {
        return static_cast<Scalar>(0);
    }
    Scalar min_dist_sq = std::numeric_limits<Scalar>::infinity();
    VecType closest;
    Scalar dsq;

    auto scan_ring = [&](const std::vector<VecType>& ring_pts) {
        const int n = static_cast<int>(ring_pts.size());
        if (n < 2) {
            return;
        }
        for (int i = 0; i < n; ++i) {
            const VecType& a = ring_pts[i];
            const VecType& b = ring_pts[(i + 1) % n];
            closest_point_on_segment(a, b, pt, closest, dsq);
            if (dsq < min_dist_sq) {
                min_dist_sq = dsq;
            }
        }
    };

    if (!poly.boundary_rings.empty()) {
        for (const auto& ring : poly.boundary_rings) {
            scan_ring(ring.points);
        }
    } else {
        for (size_t part = 0; part < poly.line_parts.size(); ++part) {
            const VecType* pts = poly.get_part_points(part);
            const int n = poly.get_part_size(part);
            if (n < 2) {
                continue;
            }
            for (int i = 0; i < n - 1; ++i) {
                closest_point_on_segment(pts[i], pts[i + 1], pt, closest, dsq);
                if (dsq < min_dist_sq) {
                    min_dist_sq = dsq;
                }
            }
        }
    }
    if (!std::isfinite(static_cast<double>(min_dist_sq))) {
        return static_cast<Scalar>(0);
    }
    return static_cast<Scalar>(std::sqrt(static_cast<double>(min_dist_sq)));
}
