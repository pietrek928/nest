#pragma once

// Arc-length densify of SolidGeometry boundary rings → interleaved xy sites.
// Unifies Python densify_points / exterior perimeter sampling for Voronoi.

#include "solid/solid_geometry.h"

#include <cmath>
#include <cstddef>

// Count sample points along one closed ring (last→first edge included).
//
// Buffer contract:
//   pts: length n (points); pts may be null only when n < 2.
//   return: number of sample *points* (not doubles). Does not allocate.
template <class VecType>
inline std::size_t sample_ring_n(
    const VecType *pts,
    int n,
    double step
) {
    if (pts == nullptr || n < 2 || !(step > 0.0)) {
        return 0;
    }
    std::size_t count = 0;
    for (int i = 0; i < n; ++i) {
        const VecType &a = pts[i];
        const VecType &b = pts[(i + 1) % n];
        const double ax = static_cast<double>(a[0]);
        const double ay = static_cast<double>(a[1]);
        const double bx = static_cast<double>(b[0]);
        const double by = static_cast<double>(b[1]);
        const double dx = bx - ax;
        const double dy = by - ay;
        const double len = std::sqrt(dx * dx + dy * dy);
        if (!(len > 1e-15)) {
            continue;
        }
        count += 1;  // vertex a
        const int n_mid = static_cast<int>(std::floor(len / step));
        for (int k = 1; k <= n_mid; ++k) {
            const double t = (static_cast<double>(k) * step) / len;
            if (t >= 1.0 - 1e-15) {
                break;
            }
            count += 1;
        }
    }
    return count;
}

// Fill sample points along one closed ring (last→first edge included).
//
// Buffer contract:
//   pts: length n (points); closed walk uses last→first.
//   out_xy: capacity out_cap_xy doubles; REQUIRE out_cap_xy >= 2 * sample_ring_n(...).
//   writes: 2 * n_points doubles interleaved; return = written doubles (even).
//   If out_cap_xy too small: write nothing, return 0.
template <class VecType>
inline std::size_t sample_ring_xy(
    const VecType *pts,
    int n,
    double step,
    double *out_xy,
    std::size_t out_cap_xy
) {
    const std::size_t need_pts = sample_ring_n(pts, n, step);
    const std::size_t need_xy = need_pts * 2;
    if (need_xy == 0) {
        return 0;
    }
    if (out_xy == nullptr || out_cap_xy < need_xy) {
        return 0;
    }
    std::size_t w = 0;
    for (int i = 0; i < n; ++i) {
        const VecType &a = pts[i];
        const VecType &b = pts[(i + 1) % n];
        const double ax = static_cast<double>(a[0]);
        const double ay = static_cast<double>(a[1]);
        const double bx = static_cast<double>(b[0]);
        const double by = static_cast<double>(b[1]);
        const double dx = bx - ax;
        const double dy = by - ay;
        const double len = std::sqrt(dx * dx + dy * dy);
        if (!(len > 1e-15)) {
            continue;
        }
        out_xy[w++] = ax;
        out_xy[w++] = ay;
        const int n_mid = static_cast<int>(std::floor(len / step));
        for (int k = 1; k <= n_mid; ++k) {
            const double t = (static_cast<double>(k) * step) / len;
            if (t >= 1.0 - 1e-15) {
                break;
            }
            out_xy[w++] = ax + t * dx;
            out_xy[w++] = ay + t * dy;
        }
    }
    return w;
}

// Count samples on an open polyline (no last→first wrap).
//
// Buffer contract:
//   pts: length n; pts may be null only when n < 2.
//   return: number of sample *points*.
template <class VecType>
inline std::size_t sample_open_polyline_n(
    const VecType *pts,
    int n,
    double step
) {
    if (pts == nullptr || n < 2 || !(step > 0.0)) {
        return 0;
    }
    std::size_t count = 0;
    for (int i = 0; i + 1 < n; ++i) {
        const VecType &a = pts[i];
        const VecType &b = pts[i + 1];
        const double ax = static_cast<double>(a[0]);
        const double ay = static_cast<double>(a[1]);
        const double bx = static_cast<double>(b[0]);
        const double by = static_cast<double>(b[1]);
        const double dx = bx - ax;
        const double dy = by - ay;
        const double len = std::sqrt(dx * dx + dy * dy);
        if (!(len > 1e-15)) {
            continue;
        }
        count += 1;
        const int n_mid = static_cast<int>(std::floor(len / step));
        for (int k = 1; k <= n_mid; ++k) {
            const double t = (static_cast<double>(k) * step) / len;
            if (t >= 1.0 - 1e-15) {
                break;
            }
            count += 1;
        }
    }
    return count;
}

template <class VecType>
inline std::size_t sample_open_polyline_xy(
    const VecType *pts,
    int n,
    double step,
    double *out_xy,
    std::size_t out_cap_xy
) {
    const std::size_t need_xy = sample_open_polyline_n(pts, n, step) * 2;
    if (need_xy == 0) {
        return 0;
    }
    if (out_xy == nullptr || out_cap_xy < need_xy) {
        return 0;
    }
    std::size_t w = 0;
    for (int i = 0; i + 1 < n; ++i) {
        const VecType &a = pts[i];
        const VecType &b = pts[i + 1];
        const double ax = static_cast<double>(a[0]);
        const double ay = static_cast<double>(a[1]);
        const double bx = static_cast<double>(b[0]);
        const double by = static_cast<double>(b[1]);
        const double dx = bx - ax;
        const double dy = by - ay;
        const double len = std::sqrt(dx * dx + dy * dy);
        if (!(len > 1e-15)) {
            continue;
        }
        out_xy[w++] = ax;
        out_xy[w++] = ay;
        const int n_mid = static_cast<int>(std::floor(len / step));
        for (int k = 1; k <= n_mid; ++k) {
            const double t = (static_cast<double>(k) * step) / len;
            if (t >= 1.0 - 1e-15) {
                break;
            }
            out_xy[w++] = ax + t * dx;
            out_xy[w++] = ay + t * dy;
        }
    }
    return w;
}

// Count densify samples over all selected boundary rings.
//
// Buffer contract:
//   return: number of sample *points* (not doubles). Does not allocate.
template <class VecType>
inline std::size_t sample_boundary_rings_n(
    const SolidGeometry<VecType> &poly,
    double step,
    bool exteriors_only
) {
    if (!(step > 0.0)) {
        return 0;
    }
    std::size_t count = 0;
    if (!poly.boundary_rings.empty()) {
        for (const auto &ring : poly.boundary_rings) {
            if (exteriors_only && ring.is_subtractive) {
                continue;
            }
            count += sample_ring_n(
                ring.points.data(),
                static_cast<int>(ring.points.size()),
                step);
        }
        return count;
    }
    for (std::size_t part = 0; part < poly.line_parts.size(); ++part) {
        if (exteriors_only && poly.line_parts[part].is_subtractive) {
            continue;
        }
        const VecType *pts = poly.get_part_points(part);
        const int n = poly.get_part_size(part);
        if (n < 2 || pts == nullptr) {
            continue;
        }
        const double dx = static_cast<double>(pts[0][0] - pts[n - 1][0]);
        const double dy = static_cast<double>(pts[0][1] - pts[n - 1][1]);
        if (dx * dx + dy * dy > 1e-24) {
            count += sample_open_polyline_n(pts, n, step);
        } else {
            count += sample_ring_n(pts, n, step);
        }
    }
    return count;
}

// Fill densify samples into caller buffer.
//
// Buffer contract:
//   out_xy: capacity out_cap_xy doubles;
//           REQUIRE out_cap_xy >= 2 * sample_boundary_rings_n(...).
//   writes: 2 * n_points doubles interleaved; return = written doubles (even).
//   If out_cap_xy too small: write nothing, return 0.
template <class VecType>
inline std::size_t sample_boundary_rings_xy(
    const SolidGeometry<VecType> &poly,
    double step,
    bool exteriors_only,
    double *out_xy,
    std::size_t out_cap_xy
) {
    const std::size_t need_xy = sample_boundary_rings_n(poly, step, exteriors_only) * 2;
    if (need_xy == 0) {
        return 0;
    }
    if (out_xy == nullptr || out_cap_xy < need_xy) {
        return 0;
    }
    std::size_t w = 0;
    if (!poly.boundary_rings.empty()) {
        for (const auto &ring : poly.boundary_rings) {
            if (exteriors_only && ring.is_subtractive) {
                continue;
            }
            const std::size_t wrote = sample_ring_xy(
                ring.points.data(),
                static_cast<int>(ring.points.size()),
                step,
                out_xy + w,
                out_cap_xy - w);
            if (wrote == 0 && sample_ring_n(
                    ring.points.data(),
                    static_cast<int>(ring.points.size()),
                    step) > 0) {
                return 0;
            }
            w += wrote;
        }
        return w;
    }
    for (std::size_t part = 0; part < poly.line_parts.size(); ++part) {
        if (exteriors_only && poly.line_parts[part].is_subtractive) {
            continue;
        }
        const VecType *pts = poly.get_part_points(part);
        const int n = poly.get_part_size(part);
        if (n < 2 || pts == nullptr) {
            continue;
        }
        const double dx = static_cast<double>(pts[0][0] - pts[n - 1][0]);
        const double dy = static_cast<double>(pts[0][1] - pts[n - 1][1]);
        std::size_t wrote = 0;
        if (dx * dx + dy * dy > 1e-24) {
            wrote = sample_open_polyline_xy(pts, n, step, out_xy + w, out_cap_xy - w);
        } else {
            wrote = sample_ring_xy(pts, n, step, out_xy + w, out_cap_xy - w);
        }
        w += wrote;
    }
    return w;
}
