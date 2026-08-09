#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include "solid/solid_geometry.h"

// Andrew's monotone chain. Returns CCW hull verts without a closing duplicate.
// Collinear middle points on an edge are dropped.
template <class VecType>
inline std::vector<VecType> convex_hull_monotone(std::vector<VecType> pts) {
    using Scalar = typename VecType::Scalar;
    if (pts.size() <= 1) {
        return pts;
    }

    auto cross = [](const VecType &o, const VecType &a, const VecType &b) -> Scalar {
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
    };

    std::sort(pts.begin(), pts.end(), [](const VecType &a, const VecType &b) {
        if (a[0] != b[0]) {
            return a[0] < b[0];
        }
        return a[1] < b[1];
    });
    pts.erase(
        std::unique(pts.begin(), pts.end(), [](const VecType &a, const VecType &b) {
            return a[0] == b[0] && a[1] == b[1];
        }),
        pts.end());
    if (pts.size() <= 2) {
        return pts;
    }

    std::vector<VecType> lower;
    lower.reserve(pts.size());
    for (const auto &p : pts) {
        while (lower.size() >= 2 &&
               cross(lower[lower.size() - 2], lower[lower.size() - 1], p) <=
                   static_cast<Scalar>(0)) {
            lower.pop_back();
        }
        lower.push_back(p);
    }

    std::vector<VecType> upper;
    upper.reserve(pts.size());
    for (auto it = pts.rbegin(); it != pts.rend(); ++it) {
        const auto &p = *it;
        while (upper.size() >= 2 &&
               cross(upper[upper.size() - 2], upper[upper.size() - 1], p) <=
                   static_cast<Scalar>(0)) {
            upper.pop_back();
        }
        upper.push_back(p);
    }

    lower.pop_back();
    upper.pop_back();
    lower.insert(lower.end(), upper.begin(), upper.end());
    return lower;
}

template <class VecType>
inline typename VecType::Scalar convex_hull_area_of_points(
    const VecType *pts,
    int n
) {
    using Scalar = typename VecType::Scalar;
    if (pts == nullptr || n < 3) {
        return static_cast<Scalar>(0);
    }
    std::vector<VecType> cloud(pts, pts + n);
    const auto hull = convex_hull_monotone(std::move(cloud));
    if (hull.size() < 3) {
        return static_cast<Scalar>(0);
    }
    Scalar area = static_cast<Scalar>(0);
    const int m = static_cast<int>(hull.size());
    for (int i = 0; i < m; ++i) {
        const auto &p1 = hull[static_cast<std::size_t>(i)];
        const auto &p2 = hull[static_cast<std::size_t>((i + 1) % m)];
        area += p1[0] * p2[1] - p1[1] * p2[0];
    }
    return std::abs(area) * static_cast<Scalar>(0.5);
}

template <class VecType>
inline void append_outer_ring_points(
    const SolidGeometry<VecType> &g,
    std::vector<VecType> &out
) {
    bool any_ring = false;
    for (const auto &ring : g.boundary_rings) {
        if (ring.is_subtractive || ring.points.size() < 2) {
            continue;
        }
        any_ring = true;
        out.insert(out.end(), ring.points.begin(), ring.points.end());
    }
    if (!any_ring && !g.line_points.empty()) {
        out.insert(out.end(), g.line_points.begin(), g.line_points.end());
    }
}

template <class VecType>
inline typename VecType::Scalar solid_convex_hull_area(
    const SolidGeometry<VecType> &g
) {
    std::vector<VecType> pts;
    append_outer_ring_points(g, pts);
    if (pts.empty()) {
        return static_cast<typename VecType::Scalar>(0);
    }
    return convex_hull_area_of_points(pts.data(), static_cast<int>(pts.size()));
}

template <class VecType>
inline typename VecType::Scalar solids_convex_hull_area(
    const SolidGeometry<VecType> *const *solids,
    std::size_t n
) {
    using Scalar = typename VecType::Scalar;
    if (solids == nullptr || n == 0) {
        return static_cast<Scalar>(0);
    }
    std::vector<VecType> pts;
    for (std::size_t i = 0; i < n; ++i) {
        if (solids[i] != nullptr) {
            append_outer_ring_points(*solids[i], pts);
        }
    }
    if (pts.empty()) {
        return static_cast<Scalar>(0);
    }
    return convex_hull_area_of_points(pts.data(), static_cast<int>(pts.size()));
}

template <class VecType>
inline typename VecType::Scalar solids_convex_hull_area(
    const std::vector<SolidGeometry<VecType>> &solids
) {
    std::vector<const SolidGeometry<VecType> *> ptrs;
    ptrs.reserve(solids.size());
    for (const auto &s : solids) {
        ptrs.push_back(&s);
    }
    return solids_convex_hull_area(ptrs.data(), ptrs.size());
}
