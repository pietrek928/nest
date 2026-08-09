#pragma once

// Mapbox polylabel (ISC), adapted to std::vector rings of Vec-like points.
// Algorithm from https://github.com/mapbox/polylabel (v1.1.0 C++ header).
//
// Copyright (c) 2016 Mapbox
//
// Permission to use, copy, modify, and/or distribute this software for any
// purpose with or without fee is hereby granted, provided that the above
// copyright notice and this permission notice appear in all copies.
//
// THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES WITH
// REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY
// AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT,
// INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM
// LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR
// OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
// PERFORMANCE OF THIS SOFTWARE.

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <tuple>
#include <utility>
#include <vector>

// Squared distance from point p to segment a–b.
template <class VecType>
inline typename VecType::Scalar polylabel_seg_dist_sq(
    const VecType &p,
    const VecType &a,
    const VecType &b
) {
    using Scalar = typename VecType::Scalar;
    Scalar x = a[0];
    Scalar y = a[1];
    Scalar dx = b[0] - x;
    Scalar dy = b[1] - y;

    if (dx != static_cast<Scalar>(0) || dy != static_cast<Scalar>(0)) {
        const Scalar t =
            ((p[0] - x) * dx + (p[1] - y) * dy) / (dx * dx + dy * dy);
        if (t > static_cast<Scalar>(1)) {
            x = b[0];
            y = b[1];
        } else if (t > static_cast<Scalar>(0)) {
            x += dx * t;
            y += dy * t;
        }
    }

    dx = p[0] - x;
    dy = p[1] - y;
    return dx * dx + dy * dy;
}

// Signed distance from point to polygon outline (negative if outside).
// rings[0] = outer, rings[1..] = holes.
template <class VecType>
inline typename VecType::Scalar polylabel_point_to_polygon_dist(
    const VecType &point,
    const std::vector<std::vector<VecType>> &rings
) {
    using Scalar = typename VecType::Scalar;
    bool inside = false;
    Scalar min_dist_sq = std::numeric_limits<Scalar>::infinity();

    for (const auto &ring : rings) {
        const std::size_t len = ring.size();
        if (len < 2) {
            continue;
        }
        for (std::size_t i = 0, j = len - 1; i < len; j = i++) {
            const VecType &a = ring[i];
            const VecType &b = ring[j];

            if ((a[1] > point[1]) != (b[1] > point[1])
                && (point[0]
                    < (b[0] - a[0]) * (point[1] - a[1]) / (b[1] - a[1]) + a[0])) {
                inside = !inside;
            }

            min_dist_sq = std::min(min_dist_sq, polylabel_seg_dist_sq(point, a, b));
        }
    }

    const Scalar dist = std::sqrt(min_dist_sq);
    return inside ? dist : -dist;
}

template <class VecType>
struct PolylabelCell {
    using Scalar = typename VecType::Scalar;

    VecType c;
    Scalar h;
    Scalar d;
    Scalar max;

    PolylabelCell(
        const VecType &c_,
        Scalar h_,
        const std::vector<std::vector<VecType>> &rings
    )
        : c(c_),
          h(h_),
          d(polylabel_point_to_polygon_dist(c_, rings)),
          max(d + h * static_cast<Scalar>(std::sqrt(2.0))) {}
};

template <class VecType>
inline PolylabelCell<VecType> polylabel_centroid_cell(
    const std::vector<std::vector<VecType>> &rings
) {
    using Scalar = typename VecType::Scalar;
    Scalar area = static_cast<Scalar>(0);
    Scalar cx = static_cast<Scalar>(0);
    Scalar cy = static_cast<Scalar>(0);
    const auto &ring = rings[0];
    const std::size_t len = ring.size();

    for (std::size_t i = 0, j = len - 1; i < len; j = i++) {
        const VecType &a = ring[i];
        const VecType &b = ring[j];
        const Scalar f = a[0] * b[1] - b[0] * a[1];
        cx += (a[0] + b[0]) * f;
        cy += (a[1] + b[1]) * f;
        area += f * static_cast<Scalar>(3);
    }

    if (area == static_cast<Scalar>(0) || len == 0) {
        return PolylabelCell<VecType>(
            len > 0 ? ring[0] : VecType({0, 0}),
            static_cast<Scalar>(0),
            rings);
    }
    return PolylabelCell<VecType>(
        VecType({cx / area, cy / area}),
        static_cast<Scalar>(0),
        rings);
}

// Pole of inaccessibility for a polygon given as outer + hole rings.
// Returns (x, y, signed distance of the pole to the outline; positive inside).
template <class VecType>
inline std::tuple<
    typename VecType::Scalar,
    typename VecType::Scalar,
    typename VecType::Scalar>
polylabel(
    const std::vector<std::vector<VecType>> &rings,
    typename VecType::Scalar precision = static_cast<typename VecType::Scalar>(1)
) {
    using Scalar = typename VecType::Scalar;

    if (rings.empty() || rings[0].size() < 2) {
        return {static_cast<Scalar>(0), static_cast<Scalar>(0), static_cast<Scalar>(0)};
    }

    const auto &outer = rings[0];
    Scalar min_x = outer[0][0];
    Scalar min_y = outer[0][1];
    Scalar max_x = min_x;
    Scalar max_y = min_y;
    for (const auto &p : outer) {
        min_x = std::min(min_x, p[0]);
        min_y = std::min(min_y, p[1]);
        max_x = std::max(max_x, p[0]);
        max_y = std::max(max_y, p[1]);
    }

    const Scalar width = max_x - min_x;
    const Scalar height = max_y - min_y;
    const Scalar cell_size = std::min(width, height);
    Scalar h = cell_size / static_cast<Scalar>(2);

    auto compare_max = [](const PolylabelCell<VecType> &a,
                          const PolylabelCell<VecType> &b) {
        return a.max < b.max;
    };
    using Queue = std::priority_queue<
        PolylabelCell<VecType>,
        std::vector<PolylabelCell<VecType>>,
        decltype(compare_max)>;
    Queue cell_queue(compare_max);

    if (cell_size == static_cast<Scalar>(0)) {
        const Scalar d = polylabel_point_to_polygon_dist(
            VecType({min_x, min_y}), rings);
        return {min_x, min_y, d};
    }

    for (Scalar x = min_x; x < max_x; x += cell_size) {
        for (Scalar y = min_y; y < max_y; y += cell_size) {
            cell_queue.push(
                PolylabelCell<VecType>(VecType({x + h, y + h}), h, rings));
        }
    }

    auto best_cell = polylabel_centroid_cell(rings);

    {
        PolylabelCell<VecType> bbox_cell(
            VecType({min_x + width / static_cast<Scalar>(2),
                     min_y + height / static_cast<Scalar>(2)}),
            static_cast<Scalar>(0),
            rings);
        if (bbox_cell.d > best_cell.d) {
            best_cell = bbox_cell;
        }
    }

    while (!cell_queue.empty()) {
        auto cell = cell_queue.top();
        cell_queue.pop();

        if (cell.d > best_cell.d) {
            best_cell = cell;
        }

        if (cell.max - best_cell.d <= precision) {
            continue;
        }

        h = cell.h / static_cast<Scalar>(2);
        cell_queue.push(
            PolylabelCell<VecType>(
                VecType({cell.c[0] - h, cell.c[1] - h}), h, rings));
        cell_queue.push(
            PolylabelCell<VecType>(
                VecType({cell.c[0] + h, cell.c[1] - h}), h, rings));
        cell_queue.push(
            PolylabelCell<VecType>(
                VecType({cell.c[0] - h, cell.c[1] + h}), h, rings));
        cell_queue.push(
            PolylabelCell<VecType>(
                VecType({cell.c[0] + h, cell.c[1] + h}), h, rings));
    }

    return {best_cell.c[0], best_cell.c[1], best_cell.d};
}
