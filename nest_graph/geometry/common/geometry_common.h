#pragma once

#include <cmath>
#include <utility>

#include <circle.h>
#include "common/tracer.h"

#ifndef NEST_GRAPH_DEFAULT_TRACER
#define NEST_GRAPH_DEFAULT_TRACER NullTracer
#endif
using DefaultTracer = NEST_GRAPH_DEFAULT_TRACER;

inline std::pair<int, int> make_sorted_pair(int a, int b) {
    return (a < b) ? std::make_pair(a, b) : std::make_pair(b, a);
}

template <class VecType>
inline typename VecType::Scalar circle_radius(const Circle<VecType>& c) {
    return static_cast<typename VecType::Scalar>(
        std::sqrt(static_cast<double>(c.square_radius())));
}

template <class VecType>
inline typename VecType::Scalar circle_center_distance_sq(
    const Circle<VecType>& a,
    const Circle<VecType>& b
) {
    return (a.center() - b.center()).len_sq();
}

template <class VecType>
inline bool circles_overlap(
    const Circle<VecType>& a,
    const Circle<VecType>& b,
    typename VecType::Scalar margin = static_cast<typename VecType::Scalar>(0)
) {
    using Scalar = typename VecType::Scalar;
    const Scalar r_sum = circle_radius(a) + circle_radius(b) + margin;
    return circle_center_distance_sq(a, b) <= r_sum * r_sum;
}

template <class VecType>
inline void closest_point_on_segment(
    const VecType &a,
    const VecType &b,
    const VecType &p,
    VecType &out,
    typename VecType::Scalar &dist_sq
) {
    using Scalar = typename VecType::Scalar;
    const VecType ab = b - a;
    const Scalar ab_sq = ab.dp(ab);
    if (ab_sq < static_cast<Scalar>(1e-18)) {
        out = a;
        const VecType d = p - a;
        dist_sq = d.dp(d);
        return;
    }
    Scalar t = (p - a).dp(ab) / ab_sq;
    if (t < static_cast<Scalar>(0)) {
        t = static_cast<Scalar>(0);
    } else if (t > static_cast<Scalar>(1)) {
        t = static_cast<Scalar>(1);
    }
    out = a + ab * t;
    const VecType d = p - out;
    dist_sq = d.dp(d);
}

template <class VecType>
inline void closest_points_between_segments(
    const VecType &a0,
    const VecType &a1,
    const VecType &b0,
    const VecType &b1,
    VecType &out_a,
    VecType &out_b,
    typename VecType::Scalar &dist_sq
) {
    using Scalar = typename VecType::Scalar;
    const VecType da = a1 - a0;
    const VecType db = b1 - b0;
    const VecType r = a0 - b0;
    const Scalar a = da.dp(da);
    const Scalar e = db.dp(db);
    const Scalar f = db.dp(r);

    Scalar s = static_cast<Scalar>(0);
    Scalar t = static_cast<Scalar>(0);

    if (a <= static_cast<Scalar>(1e-18) && e <= static_cast<Scalar>(1e-18)) {
        out_a = a0;
        out_b = b0;
        const VecType d = out_a - out_b;
        dist_sq = d.dp(d);
        return;
    }
    if (a <= static_cast<Scalar>(1e-18)) {
        s = static_cast<Scalar>(0);
        t = std::max(static_cast<Scalar>(0), std::min(static_cast<Scalar>(1), f / e));
    } else {
        const Scalar c = da.dp(r);
        const Scalar denom = a * e - da.dp(db) * da.dp(db);
        if (std::abs(denom) > static_cast<Scalar>(1e-18)) {
            s = std::max(
                static_cast<Scalar>(0),
                std::min(static_cast<Scalar>(1), (da.dp(db) * f - c * e) / denom));
        } else {
            s = static_cast<Scalar>(0);
        }
        t = (da.dp(db) * s + f) / e;
        if (t < static_cast<Scalar>(0)) {
            t = static_cast<Scalar>(0);
            s = std::max(
                static_cast<Scalar>(0),
                std::min(static_cast<Scalar>(1), -c / a));
        } else if (t > static_cast<Scalar>(1)) {
            t = static_cast<Scalar>(1);
            s = std::max(
                static_cast<Scalar>(0),
                std::min(
                    static_cast<Scalar>(1),
                    (da.dp(db) - c) / a));
        }
    }

    out_a = a0 + da * s;
    out_b = b0 + db * t;
    const VecType d = out_a - out_b;
    dist_sq = d.dp(d);
}
