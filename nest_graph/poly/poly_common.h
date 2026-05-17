#pragma once

#include <cmath>
#include <utility>

#include "circle.h"
#include "tracer.h"

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
