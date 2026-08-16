#pragma once

#include <cmath>
#include <cstdint>
#include <tuple>

/** SE(2) pose as (x, y, theta). Matches nest_graph.utils: rotate-about-origin then translate. */

struct Se2 {
    float x = 0.f;
    float y = 0.f;
    float a = 0.f;
};

inline Se2 se2_invert(Se2 t) {
    const float c = std::cos(t.a);
    const float s = std::sin(t.a);
    return Se2{-c * t.x - s * t.y, s * t.x - c * t.y, -t.a};
}

/** Compose: apply b first, then a (a ∘ b). */
inline Se2 se2_compose(Se2 a, Se2 b) {
    const float c = std::cos(a.a);
    const float s = std::sin(a.a);
    return Se2{
        a.x + c * b.x - s * b.y,
        a.y + s * b.x + c * b.y,
        a.a + b.a,
    };
}

/** Pose of t in the frame of ref: invert(ref) ∘ t. */
inline Se2 se2_relative(Se2 ref, Se2 t) {
    return se2_compose(se2_invert(ref), t);
}

inline float se2_quantize_coord(float v, float scale = 1000.f) {
    return std::round(v * scale) / scale;
}

/** Round-3 style key (×1000) for motif identity. */
inline Se2 se2_quantize3(Se2 t) {
    return Se2{
        se2_quantize_coord(t.x),
        se2_quantize_coord(t.y),
        se2_quantize_coord(t.a),
    };
}

inline std::tuple<int32_t, int32_t, int32_t> se2_key3(Se2 t) {
    const Se2 q = se2_quantize3(t);
    return {
        static_cast<int32_t>(std::lround(q.x * 1000.f)),
        static_cast<int32_t>(std::lround(q.y * 1000.f)),
        static_cast<int32_t>(std::lround(q.a * 1000.f)),
    };
}
