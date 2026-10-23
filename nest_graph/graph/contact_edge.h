#pragma once

#include <cmath>
#include <cstdint>

#include "se2.h"

/** Contact GRG edge metadata (no SolidGeometry deps — bindable from graph). */

struct ContactEdge {
    int32_t gid_a = -1;
    int32_t gid_b = -1;
    int packed_i = -1;
    int packed_j = -1;
    Se2 relative_pose{};
    float contact_score = 0.f;
    float compactness = 0.f;
    float gci = 0.f;
    /** W2: edge-flush Wang label (soft MIS / Motif adj only). */
    bool flush = false;
    int32_t edge_a = -1;
    int32_t edge_b = -1;
};

inline float clamp01(float v) {
    if (v < 0.f) {
        return 0.f;
    }
    if (v > 1.f) {
        return 1.f;
    }
    return v;
}

/** 8-way Wang edge id from contact direction (atan2). */
inline int32_t wang_edge_id(float dx, float dy) {
    if (std::fabs(dx) < 1e-12f && std::fabs(dy) < 1e-12f) {
        return 0;
    }
    const float a = std::atan2(dy, dx);
    constexpr float kPi = 3.14159265358979323846f;
    int q = static_cast<int>(std::floor((a + kPi) / (kPi * 0.25f)));
    if (q < 0) {
        q = 0;
    }
    return q % 8;
}

/** Parallel / anti-parallel relative θ (edge-aligned kiss). */
inline bool wang_flush_theta(float theta, float tol = 0.35f) {
    constexpr float kPi = 3.14159265358979323846f;
    float w = std::fmod(theta + kPi, 2.f * kPi);
    if (w < 0.f) {
        w += 2.f * kPi;
    }
    w -= kPi;
    const float aw = std::fabs(w);
    const float near0 = std::min(aw, std::fabs(aw - kPi));
    const float near90 = std::fabs(aw - 0.5f * kPi);
    return near0 <= tol || near90 <= tol;
}

/** GCI = clamp01(α·compactness + β·contact_score); flush soft-boosts (W2). */
inline float gci_surrogate(
    float compactness,
    float contact_score,
    float alpha = 0.5f,
    float beta = 0.5f,
    bool flush = false
) {
    float g = clamp01(alpha * compactness + beta * contact_score);
    if (flush) {
        g = clamp01(g + 0.1f);
    }
    return g;
}
