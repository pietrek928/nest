#pragma once

#include <cmath>
#include <cstdint>

#include "se2.h"

/** Contact GRG edge metadata (no SolidGeometry deps — bindable from elem_graph). */

struct ContactEdge {
    int32_t gid_a = -1;
    int32_t gid_b = -1;
    int packed_i = -1;
    int packed_j = -1;
    Se2 relative_pose{};
    float contact_score = 0.f;
    float compactness = 0.f;
    float gci = 0.f;
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

/** GCI = clamp01(α·compactness + β·contact_score); locked α=β=0.5 (Q78). */
inline float gci_surrogate(float compactness, float contact_score, float alpha = 0.5f, float beta = 0.5f) {
    return clamp01(alpha * compactness + beta * contact_score);
}
