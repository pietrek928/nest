#pragma once

#include <array>
#include <cmath>
#include <cstdint>

/** Coarse board signature for related-node motif warm-start (not a packing grid). */

struct NodeSignature {
    float rim_fill = 0.f;
    float void_fill = 0.f;
    std::array<uint8_t, 64> occupancy{};  // 8×8
};

inline float related_distance(const NodeSignature &a, const NodeSignature &b) {
    float d = 0.f;
    const float dr = a.rim_fill - b.rim_fill;
    const float dv = a.void_fill - b.void_fill;
    d += dr * dr + dv * dv;
    int ham = 0;
    for (std::size_t i = 0; i < a.occupancy.size(); ++i) {
        if (a.occupancy[i] != b.occupancy[i]) {
            ++ham;
        }
    }
    d += static_cast<float>(ham) / 64.f;
    return std::sqrt(d);
}

/** Mark an AABB cell occupied in the 8×8 grid covering [0,w]×[0,h]. */
inline void signature_mark_aabb(
    NodeSignature &sig,
    float xmin,
    float ymin,
    float xmax,
    float ymax,
    float sheet_w,
    float sheet_h
) {
    if (sheet_w <= 0.f || sheet_h <= 0.f) {
        return;
    }
    const int i0 = std::max(0, std::min(7, static_cast<int>(std::floor(8.f * xmin / sheet_w))));
    const int i1 = std::max(0, std::min(7, static_cast<int>(std::floor(8.f * xmax / sheet_w))));
    const int j0 = std::max(0, std::min(7, static_cast<int>(std::floor(8.f * ymin / sheet_h))));
    const int j1 = std::max(0, std::min(7, static_cast<int>(std::floor(8.f * ymax / sheet_h))));
    for (int j = j0; j <= j1; ++j) {
        for (int i = i0; i <= i1; ++i) {
            sig.occupancy[static_cast<std::size_t>(j * 8 + i)] = 1;
        }
    }
}
