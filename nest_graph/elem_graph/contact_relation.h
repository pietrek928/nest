#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include "se2.h"

#include "convex/hull.h"
#include "distance/polygon_distance.h"
#include "solid/solid_geometry.h"

/** Contact GRG edge: kiss / dist ≤ 2·gap with relative SE2 and GCI surrogate (α=β=0.5). */

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

/**
 * contact_score: 1 at touch/overlap, 0 at dist == contact_thresh.
 * compactness: (area_i+area_j) / convex_hull_area of the pair.
 */
template <class VecType>
inline float pair_compactness(
    const SolidGeometry<VecType> &a,
    const SolidGeometry<VecType> &b
) {
    using Scalar = typename VecType::Scalar;
    const Scalar area_sum = std::abs(a.area()) + std::abs(b.area());
    if (area_sum <= static_cast<Scalar>(0)) {
        return 0.f;
    }
    std::vector<SolidGeometry<VecType>> pair{a, b};
    const Scalar hull = solids_convex_hull_area(pair);
    if (hull <= static_cast<Scalar>(1e-12)) {
        return 0.f;
    }
    return clamp01(static_cast<float>(area_sum / hull));
}

template <class VecType>
inline std::vector<ContactEdge> build_contact_relations(
    const std::vector<SolidGeometry<VecType>> &geoms,
    const std::vector<Se2> &poses,
    const std::vector<int32_t> &gids,
    typename VecType::Scalar gap
) {
    using Scalar = typename VecType::Scalar;
    std::vector<ContactEdge> out;
    const int n = static_cast<int>(geoms.size());
    if (n < 2 || static_cast<int>(poses.size()) != n || static_cast<int>(gids.size()) != n) {
        return out;
    }

    const Scalar contact = static_cast<Scalar>(2) * gap;
    const Scalar contact_eps = contact + static_cast<Scalar>(1e-9);
    const Scalar aura = std::max(contact, static_cast<Scalar>(0.5)) * static_cast<Scalar>(2);

    auto results = find_polygon_distances<VecType>(
        geoms, aura, static_cast<Scalar>(0));

    out.reserve(results.size());
    for (const auto &r : results) {
        const int i = r.polyA_idx;
        const int j = r.polyB_idx;
        if (i < 0 || j < 0 || i >= n || j >= n || i >= j) {
            continue;
        }
        const Scalar dist = r.intersect
            ? static_cast<Scalar>(0)
            : std::sqrt(std::max(r.distance_sq, static_cast<Scalar>(0)));
        if (!r.intersect && dist > contact_eps) {
            continue;
        }

        float contact_score = 1.f;
        if (contact > static_cast<Scalar>(0) && !r.intersect) {
            contact_score = clamp01(1.f - static_cast<float>(dist / contact));
        }

        const float compactness = pair_compactness(geoms[static_cast<std::size_t>(i)], geoms[static_cast<std::size_t>(j)]);
        const float gci = gci_surrogate(compactness, contact_score);

        // Order-invariant: larger area (then smaller gid) as A.
        int ia = i;
        int ib = j;
        const Scalar area_i = std::abs(geoms[static_cast<std::size_t>(i)].area());
        const Scalar area_j = std::abs(geoms[static_cast<std::size_t>(j)].area());
        if (area_j > area_i + static_cast<Scalar>(1e-12)
            || (std::abs(area_j - area_i) <= static_cast<Scalar>(1e-12) && gids[static_cast<std::size_t>(j)] < gids[static_cast<std::size_t>(i)])) {
            ia = j;
            ib = i;
        }

        ContactEdge e;
        e.gid_a = gids[static_cast<std::size_t>(ia)];
        e.gid_b = gids[static_cast<std::size_t>(ib)];
        e.packed_i = ia;
        e.packed_j = ib;
        e.relative_pose = se2_relative(poses[static_cast<std::size_t>(ia)], poses[static_cast<std::size_t>(ib)]);
        e.contact_score = contact_score;
        e.compactness = compactness;
        e.gci = gci;
        out.push_back(e);
    }
    return out;
}
