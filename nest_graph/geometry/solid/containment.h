#pragma once

#include <cmath>
#include <utility>
#include <vector>
#include <algorithm>

#include "solid/solid_geometry.h"
#include "solid/point_in_solid.h"
#include "common/geometry_common.h"
#include "convex/intersect.h"

// =========================================================================
// PART 1: SWEEP-PHASE CONTAINMENT
// =========================================================================

template <class VecType>
inline bool narrow_phase_contain(
    const VecType* inner_pts, int inner_n,
    const SolidGeometry<VecType>& outer
) {
    if (inner_n <= 0) return false;
    return is_point_inside_solid_space(inner_pts[0], outer);
}

template <class VecType>
inline bool check_mutual_containment(
    const SolidGeometry<VecType>& polyA,
    const SolidGeometry<VecType>& polyB
) {
    if (polyA.line_parts.empty() || polyB.line_parts.empty()) return false;

    for (size_t p_idx = 0; p_idx < polyA.line_parts.size(); ++p_idx) {
        if (polyA.line_parts[p_idx].is_subtractive) continue;
        if (narrow_phase_contain(polyA.get_part_points(p_idx), polyA.get_part_size(p_idx), polyB)) return true;
    }

    for (size_t p_idx = 0; p_idx < polyB.line_parts.size(); ++p_idx) {
        if (polyB.line_parts[p_idx].is_subtractive) continue;
        if (narrow_phase_contain(polyB.get_part_points(p_idx), polyB.get_part_size(p_idx), polyA)) return true;
    }

    return false;
}

template <class VecType>
inline bool global_bounding_circles_overlap(
    const SolidGeometry<VecType>& polyA,
    const SolidGeometry<VecType>& polyB
) {
    return circles_overlap(polyA.get_bounding_circle(), polyB.get_bounding_circle());
}

template <class VecType>
inline bool try_add_containment_collision(
    const SolidGeometry<VecType>& polyA,
    const SolidGeometry<VecType>& polyB
) {
    if (!global_bounding_circles_overlap(polyA, polyB)) return false;
    return check_mutual_containment(polyA, polyB);
}

template <class VecType>
inline void promote_containment_pairs(
    const std::vector<SolidGeometry<VecType>>& polygons,
    const std::vector<std::pair<int, int>>& potential_containments,
    std::vector<std::pair<int, int>>& collisions
) {
    for (const auto& pair_id : potential_containments) {
        if (try_add_containment_collision(polygons[pair_id.first], polygons[pair_id.second])) {
            collisions.push_back(pair_id);
        }
    }
}

template <class VecType>
inline void promote_containment_pairs_bipartite(
    const std::vector<SolidGeometry<VecType>>& setA,
    const std::vector<SolidGeometry<VecType>>& setB,
    const std::vector<std::pair<int, int>>& potential_containments,
    std::vector<std::pair<int, int>>& collisions
) {
    for (const auto& pair_id : potential_containments) {
        if (try_add_containment_collision(setA[pair_id.first], setB[pair_id.second])) {
            collisions.push_back(pair_id);
        }
    }
}

// =========================================================================
// PART 2: STANDALONE CONTAINMENT & BATCH CACHING
// =========================================================================

// CACHED ROUTER: Preserves GJK indices across calls while still dynamically
// swapping arrays to favor gradient climbers.
template <class VecType>
inline bool cached_narrow_phase_intersect(
    const VecType* inner_pts, int inner_n,
    const VecType* outer_pts, int outer_n,
    int& outer_cache
) {
    const bool swapped = (inner_n > outer_n);
    const VecType* p1 = swapped ? outer_pts : inner_pts;
    const int s1 = swapped ? outer_n : inner_n;
    const VecType* p2 = swapped ? inner_pts : outer_pts;
    const int s2 = swapped ? outer_n : inner_n;

    // The inner shape changes every loop, so its cache starts at 0.
    // The outer shape is persistent, so we thread its cache index.
    int it1 = swapped ? outer_cache : 0;
    int it2 = swapped ? 0 : outer_cache;

    if (s1 + s2 > 24) {
        auto res = convex_linestrings_intersect_gjk_gradient<VecType>(p1, s1, p2, s2, it1, it2);
        outer_cache = swapped ? res.it1 : res.it2;
        return res.intersect;
    } else {
        auto res = convex_linestrings_intersect_gjk<VecType>(p1, s1, p2, s2, it1, it2);
        outer_cache = swapped ? res.it1 : res.it2;
        return res.intersect;
    }
}

template <class VecType>
inline bool boundaries_intersect(
    const SolidGeometry<VecType>& poly_a,
    const SolidGeometry<VecType>& poly_b
) {
    using Scalar = typename VecType::Scalar;
    const Scalar touch_eps_sq = static_cast<Scalar>(1e-12);

    for (std::size_t i = 0; i < poly_a.line_parts.size(); ++i) {
        if (poly_a.line_parts[i].is_subtractive) continue;
        const auto& circ_a = poly_a.line_parts[i].bounding_circle;

        for (std::size_t j = 0; j < poly_b.line_parts.size(); ++j) {
            if (poly_b.line_parts[j].is_subtractive) continue;
            if (!circles_overlap(circ_a, poly_b.line_parts[j].bounding_circle)) continue;

            const VecType* pts_a = poly_a.get_part_points(i);
            const int n_a = poly_a.get_part_size(i);
            const VecType* pts_b = poly_b.get_part_points(j);
            const int n_b = poly_b.get_part_size(j);

            for (int k = 0; k < std::max(1, n_a - 1); ++k) {
                const VecType a0 = pts_a[k];
                const VecType a1 = pts_a[std::min(k + 1, n_a - 1)];
                for (int l = 0; l < std::max(1, n_b - 1); ++l) {
                    const VecType b0 = pts_b[l];
                    const VecType b1 = pts_b[std::min(l + 1, n_b - 1)];
                    VecType pa;
                    VecType pb;
                    Scalar dsq = static_cast<Scalar>(0);
                    closest_points_between_segments(a0, a1, b0, b1, pa, pb, dsq);
                    if (dsq <= touch_eps_sq) {
                        return true;
                    }
                }
            }
        }
    }
    return false;
}

template <class VecType>
inline bool is_solid_fully_contained(
    const SolidGeometry<VecType>& inner,
    const SolidGeometry<VecType>& outer,
    std::vector<int>* optional_outer_cache = nullptr
) {
    (void)optional_outer_cache;
    if (!global_bounding_circles_overlap(inner, outer)) return false;

    bool any_inside = false;
    for (size_t i = 0; i < inner.line_parts.size(); ++i) {
        if (inner.line_parts[i].is_subtractive) continue;

        const VecType* inner_pts = inner.get_part_points(i);
        const int inner_n = inner.get_part_size(i);
        if (inner_n <= 0) continue;

        for (int v = 0; v < inner_n; ++v) {
            if (!is_point_inside_solid_space(inner_pts[v], outer)) {
                return false;
            }
            any_inside = true;
        }

        for (int e = 0; e < std::max(0, inner_n - 1); ++e) {
            const VecType mid = (inner_pts[e] + inner_pts[e + 1]) * static_cast<typename VecType::Scalar>(0.5);
            if (!is_point_inside_solid_space(mid, outer)) {
                return false;
            }
        }
    }

    if (!any_inside) return false;
    return !boundaries_intersect(inner, outer);
}

template <class VecType>
inline bool solid_footprint_inside(
    const SolidGeometry<VecType>& inner,
    const SolidGeometry<VecType>& outer
) {
    return is_solid_fully_contained(inner, outer);
}

// -------------------------------------------------------------------------
// CACHE-COHERENT BATCH EVALUATOR
// -------------------------------------------------------------------------
template <class VecType>
inline std::vector<bool> solid_footprint_inside(
    const std::vector<SolidGeometry<VecType>>& inners,
    const SolidGeometry<VecType>& outer
) {
    using Scalar = typename VecType::Scalar;
    int n_inners = static_cast<int>(inners.size());
    std::vector<bool> results(n_inners, false);
    if (n_inners == 0) return results;

    // 1. Spatial Sort Tracker
    struct BatchItem {
        int original_idx;
        Scalar sort_key;
    };

    std::vector<BatchItem> items(n_inners);
    for (int i = 0; i < n_inners; ++i) {
        // Sort by X-axis to group physically adjacent shapes together
        items[i] = { i, inners[i].get_bounding_circle().center()[0] };
    }

    std::sort(items.begin(), items.end(), [](const BatchItem& a, const BatchItem& b) {
        return a.sort_key < b.sort_key;
    });

    // 2. Thread the Cache Array
    // We create an array storing the last known GJK extremal index for EVERY part of the outer boundary.
    std::vector<int> outer_cache(outer.line_parts.size(), 0);

    for (int i = 0; i < n_inners; ++i) {
        int orig_idx = items[i].original_idx;
        results[orig_idx] = is_solid_fully_contained(inners[orig_idx], outer, &outer_cache);
    }

    return results;
}