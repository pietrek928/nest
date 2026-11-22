#pragma once

#include <cmath>
#include <utility>
#include <vector>
#include <algorithm>

#include "plane/plane_geometry.h"
#include "plane/point_in_plane.h"
#include "common/geometry_common.h"
#include "convex/intersect.h"

// =========================================================================
// PART 1: SWEEP-PHASE CONTAINMENT
// =========================================================================

template <class VecType>
inline bool narrow_phase_contain(
    const VecType* inner_pts, int inner_n,
    const PlaneGeometry<VecType>& owner,
    const PlaneGeometry<VecType>& outer
) {
    if (inner_n <= 0) {
        return false;
    }
    using Scalar = typename VecType::Scalar;
    // Strict-interior witness: nudge endpoint toward owner solid center so
    // shared-boundary kisses do not promote as containment.
    const VecType& cen = owner.bounding_circle.c;
    const VecType& pt = inner_pts[0];
    VecType toward({cen[0] - pt[0], cen[1] - pt[1]});
    constexpr Scalar nudge = static_cast<Scalar>(1e-6);
    const Scalar tlen = static_cast<Scalar>(
        std::sqrt(static_cast<double>(
            toward[0] * toward[0] + toward[1] * toward[1])));
    VecType probe = pt;
    if (tlen > nudge) {
        probe = VecType({
            pt[0] + toward[0] * (nudge / tlen),
            pt[1] + toward[1] * (nudge / tlen)});
    }
    return is_point_inside_plane_space(probe, outer);
}

template <class VecType>
inline bool check_mutual_containment(
    const PlaneGeometry<VecType>& polyA,
    const PlaneGeometry<VecType>& polyB
) {
    if (polyA.line_parts.empty() || polyB.line_parts.empty()) return false;

    for (size_t p_idx = 0; p_idx < polyA.line_parts.size(); ++p_idx) {
        if (polyA.line_parts[p_idx].is_subtractive) continue;
        if (narrow_phase_contain(
                polyA.get_part_points(static_cast<int>(p_idx)),
                polyA.get_part_size(static_cast<int>(p_idx)),
                polyA, polyB)) {
            return true;
        }
    }

    for (size_t p_idx = 0; p_idx < polyB.line_parts.size(); ++p_idx) {
        if (polyB.line_parts[p_idx].is_subtractive) continue;
        if (narrow_phase_contain(
                polyB.get_part_points(static_cast<int>(p_idx)),
                polyB.get_part_size(static_cast<int>(p_idx)),
                polyB, polyA)) {
            return true;
        }
    }

    return false;
}

template <class VecType>
inline bool global_bounding_circles_overlap(
    const PlaneGeometry<VecType>& polyA,
    const PlaneGeometry<VecType>& polyB,
    typename VecType::Scalar margin = static_cast<typename VecType::Scalar>(0)
) {
    // Optional margin covers Welzl is_inside slack (same idea as circle_ltrb pad).
    return circles_overlap(
        polyA.get_bounding_circle(), polyB.get_bounding_circle(), margin);
}

template <class VecType>
inline bool try_add_containment_collision(
    const PlaneGeometry<VecType>& polyA,
    const PlaneGeometry<VecType>& polyB
) {
    if (!global_bounding_circles_overlap(polyA, polyB)) return false;
    return check_mutual_containment(polyA, polyB);
}

// G2: concave-hole bars can cross solid with all vertices in the void. Additive
// GJK skips hole contours; edge midpoints of boundary_rings catch material hit.
template <class VecType>
inline bool boundary_edge_mid_in_plane(
    const PlaneGeometry<VecType>& probe,
    const PlaneGeometry<VecType>& solid
) {
    using Scalar = typename VecType::Scalar;
    for (const auto& ring : probe.boundary_rings) {
        const int n = static_cast<int>(ring.points.size());
        if (n < 2) {
            continue;
        }
        for (int i = 0; i < n; ++i) {
            const auto& p1 = ring.points[static_cast<size_t>(i)];
            const auto& p2 = ring.points[static_cast<size_t>((i + 1) % n)];
            const VecType mid({
                (p1[0] + p2[0]) * static_cast<Scalar>(0.5),
                (p1[1] + p2[1]) * static_cast<Scalar>(0.5)});
            if (is_point_inside_plane_space(mid, solid)) {
                return true;
            }
        }
    }
    return false;
}

template <class VecType>
inline bool plane_has_subtractive_ring(const PlaneGeometry<VecType>& solid) {
    for (const auto& ring : solid.boundary_rings) {
        if (ring.is_subtractive) {
            return true;
        }
    }
    return false;
}

template <class VecType>
inline bool planes_boundary_edge_overlap(
    const PlaneGeometry<VecType>& a,
    const PlaneGeometry<VecType>& b
) {
    // Only for holed solids: shared-edge kiss mids lie on the boundary and
    // would false-promote packing collide on solid–solid Touch.
    // Sample the non-holed probe's edges into the holed solid (cheap); avoid
    // walking large holed board rings against every emit candidate.
    const bool a_hole = plane_has_subtractive_ring(a);
    const bool b_hole = plane_has_subtractive_ring(b);
    if (!a_hole && !b_hole) {
        return false;
    }
    if (a_hole && !b_hole) {
        return boundary_edge_mid_in_plane(b, a);
    }
    if (b_hole && !a_hole) {
        return boundary_edge_mid_in_plane(a, b);
    }
    return boundary_edge_mid_in_plane(a, b) || boundary_edge_mid_in_plane(b, a);
}

template <class VecType>
inline void promote_containment_pairs(
    const std::vector<PlaneGeometry<VecType>>& polygons,
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
inline void promote_containment_pairs(
    const std::vector<const PlaneGeometry<VecType>*>& polygons,
    const std::vector<std::pair<int, int>>& potential_containments,
    std::vector<std::pair<int, int>>& collisions
) {
    for (const auto &pair_id : potential_containments) {
        const auto *a = polygons[static_cast<std::size_t>(pair_id.first)];
        const auto *b = polygons[static_cast<std::size_t>(pair_id.second)];
        if (a == nullptr || b == nullptr) {
            continue;
        }
        if (try_add_containment_collision(*a, *b)) {
            collisions.push_back(pair_id);
        }
    }
}

template <class VecType>
inline void promote_containment_pairs_bipartite(
    const std::vector<PlaneGeometry<VecType>>& setA,
    const std::vector<PlaneGeometry<VecType>>& setB,
    const std::vector<std::pair<int, int>>& potential_containments,
    std::vector<std::pair<int, int>>& collisions
) {
    for (const auto& pair_id : potential_containments) {
        if (try_add_containment_collision(setA[pair_id.first], setB[pair_id.second])) {
            collisions.push_back(pair_id);
        }
    }
}

template <class VecType>
inline void promote_containment_pairs_bipartite(
    const std::vector<const PlaneGeometry<VecType>*>& setA,
    const std::vector<const PlaneGeometry<VecType>*>& setB,
    const std::vector<std::pair<int, int>>& potential_containments,
    std::vector<std::pair<int, int>>& collisions
) {
    for (const auto &pair_id : potential_containments) {
        const auto *a = setA[static_cast<std::size_t>(pair_id.first)];
        const auto *b = setB[static_cast<std::size_t>(pair_id.second)];
        if (a == nullptr || b == nullptr) {
            continue;
        }
        if (try_add_containment_collision(*a, *b)) {
            collisions.push_back(pair_id);
        }
    }
}

// =========================================================================
// PART 2: STANDALONE CONTAINMENT
// =========================================================================

template <class VecType>
inline bool boundaries_intersect(
    const PlaneGeometry<VecType>& poly_a,
    const PlaneGeometry<VecType>& poly_b
) {
    using Scalar = typename VecType::Scalar;
    const Scalar touch_eps_sq = nest_touch_eps_sq<Scalar>();

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
inline bool is_plane_fully_contained(
    const PlaneGeometry<VecType>& inner,
    const PlaneGeometry<VecType>& outer
) {
    if (!global_bounding_circles_overlap(inner, outer)) return false;

    bool any_inside = false;
    for (size_t i = 0; i < inner.line_parts.size(); ++i) {
        if (inner.line_parts[i].is_subtractive) continue;

        const VecType* inner_pts = inner.get_part_points(i);
        const int inner_n = inner.get_part_size(i);
        if (inner_n <= 0) continue;

        for (int v = 0; v < inner_n; ++v) {
            if (!is_point_inside_plane_space(inner_pts[v], outer)) {
                return false;
            }
            any_inside = true;
        }

        for (int e = 0; e < std::max(0, inner_n - 1); ++e) {
            const VecType mid = (inner_pts[e] + inner_pts[e + 1]) * static_cast<typename VecType::Scalar>(0.5);
            if (!is_point_inside_plane_space(mid, outer)) {
                return false;
            }
        }
    }

    if (!any_inside) return false;
    return !boundaries_intersect(inner, outer);
}

template <class VecType>
inline bool plane_footprint_inside(
    const PlaneGeometry<VecType>& inner,
    const PlaneGeometry<VecType>& outer
) {
    return is_plane_fully_contained(inner, outer);
}

template <class VecType>
inline std::vector<bool> plane_footprint_inside(
    const std::vector<PlaneGeometry<VecType>>& inners,
    const PlaneGeometry<VecType>& outer
) {
    int n_inners = static_cast<int>(inners.size());
    std::vector<bool> results(n_inners, false);
    for (int i = 0; i < n_inners; ++i) {
        results[i] = is_plane_fully_contained(inners[i], outer);
    }
    return results;
}