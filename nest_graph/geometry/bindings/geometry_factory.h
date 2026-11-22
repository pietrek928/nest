#pragma once

#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

#include "common/geometry_common.h"
#include "common/tracer.h"
#include "distance/polygon_distance.h"
#include "guide/polygon_cast.h"
#include "plane/decompose.h"
#include "plane/containment.h"
#include "plane/plane_geometry.h"
#include "sweep/sweep_engine.h"

template <class VecType>
inline VecType plane_centroid(const PlaneGeometry<VecType> &solid) {
    using Scalar = typename VecType::Scalar;
    Scalar sx = static_cast<Scalar>(0);
    Scalar sy = static_cast<Scalar>(0);
    std::size_t count = 0;
    for (std::size_t i = 0; i < solid.line_parts.size(); ++i) {
        if (solid.line_parts[i].is_subtractive
            || solid.line_parts[i].role != ChainRole::FilledConvex) {
            continue;
        }
        const VecType *pts = solid.get_part_points(i);
        const int n = solid.get_part_size(i);
        for (int j = 0; j < n; ++j) {
            sx += pts[j][0];
            sy += pts[j][1];
            ++count;
        }
    }
    if (count == 0) {
        const auto &c = solid.get_bounding_circle().center();
        return VecType({c[0], c[1]});
    }
    const Scalar inv = static_cast<Scalar>(1) / static_cast<Scalar>(count);
    return VecType({sx * inv, sy * inv});
}

template <class VecType>
inline PlaneGeometry<VecType> plane_from_rings_coords(
    const std::vector<std::vector<VecType>> &rings,
    std::mt19937 &rng
) {
    PlaneGeometry<VecType> mesh;
    std::size_t added = 0;
    for (const auto &ring_pts : rings) {
        if (ring_pts.size() < 2) {
            continue;
        }
        mesh.add_boundary_ring(ring_pts, false);
        std::vector<VecType> closed = ring_pts;
        const VecType &first = closed.front();
        const VecType &last = closed.back();
        if (first[0] != last[0] || first[1] != last[1]) {
            closed.push_back(first);
        }
        process_boundary_to_convex_segments<VecType>(closed, mesh, rng, false);
        ++added;
    }
    if (added == 0) {
        throw std::invalid_argument(
            "from_rings: need at least one ring with 2+ points");
    }
    mesh.finalize(rng);
    return mesh;
}

template <class VecType>
inline PlaneGeometry<VecType> plane_from_ring_coords(
    const std::vector<VecType> &ring_pts,
    std::mt19937 &rng
) {
    if (ring_pts.size() < 2) {
        throw std::invalid_argument("from_ring: need at least 2 distinct points");
    }
    return plane_from_rings_coords<VecType>({{ring_pts}}, rng);
}

template <class VecType>
inline std::pair<VecType, VecType> closest_points_between_parts(
    const PlaneGeometry<VecType> &poly_a,
    int part_a,
    const PlaneGeometry<VecType> &poly_b,
    int part_b
) {
    using Scalar = typename VecType::Scalar;
    const VecType *pts_a = poly_a.get_part_points(part_a);
    const int n_a = poly_a.get_part_size(part_a);
    const VecType *pts_b = poly_b.get_part_points(part_b);
    const int n_b = poly_b.get_part_size(part_b);

    Scalar best_sq = std::numeric_limits<Scalar>::max();
    VecType best_a = pts_a[0];
    VecType best_b = pts_b[0];

    auto consider = [&](const VecType &pa, const VecType &pb) {
        const VecType d = pa - pb;
        const Scalar sq = d.dp(d);
        if (sq < best_sq) {
            best_sq = sq;
            best_a = pa;
            best_b = pb;
        }
    };

    if (n_a == 1 && n_b == 1) {
        consider(pts_a[0], pts_b[0]);
        return {best_a, best_b};
    }

    const auto role_a = poly_a.line_parts[static_cast<std::size_t>(part_a)].role;
    const auto role_b = poly_b.line_parts[static_cast<std::size_t>(part_b)].role;
    const bool closed_a = role_a == ChainRole::Stroke
        && poly_a.line_parts[static_cast<std::size_t>(part_a)].closed;
    const bool closed_b = role_b == ChainRole::Stroke
        && poly_b.line_parts[static_cast<std::size_t>(part_b)].closed;
    const int edges_a = (n_a <= 1) ? 0 : (closed_a ? n_a : std::max(1, n_a - 1));
    const int edges_b = (n_b <= 1) ? 0 : (closed_b ? n_b : std::max(1, n_b - 1));

    auto edge_end = [](const VecType *pts, int n, int i, bool closed) -> VecType {
        if (closed) {
            return pts[(i + 1) % n];
        }
        return pts[std::min(i + 1, n - 1)];
    };

    if (edges_a == 0 && n_a == 1) {
        for (int j = 0; j < edges_b; ++j) {
            const VecType b0 = pts_b[j];
            const VecType b1 = edge_end(pts_b, n_b, j, closed_b);
            VecType pa;
            VecType pb;
            Scalar dsq = static_cast<Scalar>(0);
            closest_points_between_segments(pts_a[0], pts_a[0], b0, b1, pa, pb, dsq);
            if (dsq < best_sq) {
                best_sq = dsq;
                best_a = pa;
                best_b = pb;
            }
        }
        return {best_a, best_b};
    }
    if (edges_b == 0 && n_b == 1) {
        for (int i = 0; i < edges_a; ++i) {
            const VecType a0 = pts_a[i];
            const VecType a1 = edge_end(pts_a, n_a, i, closed_a);
            VecType pa;
            VecType pb;
            Scalar dsq = static_cast<Scalar>(0);
            closest_points_between_segments(a0, a1, pts_b[0], pts_b[0], pa, pb, dsq);
            if (dsq < best_sq) {
                best_sq = dsq;
                best_a = pa;
                best_b = pb;
            }
        }
        return {best_a, best_b};
    }

    for (int i = 0; i < edges_a; ++i) {
        const VecType a0 = pts_a[i];
        const VecType a1 = edge_end(pts_a, n_a, i, closed_a);
        for (int j = 0; j < edges_b; ++j) {
            const VecType b0 = pts_b[j];
            const VecType b1 = edge_end(pts_b, n_b, j, closed_b);
            VecType pa;
            VecType pb;
            Scalar dsq = static_cast<Scalar>(0);
            closest_points_between_segments(a0, a1, b0, b1, pa, pb, dsq);
            if (dsq < best_sq) {
                best_sq = dsq;
                best_a = pa;
                best_b = pb;
            }
        }
    }

    return {best_a, best_b};
}

template <class VecType>
struct PairDistanceResult {
    ComplexDistanceResult<VecType> core;
    VecType closest_a;
    VecType closest_b;
};

// GEOS-compatible polygon-outline to open-ring distance (segment-only, no fill).
template <class VecType>
inline PairDistanceResult<VecType> standoff_distance_pair(
    const PlaneGeometry<VecType> &part,
    const PlaneGeometry<VecType> &ring
) {
    using Scalar = typename VecType::Scalar;
    PairDistanceResult<VecType> out{};
    out.core.polyA_idx = 0;
    out.core.polyB_idx = 1;
    out.core.intersect = false;
    out.core.distance_sq = std::numeric_limits<Scalar>::max();
    out.core.penetration_sq = static_cast<Scalar>(0);
    out.core.mtv = VecType();
    out.closest_a = part.get_bounding_circle().center();
    out.closest_b = ring.get_bounding_circle().center();

    for (std::size_t i = 0; i < part.line_parts.size(); ++i) {
        if (part.line_parts[i].is_subtractive) {
            continue;
        }
        const auto& circ_a = part.line_parts[i].bounding_circle;
        for (std::size_t j = 0; j < ring.line_parts.size(); ++j) {
            if (ring.line_parts[j].is_subtractive) {
                continue;
            }
            const auto& circ_b = ring.line_parts[j].bounding_circle;
            const Scalar center_dist_sq = circle_center_distance_sq(circ_a, circ_b);
            const Scalar r_sum = circle_radius(circ_a) + circle_radius(circ_b);
            const Scalar center_dist = static_cast<Scalar>(
                std::sqrt(static_cast<double>(center_dist_sq)));
            const Scalar min_possible = std::max(
                static_cast<Scalar>(0), center_dist - r_sum);
            if (min_possible * min_possible >= out.core.distance_sq) {
                continue;
            }
            auto pts = closest_points_between_parts(
                part, static_cast<int>(i), ring, static_cast<int>(j));
            const VecType d = pts.first - pts.second;
            const Scalar sq = d.dp(d);
            if (sq < out.core.distance_sq) {
                out.core.distance_sq = sq;
                out.closest_a = pts.first;
                out.closest_b = pts.second;
            }
        }
    }

    const Scalar touch_eps_sq = nest_touch_eps_sq<Scalar>();
    if (out.core.distance_sq <= touch_eps_sq) {
        out.core.intersect = true;
        out.core.distance_sq = static_cast<Scalar>(0);
    }
    return out;
}

template <class VecType>
inline PairDistanceResult<VecType> min_distance_pair(
    const PlaneGeometry<VecType> &a,
    const PlaneGeometry<VecType> &b,
    typename VecType::Scalar aura = static_cast<typename VecType::Scalar>(0.5)
) {
    using Scalar = typename VecType::Scalar;
    (void)aura;
    PairDistanceResult<VecType> out{};
    out.core.polyA_idx = 0;
    out.core.polyB_idx = 1;
    out.core.intersect = false;
    out.core.distance_sq = std::numeric_limits<Scalar>::max();
    out.core.penetration_sq = static_cast<Scalar>(0);
    out.core.mtv = VecType();
    out.closest_a = a.get_bounding_circle().center();
    out.closest_b = b.get_bounding_circle().center();

    for (size_t i = 0; i < a.line_parts.size(); ++i) {
        if (a.line_parts[i].is_subtractive) {
            continue;
        }
        const auto &ca = a.line_parts[i].bounding_circle;
        for (size_t j = 0; j < b.line_parts.size(); ++j) {
            if (b.line_parts[j].is_subtractive) {
                continue;
            }
            const auto &cb = b.line_parts[j].bounding_circle;
            const VecType d = ca.center() - cb.center();
            const Scalar center_dist_sq = d.dp(d);
            const Scalar r_sum =
                std::sqrt(static_cast<double>(ca.square_radius())) +
                std::sqrt(static_cast<double>(cb.square_radius()));

            DistanceCandidate<VecType> cand{};
            cand.pair_id = {0, 1};
            cand.partA_idx = static_cast<int>(i);
            cand.partB_idx = static_cast<int>(j);
            cand.polyA = &a;
            cand.polyB = &b;
            cand.center_dist_sq = center_dist_sq;
            cand.r_sum = static_cast<Scalar>(r_sum);

            auto eval = evaluate_distance_candidate<VecType>(cand);
            if (eval.intersect) {
                out.core = eval;
                out.core.polyA_idx = 0;
                out.core.polyB_idx = 1;
                auto pts = closest_points_between_parts(
                    a, eval.partA_idx, b, eval.partB_idx);
                out.closest_a = pts.first;
                out.closest_b = pts.second;
                return out;
            }
            if (eval.distance_sq < out.core.distance_sq) {
                out.core = eval;
                out.core.polyA_idx = 0;
                out.core.polyB_idx = 1;
                auto pts = closest_points_between_parts(
                    a, eval.partA_idx, b, eval.partB_idx);
                out.closest_a = pts.first;
                out.closest_b = pts.second;
            }
        }
    }

    if (!out.core.intersect && check_mutual_containment(a, b)) {
        out.core.intersect = true;
        out.core.distance_sq = static_cast<Scalar>(0);
        out.core.penetration_sq = std::numeric_limits<Scalar>::max();
    }
    return out;
}

template <class VecType>
inline ComplexCastResult<VecType> cast_slide(
    const PlaneGeometry<VecType> &active,
    const std::vector<PlaneGeometry<VecType>> &obstacles,
    const VecType &slide,
    typename VecType::Scalar max_t
) {
    // No active+obstacles repack: cast vs obstacle list by const ref.
    return find_closest_polygon_cast_vs_obstacles<VecType>(
        active, obstacles, slide, max_t);
}

template <class VecType>
inline std::vector<ComplexCastResult<VecType>> cast_slide_all(
    const PlaneGeometry<VecType> &active,
    const std::vector<PlaneGeometry<VecType>> &obstacles,
    const VecType &slide,
    typename VecType::Scalar max_t
) {
    return find_all_polygon_casts_vs_obstacles<VecType>(
        active, obstacles, slide, max_t);
}

template <class VecType>
inline ComplexCastResult<VecType> cast_slide(
    const PlaneGeometry<VecType> &active,
    const std::vector<const PlaneGeometry<VecType> *> &obstacles,
    const VecType &slide,
    typename VecType::Scalar max_t
) {
    return find_closest_polygon_cast_vs_obstacles<VecType>(
        active, obstacles, slide, max_t);
}

template <class VecType>
inline std::vector<ComplexCastResult<VecType>> cast_slide_all(
    const PlaneGeometry<VecType> &active,
    const std::vector<const PlaneGeometry<VecType> *> &obstacles,
    const VecType &slide,
    typename VecType::Scalar max_t
) {
    return find_all_polygon_casts_vs_obstacles<VecType>(
        active, obstacles, slide, max_t);
}

// Packing collide for two solids without owned 2-element sweep vectors.
template <class VecType, class Tracer = DefaultTracer>
inline bool planes_packing_collide(
    const PlaneGeometry<VecType> &a,
    const PlaneGeometry<VecType> &b,
    Tracer *tracer = nullptr
) {
    using Scalar = typename VecType::Scalar;
    // A5a: same padded whole-solid circle gate as intersect-sweep pre-push.
    if (!global_bounding_circles_overlap(
            a, b, static_cast<Scalar>(1e-6))) {
        return false;
    }
    for (size_t i = 0; i < a.line_parts.size(); ++i) {
        if (a.line_parts[i].is_subtractive
            || a.line_parts[i].role != ChainRole::FilledConvex) {
            continue;
        }
        for (size_t j = 0; j < b.line_parts.size(); ++j) {
            if (b.line_parts[j].is_subtractive
                || b.line_parts[j].role != ChainRole::FilledConvex) {
                continue;
            }
            if (check_part_vs_part_intersect<VecType, Tracer>(
                    a, static_cast<int>(i), b, static_cast<int>(j), tracer)) {
                return true;
            }
        }
    }
    if (try_add_containment_collision(a, b)) {
        return true;
    }
    return planes_boundary_edge_overlap(a, b);
}
