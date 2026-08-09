#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <type_traits>
#include <utility>
#include <vector>

#include "solid/solid_geometry.h"
#include "common/geometry_common.h"
#include "solid/containment.h"
#include "convex/intersect.h"
#include "convex/penetration.h"
#include "convex/distance.h"
#include "sweep/sweep.h"
#include "common/tracer.h"

// -------------------------------------------------------------------------
// Sweep element population
// -------------------------------------------------------------------------
template <class VecType>
inline void append_poly_parts_to_sweep(
    int poly_idx,
    int group_id,
    const SolidGeometry<VecType>& poly,
    const VecType& sweep_axis,
    typename VecType::Scalar axis_len_sqrt,
    std::vector<PartSweepElement<VecType>>& out_elements,
    typename VecType::Scalar aura_multiplier = static_cast<typename VecType::Scalar>(0.5),
    typename VecType::Scalar distance_margin = static_cast<typename VecType::Scalar>(0)
) {
    using Scalar = typename VecType::Scalar;
    for (size_t part = 0; part < poly.line_parts.size(); ++part) {
        if (poly.line_parts[part].is_subtractive) continue;

        const auto& bounds = poly.line_parts[part].bounding_circle;
        const Scalar proj = bounds.center().dp(sweep_axis);
        const Scalar r = circle_radius(bounds) * axis_len_sqrt;
        const Scalar margin = (r * aura_multiplier) + distance_margin;
        out_elements.push_back({
            poly_idx, static_cast<int>(part), group_id,
            (proj - r) - margin, (proj + r) + margin,
            &poly, &bounds,
        });
    }
}

// -------------------------------------------------------------------------
// Broad-phase sweep axis (shared by intersect + distance)
// -------------------------------------------------------------------------
template <class VecType>
struct SweepContext {
    VecType axis;
    typename VecType::Scalar axis_len_sqrt;
};

template <class VecType, class... Args>
inline SweepContext<VecType> prepare_sweep_axis(Args&&... args) {
    using Scalar = typename VecType::Scalar;
    VecType axis = compute_optimal_sweep_axis(std::forward<Args>(args)...);
    Scalar sq = axis.len_sq();

    if (sq < static_cast<Scalar>(1e-8)) {
        return {axis, static_cast<Scalar>(1.0)};
    }
    return {axis, static_cast<Scalar>(std::sqrt(static_cast<double>(sq)))};
}

template <class VecType>
inline size_t calculate_exact_sweep_capacity(
    const std::vector<SolidGeometry<VecType>>& polys
) {
    size_t capacity = 0;
    for (const auto& p : polys) {
        capacity += p.line_parts.size();
    }
    return capacity > 0 ? capacity : polys.size() * 4;
}

// -------------------------------------------------------------------------
// Shared pair indexing for sweep modes
// -------------------------------------------------------------------------
template <class VecType>
inline bool resolve_sweep_pair_id(
    const PartSweepElement<VecType>& el_i,
    const PartSweepElement<VecType>& el_j,
    SweepMode mode,
    int bipartite_set_a_size,
    std::pair<int, int>& pair_id,
    bool& reverse_for_distance
) {
    reverse_for_distance = false;
    const int group_i = el_i.group_id;
    const int group_j = el_j.group_id;
    const int pA_idx = el_i.poly_idx;
    const int pB_idx = el_j.poly_idx;

    if (mode == SweepMode::Bipartite) {
        if (group_i == group_j) return false;
        reverse_for_distance = (group_i == 1);
        pair_id = reverse_for_distance ? std::make_pair(pB_idx, pA_idx) : std::make_pair(pA_idx, pB_idx);
        if (bipartite_set_a_size > 0) pair_id.second -= bipartite_set_a_size;
        return true;
    }
    if (mode == SweepMode::Subset) {
        if (group_i == 0 && group_j == 0) return false;
        reverse_for_distance = (pA_idx > pB_idx);
        pair_id = reverse_for_distance ? std::make_pair(pB_idx, pA_idx) : std::make_pair(pA_idx, pB_idx);
        return true;
    }

    reverse_for_distance = (pA_idx > pB_idx);
    pair_id = make_sorted_pair(pA_idx, pB_idx);
    return true;
}

// -------------------------------------------------------------------------
// Unified narrow-phase contact (Patch 1–2)
// -------------------------------------------------------------------------
inline constexpr int nest_gjk_gradient_threshold = 24;

template <class VecType, class Tracer = DefaultTracer>
inline ContactResult<typename VecType::Scalar> evaluate_narrow_phase(
    const VecType* ptsA, int nA,
    const VecType* ptsB, int nB,
    Tracer* tracer = nullptr,
    VecType* out_mtv = nullptr
) {
    using Scalar = typename VecType::Scalar;
    ContactResult<Scalar> out;

    if (nA < 2 || nB < 2) {
        return out;
    }

    const bool swapped = (nA > nB);
    const VecType* p1 = swapped ? ptsB : ptsA;
    const int s1 = swapped ? nB : nA;
    const VecType* p2 = swapped ? ptsA : ptsB;
    const int s2 = swapped ? nA : nB;

    if constexpr (!std::is_same_v<Tracer, NullTracer>) {
        if (tracer) tracer->count_gjk_eval();
    }

    IntersectResult ir;
    if (s1 + s2 > nest_gjk_gradient_threshold) {
        ir = convex_linestrings_intersect_gjk_gradient<VecType>(p1, s1, p2, s2);
    } else {
        ir = convex_linestrings_intersect_gjk<VecType>(p1, s1, p2, s2);
    }

    if (!ir.intersect) {
        out.state = ContactState::Disjoint;
        out.warm_index_A = swapped ? ir.it2 : ir.it1;
        out.warm_index_B = swapped ? ir.it1 : ir.it2;
        return out;
    }

    auto pen = (s1 + s2 > nest_gjk_gradient_threshold)
        ? convex_linestrings_penetration_gradient<VecType>(
            p1, s1, p2, s2, ir.it1, ir.it2)
        : convex_linestrings_penetration<VecType>(
            p1, s1, p2, s2, ir.it1, ir.it2);

    out.warm_index_A = swapped ? pen.it2 : pen.it1;
    out.warm_index_B = swapped ? pen.it1 : pen.it2;

    const Scalar pen_eps_sq = nest_packing_penetration_eps_sq<Scalar>();
    // EPA {intersect=true, depth~0} or failed polytope → Touch, never Penetrating.
    if (!pen.intersect || pen.penetration_sq < pen_eps_sq) {
        out.state = ContactState::Touch;
        out.depth = static_cast<Scalar>(0);
        out.penetration_sq = static_cast<Scalar>(0);
        return out;
    }

    if constexpr (!std::is_same_v<Tracer, NullTracer>) {
        if (tracer) tracer->record_penetration();
    }
    out.state = ContactState::Penetrating;
    out.penetration_sq = pen.penetration_sq;
    out.depth = static_cast<Scalar>(
        std::sqrt(static_cast<double>(pen.penetration_sq)));
    if (out_mtv != nullptr) {
        // pen.mtv is in (p1,p2) order; if swapped, negate to A→B frame.
        *out_mtv = swapped ? (pen.mtv * static_cast<Scalar>(-1)) : pen.mtv;
    }
    return out;
}

template <class VecType>
inline bool contact_edge_mid_interior_witness(
    const VecType* pts, int n,
    const SolidGeometry<VecType>& owner,
    const SolidGeometry<VecType>& other
) {
    using Scalar = typename VecType::Scalar;
    if (n < 2) {
        return false;
    }
    // Nudge toward solid centroid (not part mean — decomp edge centroids lie on
    // shared walls and false-positive kisses).
    const VecType& cen = owner.bounding_circle.c;
    constexpr Scalar nudge = static_cast<Scalar>(1e-6);
    for (int i = 0; i < n - 1; ++i) {
        const VecType mid({
            (pts[i][0] + pts[i + 1][0]) * static_cast<Scalar>(0.5),
            (pts[i][1] + pts[i + 1][1]) * static_cast<Scalar>(0.5)});
        VecType toward({cen[0] - mid[0], cen[1] - mid[1]});
        const Scalar tlen = static_cast<Scalar>(
            std::sqrt(static_cast<double>(
                toward[0] * toward[0] + toward[1] * toward[1])));
        if (tlen <= nudge) {
            continue;
        }
        const VecType probe({
            mid[0] + toward[0] * (nudge / tlen),
            mid[1] + toward[1] * (nudge / tlen)});
        if (is_point_inside_solid_space(probe, other)) {
            return true;
        }
    }
    return false;
}

template <class VecType, class Tracer = DefaultTracer>
inline ContactResult<typename VecType::Scalar> evaluate_narrow_phase_parts(
    const SolidGeometry<VecType>& polyA, int a_idx,
    const SolidGeometry<VecType>& polyB, int b_idx,
    Tracer* tracer = nullptr
) {
    using Scalar = typename VecType::Scalar;
    const auto& circleA = polyA.line_parts[a_idx].bounding_circle;
    const auto& circleB = polyB.line_parts[b_idx].bounding_circle;
    if (!circles_overlap(circleA, circleB)) {
        if constexpr (!std::is_same_v<Tracer, NullTracer>) {
            if (tracer) tracer->count_circle_prune();
        }
        return ContactResult<Scalar>{};
    }
    auto cr = evaluate_narrow_phase<VecType, Tracer>(
        polyA.get_part_points(a_idx), polyA.get_part_size(a_idx),
        polyB.get_part_points(b_idx), polyB.get_part_size(b_idx),
        tracer);
    // EPA often returns {depth=0, intersect=true} for both kisses and solid
    // overlaps on edge-decomp parts. Promote Touch→Penetrating only when a
    // strict-interior edge-mid witness lands inside the other solid.
    if (cr.state == ContactState::Touch) {
        const VecType* ptsA = polyA.get_part_points(a_idx);
        const int nA = polyA.get_part_size(a_idx);
        const VecType* ptsB = polyB.get_part_points(b_idx);
        const int nB = polyB.get_part_size(b_idx);
        if (contact_edge_mid_interior_witness(ptsA, nA, polyA, polyB)
            || contact_edge_mid_interior_witness(ptsB, nB, polyB, polyA)) {
            if constexpr (!std::is_same_v<Tracer, NullTracer>) {
                if (tracer) tracer->record_penetration();
            }
            cr.state = ContactState::Penetrating;
            cr.depth = static_cast<Scalar>(0);
            cr.penetration_sq = static_cast<Scalar>(0);
        }
    }
    return cr;
}

// Legacy kind names → ContactState (packing collision = Penetrating only).
enum class PartIntersectKind {
    None,   // Disjoint
    Kiss,   // Touch
    Overlap // Penetrating
};

template <class VecType, class Tracer = DefaultTracer>
inline PartIntersectKind check_part_vs_part_intersect_kind(
    const SolidGeometry<VecType>& polyA, int a_idx,
    const SolidGeometry<VecType>& polyB, int b_idx,
    Tracer* tracer = nullptr
) {
    const auto cr = evaluate_narrow_phase_parts<VecType, Tracer>(
        polyA, a_idx, polyB, b_idx, tracer);
    if (cr.state == ContactState::Penetrating) {
        return PartIntersectKind::Overlap;
    }
    if (cr.state == ContactState::Touch) {
        return PartIntersectKind::Kiss;
    }
    return PartIntersectKind::None;
}

template <class VecType, class Tracer = DefaultTracer>
inline bool check_part_vs_part_intersect(
    const SolidGeometry<VecType>& polyA, int a_idx,
    const SolidGeometry<VecType>& polyB, int b_idx,
    Tracer* tracer = nullptr
) {
    return evaluate_narrow_phase_parts<VecType, Tracer>(
        polyA, a_idx, polyB, b_idx, tracer).state == ContactState::Penetrating;
}

// -------------------------------------------------------------------------
// Intersect sweep (Optimized Pipeline)
// -------------------------------------------------------------------------
template <class VecType>
struct IntersectCandidate {
    std::pair<int, int> pair_id;
    int partA_idx, partB_idx;
    const SolidGeometry<VecType>* polyA;
    const SolidGeometry<VecType>* polyB;

    bool operator<(const IntersectCandidate& o) const {
        return pair_id < o.pair_id;
    }
};

template <class VecType>
struct IntersectSweepResult {
    std::vector<std::pair<int, int>> confirmed_collisions;
    std::vector<std::pair<int, int>> potential_containments;
};

template <class VecType, class Tracer = DefaultTracer>
inline IntersectSweepResult<VecType> execute_intersect_sweep(
    std::vector<PartSweepElement<VecType>>& elements,
    SweepMode mode = SweepMode::Monopartite,
    int bipartite_set_a_size = -1,
    Tracer* tracer = nullptr
) {
    IntersectSweepResult<VecType> result;
    if (elements.empty()) return result;

    std::sort(elements.begin(), elements.end(), [](const auto& a, const auto& b) {
        return a.min_proj < b.min_proj;
    });

    // 1. COLLECT: Gather all broad-phase overlapping boundaries
    std::vector<IntersectCandidate<VecType>> candidates;
    for (size_t i = 0; i < elements.size(); ++i) {
        for (size_t j = i + 1; j < elements.size(); ++j) {
            if (elements[j].min_proj > elements[i].max_proj) break;
            if (elements[i].poly_idx == elements[j].poly_idx) continue;

            std::pair<int, int> pair_id;
            bool reverse = false;
            if (!resolve_sweep_pair_id(elements[i], elements[j], mode, bipartite_set_a_size, pair_id, reverse)) continue;

            if constexpr (!std::is_same_v<Tracer, NullTracer>) if (tracer) tracer->count_sweep_pair();

            const auto* polyA = reverse ? elements[j].poly_ptr : elements[i].poly_ptr;
            const auto* polyB = reverse ? elements[i].poly_ptr : elements[j].poly_ptr;
            int partA = reverse ? elements[j].part_idx : elements[i].part_idx;
            int partB = reverse ? elements[i].part_idx : elements[j].part_idx;

            candidates.push_back({pair_id, partA, partB, polyA, polyB});
        }
    }

    // 2. SORT: Group identically paired polygons together
    std::sort(candidates.begin(), candidates.end());

    // 3. EVALUATE: Process sequentially with early-exit logic
    std::pair<int, int> current_pair = {-1, -1};
    bool current_pair_hit = false;

    for (const auto& cand : candidates) {
        if (cand.pair_id != current_pair) {
            current_pair = cand.pair_id;
            current_pair_hit = false;
        }

        if (current_pair_hit) continue;

        TracerScope<Tracer> scope(tracer, cand.pair_id.first, cand.pair_id.second);

        const auto cr = evaluate_narrow_phase_parts<VecType, Tracer>(
            *cand.polyA, cand.partA_idx, *cand.polyB, cand.partB_idx, tracer);
        if (cr.state == ContactState::Penetrating) {
            result.confirmed_collisions.push_back(cand.pair_id);
            current_pair_hit = true;
        } else if (cr.state == ContactState::Disjoint) {
            // Only boundary-disjoint parts: may still be nested → containment.
            // Touch must not promote (strict-interior contain also guards this).
            result.potential_containments.push_back(cand.pair_id);
        }
    }

    std::sort(result.potential_containments.begin(), result.potential_containments.end());
    result.potential_containments.erase(
        std::unique(result.potential_containments.begin(), result.potential_containments.end()),
        result.potential_containments.end());

    auto new_end = std::remove_if(
        result.potential_containments.begin(),
        result.potential_containments.end(),
        [&result](const std::pair<int, int>& p) {
            return std::binary_search(
                result.confirmed_collisions.begin(),
                result.confirmed_collisions.end(),
                p);
        });
    result.potential_containments.erase(new_end, result.potential_containments.end());

    return result;
}

// -------------------------------------------------------------------------
// Distance sweep (Optimized Pipeline)
// -------------------------------------------------------------------------
template <class VecType>
struct ComplexDistanceResult {
    using Scalar = typename VecType::Scalar;
    int polyA_idx, polyB_idx;
    int partA_idx, partB_idx;
    bool intersect;
    Scalar distance_sq;
    Scalar penetration_sq;
    VecType mtv;
    VecType closest_normal{};
};

template <class VecType>
struct DistanceCandidate {
    using Scalar = typename VecType::Scalar;
    std::pair<int, int> pair_id;
    int partA_idx, partB_idx;
    const SolidGeometry<VecType>* polyA;
    const SolidGeometry<VecType>* polyB;
    Scalar center_dist_sq;
    Scalar r_sum;

    bool operator<(const DistanceCandidate& o) const {
        if (pair_id != o.pair_id) return pair_id < o.pair_id;
        // Evaluate physically closer parts first to maximize dynamic pruning
        return center_dist_sq < o.center_dist_sq;
    }
};

template <class VecType>
inline void collect_distance_candidates(
    const std::vector<PartSweepElement<VecType>>& elements,
    typename VecType::Scalar aura_multiplier,
    typename VecType::Scalar distance_margin,
    SweepMode mode,
    int bipartite_set_a_size,
    std::vector<DistanceCandidate<VecType>>& candidates
) {
    using Scalar = typename VecType::Scalar;
    candidates.clear();
    candidates.reserve(elements.size() * 2);
    for (size_t i = 0; i < elements.size(); ++i) {
        for (size_t j = i + 1; j < elements.size(); ++j) {
            if (elements[j].min_proj > elements[i].max_proj) break;
            if (elements[i].poly_idx == elements[j].poly_idx) continue;

            std::pair<int, int> pair_id;
            bool reverse = false;
            if (!resolve_sweep_pair_id(
                    elements[i], elements[j], mode, bipartite_set_a_size, pair_id, reverse)) {
                continue;
            }

            const auto* polyA = reverse ? elements[j].poly_ptr : elements[i].poly_ptr;
            const auto* polyB = reverse ? elements[i].poly_ptr : elements[j].poly_ptr;
            int partA = reverse ? elements[j].part_idx : elements[i].part_idx;
            int partB = reverse ? elements[i].part_idx : elements[j].part_idx;

            const auto& circleA = polyA->line_parts[partA].bounding_circle;
            const auto& circleB = polyB->line_parts[partB].bounding_circle;

            const Scalar rA = circle_radius(circleA);
            const Scalar rB = circle_radius(circleB);
            const Scalar r_sum = rA + rB;
            const Scalar center_dist_sq = circle_center_distance_sq(circleA, circleB);
            const Scalar dynamic_threshold =
                r_sum + (rA * aura_multiplier) + (rB * aura_multiplier) + distance_margin;

            if (center_dist_sq > dynamic_threshold * dynamic_threshold) {
                continue;
            }

            candidates.push_back({pair_id, partA, partB, polyA, polyB, center_dist_sq, r_sum});
        }
    }
    std::sort(candidates.begin(), candidates.end());
}

template <class VecType, class Tracer = DefaultTracer>
inline ComplexDistanceResult<VecType> evaluate_distance_candidate(
    const DistanceCandidate<VecType>& cand,
    Tracer* tracer = nullptr
) {
    using Scalar = typename VecType::Scalar;
    const VecType* ptsA = cand.polyA->get_part_points(cand.partA_idx);
    int nA = cand.polyA->get_part_size(cand.partA_idx);
    const VecType* ptsB = cand.polyB->get_part_points(cand.partB_idx);
    int nB = cand.polyB->get_part_size(cand.partB_idx);

    if constexpr (!std::is_same_v<Tracer, NullTracer>) {
        if (tracer) tracer->count_gjk_eval();
    }

    ComplexDistanceResult<VecType> current_eval{
        cand.pair_id.first, cand.pair_id.second, cand.partA_idx, cand.partB_idx,
        false,
        std::numeric_limits<Scalar>::max(),
        static_cast<Scalar>(0),
        VecType{},
        VecType{}
    };

    const bool circles_may_overlap = cand.center_dist_sq <= cand.r_sum * cand.r_sum;
    int warm1 = 0;
    int warm2 = 0;
    if (circles_may_overlap) {
        VecType mtv{};
        auto cr = evaluate_narrow_phase<VecType, Tracer>(
            ptsA, nA, ptsB, nB, tracer, &mtv);
        warm1 = cr.warm_index_A;
        warm2 = cr.warm_index_B;
        if (cr.state == ContactState::Penetrating) {
            current_eval.intersect = true;
            current_eval.distance_sq = 0;
            current_eval.penetration_sq = cr.penetration_sq;
            current_eval.mtv = mtv;
            return current_eval;
        }
    }

    auto dist_res = narrow_phase_distance(
        ptsA, nA, ptsB, nB, warm1, warm2,
        nest_gjk_gradient_threshold, tracer);

    const Scalar touch_eps_sq = nest_touch_eps_sq<Scalar>();
    if (dist_res.intersect || dist_res.distance_sq <= touch_eps_sq) {
        // Near-zero separation / GJK touch: contact without packing depth.
        current_eval.intersect = false;
        current_eval.distance_sq = 0;
    } else {
        current_eval.distance_sq = dist_res.distance_sq;
        current_eval.closest_normal = ptsA[dist_res.it1] - ptsB[dist_res.it2];
    }
    return current_eval;
}

template <class VecType, class Tracer = DefaultTracer>
inline std::vector<ComplexDistanceResult<VecType>> execute_distance_sweep(
    std::vector<PartSweepElement<VecType>>& elements,
    typename VecType::Scalar aura_multiplier,
    typename VecType::Scalar distance_margin = static_cast<typename VecType::Scalar>(0),
    SweepMode mode = SweepMode::Monopartite,
    int bipartite_set_a_size = -1,
    Tracer* tracer = nullptr
) {
    using Scalar = typename VecType::Scalar;
    if (elements.empty()) return {};

    std::sort(elements.begin(), elements.end(), [](const auto& a, const auto& b) {
        return a.min_proj < b.min_proj;
    });

    std::vector<DistanceCandidate<VecType>> candidates;
    collect_distance_candidates(
        elements, aura_multiplier, distance_margin, mode, bipartite_set_a_size, candidates);

    std::vector<ComplexDistanceResult<VecType>> final_results;
    if (candidates.empty()) return final_results;

    std::pair<int, int> current_pair = candidates[0].pair_id;
    ComplexDistanceResult<VecType> best_eval;
    const SolidGeometry<VecType>* current_polyA = nullptr;
    const SolidGeometry<VecType>* current_polyB = nullptr;
    bool has_eval = false;

    auto commit_best_eval = [&]() {
        if (!has_eval) return;
        if (!best_eval.intersect && current_polyA != nullptr && current_polyB != nullptr) {
            if (check_mutual_containment(*current_polyA, *current_polyB)) {
                best_eval.intersect = true;
                best_eval.distance_sq = 0;
                best_eval.penetration_sq = std::numeric_limits<Scalar>::max();
                best_eval.mtv = VecType{};
            }
        }
        final_results.push_back(best_eval);
    };

    for (size_t i = 0; i < candidates.size(); ++i) {
        const auto& cand = candidates[i];

        if (cand.pair_id != current_pair) {
            commit_best_eval();
            current_pair = cand.pair_id;
            has_eval = false;
            current_polyA = nullptr;
            current_polyB = nullptr;
        }

        if (has_eval && !best_eval.intersect) {
            const Scalar center_dist = static_cast<Scalar>(
                std::sqrt(static_cast<double>(cand.center_dist_sq)));
            const Scalar min_possible_dist = std::max(
                static_cast<Scalar>(0), center_dist - cand.r_sum);
            if (min_possible_dist * min_possible_dist >= best_eval.distance_sq) {
                continue;
            }
        }

        TracerScope<Tracer> scope(tracer, cand.pair_id.first, cand.pair_id.second);
        if constexpr (!std::is_same_v<Tracer, NullTracer>) {
            if (tracer) tracer->count_sweep_pair();
        }

        auto current_eval = evaluate_distance_candidate<VecType, Tracer>(cand, tracer);

        if (!has_eval) {
            best_eval = current_eval;
            current_polyA = cand.polyA;
            current_polyB = cand.polyB;
            has_eval = true;
        } else if (current_eval.intersect && !best_eval.intersect) {
            best_eval = current_eval;
        } else if (current_eval.intersect && best_eval.intersect) {
            if (current_eval.penetration_sq > best_eval.penetration_sq) best_eval = current_eval;
        } else if (!current_eval.intersect && !best_eval.intersect) {
            if (current_eval.distance_sq < best_eval.distance_sq) best_eval = current_eval;
        }
    }
    commit_best_eval();

    return final_results;
}

template <class VecType, class Tracer = DefaultTracer>
inline bool execute_distance_sweep_any_violation(
    std::vector<PartSweepElement<VecType>>& elements,
    typename VecType::Scalar aura_multiplier,
    typename VecType::Scalar distance_margin,
    typename VecType::Scalar margin_sq,
    SweepMode mode = SweepMode::Monopartite,
    int bipartite_set_a_size = -1,
    Tracer* tracer = nullptr
) {
    using Scalar = typename VecType::Scalar;
    if (elements.empty()) return false;

    std::sort(elements.begin(), elements.end(), [](const auto& a, const auto& b) {
        return a.min_proj < b.min_proj;
    });

    std::vector<DistanceCandidate<VecType>> candidates;
    collect_distance_candidates(
        elements, aura_multiplier, distance_margin, mode, bipartite_set_a_size, candidates);
    if (candidates.empty()) return false;

    std::pair<int, int> current_pair = candidates[0].pair_id;
    ComplexDistanceResult<VecType> best_eval;
    const SolidGeometry<VecType>* current_polyA = nullptr;
    const SolidGeometry<VecType>* current_polyB = nullptr;
    bool has_eval = false;

    auto pair_violates = [&]() -> bool {
        if (!has_eval) return false;
        if (!best_eval.intersect && current_polyA != nullptr && current_polyB != nullptr) {
            if (check_mutual_containment(*current_polyA, *current_polyB)) {
                return true;
            }
        }
        if (best_eval.intersect) return true;
        return margin_sq > static_cast<Scalar>(0)
            && best_eval.distance_sq < margin_sq;
    };

    for (size_t i = 0; i < candidates.size(); ++i) {
        const auto& cand = candidates[i];

        if (cand.pair_id != current_pair) {
            if (pair_violates()) return true;
            current_pair = cand.pair_id;
            has_eval = false;
            current_polyA = nullptr;
            current_polyB = nullptr;
        }

        if (has_eval && best_eval.intersect) {
            continue;
        }
        if (has_eval && !best_eval.intersect
            && margin_sq > static_cast<Scalar>(0)
            && best_eval.distance_sq < margin_sq) {
            return true;
        }
        if (has_eval && !best_eval.intersect) {
            const Scalar center_dist = static_cast<Scalar>(
                std::sqrt(static_cast<double>(cand.center_dist_sq)));
            const Scalar min_possible_dist = std::max(
                static_cast<Scalar>(0), center_dist - cand.r_sum);
            if (margin_sq > static_cast<Scalar>(0)
                && min_possible_dist * min_possible_dist >= margin_sq
                && min_possible_dist * min_possible_dist >= best_eval.distance_sq) {
                continue;
            }
            if (min_possible_dist * min_possible_dist >= best_eval.distance_sq) {
                continue;
            }
        }

        auto current_eval = evaluate_distance_candidate<VecType, Tracer>(cand, tracer);
        if (current_eval.intersect) {
            return true;
        }
        if (margin_sq > static_cast<Scalar>(0)
            && current_eval.distance_sq < margin_sq) {
            return true;
        }

        if (!has_eval) {
            best_eval = current_eval;
            current_polyA = cand.polyA;
            current_polyB = cand.polyB;
            has_eval = true;
        } else if (current_eval.distance_sq < best_eval.distance_sq) {
            best_eval = current_eval;
        }
    }
    return pair_violates();
}
