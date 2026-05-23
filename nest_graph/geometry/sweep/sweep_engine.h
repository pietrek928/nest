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
    typename VecType::Scalar aura_multiplier = static_cast<typename VecType::Scalar>(0)
) {
    using Scalar = typename VecType::Scalar;
    for (size_t part = 0; part < poly.line_parts.size(); ++part) {
        if (poly.line_parts[part].is_subtractive) continue;

        const auto& bounds = poly.line_parts[part].bounding_circle;
        const Scalar proj = bounds.center().dp(sweep_axis);
        const Scalar r = circle_radius(bounds) * axis_len_sqrt;
        const Scalar margin = r * aura_multiplier;
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
// Intersect narrow-phase delegation
// -------------------------------------------------------------------------
template <class VecType, class Tracer = DefaultTracer>
inline bool check_part_vs_part_intersect(
    const SolidGeometry<VecType>& polyA, int a_idx,
    const SolidGeometry<VecType>& polyB, int b_idx,
    Tracer* tracer = nullptr
) {
    const auto& circleA = polyA.line_parts[a_idx].bounding_circle;
    const auto& circleB = polyB.line_parts[b_idx].bounding_circle;

    // O(1) Pruning: If the exact bounding circles don't touch,
    // the line strings cannot physically intersect.
    if (!circles_overlap(circleA, circleB)) {
        if constexpr (!std::is_same_v<Tracer, NullTracer>) {
            if (tracer) tracer->count_circle_prune();
        }
        return false;
    }

    const VecType* ptsA = polyA.get_part_points(a_idx);
    const int nA = polyA.get_part_size(a_idx);
    const VecType* ptsB = polyB.get_part_points(b_idx);
    const int nB = polyB.get_part_size(b_idx);

    if constexpr (!std::is_same_v<Tracer, NullTracer>) {
        if (tracer) tracer->count_gjk_eval();
    }

    // Delegate to the narrow-phase router (which internally sorts by size to optimize
    // gradient caching, and dispatches to the correct O(N) or O(1) GJK implementation)
    // NOTE: Requires narrow_phase_intersect to be in scope or explicitly defined.
    // If not in scope, you can inline the (nA <= nB) swap logic here.
    bool hit = false;

    const VecType* p1 = (nA <= nB) ? ptsA : ptsB;
    const int s1 = (nA <= nB) ? nA : nB;
    const VecType* p2 = (nA <= nB) ? ptsB : ptsA;
    const int s2 = (nA <= nB) ? nB : nA;

    if (s1 + s2 > 24) {
        hit = convex_linestrings_intersect_gjk_gradient<VecType>(p1, s1, p2, s2).intersect;
    } else {
        hit = convex_linestrings_intersect_gjk<VecType>(p1, s1, p2, s2).intersect;
    }

    if (hit) {
        if constexpr (!std::is_same_v<Tracer, NullTracer>) {
            if (tracer) tracer->record_penetration();
        }
    }

    return hit;
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

        // If this polygon pair already collided on a previous part, skip the rest!
        if (current_pair_hit) continue;

        TracerScope<Tracer> scope(tracer, cand.pair_id.first, cand.pair_id.second);

        if (check_part_vs_part_intersect<VecType, Tracer>(*cand.polyA, cand.partA_idx, *cand.polyB, cand.partB_idx, tracer)) {
            result.confirmed_collisions.push_back(cand.pair_id);
            current_pair_hit = true;
        } else {
            result.potential_containments.push_back(cand.pair_id);
        }
    }

    // Deduplicate containments and remove any that were confirmed as collisions
    std::sort(result.potential_containments.begin(), result.potential_containments.end());
    result.potential_containments.erase(std::unique(result.potential_containments.begin(), result.potential_containments.end()), result.potential_containments.end());

    auto new_end = std::remove_if(result.potential_containments.begin(), result.potential_containments.end(),
        [&result](const std::pair<int, int>& p) {
            return std::binary_search(result.confirmed_collisions.begin(), result.confirmed_collisions.end(), p);
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

template <class VecType, class Tracer = DefaultTracer>
inline std::vector<ComplexDistanceResult<VecType>> execute_distance_sweep(
    std::vector<PartSweepElement<VecType>>& elements,
    typename VecType::Scalar aura_multiplier,
    SweepMode mode = SweepMode::Monopartite,
    int bipartite_set_a_size = -1,
    Tracer* tracer = nullptr
) {
    using Scalar = typename VecType::Scalar;
    if (elements.empty()) return {};

    std::sort(elements.begin(), elements.end(), [](const auto& a, const auto& b) {
        return a.min_proj < b.min_proj;
    });

    // 1. COLLECT: Gather bounds with aura threshold
    std::vector<DistanceCandidate<VecType>> candidates;
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

            const auto& circleA = polyA->line_parts[partA].bounding_circle;
            const auto& circleB = polyB->line_parts[partB].bounding_circle;

            const Scalar rA = circle_radius(circleA);
            const Scalar rB = circle_radius(circleB);
            const Scalar r_sum = rA + rB;
            const Scalar center_dist_sq = circle_center_distance_sq(circleA, circleB);
            const Scalar dynamic_threshold = r_sum + (rA * aura_multiplier) + (rB * aura_multiplier);

            if (center_dist_sq > dynamic_threshold * dynamic_threshold) {
                if constexpr (!std::is_same_v<Tracer, NullTracer>) if (tracer) tracer->count_circle_prune();
                continue;
            }

            candidates.push_back({pair_id, partA, partB, polyA, polyB, center_dist_sq, r_sum});
        }
    }

    // 2. SORT: Group pairs and put closest parts at the top of the group
    std::sort(candidates.begin(), candidates.end());

    // 3. EVALUATE: Sequentially evaluate pairs with dynamic pruning
    std::vector<ComplexDistanceResult<VecType>> final_results;
    if (candidates.empty()) return final_results;

    std::pair<int, int> current_pair = candidates[0].pair_id;
    ComplexDistanceResult<VecType> best_eval;
    bool has_eval = false;

    // Helper lambda to finalize and push the previous polygon pair
    auto commit_best_eval = [&]() {
        if (!has_eval) return;
        // If no intersection occurred, we MUST check if one is fully swallowed by the other
        if (!best_eval.intersect) {
            const auto* pA = candidates.back().polyA; // Any candidate in the group holds the pointers
            const auto* pB = candidates.back().polyB;

            if (pA->line_parts.size() > 0 && pB->line_parts.size() > 0) {
                const VecType* ptsA = pA->get_part_points(0);
                const VecType* ptsB = pB->get_part_points(0);

                // Uses the O(1) containment optimization!
                if (narrow_phase_contain(ptsA, pA->get_part_size(0), *pB) ||
                    narrow_phase_contain(ptsB, pB->get_part_size(0), *pA)) {
                    best_eval.intersect = true;
                    best_eval.distance_sq = 0;
                    best_eval.penetration_sq = std::numeric_limits<Scalar>::max();
                    best_eval.mtv = VecType{};
                }
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
        }

        // DYNAMIC PRUNING: Because candidates are sorted by distance, best_eval.distance_sq
        // drops rapidly, causing this check to skip thousands of unnecessary GJK evaluations!
        if (has_eval && !best_eval.intersect) {
            const Scalar center_dist = static_cast<Scalar>(std::sqrt(static_cast<double>(cand.center_dist_sq)));
            const Scalar min_possible_dist = std::max(static_cast<Scalar>(0), center_dist - cand.r_sum);
            if (min_possible_dist * min_possible_dist >= best_eval.distance_sq) {
                continue;
            }
        }

        TracerScope<Tracer> scope(tracer, cand.pair_id.first, cand.pair_id.second);

        const VecType* ptsA = cand.polyA->get_part_points(cand.partA_idx);
        int nA = cand.polyA->get_part_size(cand.partA_idx);
        const VecType* ptsB = cand.polyB->get_part_points(cand.partB_idx);
        int nB = cand.polyB->get_part_size(cand.partB_idx);

        if constexpr (!std::is_same_v<Tracer, NullTracer>) {
            if (tracer) tracer->count_gjk_eval();
        }

        // Uses the wrapper to handle the swapping internally!
        auto pen_res = narrow_phase_penetration(ptsA, nA, ptsB, nB, 24, tracer);

        ComplexDistanceResult<VecType> current_eval{
            cand.pair_id.first, cand.pair_id.second, cand.partA_idx, cand.partB_idx,
            pen_res.intersect,
            pen_res.intersect ? 0 : std::numeric_limits<Scalar>::max(),
            pen_res.intersect ? pen_res.penetration_sq : 0,
            pen_res.intersect ? pen_res.mtv : VecType{}
        };

        if (!pen_res.intersect) {
            auto dist_res = narrow_phase_distance(ptsA, nA, ptsB, nB, false, 24, tracer);

            const Scalar touch_eps_sq = static_cast<Scalar>(1e-10);
            if (dist_res.intersect || dist_res.distance_sq <= touch_eps_sq) {
                current_eval.intersect = true;
                current_eval.distance_sq = 0;
            } else {
                current_eval.distance_sq = dist_res.distance_sq;
            }
        }

        if (!has_eval) {
            best_eval = current_eval;
            has_eval = true;
        } else {
            if (current_eval.intersect && !best_eval.intersect) {
                best_eval = current_eval;
            } else if (current_eval.intersect && best_eval.intersect) {
                if (current_eval.penetration_sq > best_eval.penetration_sq) best_eval = current_eval;
            } else if (!current_eval.intersect && !best_eval.intersect) {
                if (current_eval.distance_sq < best_eval.distance_sq) best_eval = current_eval;
            }
        }
    }
    commit_best_eval(); // Ensure final pair is pushed

    return final_results;
}
