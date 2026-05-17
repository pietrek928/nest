#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <type_traits>
#include <utility>
#include <vector>

#include "solid_geometry.h"
#include "geometry_common.h"
#include "containment.h"
#include "convex/distance.h"
#include "convex/intersect.h"
#include "convex/penetration.h"
#include "sweep.h"
#include "tracer.h"

// -------------------------------------------------------------------------
// Sweep element population (additive parts only; holes are not colliders)
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
        if (poly.line_parts[part].is_subtractive) {
            continue;
        }
        const auto& bounds = poly.line_parts[part].bounding_circle;
        const Scalar proj = bounds.center().dp(sweep_axis);
        const Scalar r = circle_radius(bounds) * axis_len_sqrt;
        const Scalar margin = r * aura_multiplier;
        out_elements.push_back({
            poly_idx,
            static_cast<int>(part),
            group_id,
            (proj - r) - margin,
            (proj + r) + margin,
            &poly,
            &bounds,
        });
    }
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
        if (group_i == group_j) {
            return false;
        }
        reverse_for_distance = (group_i == 1);
        pair_id = reverse_for_distance
            ? std::make_pair(pB_idx, pA_idx)
            : std::make_pair(pA_idx, pB_idx);
        if (bipartite_set_a_size > 0) {
            pair_id.second -= bipartite_set_a_size;
        }
        return true;
    }

    if (mode == SweepMode::Subset) {
        if (group_i == 0 && group_j == 0) {
            return false;
        }
        reverse_for_distance = (pA_idx > pB_idx);
        pair_id = reverse_for_distance
            ? std::make_pair(pB_idx, pA_idx)
            : std::make_pair(pA_idx, pB_idx);
        return true;
    }

    reverse_for_distance = (pA_idx > pB_idx);
    pair_id = make_sorted_pair(pA_idx, pB_idx);
    return true;
}

// -------------------------------------------------------------------------
// Intersect sweep
// -------------------------------------------------------------------------
template <class VecType>
struct IntersectSweepResult {
    std::vector<std::pair<int, int>> confirmed_collisions;
    std::vector<std::pair<int, int>> potential_containments;
};

template <class VecType, class Tracer = DefaultTracer>
inline bool check_part_vs_part_intersect(
    const SolidGeometry<VecType>& polyA,
    int a_idx,
    const SolidGeometry<VecType>& polyB,
    int b_idx,
    Tracer* tracer = nullptr
) {
    const auto& circleA = polyA.line_parts[a_idx].bounding_circle;
    const auto& circleB = polyB.line_parts[b_idx].bounding_circle;

    if (!circles_overlap(circleA, circleB)) {
        if constexpr (!std::is_same_v<Tracer, NullTracer>) {
            if (tracer) {
                tracer->count_circle_prune();
            }
        }
        return false;
    }

    const VecType* ptsA = polyA.get_part_points(a_idx);
    const int nA = polyA.get_part_size(a_idx);
    const VecType* ptsB = polyB.get_part_points(b_idx);
    const int nB = polyB.get_part_size(b_idx);

    if constexpr (!std::is_same_v<Tracer, NullTracer>) {
        if (tracer) {
            tracer->count_gjk_eval();
        }
    }

    const VecType* p1 = (nA <= nB) ? ptsA : ptsB;
    const int s1 = (nA <= nB) ? nA : nB;
    const VecType* p2 = (nA <= nB) ? ptsB : ptsA;
    const int s2 = (nA <= nB) ? nB : nA;

    const bool hit = (s1 + s2 > 24)
        ? convex_linestrings_intersect_gjk_gradient<VecType>(p1, s1, p2, s2).intersect
        : convex_linestrings_intersect_gjk<VecType>(p1, s1, p2, s2).intersect;

    if (hit) {
        if constexpr (!std::is_same_v<Tracer, NullTracer>) {
            if (tracer) {
                tracer->record_penetration();
            }
        }
    }
    return hit;
}

template <class VecType, class Tracer = DefaultTracer>
inline IntersectSweepResult<VecType> execute_intersect_sweep(
    std::vector<PartSweepElement<VecType>>& elements,
    SweepMode mode = SweepMode::Monopartite,
    int bipartite_set_a_size = -1,
    Tracer* tracer = nullptr
) {
    IntersectSweepResult<VecType> result;
    if (elements.empty()) {
        return result;
    }

    std::sort(elements.begin(), elements.end(), [](const auto& a, const auto& b) {
        return a.min_proj < b.min_proj;
    });

    std::vector<std::pair<int, int>> known_collisions;

    for (size_t i = 0; i < elements.size(); ++i) {
        for (size_t j = i + 1; j < elements.size(); ++j) {
            if (elements[j].min_proj > elements[i].max_proj) {
                break;
            }
            if (elements[i].poly_idx == elements[j].poly_idx) {
                continue;
            }

            std::pair<int, int> pair_id;
            bool reverse = false;
            if (!resolve_sweep_pair_id(
                    elements[i], elements[j], mode, bipartite_set_a_size, pair_id, reverse)) {
                continue;
            }

            if constexpr (!std::is_same_v<Tracer, NullTracer>) {
                if (tracer) {
                    tracer->count_sweep_pair();
                }
            }

            auto it = std::lower_bound(known_collisions.begin(), known_collisions.end(), pair_id);
            if (it != known_collisions.end() && *it == pair_id) {
                continue;
            }

            TracerScope<Tracer> scope(tracer, pair_id.first, pair_id.second);

            const auto& polyA = *(elements[i].poly_ptr);
            const auto& polyB = *(elements[j].poly_ptr);
            const int partA = elements[i].part_idx;
            const int partB = elements[j].part_idx;

            if (check_part_vs_part_intersect<VecType, Tracer>(polyA, partA, polyB, partB, tracer)) {
                known_collisions.insert(it, pair_id);
                result.confirmed_collisions.push_back(pair_id);
            } else {
                result.potential_containments.push_back(pair_id);
            }
        }
    }

    std::sort(result.potential_containments.begin(), result.potential_containments.end());
    result.potential_containments.erase(
        std::unique(result.potential_containments.begin(), result.potential_containments.end()),
        result.potential_containments.end());

    auto new_end = std::remove_if(
        result.potential_containments.begin(),
        result.potential_containments.end(),
        [&known_collisions](const std::pair<int, int>& pair) {
            return std::binary_search(known_collisions.begin(), known_collisions.end(), pair);
        });
    result.potential_containments.erase(new_end, result.potential_containments.end());

    return result;
}

// -------------------------------------------------------------------------
// Distance sweep
// -------------------------------------------------------------------------
template <class VecType>
struct ComplexDistanceResult {
    using Scalar = typename VecType::Scalar;

    int polyA_idx;
    int polyB_idx;
    int partA_idx;
    int partB_idx;

    bool intersect;
    Scalar distance_sq;
    Scalar penetration_sq;
    VecType mtv;
};

template <class VecType>
struct PolyPairTracker {
    std::pair<int, int> pair_id;
    ComplexDistanceResult<VecType> result;

    bool operator<(const PolyPairTracker& other) const { return pair_id < other.pair_id; }
    bool operator<(const std::pair<int, int>& id) const { return pair_id < id; }
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
    std::vector<PolyPairTracker<VecType>> active_pairs;

    if (elements.empty()) {
        return {};
    }

    std::sort(elements.begin(), elements.end(), [](const auto& a, const auto& b) {
        return a.min_proj < b.min_proj;
    });

    for (size_t i = 0; i < elements.size(); ++i) {
        for (size_t j = i + 1; j < elements.size(); ++j) {
            if (elements[j].min_proj > elements[i].max_proj) {
                break;
            }
            if (elements[i].poly_idx == elements[j].poly_idx) {
                continue;
            }

            std::pair<int, int> pair_id;
            bool reverse = false;
            if (!resolve_sweep_pair_id(
                    elements[i], elements[j], mode, bipartite_set_a_size, pair_id, reverse)) {
                continue;
            }

            if constexpr (!std::is_same_v<Tracer, NullTracer>) {
                if (tracer) {
                    tracer->count_sweep_pair();
                }
            }

            const auto* poly1_ptr = reverse ? elements[j].poly_ptr : elements[i].poly_ptr;
            const auto* poly2_ptr = reverse ? elements[i].poly_ptr : elements[j].poly_ptr;
            const int part1_idx = reverse ? elements[j].part_idx : elements[i].part_idx;
            const int part2_idx = reverse ? elements[i].part_idx : elements[j].part_idx;

            const auto& circle1 = poly1_ptr->line_parts[part1_idx].bounding_circle;
            const auto& circle2 = poly2_ptr->line_parts[part2_idx].bounding_circle;

            const Scalar r1 = circle_radius(circle1);
            const Scalar r2 = circle_radius(circle2);
            const Scalar r_sum = r1 + r2;
            const Scalar center_dist_sq = circle_center_distance_sq(circle1, circle2);
            const Scalar dynamic_threshold = r_sum + (r1 * aura_multiplier) + (r2 * aura_multiplier);

            if (center_dist_sq > dynamic_threshold * dynamic_threshold) {
                if constexpr (!std::is_same_v<Tracer, NullTracer>) {
                    if (tracer) {
                        tracer->count_circle_prune();
                    }
                }
                continue;
            }

            auto it = std::lower_bound(active_pairs.begin(), active_pairs.end(), pair_id);
            const bool is_new_pair = (it == active_pairs.end() || it->pair_id != pair_id);

            if (!is_new_pair && !it->result.intersect) {
                const Scalar center_dist = static_cast<Scalar>(std::sqrt(static_cast<double>(center_dist_sq)));
                const Scalar min_possible_dist = std::max(static_cast<Scalar>(0), center_dist - r_sum);
                if (min_possible_dist * min_possible_dist >= it->result.distance_sq) {
                    continue;
                }
            }

            const VecType* pts1 = poly1_ptr->get_part_points(part1_idx);
            const int n1 = poly1_ptr->get_part_size(part1_idx);
            const VecType* pts2 = poly2_ptr->get_part_points(part2_idx);
            const int n2 = poly2_ptr->get_part_size(part2_idx);

            {
                TracerScope<Tracer> scope(tracer, pair_id.first, pair_id.second);

                if constexpr (!std::is_same_v<Tracer, NullTracer>) {
                    if (tracer) {
                        tracer->count_gjk_eval();
                    }
                }

                bool swapped = (n1 > n2);
                const VecType* p1 = swapped ? pts2 : pts1;
                const int s1 = swapped ? n2 : n1;
                const VecType* p2 = swapped ? pts1 : pts2;
                const int s2 = swapped ? n1 : n2;

                auto pen_res = (s1 + s2 > 24)
                    ? convex_linestrings_penetration_gradient<VecType>(p1, s1, p2, s2)
                    : convex_linestrings_penetration<VecType>(p1, s1, p2, s2);
                if (swapped && pen_res.intersect) {
                    pen_res.mtv = -pen_res.mtv;
                }

                if (pen_res.intersect) {
                    if constexpr (!std::is_same_v<Tracer, NullTracer>) {
                        if (tracer) {
                            tracer->record_penetration();
                        }
                    }
                }

                ComplexDistanceResult<VecType> current_eval{
                    pair_id.first,
                    pair_id.second,
                    part1_idx,
                    part2_idx,
                    pen_res.intersect,
                    pen_res.intersect ? 0 : std::numeric_limits<Scalar>::max(),
                    pen_res.intersect ? pen_res.penetration_sq : 0,
                    pen_res.intersect ? pen_res.mtv : VecType{},
                };

                if (!pen_res.intersect) {
                    if constexpr (!std::is_same_v<Tracer, NullTracer>) {
                        if (tracer) {
                            tracer->record_distance();
                        }
                    }

                    auto dist_res = (s1 + s2 > 24)
                        ? convex_linestrings_distance_gjk_gradient<VecType>(p1, s1, p2, s2, false)
                        : convex_linestrings_distance_gjk<VecType>(p1, s1, p2, s2, false);

                    if (dist_res.distance_sq > dynamic_threshold * dynamic_threshold) {
                        continue;
                    }

                    const Scalar touch_eps = static_cast<Scalar>(1e-5);
                    const Scalar touch_eps_sq = touch_eps * touch_eps;

                    if (dist_res.intersect || dist_res.distance_sq <= touch_eps_sq) {
                        current_eval.intersect = true;
                        current_eval.distance_sq = 0;
                        current_eval.penetration_sq = 0;
                        current_eval.mtv = VecType{};
                    } else {
                        current_eval.distance_sq = dist_res.distance_sq;
                    }
                }

                if (is_new_pair) {
                    active_pairs.insert(it, {pair_id, current_eval});
                } else {
                    auto& best = it->result;
                    if (current_eval.intersect && !best.intersect) {
                        best = current_eval;
                    } else if (current_eval.intersect && best.intersect) {
                        if (current_eval.penetration_sq > best.penetration_sq) {
                            best = current_eval;
                        }
                    } else if (!current_eval.intersect && !best.intersect) {
                        if (current_eval.distance_sq < best.distance_sq) {
                            best = current_eval;
                        }
                    }
                }
            }
        }
    }

    std::vector<ComplexDistanceResult<VecType>> final_results;
    final_results.reserve(active_pairs.size());

    std::vector<const SolidGeometry<VecType>*> poly_ptrs;
    for (const auto& el : elements) {
        if (el.poly_idx >= static_cast<int>(poly_ptrs.size())) {
            poly_ptrs.resize(el.poly_idx + 1, nullptr);
        }
        poly_ptrs[el.poly_idx] = el.poly_ptr;
    }

    for (auto& tracker : active_pairs) {
        if (!tracker.result.intersect) {
            const auto* polyA_ptr = poly_ptrs[tracker.result.polyA_idx];
            const auto* polyB_ptr = poly_ptrs[tracker.result.polyB_idx];
            if (polyA_ptr && polyB_ptr && !polyA_ptr->line_parts.empty() && !polyB_ptr->line_parts.empty()) {
                if (try_add_containment_collision(*polyA_ptr, *polyB_ptr)) {
                    tracker.result.intersect = true;
                    tracker.result.distance_sq = 0;
                    tracker.result.penetration_sq = std::numeric_limits<Scalar>::max();
                    tracker.result.mtv = VecType{};
                }
            }
        }
        final_results.push_back(tracker.result);
    }
    return final_results;
}
