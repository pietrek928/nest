#pragma once

#include <cstddef>
#include <vector>
#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <type_traits>

#include "poly.h"
#include "convex/distance.h"    // Updated to Line String GJK
#include "convex/penetration.h" // Updated to Line String EPA
#include "sweep.h"
#include "tracer.h"

// -------------------------------------------------------------------------
// DEFAULT TRACER ALIAS
// -------------------------------------------------------------------------
#ifndef NEST_GRAPH_DEFAULT_TRACER
#define NEST_GRAPH_DEFAULT_TRACER NullTracer
#endif
using DefaultTracer = NEST_GRAPH_DEFAULT_TRACER;

// -------------------------------------------------------------------------
// RESULT & TRACKER STRUCTURES
// -------------------------------------------------------------------------
template<class VecType>
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

template<class VecType>
struct PolyPairTracker {
    std::pair<int, int> pair_id;
    ComplexDistanceResult<VecType> result;

    bool operator<(const PolyPairTracker& other) const { return pair_id < other.pair_id; }
    bool operator<(const std::pair<int, int>& id) const { return pair_id < id; }
};

// -------------------------------------------------------------------------
// SOLID-SIDE CONTAINMENT CHECK
// -------------------------------------------------------------------------
template<class VecType>
inline bool is_point_inside_solid_space(const VecType& pt, const Polygon<VecType>& poly) {
    using Scalar = typename VecType::Scalar;
    int crossings = 0;

    for (size_t part = 0; part < poly.line_parts.size(); ++part) {
        const VecType* pts = poly.get_part_points(part);
        int n = poly.get_part_size(part);

        for (int i = 0; i < n; ++i) {
            const VecType& v1 = pts[i];
            const VecType& v2 = pts[(i + 1) % n];

            if (((v1[1] > pt[1]) != (v2[1] > pt[1])) &&
                (pt[0] < (v2[0] - v1[0]) * (pt[1] - v1[1]) / (v2[1] - v1[1]) + v1[0])) {
                crossings++;
            }
        }
    }
    return (crossings % 2) != 0;
}

// -------------------------------------------------------------------------
// NARROW PHASE ROUTERS (Smart Dispatch & MTV Correction)
// -------------------------------------------------------------------------
template<class VecType, class Tracer = DefaultTracer>
inline DistanceResult<VecType> narrow_phase_distance(
    const VecType* lsA, int nA,
    const VecType* lsB, int nB,
    bool known_overlap,
    int GRADIENT_THRESHOLD = 24,
    Tracer* tracer = nullptr
) {
    if constexpr (!std::is_same_v<Tracer, NullTracer>) {
        if (tracer) tracer->record_distance();
    }

    const VecType* p1 = (nA <= nB) ? lsA : lsB;
    int s1 = (nA <= nB) ? nA : nB;

    const VecType* p2 = (nA <= nB) ? lsB : lsA;
    int s2 = (nA <= nB) ? nB : nA;

    if (s1 + s2 > GRADIENT_THRESHOLD) {
        return convex_linestrings_distance_gjk_gradient<VecType>(p1, s1, p2, s2, known_overlap);
    }
    return convex_linestrings_distance_gjk<VecType>(p1, s1, p2, s2, known_overlap);
}

template<class VecType, class Tracer = DefaultTracer>
inline PenetrationResult<VecType> narrow_phase_penetration(
    const VecType* lsA, int nA,
    const VecType* lsB, int nB,
    int GRADIENT_THRESHOLD = 24,
    Tracer* tracer = nullptr
) {
    if constexpr (!std::is_same_v<Tracer, NullTracer>) {
        if (tracer) tracer->record_penetration();
    }

    bool swapped = (nA > nB);

    const VecType* p1 = swapped ? lsB : lsA;
    int s1 = swapped ? nB : nA;

    const VecType* p2 = swapped ? lsA : lsB;
    int s2 = swapped ? nA : nB;

    auto res = (s1 + s2 > GRADIENT_THRESHOLD)
        ? convex_linestrings_penetration_gradient<VecType>(p1, s1, p2, s2)
        : convex_linestrings_penetration<VecType>(p1, s1, p2, s2);

    if (swapped && res.intersect) {
        res.mtv = -res.mtv;
    }

    return res;
}

// -------------------------------------------------------------------------
// DISTANCE SWEEP ENGINE
// -------------------------------------------------------------------------
template<class VecType>
inline void append_poly_parts_to_sweep_with_aura(
    int poly_idx,
    int group_id,
    const Polygon<VecType>& poly,
    const VecType& sweep_axis,
    typename VecType::Scalar axis_len_sqrt,
    std::vector<PartSweepElement<VecType>>& out_elements,
    typename VecType::Scalar aura_multiplier
) {
    using Scalar = typename VecType::Scalar;
    // Updated to use line_parts
    for (size_t part = 0; part < poly.line_parts.size(); ++part) {
        const auto& bounds = poly.line_parts[part].bounding_circle;
        Scalar proj = bounds.center().dp(sweep_axis);

        Scalar r = static_cast<Scalar>(std::sqrt(static_cast<double>(bounds.square_radius()))) * axis_len_sqrt;
        Scalar margin = r * aura_multiplier;

        out_elements.push_back({poly_idx, static_cast<int>(part), group_id, (proj - r) - margin, (proj + r) + margin, &poly, &bounds});
    }
}

template<class VecType, class Tracer = DefaultTracer>
inline std::vector<ComplexDistanceResult<VecType>> execute_distance_sweep(
    std::vector<PartSweepElement<VecType>>& elements,
    typename VecType::Scalar aura_multiplier,
    SweepMode mode = SweepMode::Monopartite,
    int bipartite_set_a_size = -1,
    Tracer* tracer = nullptr
) {
    using Scalar = typename VecType::Scalar;
    std::vector<PolyPairTracker<VecType>> active_pairs;

    if (elements.empty()) return {};

    std::sort(elements.begin(), elements.end(), [](const auto& a, const auto& b) {
        return a.min_proj < b.min_proj;
    });

    for (size_t i = 0; i < elements.size(); ++i) {
        for (size_t j = i + 1; j < elements.size(); ++j) {

            if (elements[j].min_proj > elements[i].max_proj) break;

            int group_i = elements[i].group_id;
            int group_j = elements[j].group_id;
            int pA_idx = elements[i].poly_idx;
            int pB_idx = elements[j].poly_idx;

            if (pA_idx == pB_idx) continue;

            std::pair<int, int> pair_id;
            bool reverse = false;

            if (mode == SweepMode::Bipartite) {
                if (group_i == group_j) continue;
                reverse = (group_i == 1);
                pair_id = reverse ? std::make_pair(pB_idx, pA_idx) : std::make_pair(pA_idx, pB_idx);
                if (bipartite_set_a_size > 0) {
                    pair_id.second -= bipartite_set_a_size;
                }
            }
            else if (mode == SweepMode::Subset) {
                if (group_i == 0 && group_j == 0) continue;
                reverse = (pA_idx > pB_idx);
                pair_id = reverse ? std::make_pair(pB_idx, pA_idx) : std::make_pair(pA_idx, pB_idx);
            }
            else {
                reverse = (pA_idx > pB_idx);
                pair_id = reverse ? std::make_pair(pB_idx, pA_idx) : std::make_pair(pA_idx, pB_idx);
            }

            if constexpr (!std::is_same_v<Tracer, NullTracer>) {
                if (tracer) tracer->count_sweep_pair();
            }

            // Updated to line_parts
            const auto* poly1_ptr = reverse ? elements[j].poly_ptr : elements[i].poly_ptr;
            const auto* poly2_ptr = reverse ? elements[i].poly_ptr : elements[j].poly_ptr;
            int part1_idx = reverse ? elements[j].part_idx : elements[i].part_idx;
            int part2_idx = reverse ? elements[i].part_idx : elements[j].part_idx;

            const auto& circle1 = poly1_ptr->line_parts[part1_idx].bounding_circle;
            const auto& circle2 = poly2_ptr->line_parts[part2_idx].bounding_circle;

            Scalar center_dist_sq = (circle1.center() - circle2.center()).len_sq();
            Scalar r1 = static_cast<Scalar>(std::sqrt(static_cast<double>(circle1.square_radius())));
            Scalar r2 = static_cast<Scalar>(std::sqrt(static_cast<double>(circle2.square_radius())));
            Scalar r_sum = r1 + r2;

            Scalar dynamic_threshold = r_sum + (r1 * aura_multiplier) + (r2 * aura_multiplier);
            if (center_dist_sq > dynamic_threshold * dynamic_threshold) {
                if constexpr (!std::is_same_v<Tracer, NullTracer>) {
                    if (tracer) tracer->count_circle_prune();
                }
                continue;
            }

            auto it = std::lower_bound(active_pairs.begin(), active_pairs.end(), pair_id);
            bool is_new_pair = (it == active_pairs.end() || it->pair_id != pair_id);

            if (!is_new_pair && !it->result.intersect) {
                Scalar center_dist = static_cast<Scalar>(std::sqrt(static_cast<double>(center_dist_sq)));
                Scalar min_possible_dist = std::max(static_cast<Scalar>(0), center_dist - r_sum);
                if (min_possible_dist * min_possible_dist >= it->result.distance_sq) {
                    continue;
                }
            }

            const VecType* pts1 = poly1_ptr->get_part_points(part1_idx);
            int n1 = poly1_ptr->get_part_size(part1_idx);
            const VecType* pts2 = poly2_ptr->get_part_points(part2_idx);
            int n2 = poly2_ptr->get_part_size(part2_idx);

            {
                TracerScope<Tracer> scope(tracer, pair_id.first, pair_id.second);

                if constexpr (!std::is_same_v<Tracer, NullTracer>) {
                    if (tracer) tracer->count_gjk_eval();
                }

                // 1. Boundary Penetration Check
                auto pen_res = narrow_phase_penetration<VecType, Tracer>(pts1, n1, pts2, n2, 24, tracer);
                bool isValidCollision = pen_res.intersect;

                ComplexDistanceResult<VecType> current_eval = {
                    pair_id.first, pair_id.second, part1_idx, part2_idx,
                    isValidCollision,
                    isValidCollision ? 0 : std::numeric_limits<Scalar>::max(),
                    isValidCollision ? pen_res.penetration_sq : 0,
                    isValidCollision ? pen_res.mtv : VecType{}
                };

                // 2. No boundary crossing? Check if one is swallowed by the other!
                if (!isValidCollision) {
                    // Only do containment checks if their strict master bounding boxes overlap
                    if (center_dist_sq <= r_sum * r_sum) {
                        bool contained = is_point_inside_solid_space(pts1[0], *poly2_ptr) ||
                                         is_point_inside_solid_space(pts2[0], *poly1_ptr);

                        if (contained) {
                            current_eval.intersect = true;
                            current_eval.distance_sq = 0;
                            // Max penalty to force the nesting solver to eject it
                            current_eval.penetration_sq = std::numeric_limits<Scalar>::max();
                            current_eval.mtv = VecType{}; // MTV is zero since edges don't touch
                            isValidCollision = true;
                        }
                    }

                    // 3. Still no collision? Safely evaluate distance between the boundaries
                    if (!isValidCollision) {
                        auto dist_res = narrow_phase_distance<VecType, Tracer>(
                            pts1, n1, pts2, n2, false, 24, tracer);

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
                }

                // 4. Update the Active Pairs Tracker
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
    for (const auto& tracker : active_pairs) {
        final_results.push_back(tracker.result);
    }
    return final_results;
}

// -------------------------------------------------------------------------
// MAIN ENGINE ENTRY POINTS
// -------------------------------------------------------------------------

template<class VecType, class Tracer = DefaultTracer>
std::vector<ComplexDistanceResult<VecType>> find_polygon_distances(
    const std::vector<Polygon<VecType>>& polygons,
    typename VecType::Scalar aura_multiplier = static_cast<typename VecType::Scalar>(0.5),
    Tracer* tracer = nullptr
) {
    using Scalar = typename VecType::Scalar;
    if (polygons.size() < 2) return {};

    VecType sweep_axis = compute_optimal_sweep_axis(polygons);

    std::vector<PartSweepElement<VecType>> elements;
    elements.reserve(polygons.size() * 4);

    Scalar axis_sq = sweep_axis.len_sq();
    if (axis_sq < static_cast<Scalar>(1e-8)) return {};
    Scalar axis_len_sqrt = static_cast<Scalar>(std::sqrt(static_cast<double>(axis_sq)));

    for (size_t i = 0; i < polygons.size(); ++i) {
        append_poly_parts_to_sweep_with_aura(static_cast<int>(i), 0, polygons[i], sweep_axis, axis_len_sqrt, elements, aura_multiplier);
    }

    return execute_distance_sweep<VecType, Tracer>(elements, aura_multiplier, SweepMode::Monopartite, -1, tracer);
}

template<class VecType, class Tracer = DefaultTracer>
std::vector<ComplexDistanceResult<VecType>> find_polygon_distances(
    const std::vector<Polygon<VecType>>& polygons,
    const std::vector<int>& active_indices,
    typename VecType::Scalar aura_multiplier = static_cast<typename VecType::Scalar>(0.5),
    Tracer* tracer = nullptr
) {
    using Scalar = typename VecType::Scalar;

    if (active_indices.empty() || polygons.size() < 2) return {};

    VecType sweep_axis = compute_optimal_sweep_axis(polygons);

    std::vector<PartSweepElement<VecType>> elements;
    elements.reserve(polygons.size() * 4);

    Scalar axis_sq = sweep_axis.len_sq();
    if (axis_sq < static_cast<Scalar>(1e-8)) return {};
    Scalar axis_len_sqrt = static_cast<Scalar>(std::sqrt(static_cast<double>(axis_sq)));

    std::vector<int> group_ids(polygons.size(), 0);
    for (int idx : active_indices) {
        if (idx >= 0 && idx < static_cast<int>(polygons.size())) {
            group_ids[idx] = 1;
        }
    }

    for (size_t i = 0; i < polygons.size(); ++i) {
        append_poly_parts_to_sweep_with_aura(static_cast<int>(i), group_ids[i], polygons[i], sweep_axis, axis_len_sqrt, elements, aura_multiplier);
    }

    return execute_distance_sweep<VecType, Tracer>(elements, aura_multiplier, SweepMode::Subset, -1, tracer);
}

template<class VecType, class Tracer = DefaultTracer>
std::vector<ComplexDistanceResult<VecType>> find_polygon_distances(
    const std::vector<Polygon<VecType>>& setA,
    const std::vector<Polygon<VecType>>& setB,
    typename VecType::Scalar aura_multiplier = static_cast<typename VecType::Scalar>(0.5),
    Tracer* tracer = nullptr
) {
    using Scalar = typename VecType::Scalar;
    if (setA.empty() || setB.empty()) return {};

    VecType sweep_axis = compute_optimal_sweep_axis(setA, setB);

    std::vector<PartSweepElement<VecType>> elements;
    elements.reserve((setA.size() + setB.size()) * 4);

    Scalar axis_sq = sweep_axis.len_sq();
    if (axis_sq < static_cast<Scalar>(1e-8)) return {};
    Scalar axis_len_sqrt = static_cast<Scalar>(std::sqrt(static_cast<double>(axis_sq)));

    for (size_t i = 0; i < setA.size(); ++i) {
        append_poly_parts_to_sweep_with_aura(static_cast<int>(i), 0, setA[i], sweep_axis, axis_len_sqrt, elements, aura_multiplier);
    }
    for (size_t i = 0; i < setB.size(); ++i) {
        append_poly_parts_to_sweep_with_aura(
            static_cast<int>(setA.size() + i),
            1,
            setB[i],
            sweep_axis,
            axis_len_sqrt,
            elements,
            aura_multiplier);
    }

    return execute_distance_sweep<VecType, Tracer>(
        elements, aura_multiplier, SweepMode::Bipartite, static_cast<int>(setA.size()), tracer);
}
