#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "bindings/geometry_factory.h"
#include "common/geometry_common.h"
#include "distance/static_collision_scene.h"
#include "solid/solid_geometry.h"

enum class PlacementRankMode : int {
    Clearance = 0,
    Border = 1,
    Contact = 2,
    ContactHybrid = 3,
    Legacy = 4,
    Hybrid = 5,
    RuleHybrid = 6,
};

struct PlacementRankConfig {
    double min_dist = 0.0;
    double clearance_weight = 0.25;
    double tightness_weight = 0.15;
    double edge_free_weight = 6.0;
    double edge_free_band_mult = 3.5;
    double kiss_tol = 0.0;       // outline_kiss_tolerance(min_dist); filled by caller
    double tight_scale = 0.0;    // = band; filled by caller
    double part_area = 0.0;
    double sheet_area = 1.0;
    double clearance_margin_eps = 1e-9;
    PlacementRankMode mode = PlacementRankMode::ContactHybrid;
};

struct PlacementRankResult {
    bool valid = false;
    double contact_err = 0.0;
    double contact_score = 0.0;  // signed: -contact_err
    double clearance = 0.0;
    double excess = 0.0;
    double kiss = 0.0;
    double d_board = 0.0;
    double d_pack = 0.0;
    double edge_free = 0.0;
    double rank_score = 0.0;     // signed; propose sort key
    double quality = 0.0;        // non-negative; selection/DFS
};

inline double edge_free_term(
    double d_board,
    double d_pack,
    double band,
    double area_frac,
    double weight
) {
    if (!(band > 0.0) || !(weight > 0.0)) {
        return 0.0;
    }
    const double near_board = std::max(0.0, 1.0 - d_board / band);
    const double near_pack = std::max(0.0, 1.0 - d_pack / band);
    if (!(near_board > 0.0) || !(near_pack > 0.0)) {
        return 0.0;
    }
    const double harm =
        (2.0 * near_board * near_pack) / (near_board + near_pack + 1e-6);
    return harm * std::max(0.0, area_frac) * weight;
}

inline double placement_rank_band(const PlacementRankConfig &cfg) {
    return std::max(0.0, cfg.edge_free_band_mult * cfg.min_dist);
}

inline void fill_rank_scales(PlacementRankConfig &cfg) {
    const double band = placement_rank_band(cfg);
    if (!(cfg.kiss_tol > 0.0)) {
        cfg.kiss_tol = std::max(cfg.min_dist * 2.0, 1e-6);
    }
    if (!(cfg.tight_scale > 0.0)) {
        cfg.tight_scale = std::max(band, 1e-6);
    }
}

template <class VecType>
inline double solid_standoff_distance(
    const SolidGeometry<VecType> &part,
    const SolidGeometry<VecType> &ring
) {
    const auto pair = standoff_distance_pair(part, ring);
    if (pair.core.intersect) {
        return 0.0;
    }
    if (!(pair.core.distance_sq < std::numeric_limits<double>::max())) {
        return std::numeric_limits<double>::infinity();
    }
    return std::sqrt(static_cast<double>(pair.core.distance_sq));
}

template <class VecType>
inline double solid_min_distance(
    const SolidGeometry<VecType> &a,
    const SolidGeometry<VecType> &b
) {
    const auto pair = min_distance_pair(a, b);
    if (pair.core.intersect) {
        return 0.0;
    }
    if (!(pair.core.distance_sq < std::numeric_limits<double>::max())) {
        return std::numeric_limits<double>::infinity();
    }
    return std::sqrt(static_cast<double>(pair.core.distance_sq));
}

template <class VecType>
inline void nearest_pack_distance(
    const SolidGeometry<VecType> &placed,
    const StaticCollisionScene<VecType> &scene,
    bool has_packed,
    double &out_clearance,
    double &out_d_pack,
    bool &out_penetrating
) {
    out_clearance = std::numeric_limits<double>::infinity();
    out_d_pack = std::numeric_limits<double>::infinity();
    out_penetrating = false;
    if (!has_packed || scene.obstacles.empty()) {
        return;
    }
    const auto hits = scene.query_placed(placed);
    for (const auto &r : hits) {
        if (r.intersect || r.penetration_sq > nest_packing_penetration_eps_sq<
                typename VecType::Scalar>()) {
            out_penetrating = true;
            out_clearance = 0.0;
            out_d_pack = 0.0;
            return;
        }
        const double d = std::sqrt(static_cast<double>(r.distance_sq));
        out_clearance = std::min(out_clearance, d);
        out_d_pack = std::min(out_d_pack, d);
    }
}

inline PlacementRankResult assemble_placement_rank(
    double d_board,
    double d_pack,
    double clearance,
    bool penetrating,
    bool has_packed,
    bool has_focal,
    double d_focal,
    const PlacementRankConfig &cfg
) {
    PlacementRankResult out{};
    out.d_board = d_board;
    out.d_pack = has_packed ? d_pack : std::numeric_limits<double>::infinity();
    out.clearance = has_packed ? clearance : std::numeric_limits<double>::infinity();

    const double min_dist = cfg.min_dist;
    out.kiss = std::abs(d_board - min_dist);
    out.excess = has_packed ? std::max(0.0, d_pack - min_dist) : 0.0;

    double contact_err = out.kiss;
    if (has_focal) {
        contact_err += std::abs(d_focal - min_dist);
    }
    out.contact_err = contact_err;
    out.contact_score = -contact_err;

    const double band = placement_rank_band(cfg);
    const double sheet_area = std::max(cfg.sheet_area, 1e-12);
    const double area_frac = std::max(0.0, cfg.part_area) / sheet_area;
    out.edge_free = has_packed
        ? edge_free_term(d_board, d_pack, band, area_frac, cfg.edge_free_weight)
        : 0.0;

    const double kiss_tol = std::max(cfg.kiss_tol, 1e-6);
    const double tight_scale = std::max(cfg.tight_scale, 1e-6);
    const double q_contact = std::max(0.0, 1.0 - contact_err / kiss_tol);
    const double q_clearance = (has_packed && band > 0.0)
        ? std::min(clearance, band) / band
        : 0.0;
    const double tight_cost = 2.0 * out.excess + out.kiss;
    const double q_tight = std::max(0.0, 1.0 - tight_cost / tight_scale);
    const double edge_free_norm = (cfg.edge_free_weight > 0.0)
        ? out.edge_free / cfg.edge_free_weight
        : 0.0;
    out.quality = q_contact
        + cfg.clearance_weight * q_clearance
        + cfg.tightness_weight * q_tight
        + edge_free_norm;

    const double capped_clearance = has_packed
        ? std::min(clearance, band)
        : 0.0;
    const double tightness = -(2.0 * out.excess + out.kiss);

    switch (cfg.mode) {
        case PlacementRankMode::Clearance:
            out.rank_score = has_packed ? clearance : 0.0;
            break;
        case PlacementRankMode::Border:
            out.rank_score = -out.kiss;
            break;
        case PlacementRankMode::Contact:
            out.rank_score = out.contact_score;
            break;
        case PlacementRankMode::Legacy:
            // Propose-only path; C++ batch may still be called — treat as clearance.
            out.rank_score = has_packed ? clearance : 0.0;
            break;
        case PlacementRankMode::Hybrid:
        case PlacementRankMode::RuleHybrid:
        case PlacementRankMode::ContactHybrid:
        default:
            out.rank_score = out.contact_score
                + cfg.clearance_weight * capped_clearance
                + cfg.tightness_weight * tightness
                + out.edge_free;
            break;
    }

    const double gate = min_dist - cfg.clearance_margin_eps;
    out.valid = !penetrating;
    if (has_packed && out.valid && clearance < gate) {
        out.valid = false;
    }
    return out;
}

template <class VecType>
inline PlacementRankResult score_placed_contact_hybrid(
    const SolidGeometry<VecType> &placed,
    const SolidGeometry<VecType> *board_ring,
    const StaticCollisionScene<VecType> &scene,
    bool has_packed,
    const SolidGeometry<VecType> *focal,
    const PlacementRankConfig &cfg
) {
    double d_board = 0.0;
    if (board_ring != nullptr) {
        d_board = solid_standoff_distance(placed, *board_ring);
    }
    double clearance = 0.0;
    double d_pack = 0.0;
    bool penetrating = false;
    nearest_pack_distance(placed, scene, has_packed, clearance, d_pack, penetrating);

    bool has_focal = false;
    double d_focal = 0.0;
    if (focal != nullptr) {
        has_focal = true;
        d_focal = solid_min_distance(placed, *focal);
    }
    return assemble_placement_rank(
        d_board, d_pack, clearance, penetrating, has_packed, has_focal, d_focal,
        cfg);
}

template <class VecType>
inline std::vector<PlacementRankResult> batch_score_placed_contact_hybrid(
    const std::vector<SolidGeometry<VecType>> &placed,
    const SolidGeometry<VecType> *board_ring,
    std::vector<SolidGeometry<VecType>> packed_obstacles,
    const SolidGeometry<VecType> *focal,
    PlacementRankConfig cfg
) {
    fill_rank_scales(cfg);
    const bool has_packed = !packed_obstacles.empty();
    StaticCollisionScene<VecType> scene;
    if (has_packed) {
        scene.build(std::move(packed_obstacles), static_cast<typename VecType::Scalar>(0.5));
    }
    std::vector<PlacementRankResult> out;
    out.reserve(placed.size());
    for (const auto &p : placed) {
        out.push_back(score_placed_contact_hybrid(
            p, board_ring, scene, has_packed, focal, cfg));
    }
    return out;
}

template <class VecType>
inline std::vector<PlacementRankResult> batch_rank_local_placements(
    const SolidGeometry<VecType> &part,
    const std::vector<std::tuple<double, double, double>> &transforms,
    const SolidGeometry<VecType> *board_ring,
    std::vector<SolidGeometry<VecType>> obstacles,
    const SolidGeometry<VecType> *focal,
    PlacementRankConfig cfg
) {
    fill_rank_scales(cfg);
    const bool has_packed = !obstacles.empty();
    StaticCollisionScene<VecType> scene;
    if (has_packed) {
        scene.build(std::move(obstacles), static_cast<typename VecType::Scalar>(0.5));
    }

    std::unordered_map<long long, SolidGeometry<VecType>> rotated_by_angle_key;
    auto angle_key = [](double angle) -> long long {
        return static_cast<long long>(std::llround(angle * 1e6));
    };

    std::vector<PlacementRankResult> out;
    out.reserve(transforms.size());
    for (const auto &[x, y, angle] : transforms) {
        const long long key = angle_key(angle);
        auto it = rotated_by_angle_key.find(key);
        if (it == rotated_by_angle_key.end()) {
            SolidGeometry<VecType> rotated =
                part.rotate(static_cast<typename VecType::Scalar>(angle));
            it = rotated_by_angle_key.emplace(key, std::move(rotated)).first;
        }
        SolidGeometry<VecType> placed = it->second;
        placed = placed.translate(
            VecType({static_cast<typename VecType::Scalar>(x),
                     static_cast<typename VecType::Scalar>(y)}));
        out.push_back(score_placed_contact_hybrid(
            placed, board_ring, scene, has_packed, focal, cfg));
    }
    return out;
}
