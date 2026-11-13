#include <algorithm>
#include <cmath>
#include <cstdint>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include <nanobind/nanobind.h>
namespace nb = nanobind;
#include <nanobind/stl/tuple.h>
#include <nanobind/stl/vector.h>

#include "bind_internal.h"
#include "distance/static_collision_scene.h"
#include "guide/guide.h"
#include "python_converters.h"
#include "types.h"

void bind_batch_api(nb::module_ &m) {
    nb::class_<StaticCollisionSceneHolder>(m, "StaticCollisionScene")
        .def_static(
            "build",
            [](std::vector<GeometryHolder> obstacles, double aura) {
                // Scene owns solids: move-into-build. No pointer Scene + keep_alive.
                StaticCollisionSceneHolder holder;
                holder.scene.build(
                    solids_from_holders(std::move(obstacles)),
                    static_cast<Vec2d::Scalar>(aura));
                return holder;
            },
            nb::arg("obstacles"),
            nb::arg("aura") = 0.5)
        .def(
            "is_valid_placement",
            [](const StaticCollisionSceneHolder &self,
               const GeometryHolder &part,
               nb::object min_dist,
               double epsilon_ratio,
               nb::object margin_sq) {
                Vec2d::Scalar ms{};
                if (!margin_sq.is_none()) {
                    ms = static_cast<Vec2d::Scalar>(nb::cast<double>(margin_sq));
                } else if (!min_dist.is_none()) {
                    const double md = nb::cast<double>(min_dist);
                    const double margin =
                        md + std::max(1e-6, md * epsilon_ratio);
                    ms = static_cast<Vec2d::Scalar>(margin * margin);
                } else {
                    throw nb::value_error(
                        "is_valid_placement: provide min_dist or margin_sq");
                }
                return self.scene.is_valid_placement(part.solid, ms);
            },
            nb::arg("part"),
            nb::arg("min_dist") = nb::none(),
            nb::arg("epsilon_ratio") = 0.05,
            nb::arg("margin_sq") = nb::none())
        .def(
            "query_any_violation",
            [](const StaticCollisionSceneHolder &self,
               const GeometryHolder &part,
               nb::object min_dist,
               double epsilon_ratio,
               nb::object margin_sq) {
                Vec2d::Scalar ms{};
                Vec2d::Scalar distance_margin{};
                if (!margin_sq.is_none()) {
                    ms = static_cast<Vec2d::Scalar>(nb::cast<double>(margin_sq));
                    distance_margin = ms > static_cast<Vec2d::Scalar>(0)
                        ? static_cast<Vec2d::Scalar>(
                              std::sqrt(static_cast<double>(ms)))
                        : static_cast<Vec2d::Scalar>(0);
                } else if (!min_dist.is_none()) {
                    const double md = nb::cast<double>(min_dist);
                    const double margin =
                        md + std::max(1e-6, md * epsilon_ratio);
                    ms = static_cast<Vec2d::Scalar>(margin * margin);
                    distance_margin = static_cast<Vec2d::Scalar>(margin);
                } else {
                    throw nb::value_error(
                        "query_any_violation: provide min_dist or margin_sq");
                }
                return self.scene.query_any_violation(
                    part.solid, ms, distance_margin);
            },
            nb::arg("part"),
            nb::arg("min_dist") = nb::none(),
            nb::arg("epsilon_ratio") = 0.05,
            nb::arg("margin_sq") = nb::none());

    m.def(
        "batch_check_validity",
        [](const GeometryHolder &part,
           const std::vector<std::tuple<double, double, double>> &transforms,
           std::vector<GeometryHolder> obstacles,
           const GuidanceConfig2d &config,
           double min_dist,
           double epsilon_ratio) {
            (void)config;
            const double margin = min_dist + std::max(
                1e-6, min_dist * epsilon_ratio
            );
            const double margin_sq = margin * margin;
            StaticCollisionScene<Vec2d> scene;
            // Scene owns solids via move-into-build (not solid-only extract / ptr Scene).
            // search_radius is a distance margin / cast horizon, NOT an aura multiplier.
            scene.build(
                solids_from_holders(std::move(obstacles)),
                static_cast<Vec2d::Scalar>(0.5));

            std::unordered_map<long long, SolidGeometry2d> rotated_by_angle_key;
            auto angle_key = [](double angle) -> long long {
                return static_cast<long long>(std::llround(angle * 1e6));
            };

            // Buffer contract: flags[i] for transforms[i]; n_flags == n_transforms.
            // Scene owns query_elements + sweep_scratch (M1/M2); no per-pose element alloc.
            std::vector<uint8_t> flags(transforms.size(), 0);
            for (std::size_t i = 0; i < transforms.size(); ++i) {
                const auto &[x, y, angle] = transforms[i];
                if (scene.obstacles.empty()) {
                    flags[i] = 1;
                    continue;
                }
                const long long key = angle_key(angle);
                auto it = rotated_by_angle_key.find(key);
                if (it == rotated_by_angle_key.end()) {
                    SolidGeometry2d rotated = part.solid;
                    rotated = rotated.rotate(static_cast<Vec2d::Scalar>(angle));
                    it = rotated_by_angle_key.emplace(key, std::move(rotated)).first;
                }
                SolidGeometry2d placed = it->second;
                placed = placed.translate(Vec2d({x, y}));
                flags[i] = scene.is_valid_placement(
                               placed, static_cast<Vec2d::Scalar>(margin_sq))
                    ? 1
                    : 0;
            }
            std::vector<bool> out;
            out.reserve(flags.size());
            for (uint8_t f : flags) {
                out.push_back(f != 0);
            }
            return out;
        },
        nb::arg("part"),
        nb::arg("transforms"),
        nb::arg("obstacles"),
        nb::arg("config"),
        nb::arg("min_dist") = 0.0,
        nb::arg("epsilon_ratio") = 0.05);

    m.def(
        "batch_evaluate_local_placement",
        [](const GeometryHolder &part,
           const std::vector<std::tuple<double, double, double>> &transforms,
           std::vector<GeometryHolder> obstacles,
           nb::handle current_position,
           const GuidanceConfig2d &config) {
            (void)current_position;
            std::vector<SolidGeometry2d> polys =
                solids_from_holders(std::move(obstacles));
            polys.insert(polys.begin(), SolidGeometry2d{});

            std::vector<PlacementGuidance2d> out;
            out.reserve(transforms.size());
            for (const auto &[x, y, angle] : transforms) {
                polys[0] = part.solid.rotate(static_cast<Vec2d::Scalar>(angle))
                               .translate(Vec2d({x, y}));
                out.push_back(evaluate_local_placement<Vec2d>(
                    0, polys, Vec2d({x, y}), config));
            }
            return out;
        },
        nb::arg("part"),
        nb::arg("transforms"),
        nb::arg("obstacles"),
        nb::arg("current_position"),
        nb::arg("config"));

    // P1: penetrate+clearance only (no proposition / rotation sweep).
    m.def(
        "batch_evaluate_local_placement_validity",
        [](const GeometryHolder &part,
           const std::vector<std::tuple<double, double, double>> &transforms,
           std::vector<GeometryHolder> obstacles,
           const GuidanceConfig2d &config) {
            std::vector<SolidGeometry2d> polys =
                solids_from_holders(std::move(obstacles));
            polys.insert(polys.begin(), SolidGeometry2d{});

            std::unordered_map<long long, SolidGeometry2d> rotated_by_angle_key;
            auto angle_key = [](double angle) -> long long {
                return static_cast<long long>(std::llround(angle * 1e6));
            };

            std::vector<std::tuple<bool, double>> out;
            out.reserve(transforms.size());
            for (const auto &[x, y, angle] : transforms) {
                const long long key = angle_key(angle);
                auto it = rotated_by_angle_key.find(key);
                if (it == rotated_by_angle_key.end()) {
                    SolidGeometry2d rotated = part.solid;
                    rotated = rotated.rotate(static_cast<Vec2d::Scalar>(angle));
                    it = rotated_by_angle_key.emplace(key, std::move(rotated)).first;
                }
                SolidGeometry2d placed = it->second;
                placed = placed.translate(Vec2d({x, y}));
                polys[0] = placed;
                PlacementGuidance2d g =
                    evaluate_local_placement_validity<Vec2d>(0, polys, config);
                out.emplace_back(
                    g.is_penetrating, static_cast<double>(g.clearance));
            }
            return out;
        },
        nb::arg("part"),
        nb::arg("transforms"),
        nb::arg("obstacles"),
        nb::arg("config"));

    // P1: margin-0 Penetrating batch (emit_packing_clear SoT). Not Scene
    // batch_check_validity (Q13/Q327 — Scene uses min_dist+ε margin).
    // AABB BVH prefilters obstacle indices; narrow phase stays solids_packing_collide.
    m.def(
        "batch_packing_clear",
        [](const GeometryHolder &part,
           const std::vector<std::tuple<double, double, double>> &transforms,
           const std::vector<GeometryHolder> &obstacles) {
            std::unordered_map<long long, SolidGeometry2d> rotated_by_angle_key;
            auto angle_key = [](double angle) -> long long {
                return static_cast<long long>(std::llround(angle * 1e6));
            };

            AabbTree aabb_tree;
            AabbTreeScratch aabb_scratch;
            std::vector<double> obs_ltrb(obstacles.size() * 4);
            bool aabb_ok = false;
            if (!obstacles.empty()) {
                for (std::size_t i = 0; i < obstacles.size(); ++i) {
                    solid_bounding_circle_ltrb(
                        obstacles[i].solid, obs_ltrb.data() + 4 * i);
                }
                aabb_ok = aabb_tree_rebuild_boxes(
                    obs_ltrb.data(), obstacles.size(), aabb_tree);
            }
            std::vector<int> cands(obstacles.size());

            std::vector<bool> out;
            out.reserve(transforms.size());
            const bool empty_obs = obstacles.empty();
            for (const auto &[x, y, angle] : transforms) {
                if (empty_obs) {
                    out.push_back(true);
                    continue;
                }
                // Non-finite pose: not clear (never treat invalid query as clear).
                if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(angle)) {
                    out.push_back(false);
                    continue;
                }
                const long long key = angle_key(angle);
                auto it = rotated_by_angle_key.find(key);
                if (it == rotated_by_angle_key.end()) {
                    SolidGeometry2d rotated = part.solid;
                    rotated = rotated.rotate(static_cast<Vec2d::Scalar>(angle));
                    it = rotated_by_angle_key.emplace(key, std::move(rotated)).first;
                }
                SolidGeometry2d placed = it->second;
                placed = placed.translate(Vec2d({x, y}));
                std::size_t n_hit = 0;
                if (aabb_ok) {
                    double pl[4];
                    solid_bounding_circle_ltrb(placed, pl);
                    if (aabb2_ltrb_valid(pl[0], pl[1], pl[2], pl[3])) {
                        n_hit = aabb_tree_query_box(
                            aabb_tree,
                            aabb_scratch,
                            pl[0],
                            pl[1],
                            pl[2],
                            pl[3],
                            cands.data(),
                            cands.size());
                    } else {
                        // Invalid placed box → brute all indices.
                        for (std::size_t i = 0; i < obstacles.size(); ++i) {
                            cands[i] = static_cast<int>(i);
                        }
                        n_hit = obstacles.size();
                    }
                } else {
                    for (std::size_t i = 0; i < obstacles.size(); ++i) {
                        cands[i] = static_cast<int>(i);
                    }
                    n_hit = obstacles.size();
                }
                bool clear = true;
                for (std::size_t i = 0; i < n_hit; ++i) {
                    if (solids_packing_collide(
                            placed,
                            obstacles[static_cast<std::size_t>(cands[i])].solid)) {
                        clear = false;
                        break;
                    }
                }
                out.push_back(clear);
            }
            return out;
        },
        nb::arg("part"),
        nb::arg("transforms"),
        nb::arg("obstacles"));
}
