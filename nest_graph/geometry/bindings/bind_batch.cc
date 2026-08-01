#include <algorithm>
#include <cmath>
#include <tuple>
#include <unordered_map>

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
    m.def(
        "batch_check_validity",
        [](const GeometryHolder &part,
           const std::vector<std::tuple<double, double, double>> &transforms,
           const std::vector<GeometryHolder> &obstacles,
           const GuidanceConfig2d &config,
           double min_dist,
           double epsilon_ratio) {
            (void)config;
            const double margin = min_dist + std::max(
                1e-6, min_dist * epsilon_ratio
            );
            const double margin_sq = margin * margin;
            const std::vector<SolidGeometry2d> obs_solids =
                solids_from_holders(obstacles);
            StaticCollisionScene<Vec2d> scene;
            // search_radius is a distance margin / cast horizon, NOT an aura multiplier.
            scene.build(obs_solids, static_cast<Vec2d::Scalar>(0.5));

            std::unordered_map<long long, SolidGeometry2d> rotated_by_angle_key;
            auto angle_key = [](double angle) -> long long {
                return static_cast<long long>(std::llround(angle * 1e6));
            };

            std::vector<bool> out;
            out.reserve(transforms.size());
            for (const auto &[x, y, angle] : transforms) {
                if (obs_solids.empty()) {
                    out.push_back(true);
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
                out.push_back(scene.is_valid_placement(
                    placed, static_cast<Vec2d::Scalar>(margin_sq)));
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
           const std::vector<GeometryHolder> &obstacles,
           nb::handle current_position,
           const GuidanceConfig2d &config) {
            (void)current_position;
            const std::vector<SolidGeometry2d> obs_solids =
                solids_from_holders(obstacles);

            std::vector<PlacementGuidance2d> out;
            out.reserve(transforms.size());
            for (const auto &[x, y, angle] : transforms) {
                SolidGeometry2d placed = part.solid;
                placed = placed.rotate(static_cast<Vec2d::Scalar>(angle))
                                 .translate(Vec2d({x, y}));
                std::vector<SolidGeometry2d> polys;
                polys.reserve(1 + obs_solids.size());
                polys.push_back(std::move(placed));
                for (const auto &obs : obs_solids) {
                    polys.push_back(obs);
                }
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
}
