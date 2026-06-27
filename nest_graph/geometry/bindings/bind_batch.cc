#include <algorithm>
#include <cmath>
#include <tuple>

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
            const double margin = min_dist + std::max(
                1e-6, min_dist * epsilon_ratio
            );
            const double margin_sq = margin * margin;
            const std::vector<SolidGeometry2d> obs_solids =
                solids_from_holders(obstacles);
            StaticCollisionScene<Vec2d> scene;
            const auto aura = static_cast<Vec2d::Scalar>(config.search_radius);
            scene.build(obs_solids, aura);

            std::vector<bool> out;
            out.reserve(transforms.size());
            for (const auto &[x, y, angle] : transforms) {
                SolidGeometry2d placed = part.solid;
                placed = placed.rotate(static_cast<Vec2d::Scalar>(angle))
                                 .translate(Vec2d({x, y}));
                if (obs_solids.empty()) {
                    out.push_back(true);
                    continue;
                }
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
            const Vec2d base_pos = vec2d_from_tuple(current_position);
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
                polys.push_back(placed);
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
