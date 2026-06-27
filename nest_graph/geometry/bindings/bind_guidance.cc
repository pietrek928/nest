#include <cmath>
#include <tuple>
#include <utility>

#include <nanobind/nanobind.h>
namespace nb = nanobind;
#include <nanobind/stl/string.h>
#include <nanobind/stl/tuple.h>
#include <nanobind/stl/vector.h>

#include "bind_internal.h"
#include "common/tracer.h"
#include "guide/guide.h"
#include "python_converters.h"
#include "types.h"

void bind_guidance_types(nb::module_ &m) {
    nb::class_<GuidanceConfig2d>(m, "GuidanceConfig")
        .def(nb::init<>())
        .def_rw("minimum_placing_distance", &GuidanceConfig2d::minimum_placing_distance)
        .def_rw("use_tight_packing", &GuidanceConfig2d::use_tight_packing)
        .def_rw("use_corner_alignment", &GuidanceConfig2d::use_corner_alignment)
        .def_rw("use_hole_seeking", &GuidanceConfig2d::use_hole_seeking)
        .def_rw("max_hole_size_ratio", &GuidanceConfig2d::max_hole_size_ratio)
        .def_rw("use_gravity", &GuidanceConfig2d::use_gravity)
        .def_rw("use_target_attractor", &GuidanceConfig2d::use_target_attractor)
        .def_rw("target_angle_rad", &GuidanceConfig2d::target_angle_rad)
        .def_rw("max_propositions", &GuidanceConfig2d::max_propositions)
        .def_rw("diversity_distance_threshold", &GuidanceConfig2d::diversity_distance_threshold)
        .def_rw("diversity_angle_rad_threshold", &GuidanceConfig2d::diversity_angle_rad_threshold)
        .def_rw("enable_grid_exploration", &GuidanceConfig2d::enable_grid_exploration)
        .def_rw("max_alternative_angles", &GuidanceConfig2d::max_alternative_angles)
        .def_rw("slide_escape_multiplier", &GuidanceConfig2d::slide_escape_multiplier)
        .def_rw("attraction_weight", &GuidanceConfig2d::attraction_weight)
        .def_rw("alignment_weight", &GuidanceConfig2d::alignment_weight)
        .def_rw("escape_radius_multiplier", &GuidanceConfig2d::escape_radius_multiplier)
        .def_rw("search_radius", &GuidanceConfig2d::search_radius)
        .def_rw("use_board_edge_cast", &GuidanceConfig2d::use_board_edge_cast)
        .def_rw("max_hole_seeks", &GuidanceConfig2d::max_hole_seeks)
        .def_rw("max_void_polygons_for_holes", &GuidanceConfig2d::max_void_polygons_for_holes)
        .def_prop_rw(
            "target_position",
            [](const GuidanceConfig2d &c) { return vec2d_to_tuple(c.target_position); },
            [](GuidanceConfig2d &c, nb::handle o) {
                c.target_position = vec2d_from_tuple(o);
            })
        .def_prop_rw(
            "gravity_vector",
            [](const GuidanceConfig2d &c) { return vec2d_to_tuple(c.gravity_vector); },
            [](GuidanceConfig2d &c, nb::handle o) {
                c.gravity_vector = vec2d_from_tuple(o);
            });

    nb::class_<PlacementProposition2d>(m, "PlacementProposition")
        .def_ro("rotation_rad", &PlacementProposition2d::rotation_rad)
        .def_ro("heuristic_score", &PlacementProposition2d::heuristic_score)
        .def_ro("move_type", &PlacementProposition2d::move_type)
        .def_ro("closest_poly_idx", &PlacementProposition2d::closest_poly_idx)
        .def_prop_ro(
            "translation",
            [](const PlacementProposition2d &p) {
                return vec2d_to_tuple(p.translation);
            });

    nb::class_<PlacementGuidance2d>(m, "PlacementGuidance")
        .def_ro("is_penetrating", &PlacementGuidance2d::is_penetrating)
        .def_ro("clearance", &PlacementGuidance2d::clearance)
        .def_prop_ro(
            "propositions",
            [](const PlacementGuidance2d &g) {
                nb::list out;
                for (const auto &p : g.propositions) {
                    out.append(p);
                }
                return out;
            });
}

void bind_guidance_api(nb::module_ &m) {
    m.def(
        "evaluate_local_placement",
        [](int placed_poly_idx,
           const std::vector<GeometryHolder> &polygons,
           nb::handle current_position,
           nb::handle config) {
            if (placed_poly_idx < 0 || placed_poly_idx >= static_cast<int>(polygons.size())) {
                throw nb::index_error("placed_poly_idx out of bounds");
            }
            Vec2d pos = vec2d_from_tuple(current_position);
            GuidanceConfig2d cfg;
            if (!config.is_none()) {
                cfg = nb::cast<GuidanceConfig2d>(config);
            }
            return evaluate_local_placement<Vec2d>(
                placed_poly_idx, solids_from_holders(polygons), pos, cfg);
        },
        nb::arg("placed_poly_idx"),
        nb::arg("polygons"),
        nb::arg("current_position"),
        nb::arg("config") = nb::none());

    m.def(
        "evaluate_local_placement_traced",
        [](int placed_poly_idx,
           const std::vector<GeometryHolder> &polygons,
           nb::handle current_position,
           nb::handle config) {
            if (placed_poly_idx < 0 || placed_poly_idx >= static_cast<int>(polygons.size())) {
                throw nb::index_error("placed_poly_idx out of bounds");
            }
            Vec2d pos = vec2d_from_tuple(current_position);
            GuidanceConfig2d cfg;
            if (!config.is_none()) {
                cfg = nb::cast<GuidanceConfig2d>(config);
            }
            DebugTracer tracer;
            tracer.reset();
            auto guidance = evaluate_local_placement<Vec2d, DebugTracer>(
                placed_poly_idx, solids_from_holders(polygons), pos, cfg, &tracer);
            return std::make_tuple(
                guidance,
                tracer.stat_sweep_pairs,
                tracer.stat_circle_pruned,
                tracer.stat_gjk_evals);
        },
        nb::arg("placed_poly_idx"),
        nb::arg("polygons"),
        nb::arg("current_position"),
        nb::arg("config") = nb::none());
}
