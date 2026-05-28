#include <cmath>
#include <cstdint>
#include <random>
#include <utility>
#include <vector>

#include <nanobind/nanobind.h>
namespace nb = nanobind;
#include <nanobind/stl/pair.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/tuple.h>
#include <nanobind/stl/vector.h>

#include "bindings.h"
#include "shapely.h"
#include "distance/polygon_distance.h"
#include "guide/guide.h"
#include "intersect/polygon_intersect.h"
#include "solid/decompose.h"
#include "solid/containment.h"
#include "solid/point_in_solid.h"
#include "solid/solid_geometry.h"

using SolidGeometry2d = SolidGeometry<Vec2d>;
using DistanceResult2d = ComplexDistanceResult<Vec2d>;
using GuidanceConfig2d = GuidanceConfig<Vec2d>;
using PlacementGuidance2d = PlacementGuidance<Vec2d>;
using PlacementProposition2d = PlacementProposition<Vec2d>;

struct GeometryHolder {
    SolidGeometry2d solid;
    std::mt19937 rng;

    GeometryHolder()
        : rng(std::random_device{}()) {}

    explicit GeometryHolder(std::uint32_t seed)
        : rng(seed != 0 ? std::mt19937(seed) : std::mt19937(std::random_device{}())) {}

    explicit GeometryHolder(SolidGeometry2d mesh)
        : solid(std::move(mesh)), rng(std::random_device{}()) {}
};

nb::tuple circle_bounds_tuple(const GeometryHolder &g) {
    const auto &c = g.solid.get_bounding_circle();
    const double cx = c.center()[0];
    const double cy = c.center()[1];
    const double r = std::sqrt(static_cast<double>(c.square_radius()));
    return nb::make_tuple(cx - r, cy - r, cx + r, cy + r);
}

std::vector<SolidGeometry2d> solids_from_holders(
    const std::vector<GeometryHolder> &holders
) {
    std::vector<SolidGeometry2d> out;
    out.reserve(holders.size());
    for (const auto &h : holders) {
        out.push_back(h.solid);
    }
    return out;
}

void bind_geometry(nb::module_ &m) {
    nb::class_<GeometryHolder>(m, "Geometry")
        .def(nb::init<>())
        .def(nb::init<std::uint32_t>(), nb::arg("seed"))
        .def_static(
            "from_convex_polygon",
            [](nb::handle points) {
                std::vector<Vec2d> pts;
                points_from_iterable(points, pts);
                if (pts.size() < 3) {
                    throw nb::value_error(
                        "from_convex_polygon: need at least 3 distinct points");
                }
                GeometryHolder holder;
                holder.solid.add_boundary_ring(pts);
                std::vector<Vec2d> closed = pts;
                const Vec2d &first = closed.front();
                const Vec2d &last = closed.back();
                if (first[0] != last[0] || first[1] != last[1]) {
                    closed.push_back(first);
                }
                holder.solid.append_line_poly(
                    closed.data(),
                    static_cast<int>(closed.size()),
                    holder.rng,
                    false);
                holder.solid.finalize(holder.rng);
                return holder;
            },
            nb::arg("points"))
        .def_static(
            "from_shapely",
            [](nb::handle geom) {
                std::vector<std::vector<Vec2d>> outers;
                std::vector<std::vector<Vec2d>> holes;
                collect_from_shapely(geom, outers, holes);
                if (outers.empty()) {
                    throw nb::value_error(
                        "Geometry.from_shapely: no usable polygon rings "
                        "(empty or unsupported geometry types are skipped)");
                }
                return GeometryHolder(decompose_complex_polygon<Vec2d>(outers, holes));
            },
            nb::arg("geom"))
        .def(
            "append_convex_poly",
            [](GeometryHolder &holder, nb::handle points) {
                std::vector<Vec2d> pts;
                points_from_iterable(points, pts);
                holder.solid.append_line_poly(
                    pts.data(), static_cast<int>(pts.size()), holder.rng, false);
            },
            nb::arg("points"))
        .def(
            "append_convex_hole",
            [](GeometryHolder &holder, nb::handle points) {
                std::vector<Vec2d> ring = ring_from_coords(points);
                if (ring.size() < 3) {
                    return;
                }
                auto reversed = reverse_ring(ring);
                holder.solid.add_boundary_ring(reversed, true);
                process_boundary_to_convex_segments<Vec2d>(
                    reversed, holder.solid, holder.rng, true);
            },
            nb::arg("points"))
        .def(
            "finalize",
            [](GeometryHolder &holder) { holder.solid.finalize(holder.rng); })
        .def(
            "area",
            [](const GeometryHolder &holder) {
                return static_cast<double>(holder.solid.area());
            })
        .def(
            "translate",
            [](const GeometryHolder &g, double dx, double dy) {
                return GeometryHolder(g.solid.translate(Vec2d({dx, dy})));
            },
            nb::arg("dx"),
            nb::arg("dy"))
        .def(
            "translate",
            [](const GeometryHolder &g, nb::handle offset) {
                double x = 0.0;
                double y = 0.0;
                if (!read_xy(offset, x, y)) {
                    throw nb::type_error(
                        "translate(offset): expected a length-2 tuple or sequence");
                }
                return GeometryHolder(g.solid.translate(Vec2d({x, y})));
            },
            nb::arg("offset"))
        .def(
            "rotate",
            [](const GeometryHolder &g, double angle, nb::handle origin) {
                Vec2d o({0.0, 0.0});
                if (!origin.is_none()) {
                    double x = 0.0;
                    double y = 0.0;
                    if (!read_xy(origin, x, y)) {
                        throw nb::type_error(
                            "rotate(..., origin): expected a length-2 tuple or sequence");
                    }
                    o = Vec2d({x, y});
                }
                return GeometryHolder(g.solid.rotate(angle, o));
            },
            nb::arg("angle"),
            nb::arg("origin") = nb::none())
        .def(
            "apply_transform",
            [](const GeometryHolder &g, nb::args args) {
                double x = 0.0;
                double y = 0.0;
                double angle = 0.0;
                if (args.size() == 1 && read_transform(args[0], x, y, angle)) {
                    return GeometryHolder(
                        g.solid.rotate(angle).translate(Vec2d({x, y})));
                }
                if (args.size() == 3) {
                    x = nb::cast<double>(args[0]);
                    y = nb::cast<double>(args[1]);
                    angle = nb::cast<double>(args[2]);
                    return GeometryHolder(
                        g.solid.rotate(angle).translate(Vec2d({x, y})));
                }
                throw nb::type_error(
                    "apply_transform: expected (x, y, angle) or a length-3 sequence");
            })
        .def(
            "center",
            [](const GeometryHolder &g) {
                const auto &c = g.solid.get_bounding_circle();
                auto cen = c.center();
                return nb::make_tuple(cen[0], cen[1]);
            })
        .def(
            "radius",
            [](const GeometryHolder &g) {
                return std::sqrt(
                    static_cast<double>(g.solid.get_bounding_circle().square_radius()));
            })
        .def("bounds", &circle_bounds_tuple)
        .def(
            "vertices",
            [](const GeometryHolder &g) {
                nb::list out;
                for (const auto &p : g.solid.line_points) {
                    out.append(nb::make_tuple(p[0], p[1]));
                }
                return out;
            })
        .def(
            "contains_point",
            [](const GeometryHolder &g, double x, double y) {
                return is_point_inside_solid_space(Vec2d({x, y}), g.solid);
            },
            nb::arg("x"),
            nb::arg("y"))
        .def(
            "footprint_inside",
            [](const GeometryHolder &inner, const GeometryHolder &outer) {
                return solid_footprint_inside(inner.solid, outer.solid);
            },
            nb::arg("container"))
        .def(
            "footprint_inside_batch",
            [](const GeometryHolder &outer, const std::vector<GeometryHolder> &inners) {
                std::vector<SolidGeometry2d> inner_solids;
                inner_solids.reserve(inners.size());
                for (const auto &h : inners) {
                    inner_solids.push_back(h.solid);
                }
                const auto flags = solid_footprint_inside(inner_solids, outer.solid);
                nb::list out;
                for (bool ok : flags) {
                    out.append(ok);
                }
                return out;
            },
            nb::arg("inners"))
        .def(
            "intersects",
            [](const GeometryHolder &a, const GeometryHolder &b) {
                return !find_polygon_intersections<Vec2d>({a.solid, b.solid}).empty();
            },
            nb::arg("other"))
        .def(
            "intersects_any",
            [](const GeometryHolder &a, const std::vector<GeometryHolder> &others) {
                if (others.empty()) {
                    return false;
                }
                auto solids = solids_from_holders(others);
                return !find_polygon_intersections<Vec2d>({a.solid}, solids).empty();
            },
            nb::arg("others"))
        .def(
            "get_bounding_circle",
            [](const GeometryHolder &poly) {
                const auto &c = poly.solid.get_bounding_circle();
                auto cen = c.center();
                double r = std::sqrt(static_cast<double>(c.square_radius()));
                return nb::make_tuple(cen[0], cen[1], r);
            })
        .def("__repr__", [](const GeometryHolder &) {
            return "<nest_graph.geometry.Geometry>";
        });

    nb::class_<GuidanceConfig2d>(m, "GuidanceConfig")
        .def(nb::init<>())
        .def_rw("minimum_placing_distance", &GuidanceConfig2d::minimum_placing_distance)
        .def_rw("use_tight_packing", &GuidanceConfig2d::use_tight_packing)
        .def_rw("squeeze_weight", &GuidanceConfig2d::squeeze_weight)
        .def_rw("use_hole_seeking", &GuidanceConfig2d::use_hole_seeking)
        .def_rw("hole_seeking_weight", &GuidanceConfig2d::hole_seeking_weight)
        .def_rw("use_gravity", &GuidanceConfig2d::use_gravity)
        .def_rw("use_target_attractor", &GuidanceConfig2d::use_target_attractor)
        .def_rw("target_angle_rad", &GuidanceConfig2d::target_angle_rad)
        .def_rw("max_propositions", &GuidanceConfig2d::max_propositions)
        .def_rw("diversity_distance_threshold", &GuidanceConfig2d::diversity_distance_threshold)
        .def_rw("diversity_angle_rad_threshold", &GuidanceConfig2d::diversity_angle_rad_threshold)
        .def_rw("enable_grid_exploration", &GuidanceConfig2d::enable_grid_exploration)
        .def_rw("grid_exploration_step", &GuidanceConfig2d::grid_exploration_step)
        .def_rw("max_alternative_angles", &GuidanceConfig2d::max_alternative_angles)
        .def_rw("slide_escape_multiplier", &GuidanceConfig2d::slide_escape_multiplier)
        .def_rw("attraction_weight", &GuidanceConfig2d::attraction_weight)
        .def_rw("alignment_weight", &GuidanceConfig2d::alignment_weight)
        .def_rw("escape_radius_multiplier", &GuidanceConfig2d::escape_radius_multiplier)
        .def_rw("search_radius", &GuidanceConfig2d::search_radius)
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

    nb::class_<DistanceResult2d>(m, "ComplexDistanceResult")
        .def_ro("polyA_idx", &DistanceResult2d::polyA_idx)
        .def_ro("polyB_idx", &DistanceResult2d::polyB_idx)
        .def_ro("partA_idx", &DistanceResult2d::partA_idx)
        .def_ro("partB_idx", &DistanceResult2d::partB_idx)
        .def_ro("intersect", &DistanceResult2d::intersect)
        .def_ro("distance_sq", &DistanceResult2d::distance_sq)
        .def_ro("penetration_sq", &DistanceResult2d::penetration_sq)
        .def_prop_ro(
            "mtv",
            [](const DistanceResult2d &r) { return nb::make_tuple(r.mtv[0], r.mtv[1]); });

    m.def(
        "find_polygon_intersections",
        [](const std::vector<GeometryHolder> &polygons) {
            return find_polygon_intersections<Vec2d>(solids_from_holders(polygons));
        },
        nb::arg("polygons"));

    m.def(
        "find_polygon_intersections_active",
        [](const std::vector<GeometryHolder> &polygons,
           const std::vector<int> &active_indices) {
            return find_polygon_intersections<Vec2d>(
                solids_from_holders(polygons), active_indices);
        },
        nb::arg("polygons"),
        nb::arg("active_indices"));

    m.def(
        "find_polygon_intersections_bipartite",
        [](const std::vector<GeometryHolder> &set_a,
           const std::vector<GeometryHolder> &set_b) {
            return find_polygon_intersections<Vec2d>(
                solids_from_holders(set_a), solids_from_holders(set_b));
        },
        nb::arg("set_a"),
        nb::arg("set_b"));

    m.def(
        "find_polygon_distances",
        [](const std::vector<GeometryHolder> &polygons, double aura) {
            return find_polygon_distances<Vec2d>(
                solids_from_holders(polygons), static_cast<Vec2d::Scalar>(aura));
        },
        nb::arg("polygons"),
        nb::arg("aura") = 0.5);

    m.def(
        "find_polygon_distances_active",
        [](const std::vector<GeometryHolder> &polygons,
           const std::vector<int> &active_indices,
           double aura) {
            return find_polygon_distances<Vec2d>(
                solids_from_holders(polygons),
                active_indices,
                static_cast<Vec2d::Scalar>(aura));
        },
        nb::arg("polygons"),
        nb::arg("active_indices"),
        nb::arg("aura") = 0.5);

    m.def(
        "find_polygon_distances_bipartite",
        [](const std::vector<GeometryHolder> &set_a,
           const std::vector<GeometryHolder> &set_b,
           double aura) {
            return find_polygon_distances<Vec2d>(
                solids_from_holders(set_a),
                solids_from_holders(set_b),
                static_cast<Vec2d::Scalar>(aura));
        },
        nb::arg("set_a"),
        nb::arg("set_b"),
        nb::arg("aura") = 0.5);

    m.def(
        "evaluate_local_placement",
        [](int placed_poly_idx,
           const std::vector<GeometryHolder> &polygons,
           nb::handle current_position,
           nb::handle config) {
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
}
