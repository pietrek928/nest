#include <cmath>
#include <limits>

#include <nanobind/nanobind.h>
namespace nb = nanobind;
#include <nanobind/stl/tuple.h>
#include <nanobind/stl/vector.h>

#include "bind_internal.h"
#include "geometry_factory.h"
#include "intersect/polygon_intersect.h"
#include "python_converters.h"
#include "shapely.h"
#include "solid/containment.h"
#include "solid/decompose.h"
#include "solid/point_in_solid.h"
#include "types.h"

void bind_geometry_class(nb::module_ &m) {
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
            "from_ring",
            [](nb::handle coords) {
                std::vector<Vec2d> pts;
                points_from_iterable(coords, pts);
                return geometry_from_line_coords(std::move(pts));
            },
            nb::arg("coords"))
        .def_static(
            "from_rings",
            [](nb::handle rings_handle) {
                std::vector<std::vector<Vec2d>> rings;
                for (nb::handle ring : nb::iter(rings_handle)) {
                    std::vector<Vec2d> pts;
                    points_from_iterable(ring, pts);
                    if (pts.size() >= 2) {
                        rings.push_back(std::move(pts));
                    }
                }
                return geometry_from_rings_coords(std::move(rings));
            },
            nb::arg("rings"))
        .def_static(
            "from_shapely",
            [](nb::handle geom) {
                if (geom_type_is(geom, "LineString")
                    || geom_type_is(geom, "LinearRing")) {
                    std::vector<Vec2d> pts = ring_from_coords(geom.attr("coords"));
                    return geometry_from_line_coords(std::move(pts));
                }
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
            "centroid",
            [](const GeometryHolder &g) {
                const auto cen = solid_centroid(g.solid);
                return nb::make_tuple(cen[0], cen[1]);
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
        .def("aabb", &solid_aabb_tuple)
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
            "fully_inside",
            [](const GeometryHolder &inner, const GeometryHolder &outer) {
                return is_solid_fully_contained(inner.solid, outer.solid);
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
            "distance",
            [](const GeometryHolder &a, const GeometryHolder &b) {
                const auto pair = min_distance_pair(a.solid, b.solid);
                if (pair.core.intersect) {
                    return 0.0;
                }
                return std::sqrt(static_cast<double>(pair.core.distance_sq));
            },
            nb::arg("other"))
        .def(
            "min_distance",
            [](const GeometryHolder &a, const GeometryHolder &b) {
                return min_distance_pair(a.solid, b.solid);
            },
            nb::arg("other"))
        .def(
            "standoff_distance",
            [](const GeometryHolder &part, const GeometryHolder &ring) {
                const auto pair = standoff_distance_pair(part.solid, ring.solid);
                if (pair.core.intersect) {
                    return 0.0;
                }
                return std::sqrt(static_cast<double>(pair.core.distance_sq));
            },
            nb::arg("ring"))
        .def(
            "standoff_min_distance",
            [](const GeometryHolder &part, const GeometryHolder &ring) {
                return standoff_distance_pair(part.solid, ring.solid);
            },
            nb::arg("ring"))
        .def(
            "cast_slide",
            [](const GeometryHolder &active,
               const std::vector<GeometryHolder> &obstacles,
               nb::handle slide,
               double max_t) {
                const Vec2d slide_vec = slide_vector_from_handle(slide);
                return cast_slide(
                    active.solid,
                    solids_from_holders(obstacles),
                    slide_vec,
                    static_cast<Vec2d::Scalar>(max_t));
            },
            nb::arg("obstacles"),
            nb::arg("slide"),
            nb::arg("max_t") = std::numeric_limits<double>::infinity())
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
}
