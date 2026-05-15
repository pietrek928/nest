#include <nanobind/nanobind.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/tuple.h>
#include <nanobind/stl/vector.h>
#include <cmath>
#include <vector>

#include "poly.h"
#include "poly_distance.h"
#include "poly_intersect.h"
#include <vec.h>

namespace nb = nanobind;

using Vec2d = Vec<2, double>;
using Polygon2d = Polygon<Vec2d>;
using DistanceResult2d = ComplexDistanceResult<Vec2d>;

static std::vector<Vec2d> points_from_iterable(nb::handle points) {
    std::vector<Vec2d> out;
    for (nb::handle h : points) {
        nb::tuple t = nb::cast<nb::tuple>(h);
        double x = nb::cast<double>(t[0]);
        double y = nb::cast<double>(t[1]);
        out.push_back(Vec2d({x, y}));
    }
    return out;
}

NB_MODULE(poly_module, m) {
    // Intersect/distance entry points compute the 1D sweep direction internally (see sweep.h).
    // There is no sweep_axis argument from Python.

    nb::class_<Polygon2d>(m, "Polygon")
        .def(nb::init<>())
        .def(
            "append_convex_poly",
            [](Polygon2d &poly, nb::handle points) {
                auto pts = points_from_iterable(points);
                poly.append_convex_poly(pts.data(), static_cast<int>(pts.size()));
            },
            nb::arg("points"))
        .def(
            "append_convex_hole",
            [](Polygon2d &poly, nb::handle points) {
                auto pts = points_from_iterable(points);
                poly.append_convex_hole(pts.data(), static_cast<int>(pts.size()));
            },
            nb::arg("points"))
        .def("finalize", &Polygon2d::finalize)
        .def(
            "get_bounding_circle",
            [](const Polygon2d &poly) {
                const auto &c = poly.get_bounding_circle();
                auto cen = c.center();
                double r =
                    std::sqrt(static_cast<double>(c.square_radius()));
                return nb::make_tuple(cen[0], cen[1], r);
            })
        .def("__repr__", [](const Polygon2d &) {
            return "<nest_graph.poly_module.Polygon>";
        });

    // Sweep axis is always chosen inside the engine from bounds (no sweep_axis parameter).
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
            [](const DistanceResult2d &r) {
                return nb::make_tuple(r.mtv[0], r.mtv[1]);
            });

    m.def(
        "find_polygon_intersections",
        [](const std::vector<Polygon2d> &polygons) {
            return find_polygon_intersections(polygons);
        },
        nb::arg("polygons"));

    m.def(
        "find_polygon_intersections_active",
        [](const std::vector<Polygon2d> &polygons,
           const std::vector<int> &active_indices) {
            return find_polygon_intersections(polygons, active_indices);
        },
        nb::arg("polygons"),
        nb::arg("active_indices"));

    m.def(
        "find_polygon_distances",
        [](const std::vector<Polygon2d> &polygons, double aura) {
            return find_polygon_distances<Vec2d>(
                polygons, static_cast<Vec2d::Scalar>(aura));
        },
        nb::arg("polygons"),
        nb::arg("aura") = 0.5);

    m.def(
        "find_polygon_distances_active",
        [](const std::vector<Polygon2d> &polygons,
           const std::vector<int> &active_indices,
           double aura) {
            return find_polygon_distances<Vec2d>(
                polygons,
                active_indices,
                static_cast<Vec2d::Scalar>(aura));
        },
        nb::arg("polygons"),
        nb::arg("active_indices"),
        nb::arg("aura") = 0.5);

    m.def(
        "find_polygon_distances_bipartite",
        [](const std::vector<Polygon2d> &set_a,
           const std::vector<Polygon2d> &set_b,
           double aura) {
            return find_polygon_distances<Vec2d>(
                set_a,
                set_b,
                static_cast<Vec2d::Scalar>(aura));
        },
        nb::arg("set_a"),
        nb::arg("set_b"),
        nb::arg("aura") = 0.5);
}
