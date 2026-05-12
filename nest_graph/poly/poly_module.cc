#include <nanobind/nanobind.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/tuple.h>
#include <nanobind/stl/vector.h>
#include <cmath>
#include <vector>

#include "poly.h"
#include "poly_intersect.h"
#include "vec.h"

namespace nb = nanobind;

using Vec2d = Vec<2, double>;
using Polygon2d = Polygon<double, Vec2d>;

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

    m.def(
        "find_polygon_intersections",
        [](const std::vector<Polygon2d> &polygons, nb::tuple axis) {
            double ax = nb::cast<double>(axis[0]);
            double ay = nb::cast<double>(axis[1]);
            Vec2d sweep({ax, ay});
            return find_polygon_intersections(polygons, sweep);
        },
        nb::arg("polygons"),
        nb::arg("sweep_axis"));

    m.def(
        "find_polygon_intersections_active",
        [](const std::vector<Polygon2d> &polygons,
           const std::vector<int> &active_indices,
           nb::tuple axis) {
            double ax = nb::cast<double>(axis[0]);
            double ay = nb::cast<double>(axis[1]);
            Vec2d sweep({ax, ay});
            return find_polygon_intersections(polygons, active_indices, sweep);
        },
        nb::arg("polygons"),
        nb::arg("active_indices"),
        nb::arg("sweep_axis"));
}
