#include <nanobind/nanobind.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/tuple.h>
#include <nanobind/stl/vector.h>
#include <cmath>
#include <string>
#include <vector>

#include "decompose.h"
#include "solid_geometry.h"
#include "polygon_distance.h"
#include "polygon_intersect.h"
#include <vec.h>

namespace nb = nanobind;

using Vec2d = Vec<2, double>;
using SolidGeometry2d = SolidGeometry<Vec2d>;
using DistanceResult2d = ComplexDistanceResult<Vec2d>;

static bool nb_is_empty(nb::handle geom) {
    nb::object v = geom.attr("is_empty");
    bool empty = false;
    if (nb::try_cast(v, empty)) {
        return empty;
    }
    return static_cast<bool>(nb::bool_(v));
}

static std::string nb_geom_type(nb::handle geom) {
    nb::object v = geom.attr("geom_type");
    return nb::cast<std::string>(v);
}

static bool read_xy(nb::handle pt, double& x, double& y) {
    if (nb::isinstance<nb::tuple>(pt)) {
        nb::tuple t = nb::borrow<nb::tuple>(pt);
        if (nb::len(t) < 2) {
            return false;
        }
        x = nb::cast<double>(t[0]);
        y = nb::cast<double>(t[1]);
        return true;
    }
    if (nb::isinstance<nb::sequence>(pt)) {
        nb::sequence s = nb::borrow<nb::sequence>(pt);
        if (nb::len(s) < 2) {
            return false;
        }
        x = nb::cast<double>(s[0]);
        y = nb::cast<double>(s[1]);
        return true;
    }
    return false;
}

static void append_coords(nb::handle coords, std::vector<Vec2d>& out) {
    for (nb::handle pt : nb::borrow<nb::object>(coords)) {
        double x = 0.0;
        double y = 0.0;
        if (read_xy(pt, x, y)) {
            out.push_back(Vec2d({x, y}));
        }
    }
}

static std::vector<Vec2d> points_from_iterable(nb::object points) {
    std::vector<Vec2d> out;
    append_coords(points, out);
    return out;
}

static std::vector<Vec2d> ring_from_coords(nb::object coords_iterable) {
    std::vector<Vec2d> out = points_from_iterable(coords_iterable);
    if (out.size() >= 2) {
        const Vec2d& a = out.front();
        const Vec2d& b = out.back();
        if (a[0] == b[0] && a[1] == b[1]) {
            out.pop_back();
        }
    }
    return out;
}

static void collect_polygon_rings(
    nb::handle poly,
    std::vector<std::vector<Vec2d>>& outers,
    std::vector<std::vector<Vec2d>>& holes
) {
    nb::object exterior = poly.attr("exterior");
    nb::object exterior_coords = exterior.attr("coords");
    std::vector<Vec2d> outer = ring_from_coords(exterior_coords);
    if (outer.size() >= 3) {
        outers.push_back(std::move(outer));
    }

    nb::object interiors = poly.attr("interiors");
    for (nb::handle interior : interiors) {
        nb::object coords = interior.attr("coords");
        std::vector<Vec2d> hole = ring_from_coords(coords);
        if (hole.size() >= 3) {
            holes.push_back(std::move(hole));
        }
    }
}

static void collect_from_shapely(
    nb::handle geom,
    std::vector<std::vector<Vec2d>>& outers,
    std::vector<std::vector<Vec2d>>& holes
) {
    if (nb_is_empty(geom)) {
        return;
    }

    const std::string gt = nb_geom_type(geom);

    if (gt == "Polygon") {
        collect_polygon_rings(geom, outers, holes);
        return;
    }
    if (gt == "MultiPolygon") {
        nb::object geoms = geom.attr("geoms");
        for (nb::handle part : geoms) {
            collect_polygon_rings(part, outers, holes);
        }
        return;
    }
    if (gt == "GeometryCollection") {
        nb::object geoms = geom.attr("geoms");
        for (nb::handle part : geoms) {
            collect_from_shapely(part, outers, holes);
        }
        return;
    }

    // Point, LineString, LinearRing, MultiPoint, MultiLineString: ignored for now
}

NB_MODULE(_geometry, m) {
    nb::class_<SolidGeometry2d>(m, "Geometry")
        .def(nb::init<>())
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
                return decompose_complex_polygon<Vec2d>(outers, holes);
            },
            nb::arg("geom"))
        .def(
            "append_convex_poly",
            [](SolidGeometry2d& poly, nb::object points) {
                auto pts = points_from_iterable(points);
                poly.append_line_poly(pts.data(), static_cast<int>(pts.size()), false);
            },
            nb::arg("points"))
        .def(
            "append_convex_hole",
            [](SolidGeometry2d& poly, nb::object points) {
                std::vector<Vec2d> ring = ring_from_coords(points);
                if (ring.size() < 3) {
                    return;
                }
                auto reversed = reverse_ring(ring);
                poly.add_boundary_ring(reversed, true);
                process_boundary_to_convex_segments<Vec2d>(reversed, poly, true);
            },
            nb::arg("points"))
        .def("finalize", &SolidGeometry2d::finalize)
        .def(
            "get_bounding_circle",
            [](const SolidGeometry2d& poly) {
                const auto& c = poly.get_bounding_circle();
                auto cen = c.center();
                double r =
                    std::sqrt(static_cast<double>(c.square_radius()));
                return nb::make_tuple(cen[0], cen[1], r);
            })
        .def("__repr__", [](const SolidGeometry2d&) {
            return "<nest_graph.geometry.Geometry>";
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
            [](const DistanceResult2d& r) {
                return nb::make_tuple(r.mtv[0], r.mtv[1]);
            });

    m.def(
        "find_polygon_intersections",
        [](const std::vector<SolidGeometry2d>& polygons) {
            return find_polygon_intersections<Vec2d>(polygons);
        },
        nb::arg("polygons"));

    m.def(
        "find_polygon_intersections_active",
        [](const std::vector<SolidGeometry2d>& polygons,
           const std::vector<int>& active_indices) {
            return find_polygon_intersections<Vec2d>(polygons, active_indices);
        },
        nb::arg("polygons"),
        nb::arg("active_indices"));

    m.def(
        "find_polygon_intersections_bipartite",
        [](const std::vector<SolidGeometry2d>& set_a,
           const std::vector<SolidGeometry2d>& set_b) {
            return find_polygon_intersections<Vec2d>(set_a, set_b);
        },
        nb::arg("set_a"),
        nb::arg("set_b"));

    m.def(
        "find_polygon_distances",
        [](const std::vector<SolidGeometry2d>& polygons, double aura) {
            return find_polygon_distances<Vec2d>(
                polygons, static_cast<Vec2d::Scalar>(aura));
        },
        nb::arg("polygons"),
        nb::arg("aura") = 0.5);

    m.def(
        "find_polygon_distances_active",
        [](const std::vector<SolidGeometry2d>& polygons,
           const std::vector<int>& active_indices,
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
        [](const std::vector<SolidGeometry2d>& set_a,
           const std::vector<SolidGeometry2d>& set_b,
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
