#include <nanobind/nanobind.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/tuple.h>
#include <nanobind/stl/vector.h>
#include <cmath>
#include <string>
#include <vector>

#include "decompose.h"
#include "point_in_solid.h"
#include "solid_geometry.h"
#include "polygon_distance.h"
#include "polygon_intersect.h"
#include "guide.h"
#include <vec.h>

namespace nb = nanobind;

using Vec2d = Vec<2, double>;
using SolidGeometry2d = SolidGeometry<Vec2d>;
using DistanceResult2d = ComplexDistanceResult<Vec2d>;
using GuidanceConfig2d = GuidanceConfig<Vec2d>;
using PlacementGuidance2d = PlacementGuidance<Vec2d>;

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

static nb::tuple vec2d_to_tuple(const Vec2d& v) {
    return nb::make_tuple(v[0], v[1]);
}

static Vec2d vec2d_from_tuple(nb::object o) {
    double x = 0.0;
    double y = 0.0;
    if (!read_xy(o, x, y)) {
        throw nb::type_error("expected a length-2 tuple or sequence for Vec2");
    }
    return Vec2d({x, y});
}

static bool read_transform(nb::handle t, double& x, double& y, double& angle) {
    if (nb::isinstance<nb::tuple>(t) || nb::isinstance<nb::sequence>(t)) {
        nb::sequence s = nb::borrow<nb::sequence>(t);
        if (nb::len(s) < 3) {
            return false;
        }
        x = nb::cast<double>(s[0]);
        y = nb::cast<double>(s[1]);
        angle = nb::cast<double>(s[2]);
        return true;
    }
    return false;
}

static nb::tuple circle_bounds_tuple(const SolidGeometry2d& g) {
    const auto& c = g.get_bounding_circle();
    const double cx = c.center()[0];
    const double cy = c.center()[1];
    const double r = std::sqrt(static_cast<double>(c.square_radius()));
    return nb::make_tuple(cx - r, cy - r, cx + r, cy + r);
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
            "from_convex_polygon",
            [](nb::object points) {
                auto pts = points_from_iterable(points);
                if (pts.size() < 3) {
                    throw nb::value_error(
                        "from_convex_polygon: need at least 3 distinct points");
                }
                SolidGeometry2d poly;
                poly.add_boundary_ring(pts);
                std::vector<Vec2d> closed = pts;
                const Vec2d& first = closed.front();
                const Vec2d& last = closed.back();
                if (first[0] != last[0] || first[1] != last[1]) {
                    closed.push_back(first);
                }
                poly.append_line_poly(
                    closed.data(),
                    static_cast<int>(closed.size()),
                    false);
                poly.finalize();
                return poly;
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
            "translate",
            [](const SolidGeometry2d& g, double dx, double dy) {
                return g.translate(Vec2d({dx, dy}));
            },
            nb::arg("dx"),
            nb::arg("dy"))
        .def(
            "translate",
            [](const SolidGeometry2d& g, nb::object offset) {
                double x = 0.0;
                double y = 0.0;
                if (!read_xy(offset, x, y)) {
                    throw nb::type_error(
                        "translate(offset): expected a length-2 tuple or sequence");
                }
                return g.translate(Vec2d({x, y}));
            },
            nb::arg("offset"))
        .def(
            "rotate",
            [](const SolidGeometry2d& g, double angle, nb::object origin) {
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
                return g.rotate(angle, o);
            },
            nb::arg("angle"),
            nb::arg("origin") = nb::none())
        .def(
            "apply_transform",
            [](const SolidGeometry2d& g, nb::args args) {
                double x = 0.0;
                double y = 0.0;
                double angle = 0.0;
                if (args.size() == 1 && read_transform(args[0], x, y, angle)) {
                    return g.rotate(angle).translate(Vec2d({x, y}));
                }
                if (args.size() == 3) {
                    x = nb::cast<double>(args[0]);
                    y = nb::cast<double>(args[1]);
                    angle = nb::cast<double>(args[2]);
                    return g.rotate(angle).translate(Vec2d({x, y}));
                }
                throw nb::type_error(
                    "apply_transform: expected (x, y, angle) or a length-3 sequence");
            })
        .def(
            "center",
            [](const SolidGeometry2d& g) {
                const auto& c = g.get_bounding_circle();
                auto cen = c.center();
                return nb::make_tuple(cen[0], cen[1]);
            })
        .def(
            "radius",
            [](const SolidGeometry2d& g) {
                return std::sqrt(
                    static_cast<double>(g.get_bounding_circle().square_radius()));
            })
        .def("bounds", &circle_bounds_tuple)
        .def(
            "vertices",
            [](const SolidGeometry2d& g) {
                nb::list out;
                for (const auto& p : g.line_points) {
                    out.append(nb::make_tuple(p[0], p[1]));
                }
                return out;
            })
        .def(
            "contains_point",
            [](const SolidGeometry2d& g, double x, double y) {
                return is_point_inside_solid_space(Vec2d({x, y}), g);
            },
            nb::arg("x"),
            nb::arg("y"))
        .def(
            "intersects",
            [](const SolidGeometry2d& a, const SolidGeometry2d& b) {
                return !find_polygon_intersections<Vec2d>({a, b}).empty();
            },
            nb::arg("other"))
        .def(
            "intersects_any",
            [](const SolidGeometry2d& a,
               const std::vector<SolidGeometry2d>& others) {
                if (others.empty()) {
                    return false;
                }
                return !find_polygon_intersections<Vec2d>({a}, others).empty();
            },
            nb::arg("others"))
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

    nb::class_<GuidanceConfig2d>(m, "GuidanceConfig")
        .def(nb::init<>())
        .def_rw("use_target_attractor", &GuidanceConfig2d::use_target_attractor)
        .def_rw("target_angle_rad", &GuidanceConfig2d::target_angle_rad)
        .def_rw("use_gravity", &GuidanceConfig2d::use_gravity)
        .def_rw("attraction_weight", &GuidanceConfig2d::attraction_weight)
        .def_rw("alignment_weight", &GuidanceConfig2d::alignment_weight)
        .def_rw("search_radius", &GuidanceConfig2d::search_radius)
        .def_prop_rw(
            "target_position",
            [](const GuidanceConfig2d& c) { return vec2d_to_tuple(c.target_position); },
            [](GuidanceConfig2d& c, nb::object o) {
                c.target_position = vec2d_from_tuple(o);
            })
        .def_prop_rw(
            "gravity_vector",
            [](const GuidanceConfig2d& c) { return vec2d_to_tuple(c.gravity_vector); },
            [](GuidanceConfig2d& c, nb::object o) {
                c.gravity_vector = vec2d_from_tuple(o);
            });

    nb::class_<PlacementGuidance2d>(m, "PlacementGuidance")
        .def_ro("is_penetrating", &PlacementGuidance2d::is_penetrating)
        .def_ro("suggested_rotation_rad", &PlacementGuidance2d::suggested_rotation_rad)
        .def_ro("clearance", &PlacementGuidance2d::clearance)
        .def_prop_ro(
            "ejection_vector",
            [](const PlacementGuidance2d& g) {
                return vec2d_to_tuple(g.ejection_vector);
            })
        .def_prop_ro(
            "suggested_translation",
            [](const PlacementGuidance2d& g) {
                return vec2d_to_tuple(g.suggested_translation);
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

    m.def(
        "evaluate_local_placement",
        [](int placed_poly_idx,
           const std::vector<SolidGeometry2d>& polygons,
           nb::object current_position,
           nb::object config) {
            Vec2d pos = vec2d_from_tuple(current_position);
            GuidanceConfig2d cfg;
            if (!config.is_none()) {
                cfg = nb::cast<GuidanceConfig2d>(config);
            }
            return evaluate_local_placement<Vec2d>(
                placed_poly_idx, polygons, pos, cfg);
        },
        nb::arg("placed_poly_idx"),
        nb::arg("polygons"),
        nb::arg("current_position"),
        nb::arg("config") = nb::none());
}
