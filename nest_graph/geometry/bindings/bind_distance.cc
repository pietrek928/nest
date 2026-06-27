#include <cmath>

#include <nanobind/nanobind.h>
namespace nb = nanobind;
#include <nanobind/stl/tuple.h>
#include <nanobind/stl/vector.h>

#include "bind_internal.h"
#include "distance/polygon_distance.h"
#include "geometry_factory.h"
#include "types.h"

void bind_distance_types(nb::module_ &m) {
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
            [](const DistanceResult2d &r) { return nb::make_tuple(r.mtv[0], r.mtv[1]); })
        .def_prop_ro(
            "distance",
            [](const DistanceResult2d &r) {
                if (r.intersect) {
                    return 0.0;
                }
                return std::sqrt(static_cast<double>(r.distance_sq));
            });

    nb::class_<PairDistanceResult2d>(m, "MinDistanceResult")
        .def_prop_ro(
            "polyA_idx",
            [](const PairDistanceResult2d &r) { return r.core.polyA_idx; })
        .def_prop_ro(
            "polyB_idx",
            [](const PairDistanceResult2d &r) { return r.core.polyB_idx; })
        .def_prop_ro(
            "partA_idx",
            [](const PairDistanceResult2d &r) { return r.core.partA_idx; })
        .def_prop_ro(
            "partB_idx",
            [](const PairDistanceResult2d &r) { return r.core.partB_idx; })
        .def_prop_ro(
            "intersect",
            [](const PairDistanceResult2d &r) { return r.core.intersect; })
        .def_prop_ro(
            "distance_sq",
            [](const PairDistanceResult2d &r) { return r.core.distance_sq; })
        .def_prop_ro(
            "penetration_sq",
            [](const PairDistanceResult2d &r) { return r.core.penetration_sq; })
        .def_prop_ro(
            "distance",
            [](const PairDistanceResult2d &r) {
                if (r.core.intersect) {
                    return 0.0;
                }
                return std::sqrt(static_cast<double>(r.core.distance_sq));
            })
        .def_prop_ro(
            "closest_a",
            [](const PairDistanceResult2d &r) {
                return nb::make_tuple(r.closest_a[0], r.closest_a[1]);
            })
        .def_prop_ro(
            "closest_b",
            [](const PairDistanceResult2d &r) {
                return nb::make_tuple(r.closest_b[0], r.closest_b[1]);
            })
        .def_prop_ro(
            "mtv",
            [](const PairDistanceResult2d &r) {
                return nb::make_tuple(r.core.mtv[0], r.core.mtv[1]);
            });
}

void bind_distance_api(nb::module_ &m) {
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
        "min_distance_pair",
        [](const GeometryHolder &a, const GeometryHolder &b) {
            return min_distance_pair(a.solid, b.solid);
        },
        nb::arg("a"),
        nb::arg("b"));

    m.def(
        "standoff_distance_pair",
        [](const GeometryHolder &part, const GeometryHolder &ring) {
            return standoff_distance_pair(part.solid, ring.solid);
        },
        nb::arg("part"),
        nb::arg("ring"));
}
