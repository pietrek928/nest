#include <nanobind/nanobind.h>
namespace nb = nanobind;
#include <nanobind/stl/pair.h>
#include <nanobind/stl/vector.h>

#include "bind_internal.h"
#include "intersect/polygon_intersect.h"
#include "types.h"

void bind_intersect_api(nb::module_ &m) {
    m.def(
        "find_polygon_intersections",
        [](std::vector<GeometryHolder> polygons) {
            return find_polygon_intersections<Vec2d>(solids_from_holders(std::move(polygons)));
        },
        nb::arg("polygons"));

    m.def(
        "find_polygon_intersections_active",
        [](std::vector<GeometryHolder> polygons,
           const std::vector<int> &active_indices) {
            return find_polygon_intersections<Vec2d>(
                solids_from_holders(std::move(polygons)), active_indices);
        },
        nb::arg("polygons"),
        nb::arg("active_indices"));

    m.def(
        "find_polygon_intersections_bipartite",
        [](std::vector<GeometryHolder> set_a,
           std::vector<GeometryHolder> set_b) {
            return find_polygon_intersections<Vec2d>(
                solids_from_holders(std::move(set_a)), solids_from_holders(std::move(set_b)));
        },
        nb::arg("set_a"),
        nb::arg("set_b"));
}
