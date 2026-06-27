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
}
