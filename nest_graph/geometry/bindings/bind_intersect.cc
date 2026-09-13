#include <nanobind/nanobind.h>
namespace nb = nanobind;
#include <nanobind/stl/pair.h>
#include <nanobind/stl/vector.h>

#include "bind_internal.h"
#include "intersect/polygon_intersect.h"
#include "types.h"

void bind_intersect_api(nb::module_ &m) {
    // By-value holders keep solids alive for the call; engines take solid ptrs
    // (no second owned SolidGeometry vector). Do not take const& — nanobind
    // conversion temporaries can dangle into solid_ptrs_from_holders.
    m.def(
        "find_polygon_intersections",
        [](std::vector<GeometryHolder> polygons) {
            return find_polygon_intersections<Vec2d>(solid_ptrs_from_holders(polygons));
        },
        nb::arg("polygons"));

    m.def(
        "find_polygon_intersections_active",
        [](std::vector<GeometryHolder> polygons,
           const std::vector<int> &active_indices) {
            return find_polygon_intersections<Vec2d>(
                solid_ptrs_from_holders(polygons), active_indices);
        },
        nb::arg("polygons"),
        nb::arg("active_indices"));

    m.def(
        "find_polygon_intersections_bipartite",
        [](std::vector<GeometryHolder> set_a,
           std::vector<GeometryHolder> set_b) {
            return find_polygon_intersections<Vec2d>(
                solid_ptrs_from_holders(set_a), solid_ptrs_from_holders(set_b));
        },
        nb::arg("set_a"),
        nb::arg("set_b"));
}
