#include <limits>

#include <nanobind/nanobind.h>
namespace nb = nanobind;
#include <nanobind/stl/vector.h>

#include "bind_internal.h"
#include "geometry_factory.h"
#include "types.h"

void bind_cast_types(nb::module_ &m) {
    nb::class_<CastResult2d>(m, "CastResult")
        .def_ro("intersects_path", &CastResult2d::intersects_path)
        .def_ro("t_entry", &CastResult2d::t_entry)
        .def_ro("t_exit", &CastResult2d::t_exit)
        .def_ro("polyA_idx", &CastResult2d::polyA_idx)
        .def_ro("partA_idx", &CastResult2d::partA_idx)
        .def_ro("polyB_idx", &CastResult2d::polyB_idx)
        .def_ro("partB_idx", &CastResult2d::partB_idx);
}

void bind_cast_api(nb::module_ &m) {
    m.def(
        "find_closest_polygon_cast",
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
        nb::arg("active"),
        nb::arg("obstacles"),
        nb::arg("slide"),
        nb::arg("max_t") = std::numeric_limits<double>::infinity());

    m.def(
        "find_all_polygon_casts",
        [](const GeometryHolder &active,
           const std::vector<GeometryHolder> &obstacles,
           nb::handle slide,
           double max_t) {
            const Vec2d slide_vec = slide_vector_from_handle(slide);
            return cast_slide_all(
                active.solid,
                solids_from_holders(obstacles),
                slide_vec,
                static_cast<Vec2d::Scalar>(max_t));
        },
        nb::arg("active"),
        nb::arg("obstacles"),
        nb::arg("slide"),
        nb::arg("max_t") = std::numeric_limits<double>::infinity());
}
