#include <limits>
#include <optional>
#include <tuple>
#include <vector>

#include <nanobind/nanobind.h>
namespace nb = nanobind;
#include <nanobind/stl/optional.h>
#include <nanobind/stl/tuple.h>
#include <nanobind/stl/vector.h>

#include "bind_internal.h"
#include "common/snap_pose.h"
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

void bind_snap_api(nb::module_ &m) {
    m.def(
        "snap_pose_to_ring",
        [](const GeometryHolder &part,
           const GeometryHolder &boundary_ring,
           nb::handle contact,
           nb::handle inward,
           double angle,
           double min_dist,
           nb::object board,
           nb::object standoff_pad) -> std::optional<std::tuple<double, double, double>> {
            nb::sequence cseq = nb::cast<nb::sequence>(contact);
            nb::sequence iseq = nb::cast<nb::sequence>(inward);
            if (nb::len(cseq) < 2 || nb::len(iseq) < 2) {
                throw nb::value_error("snap_pose_to_ring: contact/inward need length 2");
            }
            const Vec2d contact_v({
                nb::cast<double>(cseq[0]),
                nb::cast<double>(cseq[1]),
            });
            const Vec2d inward_v({
                nb::cast<double>(iseq[0]),
                nb::cast<double>(iseq[1]),
            });

            const SolidGeometry2d *board_ptr = nullptr;
            if (!board.is_none()) {
                board_ptr = &nb::cast<const GeometryHolder &>(board).solid;
            }

            std::optional<double> pad;
            if (!standoff_pad.is_none()) {
                pad = nb::cast<double>(standoff_pad);
            }

            return snap_pose_to_ring(
                part.solid,
                boundary_ring.solid,
                contact_v,
                inward_v,
                static_cast<Vec2d::Scalar>(angle),
                static_cast<Vec2d::Scalar>(min_dist),
                board_ptr,
                pad);
        },
        nb::arg("part"),
        nb::arg("boundary_ring"),
        nb::arg("contact"),
        nb::arg("inward"),
        nb::arg("angle"),
        nb::arg("min_dist"),
        nb::arg("board") = nb::none(),
        nb::arg("standoff_pad") = nb::none());
}
