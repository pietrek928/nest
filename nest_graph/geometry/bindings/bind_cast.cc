#include <limits>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

#include <nanobind/nanobind.h>
namespace nb = nanobind;
#include <nanobind/stl/optional.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/tuple.h>
#include <nanobind/stl/vector.h>

#include "bind_internal.h"
#include "common/polish_se2.h"
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
        "polish_se2_part",
        [](const GeometryHolder &part,
           nb::handle pose,
           const std::vector<GeometryHolder> &obstacles,
           nb::object board,
           const std::vector<std::tuple<double, double>> &dirs,
           int n_angles,
           double max_t,
           double min_dist,
           const std::string &mode,
           nb::object pole) -> std::optional<std::tuple<double, double, double>> {
            double px = 0.0;
            double py = 0.0;
            double pth = 0.0;
            if (!read_transform(pose, px, py, pth)) {
                throw nb::value_error("polish_se2_part: pose needs (x, y, theta)");
            }
            const SolidGeometry2d *board_ptr = nullptr;
            if (!board.is_none()) {
                board_ptr = &nb::cast<const GeometryHolder &>(board).solid;
            }
            std::vector<Vec2d> dir_vecs;
            dir_vecs.reserve(dirs.size());
            for (const auto &[dx, dy] : dirs) {
                dir_vecs.push_back(Vec2d({dx, dy}));
            }
            const Vec2d *pole_ptr = nullptr;
            Vec2d pole_v;
            if (!pole.is_none()) {
                double ox = 0.0;
                double oy = 0.0;
                if (!read_xy(pole, ox, oy)) {
                    throw nb::value_error("polish_se2_part: pole needs (x, y)");
                }
                pole_v = Vec2d({ox, oy});
                pole_ptr = &pole_v;
            }
            return polish_se2_part(
                part.solid,
                static_cast<Vec2d::Scalar>(px),
                static_cast<Vec2d::Scalar>(py),
                static_cast<Vec2d::Scalar>(pth),
                solids_from_holders(obstacles),
                board_ptr,
                dir_vecs,
                n_angles,
                static_cast<Vec2d::Scalar>(max_t),
                static_cast<Vec2d::Scalar>(min_dist),
                mode,
                pole_ptr);
        },
        nb::arg("part"),
        nb::arg("pose"),
        nb::arg("obstacles"),
        nb::arg("board") = nb::none(),
        nb::arg("dirs"),
        nb::arg("n_angles") = 4,
        nb::arg("max_t"),
        nb::arg("min_dist") = 0.0,
        nb::arg("mode") = "pole",
        nb::arg("pole") = nb::none());

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
