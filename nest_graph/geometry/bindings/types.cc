#include "types.h"

#include <cmath>

#include <nanobind/stl/tuple.h>
#include <nanobind/stl/vector.h>

#include "python_converters.h"

nb::tuple circle_bounds_tuple(const GeometryHolder &g) {
    const auto &c = g.solid.get_bounding_circle();
    const double cx = c.center()[0];
    const double cy = c.center()[1];
    const double r = std::sqrt(static_cast<double>(c.square_radius()));
    return nb::make_tuple(cx - r, cy - r, cx + r, cy + r);
}

nb::tuple solid_aabb_tuple(const GeometryHolder &g) {
    const auto &pts = g.solid.line_points;
    if (pts.empty()) {
        return nb::make_tuple(0.0, 0.0, 0.0, 0.0);
    }
    double minx = static_cast<double>(pts[0][0]);
    double miny = static_cast<double>(pts[0][1]);
    double maxx = minx;
    double maxy = miny;
    for (std::size_t i = 1; i < pts.size(); ++i) {
        const double x = static_cast<double>(pts[i][0]);
        const double y = static_cast<double>(pts[i][1]);
        if (x < minx) {
            minx = x;
        }
        if (y < miny) {
            miny = y;
        }
        if (x > maxx) {
            maxx = x;
        }
        if (y > maxy) {
            maxy = y;
        }
    }
    return nb::make_tuple(minx, miny, maxx, maxy);
}

std::vector<SolidGeometry2d> solids_from_holders(
    const std::vector<GeometryHolder> &holders
) {
    std::vector<SolidGeometry2d> out;
    out.reserve(holders.size());
    for (const auto &h : holders) {
        out.push_back(h.solid);
    }
    return out;
}

Vec2d slide_vector_from_handle(nb::handle slide) {
    double sx = 0.0;
    double sy = 0.0;
    if (!read_xy(slide, sx, sy)) {
        throw nb::type_error(
            "slide: expected a length-2 tuple or sequence");
    }
    return Vec2d({sx, sy});
}

GeometryHolder geometry_from_line_coords(std::vector<Vec2d> pts) {
    if (pts.size() < 2) {
        throw nb::value_error("from_ring: need at least 2 distinct points");
    }
    GeometryHolder holder;
    holder.solid = solid_from_ring_coords<Vec2d>(pts, holder.rng);
    return holder;
}

GeometryHolder geometry_from_rings_coords(
    std::vector<std::vector<Vec2d>> rings
) {
    if (rings.empty()) {
        throw nb::value_error(
            "from_rings: need at least one ring with 2+ points");
    }
    GeometryHolder holder;
    holder.solid = solid_from_rings_coords<Vec2d>(rings, holder.rng);
    return holder;
}
