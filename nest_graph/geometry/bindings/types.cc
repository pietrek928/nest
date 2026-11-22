#include "types.h"

#include <cmath>
#include <sstream>

#include <nanobind/stl/tuple.h>
#include <nanobind/stl/vector.h>

#include "python_converters.h"
#include "plane/decompose.h"
#include "plane/point_in_plane.h"
#include "shapely.h"

nb::tuple circle_bounds_tuple(const GeometryHolder &g) {
    const auto &c = g.plane.get_bounding_circle();
    const double cx = c.center()[0];
    const double cy = c.center()[1];
    const double r = std::sqrt(static_cast<double>(c.square_radius()));
    return nb::make_tuple(cx - r, cy - r, cx + r, cy + r);
}

nb::tuple plane_aabb_tuple(const GeometryHolder &g) {
    const auto &pts = g.plane.line_points;
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

std::vector<PlaneGeometry2d> planes_from_holders(
    std::vector<GeometryHolder> holders
) {
    std::vector<PlaneGeometry2d> out;
    out.reserve(holders.size());
    for (auto &h : holders) {
        out.push_back(std::move(h.plane));
    }
    return out;
}

std::vector<const PlaneGeometry2d *> plane_ptrs_from_holders(
    const std::vector<GeometryHolder> &holders
) {
    std::vector<const PlaneGeometry2d *> out;
    out.reserve(holders.size());
    for (const auto &h : holders) {
        out.push_back(&h.plane);
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
    holder.plane = plane_from_ring_coords<Vec2d>(pts, holder.rng);
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
    holder.plane = plane_from_rings_coords<Vec2d>(rings, holder.rng);
    return holder;
}

GeometryHolder geometry_from_stroke_coords(std::vector<Vec2d> pts) {
    if (pts.size() < 2) {
        throw nb::value_error("from_stroke: need at least 2 points");
    }
    bool closed = false;
    if (pts.size() >= 3) {
        const Vec2d &a = pts.front();
        const Vec2d &b = pts.back();
        if (a[0] == b[0] && a[1] == b[1]) {
            closed = true;
            pts.pop_back();
        }
    }
    if (pts.size() < 2) {
        throw nb::value_error("from_stroke: need at least 2 distinct points");
    }
    GeometryHolder holder;
    holder.plane.append_stroke(
        pts.data(), static_cast<int>(pts.size()), holder.rng, closed);
    holder.plane.finalize(holder.rng);
    return holder;
}

GeometryHolder geometry_from_point_xy(double x, double y) {
    GeometryHolder holder;
    holder.plane.append_point(Vec2d({x, y}), holder.rng);
    holder.plane.finalize(holder.rng);
    return holder;
}

GeometryHolder geometry_holder_from_shapely(nb::handle geom) {
    const std::string gtype = geom_type_string(geom);
    if (gtype == "Point") {
        const double x = nb::cast<double>(geom.attr("x"));
        const double y = nb::cast<double>(geom.attr("y"));
        return geometry_from_point_xy(x, y);
    }
    if (gtype == "MultiPoint") {
        GeometryHolder holder;
        bool any = false;
        for (nb::handle part : geom.attr("geoms")) {
            const double x = nb::cast<double>(part.attr("x"));
            const double y = nb::cast<double>(part.attr("y"));
            holder.plane.append_point(Vec2d({x, y}), holder.rng);
            any = true;
        }
        if (!any) {
            throw nb::value_error("Geometry.from_shapely: empty MultiPoint");
        }
        holder.plane.finalize(holder.rng);
        return holder;
    }
    if (gtype == "LineString") {
        return geometry_from_stroke_coords(ring_from_coords(geom.attr("coords")));
    }
    if (gtype == "LinearRing") {
        // Filled closed ring — outline / from_ring path.
        return geometry_from_line_coords(ring_from_coords(geom.attr("coords")));
    }
    if (gtype == "MultiLineString") {
        GeometryHolder holder;
        bool any = false;
        for (nb::handle part : geom.attr("geoms")) {
            std::vector<Vec2d> pts = ring_from_coords(part.attr("coords"));
            if (pts.size() < 2) {
                continue;
            }
            bool closed = false;
            if (pts.size() >= 3) {
                const Vec2d &a = pts.front();
                const Vec2d &b = pts.back();
                if (a[0] == b[0] && a[1] == b[1]) {
                    closed = true;
                    pts.pop_back();
                }
            }
            if (pts.size() < 2) {
                continue;
            }
            holder.plane.append_stroke(
                pts.data(), static_cast<int>(pts.size()), holder.rng, closed);
            any = true;
        }
        if (!any) {
            throw nb::value_error("Geometry.from_shapely: empty MultiLineString");
        }
        holder.plane.finalize(holder.rng);
        return holder;
    }
    std::vector<std::vector<Vec2d>> outers;
    std::vector<std::vector<Vec2d>> holes;
    collect_from_shapely(geom, outers, holes, true);
    if (outers.empty()) {
        throw nb::value_error(
            "Geometry.from_shapely: no usable polygon rings "
            "(empty or unsupported geometry types are skipped)");
    }
    return GeometryHolder(decompose_complex_polygon<Vec2d>(outers, holes));
}

static void append_ring_wkt(std::ostringstream &os, const std::vector<Vec2d> &pts, bool close) {
    os << "(";
    for (std::size_t i = 0; i < pts.size(); ++i) {
        if (i) {
            os << ", ";
        }
        os << pts[i][0] << " " << pts[i][1];
    }
    if (close && !pts.empty()) {
        const Vec2d &a = pts.front();
        const Vec2d &b = pts.back();
        if (a[0] != b[0] || a[1] != b[1]) {
            os << ", " << a[0] << " " << a[1];
        }
    }
    os << ")";
}

std::string geometry_to_wkt(const GeometryHolder &g) {
    const auto &mesh = g.plane;
    if (mesh.has_non_filled_parts() && !mesh.boundary_rings.empty()) {
        throw nb::value_error(
            "Geometry.to_string: mixed filled+stroke/point not supported");
    }

    // Points only
    std::vector<Vec2d> points;
    std::vector<std::pair<std::vector<Vec2d>, bool>> strokes;
    for (std::size_t i = 0; i < mesh.line_parts.size(); ++i) {
        const auto &part = mesh.line_parts[i];
        const Vec2d *pts = mesh.get_part_points(i);
        const int n = mesh.get_part_size(i);
        if (part.role == ChainRole::Point && n >= 1) {
            points.push_back(pts[0]);
        } else if (part.role == ChainRole::Stroke && n >= 2) {
            strokes.push_back({std::vector<Vec2d>(pts, pts + n), part.closed});
        }
    }

    if (!points.empty() && strokes.empty() && mesh.boundary_rings.empty()) {
        if (points.size() == 1) {
            std::ostringstream os;
            os << "POINT (" << points[0][0] << " " << points[0][1] << ")";
            return os.str();
        }
        std::ostringstream os;
        os << "MULTIPOINT (";
        for (std::size_t i = 0; i < points.size(); ++i) {
            if (i) {
                os << ", ";
            }
            os << "(" << points[i][0] << " " << points[i][1] << ")";
        }
        os << ")";
        return os.str();
    }

    if (!strokes.empty() && points.empty() && mesh.boundary_rings.empty()) {
        if (strokes.size() == 1) {
            std::ostringstream os;
            os << "LINESTRING ";
            append_ring_wkt(os, strokes[0].first, strokes[0].second);
            return os.str();
        }
        std::ostringstream os;
        os << "MULTILINESTRING (";
        for (std::size_t i = 0; i < strokes.size(); ++i) {
            if (i) {
                os << ", ";
            }
            append_ring_wkt(os, strokes[i].first, strokes[i].second);
        }
        os << ")";
        return os.str();
    }

    std::vector<std::vector<Vec2d>> outers;
    std::vector<std::vector<Vec2d>> holes;
    for (const auto &ring : mesh.boundary_rings) {
        if (ring.points.size() < 3) {
            continue;
        }
        if (ring.is_subtractive) {
            holes.push_back(ring.points);
        } else {
            outers.push_back(ring.points);
        }
    }
    if (outers.empty()) {
        return "GEOMETRYCOLLECTION EMPTY";
    }

    auto emit_polygon = [&](std::ostringstream &os, const std::vector<Vec2d> &outer,
                            const std::vector<std::vector<Vec2d>> &poly_holes) {
        os << "(";
        append_ring_wkt(os, outer, true);
        for (const auto &h : poly_holes) {
            os << ", ";
            append_ring_wkt(os, h, true);
        }
        os << ")";
    };

    if (outers.size() == 1) {
        std::ostringstream os;
        os << "POLYGON ";
        emit_polygon(os, outers[0], holes);
        return os.str();
    }

    // Assign each hole to the first outer that contains a sample vertex.
    std::vector<std::vector<std::vector<Vec2d>>> holes_per_outer(outers.size());
    for (const auto &hole : holes) {
        bool assigned = false;
        for (std::size_t oi = 0; oi < outers.size(); ++oi) {
            PlaneGeometry2d probe;
            probe.add_boundary_ring(outers[oi], false);
            if (is_point_inside_plane_space(hole[0], probe)) {
                holes_per_outer[oi].push_back(hole);
                assigned = true;
                break;
            }
        }
        if (!assigned) {
            throw nb::value_error(
                "Geometry.to_string: hole not inside any outer ring");
        }
    }
    std::ostringstream os;
    os << "MULTIPOLYGON (";
    for (std::size_t oi = 0; oi < outers.size(); ++oi) {
        if (oi) {
            os << ", ";
        }
        emit_polygon(os, outers[oi], holes_per_outer[oi]);
    }
    os << ")";
    return os.str();
}
