#include <cmath>
#include <limits>
#include <string>

#include <Python.h>
#include <nanobind/nanobind.h>
namespace nb = nanobind;
#include <nanobind/stl/tuple.h>
#include <nanobind/stl/vector.h>

#include "bind_internal.h"
#include "common/aabb_tree.h"
#include "common/polylabel.h"
#include "common/sample_boundary.h"
#include "common/voronoi_vertices.h"
#include "convex/hull.h"
#include "geometry_factory.h"
#include "intersect/polygon_intersect.h"
#include "python_converters.h"
#include "shapely.h"
#include "solid/containment.h"
#include "solid/decompose.h"
#include "solid/point_in_solid.h"
#include "types.h"

inline std::vector<double> aabb_bind_ltrb_from_tuples(
    const std::vector<std::tuple<double, double, double, double>> &boxes
) {
    std::vector<double> ltrb;
    ltrb.reserve(boxes.size() * 4);
    for (const auto &b : boxes) {
        ltrb.push_back(std::get<0>(b));
        ltrb.push_back(std::get<1>(b));
        ltrb.push_back(std::get<2>(b));
        ltrb.push_back(std::get<3>(b));
    }
    return ltrb;
}

std::vector<std::vector<Vec2d>> rings_from_geometry(const GeometryHolder &g) {
    std::vector<std::vector<Vec2d>> outers;
    std::vector<std::vector<Vec2d>> holes;
    for (const auto &ring : g.solid.boundary_rings) {
        if (ring.points.size() < 2) {
            continue;
        }
        if (ring.is_subtractive) {
            holes.push_back(ring.points);
        } else {
            outers.push_back(ring.points);
        }
    }
    if (outers.empty()) {
        return {};
    }
    // Prefer the largest outer when Geometry holds multiple shells.
    std::size_t best = 0;
    double best_area = -1.0;
    for (std::size_t i = 0; i < outers.size(); ++i) {
        double area = 0.0;
        const auto &ring = outers[i];
        for (std::size_t k = 0, j = ring.size() - 1; k < ring.size(); j = k++) {
            area += static_cast<double>(ring[j][0] * ring[k][1] - ring[k][0] * ring[j][1]);
        }
        area = std::abs(area) * 0.5;
        if (area > best_area) {
            best_area = area;
            best = i;
        }
    }
    std::vector<std::vector<Vec2d>> rings;
    rings.reserve(1 + holes.size());
    rings.push_back(std::move(outers[best]));
    for (auto &hole : holes) {
        rings.push_back(std::move(hole));
    }
    return rings;
}

std::vector<std::vector<Vec2d>> rings_from_python(nb::handle rings_handle) {
    std::vector<std::vector<Vec2d>> rings;
    for (nb::handle ring : nb::iter(rings_handle)) {
        // Close-pop via ring_from_coords; open outlines allow ≥2 verts.
        std::vector<Vec2d> pts = ring_from_coords(ring);
        if (pts.size() >= 2) {
            rings.push_back(std::move(pts));
        }
    }
    return rings;
}

// Reusable scratch for clip_ray_interior_samples (bind/batch edge owns it).
//
// Buffer contract:
//   inside_intervals: cleared each call; at most march_steps/2+1 pairs.
//   Used only during the call; samples go to caller out buffer.
struct ClipRayScratch {
    std::vector<std::pair<double, double>> inside_intervals;
};

// Sample points along inside intervals of a ray through solid.
//
// Buffer contract:
//   sample_fracs: n_fracs doubles in [0,1]; may be null only if n_fracs==0.
//   out_xy: capacity out_cap_xy doubles interleaved;
//           REQUIRE out_cap_xy >= 2 * n_fracs * n_intervals (n_intervals ≤ 33).
//           Safe UB: out_cap_xy >= 2 * n_fracs * 33.
//   return: written doubles (even), or 0 if empty/fail/cap too small (no write).
inline std::size_t clip_ray_interior_samples_xy(
    const SolidGeometry2d &solid,
    double ox,
    double oy,
    double dx,
    double dy,
    double max_t,
    const double *sample_fracs,
    std::size_t n_fracs,
    ClipRayScratch &scratch,
    double *out_xy,
    std::size_t out_cap_xy
) {
    scratch.inside_intervals.clear();
    if (max_t <= 0.0 || n_fracs == 0 || sample_fracs == nullptr) {
        return 0;
    }
    const double len = std::hypot(dx, dy);
    if (len <= 1e-15) {
        return 0;
    }
    const double ux = dx / len;
    const double uy = dy / len;
    constexpr int march_steps = 64;
    bool prev_inside = false;
    double interval_start = 0.0;
    for (int i = 0; i <= march_steps; ++i) {
        const double f = static_cast<double>(i) / static_cast<double>(march_steps);
        const double t = f * max_t;
        const double x = ox + ux * t;
        const double y = oy + uy * t;
        const bool inside = is_point_inside_solid_space(Vec2d({x, y}), solid);
        if (inside && !prev_inside) {
            interval_start = t;
        } else if (!inside && prev_inside) {
            scratch.inside_intervals.push_back({interval_start, t});
        }
        prev_inside = inside;
    }
    if (prev_inside) {
        scratch.inside_intervals.push_back({interval_start, max_t});
    }

    std::size_t need = 0;
    for (const auto &[t0, t1] : scratch.inside_intervals) {
        if (t1 - t0 <= 1e-15) {
            continue;
        }
        for (std::size_t fi = 0; fi < n_fracs; ++fi) {
            const double sf = sample_fracs[fi];
            if (sf < 0.0 || sf > 1.0) {
                continue;
            }
            need += 2;
        }
    }
    if (need == 0) {
        return 0;
    }
    if (out_xy == nullptr || out_cap_xy < need) {
        return 0;
    }
    std::size_t w = 0;
    for (const auto &[t0, t1] : scratch.inside_intervals) {
        const double span = t1 - t0;
        if (span <= 1e-15) {
            continue;
        }
        for (std::size_t fi = 0; fi < n_fracs; ++fi) {
            const double sf = sample_fracs[fi];
            if (sf < 0.0 || sf > 1.0) {
                continue;
            }
            const double t = t0 + sf * span;
            out_xy[w++] = ox + ux * t;
            out_xy[w++] = oy + uy * t;
        }
    }
    return w;
}

inline std::vector<Vec2d> clip_ray_interior_samples(
    const SolidGeometry2d &solid,
    double ox,
    double oy,
    double dx,
    double dy,
    double max_t,
    const std::vector<double> &sample_fracs
) {
    ClipRayScratch scratch;
    // UB: 2 * n_fracs * 33 doubles.
    std::vector<double> buf(sample_fracs.size() * 66, 0.0);
    const std::size_t n = clip_ray_interior_samples_xy(
        solid, ox, oy, dx, dy, max_t,
        sample_fracs.data(), sample_fracs.size(), scratch,
        buf.data(), buf.size());
    std::vector<Vec2d> out;
    out.reserve(n / 2);
    for (std::size_t i = 0; i + 1 < n; i += 2) {
        out.push_back(Vec2d({buf[i], buf[i + 1]}));
    }
    return out;
}

void bind_geometry_class(nb::module_ &m) {
    nb::class_<GeometryHolder>(m, "Geometry")
        .def(nb::init<>())
        .def(nb::init<std::uint32_t>(), nb::arg("seed"))
        .def_static(
            "from_convex_polygon",
            [](nb::handle points) {
                std::vector<Vec2d> pts;
                points_from_iterable(points, pts);
                if (pts.size() < 3) {
                    throw nb::value_error(
                        "from_convex_polygon: need at least 3 distinct points");
                }
                GeometryHolder holder;
                holder.solid.add_boundary_ring(pts);
                std::vector<Vec2d> closed = pts;
                const Vec2d &first = closed.front();
                const Vec2d &last = closed.back();
                if (first[0] != last[0] || first[1] != last[1]) {
                    closed.push_back(first);
                }
                holder.solid.append_line_poly(
                    closed.data(),
                    static_cast<int>(closed.size()),
                    holder.rng,
                    false);
                holder.solid.finalize(holder.rng);
                return holder;
            },
            nb::arg("points"))
        .def_static(
            "from_ring",
            [](nb::handle coords) {
                return geometry_from_line_coords(ring_from_coords(coords));
            },
            nb::arg("coords"))
        .def_static(
            "from_rings",
            [](nb::handle rings_handle) {
                return geometry_from_rings_coords(rings_from_python(rings_handle));
            },
            nb::arg("rings"))
        .def_static(
            "from_shapely",
            [](nb::handle geom) {
                const std::string gtype = geom_type_string(geom);
                if (gtype == "LineString" || gtype == "LinearRing") {
                    return geometry_from_line_coords(
                        ring_from_coords(geom.attr("coords")));
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
            },
            nb::arg("geom"))
        .def_static(
            "from_shapely_outline",
            [](nb::handle geom) {
                // Exterior ring(s) only — line Geometry for standoff / kiss (C0).
                const std::string gtype = geom_type_string(geom);
                if (gtype == "LineString" || gtype == "LinearRing") {
                    return geometry_from_line_coords(
                        ring_from_coords(geom.attr("coords")));
                }
                std::vector<std::vector<Vec2d>> outers;
                std::vector<std::vector<Vec2d>> holes;
                collect_from_shapely(geom, outers, holes, /*include_holes=*/false);
                if (outers.empty()) {
                    throw nb::value_error(
                        "Geometry.from_shapely_outline: no usable exterior rings");
                }
                if (outers.size() == 1) {
                    return geometry_from_line_coords(std::move(outers[0]));
                }
                return geometry_from_rings_coords(std::move(outers));
            },
            nb::arg("geom"))
        .def(
            "append_convex_poly",
            [](GeometryHolder &holder, nb::handle points) {
                std::vector<Vec2d> pts;
                points_from_iterable(points, pts);
                holder.solid.append_line_poly(
                    pts.data(), static_cast<int>(pts.size()), holder.rng, false);
            },
            nb::arg("points"))
        .def(
            "append_convex_hole",
            [](GeometryHolder &holder, nb::handle points) {
                std::vector<Vec2d> ring = ring_from_coords(points);
                if (ring.size() < 3) {
                    return;
                }
                auto reversed = reverse_ring(ring);
                holder.solid.add_boundary_ring(reversed, true);
                process_boundary_to_convex_segments<Vec2d>(
                    reversed, holder.solid, holder.rng, true);
            },
            nb::arg("points"))
        .def(
            "finalize",
            [](GeometryHolder &holder) { holder.solid.finalize(holder.rng); })
        .def(
            "area",
            [](const GeometryHolder &holder) {
                return static_cast<double>(holder.solid.area());
            })
        .def(
            "convex_hull_area",
            [](const GeometryHolder &holder) {
                return static_cast<double>(solid_convex_hull_area(holder.solid));
            })
        .def(
            "translate",
            [](const GeometryHolder &g, double dx, double dy) {
                return GeometryHolder(g.solid.translate(Vec2d({dx, dy})));
            },
            nb::arg("dx"),
            nb::arg("dy"))
        .def(
            "translate",
            [](const GeometryHolder &g, nb::handle offset) {
                double x = 0.0;
                double y = 0.0;
                if (!read_xy(offset, x, y)) {
                    throw nb::type_error(
                        "translate(offset): expected a length-2 tuple or sequence");
                }
                return GeometryHolder(g.solid.translate(Vec2d({x, y})));
            },
            nb::arg("offset"))
        .def(
            "rotate",
            [](const GeometryHolder &g, double angle, nb::handle origin) {
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
                return GeometryHolder(g.solid.rotate(angle, o));
            },
            nb::arg("angle"),
            nb::arg("origin") = nb::none())
        .def(
            "apply_transform",
            [](const GeometryHolder &g, nb::args args) {
                double x = 0.0;
                double y = 0.0;
                double angle = 0.0;
                auto apply_se2 = [](const SolidGeometry2d &solid, double x, double y,
                                    double angle) {
                    // Single-clone SE2: rotate then translate in place on the copy.
                    SolidGeometry2d out = solid.rotate(static_cast<Vec2d::Scalar>(angle));
                    const Vec2d offset({x, y});
                    for (auto &p : out.line_points) {
                        p = p + offset;
                    }
                    for (auto &ring : out.boundary_rings) {
                        for (auto &p : ring.points) {
                            p = p + offset;
                        }
                    }
                    for (auto &part : out.line_parts) {
                        part.bounding_circle.c = part.bounding_circle.c + offset;
                    }
                    out.bounding_circle.c = out.bounding_circle.c + offset;
                    return GeometryHolder(std::move(out));
                };
                if (args.size() == 1 && read_transform(args[0], x, y, angle)) {
                    return apply_se2(g.solid, x, y, angle);
                }
                if (args.size() == 3) {
                    x = nb::cast<double>(args[0]);
                    y = nb::cast<double>(args[1]);
                    angle = nb::cast<double>(args[2]);
                    return apply_se2(g.solid, x, y, angle);
                }
                throw nb::type_error(
                    "apply_transform: expected (x, y, angle) or a length-3 sequence");
            })
        .def(
            "centroid",
            [](const GeometryHolder &g) {
                const auto cen = solid_centroid(g.solid);
                return nb::make_tuple(cen[0], cen[1]);
            })
        .def(
            "center",
            [](const GeometryHolder &g) {
                const auto &c = g.solid.get_bounding_circle();
                auto cen = c.center();
                return nb::make_tuple(cen[0], cen[1]);
            })
        .def(
            "radius",
            [](const GeometryHolder &g) {
                return std::sqrt(
                    static_cast<double>(g.solid.get_bounding_circle().square_radius()));
            })
        .def("bounds", &circle_bounds_tuple)
        .def("aabb", &solid_aabb_tuple)
        .def(
            "vertices",
            [](const GeometryHolder &g) {
                nb::list out;
                for (const auto &p : g.solid.line_points) {
                    out.append(nb::make_tuple(p[0], p[1]));
                }
                return out;
            })
        .def(
            "boundary_rings",
            [](const GeometryHolder &g) {
                nb::list out;
                for (const auto &ring : g.solid.boundary_rings) {
                    nb::list coords;
                    for (const auto &p : ring.points) {
                        coords.append(nb::make_tuple(p[0], p[1]));
                    }
                    out.append(nb::make_tuple(coords, ring.is_subtractive));
                }
                return out;
            })
        .def(
            "boundary_rings_flat",
            [](const GeometryHolder &g) {
                // Flat harvest: coords xy-interleaved, ring offsets, subtractive flags.
                nb::list coords;
                nb::list offsets;
                nb::list subtractive;
                offsets.append(0);
                for (const auto &ring : g.solid.boundary_rings) {
                    for (const auto &p : ring.points) {
                        coords.append(static_cast<double>(p[0]));
                        coords.append(static_cast<double>(p[1]));
                    }
                    offsets.append(static_cast<int>(coords.size() / 2));
                    subtractive.append(ring.is_subtractive);
                }
                return nb::make_tuple(coords, offsets, subtractive);
            })
        .def(
            "contains_point",
            [](const GeometryHolder &g, double x, double y) {
                return is_point_inside_solid_space(Vec2d({x, y}), g.solid);
            },
            nb::arg("x"),
            nb::arg("y"))
        .def(
            "contains_points_batch",
            [](const GeometryHolder &g,
               const std::vector<std::tuple<double, double>> &pts) {
                // Contract: xy = 2*n doubles; flags = n bytes.
                std::vector<double> xy;
                xy.reserve(pts.size() * 2);
                for (const auto &t : pts) {
                    xy.push_back(std::get<0>(t));
                    xy.push_back(std::get<1>(t));
                }
                std::vector<uint8_t> flags(pts.size(), 0);
                contains_points_batch_xy(
                    g.solid, xy.data(), xy.size(), flags.data(), flags.size());
                nb::list out;
                for (const auto f : flags) {
                    out.append(f != 0);
                }
                return out;
            },
            nb::arg("points"),
            "Batch contains_point over [(x,y), ...] (same SoT).")
        .def(
            "sample_boundary_rings",
            [](const GeometryHolder &g, double step, bool exteriors_only) {
                // Contract: out_xy capacity >= 2 * sample_boundary_rings_n.
                const std::size_t n_pts =
                    sample_boundary_rings_n(g.solid, step, exteriors_only);
                std::vector<double> xy(n_pts * 2, 0.0);
                const std::size_t wrote = sample_boundary_rings_xy(
                    g.solid, step, exteriors_only, xy.data(), xy.size());
                std::vector<std::tuple<double, double>> out;
                out.reserve(wrote / 2);
                for (std::size_t i = 0; i + 1 < wrote; i += 2) {
                    out.emplace_back(xy[i], xy[i + 1]);
                }
                return out;
            },
            nb::arg("step"),
            nb::arg("exteriors_only") = true,
            "Arc-length densify of boundary rings → [(x,y), ...].")
        .def(
            "clip_ray_interior",
            [](const GeometryHolder &g,
               nb::tuple origin_xy,
               nb::tuple direction_xy,
               double max_t,
               nb::object sample_fracs) {
                const double ox = nb::cast<double>(origin_xy[0]);
                const double oy = nb::cast<double>(origin_xy[1]);
                const double dx = nb::cast<double>(direction_xy[0]);
                const double dy = nb::cast<double>(direction_xy[1]);
                const std::vector<double> fracs = sample_fracs_from_python(sample_fracs);
                const auto pts = clip_ray_interior_samples(
                    g.solid, ox, oy, dx, dy, max_t, fracs);
                nb::list out;
                for (const auto &p : pts) {
                    out.append(nb::make_tuple(p[0], p[1]));
                }
                return out;
            },
            nb::arg("origin_xy"),
            nb::arg("direction_xy"),
            nb::arg("max_t"),
            nb::arg("sample_fracs") = nb::make_tuple(0.1, 0.5))
        .def(
            "clip_ray_interior_batch",
            [](const GeometryHolder &g,
               nb::object origins_xy,
               nb::object directions_xy,
               double max_t,
               nb::object sample_fracs) {
                // Flat I/O: origins/dirs as length-2N float sequences (or list of pairs);
                // returns (coords_flat xy-interleaved, offsets length N+1).
                const std::vector<double> fracs = sample_fracs_from_python(sample_fracs);
                std::vector<double> origins;
                std::vector<double> dirs;
                auto fill_xy = [](nb::object seq, std::vector<double> &out) {
                    if (nb::isinstance<nb::list>(seq) || nb::isinstance<nb::sequence>(seq)) {
                        const Py_ssize_t n = nb::len(seq);
                        if (n > 0 && nb::isinstance<nb::tuple>(seq[0])) {
                            out.reserve(static_cast<std::size_t>(n) * 2);
                            for (Py_ssize_t i = 0; i < n; ++i) {
                                nb::tuple xy = nb::cast<nb::tuple>(seq[i]);
                                out.push_back(nb::cast<double>(xy[0]));
                                out.push_back(nb::cast<double>(xy[1]));
                            }
                            return;
                        }
                    }
                    for (nb::handle v : nb::iter(seq)) {
                        out.push_back(nb::cast<double>(v));
                    }
                };
                fill_xy(origins_xy, origins);
                fill_xy(directions_xy, dirs);
                if (origins.size() != dirs.size() || (origins.size() % 2) != 0) {
                    throw nb::value_error(
                        "clip_ray_interior_batch: origins/directions must be equal "
                        "even-length flat xy or list of pairs");
                }
                const size_t n = origins.size() / 2;
                // Contract: per-ray UB = 2 * n_fracs * 33 doubles; reuse one scratch.
                ClipRayScratch scratch;
                std::vector<double> ray_buf(std::max<std::size_t>(fracs.size() * 66, 8), 0.0);
                std::vector<double> coords_flat;
                coords_flat.reserve(n * fracs.size() * 2);
                nb::list offsets;
                offsets.append(0);
                for (size_t i = 0; i < n; ++i) {
                    const double ox = origins[2 * i];
                    const double oy = origins[2 * i + 1];
                    const double dx = dirs[2 * i];
                    const double dy = dirs[2 * i + 1];
                    const std::size_t wrote = clip_ray_interior_samples_xy(
                        g.solid, ox, oy, dx, dy, max_t,
                        fracs.data(), fracs.size(), scratch,
                        ray_buf.data(), ray_buf.size());
                    for (std::size_t k = 0; k < wrote; ++k) {
                        coords_flat.push_back(ray_buf[k]);
                    }
                    offsets.append(static_cast<int>(coords_flat.size() / 2));
                }
                nb::list coords;
                for (double v : coords_flat) {
                    coords.append(v);
                }
                return nb::make_tuple(coords, offsets);
            },
            nb::arg("origins_xy"),
            nb::arg("directions_xy"),
            nb::arg("max_t"),
            nb::arg("sample_fracs") = nb::make_tuple(0.1, 0.5))
        .def(
            "boundary_clearance",
            [](const GeometryHolder &g, double x, double y) {
                return static_cast<double>(
                    point_boundary_clearance(Vec2d({x, y}), g.solid));
            },
            nb::arg("x"),
            nb::arg("y"))
        .def(
            "footprint_inside",
            [](const GeometryHolder &inner, const GeometryHolder &outer) {
                return solid_footprint_inside(inner.solid, outer.solid);
            },
            nb::arg("container"))
        .def(
            "fully_inside",
            [](const GeometryHolder &inner, const GeometryHolder &outer) {
                return is_solid_fully_contained(inner.solid, outer.solid);
            },
            nb::arg("container"))
        .def(
            "footprint_inside_batch",
            [](const GeometryHolder &outer, std::vector<GeometryHolder> inners) {
                // Avoid deep-copying all inners into a temporary solid vector.
                nb::list out;
                for (const auto &h : inners) {
                    out.append(solid_footprint_inside(h.solid, outer.solid));
                }
                return out;
            },
            nb::arg("inners"))
        .def(
            "intersects",
            [](const GeometryHolder &a, const GeometryHolder &b) {
                return solids_packing_collide(a.solid, b.solid);
            },
            nb::arg("other"))
        .def(
            "intersects_any",
            [](const GeometryHolder &a, const std::vector<GeometryHolder> &others) {
                for (const auto &o : others) {
                    if (solids_packing_collide(a.solid, o.solid)) {
                        return true;
                    }
                }
                return false;
            },
            nb::arg("others"))
        .def(
            "distance",
            [](const GeometryHolder &a, const GeometryHolder &b) {
                const auto pair = min_distance_pair(a.solid, b.solid);
                if (pair.core.intersect) {
                    return 0.0;
                }
                return std::sqrt(static_cast<double>(pair.core.distance_sq));
            },
            nb::arg("other"))
        .def(
            "min_distance",
            [](const GeometryHolder &a, const GeometryHolder &b) {
                return min_distance_pair(a.solid, b.solid);
            },
            nb::arg("other"))
        .def(
            "standoff_distance",
            [](const GeometryHolder &part, const GeometryHolder &ring) {
                const auto pair = standoff_distance_pair(part.solid, ring.solid);
                if (pair.core.intersect) {
                    return 0.0;
                }
                return std::sqrt(static_cast<double>(pair.core.distance_sq));
            },
            nb::arg("ring"))
        .def(
            "standoff_min_distance",
            [](const GeometryHolder &part, const GeometryHolder &ring) {
                return standoff_distance_pair(part.solid, ring.solid);
            },
            nb::arg("ring"))
        .def(
            "cast_slide",
            [](const GeometryHolder &active,
               std::vector<GeometryHolder> obstacles,
               nb::handle slide,
               double max_t) {
                const Vec2d slide_vec = slide_vector_from_handle(slide);
                return cast_slide(
                    active.solid,
                    solid_ptrs_from_holders(obstacles),
                    slide_vec,
                    static_cast<Vec2d::Scalar>(max_t));
            },
            nb::arg("obstacles"),
            nb::arg("slide"),
            nb::arg("max_t") = std::numeric_limits<double>::infinity())
        .def(
            "get_bounding_circle",
            [](const GeometryHolder &poly) {
                const auto &c = poly.solid.get_bounding_circle();
                auto cen = c.center();
                double r = std::sqrt(static_cast<double>(c.square_radius()));
                return nb::make_tuple(cen[0], cen[1], r);
            })
        .def("__repr__", [](const GeometryHolder &) {
            return "<nest_graph.geometry.Geometry>";
        });

    m.def(
        "polylabel",
        [](const GeometryHolder &g, double precision) {
            const auto rings = rings_from_geometry(g);
            if (rings.empty()) {
                throw nb::value_error("polylabel: Geometry has no usable rings");
            }
            const auto [x, y, r] = polylabel(rings, precision);
            return nb::make_tuple(x, y, r);
        },
        nb::arg("geometry"),
        nb::arg("precision") = 1.0,
        "Mapbox polylabel pole of inaccessibility (x, y, distance).");

    m.def(
        "polylabel_rings",
        [](nb::handle rings_handle, double precision) {
            const auto rings = rings_from_python(rings_handle);
            if (rings.empty()) {
                throw nb::value_error("polylabel_rings: need at least one ring");
            }
            const auto [x, y, r] = polylabel(rings, precision);
            return nb::make_tuple(x, y, r);
        },
        nb::arg("rings"),
        nb::arg("precision") = 1.0,
        "Mapbox polylabel on [outer, *holes] rings → (x, y, distance).");

    m.def("coord_copy_buffer_n", &coord_copy_buffer_n);
    m.def("coord_copy_iter_n", &coord_copy_iter_n);
    m.def("reset_coord_copy_telem", &reset_coord_copy_telem);

    m.def(
        "voronoi_vertices",
        [](const std::vector<std::tuple<double, double>> &sites) {
            // Contract: site_xy = 2*n_sites; out_xy >= 4*n_sites doubles UB.
            std::vector<double> xy;
            xy.reserve(sites.size() * 2);
            for (const auto &t : sites) {
                xy.push_back(std::get<0>(t));
                xy.push_back(std::get<1>(t));
            }
            VoronoiScratch scratch;
            // 4*n_sites doubles = 2*n_sites points (DT triangle UB).
            std::vector<double> out_xy(std::max<std::size_t>(xy.size() * 2, 8), 0.0);
            const std::size_t n_out = voronoi_vertices_xy(
                xy.data(), xy.size(), scratch, out_xy.data(), out_xy.size());
            std::vector<std::tuple<double, double>> out;
            out.reserve(n_out / 2);
            for (std::size_t i = 0; i + 1 < n_out; i += 2) {
                out.emplace_back(out_xy[i], out_xy[i + 1]);
            }
            return out;
        },
        nb::arg("sites"),
        "Finite Voronoi vertices (Delaunay dual circumcenters), lex-sorted.");

    m.def(
        "aabb_tree_query_box",
        [](const std::vector<std::tuple<double, double, double, double>> &boxes,
           double minx,
           double miny,
           double maxx,
           double maxy) {
            const std::vector<double> ltrb = aabb_bind_ltrb_from_tuples(boxes);
            AabbTree tree;
            AabbTreeScratch scratch;
            if (!aabb_tree_rebuild_boxes(ltrb.data(), boxes.size(), tree)) {
                throw nb::value_error("aabb_tree_rebuild_boxes: invalid boxes");
            }
            std::vector<int> out(boxes.size());
            const std::size_t n = aabb_tree_query_box(tree, scratch, minx, miny, maxx, maxy, out.data(), out.size());
            out.resize(n);
            return out;
        },
        nb::arg("boxes"),
        nb::arg("minx"),
        nb::arg("miny"),
        nb::arg("maxx"),
        nb::arg("maxy"),
        "AABB BVH box overlap indices (ltrb tuples).");

    m.def(
        "aabb_tree_query_radius",
        [](const std::vector<std::tuple<double, double, double, double>> &boxes,
           double qx,
           double qy,
           double radius_sq) {
            const std::vector<double> ltrb = aabb_bind_ltrb_from_tuples(boxes);
            AabbTree tree;
            AabbTreeScratch scratch;
            if (!aabb_tree_rebuild_boxes(ltrb.data(), boxes.size(), tree)) {
                throw nb::value_error("aabb_tree_rebuild_boxes: invalid boxes");
            }
            std::vector<int> out(boxes.size());
            const std::size_t n = aabb_tree_query_radius(tree, scratch, qx, qy, radius_sq, out.data(), out.size());
            out.resize(n);
            return out;
        },
        nb::arg("boxes"),
        nb::arg("qx"),
        nb::arg("qy"),
        nb::arg("radius_sq"),
        "AABB BVH radius query (squared; inclusive).");

    m.def(
        "aabb_tree_knn",
        [](const std::vector<std::tuple<double, double, double, double>> &boxes,
           double qx,
           double qy,
           std::size_t k) {
            const std::vector<double> ltrb = aabb_bind_ltrb_from_tuples(boxes);
            AabbTree tree;
            AabbTreeScratch scratch;
            if (!aabb_tree_rebuild_boxes(ltrb.data(), boxes.size(), tree)) {
                throw nb::value_error("aabb_tree_rebuild_boxes: invalid boxes");
            }
            const std::size_t kk = std::min(k, boxes.size());
            std::vector<int> out(kk);
            std::vector<double> dist(kk);
            const std::size_t n =
                aabb_tree_knn(tree, scratch, qx, qy, kk, out.data(), dist.data(), kk);
            out.resize(n);
            dist.resize(n);
            return nb::make_tuple(out, dist);
        },
        nb::arg("boxes"),
        nb::arg("qx"),
        nb::arg("qy"),
        nb::arg("k"),
        "AABB BVH kNN → (indices, dist_sq).");

    m.def(
        "aabb_tree_self_overlap_counts",
        [](const std::vector<std::tuple<double, double, double, double>> &boxes) {
            const std::vector<double> ltrb = aabb_bind_ltrb_from_tuples(boxes);
            AabbTree tree;
            AabbTreeScratch scratch;
            if (!aabb_tree_rebuild_boxes(ltrb.data(), boxes.size(), tree)) {
                throw nb::value_error("aabb_tree_rebuild_boxes: invalid boxes");
            }
            std::vector<int> counts(boxes.size(), 0);
            std::vector<int> hits(boxes.size());
            for (std::size_t i = 0; i < boxes.size(); ++i) {
                const auto &b = boxes[i];
                const std::size_t n = aabb_tree_query_box(tree, scratch,
                    std::get<0>(b),
                    std::get<1>(b),
                    std::get<2>(b),
                    std::get<3>(b),
                    hits.data(),
                    hits.size());
                counts[i] = static_cast<int>(n > 0 ? n - 1 : 0);
            }
            return counts;
        },
        nb::arg("boxes"),
        "Per-box AABB overlap counts excluding self (one rebuild).");
}
