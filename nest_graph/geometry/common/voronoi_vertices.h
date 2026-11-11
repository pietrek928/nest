#pragma once

// Finite Voronoi vertices = Delaunay triangle circumcenters (dual).
// Delaunay via vendored Mapbox-lineage delaunator (see delaunator.hpp).
// Pin: delfrrr/delaunator-cpp include/delaunator.hpp (patched: inline ODR,
// lowest() for bbox, MIT notice). Upstream sha256 before patch:
// c32873a9416d925f348bc559d87ea028238b9dba7db3bac052cb1460ac9fa25d

#include "common/delaunator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

inline bool voronoi_site_near(
    double ax,
    double ay,
    double bx,
    double by,
    double eps
) {
    const double dx = ax - bx;
    const double dy = ay - by;
    return dx * dx + dy * dy <= eps * eps;
}

inline bool voronoi_circumcenter_finite(
    double ax,
    double ay,
    double bx,
    double by,
    double cx,
    double cy,
    double *ox,
    double *oy,
    double area_eps
) {
    const double dx = bx - ax;
    const double dy = by - ay;
    const double ex = cx - ax;
    const double ey = cy - ay;
    const double d = dx * ey - dy * ex;
    if (!(std::fabs(d) > area_eps)) {
        return false;
    }
    const double bl = dx * dx + dy * dy;
    const double cl = ex * ex + ey * ey;
    const double x = ax + (ey * bl - dy * cl) * 0.5 / d;
    const double y = ay + (dx * cl - ex * bl) * 0.5 / d;
    if (!std::isfinite(x) || !std::isfinite(y)) {
        return false;
    }
    *ox = x;
    *oy = y;
    return true;
}

// Caller-owned scratch for voronoi_vertices_xy. Kernel clears/reuses members;
// no other heap allocations inside the kernel.
//
// Buffer contract (typical sizes vs n_sites = n_xy/2):
//   centered: reused; kernel reserves >= n_xy doubles (deduped + mid-shifted sites).
//   verts:    reused; kernel reserves <= ~2*n_sites (DT triangle upper bound).
struct VoronoiScratch {
    std::vector<double> centered;
    std::vector<std::pair<double, double>> verts;
};

// Finite Voronoi vertices for point sites (interleaved x,y). Lex-sorted unique.
//
// Buffer contract:
//   site_xy: n_xy doubles interleaved (x0,y0,...); n_xy even; n_sites = n_xy/2.
//            site_xy may be null only when n_xy == 0. n_xy < 6 => return 0.
//   scratch: caller-owned; kernel mutates centered + verts only (clear/reserve).
//   out_xy:  capacity out_cap_xy doubles;
//            REQUIRE out_cap_xy >= 2 * unique_count after DT+dedupe.
//            Bind/test: reserve at least 4*n_sites doubles (2*n_sites points UB).
//   return:  written doubles (lex-unique interleaved), or 0 if empty/fail/
//            out_cap_xy too small (writes nothing on underflow).
inline std::size_t voronoi_vertices_xy(
    const double *site_xy,
    std::size_t n_xy,
    VoronoiScratch &scratch,
    double *out_xy,
    std::size_t out_cap_xy
) {
    scratch.centered.clear();
    scratch.verts.clear();
    if (n_xy < 6 || (n_xy % 2) != 0 || site_xy == nullptr) {
        return 0;
    }

    bool have_finite = false;
    double min_x = 0.0;
    double max_x = 0.0;
    double min_y = 0.0;
    double max_y = 0.0;
    const std::size_t n_raw = n_xy / 2;
    scratch.verts.reserve(n_raw);
    for (std::size_t i = 0; i < n_raw; ++i) {
        const double x = site_xy[2 * i];
        const double y = site_xy[2 * i + 1];
        if (!std::isfinite(x) || !std::isfinite(y)) {
            continue;
        }
        if (!have_finite) {
            min_x = max_x = x;
            min_y = max_y = y;
            have_finite = true;
        } else {
            min_x = std::min(min_x, x);
            max_x = std::max(max_x, x);
            min_y = std::min(min_y, y);
            max_y = std::max(max_y, y);
        }
        scratch.verts.emplace_back(x, y);
    }
    if (!have_finite || scratch.verts.size() < 3) {
        scratch.verts.clear();
        return 0;
    }
    const double extent = std::max(max_x - min_x, max_y - min_y);
    const double dedupe_eps = std::max(1e-12, 1e-9 * std::max(extent, 1.0));
    const double area_eps = std::max(1e-18, 1e-14 * extent * extent);

    // Sort + x-window dedupe into verts (reuse as site staging).
    std::sort(scratch.verts.begin(), scratch.verts.end());
    {
        std::size_t uniq_n = 0;
        for (std::size_t i = 0; i < scratch.verts.size(); ++i) {
            const auto p = scratch.verts[i];
            bool dup = false;
            for (std::size_t k = uniq_n; k > 0; --k) {
                const auto &q = scratch.verts[k - 1];
                if (p.first - q.first > dedupe_eps) {
                    break;
                }
                if (voronoi_site_near(p.first, p.second, q.first, q.second, dedupe_eps)) {
                    dup = true;
                    break;
                }
            }
            if (!dup) {
                scratch.verts[uniq_n++] = p;
            }
        }
        scratch.verts.resize(uniq_n);
    }
    if (scratch.verts.size() < 3) {
        scratch.verts.clear();
        return 0;
    }

    const double mid_x = 0.5 * (min_x + max_x);
    const double mid_y = 0.5 * (min_y + max_y);
    scratch.centered.clear();
    scratch.centered.reserve(scratch.verts.size() * 2);
    for (const auto &p : scratch.verts) {
        scratch.centered.push_back(p.first - mid_x);
        scratch.centered.push_back(p.second - mid_y);
    }

    try {
        delaunator::Delaunator d(scratch.centered);
        scratch.verts.clear();
        scratch.verts.reserve(d.triangles.size() / 3);
        for (std::size_t i = 0; i + 2 < d.triangles.size(); i += 3) {
            const std::size_t i0 = d.triangles[i];
            const std::size_t i1 = d.triangles[i + 1];
            const std::size_t i2 = d.triangles[i + 2];
            double cx = 0.0;
            double cy = 0.0;
            if (!voronoi_circumcenter_finite(
                    scratch.centered[2 * i0],
                    scratch.centered[2 * i0 + 1],
                    scratch.centered[2 * i1],
                    scratch.centered[2 * i1 + 1],
                    scratch.centered[2 * i2],
                    scratch.centered[2 * i2 + 1],
                    &cx,
                    &cy,
                    area_eps)) {
                continue;
            }
            scratch.verts.emplace_back(cx + mid_x, cy + mid_y);
        }
        if (scratch.verts.empty()) {
            return 0;
        }
        std::sort(scratch.verts.begin(), scratch.verts.end());
        std::size_t uniq_n = 0;
        for (std::size_t i = 0; i < scratch.verts.size(); ++i) {
            const auto p = scratch.verts[i];
            bool dup = false;
            for (std::size_t k = uniq_n; k > 0; --k) {
                const auto &q = scratch.verts[k - 1];
                if (p.first - q.first > dedupe_eps) {
                    break;
                }
                if (voronoi_site_near(
                        p.first, p.second, q.first, q.second, dedupe_eps)) {
                    dup = true;
                    break;
                }
            }
            if (!dup) {
                scratch.verts[uniq_n++] = p;
            }
        }
        scratch.verts.resize(uniq_n);
        const std::size_t need_xy = scratch.verts.size() * 2;
        if (need_xy == 0) {
            return 0;
        }
        if (out_xy == nullptr || out_cap_xy < need_xy) {
            return 0;
        }
        for (std::size_t i = 0; i < scratch.verts.size(); ++i) {
            out_xy[2 * i] = scratch.verts[i].first;
            out_xy[2 * i + 1] = scratch.verts[i].second;
        }
        return need_xy;
    } catch (const std::runtime_error &) {
        return 0;
    }
}

// Catch2 / convenience wrapper: owns local scratch + out vector at call site.
inline std::vector<std::pair<double, double>> voronoi_vertices(
    const std::vector<std::pair<double, double>> &sites
) {
    std::vector<double> xy;
    xy.reserve(sites.size() * 2);
    for (const auto &p : sites) {
        xy.push_back(p.first);
        xy.push_back(p.second);
    }
    VoronoiScratch scratch;
    // 4*n_sites doubles = 2*n_sites points upper bound for unique circumcenters.
    std::vector<double> out_xy(std::max<std::size_t>(xy.size() * 2, 8), 0.0);
    const std::size_t n_out = voronoi_vertices_xy(
        xy.data(), xy.size(), scratch, out_xy.data(), out_xy.size());
    std::vector<std::pair<double, double>> out;
    out.reserve(n_out / 2);
    for (std::size_t i = 0; i + 1 < n_out; i += 2) {
        out.emplace_back(out_xy[i], out_xy[i + 1]);
    }
    return out;
}
