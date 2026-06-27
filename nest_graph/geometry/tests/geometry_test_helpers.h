#pragma once

#include <cmath>
#include <cstdlib>
#include <initializer_list>
#include <random>
#include <vector>

#include "geometry/solid/decompose.h"
#include "geometry/solid/solid_geometry.h"
#include "geometry/common/tracer.h"
#include <vec.h>

using PolyTestVec2 = Vec<2, double>;
using PolyTestSolidGeometry2 = SolidGeometry<PolyTestVec2>;

inline bool polytest_near(double a, double b, double tol = 1e-9) {
    return std::abs(a - b) < tol;
}

// Single closed ring: one line part for GJK plus a boundary ring for point-in-solid.
inline PolyTestSolidGeometry2 polygon_from_quad(
    std::initializer_list<std::initializer_list<double>> pts4
) {
    PolyTestSolidGeometry2 poly;
    std::vector<PolyTestVec2> ring;
    ring.reserve(5);
    for (const auto& p : pts4) {
        auto it = p.begin();
        ring.emplace_back(std::initializer_list<double>{*it, *++it});
    }
    if (ring.size() >= 3) {
        poly.add_boundary_ring(ring);
    }
    if (!ring.empty()) {
        ring.push_back(ring.front());
    }
    std::mt19937 gen(42);
    poly.append_line_poly(ring.data(), static_cast<int>(ring.size()), gen);
    poly.finalize(gen);
    return poly;
}

// Concave or arbitrary closed ring: boundary ring + convex decomposition.
inline PolyTestSolidGeometry2 polygon_from_ring(
    std::initializer_list<std::initializer_list<double>> pts
) {
    PolyTestSolidGeometry2 poly;
    std::vector<PolyTestVec2> ring;
    ring.reserve(pts.size() + 1);
    for (const auto& p : pts) {
        auto it = p.begin();
        ring.emplace_back(std::initializer_list<double>{*it, *++it});
    }
    if (ring.size() < 3) {
        return poly;
    }
    if (ring.front()[0] != ring.back()[0] || ring.front()[1] != ring.back()[1]) {
        ring.push_back(ring.front());
    }
    std::vector<PolyTestVec2> boundary = ring;
    if (!boundary.empty()) {
        boundary.pop_back();
    }
    poly.add_boundary_ring(boundary);
    std::mt19937 gen(42);
    process_boundary_to_convex_segments<PolyTestVec2>(ring, poly, gen, false);
    poly.finalize(gen);
    return poly;
}

inline PolyTestSolidGeometry2 make_box(double cx, double cy, double half_w, double half_h) {
    return polygon_from_quad({
        {cx - half_w, cy - half_h},
        {cx + half_w, cy - half_h},
        {cx + half_w, cy + half_h},
        {cx - half_w, cy + half_h},
    });
}

inline PolyTestSolidGeometry2 polygon_c_shape_three_parts() {
    PolyTestSolidGeometry2 poly;
    std::vector<PolyTestVec2> bottom{
        PolyTestVec2{{0.0, 0.0}},
        PolyTestVec2{{10.0, 0.0}},
        PolyTestVec2{{10.0, 2.0}},
        PolyTestVec2{{0.0, 2.0}},
        PolyTestVec2{{0.0, 0.0}},
    };
    std::vector<PolyTestVec2> back{
        PolyTestVec2{{0.0, 2.0}},
        PolyTestVec2{{2.0, 2.0}},
        PolyTestVec2{{2.0, 8.0}},
        PolyTestVec2{{0.0, 8.0}},
        PolyTestVec2{{0.0, 2.0}},
    };
    std::vector<PolyTestVec2> top{
        PolyTestVec2{{0.0, 8.0}},
        PolyTestVec2{{10.0, 8.0}},
        PolyTestVec2{{10.0, 10.0}},
        PolyTestVec2{{0.0, 10.0}},
        PolyTestVec2{{0.0, 8.0}},
    };
    poly.add_boundary_ring(std::vector<PolyTestVec2>(bottom.begin(), bottom.end() - 1));
    poly.add_boundary_ring(std::vector<PolyTestVec2>(back.begin(), back.end() - 1));
    poly.add_boundary_ring(std::vector<PolyTestVec2>(top.begin(), top.end() - 1));
    std::mt19937 gen(42);
    poly.append_line_poly(bottom.data(), static_cast<int>(bottom.size()), gen);
    poly.append_line_poly(back.data(), static_cast<int>(back.size()), gen);
    poly.append_line_poly(top.data(), static_cast<int>(top.size()), gen);
    poly.finalize(gen);
    return poly;
}

inline PolyTestSolidGeometry2 make_donut(
    const std::vector<PolyTestVec2>& outer,
    const std::vector<PolyTestVec2>& hole
) {
    return decompose_complex_polygon<PolyTestVec2>({outer}, {hole});
}

inline PolyTestSolidGeometry2 polygon_outer_with_square_hole(
    double ox0, double oy0, double ox1, double oy1,
    double hx0, double hy0, double hx1, double hy1
) {
    std::vector<PolyTestVec2> outer{
        PolyTestVec2{{ox0, oy0}},
        PolyTestVec2{{ox1, oy0}},
        PolyTestVec2{{ox1, oy1}},
        PolyTestVec2{{ox0, oy1}},
    };
    std::vector<PolyTestVec2> hole{
        PolyTestVec2{{hx0, hy0}},
        PolyTestVec2{{hx1, hy0}},
        PolyTestVec2{{hx1, hy1}},
        PolyTestVec2{{hx0, hy1}},
    };
    return make_donut(outer, hole);
}

inline PolyTestSolidGeometry2 polygon_donut_with_square_hole() {
    std::vector<PolyTestVec2> outer{
        PolyTestVec2{{20.0, 0.0}},
        PolyTestVec2{{30.0, 0.0}},
        PolyTestVec2{{30.0, 10.0}},
        PolyTestVec2{{20.0, 10.0}},
    };
    std::vector<PolyTestVec2> hole{
        PolyTestVec2{{22.0, 2.0}},
        PolyTestVec2{{28.0, 2.0}},
        PolyTestVec2{{28.0, 8.0}},
        PolyTestVec2{{22.0, 8.0}},
    };
    return make_donut(outer, hole);
}

inline void maybe_print_physics_telemetry(const DebugTracer& tracer) {
    if (std::getenv("NEST_PRINT_TELEMETRY")) {
        tracer.print_telemetry();
    }
}
