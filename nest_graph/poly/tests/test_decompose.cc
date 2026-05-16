#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "poly/decompose.h"
#include "poly/poly.h"
#include <vec.h>

using Vec2f = Vec<2, float>;
using Polygon2f = Polygon<Vec2f>;

namespace {

// If ear-clipping appears to hang, re-run with NEST_DECOMPOSE_DEBUG=1 to print vertices to stderr.
void log_poly_xy_stderr(const char* tag, const std::vector<Vec2f>& poly) {
    if (!std::getenv("NEST_DECOMPOSE_DEBUG")) {
        return;
    }
    std::fprintf(stderr, "%s (%zu verts):\n", tag, poly.size());
    for (size_t i = 0; i < poly.size(); ++i) {
        std::fprintf(stderr, "  %zu: %g %g\n", i, static_cast<double>(poly[i][0]),
                     static_cast<double>(poly[i][1]));
    }
}

void log_holes_xy_stderr(const char* tag, const std::vector<std::vector<Vec2f>>& holes) {
    if (!std::getenv("NEST_DECOMPOSE_DEBUG")) {
        return;
    }
    std::fprintf(stderr, "%s (%zu holes)\n", tag, holes.size());
    for (size_t h = 0; h < holes.size(); ++h) {
        log_poly_xy_stderr(("  hole " + std::to_string(h)).c_str(), holes[h]);
    }
}

template <class VecType>
bool is_strictly_convex(const std::vector<VecType>& poly) {
    if (poly.size() < 3) {
        return false;
    }
    bool sign_set = false;
    bool sign = false;
    for (size_t i = 0; i < poly.size(); ++i) {
        VecType p0 = poly[(i > 0) ? i - 1 : poly.size() - 1];
        VecType p1 = poly[i];
        VecType p2 = poly[(i + 1) % poly.size()];

        auto cp = cross_2d(p0, p1, p2);
        if (!sign_set) {
            sign = cp > 0;
            sign_set = true;
        } else if ((cp > 0) != sign && cp != typename VecType::Scalar{0}) {
            return false;
        }
    }
    return true;
}

void require_loads_into_polygon(const std::vector<std::vector<Vec2f>>& convex_parts) {
    Polygon2f poly;
    for (const auto& part : convex_parts) {
        REQUIRE(part.size() >= 3u);
        poly.append_convex_poly(part.data(), static_cast<int>(part.size()));
    }
    poly.finalize();
    REQUIRE(poly.convex_parts.size() == convex_parts.size());
    size_t total = 0;
    for (size_t i = 0; i < poly.convex_parts.size(); ++i) {
        total += static_cast<size_t>(poly.get_part_size(i));
    }
    REQUIRE(total == poly.poly_points.size());
}

} // namespace

TEST_CASE("Geometry: Hertel-Mehlhorn L-Shape Optimization", "[decomposition]") {
    std::vector<Vec2f> l_shape = {
        {{0.f, 0.f}}, {{6.f, 0.f}}, {{6.f, 2.f}}, {{2.f, 2.f}}, {{2.f, 6.f}}, {{0.f, 6.f}}};

    log_poly_xy_stderr("L-shape", l_shape);
    auto convex_parts = decompose_to_convex_optimal(l_shape);

    REQUIRE(convex_parts.size() == 2);
    for (const auto& part : convex_parts) {
        REQUIRE(is_strictly_convex(part));
        REQUIRE(part.size() >= 3u);
    }
}

TEST_CASE("Geometry: The Bridge-Builder Donut Test", "[decomposition][holes]") {
    std::vector<Vec2f> outer = {
        {{-5.f, -5.f}}, {{5.f, -5.f}}, {{5.f, 5.f}}, {{-5.f, 5.f}}};

    std::vector<Vec2f> hole = {
        {{-2.f, -2.f}}, {{2.f, -2.f}}, {{2.f, 2.f}}, {{-2.f, 2.f}}};

    std::vector<std::vector<Vec2f>> holes = {hole};

    log_poly_xy_stderr("Donut outer", outer);
    log_holes_xy_stderr("Donut holes", holes);
    auto convex_parts = decompose_complex_polygon(outer, holes);

    REQUIRE(convex_parts.size() == 4);
    for (const auto& part : convex_parts) {
        REQUIRE(is_strictly_convex(part));
    }
}

TEST_CASE("Geometry: Winding Order Auto-Correction", "[decomposition]") {
    std::vector<Vec2f> cw_triangle = {
        {{0.f, 10.f}}, {{10.f, 0.f}}, {{0.f, 0.f}}};

    log_poly_xy_stderr("CW triangle", cw_triangle);
    auto convex_parts = decompose_to_convex_optimal(cw_triangle);

    REQUIRE(convex_parts.size() == 1);
    REQUIRE(is_strictly_convex(convex_parts[0]));
}

TEST_CASE("Geometry: Decomposition loads into Polygon", "[decomposition]") {
    std::vector<Vec2f> l_shape = {
        {{0.f, 0.f}}, {{6.f, 0.f}}, {{6.f, 2.f}}, {{2.f, 2.f}}, {{2.f, 6.f}}, {{0.f, 6.f}}};
    log_poly_xy_stderr("L-shape (polygon load)", l_shape);
    require_loads_into_polygon(decompose_to_convex_optimal(l_shape));

    std::vector<Vec2f> outer = {
        {{-5.f, -5.f}}, {{5.f, -5.f}}, {{5.f, 5.f}}, {{-5.f, 5.f}}};
    std::vector<Vec2f> hole = {
        {{-2.f, -2.f}}, {{2.f, -2.f}}, {{2.f, 2.f}}, {{-2.f, 2.f}}};
    log_poly_xy_stderr("Donut outer (polygon load)", outer);
    log_holes_xy_stderr("Donut holes (polygon load)", std::vector<std::vector<Vec2f>>{hole});
    require_loads_into_polygon(decompose_complex_polygon(outer, {hole}));
}
