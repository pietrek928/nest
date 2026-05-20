#include <catch2/catch_test_macros.hpp>
#include <cmath>

#include "geometry/point_in_solid.h"
#include "geometry/polygon_intersect.h"
#include "geometry/tests/geometry_test_helpers.h"

using PolyTestVec2 = Vec<2, double>;
using SolidGeometry2 = PolyTestSolidGeometry2;

namespace {

constexpr double kTol = 1e-6;

bool near(double a, double b) {
    return std::abs(a - b) < kTol;
}

PolyTestVec2 find_corner(const SolidGeometry2& g, double x, double y) {
    for (const auto& p : g.line_points) {
        if (near(p[0], x) && near(p[1], y)) {
            return p;
        }
    }
    return PolyTestVec2({-1e9, -1e9});
}

}  // namespace

TEST_CASE("Geometry API: from_convex_polygon", "[geometry_api]") {
    SolidGeometry2 g = polygon_from_quad({
        {0, 0},
        {1, 0},
        {1, 1},
        {0, 1},
    });
    REQUIRE_FALSE(g.line_points.empty());
    const auto& c = g.get_bounding_circle();
    REQUIRE(c.square_radius() > 0.0);
}

TEST_CASE("Geometry API: apply_transform translate", "[geometry_api]") {
    SolidGeometry2 g = polygon_from_quad({
        {0, 0},
        {1, 0},
        {1, 1},
        {0, 1},
    });
    SolidGeometry2 moved = g.translate(PolyTestVec2({2.0, 3.0}));
    REQUIRE(near(find_corner(moved, 2.0, 3.0)[0], 2.0));
    REQUIRE(near(find_corner(moved, 3.0, 4.0)[0], 3.0));
}

TEST_CASE("Geometry API: apply_transform rotate 90 degrees", "[geometry_api]") {
    SolidGeometry2 g = polygon_from_quad({
        {0, 0},
        {1, 0},
        {1, 1},
        {0, 1},
    });
    const double half_pi = 0.5 * M_PI;
    SolidGeometry2 rotated = g.rotate(half_pi);
    REQUIRE(near(find_corner(rotated, 0.0, 1.0)[1], 1.0));
}

TEST_CASE("Geometry API: rotate then translate matches placement order", "[geometry_api]") {
    SolidGeometry2 g = polygon_from_quad({
        {0, 0},
        {1, 0},
        {1, 1},
        {0, 1},
    });
    SolidGeometry2 chained = g.rotate(0.25).translate(PolyTestVec2({1.0, 2.0}));
    SolidGeometry2 placed = g.rotate(0.25);
    placed = placed.translate(PolyTestVec2({1.0, 2.0}));
    REQUIRE(near(
        find_corner(chained, find_corner(placed, 0, 0)[0], find_corner(placed, 0, 0)[1])[0],
        find_corner(placed, 0, 0)[0]));
}

TEST_CASE("Geometry API: bounding circle bounds consistency", "[geometry_api]") {
    SolidGeometry2 g = polygon_from_quad({
        {0, 0},
        {2, 0},
        {2, 2},
        {0, 2},
    });
    const auto& c = g.get_bounding_circle();
    const double cx = c.center()[0];
    const double cy = c.center()[1];
    const double r = std::sqrt(c.square_radius());
    REQUIRE(near(cx, 1.0));
    REQUIRE(near(cy, 1.0));
    REQUIRE(near(r, std::sqrt(2.0)));
}

TEST_CASE("Geometry API: contains_point triangle", "[geometry_api]") {
    SolidGeometry2 tri = polygon_from_quad({
        {0, 0},
        {2, 0},
        {1, 1},
        {0, 0},
    });
    REQUIRE(is_point_inside_solid_space(PolyTestVec2({1.0, 0.2}), tri));
    REQUIRE_FALSE(is_point_inside_solid_space(PolyTestVec2({5.0, 5.0}), tri));
}

TEST_CASE("Geometry API: contains_point donut hole", "[geometry_api][holes]") {
    SolidGeometry2 donut = make_donut(
        {{0, 0}, {6, 0}, {6, 6}, {0, 6}},
        {{2, 2}, {4, 2}, {4, 4}, {2, 4}}
    );
    REQUIRE(is_point_inside_solid_space(PolyTestVec2({1.0, 1.0}), donut));
    REQUIRE_FALSE(is_point_inside_solid_space(PolyTestVec2({3.0, 3.0}), donut));
}

TEST_CASE("Geometry API: intersects disjoint and overlap", "[geometry_api]") {
    SolidGeometry2 left = polygon_from_quad({{0, 0}, {2, 0}, {2, 2}, {0, 2}});
    SolidGeometry2 right_sep = polygon_from_quad({{4, 0}, {6, 0}, {6, 2}, {4, 2}});
    SolidGeometry2 overlap = polygon_from_quad({{1, 0}, {3, 0}, {3, 2}, {1, 2}});

    REQUIRE(find_polygon_intersections<PolyTestVec2>({left, right_sep}).empty());
    REQUIRE_FALSE(find_polygon_intersections<PolyTestVec2>({left, overlap}).empty());
}
