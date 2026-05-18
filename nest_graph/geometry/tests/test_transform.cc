#include <catch2/catch_test_macros.hpp>
#include <cmath>

#include "geometry/tests/geometry_test_helpers.h"

using PolyTestVec2 = Vec<2, double>;
using SolidGeometry2 = PolyTestSolidGeometry2;

namespace {

constexpr double kTol = 1e-9;

bool near(double a, double b) {
    return std::abs(a - b) < kTol;
}

PolyTestVec2 find_point(const SolidGeometry2& g, double x, double y) {
    for (const auto& p : g.line_points) {
        if (near(p[0], x) && near(p[1], y)) {
            return p;
        }
    }
    return PolyTestVec2({-1e9, -1e9});
}

}  // namespace

TEST_CASE("SolidGeometry: translate moves line points", "[transform]") {
    SolidGeometry2 square = polygon_from_quad({
        {0, 0},
        {1, 0},
        {1, 1},
        {0, 1},
    });

    SolidGeometry2 moved = square.translate(PolyTestVec2({1.0, 2.0}));

    REQUIRE(find_point(moved, 1.0, 2.0)[0] == 1.0);
    REQUIRE(find_point(moved, 2.0, 2.0)[0] == 2.0);
    REQUIRE(find_point(moved, 2.0, 3.0)[0] == 2.0);
    REQUIRE(find_point(moved, 1.0, 3.0)[0] == 1.0);
}

TEST_CASE("SolidGeometry: rotate 90 degrees about origin", "[transform]") {
    SolidGeometry2 square = polygon_from_quad({
        {0, 0},
        {1, 0},
        {1, 1},
        {0, 1},
    });

    const double half_pi = 0.5 * M_PI;
    SolidGeometry2 rotated = square.rotate(half_pi);

    REQUIRE(near(find_point(rotated, 0.0, 1.0)[0], 0.0));
    REQUIRE(near(find_point(rotated, 0.0, 1.0)[1], 1.0));
    REQUIRE(near(find_point(rotated, 0.0, 0.0)[0], 0.0));
    REQUIRE(near(find_point(rotated, 0.0, 0.0)[1], 0.0));
}

TEST_CASE("SolidGeometry: rotate about arbitrary origin", "[transform]") {
    SolidGeometry2 square = polygon_from_quad({
        {0, 0},
        {2, 0},
        {2, 2},
        {0, 2},
    });

    PolyTestVec2 origin({1.0, 1.0});
    const double angle = M_PI;
    SolidGeometry2 rotated = square.rotate(angle, origin);

    REQUIRE(near(find_point(rotated, 2.0, 2.0)[0], 2.0));
    REQUIRE(near(find_point(rotated, 2.0, 2.0)[1], 2.0));
    REQUIRE(near(find_point(rotated, 0.0, 0.0)[0], 0.0));
    REQUIRE(near(find_point(rotated, 0.0, 0.0)[1], 0.0));
}

TEST_CASE("SolidGeometry: translate preserves boundary ring metadata", "[transform]") {
    SolidGeometry2 poly = polygon_from_quad({
        {0, 0},
        {4, 0},
        {4, 4},
        {0, 4},
    });

    std::vector<PolyTestVec2> hole = {
        {1, 1},
        {3, 1},
        {3, 3},
        {1, 3},
    };
    poly.add_boundary_ring(hole, true);
    const std::size_t ring_count = poly.boundary_rings.size();
    REQUIRE(ring_count >= 2);

    SolidGeometry2 moved = poly.translate(PolyTestVec2({10.0, -5.0}));

    REQUIRE(moved.boundary_rings.size() == ring_count);
    bool found_subtractive = false;
    for (const auto& ring : moved.boundary_rings) {
        if (ring.is_subtractive && ring.points.size() == 4) {
            found_subtractive = true;
            REQUIRE(near(ring.points[0][0], 11.0));
            REQUIRE(near(ring.points[0][1], -4.0));
        }
    }
    REQUIRE(found_subtractive);
}
