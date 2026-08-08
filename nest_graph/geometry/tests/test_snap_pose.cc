#include <cmath>
#include <optional>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "geometry/common/snap_pose.h"
#include "geometry/solid/containment.h"
#include "tests/geometry_test_helpers.h"

using Vec2 = PolyTestVec2;
using SolidGeometry2 = PolyTestSolidGeometry2;

TEST_CASE("snap_pose_to_ring docks rect to board edge", "[snap]") {
    auto board = polygon_from_quad({{0, 0}, {10, 0}, {10, 10}, {0, 10}});
    auto part = polygon_from_quad({{0, 0}, {1, 0}, {1, 0.5}, {0, 0.5}});
    // Open ring along bottom edge of board (outline-style).
    SolidGeometry2 ring;
    {
        std::vector<Vec2> pts = {
            Vec2{{0, 0}}, Vec2{{10, 0}}, Vec2{{10, 10}}, Vec2{{0, 10}}, Vec2{{0, 0}},
        };
        // from_ring style: line only
        ring = polygon_from_quad({{0, 0}, {10, 0}, {10, 10}, {0, 10}});
        (void)pts;
    }

    const double min_dist = 0.05;
    const Vec2 contact{{5.0, 0.0}};
    const Vec2 inward{{0.0, 1.0}};
    auto snapped = snap_pose_to_ring(
        part, ring, contact, inward, 0.0, min_dist, &board, std::nullopt);
    REQUIRE(snapped.has_value());
    const auto [dx, dy, ang] = *snapped;
    REQUIRE(ang == Catch::Approx(0.0));
    // Translation should place part above the bottom edge.
    REQUIRE(dy > 0.0);
    auto placed = part.rotate(ang).translate(Vec2{{dx, dy}});
    auto gap_pair = standoff_distance_pair(placed, ring);
    double gap = gap_pair.core.intersect
        ? 0.0
        : std::sqrt(static_cast<double>(gap_pair.core.distance_sq));
    REQUIRE(gap + 1e-5 >= min_dist);
    REQUIRE(solid_footprint_inside(placed, board));
}

TEST_CASE("snap_pose_to_ring rejects zero inward", "[snap]") {
    auto board = polygon_from_quad({{0, 0}, {4, 0}, {4, 4}, {0, 4}});
    auto part = polygon_from_quad({{0, 0}, {1, 0}, {1, 1}, {0, 1}});
    auto ring = board;
    auto snapped = snap_pose_to_ring(
        part, ring, Vec2{{2, 0}}, Vec2{{0, 0}}, 0.0, 0.1, &board, std::nullopt);
    REQUIRE_FALSE(snapped.has_value());
}
