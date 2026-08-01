#include <cmath>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "geometry/distance/polygon_distance.h"
#include "geometry/distance/static_collision_scene.h"
#include "geometry/guide/polygon_cast.h"
#include "tests/geometry_test_helpers.h"

using Vec2 = PolyTestVec2;
using SolidGeometry2 = PolyTestSolidGeometry2;

TEST_CASE("distance sweep: swallowed containment mid-list", "[distance][containment][gap]") {
    // Three pairs: A/B far, C fully inside D, E/F touching gap.
    // Intermediate swallowed pair must still report intersect.
    auto outer = polygon_from_quad({{0, 0}, {10, 0}, {10, 10}, {0, 10}});
    auto inner = polygon_from_quad({{3, 3}, {5, 3}, {5, 5}, {3, 5}});
    auto left = polygon_from_quad({{-20, 0}, {-18, 0}, {-18, 2}, {-20, 2}});
    auto right = polygon_from_quad({{20, 0}, {22, 0}, {22, 2}, {20, 2}});

    std::vector<SolidGeometry2> polys = {left, right, outer, inner};
    auto results = find_polygon_distances<Vec2>(polys, 0.5, 0.0);

    const ComplexDistanceResult<Vec2>* swallowed = nullptr;
    for (const auto& r : results) {
        if ((r.polyA_idx == 2 && r.polyB_idx == 3) || (r.polyA_idx == 3 && r.polyB_idx == 2)) {
            swallowed = &r;
            break;
        }
    }
    REQUIRE(swallowed != nullptr);
    REQUIRE(swallowed->intersect);
}

TEST_CASE("StaticCollisionScene: margin prune with default aura", "[distance][validity][gap]") {
    auto obstacle = polygon_from_quad({{0, 0}, {2, 0}, {2, 2}, {0, 2}});
    auto part = polygon_from_quad({{0, 0}, {1, 0}, {1, 1}, {0, 1}});

    StaticCollisionScene<Vec2> scene;
    scene.build({obstacle}, 0.5);

    // Centers ~3 apart, clearance margin 0.5 -> valid (gap ~2).
    auto far = part.translate(Vec2{{3.5, 0.5}});
    REQUIRE(scene.is_valid_placement(far, 0.5 * 0.5));

    // Overlapping placement is invalid.
    auto overlap = part.translate(Vec2{{0.5, 0.5}});
    REQUIRE_FALSE(scene.is_valid_placement(overlap, 0.5 * 0.5));

    // Under clearance: part box [2.2,3.2]x[0,1] vs obstacle [0,2]x[0,2] -> gap 0.2 < 0.5.
    auto tight = part.translate(Vec2{{2.2, 0.5}});
    REQUIRE_FALSE(scene.is_valid_placement(tight, 0.5 * 0.5));
}

TEST_CASE("cast skips subtractive hole parts but still docks to shell", "[cast][holes][gap]") {
    auto shell = polygon_outer_with_square_hole(0, 0, 10, 10, 4, 4, 6, 6);
    auto probe = polygon_from_quad({{4.5, 7.5}, {5.5, 7.5}, {5.5, 8.5}, {4.5, 8.5}});

    // Ensure shell has at least one subtractive part from hole decomposition.
    bool has_subtractive = false;
    for (const auto& part : shell.line_parts) {
        if (part.is_subtractive) {
            has_subtractive = true;
            break;
        }
    }
    REQUIRE(has_subtractive);

    std::vector<SolidGeometry2> polys = {probe, shell};
    // Cast downward toward the hole / shell material.
    auto hit = find_closest_polygon_cast<Vec2>(0, polys, Vec2{{0.0, -1.0}}, 10.0);
    REQUIRE(hit.intersects_path);
    REQUIRE(hit.t_entry >= 0.0);
    REQUIRE(hit.t_entry < 10.0);
    // Must hit an additive shell part, not a subtractive hole ring.
    REQUIRE_FALSE(shell.line_parts[static_cast<size_t>(hit.partB_idx)].is_subtractive);
}
