#include <cmath>
#include <string>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "geometry/guide/guide.h"
#include "tests/geometry_test_helpers.h"

using Vec2 = PolyTestVec2;
using SolidGeometry2 = PolyTestSolidGeometry2;

bool has_move_type(const std::vector<PlacementProposition<Vec2>>& props, const std::string& tag) {
    for (const auto& p : props) {
        if (p.move_type.find(tag) != std::string::npos) {
            return true;
        }
    }
    return false;
}

int count_move_type(const std::vector<PlacementProposition<Vec2>>& props, const std::string& tag) {
    int n = 0;
    for (const auto& p : props) {
        if (p.move_type.find(tag) != std::string::npos) {
            ++n;
        }
    }
    return n;
}

TEST_CASE("guide forward gravity snap toward obstacle", "[guide]") {
    std::vector<SolidGeometry2> polys{
        make_box(0.0, 0.0, 0.5, 0.5),
        make_box(5.0, 0.0, 0.5, 0.5),
    };

    GuidanceConfig<Vec2> cfg;
    cfg.use_gravity = true;
    cfg.gravity_vector = Vec2{{1.0, 0.0}};
    cfg.use_target_attractor = false;
    cfg.search_radius = 10.0;
    cfg.minimum_placing_distance = 1e-6;

    auto guidance = evaluate_local_placement<Vec2>(0, polys, Vec2{{0.0, 0.0}}, cfg);

    REQUIRE_FALSE(guidance.is_penetrating);
    REQUIRE(has_move_type(guidance.propositions, "Exact Gravity Dock"));

    bool found_forward = false;
    for (const auto& p : guidance.propositions) {
        if (p.move_type.find("Exact Gravity Dock") != std::string::npos) {
            REQUIRE(p.translation[0] > 0.0);
            REQUIRE(p.translation[0] == Catch::Approx(4.0).margin(0.05));
            found_forward = true;
        }
    }
    REQUIRE(found_forward);
}

TEST_CASE("guide behind-ray cast is a miss", "[guide]") {
    std::vector<SolidGeometry2> polys{
        make_box(0.0, 0.0, 0.5, 0.5),
        make_box(-5.0, 0.0, 0.5, 0.5),
    };

    auto cast_res = find_closest_polygon_cast<Vec2>(
        0, polys, Vec2{{1.0, 0.0}}, 10.0
    );

    REQUIRE_FALSE(cast_res.intersects_path);
}

TEST_CASE("guide overlap triggers primary ejection", "[guide]") {
    std::vector<SolidGeometry2> polys{
        make_box(0.0, 0.0, 0.5, 0.5),
        make_box(0.2, 0.0, 0.5, 0.5),
    };

    GuidanceConfig<Vec2> cfg;
    cfg.use_target_attractor = false;
    cfg.search_radius = 10.0;

    auto guidance = evaluate_local_placement<Vec2>(0, polys, Vec2{{0.0, 0.0}}, cfg);

    REQUIRE(guidance.is_penetrating);
    REQUIRE(has_move_type(guidance.propositions, "Primary Ejection"));
}

TEST_CASE("guide non-empty menu without attractor", "[guide]") {
    std::vector<SolidGeometry2> polys{
        make_box(0.0, 0.0, 0.5, 0.5),
        make_box(5.0, 0.0, 0.5, 0.5),
    };

    GuidanceConfig<Vec2> cfg;
    cfg.use_gravity = true;
    cfg.gravity_vector = Vec2{{1.0, 0.0}};
    cfg.use_target_attractor = false;
    cfg.search_radius = 10.0;

    auto guidance = evaluate_local_placement<Vec2>(0, polys, Vec2{{0.0, 0.0}}, cfg);

    REQUIRE(guidance.propositions.size() >= 1);
}

TEST_CASE("guide secondary neighbor snap between two obstacles", "[guide]") {
    std::vector<SolidGeometry2> polys{
        make_box(5.0, 5.0, 0.5, 0.5),
        make_box(2.0, 5.0, 0.5, 0.5),
        make_box(8.0, 5.0, 0.5, 0.5),
    };

    GuidanceConfig<Vec2> cfg;
    cfg.use_gravity = false;
    cfg.use_target_attractor = false;
    cfg.use_tight_packing = true;
    cfg.use_corner_alignment = false;
    cfg.enable_grid_exploration = false;
    cfg.max_alternative_angles = 0;
    cfg.search_radius = 10.0;
    cfg.diversity_distance_threshold = 0.01;
    cfg.max_propositions = 16;

    auto guidance = evaluate_local_placement<Vec2>(0, polys, Vec2{{5.0, 5.0}}, cfg);

    REQUIRE(count_move_type(guidance.propositions, "Exact Neighbor Snap") >= 2);
}
