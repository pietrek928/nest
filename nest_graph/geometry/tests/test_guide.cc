#include <cmath>
#include <random>
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

TEST_CASE("guide post-rotation cast yields alt-angle propositions", "[guide]") {
    std::vector<SolidGeometry2> polys{
        make_box(1.5, 1.5, 0.35, 0.35),
        make_box(0.0, 0.0, 0.08, 2.0),
        make_box(0.0, 0.0, 2.0, 0.08),
    };

    GuidanceConfig<Vec2> cfg;
    cfg.use_gravity = false;
    cfg.use_target_attractor = false;
    cfg.use_tight_packing = true;
    cfg.use_corner_alignment = true;
    cfg.max_alternative_angles = 3;
    cfg.search_radius = 10.0;
    cfg.diversity_angle_rad_threshold = 0.01;

    auto guidance = evaluate_local_placement<Vec2>(0, polys, Vec2{{1.5, 1.5}}, cfg);

    bool has_alt_angle = false;
    for (const auto& p : guidance.propositions) {
        if (std::fabs(p.rotation_rad) > 1e-4) {
            has_alt_angle = true;
            break;
        }
    }
    REQUIRE(has_alt_angle);
}

PolyTestSolidGeometry2 make_star_polygon(double cx, double cy, double outer_r, int points = 8) {
    std::vector<PolyTestVec2> ring;
    ring.reserve(static_cast<size_t>(points) * 2 + 1);
    for (int i = 0; i < points * 2; ++i) {
        double angle = (M_PI * static_cast<double>(i)) / static_cast<double>(points);
        double r = (i % 2 == 0) ? outer_r : outer_r * 0.45;
        ring.emplace_back(std::initializer_list<double>{
            cx + r * std::cos(angle),
            cy + r * std::sin(angle),
        });
    }
    ring.push_back(ring.front());
    PolyTestSolidGeometry2 poly;
    std::mt19937 gen(42);
    poly.append_line_poly(ring.data(), static_cast<int>(ring.size()), gen);
    poly.finalize(gen);
    return poly;
}

TEST_CASE("guide star ring cavity neighbor snap", "[guide][hardened]") {
    std::vector<SolidGeometry2> polys{
        make_star_polygon(0.0, 0.0, 0.35, 8),
    };
    const double ring_r = 1.4;
    for (int i = 0; i < 6; ++i) {
        double angle = (2.0 * M_PI * static_cast<double>(i)) / 6.0;
        polys.push_back(make_star_polygon(
            ring_r * std::cos(angle),
            ring_r * std::sin(angle),
            0.25,
            6
        ));
    }

    GuidanceConfig<Vec2> cfg;
    cfg.use_gravity = false;
    cfg.use_target_attractor = false;
    cfg.use_tight_packing = true;
    cfg.use_corner_alignment = true;
    cfg.enable_grid_exploration = false;
    cfg.max_alternative_angles = 1;
    cfg.search_radius = 3.0;
    cfg.diversity_distance_threshold = 0.01;
    cfg.max_propositions = 16;

    auto guidance = evaluate_local_placement<Vec2>(0, polys, Vec2{{0.0, 0.0}}, cfg);

    REQUIRE_FALSE(guidance.is_penetrating);
    REQUIRE(has_move_type(guidance.propositions, "Exact Neighbor Snap"));
}

TEST_CASE("guide gear pinch penetrating escape menu", "[guide][hardened]") {
    std::vector<SolidGeometry2> polys{
        make_box(0.0, 0.0, 0.35, 0.35),
        make_box(-0.55, 0.0, 0.35, 0.35),
        make_box(1.5, 0.0, 0.35, 0.35),
    };

    GuidanceConfig<Vec2> cfg;
    cfg.use_gravity = false;
    cfg.use_target_attractor = false;
    cfg.max_alternative_angles = 0;
    cfg.search_radius = 3.0;
    cfg.diversity_distance_threshold = 0.01;
    cfg.max_propositions = 12;

    auto guidance = evaluate_local_placement<Vec2>(0, polys, Vec2{{0.0, 0.0}}, cfg);

    REQUIRE(guidance.is_penetrating);
    REQUIRE(has_move_type(guidance.propositions, "Primary Ejection"));
    const bool has_slide_escape =
        has_move_type(guidance.propositions, "Slide Escape L")
        || has_move_type(guidance.propositions, "Slide Escape R");
    REQUIRE(has_slide_escape);
}

TEST_CASE("guide post-ejection cast along escape normal", "[guide][hardened]") {
    std::vector<SolidGeometry2> polys{
        make_box(0.0, 0.0, 0.5, 0.5),
        make_box(-0.45, 0.0, 0.5, 0.5),
        make_box(1.2, 0.0, 0.5, 0.5),
    };

    GuidanceConfig<Vec2> cfg;
    cfg.use_gravity = false;
    cfg.use_target_attractor = false;
    cfg.max_alternative_angles = 0;
    cfg.escape_radius_multiplier = 0.35;
    cfg.search_radius = 3.0;
    cfg.diversity_distance_threshold = 0.01;
    cfg.max_propositions = 12;

    auto guidance = evaluate_local_placement<Vec2>(0, polys, Vec2{{0.0, 0.0}}, cfg);

    REQUIRE(guidance.is_penetrating);
    REQUIRE(has_move_type(guidance.propositions, "Primary Ejection"));
    REQUIRE(guidance.propositions[0].translation[0] > 0.0);
}
