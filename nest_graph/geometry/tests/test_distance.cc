#include <cmath>
#include <initializer_list>
#include <utility>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "geometry/convex/penetration.h"
#include "geometry/distance/polygon_distance.h"
#include "tests/geometry_test_helpers.h"
#include <vec.h>

using Vec2 = PolyTestVec2;
using SolidGeometry2 = PolyTestSolidGeometry2;

SolidGeometry2 polygon_from_ring(std::initializer_list<std::initializer_list<double>> pts) {
    SolidGeometry2 poly;
    std::vector<Vec2> ring;
    for (const auto& p : pts) {
        auto it = p.begin();
        ring.emplace_back(std::initializer_list<double>{*it, *++it});
    }
    if (!ring.empty()) ring.push_back(ring.front());
    poly.append_line_poly(ring.data(), static_cast<int>(ring.size()));
    poly.finalize();
    return poly;
}

// Stress-spec hex topology; shifted −2 in X so the hex overlaps the 2×2 box in area (EPA needs interior overlap).
SolidGeometry2 polygon_tc3_spec_hex() {
    return polygon_from_ring({
        {0, 0},
        {2, -1},
        {4, 0},
        {4, 2},
        {2, 3},
        {0, 2},
    });
}

const ComplexDistanceResult<Vec2>* find_distance_result(
    const std::vector<ComplexDistanceResult<Vec2>>& results,
    int a,
    int b
) {
    int lo = std::min(a, b);
    int hi = std::max(a, b);
    for (const auto& r : results) {
        if (r.polyA_idx == lo && r.polyB_idx == hi) {
            return &r;
        }
    }
    return nullptr;
}

std::vector<SolidGeometry2> make_tc5_subset_world() {
    return {
        polygon_from_quad({
            {-100, -5},
            {100, -5},
            {100, 0},
            {-100, 0},
        }),
        polygon_from_quad({
            {90, 0},
            {100, 0},
            {100, 50},
            {90, 50},
        }),
        polygon_from_quad({
            {95, -2},
            {105, -2},
            {105, 8},
            {95, 8},
        }),
    };
}

void assert_tc5_subset_distance_and_narrow_phase(
    const std::vector<ComplexDistanceResult<Vec2>>& results,
    const DebugTracer& tracer
) {
    const auto* r02 = find_distance_result(results, 0, 2);
    const auto* r12 = find_distance_result(results, 1, 2);
    REQUIRE(r02 != nullptr);
    REQUIRE(r12 != nullptr);
    REQUIRE(r02->intersect);
    REQUIRE(r12->intersect);
    REQUIRE(r02->penetration_sq > 0.0);
    REQUIRE(r12->penetration_sq > 0.0);

    REQUIRE_FALSE(tracer.saw_any_narrow_pair(0, 1));
    REQUIRE(tracer.saw_penetration_pair(0, 2));
    REQUIRE(tracer.saw_penetration_pair(1, 2));
}

TEST_CASE("stress distance TC1 giant floor aura and P3 pruning", "[poly_distance][stress]") {
    DebugTracer tracer;
    tracer.reset();

    const double aura = 0.5;

    auto p0 = polygon_from_quad({
        {-100, -5},
        {100, -5},
        {100, 0},
        {-100, 0},
    });
    auto p1 = polygon_from_quad({
        {-5, -2},
        {5, -2},
        {5, 8},
        {-5, 8},
    });
    auto p2 = polygon_from_quad({
        {15, 5},
        {25, 5},
        {25, 15},
        {15, 15},
    });
    auto p3 = polygon_from_quad({
        {200, 0},
        {210, 0},
        {210, 10},
        {200, 10},
    });

    std::vector<SolidGeometry2> polys = {p0, p1, p2, p3};
    auto dist_results = find_polygon_distances(polys, aura, &tracer);

    for (const auto& r : dist_results) {
        REQUIRE(r.polyA_idx != 3);
        REQUIRE(r.polyB_idx != 3);
    }

    const auto* r01 = find_distance_result(dist_results, 0, 1);
    REQUIRE(r01 != nullptr);
    REQUIRE(r01->intersect);
    REQUIRE(r01->penetration_sq == Catch::Approx(4.0).epsilon(1e-9));

    const auto* r02 = find_distance_result(dist_results, 0, 2);
    REQUIRE(r02 != nullptr);
    REQUIRE_FALSE(r02->intersect);
    REQUIRE(r02->distance_sq == Catch::Approx(25.0).epsilon(1e-9));

    REQUIRE_FALSE(tracer.saw_any_narrow_pair(0, 3));
    REQUIRE_FALSE(tracer.saw_any_narrow_pair(1, 3));
    REQUIRE_FALSE(tracer.saw_any_narrow_pair(2, 3));

    REQUIRE(tracer.stat_sweep_pairs > 0);
    REQUIRE(tracer.stat_gjk_evals > 0);
    REQUIRE(tracer.stat_gjk_evals < 50);
}

TEST_CASE("stress distance TC2 concave C gap and donut hole invalidation", "[poly_distance][stress]") {
    DebugTracer tracer;
    tracer.reset();

    const double aura = 0.5;

    auto c_shape = polygon_c_shape_three_parts();
    auto trapped = polygon_from_quad({{4, 4}, {6, 4}, {6, 6}, {4, 6}});
    auto donut = polygon_donut_with_square_hole();
    auto core = polygon_from_quad({{24, 4}, {26, 4}, {26, 6}, {24, 6}});

    std::vector<SolidGeometry2> polys = {c_shape, trapped, donut, core};
    auto results = find_polygon_distances(polys, aura, &tracer);

    const auto* r01 = find_distance_result(results, 0, 1);
    REQUIRE(r01 != nullptr);
    REQUIRE_FALSE(r01->intersect);
    REQUIRE(r01->distance_sq == Catch::Approx(4.0).epsilon(1e-9));

    const auto* r23 = find_distance_result(results, 2, 3);
    REQUIRE(r23 != nullptr);
    REQUIRE_FALSE(r23->intersect);

    REQUIRE(tracer.stat_sweep_pairs > 0);
    REQUIRE(tracer.stat_gjk_evals > 0);
}

TEST_CASE("stress distance TC3 spec hex EPA regression direct penetration", "[poly_distance][TC3][regression][penetration]") {
    auto small_box = polygon_from_quad({{0, 0}, {2, 0}, {2, 2}, {0, 2}});
    auto hex = polygon_tc3_spec_hex();
    REQUIRE(hex.get_part_size(0) == 7);

    auto pr = convex_linestrings_penetration<Vec2>(
        small_box.get_part_points(0), small_box.get_part_size(0),
        hex.get_part_points(0), hex.get_part_size(0));

    REQUIRE(pr.intersect);
    REQUIRE(pr.penetration_sq > 0.0);
    REQUIRE(pr.mtv.qlen() > 1e-12);
}

TEST_CASE("stress distance TC3 EPA MTV inversion small box first", "[poly_distance][stress]") {
    const double aura = 0.5;

    auto small_box = polygon_from_quad({{0, 0}, {2, 0}, {2, 2}, {0, 2}});
    auto hex = polygon_tc3_spec_hex();
    REQUIRE(small_box.get_part_size(0) == 5);
    REQUIRE(hex.get_part_size(0) == 7);
    std::vector<SolidGeometry2> polys = {small_box, hex};

    auto results = find_polygon_distances<Vec2>(polys, aura);
    const auto* r = find_distance_result(results, 0, 1);
    REQUIRE(r != nullptr);
    REQUIRE(r->intersect);
    REQUIRE(r->penetration_sq > 0.0);

    REQUIRE(r->mtv[0] < 0.0);
    REQUIRE(std::abs(r->mtv[0]) > std::abs(r->mtv[1]) + 1e-6);
}

TEST_CASE("stress distance TC3 EPA MTV inversion hex first", "[poly_distance][stress]") {
    const double aura = 0.5;

    auto small_box = polygon_from_quad({{0, 0}, {2, 0}, {2, 2}, {0, 2}});
    auto hex = polygon_tc3_spec_hex();
    std::vector<SolidGeometry2> polys = {hex, small_box};

    auto results = find_polygon_distances<Vec2>(polys, aura);
    const auto* r = find_distance_result(results, 0, 1);
    REQUIRE(r != nullptr);
    REQUIRE(r->intersect);
    REQUIRE(r->penetration_sq > 0.0);

    REQUIRE(r->mtv[0] > 0.0);
    REQUIRE(std::abs(r->mtv[0]) > std::abs(r->mtv[1]) + 1e-6);
}

TEST_CASE("stress distance TC5 subset sweep active vs static no static-static narrow phase", "[poly_distance][stress][subset]") {
    DebugTracer tracer;
    tracer.reset();

    const double aura = 0.5;

    std::vector<SolidGeometry2> polys = make_tc5_subset_world();
    auto results = find_polygon_distances(polys, std::vector<int>{2}, aura, &tracer);

    assert_tc5_subset_distance_and_narrow_phase(results, tracer);
    REQUIRE(tracer.stat_sweep_pairs == 2);
}

TEST_CASE("stress distance TC6 collinear seam hover distance", "[poly_distance][stress][seam]") {
    DebugTracer tracer;
    tracer.reset();

    const double aura = 0.5;

    auto tile_l = polygon_from_quad({{-10, 0}, {0, 0}, {0, -5}, {-10, -5}});
    auto tile_r = polygon_from_quad({{0, 0}, {10, 0}, {10, -5}, {0, -5}});
    auto hover = polygon_from_quad({{-2, 2}, {2, 2}, {2, 4}, {-2, 4}});

    std::vector<SolidGeometry2> polys = {tile_l, tile_r, hover};
    auto results = find_polygon_distances<Vec2, DebugTracer>(polys, aura, &tracer);

    const auto* r02 = find_distance_result(results, 0, 2);
    const auto* r12 = find_distance_result(results, 1, 2);
    REQUIRE(r02 != nullptr);
    REQUIRE(r12 != nullptr);
    REQUIRE_FALSE(r02->intersect);
    REQUIRE_FALSE(r12->intersect);
    REQUIRE(r02->distance_sq == Catch::Approx(4.0).epsilon(1e-9));
    REQUIRE(r12->distance_sq == Catch::Approx(4.0).epsilon(1e-9));

    REQUIRE(tracer.stat_sweep_pairs == 3);
    REQUIRE(tracer.stat_gjk_evals == 3);
}

TEST_CASE("stress distance TC7 kissing edge contact zero penetration", "[poly_distance][stress][kiss]") {
    DebugTracer tracer;
    tracer.reset();

    const double aura = 0.5;

    auto a = polygon_from_quad({{0, 0}, {2, 0}, {2, 2}, {0, 2}});
    auto b = polygon_from_quad({{2, 0}, {4, 0}, {4, 2}, {2, 2}});

    auto results = find_polygon_distances<Vec2, DebugTracer>({a, b}, aura, &tracer);
    const auto* r = find_distance_result(results, 0, 1);
    REQUIRE(r != nullptr);
    REQUIRE(r->intersect);
    REQUIRE(r->penetration_sq == Catch::Approx(0.0).margin(1e-9));
    REQUIRE(r->mtv.qlen() < 1e-8);

    REQUIRE(tracer.stat_sweep_pairs >= 1);
    REQUIRE(tracer.stat_gjk_evals >= 1);
    REQUIRE(tracer.stat_gjk_evals < 20);
}

TEST_CASE("Physics Telemetry: Static vs Static Pruning (Subset Mode)", "[telemetry][broadphase][subset]") {
    DebugTracer tracer;
    tracer.reset();

    const double aura = 0.5;

    std::vector<SolidGeometry2> polys = make_tc5_subset_world();
    auto results = find_polygon_distances<Vec2, DebugTracer>(polys, std::vector<int>{2}, aura, &tracer);

    assert_tc5_subset_distance_and_narrow_phase(results, tracer);
    REQUIRE(tracer.stat_sweep_pairs == 2);

    maybe_print_physics_telemetry(tracer);
}

TEST_CASE("Physics Telemetry: The Planar Seam Sliding Test", "[telemetry][narrowphase][gjk][seam]") {
    DebugTracer tracer;
    tracer.reset();

    const double aura = 0.5;

    // Two floor tiles meeting at x = 0, top at y = 0; hover y = 5..9 -> vertical gap 5, dist^2 = 25.
    auto tile_l = make_box(-10.0, -5.0, 10.0, 5.0);
    auto tile_r = make_box(10.0, -5.0, 10.0, 5.0);
    auto hover = make_box(0.0, 7.0, 2.0, 2.0);

    std::vector<SolidGeometry2> world = {tile_l, tile_r, hover};
    auto dist_results = find_polygon_distances<Vec2, DebugTracer>(world, aura, &tracer);

    const auto* r02 = find_distance_result(dist_results, 0, 2);
    const auto* r12 = find_distance_result(dist_results, 1, 2);
    REQUIRE(r02 != nullptr);
    REQUIRE(r12 != nullptr);
    REQUIRE_FALSE(r02->intersect);
    REQUIRE_FALSE(r12->intersect);
    REQUIRE(r02->distance_sq == Catch::Approx(25.0).epsilon(1e-9));
    REQUIRE(r12->distance_sq == Catch::Approx(25.0).epsilon(1e-9));

    REQUIRE(tracer.stat_sweep_pairs == 3);
    REQUIRE(tracer.stat_gjk_evals == 3);

    maybe_print_physics_telemetry(tracer);
}
