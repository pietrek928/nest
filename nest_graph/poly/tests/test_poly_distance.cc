#include <cmath>
#include <initializer_list>
#include <utility>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "poly/convex/penetration.h"
#include "poly/poly.h"
#include "poly/poly_distance.h"
#include "poly/tracer.h"
#include "vec.h"

using Vec2 = Vec<2, double>;
using Polygon2 = Polygon<Vec2>;

namespace {

Polygon2 polygon_from_quad(std::initializer_list<std::initializer_list<double>> pts4) {
    Polygon2 poly;
    std::vector<Vec2> ring;
    ring.reserve(4);
    for (const auto& p : pts4) {
        auto it = p.begin();
        ring.emplace_back(std::initializer_list<double>{*it, *++it});
    }
    poly.append_convex_poly(ring.data(), static_cast<int>(ring.size()));
    poly.finalize();
    return poly;
}

Polygon2 polygon_from_ring(std::initializer_list<std::initializer_list<double>> pts) {
    Polygon2 poly;
    std::vector<Vec2> ring;
    for (const auto& p : pts) {
        auto it = p.begin();
        ring.emplace_back(std::initializer_list<double>{*it, *++it});
    }
    poly.append_convex_poly(ring.data(), static_cast<int>(ring.size()));
    poly.finalize();
    return poly;
}

Polygon2 polygon_c_shape_three_parts() {
    Polygon2 poly;
    std::vector<Vec2> bottom{
        Vec2{{0.0, 0.0}},
        Vec2{{10.0, 0.0}},
        Vec2{{10.0, 2.0}},
        Vec2{{0.0, 2.0}},
    };
    std::vector<Vec2> back{
        Vec2{{0.0, 2.0}},
        Vec2{{2.0, 2.0}},
        Vec2{{2.0, 8.0}},
        Vec2{{0.0, 8.0}},
    };
    std::vector<Vec2> top{
        Vec2{{0.0, 8.0}},
        Vec2{{10.0, 8.0}},
        Vec2{{10.0, 10.0}},
        Vec2{{0.0, 10.0}},
    };
    poly.append_convex_poly(bottom.data(), static_cast<int>(bottom.size()));
    poly.append_convex_poly(back.data(), static_cast<int>(back.size()));
    poly.append_convex_poly(top.data(), static_cast<int>(top.size()));
    poly.finalize();
    return poly;
}

Polygon2 polygon_donut_with_square_hole() {
    Polygon2 poly;
    std::vector<Vec2> outer{
        Vec2{{20.0, 0.0}},
        Vec2{{30.0, 0.0}},
        Vec2{{30.0, 10.0}},
        Vec2{{20.0, 10.0}},
    };
    std::vector<Vec2> hole{
        Vec2{{22.0, 2.0}},
        Vec2{{28.0, 2.0}},
        Vec2{{28.0, 8.0}},
        Vec2{{22.0, 8.0}},
    };
    poly.append_convex_poly(outer.data(), static_cast<int>(outer.size()));
    poly.append_convex_hole(hole.data(), static_cast<int>(hole.size()));
    poly.finalize();
    return poly;
}

// Stress-spec hex topology; shifted −2 in X so the hex overlaps the 2×2 box in area (EPA needs interior overlap).
Polygon2 polygon_tc3_spec_hex() {
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

} // namespace

TEST_CASE("stress distance TC1 giant floor aura and P3 pruning", "[poly_distance][stress]") {
    Vec2 axis{{1.0, 0.0}};
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

    std::vector<Polygon2> polys = {p0, p1, p2, p3};
    auto dist_results = find_polygon_distances<Vec2>(polys, axis, aura);

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
}

TEST_CASE("stress distance TC2 concave C gap and donut hole invalidation", "[poly_distance][stress]") {
    Vec2 axis{{1.0, 0.0}};
    const double aura = 0.5;

    auto c_shape = polygon_c_shape_three_parts();
    auto trapped = polygon_from_quad({{4, 4}, {6, 4}, {6, 6}, {4, 6}});
    auto donut = polygon_donut_with_square_hole();
    auto core = polygon_from_quad({{24, 4}, {26, 4}, {26, 6}, {24, 6}});

    std::vector<Polygon2> polys = {c_shape, trapped, donut, core};
    auto results = find_polygon_distances<Vec2>(polys, axis, aura);

    const auto* r01 = find_distance_result(results, 0, 1);
    REQUIRE(r01 != nullptr);
    REQUIRE_FALSE(r01->intersect);
    REQUIRE(r01->distance_sq == Catch::Approx(4.0).epsilon(1e-9));

    const auto* r23 = find_distance_result(results, 2, 3);
    REQUIRE(r23 != nullptr);
    REQUIRE_FALSE(r23->intersect);
}

TEST_CASE("stress distance TC3 spec hex EPA regression direct penetration", "[poly_distance][TC3][regression][penetration]") {
    auto small_box = polygon_from_quad({{0, 0}, {2, 0}, {2, 2}, {0, 2}});
    auto hex = polygon_tc3_spec_hex();
    REQUIRE(hex.get_part_size(0) == 6);

    auto pr = convex_polygons_penetration<Vec2>(
        small_box.get_part_points(0), small_box.get_part_size(0),
        hex.get_part_points(0), hex.get_part_size(0));

    REQUIRE(pr.intersect);
    REQUIRE(pr.penetration_sq > 0.0);
    REQUIRE(pr.mtv.qlen() > 1e-12);
}

TEST_CASE("stress distance TC3 EPA MTV inversion small box first", "[poly_distance][stress]") {
    Vec2 axis{{1.0, 1.0}};
    const double aura = 0.5;

    auto small_box = polygon_from_quad({{0, 0}, {2, 0}, {2, 2}, {0, 2}});
    auto hex = polygon_tc3_spec_hex();
    REQUIRE(small_box.get_part_size(0) == 4);
    REQUIRE(hex.get_part_size(0) == 6);
    std::vector<Polygon2> polys = {small_box, hex};

    auto results = find_polygon_distances<Vec2>(polys, axis, aura);
    const auto* r = find_distance_result(results, 0, 1);
    REQUIRE(r != nullptr);
    REQUIRE(r->intersect);
    REQUIRE(r->penetration_sq > 0.0);

    REQUIRE(r->mtv[0] < 0.0);
    REQUIRE(std::abs(r->mtv[0]) > std::abs(r->mtv[1]) + 1e-6);
}

TEST_CASE("stress distance TC3 EPA MTV inversion hex first", "[poly_distance][stress]") {
    Vec2 axis{{1.0, 1.0}};
    const double aura = 0.5;

    auto small_box = polygon_from_quad({{0, 0}, {2, 0}, {2, 2}, {0, 2}});
    auto hex = polygon_tc3_spec_hex();
    std::vector<Polygon2> polys = {hex, small_box};

    auto results = find_polygon_distances<Vec2>(polys, axis, aura);
    const auto* r = find_distance_result(results, 0, 1);
    REQUIRE(r != nullptr);
    REQUIRE(r->intersect);
    REQUIRE(r->penetration_sq > 0.0);

    REQUIRE(r->mtv[0] > 0.0);
    REQUIRE(std::abs(r->mtv[0]) > std::abs(r->mtv[1]) + 1e-6);
}

#ifdef NEST_GRAPH_NARROW_PHASE_SPY

TEST_CASE("stress distance TC5 subset sweep active vs static no static-static narrow phase", "[poly_distance][stress][subset]") {
    nest_graph::narrow_phase_spy_clear();

    Vec2 axis{{1.0, 0.0}};
    const double aura = 0.5;

    auto floor = polygon_from_quad({
        {-100, -5},
        {100, -5},
        {100, 0},
        {-100, 0},
    });
    auto wall = polygon_from_quad({
        {90, 0},
        {100, 0},
        {100, 50},
        {90, 50},
    });
    auto active = polygon_from_quad({
        {95, -2},
        {105, -2},
        {105, 8},
        {95, 8},
    });

    std::vector<Polygon2> polys = {floor, wall, active};
    auto results = find_polygon_distances<Vec2>(polys, std::vector<int>{2}, axis, aura);

    const auto* r02 = find_distance_result(results, 0, 2);
    const auto* r12 = find_distance_result(results, 1, 2);
    REQUIRE(r02 != nullptr);
    REQUIRE(r12 != nullptr);
    REQUIRE(r02->intersect);
    REQUIRE(r12->intersect);
    REQUIRE(r02->penetration_sq > 0.0);
    REQUIRE(r12->penetration_sq > 0.0);

    REQUIRE_FALSE(nest_graph::narrow_phase_spy_saw_any_narrow_pair(0, 1));
    REQUIRE(nest_graph::narrow_phase_spy_saw_penetration_pair(0, 2));
    REQUIRE(nest_graph::narrow_phase_spy_saw_penetration_pair(1, 2));
}

#endif

TEST_CASE("stress distance TC6 collinear seam hover distance", "[poly_distance][stress][seam]") {
    Vec2 axis{{1.0, 0.0}};
    const double aura = 0.5;

    auto tile_l = polygon_from_quad({{-10, 0}, {0, 0}, {0, -5}, {-10, -5}});
    auto tile_r = polygon_from_quad({{0, 0}, {10, 0}, {10, -5}, {0, -5}});
    auto hover = polygon_from_quad({{-2, 2}, {2, 2}, {2, 4}, {-2, 4}});

    std::vector<Polygon2> polys = {tile_l, tile_r, hover};
    auto results = find_polygon_distances<Vec2>(polys, axis, aura);

    const auto* r02 = find_distance_result(results, 0, 2);
    const auto* r12 = find_distance_result(results, 1, 2);
    REQUIRE(r02 != nullptr);
    REQUIRE(r12 != nullptr);
    REQUIRE_FALSE(r02->intersect);
    REQUIRE_FALSE(r12->intersect);
    REQUIRE(r02->distance_sq == Catch::Approx(4.0).epsilon(1e-9));
    REQUIRE(r12->distance_sq == Catch::Approx(4.0).epsilon(1e-9));
}

TEST_CASE("stress distance TC7 kissing edge contact zero penetration", "[poly_distance][stress][kiss]") {
    Vec2 axis{{1.0, 0.0}};
    const double aura = 0.5;

    auto a = polygon_from_quad({{0, 0}, {2, 0}, {2, 2}, {0, 2}});
    auto b = polygon_from_quad({{2, 0}, {4, 0}, {4, 2}, {2, 2}});

    auto results = find_polygon_distances<Vec2>({a, b}, axis, aura);
    const auto* r = find_distance_result(results, 0, 1);
    REQUIRE(r != nullptr);
    REQUIRE(r->intersect);
    REQUIRE(r->penetration_sq == Catch::Approx(0.0).margin(1e-9));
    REQUIRE(r->mtv.qlen() < 1e-8);
}
