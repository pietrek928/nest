#include <catch2/catch_test_macros.hpp>

#include "poly/decompose.h"
#include "poly/point_in_solid.h"
#include "poly/poly_containment.h"
#include "poly/poly_distance.h"
#include "poly/poly_intersect.h"
#include "tests/poly_test_helpers.h"

using Vec2 = PolyTestVec2;
using SolidGeometry2 = PolyTestSolidGeometry2;

TEST_CASE("point in solid: annulus vs hole", "[holes][point_in_solid]") {
    std::vector<Vec2> outer{
        Vec2{{0.0, 0.0}},
        Vec2{{10.0, 0.0}},
        Vec2{{10.0, 10.0}},
        Vec2{{0.0, 10.0}},
    };
    std::vector<Vec2> hole{
        Vec2{{4.0, 4.0}},
        Vec2{{6.0, 4.0}},
        Vec2{{6.0, 6.0}},
        Vec2{{4.0, 6.0}},
    };
    SolidGeometry2 donut = make_donut(outer, hole);

    REQUIRE(is_point_inside_solid_space(Vec2{{2.0, 2.0}}, donut));
    REQUIRE_FALSE(is_point_inside_solid_space(Vec2{{5.0, 5.0}}, donut));
}

TEST_CASE("intersection: probe inside hole is disjoint", "[holes][poly_intersect]") {
    auto shell = polygon_outer_with_square_hole(0, 0, 10, 10, 4, 4, 6, 6);
    auto probe_inside_void = polygon_from_quad({
        {4.25, 4.25},
        {5.75, 4.25},
        {5.75, 5.75},
        {4.25, 5.75},
    });
    REQUIRE(find_polygon_intersections<Vec2>({shell, probe_inside_void}).empty());
}

TEST_CASE("point in solid: thin slab does not contain distant point", "[holes][point_in_solid]") {
    auto wall = polygon_from_quad({{10, 0}, {11, 0}, {11, 10}, {10, 10}});
    REQUIRE_FALSE(is_point_inside_solid_space(Vec2{{5.0, 5.0}}, wall));
    REQUIRE_FALSE(try_add_containment_collision(wall, polygon_from_quad({{5, 5}, {6, 5}, {6, 6}, {5, 6}})));
}

TEST_CASE("intersection: probe in annulus overlaps shell", "[holes][poly_intersect]") {
    auto shell = polygon_outer_with_square_hole(0, 0, 10, 10, 4, 4, 6, 6);
    auto probe_in_annulus = polygon_from_quad({{1, 1}, {3, 1}, {3, 3}, {1, 3}});
    REQUIRE_FALSE(find_polygon_intersections<Vec2>({shell, probe_in_annulus}).empty());
}

TEST_CASE("intersection: symmetric host polygon hole and foreign solid", "[holes][poly_intersect]") {
    auto shell_b = polygon_outer_with_square_hole(20, 0, 30, 10, 24, 4, 26, 6);
    auto probe_b = polygon_from_quad({
        {24.25, 4.25},
        {25.75, 4.25},
        {25.75, 5.75},
        {24.25, 5.75},
    });
    REQUIRE(find_polygon_intersections<Vec2>({shell_b, probe_b}).empty());

    auto solid_a = polygon_from_quad({{0, 0}, {2, 0}, {2, 2}, {0, 2}});
    REQUIRE(find_polygon_intersections<Vec2>({solid_a, shell_b}).empty());
}

TEST_CASE("decomposed donut has subtractive line parts excluded from sweep", "[holes]") {
    auto donut = polygon_donut_with_square_hole();
    bool has_subtractive = false;
    for (const auto& part : donut.line_parts) {
        if (part.is_subtractive) {
            has_subtractive = true;
            break;
        }
    }
    REQUIRE(has_subtractive);

    auto core = polygon_from_quad({{24, 4}, {26, 4}, {26, 6}, {24, 6}});
    const auto dist = find_polygon_distances<Vec2>({donut, core});
    const ComplexDistanceResult<Vec2>* found = nullptr;
    for (const auto& d : dist) {
        if ((d.polyA_idx == 0 && d.polyB_idx == 1) || (d.polyA_idx == 1 && d.polyB_idx == 0)) {
            found = &d;
            break;
        }
    }
    REQUIRE(found != nullptr);
    REQUIRE_FALSE(found->intersect);
}
