#include <initializer_list>
#include <utility>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "poly/poly.h"
#include "poly/poly_intersect.h"
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

// CCW rectangle with dense vertices along bottom edge [x0,x1] at y = y_lo (integer x step).
Polygon2 polygon_dense_bottom_rect(int x0, int x1, double y_lo, double y_hi) {
    Polygon2 poly;
    std::vector<Vec2> ring;
    for (int x = x0; x <= x1; ++x) {
        ring.emplace_back(std::initializer_list<double>{static_cast<double>(x), y_lo});
    }
    ring.emplace_back(std::initializer_list<double>{static_cast<double>(x1), y_hi});
    ring.emplace_back(std::initializer_list<double>{static_cast<double>(x0), y_hi});
    poly.append_convex_poly(ring.data(), static_cast<int>(ring.size()));
    poly.finalize();
    return poly;
}

Polygon2 polygon_outer_with_square_hole(
    double ox0, double oy0, double ox1, double oy1,
    double hx0, double hy0, double hx1, double hy1
) {
    Polygon2 poly;
    std::vector<Vec2> outer{
        Vec2{{ox0, oy0}},
        Vec2{{ox1, oy0}},
        Vec2{{ox1, oy1}},
        Vec2{{ox0, oy1}},
    };
    std::vector<Vec2> hole{
        Vec2{{hx0, hy0}},
        Vec2{{hx1, hy0}},
        Vec2{{hx1, hy1}},
        Vec2{{hx0, hy1}},
    };
    poly.append_convex_poly(outer.data(), static_cast<int>(outer.size()));
    poly.append_convex_hole(hole.data(), static_cast<int>(hole.size()));
    poly.finalize();
    return poly;
}

Polygon2 polygon_two_disjoint_parts() {
    Polygon2 poly;
    std::vector<Vec2> r1{
        Vec2{{0.0, 0.0}},
        Vec2{{1.0, 0.0}},
        Vec2{{1.0, 1.0}},
        Vec2{{0.0, 1.0}},
    };
    std::vector<Vec2> r2{
        Vec2{{50.0, 50.0}},
        Vec2{{51.0, 50.0}},
        Vec2{{51.0, 51.0}},
        Vec2{{50.0, 51.0}},
    };
    poly.append_convex_poly(r1.data(), static_cast<int>(r1.size()));
    poly.append_convex_poly(r2.data(), static_cast<int>(r2.size()));
    poly.finalize();
    return poly;
}

Polygon2 polygon_two_overlapping_parts_same_partner() {
    Polygon2 poly;
    std::vector<Vec2> bar_low{
        Vec2{{0.0, 0.0}},
        Vec2{{4.0, 0.0}},
        Vec2{{4.0, 1.0}},
        Vec2{{0.0, 1.0}},
    };
    std::vector<Vec2> bar_high{
        Vec2{{0.0, 3.0}},
        Vec2{{4.0, 3.0}},
        Vec2{{4.0, 4.0}},
        Vec2{{0.0, 4.0}},
    };
    poly.append_convex_poly(bar_low.data(), static_cast<int>(bar_low.size()));
    poly.append_convex_poly(bar_high.data(), static_cast<int>(bar_high.size()));
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

} // namespace

TEST_CASE("narrow_phase_intersect disjoint convex", "[poly_intersect][router]") {
    auto sq_a = polygon_from_quad({{0, 0}, {2, 0}, {2, 2}, {0, 2}});
    auto sq_b = polygon_from_quad({{5, 0}, {7, 0}, {7, 2}, {5, 2}});
    REQUIRE_FALSE(narrow_phase_intersect<Vec2>(
        sq_a.get_part_points(0), sq_a.get_part_size(0),
        sq_b.get_part_points(0), sq_b.get_part_size(0)));
}

TEST_CASE("narrow_phase_intersect touch vertex", "[poly_intersect][router]") {
    auto poly_a = polygon_from_quad({{0, 0}, {2, 0}, {2, 2}, {0, 2}});
    auto poly_b = polygon_from_quad({{2, 2}, {4, 2}, {4, 4}, {2, 4}});
    REQUIRE(narrow_phase_intersect<Vec2>(
        poly_a.get_part_points(0), poly_a.get_part_size(0),
        poly_b.get_part_points(0), poly_b.get_part_size(0)));
}

TEST_CASE("narrow_phase_intersect touch edge", "[poly_intersect][router]") {
    auto poly_a = polygon_from_quad({{0, 0}, {2, 0}, {2, 2}, {0, 2}});
    auto poly_b = polygon_from_quad({{2, 0}, {4, 0}, {4, 2}, {2, 2}});
    REQUIRE(narrow_phase_intersect<Vec2>(
        poly_a.get_part_points(0), poly_a.get_part_size(0),
        poly_b.get_part_points(0), poly_b.get_part_size(0)));
}

TEST_CASE("narrow_phase_intersect overlap interior", "[poly_intersect][router]") {
    auto poly_a = polygon_from_quad({{0, 0}, {2, 0}, {2, 2}, {0, 2}});
    auto poly_b = polygon_from_quad({{1, 1}, {3, 1}, {3, 3}, {1, 3}});
    REQUIRE(narrow_phase_intersect<Vec2>(
        poly_a.get_part_points(0), poly_a.get_part_size(0),
        poly_b.get_part_points(0), poly_b.get_part_size(0)));
}

TEST_CASE("narrow_phase_intersect swap argument order symmetry", "[poly_intersect][router]") {
    auto poly_a = polygon_from_quad({{0, 0}, {2, 0}, {2, 2}, {0, 2}});
    auto poly_b = polygon_from_quad({{5, 0}, {7, 0}, {7, 2}, {5, 2}});
    bool ab = narrow_phase_intersect<Vec2>(
        poly_a.get_part_points(0), poly_a.get_part_size(0),
        poly_b.get_part_points(0), poly_b.get_part_size(0));
    bool ba = narrow_phase_intersect<Vec2>(
        poly_b.get_part_points(0), poly_b.get_part_size(0),
        poly_a.get_part_points(0), poly_a.get_part_size(0));
    REQUIRE(ab == ba);
}

TEST_CASE("narrow_phase_intersect gradient threshold parity", "[poly_intersect][router]") {
    auto sq_a = polygon_from_quad({{0, 0}, {2, 0}, {2, 2}, {0, 2}});
    // n_a + n_b = 23
    auto dense23 = polygon_dense_bottom_rect(20, 36, 0.0, 2.0);
    REQUIRE(sq_a.get_part_size(0) + dense23.get_part_size(0) == 23);

    bool hit23 = narrow_phase_intersect<Vec2>(
        sq_a.get_part_points(0), sq_a.get_part_size(0),
        dense23.get_part_points(0), dense23.get_part_size(0));

    // n_a + n_b = 26 (gradient routing)
    auto dense26 = polygon_dense_bottom_rect(20, 39, 0.0, 2.0);
    REQUIRE(sq_a.get_part_size(0) + dense26.get_part_size(0) == 26);

    bool hit26 = narrow_phase_intersect<Vec2>(
        sq_a.get_part_points(0), sq_a.get_part_size(0),
        dense26.get_part_points(0), dense26.get_part_size(0));

    REQUIRE(hit23 == hit26);
    REQUIRE_FALSE(hit23);

    auto overlap_square = polygon_from_quad({{18, 0}, {22, 0}, {22, 2}, {18, 2}});
    bool ov23 = narrow_phase_intersect<Vec2>(
        overlap_square.get_part_points(0), overlap_square.get_part_size(0),
        dense23.get_part_points(0), dense23.get_part_size(0));
    bool ov26 = narrow_phase_intersect<Vec2>(
        overlap_square.get_part_points(0), overlap_square.get_part_size(0),
        dense26.get_part_points(0), dense26.get_part_size(0));
    REQUIRE(ov23);
    REQUIRE(ov26);
}

TEST_CASE("narrow_phase_contain strict inside vs escaped vertex", "[poly_intersect][router]") {
    auto outer = polygon_from_quad({{0, 0}, {4, 0}, {4, 4}, {0, 4}});
    auto inner_ok = polygon_from_quad({{1.5, 1.5}, {2.5, 1.5}, {2.5, 2.5}, {1.5, 2.5}});
    REQUIRE(narrow_phase_contain<Vec2>(
        inner_ok.get_part_points(0), inner_ok.get_part_size(0),
        outer.get_part_points(0), outer.get_part_size(0)));

    auto inner_bad = polygon_from_quad({{1.5, 1.5}, {5.5, 1.5}, {2.5, 2.5}, {1.5, 2.5}});
    REQUIRE_FALSE(narrow_phase_contain<Vec2>(
        inner_bad.get_part_points(0), inner_bad.get_part_size(0),
        outer.get_part_points(0), outer.get_part_size(0)));
}

TEST_CASE("narrow_phase_contain gradient threshold parity", "[poly_intersect][router]") {
    auto outer = polygon_from_quad({{0, 0}, {25, 0}, {25, 6}, {0, 6}});
    // Inner hull vertex counts 19 + outer 4 = 23
    auto dense_inside23 = polygon_dense_bottom_rect(5, 21, 1.0, 3.0);
    REQUIRE(outer.get_part_size(0) + dense_inside23.get_part_size(0) == 23);

    bool in23 = narrow_phase_contain<Vec2>(
        dense_inside23.get_part_points(0), dense_inside23.get_part_size(0),
        outer.get_part_points(0), outer.get_part_size(0));

    // Inner hull 22 + outer 4 = 26
    auto dense_inside26 = polygon_dense_bottom_rect(5, 24, 1.0, 3.0);
    REQUIRE(outer.get_part_size(0) + dense_inside26.get_part_size(0) == 26);

    bool in26 = narrow_phase_contain<Vec2>(
        dense_inside26.get_part_points(0), dense_inside26.get_part_size(0),
        outer.get_part_points(0), outer.get_part_size(0));

    REQUIRE(in23 == in26);
    REQUIRE(in23);

    auto dense_escape23 = polygon_dense_bottom_rect(5, 21, 5.5, 7.5);
    auto dense_escape26 = polygon_dense_bottom_rect(5, 24, 5.5, 7.5);
    REQUIRE_FALSE(narrow_phase_contain<Vec2>(
        dense_escape23.get_part_points(0), dense_escape23.get_part_size(0),
        outer.get_part_points(0), outer.get_part_size(0)));
    REQUIRE_FALSE(narrow_phase_contain<Vec2>(
        dense_escape26.get_part_points(0), dense_escape26.get_part_size(0),
        outer.get_part_points(0), outer.get_part_size(0)));
}

TEST_CASE("find_polygon_intersections guards empty corpus", "[poly_intersect][sweep]") {
    Vec2 axis{{1.0, 0.0}};
    REQUIRE(find_polygon_intersections<Vec2>({}, axis).empty());

    Polygon2 single = polygon_from_quad({{0, 0}, {2, 0}, {2, 2}, {0, 2}});
    REQUIRE(find_polygon_intersections<Vec2>({single}, axis).empty());

    REQUIRE(find_polygon_intersections<Vec2>(
                {single}, std::vector<int>{}, axis).empty());

    REQUIRE(find_polygon_intersections<Vec2>(
                {single}, std::vector<int>{0}, axis).empty());

    auto far_a = polygon_from_quad({{0, 0}, {2, 0}, {2, 2}, {0, 2}});
    auto far_b = polygon_from_quad({{50, 0}, {52, 0}, {52, 2}, {50, 2}});
    REQUIRE(find_polygon_intersections<Vec2>(
                {far_a, far_b}, Vec2{{1e-10, 0.0}}).empty());
}

TEST_CASE("find_polygon_intersections simple hit and miss", "[poly_intersect][sweep]") {
    DebugTracer tracer;
    tracer.reset();

    Vec2 axis{{1.0, 0.0}};
    auto left = polygon_from_quad({{0, 0}, {2, 0}, {2, 2}, {0, 2}});
    auto right_sep = polygon_from_quad({{10, 0}, {12, 0}, {12, 2}, {10, 2}});
    auto overlap = polygon_from_quad({{1, 1}, {3, 1}, {3, 3}, {1, 3}});

    REQUIRE(find_polygon_intersections<Vec2>({left, right_sep}, axis, &tracer).empty());

    auto hits = find_polygon_intersections<Vec2>({left, overlap}, axis, &tracer);
    REQUIRE_FALSE(hits.empty());
    REQUIRE(hits.front().first == 0);
    REQUIRE(hits.front().second == 1);
    REQUIRE(tracer.saw_penetration_pair(0, 1));
    REQUIRE(tracer.stat_gjk_evals > 0);
    REQUIRE(tracer.stat_sweep_pairs > 0);
}

TEST_CASE("find_polygon_intersections three polygons subset pairs", "[poly_intersect][sweep]") {
    Vec2 axis{{1.0, 0.0}};
    auto base = polygon_from_quad({{0, 0}, {2, 0}, {2, 2}, {0, 2}});
    auto orphan = polygon_from_quad({{100, 100}, {101, 100}, {101, 101}, {100, 101}});
    auto buddy = polygon_from_quad({{1, 1}, {3, 1}, {3, 3}, {1, 3}});
    std::vector<Polygon2> polys = {base, orphan, buddy};
    auto hits = find_polygon_intersections<Vec2>(polys, axis);
    REQUIRE(hits.size() == 1);
    REQUIRE(hits[0] == std::make_pair(0, 2));
}

TEST_CASE("find_polygon_intersections active_indices overload", "[poly_intersect][sweep]") {
    Vec2 axis{{1.0, 0.0}};
    auto base = polygon_from_quad({{0, 0}, {2, 0}, {2, 2}, {0, 2}});
    auto orphan = polygon_from_quad({{100, 100}, {101, 100}, {101, 101}, {100, 101}});
    auto buddy = polygon_from_quad({{1, 1}, {3, 1}, {3, 3}, {1, 3}});
    std::vector<Polygon2> polys = {base, orphan, buddy};

    auto overlap_only = find_polygon_intersections<Vec2>(polys, std::vector<int>{0, 2}, axis);
    REQUIRE(overlap_only.size() == 1);
    REQUIRE(overlap_only[0] == std::make_pair(0, 2));

    std::vector<Polygon2> two_far = {base, orphan};
    auto disjoint_only = find_polygon_intersections<Vec2>(two_far, std::vector<int>{0, 1}, axis);
    REQUIRE(disjoint_only.empty());
}

TEST_CASE("find_polygon_intersections subset mode static-static never narrow-phased", "[poly_intersect][sweep][subset]") {
    DebugTracer tracer;
    tracer.reset();

    Vec2 axis{{1.0, 0.0}};
    auto base = polygon_from_quad({{0, 0}, {2, 0}, {2, 2}, {0, 2}});
    auto orphan = polygon_from_quad({{100, 100}, {101, 100}, {101, 101}, {100, 101}});
    auto buddy = polygon_from_quad({{1, 1}, {3, 1}, {3, 3}, {1, 3}});
    std::vector<Polygon2> polys = {base, orphan, buddy};

    auto hits = find_polygon_intersections<Vec2>(polys, std::vector<int>{2}, axis, &tracer);
    REQUIRE(hits.size() >= 1);
    REQUIRE(tracer.saw_penetration_pair(0, 2));
    REQUIRE_FALSE(tracer.saw_any_narrow_pair(0, 1));
}

TEST_CASE("find_polygon_intersections multipolygon dedupes polygon pair", "[poly_intersect][sweep]") {
    Vec2 axis{{1.0, 0.0}};
    auto compound = polygon_two_overlapping_parts_same_partner();
    REQUIRE(compound.convex_parts.size() == 2);

    auto stripe = polygon_from_quad({{1.0, -1.0}, {3.0, -1.0}, {3.0, 5.0}, {1.0, 5.0}});
    std::vector<Polygon2> polys = {compound, stripe};
    auto hits = find_polygon_intersections<Vec2>(polys, axis);
    REQUIRE(hits.size() == 1);
    REQUIRE(hits[0] == std::make_pair(0, 1));
}

TEST_CASE("find_polygon_intersections no self collision across parts", "[poly_intersect][sweep]") {
    Vec2 axis{{1.0, 0.0}};
    auto compound = polygon_two_disjoint_parts();
    REQUIRE(compound.convex_parts.size() == 2);
    // Must pass at least two top-level polygons (entry guard); distant third body
    // should not intersect. Pairs sharing poly_idx==0 are skipped in the sweep.
    auto far = polygon_from_quad({{200.0, 200.0}, {201.0, 200.0}, {201.0, 201.0}, {200.0, 201.0}});
    auto hits = find_polygon_intersections<Vec2>({compound, far}, axis);
    REQUIRE(hits.empty());
}

TEST_CASE("find_polygon_intersections diagonal sweep axis overlap", "[poly_intersect][sweep]") {
    Vec2 axis{{1.0, 1.0}};
    auto a = polygon_from_quad({{0, 0}, {2, 0}, {2, 2}, {0, 2}});
    auto b = polygon_from_quad({{1, 1}, {3, 1}, {3, 3}, {1, 3}});
    auto hits = find_polygon_intersections<Vec2>({a, b}, axis);
    REQUIRE_FALSE(hits.empty());
}

TEST_CASE("hole invalidation swallows probe fully inside hole", "[poly_intersect][holes]") {
    Vec2 axis{{1.0, 0.0}};
    auto shell = polygon_outer_with_square_hole(0, 0, 10, 10, 4, 4, 6, 6);
    auto probe_inside_void = polygon_from_quad({
        {4.25, 4.25},
        {5.75, 4.25},
        {5.75, 5.75},
        {4.25, 5.75},
    });
    REQUIRE(find_polygon_intersections<Vec2>({shell, probe_inside_void}, axis).empty());
}

TEST_CASE("hole invalidation detects overlap with solid shell", "[poly_intersect][holes]") {
    Vec2 axis{{1.0, 0.0}};
    auto shell = polygon_outer_with_square_hole(0, 0, 10, 10, 4, 4, 6, 6);
    auto probe_in_annulus = polygon_from_quad({{1, 1}, {3, 1}, {3, 3}, {1, 3}});
    auto hits = find_polygon_intersections<Vec2>({shell, probe_in_annulus}, axis);
    REQUIRE_FALSE(hits.empty());
}

TEST_CASE("hole invalidation symmetric host polygon", "[poly_intersect][holes]") {
    Vec2 axis{{1.0, 0.0}};
    auto shell_b = polygon_outer_with_square_hole(20, 0, 30, 10, 24, 4, 26, 6);
    auto probe_b = polygon_from_quad({
        {24.25, 4.25},
        {25.75, 4.25},
        {25.75, 5.75},
        {24.25, 5.75},
    });
    REQUIRE(find_polygon_intersections<Vec2>({shell_b, probe_b}, axis).empty());

    auto solid_a = polygon_from_quad({{0, 0}, {2, 0}, {2, 2}, {0, 2}});
    REQUIRE(find_polygon_intersections<Vec2>({solid_a, shell_b}, axis).empty());
}

TEST_CASE("stress intersect TC1 giant floor only P0 P1", "[poly_intersect][stress]") {
    DebugTracer tracer;
    tracer.reset();

    Vec2 axis{{1.0, 0.0}};
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
    auto hits = find_polygon_intersections<Vec2>(polys, axis, &tracer);
    REQUIRE(hits.size() == 1);
    REQUIRE(hits[0] == std::make_pair(0, 1));
    REQUIRE(tracer.saw_penetration_pair(0, 1));
    REQUIRE_FALSE(tracer.saw_any_narrow_pair(0, 3));
    REQUIRE_FALSE(tracer.saw_any_narrow_pair(1, 3));
    REQUIRE_FALSE(tracer.saw_any_narrow_pair(2, 3));
}

TEST_CASE("stress intersect TC2 concave C trapped box donut no hits", "[poly_intersect][stress]") {
    DebugTracer tracer;
    tracer.reset();

    Vec2 axis{{1.0, 0.0}};
    auto c_shape = polygon_c_shape_three_parts();
    auto trapped = polygon_from_quad({{4, 4}, {6, 4}, {6, 6}, {4, 6}});
    auto donut = polygon_donut_with_square_hole();
    auto core = polygon_from_quad({{24, 4}, {26, 4}, {26, 6}, {24, 6}});
    std::vector<Polygon2> polys = {c_shape, trapped, donut, core};
    REQUIRE(find_polygon_intersections<Vec2>(polys, axis, &tracer).empty());
    REQUIRE(tracer.stat_hole_invalidations > 0);
    REQUIRE(tracer.stat_sweep_pairs > 0);
    REQUIRE(tracer.stat_gjk_evals > 0);
}

TEST_CASE("stress intersect TC4 bipartite walls vs bullets", "[poly_intersect][stress]") {
    DebugTracer tracer;
    tracer.reset();

    Vec2 axis{{1.0, 0.0}};
    auto wall_l = polygon_from_quad({{0, 0}, {1, 0}, {1, 10}, {0, 10}});
    auto wall_r = polygon_from_quad({{10, 0}, {11, 0}, {11, 10}, {10, 10}});
    auto bullet1 = polygon_from_quad({{0.5, 5}, {1.5, 5}, {1.5, 6}, {0.5, 6}});
    auto bullet2 = polygon_from_quad({{5, 5}, {6, 5}, {6, 6}, {5, 6}});
    std::vector<Polygon2> walls = {wall_l, wall_r};
    std::vector<Polygon2> bullets = {bullet1, bullet2};
    auto hits = find_polygon_intersections<Vec2>(walls, bullets, axis, &tracer);
    REQUIRE(hits.size() == 1);
    REQUIRE(hits[0] == std::make_pair(0, 0));
    REQUIRE(tracer.saw_penetration_pair(0, 0));
    REQUIRE_FALSE(tracer.saw_any_narrow_pair(0, 1));
    // If these four bodies lived in one array as {wall_l, wall_r, bullet1, bullet2}, this is global (0, 2).
    // In bipartite mode, execute_part_sweep skips pairs with the same group_id, so the two walls are never narrow-phased together.
}
