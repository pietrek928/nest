#include <cmath>
#include <random>
#include <stdexcept>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <circle.h>
#include <vec.h>

using Vec2 = Vec<2, double>;
using Circle2 = Circle<Vec2>;

namespace {

bool circle_contains_points(const Circle2& c, const std::vector<Vec2>& pts, double tol = 1e-7) {
    for (const auto& p : pts) {
        if (!c.is_inside(p, tol)) {
            return false;
        }
    }
    return true;
}

void expect_circle_near(const Circle2& a, const Circle2& b, double tol = 1e-9) {
    REQUIRE(a.center()[0] == Catch::Approx(b.center()[0]).margin(tol));
    REQUIRE(a.center()[1] == Catch::Approx(b.center()[1]).margin(tol));
    REQUIRE(a.square_radius() == Catch::Approx(b.square_radius()).margin(tol));
}

} // namespace

TEST_CASE("Circle default and explicit ctor accessors", "[circle]") {
    Circle2 d;
    REQUIRE(d.square_radius() == Catch::Approx(0.0));
    REQUIRE(d.is_inside(Vec2{{0.0, 0.0}}));

    Circle2 c(Vec2{{3.0, -2.0}}, 25.0);
    REQUIRE(c.center()[0] == Catch::Approx(3.0));
    REQUIRE(c.center()[1] == Catch::Approx(-2.0));
    REQUIRE(c.square_radius() == Catch::Approx(25.0));
}

TEST_CASE("Circle::from two points diameter axis-aligned", "[circle]") {
    Vec2 A{{0.0, 0.0}};
    Vec2 B{{4.0, 0.0}};
    Circle2 c = Circle2::from(A, B);
    REQUIRE(c.center()[0] == Catch::Approx(2.0));
    REQUIRE(c.center()[1] == Catch::Approx(0.0));
    REQUIRE(c.square_radius() == Catch::Approx(4.0));
    REQUIRE(c.is_inside(A));
    REQUIRE(c.is_inside(B));
}

TEST_CASE("Circle::from two points diagonal", "[circle]") {
    Vec2 A{{1.0, 1.0}};
    Vec2 B{{5.0, 4.0}};
    Circle2 c = Circle2::from(A, B);
    Vec2 mid{{3.0, 2.5}};
    REQUIRE((c.center()[0] - mid[0]) == Catch::Approx(0.0).margin(1e-12));
    REQUIRE((c.center()[1] - mid[1]) == Catch::Approx(0.0).margin(1e-12));
    double expected_rsq = static_cast<double>((B - A).len_sq()) * 0.25;
    REQUIRE(c.square_radius() == Catch::Approx(expected_rsq));
    REQUIRE((A - c.center()).len_sq() == Catch::Approx(c.square_radius()).margin(1e-9));
    REQUIRE((B - c.center()).len_sq() == Catch::Approx(c.square_radius()).margin(1e-9));
}

TEST_CASE("Circle::from three points right triangle circumcircle", "[circle]") {
    Vec2 A{{0.0, 0.0}};
    Vec2 B{{2.0, 0.0}};
    Vec2 C{{0.0, 2.0}};
    Circle2 c = Circle2::from(A, B, C);
    REQUIRE(c.center()[0] == Catch::Approx(1.0));
    REQUIRE(c.center()[1] == Catch::Approx(1.0));
    REQUIRE(c.square_radius() == Catch::Approx(2.0));
    REQUIRE(circle_contains_points(c, {A, B, C}));
}

TEST_CASE("Circle::from three points collinear diameter fallback", "[circle]") {
    Vec2 A{{0.0, 0.0}};
    Vec2 B{{1.0, 0.0}};
    Vec2 C{{2.0, 0.0}};
    Circle2 c = Circle2::from(A, B, C);
    REQUIRE(circle_contains_points(c, {A, B, C}, 1e-6));
    REQUIRE(c.center()[1] == Catch::Approx(0.0).margin(1e-9));
    REQUIRE(c.square_radius() >= Catch::Approx(1.0).margin(1e-9));
}

TEST_CASE("Circle::from pointer overload matches factories", "[circle]") {
    Vec2 tri[] = {{{0.0, 0.0}}, {{2.0, 0.0}}, {{0.0, 2.0}}};
    Circle2 c3 = Circle2::from(tri, 3);
    Circle2 ref = Circle2::from(tri[0], tri[1], tri[2]);
    expect_circle_near(c3, ref);

    Vec2 seg[] = {{{0.0, 0.0}}, {{4.0, 0.0}}};
    Circle2 c2 = Circle2::from(seg, 2);
    expect_circle_near(c2, Circle2::from(seg[0], seg[1]));

    Vec2 one[] = {{{5.0, -3.0}}};
    Circle2 c1 = Circle2::from(one, 1);
    REQUIRE(c1.center()[0] == Catch::Approx(5.0));
    REQUIRE(c1.center()[1] == Catch::Approx(-3.0));
    REQUIRE(c1.square_radius() == Catch::Approx(0.0));
}

TEST_CASE("Circle::from invalid n throws", "[circle]") {
    Vec2 p{{0.0, 0.0}};
    REQUIRE_THROWS_AS(Circle2::from(&p, 0), std::invalid_argument);
    Vec2 pts[] = {{{0.0, 0.0}}, {{1.0, 0.0}}, {{0.0, 1.0}}, {{1.0, 1.0}}};
    REQUIRE_THROWS_AS(Circle2::from(pts, 4), std::invalid_argument);
}

TEST_CASE("Circle is_inside interior boundary exterior", "[circle]") {
    Circle2 c(Vec2{{1.0, 2.0}}, 9.0);
    REQUIRE(c.is_inside(Vec2{{1.0, 2.0}}));
    REQUIRE(c.is_inside(Vec2{{4.0, 2.0}})); // distance 3, r_sq=9 boundary
    REQUIRE_FALSE(c.is_inside(Vec2{{4.0 + 1e-4}, {2.0}}));
}

TEST_CASE("compute_exact_bounding_circle degenerate counts", "[circle]") {
    std::mt19937 gen(42);
    Circle2 empty = compute_exact_bounding_circle<Vec2>(nullptr, 0, gen);
    REQUIRE(empty.square_radius() == Catch::Approx(0.0));

    Vec2 one[] = {{{3.0, -4.0}}};
    Circle2 single = compute_exact_bounding_circle<Vec2>(one, 1, gen);
    REQUIRE(single.center()[0] == Catch::Approx(3.0));
    REQUIRE(single.center()[1] == Catch::Approx(-4.0));
    REQUIRE(single.square_radius() == Catch::Approx(0.0));

    Vec2 seg[] = {{{0.0, 0.0}}, {{4.0, 0.0}}};
    Circle2 dia = compute_exact_bounding_circle<Vec2>(seg, 2, gen);
    expect_circle_near(dia, Circle2::from(seg[0], seg[1]));
}

TEST_CASE("compute_exact_bounding_circle square", "[circle]") {
    std::vector<Vec2> sq = {
        {{0.0, 0.0}}, {{2.0, 0.0}}, {{2.0, 2.0}}, {{0.0, 2.0}},
    };
    std::mt19937 g0(0);
    std::mt19937 g999(999);
    Circle2 c0 = compute_exact_bounding_circle<Vec2>(sq.data(), static_cast<int>(sq.size()), g0);
    Circle2 c1 = compute_exact_bounding_circle<Vec2>(sq.data(), static_cast<int>(sq.size()), g999);
    expect_circle_near(c0, c1);
    REQUIRE(c0.center()[0] == Catch::Approx(1.0));
    REQUIRE(c0.center()[1] == Catch::Approx(1.0));
    REQUIRE(c0.square_radius() == Catch::Approx(2.0));
    REQUIRE(circle_contains_points(c0, sq));
}

TEST_CASE("compute_exact_bounding_circle collinear cloud", "[circle]") {
    std::vector<Vec2> line;
    for (int i = 0; i <= 8; ++i) {
        line.push_back(Vec2{{static_cast<double>(i), 0.0}});
    }
    std::mt19937 gen(7);
    Circle2 c = compute_exact_bounding_circle<Vec2>(
        line.data(), static_cast<int>(line.size()), gen);
    expect_circle_near(c, Circle2::from(line.front(), line.back()));
    REQUIRE(circle_contains_points(c, line, 1e-6));
}

TEST_CASE("compute_exact_bounding_circle matches three-point circumcircle", "[circle]") {
    Vec2 tri[] = {{{0.0, 0.0}}, {{2.0, 0.0}}, {{0.0, 2.0}}};
    std::mt19937 gen(123);
    Circle2 welzl = compute_exact_bounding_circle<Vec2>(tri, 3, gen);
    Circle2 circum = Circle2::from(tri[0], tri[1], tri[2]);
    expect_circle_near(welzl, circum, 1e-8);
}
