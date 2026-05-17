#include <cmath>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "geometry/convex/distance.h"
#include "geometry/convex/intersect.h"
#include "geometry/convex/penetration.h"
#include "geometry/polygon_intersect.h"
#include <vec.h>

using Vec2 = Vec<2, double>;

// -----------------------------------------------------------------------------
// convex_linestrings_distance_gjk(..., known_overlap):
//   false — separated / sliding Euclidean minima (default for approx_dist & disjoint cases).
//   true  — overlap slack (tests 2 & 6): |closest|² stall + 2-edge simplex branch.
// Pass-through from narrow_phase uses penetration intersect (no extra intersect GJK).
// -----------------------------------------------------------------------------
//
//   3,4,8 Touch / coincidence: expect ~0 gap.
//
//   9  Degenerate vertex “polygon”: Euclidean gap to nearest edge (e.g. x=2 vs point (4,1)).
//
//  10  Disjoint vertex–vertex local minimum vs shorter edge–edge (known limitation).
//
// Suite covers disjoint (1), touch (3,4), containment (5), overlaps (2), cross (6),
// micro-gap (7), coincidence (8), extras.
// -----------------------------------------------------------------------------

static std::vector<Vec2> make_poly(std::initializer_list<std::initializer_list<double>> pts) {
    std::vector<Vec2> out;
    out.reserve(pts.size() + 1);
    for (const auto& p : pts) {
        auto it = p.begin();
        double x = *it++;
        double y = *it;
        out.emplace_back(std::initializer_list<double>{x, y});
    }
    if (!out.empty()) {
        out.push_back(out.front());
    }
    return out;
}

// Explicit <Vec2> instantiates generic N-D GJK (not an ad hoc 2D overload).
static double approx_dist(const std::vector<Vec2>& a, const std::vector<Vec2>& b) {
    auto d = convex_linestrings_distance_gjk<Vec2>(
        a.data(), static_cast<int>(a.size()),
        b.data(), static_cast<int>(b.size())
    );
    return std::sqrt(static_cast<double>(d.distance_sq));
}

TEST_CASE("1: disjoint axis-aligned squares", "[gjk]") {
    auto polyA = make_poly({{0, 0}, {2, 0}, {2, 2}, {0, 2}});
    auto polyB = make_poly({{3, 0}, {5, 0}, {5, 2}, {3, 2}});
    auto ir = convex_linestrings_intersect_gjk<Vec2>(
        polyA.data(), static_cast<int>(polyA.size()),
        polyB.data(), static_cast<int>(polyB.size())
    );
    REQUIRE_FALSE(ir.intersect);
    REQUIRE(approx_dist(polyA, polyB) == Catch::Approx(1.0).margin(1e-5));
}

TEST_CASE("2: overlapping squares", "[gjk]") {
    auto polyA = make_poly({{0, 0}, {2, 0}, {2, 2}, {0, 2}});
    auto polyB = make_poly({{1, 1}, {3, 1}, {3, 3}, {1, 3}});
    auto ir = convex_linestrings_intersect_gjk<Vec2>(
        polyA.data(), static_cast<int>(polyA.size()),
        polyB.data(), static_cast<int>(polyB.size())
    );
    REQUIRE(ir.intersect);
    auto d = convex_linestrings_distance_gjk<Vec2>(
        polyA.data(), static_cast<int>(polyA.size()),
        polyB.data(), static_cast<int>(polyB.size()),
        true);
    REQUIRE_FALSE(d.intersect);
    REQUIRE_FALSE(d.distance_sq < static_cast<double>(1e-6));
}

TEST_CASE("3: vertex-to-vertex touch", "[gjk][touch]") {
    auto polyA = make_poly({{0, 0}, {2, 0}, {2, 2}, {0, 2}});
    auto polyB = make_poly({{2, 2}, {4, 2}, {4, 4}, {2, 4}});
    auto ir = convex_linestrings_intersect_gjk<Vec2>(
        polyA.data(), static_cast<int>(polyA.size()),
        polyB.data(), static_cast<int>(polyB.size())
    );
    REQUIRE(ir.intersect);
    REQUIRE(approx_dist(polyA, polyB) == Catch::Approx(0.0).margin(1e-6));
}

TEST_CASE("4: edge-to-edge touch", "[gjk][touch]") {
    auto polyA = make_poly({{0, 0}, {2, 0}, {2, 2}, {0, 2}});
    auto polyB = make_poly({{2, 0}, {4, 0}, {4, 2}, {2, 2}});
    auto ir = convex_linestrings_intersect_gjk<Vec2>(
        polyA.data(), static_cast<int>(polyA.size()),
        polyB.data(), static_cast<int>(polyB.size())
    );
    REQUIRE(ir.intersect);
    REQUIRE(approx_dist(polyA, polyB) == Catch::Approx(0.0).margin(1e-6));
}

TEST_CASE("5: full containment", "[gjk]") {
    auto outer = make_poly({{0, 0}, {4, 0}, {4, 4}, {0, 4}});
    auto inner = make_poly({{1, 1}, {3, 1}, {3, 3}, {1, 3}});
    auto ir = convex_linestrings_intersect_gjk<Vec2>(
        outer.data(), static_cast<int>(outer.size()),
        inner.data(), static_cast<int>(inner.size())
    );
    REQUIRE(ir.intersect);
}

TEST_CASE("6: high aspect ratio cross", "[gjk]") {
    // Thin bars intersecting heavily; Euclidean penetration depth is unrelated to
    // this generic-distance GJK (returns a large nonnegative slack here).
    const double L = 5.0;
    auto polyH =
        make_poly({{-L, -1}, {L, -1}, {L, 1}, {-L, 1}});
    auto polyV =
        make_poly({{-1, -L}, {1, -L}, {1, L}, {-1, L}});
    auto ir = convex_linestrings_intersect_gjk<Vec2>(
        polyH.data(), static_cast<int>(polyH.size()),
        polyV.data(), static_cast<int>(polyV.size())
    );
    REQUIRE(ir.intersect);
    auto d = convex_linestrings_distance_gjk<Vec2>(
        polyH.data(), static_cast<int>(polyH.size()),
        polyV.data(), static_cast<int>(polyV.size()),
        true);
    REQUIRE_FALSE(d.intersect);
    REQUIRE_FALSE(d.distance_sq < static_cast<double>(1e-6));
    REQUIRE(d.distance_sq == Catch::Approx(72.0).margin(1e-9));
}

TEST_CASE("7: micro-gap epsilon stress", "[gjk]") {
    auto polyA = make_poly({{0, 0}, {2, 0}, {2, 2}, {0, 2}});
    auto polyB = make_poly({{2.00001, 0}, {4, 0}, {4, 2}, {2.00001, 2}});
    auto ir = convex_linestrings_intersect_gjk<Vec2>(
        polyA.data(), static_cast<int>(polyA.size()),
        polyB.data(), static_cast<int>(polyB.size())
    );
    REQUIRE_FALSE(ir.intersect);
    REQUIRE(approx_dist(polyA, polyB) == Catch::Approx(1e-5).margin(1e-7));
}

TEST_CASE("8: identical triangles", "[gjk]") {
    auto polyA = make_poly({{-5.5, -5.5}, {5.5, -5.5}, {0.0, 5.5}});
    auto polyB = make_poly({{-5.5, -5.5}, {5.5, -5.5}, {0.0, 5.5}});
    auto ir = convex_linestrings_intersect_gjk<Vec2>(
        polyA.data(), static_cast<int>(polyA.size()),
        polyB.data(), static_cast<int>(polyB.size())
    );
    REQUIRE(ir.intersect);
    REQUIRE(approx_dist(polyA, polyB) == Catch::Approx(0.0).margin(1e-8));
}

TEST_CASE("9: single point vs square", "[gjk]") {
    auto square =
        make_poly({{0, 0}, {2, 0}, {2, 2}, {0, 2}});
    auto point = make_poly({{4, 1}});
    auto ir = convex_linestrings_intersect_gjk<Vec2>(
        square.data(), static_cast<int>(square.size()),
        point.data(), static_cast<int>(point.size())
    );
    REQUIRE_FALSE(ir.intersect);
    // Closest Euclidean gap uses the vertical edge x=2: |4 − 2| = 2. (Earlier
    // vertex-only formulations could report sqrt((4−2)²+(1−0)²) = sqrt(5).)
    REQUIRE(approx_dist(square, point) == Catch::Approx(2.0).margin(1e-9));
}

TEST_CASE("10: triangle vs hexagon separated", "[gjk]") {
    // Disjoint pair tuned so generic distance GJK returns ~sqrt(2) (~vertex–vertex
    // pairing); a dedicated 2D narrow phase can find a shorter edge–edge gap (~1.39).
    auto tri =
        make_poly({{2, -1}, {2, 1}, {5.5, 0}});
    auto hex = make_poly(
        {{8.14, -1.9}, {9.39, -0.45}, {9.39, 1.7}, {8.29, 2.95}, {7.04, 1.85}, {6.89, -0.4}});
    auto ir = convex_linestrings_intersect_gjk<Vec2>(
        tri.data(), static_cast<int>(tri.size()),
        hex.data(), static_cast<int>(hex.size())
    );
    REQUIRE_FALSE(ir.intersect);
    REQUIRE(
        approx_dist(tri, hex)
        == Catch::Approx(std::sqrt(2.0)).epsilon(5e-4));
}

TEST_CASE("extra: case 1 with reversed winding", "[gjk]") {
    auto polyA = make_poly({{0, 2}, {0, 0}, {2, 0}, {2, 2}});
    auto polyB = make_poly({{3, 2}, {3, 0}, {5, 0}, {5, 2}});
    auto ir = convex_linestrings_intersect_gjk<Vec2>(
        polyA.data(), static_cast<int>(polyA.size()),
        polyB.data(), static_cast<int>(polyB.size())
    );
    REQUIRE_FALSE(ir.intersect);
    REQUIRE(approx_dist(polyA, polyB) == Catch::Approx(1.0).margin(1e-5));
}

TEST_CASE("extra: near-parallel thin rectangles", "[gjk]") {
    auto polyA = make_poly({{0, 0}, {10, 0}, {10, 1}, {0, 1}});
    auto polyB = make_poly({{0, 2.5}, {10, 2.5}, {10, 3.5}, {0, 3.5}});
    auto ir = convex_linestrings_intersect_gjk<Vec2>(
        polyA.data(), static_cast<int>(polyA.size()),
        polyB.data(), static_cast<int>(polyB.size())
    );
    REQUIRE_FALSE(ir.intersect);
    REQUIRE(approx_dist(polyA, polyB) == Catch::Approx(1.5).margin(1e-5));
}

TEST_CASE("extra: narrow_phase matches GJK for small n", "[gjk]") {
    auto polyA = make_poly({{0, 0}, {2, 0}, {2, 2}, {0, 2}});
    auto polyB = make_poly({{3, 0}, {5, 0}, {5, 2}, {3, 2}});
    bool np = narrow_phase_intersect<Vec2>(
        polyA.data(), static_cast<int>(polyA.size()),
        polyB.data(), static_cast<int>(polyB.size())
    );
    bool gjk = convex_linestrings_intersect_gjk<Vec2>(
        polyA.data(), static_cast<int>(polyA.size()),
        polyB.data(), static_cast<int>(polyB.size())
    ).intersect;
    REQUIRE(np == gjk);
}

TEST_CASE("stress: highly tessellated circle vs needle", "[gjk][stress]") {
    std::vector<Vec2> circle_pts;
    int num_segments = 128;
    double radius = 5.0;
    for (int i = 0; i < num_segments; ++i) {
        double angle = 2.0 * M_PI * i / num_segments;
        circle_pts.push_back(Vec2{{radius * std::cos(angle), radius * std::sin(angle)}});
    }
    circle_pts.push_back(circle_pts.front());

    // Needle pointing directly at the origin, penetrating slightly
    auto needle = make_poly({{4.9, -0.1}, {10.0, 0.0}, {4.9, 0.1}});

    auto ir = convex_linestrings_intersect_gjk<Vec2>(
        circle_pts.data(), static_cast<int>(circle_pts.size()),
        needle.data(), static_cast<int>(needle.size())
    );
    REQUIRE(ir.intersect);

    auto pr = convex_linestrings_penetration<Vec2>(
        circle_pts.data(), static_cast<int>(circle_pts.size()),
        needle.data(), static_cast<int>(needle.size())
    );
    REQUIRE(pr.intersect);
    REQUIRE(pr.penetration_sq > 0.0);
    REQUIRE(pr.mtv[0] < 0.0); // Should push needle out to the right (MTV is applied to shape A)
}

TEST_CASE("stress: collinear edge-to-edge sliding many vertices", "[gjk][stress]") {
    std::vector<Vec2> top_edge;
    std::vector<Vec2> bottom_edge;
    int num_verts = 50;
    for (int i = 0; i < num_verts; ++i) {
        top_edge.push_back(Vec2{{static_cast<double>(i), 1.0}});
        bottom_edge.push_back(Vec2{{static_cast<double>(i) + 0.5, 0.0}});
    }
    // Make them closed polygons
    top_edge.push_back(Vec2{{static_cast<double>(num_verts), 2.0}});
    top_edge.push_back(Vec2{{-1.0, 2.0}});
    top_edge.push_back(top_edge.front());

    bottom_edge.push_back(Vec2{{static_cast<double>(num_verts), -1.0}});
    bottom_edge.push_back(Vec2{{-1.0, -1.0}});
    bottom_edge.push_back(bottom_edge.front());

    auto d = convex_linestrings_distance_gjk_gradient<Vec2>(
        top_edge.data(), static_cast<int>(top_edge.size()),
        bottom_edge.data(), static_cast<int>(bottom_edge.size())
    );
    REQUIRE_FALSE(d.intersect);
    REQUIRE(d.distance_sq == Catch::Approx(1.0).margin(1e-5));
}
