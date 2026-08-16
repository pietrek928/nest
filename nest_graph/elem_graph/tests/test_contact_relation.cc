#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <vector>

#include "contact_relation.h"

#include "geometry/tests/geometry_test_helpers.h"

using Vec2 = PolyTestVec2;
using Solid = PolyTestSolidGeometry2;

TEST_CASE("contact: kissing squares yield one edge", "[contact_relation]") {
    // Unit squares: [0,1]x[0,1] and [1,2]x[0,1] — share edge (touch).
    Solid a = polygon_from_quad({{0, 0}, {1, 0}, {1, 1}, {0, 1}});
    Solid b = polygon_from_quad({{1, 0}, {2, 0}, {2, 1}, {1, 1}});
    std::vector<Solid> geoms{a, b};
    std::vector<Se2> poses{{0.f, 0.f, 0.f}, {1.f, 0.f, 0.f}};
    std::vector<int32_t> gids{1, 2};
    const double gap = 0.1;
    auto edges = build_contact_relations(geoms, poses, gids, gap);
    REQUIRE(edges.size() == 1);
    REQUIRE(edges[0].contact_score == Catch::Approx(1.f).margin(1e-3));
    REQUIRE(edges[0].gci > 0.f);
    REQUIRE(edges[0].relative_pose.x == Catch::Approx(1.f).margin(1e-4));
}

TEST_CASE("contact: far squares yield zero edges", "[contact_relation]") {
    Solid a = polygon_from_quad({{0, 0}, {1, 0}, {1, 1}, {0, 1}});
    Solid b = polygon_from_quad({{10, 0}, {11, 0}, {11, 1}, {10, 1}});
    std::vector<Solid> geoms{a, b};
    std::vector<Se2> poses{{0.f, 0.f, 0.f}, {10.f, 0.f, 0.f}};
    std::vector<int32_t> gids{1, 2};
    auto edges = build_contact_relations(geoms, poses, gids, 0.1);
    REQUIRE(edges.empty());
}

TEST_CASE("contact: translated pair same relative key", "[contact_relation]") {
    Solid a0 = polygon_from_quad({{0, 0}, {1, 0}, {1, 1}, {0, 1}});
    Solid b0 = polygon_from_quad({{1.05, 0}, {2.05, 0}, {2.05, 1}, {1.05, 1}});
    Solid a1 = a0.translate(Vec2({5.0, 3.0}));
    Solid b1 = b0.translate(Vec2({5.0, 3.0}));
    std::vector<Se2> poses0{{0.f, 0.f, 0.f}, {1.05f, 0.f, 0.f}};
    std::vector<Se2> poses1{{5.f, 3.f, 0.f}, {6.05f, 3.f, 0.f}};
    std::vector<int32_t> gids{3, 4};
    const double gap = 0.1;
    auto e0 = build_contact_relations(std::vector<Solid>{a0, b0}, poses0, gids, gap);
    auto e1 = build_contact_relations(std::vector<Solid>{a1, b1}, poses1, gids, gap);
    REQUIRE(e0.size() == 1);
    REQUIRE(e1.size() == 1);
    REQUIRE(se2_key3(e0[0].relative_pose) == se2_key3(e1[0].relative_pose));
}

TEST_CASE("gci: denser pair scores higher than loose", "[contact_relation]") {
    Solid a = polygon_from_quad({{0, 0}, {2, 0}, {2, 2}, {0, 2}});
    Solid tight = polygon_from_quad({{2, 0}, {4, 0}, {4, 2}, {2, 2}});
    Solid loose = polygon_from_quad({{2.15, 0}, {4.15, 0}, {4.15, 2}, {2.15, 2}});
    const double gap = 0.2;
    std::vector<int32_t> gids{1, 2};
    auto e_tight = build_contact_relations(
        std::vector<Solid>{a, tight},
        std::vector<Se2>{{0.f, 0.f, 0.f}, {2.f, 0.f, 0.f}},
        gids,
        gap);
    auto e_loose = build_contact_relations(
        std::vector<Solid>{a, loose},
        std::vector<Se2>{{0.f, 0.f, 0.f}, {2.15f, 0.f, 0.f}},
        gids,
        gap);
    REQUIRE(e_tight.size() == 1);
    REQUIRE(e_loose.size() == 1);
    REQUIRE(e_tight[0].gci >= e_loose[0].gci - 1e-5f);
    REQUIRE(e_tight[0].contact_score >= e_loose[0].contact_score - 1e-5f);
}
