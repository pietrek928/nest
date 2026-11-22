#include <cmath>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "geometry/distance/polygon_distance.h"
#include "geometry/distance/static_collision_scene.h"
#include "geometry/guide/polygon_cast.h"
#include "tests/geometry_test_helpers.h"

using Vec2 = PolyTestVec2;
using PlaneGeometry2 = PolyTestPlaneGeometry2;

TEST_CASE("distance sweep: swallowed containment mid-list", "[distance][containment][gap]") {
    // Three pairs: A/B far, C fully inside D, E/F touching gap.
    // Intermediate swallowed pair must still report intersect.
    auto outer = polygon_from_quad({{0, 0}, {10, 0}, {10, 10}, {0, 10}});
    auto inner = polygon_from_quad({{3, 3}, {5, 3}, {5, 5}, {3, 5}});
    auto left = polygon_from_quad({{-20, 0}, {-18, 0}, {-18, 2}, {-20, 2}});
    auto right = polygon_from_quad({{20, 0}, {22, 0}, {22, 2}, {20, 2}});

    std::vector<PlaneGeometry2> polys = {left, right, outer, inner};
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

TEST_CASE(
    "StaticCollisionScene: multi-part obstacle margin catches Penetrating",
    "[distance][validity][gap]"
) {
    // Many additive convex pieces under one PlaneGeometry (pack-union style).
    // Distance-only GJK can miss overlaps; Scene clearance must still violate.
    PlaneGeometry2 multi;
    std::mt19937 gen(7);
    auto add_quad = [&](double x0, double y0, double x1, double y1) {
        std::vector<Vec2> ring = {
            Vec2{{x0, y0}}, Vec2{{x1, y0}}, Vec2{{x1, y1}}, Vec2{{x0, y1}},
        };
        multi.add_boundary_ring(ring);
        ring.push_back(ring.front());
        multi.append_line_poly(ring.data(), static_cast<int>(ring.size()), gen);
    };
    for (int i = 0; i < 20; ++i) {
        const double x = 0.05 + 0.2 * (i % 5);
        const double y = 0.05 + 0.2 * (i / 5);
        add_quad(x, y, x + 0.08, y + 0.08);
    }
    // Overlap target near (0.55, 0.55).
    add_quad(0.50, 0.50, 0.60, 0.60);
    multi.finalize(gen);

    auto probe = polygon_from_quad({{0.52, 0.52}, {0.58, 0.52}, {0.58, 0.58}, {0.52, 0.58}});
    StaticCollisionScene<Vec2> scene;
    scene.build({multi}, 0.5);

    REQUIRE(scene.query_any_violation(probe, 0.0, 0.0));
    const double md = 0.001;
    const double margin = md + std::max(1e-6, md * 0.05);
    REQUIRE(scene.query_any_violation(probe, margin * margin, margin));
    REQUIRE_FALSE(scene.is_valid_placement(probe, margin * margin));
}

TEST_CASE("StaticCollisionScene: query_elements scratch reuse parity", "[distance][validity]") {
    auto obstacle = polygon_from_quad({{0, 0}, {4, 0}, {4, 4}, {0, 4}});
    auto part = polygon_from_quad({{0, 0}, {1, 0}, {1, 1}, {0, 1}});
    StaticCollisionScene<Vec2> scene;
    scene.build({obstacle}, 0.5);

    auto a = part.translate(Vec2{{6.0, 1.0}});
    auto b = part.translate(Vec2{{0.5, 0.5}});
    const auto hits_a1 = scene.query_placed(a);
    const auto hits_a2 = scene.query_placed(a);
    REQUIRE(hits_a1.size() == hits_a2.size());
    for (size_t i = 0; i < hits_a1.size(); ++i) {
        REQUIRE(hits_a1[i].intersect == hits_a2[i].intersect);
        REQUIRE(hits_a1[i].distance_sq == Catch::Approx(hits_a2[i].distance_sq).margin(1e-12));
    }
    REQUIRE(scene.is_valid_placement(a, 0.0));
    REQUIRE_FALSE(scene.is_valid_placement(b, 0.0));
    // Scratch reused across mixed query kinds.
    REQUIRE(scene.query_any_violation(b, 0.0, 0.0));
    REQUIRE_FALSE(scene.query_any_violation(a, 0.0, 0.0));
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

    std::vector<PlaneGeometry2> polys = {probe, shell};
    // Cast downward toward the hole / shell material.
    auto hit = find_closest_polygon_cast<Vec2>(0, polys, Vec2{{0.0, -1.0}}, 10.0);
    REQUIRE(hit.intersects_path);
    REQUIRE(hit.t_entry >= 0.0);
    REQUIRE(hit.t_entry < 10.0);
    // Must hit an additive shell part, not a subtractive hole ring.
    REQUIRE_FALSE(shell.line_parts[static_cast<size_t>(hit.partB_idx)].is_subtractive);
}
