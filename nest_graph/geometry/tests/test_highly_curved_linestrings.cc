#include <catch2/catch_test_macros.hpp>
#include "geometry/intersect/polygon_intersect.h"
#include "geometry/solid/decompose.h"
#include "tests/geometry_test_helpers.h"

using Vec2 = PolyTestVec2;
using SolidGeometry2 = PolyTestSolidGeometry2;

TEST_CASE("highly curved linestrings decomposition and intersection", "[bug][decomposition]") {
    // Shapely reference (scripts/generate_test_refs.py):
    //   spiral.intersects(box) = true
    //   spiral.distance(box) = 0.0
    // This test ensures that highly curved shapes (like spirals) are properly decomposed
    // into segments that turn no more than 90 degrees, preventing GJK support mapping failures.
    
    // A simplified spiral-like shape that turns more than 180 degrees
    std::vector<Vec2> spiral_ring = {
        Vec2{{10.0, 0.0}},
        Vec2{{0.0, 10.0}},
        Vec2{{-10.0, 0.0}},
        Vec2{{0.0, -10.0}},
        Vec2{{8.0, 0.0}},
        Vec2{{0.0, 8.0}},
        Vec2{{-8.0, 0.0}},
        Vec2{{0.0, -8.0}},
        Vec2{{6.0, 0.0}}
    };
    
    SolidGeometry2 spiral_mesh;
    process_boundary_to_convex_segments<Vec2>(spiral_ring, spiral_mesh);
    spiral_mesh.finalize();
    
    // If the >90 degree split logic is working, this should be split into multiple parts
    // (at least 4 parts since it does almost two full turns)
    REQUIRE(spiral_mesh.line_parts.size() >= 4);
    
    // A small box that is completely inside the spiral's bounding box,
    // but intersects one of its inner segments
    std::vector<Vec2> box_ring = {
        Vec2{{5.0, -1.0}},
        Vec2{{7.0, -1.0}},
        Vec2{{7.0, 1.0}},
        Vec2{{5.0, 1.0}}
    };
    
    SolidGeometry2 box_mesh;
    process_boundary_to_convex_segments<Vec2>(box_ring, box_mesh);
    box_mesh.finalize();
    
    // They should intersect
    auto hits = find_polygon_intersections<Vec2>({spiral_mesh, box_mesh});
    REQUIRE(hits.size() > 0);
}
