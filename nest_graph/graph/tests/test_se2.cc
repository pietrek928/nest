#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <cmath>

#include "se2.h"

TEST_CASE("se2 invert compose relative round-trip", "[se2]") {
    const Se2 ref{1.f, 2.f, 0.3f};
    const Se2 t{4.f, -1.f, -0.5f};

    const Se2 rel = se2_relative(ref, t);
    const Se2 back = se2_compose(ref, rel);
    REQUIRE(back.x == Catch::Approx(t.x).margin(1e-5));
    REQUIRE(back.y == Catch::Approx(t.y).margin(1e-5));
    REQUIRE(back.a == Catch::Approx(t.a).margin(1e-5));

    const Se2 id = se2_compose(ref, se2_invert(ref));
    REQUIRE(id.x == Catch::Approx(0.f).margin(1e-5));
    REQUIRE(id.y == Catch::Approx(0.f).margin(1e-5));
    REQUIRE(id.a == Catch::Approx(0.f).margin(1e-5));
}

TEST_CASE("se2 matches python utils golden", "[se2]") {
    // Golden from nest_graph.utils: ref=(10,0,pi/2), t=(10,5,pi/2) → relative ≈ (5,0,0)
    const float half_pi = 0.5f * static_cast<float>(M_PI);
    const Se2 ref{10.f, 0.f, half_pi};
    const Se2 t{10.f, 5.f, half_pi};
    const Se2 rel = se2_relative(ref, t);
    REQUIRE(rel.x == Catch::Approx(5.f).margin(1e-4));
    REQUIRE(rel.y == Catch::Approx(0.f).margin(1e-4));
    REQUIRE(rel.a == Catch::Approx(0.f).margin(1e-4));
}

TEST_CASE("se2_key3 translation invariant for relative", "[se2]") {
    const Se2 ref_a{0.f, 0.f, 0.f};
    const Se2 t_a{1.23456f, -2.34567f, 0.111f};
    const Se2 ref_b{10.f, 20.f, 0.f};
    const Se2 t_b = se2_compose(ref_b, se2_relative(ref_a, t_a));
    const auto k1 = se2_key3(se2_relative(ref_a, t_a));
    const auto k2 = se2_key3(se2_relative(ref_b, t_b));
    REQUIRE(k1 == k2);
}
