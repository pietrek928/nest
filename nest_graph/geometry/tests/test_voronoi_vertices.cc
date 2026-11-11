#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include "geometry/common/voronoi_vertices.h"

TEST_CASE("voronoi_vertices: n<3 empty", "[voronoi]") {
    REQUIRE(voronoi_vertices({}).empty());
    REQUIRE(voronoi_vertices({{0.0, 0.0}}).empty());
    REQUIRE(voronoi_vertices({{0.0, 0.0}, {1.0, 0.0}}).empty());
}

TEST_CASE("voronoi_vertices: collinear empty", "[voronoi]") {
    auto v = voronoi_vertices({{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}, {3.0, 0.0}});
    REQUIRE(v.empty());
}

TEST_CASE("voronoi_vertices: near-collinear slanted empty", "[voronoi]") {
    auto v = voronoi_vertices(
        {{0.0, 0.0}, {1.0, 1e-14}, {2.0, 2e-14}, {3.0, 3e-14}});
    REQUIRE(v.empty());
}

TEST_CASE("voronoi_vertices: all identical empty", "[voronoi]") {
    auto v = voronoi_vertices({{1.0, 1.0}, {1.0, 1.0}, {1.0, 1.0}});
    REQUIRE(v.empty());
}

TEST_CASE("voronoi_vertices: equilateral circumcenter", "[voronoi]") {
    const double s = std::sqrt(3.0) / 2.0;
    auto v = voronoi_vertices({{0.0, 0.0}, {1.0, 0.0}, {0.5, s}});
    REQUIRE(v.size() == 1);
    REQUIRE(v[0].first == Catch::Approx(0.5).margin(1e-9));
    REQUIRE(v[0].second == Catch::Approx(s / 3.0).margin(1e-9));
}

TEST_CASE("voronoi_vertices: square cocircular one center", "[voronoi]") {
    auto v = voronoi_vertices({{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}});
    REQUIRE(v.size() == 1);
    REQUIRE(v[0].first == Catch::Approx(0.5).margin(1e-9));
    REQUIRE(v[0].second == Catch::Approx(0.5).margin(1e-9));
}

TEST_CASE("voronoi_vertices: 6x6 lattice unique count", "[voronoi]") {
    std::vector<std::pair<double, double>> sites;
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            sites.emplace_back(static_cast<double>(i), static_cast<double>(j));
        }
    }
    auto v = voronoi_vertices(sites);
    // Interior dual of unit lattice: 5x5 = 25 unique circumcenters.
    REQUIRE(v.size() == 25);
}

TEST_CASE("voronoi_vertices: closing duplicate same as deduped", "[voronoi]") {
    auto a = voronoi_vertices({{0.0, 0.0}, {1.0, 0.0}, {0.5, 0.8}, {0.0, 0.0}});
    auto b = voronoi_vertices({{0.0, 0.0}, {1.0, 0.0}, {0.5, 0.8}});
    REQUIRE(a.size() == b.size());
    for (std::size_t i = 0; i < a.size(); ++i) {
        REQUIRE(a[i].first == Catch::Approx(b[i].first).margin(1e-9));
        REQUIRE(a[i].second == Catch::Approx(b[i].second).margin(1e-9));
    }
}

TEST_CASE("voronoi_vertices: rectangle densify sites finite", "[voronoi]") {
    std::vector<std::pair<double, double>> sites;
    for (double x = 0.0; x <= 2.0 + 1e-15; x += 0.25) {
        sites.emplace_back(x, 0.0);
        sites.emplace_back(x, 1.0);
    }
    for (double y = 0.25; y < 1.0; y += 0.25) {
        sites.emplace_back(0.0, y);
        sites.emplace_back(2.0, y);
    }
    auto v = voronoi_vertices(sites);
    for (const auto &p : v) {
        REQUIRE(std::isfinite(p.first));
        REQUIRE(std::isfinite(p.second));
        REQUIRE(std::fabs(p.first) < 1e6);
        REQUIRE(std::fabs(p.second) < 1e6);
    }
}

TEST_CASE("voronoi_vertices: translate 1e6 shifts only", "[voronoi]") {
    auto base = voronoi_vertices({{0.0, 0.0}, {1.0, 0.0}, {0.5, 0.8}});
    const double sh = 1e6;
    auto shifted = voronoi_vertices(
        {{sh, sh}, {1.0 + sh, sh}, {0.5 + sh, 0.8 + sh}});
    REQUIRE(base.size() == shifted.size());
    REQUIRE(base.size() >= 1);
    for (std::size_t i = 0; i < base.size(); ++i) {
        REQUIRE(shifted[i].first == Catch::Approx(base[i].first + sh).margin(1e-6));
        REQUIRE(shifted[i].second == Catch::Approx(base[i].second + sh).margin(1e-6));
    }
}

TEST_CASE("voronoi_vertices: all-negative sites", "[voronoi]") {
    const double s = std::sqrt(3.0) / 2.0;
    auto v = voronoi_vertices(
        {{-100.0, -100.0}, {-99.0, -100.0}, {-99.5, -100.0 + s}});
    REQUIRE(v.size() == 1);
    REQUIRE(v[0].first == Catch::Approx(-99.5).margin(1e-9));
    REQUIRE(v[0].second == Catch::Approx(-100.0 + s / 3.0).margin(1e-9));
}

TEST_CASE("voronoi_vertices: NaN Inf sites ignored", "[voronoi]") {
    const auto clean = voronoi_vertices(
        {{0.0, 0.0}, {1.0, 0.0}, {0.5, 0.8}});
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    auto dirty = voronoi_vertices(
        {{nan, 0.0},
         {0.0, 0.0},
         {1.0, 0.0},
         {inf, 1.0},
         {0.5, 0.8},
         {0.0, -inf}});
    REQUIRE(dirty.size() == clean.size());
    for (std::size_t i = 0; i < clean.size(); ++i) {
        REQUIRE(dirty[i].first == Catch::Approx(clean[i].first).margin(1e-9));
        REQUIRE(dirty[i].second == Catch::Approx(clean[i].second).margin(1e-9));
    }
}

TEST_CASE("voronoi_vertices: near-dup lex-split deduped", "[voronoi]") {
    // Two near-duplicate centers that would sort apart by y if x equal within eps.
    std::vector<std::pair<double, double>> sites = {
        {0.0, 0.0},
        {2.0, 0.0},
        {1.0, 1.73205080757},
        {0.0, 2.0},
        {2.0, 2.0},
        {1.0, 0.267949},
    };
    auto v = voronoi_vertices(sites);
    for (std::size_t i = 1; i < v.size(); ++i) {
        const double dx = v[i].first - v[i - 1].first;
        const double dy = v[i].second - v[i - 1].second;
        const bool far = (dx * dx + dy * dy) > 1e-20;
        REQUIRE(far);
    }
}

TEST_CASE("voronoi_vertices: shuffled sites same lex", "[voronoi]") {
    std::vector<std::pair<double, double>> a = {
        {0.0, 0.0}, {2.0, 0.0}, {2.0, 2.0}, {0.0, 2.0}, {1.0, 1.0}};
    std::vector<std::pair<double, double>> b = {
        {1.0, 1.0}, {0.0, 2.0}, {2.0, 2.0}, {0.0, 0.0}, {2.0, 0.0}};
    REQUIRE(voronoi_vertices(a) == voronoi_vertices(b));
}

TEST_CASE("voronoi_vertices: identical calls same lex order", "[voronoi]") {
    const std::vector<std::pair<double, double>> sites = {
        {0.0, 0.0},
        {2.0, 0.0},
        {2.0, 2.0},
        {0.0, 2.0},
        {1.0, 1.0},
    };
    auto a = voronoi_vertices(sites);
    auto b = voronoi_vertices(sites);
    REQUIRE(a == b);
    for (std::size_t i = 1; i < a.size(); ++i) {
        const bool ordered =
            (a[i - 1].first < a[i].first)
            || (a[i - 1].first == a[i].first && a[i - 1].second <= a[i].second);
        REQUIRE(ordered);
    }
}

TEST_CASE("voronoi_vertices: cap too small returns 0 untouched", "[voronoi]") {
    const double sites[] = {0.0, 0.0, 1.0, 0.0, 0.5, 0.8};
    VoronoiScratch scratch;
    double out[2] = {42.0, 43.0};  // undersized vs 1 unique center (needs 2)
    // Force undersize after we know need: first call with tiny cap.
    const std::size_t n = voronoi_vertices_xy(sites, 6, scratch, out, 0);
    REQUIRE(n == 0);
    REQUIRE(out[0] == 42.0);
    REQUIRE(out[1] == 43.0);
}
