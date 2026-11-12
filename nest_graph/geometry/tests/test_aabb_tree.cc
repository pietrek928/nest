#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cstddef>
#include <limits>
#include <random>
#include <set>
#include <utility>
#include <vector>

#include "geometry/common/aabb_tree.h"

inline std::vector<double> aabb_test_ltrb_from_boxes(const std::vector<Aabb2>& boxes) {
    std::vector<double> out;
    out.reserve(boxes.size() * 4);
    for (const auto& b : boxes) {
        out.push_back(b.start[0]);
        out.push_back(b.start[1]);
        out.push_back(b.end[0]);
        out.push_back(b.end[1]);
    }
    return out;
}

inline std::set<int> aabb_test_brute_box(
    const std::vector<Aabb2>& boxes,
    double minx,
    double miny,
    double maxx,
    double maxy
) {
    const Aabb2 q = aabb2_from_ltrb(minx, miny, maxx, maxy);
    std::set<int> hit;
    for (std::size_t i = 0; i < boxes.size(); ++i) {
        if (aabb2_intersects(boxes[i], q)) {
            hit.insert(static_cast<int>(i));
        }
    }
    return hit;
}

inline std::set<int> aabb_test_brute_radius(
    const std::vector<Aabb2>& boxes,
    double qx,
    double qy,
    double radius_sq
) {
    std::set<int> hit;
    for (std::size_t i = 0; i < boxes.size(); ++i) {
        if (aabb2_point_dist_sq(boxes[i], qx, qy) <= radius_sq) {
            hit.insert(static_cast<int>(i));
        }
    }
    return hit;
}

inline std::vector<int> aabb_test_brute_knn(
    const std::vector<Aabb2>& boxes,
    double qx,
    double qy,
    std::size_t k
) {
    std::vector<std::pair<double, int>> all;
    all.reserve(boxes.size());
    for (std::size_t i = 0; i < boxes.size(); ++i) {
        all.emplace_back(aabb2_point_dist_sq(boxes[i], qx, qy), static_cast<int>(i));
    }
    std::sort(all.begin(), all.end(), [](const auto& a, const auto& b) {
        if (a.first != b.first) {
            return a.first < b.first;
        }
        return a.second < b.second;
    });
    std::vector<int> out;
    const std::size_t n = std::min(k, all.size());
    out.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
        out.push_back(all[i].second);
    }
    return out;
}

TEST_CASE("aabb_tree: empty rebuild", "[aabb_tree]") {
    AabbTree tree;
    AabbTreeScratch scratch;
    REQUIRE(aabb_tree_rebuild_boxes(nullptr, 0, tree));
    REQUIRE(tree.nodes.empty());
    int out[4] = {-1, -1, -1, -1};
    REQUIRE(aabb_tree_query_box(tree, scratch, 0, 0, 1, 1, out, 4) == 0);
    REQUIRE(aabb_tree_query_radius(tree, scratch, 0, 0, 1.0, out, 4) == 0);
    double dist[4];
    REQUIRE(aabb_tree_knn(tree, scratch, 0, 0, 2, out, dist, 4) == 0);
}

TEST_CASE("aabb_tree: n=1 leaf", "[aabb_tree]") {
    const double ltrb[] = {1.0, 2.0, 3.0, 4.0};
    AabbTree tree;
    AabbTreeScratch scratch;
    REQUIRE(aabb_tree_rebuild_boxes(ltrb, 1, tree));
    REQUIRE_FALSE(tree.nodes.empty());
    int out[4];
    REQUIRE(aabb_tree_query_box(tree, scratch, 0, 0, 10, 10, out, 4) == 1);
    REQUIRE(out[0] == 0);
    REQUIRE(aabb_tree_query_box(tree, scratch, 5, 5, 6, 6, out, 4) == 0);
    REQUIRE(aabb_tree_query_radius(tree, scratch, 2, 3, 0.0, out, 4) == 1);
    double dist[1];
    REQUIRE(aabb_tree_knn(tree, scratch, 0, 0, 1, out, dist, 1) == 1);
    REQUIRE(out[0] == 0);
}

TEST_CASE("aabb_tree: coincident boxes", "[aabb_tree]") {
    std::vector<Aabb2> boxes = {
        aabb2_from_ltrb(0, 0, 1, 1),
        aabb2_from_ltrb(0, 0, 1, 1),
        aabb2_from_ltrb(0, 0, 1, 1),
    };
    auto ltrb = aabb_test_ltrb_from_boxes(boxes);
    AabbTree tree;
    AabbTreeScratch scratch;
    REQUIRE(aabb_tree_rebuild_boxes(ltrb.data(), boxes.size(), tree));
    int out[8];
    const auto n = aabb_tree_query_box(tree, scratch, 0.25, 0.25, 0.75, 0.75, out, 8);
    REQUIRE(n == 3);
    std::set<int> hit(out, out + n);
    REQUIRE(hit == aabb_test_brute_box(boxes, 0.25, 0.25, 0.75, 0.75));
}

TEST_CASE("aabb_tree: collinear point sites", "[aabb_tree]") {
    const double xy[] = {0, 0, 1, 0, 2, 0, 3, 0, 4, 0};
    AabbTree tree;
    AabbTreeScratch scratch;
    REQUIRE(aabb_tree_rebuild_points(xy, 5, tree));
    int out[8];
    double dist[8];
    const auto n = aabb_tree_knn(tree, scratch, 2.1, 0.0, 3, out, dist, 8);
    REQUIRE(n == 3);
    REQUIRE(out[0] == 2);
}

TEST_CASE("aabb_tree: kissing AABBs inclusive", "[aabb_tree]") {
    // Boxes share an edge at x=1 (kiss).
    std::vector<Aabb2> boxes = {
        aabb2_from_ltrb(0, 0, 1, 1),
        aabb2_from_ltrb(1, 0, 2, 1),
    };
    auto ltrb = aabb_test_ltrb_from_boxes(boxes);
    AabbTree tree;
    AabbTreeScratch scratch;
    REQUIRE(aabb_tree_rebuild_boxes(ltrb.data(), boxes.size(), tree));
    int out[4];
    // Query exactly the shared edge strip.
    auto n = aabb_tree_query_box(tree, scratch, 1, 0, 1, 1, out, 4);
    REQUIRE(n == 2);
    // Point on kiss edge: dist_sq == 0 for both.
    n = aabb_tree_query_radius(tree, scratch, 1.0, 0.5, 0.0, out, 4);
    REQUIRE(n == 2);
}

TEST_CASE("aabb_tree: box/radius/knn vs brute oracle", "[aabb_tree]") {
    std::vector<Aabb2> boxes;
    for (int i = 0; i < 40; ++i) {
        const double x = static_cast<double>(i % 8);
        const double y = static_cast<double>(i / 8);
        boxes.push_back(aabb2_from_ltrb(x, y, x + 0.6, y + 0.6));
    }
    auto ltrb = aabb_test_ltrb_from_boxes(boxes);
    AabbTree tree;
    AabbTreeScratch scratch;
    REQUIRE(aabb_tree_rebuild_boxes(ltrb.data(), boxes.size(), tree));

    const double qminx = 1.5, qminy = 1.5, qmaxx = 3.2, qmaxy = 3.0;
    int out[64];
    auto n = aabb_tree_query_box(tree, scratch, qminx, qminy, qmaxx, qmaxy, out, 64);
    std::set<int> got(out, out + n);
    REQUIRE(got == aabb_test_brute_box(boxes, qminx, qminy, qmaxx, qmaxy));

    const double qx = 2.1, qy = 1.9, rsq = 1.5;
    n = aabb_tree_query_radius(tree, scratch, qx, qy, rsq, out, 64);
    got = std::set<int>(out, out + n);
    REQUIRE(got == aabb_test_brute_radius(boxes, qx, qy, rsq));

    double dist[16];
    n = aabb_tree_knn(tree, scratch, qx, qy, 5, out, dist, 16);
    auto expect = aabb_test_brute_knn(boxes, qx, qy, 5);
    REQUIRE(n == expect.size());
    for (std::size_t i = 0; i < n; ++i) {
        REQUIRE(out[i] == expect[i]);
        REQUIRE(dist[i] == Catch::Approx(aabb2_point_dist_sq(boxes[static_cast<std::size_t>(out[i])], qx, qy)));
    }
}

TEST_CASE("aabb_tree: knn k greater than n returns all sorted", "[aabb_tree]") {
    std::vector<Aabb2> boxes = {
        aabb2_from_point(0, 0),
        aabb2_from_point(5, 0),
        aabb2_from_point(1, 0),
    };
    auto ltrb = aabb_test_ltrb_from_boxes(boxes);
    AabbTree tree;
    AabbTreeScratch scratch;
    REQUIRE(aabb_tree_rebuild_boxes(ltrb.data(), boxes.size(), tree));
    int out[8];
    double dist[8];
    // out_cap == n < k: still returns all (k clamped to n).
    auto n = aabb_tree_knn(tree, scratch, 0, 0, 8, out, dist, 3);
    REQUIRE(n == 3);
    REQUIRE(out[0] == 0);
    REQUIRE(out[1] == 2);
    REQUIRE(out[2] == 1);
    n = aabb_tree_knn(tree, scratch, 0, 0, 8, out, dist, 8);
    REQUIRE(n == 3);
}

TEST_CASE("aabb_tree: infinite radius returns all ascending", "[aabb_tree]") {
    const double ltrb[] = {0, 0, 1, 1, 2, 2, 3, 3};
    AabbTree tree;
    AabbTreeScratch scratch;
    REQUIRE(aabb_tree_rebuild_boxes(ltrb, 2, tree));
    int out[4];
    const auto n = aabb_tree_query_radius(tree, scratch, 0, 0, std::numeric_limits<double>::infinity(), out, 4);
    REQUIRE(n == 2);
    REQUIRE(out[0] == 0);
    REQUIRE(out[1] == 1);
}

TEST_CASE("aabb_tree: NaN query returns 0", "[aabb_tree]") {
    const double ltrb[] = {0, 0, 1, 1};
    AabbTree tree;
    AabbTreeScratch scratch;
    REQUIRE(aabb_tree_rebuild_boxes(ltrb, 1, tree));
    int out[4] = {9, 9, 9, 9};
    const double nan = std::numeric_limits<double>::quiet_NaN();
    REQUIRE(aabb_tree_query_box(tree, scratch, nan, 0, 1, 1, out, 4) == 0);
    REQUIRE(aabb_tree_query_radius(tree, scratch, nan, 0, 1.0, out, 4) == 0);
    double dist[4];
    REQUIRE(aabb_tree_knn(tree, scratch, nan, 0, 1, out, dist, 4) == 0);
    REQUIRE(out[0] == 9);
}

TEST_CASE("aabb_tree: null ptr with n>0 fails rebuild", "[aabb_tree]") {
    AabbTree tree;
    REQUIRE_FALSE(aabb_tree_rebuild_boxes(nullptr, 3, tree));
    REQUIRE(tree.nodes.empty());
    REQUIRE_FALSE(aabb_tree_rebuild_points(nullptr, 3, tree));
    REQUIRE(tree.nodes.empty());
}

TEST_CASE("aabb_tree: short stack returns 0 writes nothing", "[aabb_tree]") {
    // n >= 13 forces internal nodes (leaf size 12).
    std::vector<Aabb2> boxes;
    for (int i = 0; i < 13; ++i) {
        const double x = static_cast<double>(i);
        boxes.push_back(aabb2_from_ltrb(x, 0, x + 0.5, 0.5));
    }
    auto ltrb = aabb_test_ltrb_from_boxes(boxes);
    AabbTree tree;
    REQUIRE(aabb_tree_rebuild_boxes(ltrb.data(), boxes.size(), tree));
    REQUIRE(tree.nodes.size() >= 2);
    std::vector<int> stack(tree.nodes.size() - 1, 0);
    int out[4] = {7, 7, 7, 7};
    double dist[4] = {1, 1, 1, 1};
    REQUIRE(aabb_tree_query_box(
                tree,
                stack.data(),
                stack.size(),
                -1,
                -1,
                20,
                1,
                out,
                4)
            == 0);
    REQUIRE(out[0] == 7);
    REQUIRE(aabb_tree_query_radius(
                tree, stack.data(), stack.size(), 0, 0, 100.0, out, 4)
            == 0);
    REQUIRE(out[0] == 7);
    REQUIRE(aabb_tree_knn(
                tree, stack.data(), stack.size(), 0, 0, 2, out, dist, 4)
            == 0);
    REQUIRE(out[0] == 7);
    REQUIRE(dist[0] == Catch::Approx(1.0));
}

TEST_CASE("aabb_tree: inverted query box no hits", "[aabb_tree]") {
    const double ltrb[] = {0, 0, 1, 1, 2, 2, 3, 3};
    AabbTree tree;
    AabbTreeScratch scratch;
    REQUIRE(aabb_tree_rebuild_boxes(ltrb, 2, tree));
    int out[4] = {1, 1, 1, 1};
    REQUIRE(aabb_tree_query_box(tree, scratch, 2, 2, 1, 1, out, 4) == 0);
}

TEST_CASE("aabb_tree: cap too small returns 0 writes nothing", "[aabb_tree]") {
    std::vector<Aabb2> boxes = {
        aabb2_from_ltrb(0, 0, 1, 1),
        aabb2_from_ltrb(2, 2, 3, 3),
        aabb2_from_ltrb(4, 4, 5, 5),
    };
    auto ltrb = aabb_test_ltrb_from_boxes(boxes);
    AabbTree tree;
    AabbTreeScratch scratch;
    REQUIRE(aabb_tree_rebuild_boxes(ltrb.data(), boxes.size(), tree));
    int out[2] = {99, 99};
    // All three overlap a huge query; cap=2 too small.
    REQUIRE(aabb_tree_query_box(tree, scratch, -10, -10, 10, 10, out, 2) == 0);
    REQUIRE(out[0] == 99);
    REQUIRE(out[1] == 99);
    out[0] = 77;
    out[1] = 77;
    REQUIRE(aabb_tree_query_radius(tree, scratch, 2.5, 2.5, 100.0, out, 2) == 0);
    REQUIRE(out[0] == 77);
    // knn: out_cap < k → 0
    double dist[1] = {1.0};
    int knn_out[1] = {42};
    REQUIRE(aabb_tree_knn(tree, scratch, 0, 0, 2, knn_out, dist, 1) == 0);
    REQUIRE(knn_out[0] == 42);
}

TEST_CASE("aabb_tree: non-finite or inverted input box fails rebuild", "[aabb_tree]") {
    AabbTree tree;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    const double bad_nan[] = {0, 0, nan, 1};
    const double bad_inf[] = {0, 0, inf, 1};
    const double bad_inv[] = {2, 0, 1, 1};
    REQUIRE_FALSE(aabb_tree_rebuild_boxes(bad_nan, 1, tree));
    REQUIRE(tree.nodes.empty());
    REQUIRE_FALSE(aabb_tree_rebuild_boxes(bad_inf, 1, tree));
    REQUIRE_FALSE(aabb_tree_rebuild_boxes(bad_inv, 1, tree));
    const double bad_pt[] = {nan, 0.0};
    REQUIRE_FALSE(aabb_tree_rebuild_points(bad_pt, 1, tree));
}

TEST_CASE("aabb_tree: exact cap boundary returns hits", "[aabb_tree]") {
    std::vector<Aabb2> boxes = {
        aabb2_from_ltrb(0, 0, 1, 1),
        aabb2_from_ltrb(0.5, 0.5, 1.5, 1.5),
    };
    auto ltrb = aabb_test_ltrb_from_boxes(boxes);
    AabbTree tree;
    AabbTreeScratch scratch;
    REQUIRE(aabb_tree_rebuild_boxes(ltrb.data(), boxes.size(), tree));
    int out[2] = {-1, -1};
    REQUIRE(aabb_tree_query_box(tree, scratch, 0, 0, 2, 2, out, 2) == 2);
    REQUIRE(out[0] == 0);
    REQUIRE(out[1] == 1);
}

TEST_CASE("aabb_tree: many coincident boxes force split", "[aabb_tree]") {
    std::vector<Aabb2> boxes(100, aabb2_from_ltrb(0, 0, 1, 1));
    auto ltrb = aabb_test_ltrb_from_boxes(boxes);
    AabbTree tree;
    AabbTreeScratch scratch;
    REQUIRE(aabb_tree_rebuild_boxes(ltrb.data(), boxes.size(), tree));
    std::vector<int> out(100, -1);
    REQUIRE(aabb_tree_query_box(tree, scratch, 0.25, 0.25, 0.75, 0.75, out.data(), 100) == 100);
    out.assign(99, -1);
    int sentinel = 42;
    out.push_back(sentinel);  // unused; cap is 99
    REQUIRE(aabb_tree_query_box(tree, scratch, 0.25, 0.25, 0.75, 0.75, out.data(), 99) == 0);
    REQUIRE(out[0] == -1);
}

TEST_CASE("aabb_tree: radius boundary inclusive exact", "[aabb_tree]") {
    const double ltrb[] = {3, 4, 3, 4};  // point at (3,4)
    AabbTree tree;
    AabbTreeScratch scratch;
    REQUIRE(aabb_tree_rebuild_boxes(ltrb, 1, tree));
    int out[1];
    REQUIRE(aabb_tree_query_radius(tree, scratch, 0, 0, 25.0, out, 1) == 1);
    REQUIRE(aabb_tree_query_radius(tree, scratch, 0, 0, std::nextafter(25.0, 0.0), out, 1)
            == 0);
}

TEST_CASE("aabb_tree: knn null out_dist_sq", "[aabb_tree]") {
    const double ltrb[] = {0, 0, 0, 0, 1, 0, 1, 0};
    AabbTree tree;
    AabbTreeScratch scratch;
    REQUIRE(aabb_tree_rebuild_boxes(ltrb, 2, tree));
    int out_a[2] = {-1, -1};
    int out_b[2] = {-1, -1};
    double dist_out[2] = {-1, -1};
    double dist_scratch[2] = {-1, -1};
    REQUIRE(aabb_tree_knn(tree, scratch, 0, 0, 2, out_a, dist_out, 2) == 2);
    REQUIRE(aabb_tree_knn(tree, scratch, 0, 0, 2, out_b, nullptr, 2, dist_scratch)
            == 2);
    REQUIRE(out_a[0] == out_b[0]);
    REQUIRE(out_a[1] == out_b[1]);
    REQUIRE(dist_scratch[0] == Catch::Approx(dist_out[0]));
    REQUIRE(dist_scratch[1] == Catch::Approx(dist_out[1]));
    // Both dist pointers null → 0, out untouched.
    int out_null[2] = {11, 22};
    REQUIRE(aabb_tree_knn(tree, scratch, 0, 0, 2, out_null, nullptr, 2, nullptr)
            == 0);
    REQUIRE(out_null[0] == 11);
    REQUIRE(out_null[1] == 22);
}

TEST_CASE("aabb_tree: ptr stack reuse across queries", "[aabb_tree]") {
    std::vector<Aabb2> boxes;
    for (int i = 0; i < 20; ++i) {
        const double x = static_cast<double>(i);
        boxes.push_back(aabb2_from_ltrb(x, 0, x + 0.4, 0.4));
    }
    auto ltrb = aabb_test_ltrb_from_boxes(boxes);
    AabbTree tree;
    REQUIRE(aabb_tree_rebuild_boxes(ltrb.data(), boxes.size(), tree));
    std::vector<int> stack(tree.nodes.size());
    std::vector<int> out(boxes.size(), -1);
    auto n = aabb_tree_query_box(
        tree,
        stack.data(),
        stack.size(),
        -1,
        -1,
        3,
        1,
        out.data(),
        out.size());
    REQUIRE(std::set<int>(out.begin(), out.begin() + static_cast<std::ptrdiff_t>(n))
            == aabb_test_brute_box(boxes, -1, -1, 3, 1));
    n = aabb_tree_query_radius(
        tree, stack.data(), stack.size(), 0, 0, 4.0, out.data(), out.size());
    REQUIRE(std::set<int>(out.begin(), out.begin() + static_cast<std::ptrdiff_t>(n))
            == aabb_test_brute_radius(boxes, 0, 0, 4.0));
    // Shrink tree; reuse same stack buffer (oversized is fine).
    const double one[] = {0, 0, 1, 1};
    REQUIRE(aabb_tree_rebuild_boxes(one, 1, tree));
    out[0] = -1;
    n = aabb_tree_query_box(
        tree, stack.data(), stack.size(), -1, -1, 2, 2, out.data(), out.size());
    REQUIRE(n == 1);
    REQUIRE(out[0] == 0);
}

TEST_CASE("aabb_tree: const tree scratch reuse overflow then success", "[aabb_tree]") {
    std::vector<Aabb2> boxes = {
        aabb2_from_ltrb(0, 0, 1, 1),
        aabb2_from_ltrb(2, 2, 3, 3),
        aabb2_from_ltrb(4, 4, 5, 5),
    };
    auto ltrb = aabb_test_ltrb_from_boxes(boxes);
    AabbTree tree;
    REQUIRE(aabb_tree_rebuild_boxes(ltrb.data(), boxes.size(), tree));
    const AabbTree& ctree = tree;
    AabbTreeScratch scratch;
    int out[3] = {9, 9, 9};
    REQUIRE(aabb_tree_query_box(ctree, scratch, -10, -10, 10, 10, out, 2) == 0);
    REQUIRE(out[0] == 9);
    REQUIRE(aabb_tree_query_box(ctree, scratch, -10, -10, 10, 10, out, 3) == 3);
    REQUIRE(out[0] == 0);
    REQUIRE(aabb_tree_query_radius(ctree, scratch, 2.5, 2.5, 0.0, out, 3) == 1);
    double dist[3];
    REQUIRE(aabb_tree_knn(ctree, scratch, 0, 0, 2, out, dist, 3) == 2);
    // Rebuild smaller tree; same scratch must still work.
    const double one[] = {0, 0, 1, 1};
    REQUIRE(aabb_tree_rebuild_boxes(one, 1, tree));
    REQUIRE(aabb_tree_query_box(ctree, scratch, -1, -1, 2, 2, out, 3) == 1);
    REQUIRE(out[0] == 0);
}

TEST_CASE("aabb_tree: output ascending ids", "[aabb_tree]") {
    std::vector<Aabb2> boxes;
    for (int i = 0; i < 30; ++i) {
        const double x = static_cast<double>(i % 6);
        const double y = static_cast<double>(i / 6);
        boxes.push_back(aabb2_from_ltrb(x, y, x + 0.5, y + 0.5));
    }
    auto ltrb = aabb_test_ltrb_from_boxes(boxes);
    AabbTree tree;
    AabbTreeScratch scratch;
    REQUIRE(aabb_tree_rebuild_boxes(ltrb.data(), boxes.size(), tree));
    std::vector<int> out(boxes.size());
    const auto n = aabb_tree_query_box(tree, scratch, -1, -1, 10, 10, out.data(), out.size());
    REQUIRE(n == boxes.size());
    for (std::size_t i = 1; i < n; ++i) {
        REQUIRE(out[i - 1] < out[i]);
    }
}

TEST_CASE("aabb_tree: randomized multi-level vs brute oracle", "[aabb_tree]") {
    std::mt19937 rng(42);
    std::uniform_real_distribution<double> pos(-5.0, 5.0);
    std::uniform_real_distribution<double> sz(0.0, 1.5);
    for (int n : {13, 25, 200}) {
        std::vector<Aabb2> boxes;
        boxes.reserve(static_cast<std::size_t>(n));
        for (int i = 0; i < n; ++i) {
            const double x = pos(rng);
            const double y = pos(rng);
            boxes.push_back(aabb2_from_ltrb(x, y, x + sz(rng), y + sz(rng)));
        }
        auto ltrb = aabb_test_ltrb_from_boxes(boxes);
        AabbTree tree;
        AabbTreeScratch scratch;
        REQUIRE(aabb_tree_rebuild_boxes(ltrb.data(), boxes.size(), tree));
        std::vector<int> out(static_cast<std::size_t>(n));
        const double qminx = -1.0, qminy = -1.0, qmaxx = 1.0, qmaxy = 1.0;
        auto nh = aabb_tree_query_box(tree, scratch, qminx, qminy, qmaxx, qmaxy, out.data(), out.size());
        REQUIRE(std::set<int>(out.begin(), out.begin() + static_cast<std::ptrdiff_t>(nh))
                == aabb_test_brute_box(boxes, qminx, qminy, qmaxx, qmaxy));
        nh = aabb_tree_query_radius(tree, scratch, 0.0, 0.0, 4.0, out.data(), out.size());
        REQUIRE(std::set<int>(out.begin(), out.begin() + static_cast<std::ptrdiff_t>(nh))
                == aabb_test_brute_radius(boxes, 0.0, 0.0, 4.0));
        double dist[16];
        const std::size_t k = 5;
        nh = aabb_tree_knn(tree, scratch, 0.0, 0.0, k, out.data(), dist, 16);
        auto expect = aabb_test_brute_knn(boxes, 0.0, 0.0, k);
        REQUIRE(nh == expect.size());
        for (std::size_t i = 0; i < nh; ++i) {
            REQUIRE(out[i] == expect[i]);
        }
    }
}
