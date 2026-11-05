#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "decision_arena.h"
#include "motif_base.h"
#include "node_signature.h"

TEST_CASE("DecisionArena tree links and visits", "[decision_arena]") {
    DecisionArena arena;
    REQUIRE(arena.root_id() == 0);
    MacroAction a{};
    a.region = MacroRegion::Rim;
    a.rule_id = 1;
    const int32_t c0 = arena.add_node(0, a);
    a.region = MacroRegion::Void;
    const int32_t c1 = arena.add_node(0, a);
    REQUIRE(c0 == 1);
    REQUIRE(c1 == 2);
    REQUIRE(arena.node(0).first_child_id == c0);
    REQUIRE(arena.node(c0).next_sibling_id == c1);
    arena.record_visit(c0, 0.5f);
    arena.record_visit(c0, 0.7f);
    REQUIRE(arena.node(c0).visits == 2);
    REQUIRE(arena.node(c0).total_reward == Catch::Approx(1.2f));
}

TEST_CASE("DecisionArena progressive widening", "[decision_arena]") {
    DecisionArena arena;
    MacroAction a{};
    REQUIRE(arena.may_expand(0));
    arena.add_node(0, a);
    // visits=0 → treated as 1; c*1^α = 1.5 → one child still may expand
    REQUIRE(arena.may_expand(0));
    arena.record_visit(0, 0.f);
    arena.record_visit(0, 0.f);
    arena.record_visit(0, 0.f);
    arena.record_visit(0, 0.f);
    // After more visits, cap grows
    REQUIRE(arena.may_expand(0));
}

TEST_CASE("MotifBase order-invariant Triangle-Square", "[motif_base]") {
    MotifBase base;
    MotifRecord ab;
    ab.gid_a = 1;
    ab.gid_b = 2;
    ab.relative = Se2{1.f, 0.f, 0.f};
    ab.gci = 0.8f;
    ab.compactness = 0.9f;
    ab.area_a = 10.f;
    ab.area_b = 5.f;
    const int32_t id0 = base.upsert(ab, 0.5f);
    REQUIRE(id0 == 0);

    MotifRecord ba;
    ba.gid_a = 2;
    ba.gid_b = 1;
    ba.relative = se2_invert(Se2{1.f, 0.f, 0.f});
    ba.gci = 0.7f;
    ba.compactness = 0.9f;
    ba.area_a = 5.f;
    ba.area_b = 10.f;
    const int32_t id1 = base.upsert(ba, 0.5f);
    REQUIRE(id1 == 0);
    REQUIRE(base.size() == 1);
    REQUIRE(base.at(0).accept_count == 2);
}

TEST_CASE("MotifBase rejects low compactness", "[motif_base]") {
    MotifBase base;
    MotifRecord m;
    m.gid_a = 1;
    m.gid_b = 2;
    m.relative = Se2{0.f, 1.f, 0.f};
    m.gci = 0.5f;
    m.compactness = 0.1f;
    m.area_a = 1.f;
    m.area_b = 1.f;
    REQUIRE(base.upsert(m, 0.5f) < 0);
    REQUIRE(base.size() == 0);
}

TEST_CASE("MotifBase TTL age and list_for_inject", "[motif_base]") {
    MotifBase base;
    MotifRecord m;
    m.gid_a = 1;
    m.gid_b = 2;
    m.relative = Se2{1.f, 0.f, 0.f};
    m.gci = 0.8f;
    m.compactness = 0.9f;
    m.area_a = 2.f;
    m.area_b = 1.f;
    // Upsert sets accept_count=1 → Q113 floors TTL at 1 instead of deleting.
    REQUIRE(base.upsert(m, 0.f, 2) == 0);
    REQUIRE(base.at(0).ttl_remaining == 2);
    REQUIRE(base.list_for_inject(4).size() == 1);
    REQUIRE(base.age(1) == 0);
    REQUIRE(base.at(0).ttl_remaining == 1);
    REQUIRE(base.age(1) == 0);  // floored, not dropped
    REQUIRE(base.size() == 1);
    REQUIRE(base.at(0).ttl_remaining == 1);
    // Unproven (accept_count forced 0) still drops.
    base.at(0).accept_count = 0;
    base.at(0).ttl_remaining = 1;
    REQUIRE(base.age(1) == 1);
    REQUIRE(base.size() == 0);
}

TEST_CASE("MotifBase flush even coexists with odd-octant stair", "[motif_base]") {
    // A: do not overwrite hypotenuse; flush wins inject via higher gci (+flush boost).
    MotifBase base;
    MotifRecord stair;
    stair.gid_a = 1;
    stair.gid_b = 2;
    stair.relative = Se2{1.f, 1.f, 0.4f};
    stair.gci = 0.5f;
    stair.compactness = 0.9f;
    stair.area_a = 1.f;
    stair.area_b = 1.f;
    stair.flush = false;
    wang_edge_pair_from_relative(stair.relative, stair.edge_a, stair.edge_b);
    REQUIRE(wang_odd_octant(stair.edge_a, stair.edge_b));
    REQUIRE(base.upsert(stair, 0.f) == 0);

    MotifRecord flush_even;
    flush_even.gid_a = 1;
    flush_even.gid_b = 2;
    flush_even.relative = Se2{1.5f, 0.f, 0.f};
    flush_even.gci = 0.7f;
    flush_even.compactness = 0.9f;
    flush_even.area_a = 1.f;
    flush_even.area_b = 1.f;
    flush_even.flush = true;
    wang_edge_pair_from_relative(
        flush_even.relative, flush_even.edge_a, flush_even.edge_b
    );
    REQUIRE(wang_even_flush_pair(
        flush_even.flush, flush_even.edge_a, flush_even.edge_b
    ));
    REQUIRE(base.upsert(flush_even, 0.f) == 1);
    REQUIRE(base.size() == 2);
    REQUIRE(wang_odd_octant(base.at(0).edge_a, base.at(0).edge_b));
    REQUIRE(base.at(1).flush);
    REQUIRE(base.at(1).gci > base.at(0).gci);
    const auto inject = base.list_for_inject(1);
    REQUIRE(inject.size() == 1);
    REQUIRE(inject[0] == 1);
}

TEST_CASE("MotifBase truncate by accept then gci", "[motif_base]") {
    MotifBase base;
    for (int i = 0; i < 5; ++i) {
        MotifRecord m;
        m.gid_a = 1;
        m.gid_b = 2 + i;
        m.relative = Se2{static_cast<float>(i), 0.f, 0.f};
        m.gci = 0.1f * static_cast<float>(i);
        m.compactness = 0.9f;
        m.area_a = 2.f;
        m.area_b = 1.f;
        m.accept_count = i;
        base.upsert(m, 0.f, 4);
    }
    base.truncate(2);
    REQUIRE(base.size() == 2);
}

TEST_CASE("NodeSignature self distance zero", "[node_signature]") {
    NodeSignature a{};
    a.rim_fill = 0.4f;
    a.void_fill = 0.2f;
    signature_mark_aabb(a, 0.f, 0.f, 10.f, 10.f, 100.f, 100.f);
    REQUIRE(related_distance(a, a) == Catch::Approx(0.f).margin(1e-6));
    NodeSignature b = a;
    b.rim_fill = 1.f;
    REQUIRE(related_distance(a, b) > 0.f);
}

TEST_CASE("DecisionArena snapshots copy parent then mutate", "[decision_arena]") {
    DecisionArena arena;
    REQUIRE(arena.size() == 1);
    BoardSnapshot root;
    root.coverage = 0.25f;
    board_snapshot_set_remaining(root, {0, 1});
    root.free_kind = FreeKind::LargeVoid;
    arena.set_snapshot(0, root);
    REQUIRE(arena.snapshot(0).coverage == Catch::Approx(0.25f));
    REQUIRE(arena.snapshot(0).has_remaining());
    REQUIRE(board_snapshot_remaining_gids(arena.snapshot(0)) == std::vector<int32_t>({0, 1}));

    MacroAction a{};
    a.region = MacroRegion::Void;
    const int32_t c0 = arena.add_node(0, a);
    REQUIRE(arena.size() == 2);
    REQUIRE(arena.snapshot(c0).arena_node_id == c0);
    REQUIRE(arena.snapshot(c0).coverage == Catch::Approx(0.25f));
    REQUIRE(arena.snapshot(c0).free_kind == FreeKind::LargeVoid);
    arena.snapshot(c0).coverage = 0.4f;
    REQUIRE(arena.snapshot(0).coverage == Catch::Approx(0.25f));
    REQUIRE(arena.snapshot(c0).coverage == Catch::Approx(0.4f));
}

TEST_CASE("BoardSnapshot remaining bitmask catalog types", "[decision_arena]") {
    BoardSnapshot snap;
    board_snapshot_set_remaining(snap, {10, 11, 12});
    REQUIRE(snap.remaining_mask == ((uint64_t{1} << 10) | (uint64_t{1} << 11) | (uint64_t{1} << 12)));
    const auto gids = board_snapshot_remaining_gids(snap);
    REQUIRE(gids.size() == 3);
    REQUIRE(gids[0] == 10);
    REQUIRE(gids[1] == 11);
    REQUIRE(gids[2] == 12);
    REQUIRE(snap.n_packed() == 0);
}
