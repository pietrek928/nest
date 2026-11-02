#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "decision_graph.h"
#include "graph_test_helpers.h"

TEST_CASE("DecisionGraph replace_poses copy-in keeps macros", "[decision_graph]") {
    DecisionGraph dg;
    MacroAction a{};
    a.region = MacroRegion::Void;
    const int32_t child = dg.macros().add_node(0, a);
    dg.macros().record_visit(child, 0.5f);
    REQUIRE(dg.macros().size() == 2);

    PoseGraph g;
    append_elem_at(g, 0, 0.0f, 0.0f);
    append_elem_at(g, 1, 2.0f, 0.0f);
    dg.replace_poses(g);
    REQUIRE(dg.poses().size() == 2);
    REQUIRE(g.size() == 2);
    REQUIRE(dg.macros().size() == 2);
    REQUIRE(dg.macros().node(child).visits == 1);
    REQUIRE(dg.attach_n() == 0);
    REQUIRE(dg.kind_tagged_n() == 0);
    REQUIRE(dg.pose_kind().size() == 2);
    REQUIRE(dg.pose_kind()[0] == kPoseKindUntagged);
}

TEST_CASE("DecisionGraph add_attach skips Collision like add_attract", "[decision_graph]") {
    DecisionGraph dg;
    PoseGraph g;
    append_elem_at(g, 0, 0.0f, 0.0f);
    append_elem_at(g, 0, 1.0f, 0.0f);
    append_elem_at(g, 0, 2.0f, 0.0f);
    add_collision_pair(g, 0, 1);
    dg.replace_poses(g);
    dg.add_attach(0, 1);
    REQUIRE(dg.attach_n() == 0);
    dg.add_attach(0, 2);
    REQUIRE(dg.attach_n() == 1);
    dg.add_attach(0, 2);
    REQUIRE(dg.attach_n() == 1);
    dg.add_attach(2, 2);
    REQUIRE(dg.attach_n() == 1);
}

TEST_CASE("DecisionGraph Mutex is derived via conflicts PathNode", "[decision_graph]") {
    DecisionGraph dg;
    PoseGraph g;
    append_elem_at(g, 0, 0.0f, 0.0f);
    append_elem_at(g, 0, 1.0f, 0.0f);
    append_elem_at(g, 0, 2.0f, 0.0f);
    append_elem_at(g, 0, 3.0f, 0.0f);
    add_collision_pair(g, 1, 2);
    dg.replace_poses(g);
    dg.add_attach(0, 1);
    dg.add_attach(2, 3);
    REQUIRE(dg.attach_n() == 2);
    REQUIRE(dg.conflicts(dg.path_node_attach(0), dg.path_node_attach(1)));
    REQUIRE(dg.mutex_n() == 1);
    dg.add_motif_join(0, 0, 3);
    REQUIRE(dg.mutex_n() == 1);
}

TEST_CASE("DecisionGraph conflict_p soft Macro vs hard Pose", "[decision_graph]") {
    DecisionGraph dg;
    PoseGraph g;
    append_elem_at(g, 0, 0.0f, 0.0f);
    append_elem_at(g, 0, 1.0f, 0.0f);
    add_collision_pair(g, 0, 1);
    dg.replace_poses(g);
    dg.set_pose_kind(0, kPoseKindVoid);
    const PathNode macro = node_macro(0);
    const PathNode pose0 = node_pose(0);
    const PathNode pose1 = node_pose(1);
    REQUIRE_FALSE(dg.conflicts(macro, pose0));
    const float p_partial = dg.conflict_p(macro, pose0, FreeKind::LargeVoid, 0.5f);
    REQUIRE(p_partial > 0.f);
    REQUIRE(p_partial < kConflictPAlpha);
    REQUIRE(p_partial == Catch::Approx(kConflictPAlpha * 0.5f).margin(1e-5f));
    const float p_phantom = dg.conflict_p(macro, pose0, FreeKind::LargeVoid, 0.f);
    REQUIRE(p_phantom == Catch::Approx(kConflictPAlpha).margin(1e-5f));
    REQUIRE(dg.conflict_p(macro, pose0, FreeKind::Empty, 0.f) == 0.f);
    REQUIRE(dg.conflict_p(pose0, pose1, FreeKind::LargeVoid, 0.f) == 1.f);
    REQUIRE(dg.conflicts(pose0, pose1));
}

TEST_CASE("nest_by_scores DecisionGraph forwards to poses", "[decision_graph]") {
    DecisionGraph dg;
    PoseGraph g;
    append_elem_at(g, 0, 0.0f, 0.0f);
    append_elem_at(g, 0, 1.0f, 0.0f);
    add_collision_pair(g, 0, 1);
    dg.replace_poses(g);
    const std::vector<Tscore> scores{2.0f, 1.0f};
    const auto selected = nest_by_scores(dg, scores);
    REQUIRE(selected.size() == 1);
    REQUIRE(selected[0] == 0);
}

TEST_CASE("realize flags Attach whose members survived", "[decision_graph]") {
    DecisionGraph dg;
    PoseGraph g;
    append_elem_at(g, 0, 0.0f, 0.0f);
    append_elem_at(g, 0, 2.0f, 0.0f);
    append_elem_at(g, 0, 4.0f, 0.0f);
    dg.replace_poses(g);
    dg.set_pose_kind(0, static_cast<uint8_t>(MacroRegion::Void));
    dg.set_pose_kind(1, static_cast<uint8_t>(MacroRegion::Void));
    dg.add_attach(0, 1);
    dg.add_attach(1, 2);
    const auto st = dg.realize({0, 1});
    REQUIRE(st.attach == 1);
    REQUIRE(st.member_hits == 2);
    REQUIRE(st.kind_count[static_cast<int>(MacroRegion::Void)] == 2);
    REQUIRE(dg.attach()[0].realized);
    REQUIRE_FALSE(dg.attach()[1].realized);
    REQUIRE(dg.realized(dg.path_node_attach(0)));
    REQUIRE_FALSE(dg.realized(dg.path_node_attach(1)));
}

TEST_CASE("neighbors MotifJoin Pose and survive_counts", "[decision_graph]") {
    DecisionGraph dg;
    PoseGraph g;
    append_elem_at(g, 0, 0.0f, 0.0f);
    append_elem_at(g, 0, 2.0f, 0.0f);
    append_elem_at(g, 0, 4.0f, 0.0f);
    dg.replace_poses(g);
    dg.set_pose_kind(0, static_cast<uint8_t>(MacroRegion::Motif));
    dg.set_pose_kind(1, static_cast<uint8_t>(MacroRegion::Motif));
    dg.add_motif_join(7, 0, 1);
    dg.add_motif_join(7, 1, 2);
    dg.realize({0, 1});
    const auto nbrs = dg.neighbors(dg.path_node_motif(0));
    REQUIRE(nbrs.size() >= 2);
    const auto sc = dg.survive_counts();
    REQUIRE(sc.size() == 1);
    REQUIRE(sc[0].first == 7);
    REQUIRE(sc[0].second == 1);
}

TEST_CASE("Macro neighbors union Attach touching tagged MotifJoin", "[decision_graph]") {
    DecisionGraph dg;
    MacroAction a{};
    a.region = MacroRegion::Motif;
    const int32_t macro_id = dg.macros().add_node(0, a);
    PoseGraph g;
    append_elem_at(g, 0, 0.0f, 0.0f);
    append_elem_at(g, 0, 2.0f, 0.0f);
    append_elem_at(g, 0, 4.0f, 0.0f);
    dg.replace_poses(g);
    dg.set_pose_kind(0, static_cast<uint8_t>(MacroRegion::Motif));
    dg.set_pose_kind(1, static_cast<uint8_t>(MacroRegion::Motif));
    dg.add_motif_join(3, 0, 1);
    dg.add_attach(1, 2);
    dg.add_attach(0, 2);
    const auto nbrs = dg.neighbors(node_macro(macro_id));
    int motif_n = 0;
    int attach_n = 0;
    for (const PathNode &n : nbrs) {
        if (n.kind == PathKind::MotifJoin) {
            motif_n += 1;
        } else if (n.kind == PathKind::Attach) {
            attach_n += 1;
        }
    }
    REQUIRE(motif_n >= 1);
    REQUIRE(attach_n >= 1);
}

TEST_CASE("Attach neighbors include MotifJoin sharing endpoint", "[decision_graph]") {
    DecisionGraph dg;
    PoseGraph g;
    append_elem_at(g, 0, 0.0f, 0.0f);
    append_elem_at(g, 0, 2.0f, 0.0f);
    append_elem_at(g, 0, 4.0f, 0.0f);
    dg.replace_poses(g);
    dg.add_motif_join(5, 0, 1);
    dg.add_attach(1, 2);
    const auto nbrs = dg.neighbors(dg.path_node_attach(0));
    int pose_n = 0;
    int motif_n = 0;
    for (const PathNode &n : nbrs) {
        if (n.kind == PathKind::Pose) {
            pose_n += 1;
        } else if (n.kind == PathKind::MotifJoin) {
            motif_n += 1;
        }
    }
    REQUIRE(pose_n >= 2);
    REQUIRE(motif_n >= 1);
}
