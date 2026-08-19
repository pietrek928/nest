#include <catch2/catch_test_macros.hpp>

#include "decision_graph.h"
#include "elem_graph_test_helpers.h"

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

TEST_CASE("DecisionGraph Mutex is derived Collision among members", "[decision_graph]") {
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
    REQUIRE(dg.attach_conflicts(0, 1));
    REQUIRE(dg.mutex_n() == 1);
    dg.add_motif_join(0, 0, 3);
    REQUIRE(dg.mutex_n() == 1);
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

TEST_CASE("materialize_selection flags Attach whose members survived", "[decision_graph]") {
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
    const auto st = dg.materialize_selection({0, 1});
    REQUIRE(st.materialized_attach == 1);
    REQUIRE(st.member_hits == 2);
    REQUIRE(st.kind_count[static_cast<int>(MacroRegion::Void)] == 2);
    REQUIRE(dg.attach()[0].realized);
    REQUIRE_FALSE(dg.attach()[1].realized);
}
