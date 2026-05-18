#include <cmath>

#include <nanobind/nanobind.h>
#include <nanobind/stl/vector.h>

#include "elem_graph.h"
#include "rules.h"
#include "types.h"

namespace nb = nanobind;

NB_MODULE(_elem_graph, m) {
    nb::class_<Vec2f>(m, "Vec2")
        .def(nb::init<float, float>())
        .def_prop_rw(
            "x",
            [](const Vec2f& v) { return v[0]; },
            [](Vec2f& v, float x) { v.get_(0) = x; })
        .def_prop_rw(
            "y",
            [](const Vec2f& v) { return v[1]; },
            [](Vec2f& v, float y) { v.get_(1) = y; })
        .def("__getitem__", [](const Vec2f& v, int i) { return v[i]; })
        .def("__setitem__", [](Vec2f& v, int i, float x) { v.get_(i) = x; })
        .def("__len__", [](const Vec2f&) { return 2; });

    nb::class_<Circle2f>(m, "Circle")
        .def(nb::init<>())
        .def(nb::init<Vec2f, float>())
        .def_prop_rw(
            "center",
            [](Circle2f& c) -> Vec2f& { return c.c; },
            [](Circle2f& c, const Vec2f& v) { c.c = v; })
        .def_rw("r_sq", &Circle2f::r_sq)
        .def_prop_ro("radius", [](const Circle2f& c) {
            return std::sqrt(static_cast<double>(c.r_sq));
        });

    nb::class_<ElemPlace>(m, "ElemPlace")
        .def(nb::init<>())
        .def_rw("pos", &ElemPlace::pos)
        .def_rw("a", &ElemPlace::a);

    nb::class_<PointPlaceRule>(m, "PointPlaceRule")
        .def(nb::init<>())
        .def_rw("pos", &PointPlaceRule::pos)
        .def_rw("r", &PointPlaceRule::r)
        .def_rw("w", &PointPlaceRule::w)
        .def_rw("group", &PointPlaceRule::group);

    nb::class_<CirclePlaceRule>(m, "CirclePlaceRule")
        .def(nb::init<>())
        .def_rw("circle", &CirclePlaceRule::circle)
        .def_rw("r", &CirclePlaceRule::r)
        .def_rw("w", &CirclePlaceRule::w)
        .def_rw("group", &CirclePlaceRule::group);

    nb::class_<PointAngleRule>(m, "PointAngleRule")
        .def(nb::init<>())
        .def_rw("pos", &PointAngleRule::pos)
        .def_rw("a", &PointAngleRule::a)
        .def_rw("r", &PointAngleRule::r)
        .def_rw("w", &PointAngleRule::w)
        .def_rw("group", &PointAngleRule::group);

    nb::class_<CircleAngleRule>(m, "CircleAngleRule")
        .def(nb::init<>())
        .def_rw("circle", &CircleAngleRule::circle)
        .def_rw("a", &CircleAngleRule::a)
        .def_rw("r", &CircleAngleRule::r)
        .def_rw("w", &CircleAngleRule::w)
        .def_rw("group", &CircleAngleRule::group);

    nb::class_<PlacementRuleSet>(m, "PlacementRuleSet")
        .def(nb::init<>())
        .def_rw("point_rules", &PlacementRuleSet::point_rules)
        .def_rw("circle_rules", &PlacementRuleSet::circle_rules)
        .def_rw("point_angle_rules", &PlacementRuleSet::point_angle_rules)
        .def_rw("circle_angle_rules", &PlacementRuleSet::circle_angle_rules)
        .def("size", [](const PlacementRuleSet &s) { return s.size(); });

    nb::class_<RuleMutationSettings>(m, "RuleMutationSettings")
        .def(nb::init<>())
        .def_rw("region", &RuleMutationSettings::region)
        .def_rw("dpos", &RuleMutationSettings::dpos)
        .def_rw("dw", &RuleMutationSettings::dw)
        .def_rw("da", &RuleMutationSettings::da)
        .def_rw("insert_p", &RuleMutationSettings::insert_p)
        .def_rw("remove_p", &RuleMutationSettings::remove_p)
        .def_rw("mutate_p", &RuleMutationSettings::mutate_p)
        .def_rw("ngroups", &RuleMutationSettings::ngroups);

    nb::class_<ElemGraph>(m, "ElemGraph")
        .def(nb::init<>())
        .def_rw("group_id", &ElemGraph::group_id)
        .def_rw("elems", &ElemGraph::elems)
        .def_rw("coords", &ElemGraph::coords)
        .def_rw("collisions", &ElemGraph::collisions);

    m.def(
        "augment_rules",
        [](const std::vector<PlacementRuleSet> &rules,
           const RuleMutationSettings &settings) {
            return ::augment_rules(rules, settings);
        });

    m.def(
        "nest_by_graph",
        [](const ElemGraph &g, const std::vector<PlacementRuleSet> &cases) {
            return ::nest_by_graph(g, cases);
        });

    m.def(
        "sort_graph",
        &sort_graph,
        nb::arg("g"),
        nb::arg("rules"),
        nb::arg("reverse") = false);

    m.def(
        "score_rules",
        [](const std::vector<ElemGraph> &graphs,
           const std::vector<PlacementRuleSet> &rules) {
            return ::score_rules(graphs, rules);
        });

    m.def("score_elems", &score_elems, nb::arg("g"), nb::arg("rules"));

    m.def(
        "increase_selection_dfs",
        &increase_selection_dfs,
        nb::arg("g"),
        nb::arg("selected_nodes"),
        nb::arg("max_tries"),
        nb::arg("min_collisions"));

    m.def(
        "increase_score_dfs",
        &increase_score_dfs,
        nb::arg("g"),
        nb::arg("selected_nodes"),
        nb::arg("scores"));
}
