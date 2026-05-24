#include <cmath>

#include <nanobind/nanobind.h>
namespace nb = nanobind;
#include <nanobind/stl/vector.h>

#include "bindings.h"
#include "graph/graph.h"
#include "rules/rules.h"
#include "rules/rules_util.h"
#include "types/types.h"

void bind_elem_graph_types(nb::module_ &m) {
    nb::enum_<ScoreAggregation>(m, "ScoreAggregation")
        .value("Sum", ScoreAggregation::Sum)
        .value("Max", ScoreAggregation::Max);

    nb::enum_<SelectMode>(m, "SelectMode")
        .value("GreedyScore", SelectMode::GreedyScore)
        .value("WeightedGreedy", SelectMode::WeightedGreedy);

    nb::class_<SelectOptions>(m, "SelectOptions")
        .def(nb::init<>())
        .def_rw("mode", &SelectOptions::mode)
        .def_rw("local_swap", &SelectOptions::local_swap)
        .def_rw("aggregation", &SelectOptions::aggregation);

    nb::class_<RefineSelectionOptions>(m, "RefineSelectionOptions")
        .def(nb::init<>())
        .def_rw("min_score_delta", &RefineSelectionOptions::min_score_delta)
        .def_rw("max_passes", &RefineSelectionOptions::max_passes)
        .def_rw("max_depth", &RefineSelectionOptions::max_depth)
        .def_rw("seed", &RefineSelectionOptions::seed)
        .def_rw("explore_shuffle", &RefineSelectionOptions::explore_shuffle)
        .def_rw("beam_width", &RefineSelectionOptions::beam_width)
        .def_rw("max_root_collisions", &RefineSelectionOptions::max_root_collisions)
        .def_rw("max_tries", &RefineSelectionOptions::max_tries)
        .def_rw("min_collisions", &RefineSelectionOptions::min_collisions);

    nb::class_<FinalizeSelectionOptions>(m, "FinalizeSelectionOptions")
        .def(nb::init<>())
        .def_rw("repair_passes", &FinalizeSelectionOptions::repair_passes)
        .def_rw("max_exact_component_size", &FinalizeSelectionOptions::max_exact_component_size);

    nb::class_<Vec2f>(m, "Vec2")
        .def(nb::init<float, float>(), nb::arg("x"), nb::arg("y"))
        .def_prop_rw(
            "x",
            [](const Vec2f &v) { return v[0]; },
            [](Vec2f &v, float x) { v.get_(0) = x; })
        .def_prop_rw(
            "y",
            [](const Vec2f &v) { return v[1]; },
            [](Vec2f &v, float y) { v.get_(1) = y; })
        .def("__getitem__", [](const Vec2f &v, int i) { return v[i]; })
        .def("__setitem__", [](Vec2f &v, int i, float x) { v.get_(i) = x; })
        .def("__len__", [](const Vec2f &) { return 2; });

    nb::class_<Circle2f>(m, "Circle")
        .def(nb::init<>())
        .def(nb::init<Vec2f, float>())
        .def_prop_rw(
            "center",
            [](Circle2f &c) -> Vec2f & { return c.c; },
            [](Circle2f &c, const Vec2f &v) { c.c = v; })
        .def_rw("r_sq", &Circle2f::r_sq)
        .def_prop_ro("radius", [](const Circle2f &c) {
            return std::sqrt(static_cast<double>(c.r_sq));
        })
        .def_static(
            "from_bounds",
            [](float xmin, float ymin, float xmax, float ymax) {
                const float cx = 0.5f * (xmin + xmax);
                const float cy = 0.5f * (ymin + ymax);
                const float hx = 0.5f * (xmax - xmin);
                const float hy = 0.5f * (ymax - ymin);
                return Circle2f(Vec2f({cx, cy}), hx * hx + hy * hy);
            },
            nb::arg("xmin"),
            nb::arg("ymin"),
            nb::arg("xmax"),
            nb::arg("ymax"))
        .def_static(
            "from_center_radius",
            [](float cx, float cy, float radius) {
                const float r = radius;
                return Circle2f(Vec2f({cx, cy}), r * r);
            },
            nb::arg("cx"),
            nb::arg("cy"),
            nb::arg("radius"));

    nb::class_<ElemPlace>(m, "ElemPlace")
        .def(nb::init<>())
        .def_rw("pos", &ElemPlace::pos)
        .def_rw("a", &ElemPlace::a);

    nb::class_<PointPlaceRule>(m, "PointPlaceRule")
        .def(
            nb::init<Vec2f, float, float, int>(),
            nb::arg("pos"),
            nb::arg("r"),
            nb::arg("w"),
            nb::arg("group"))
        .def_rw("pos", &PointPlaceRule::pos)
        .def_rw("r", &PointPlaceRule::r)
        .def_rw("w", &PointPlaceRule::w)
        .def_rw("group", &PointPlaceRule::group);

    nb::class_<CirclePlaceRule>(m, "CirclePlaceRule")
        .def(
            nb::init<Circle2f, float, float, int>(),
            nb::arg("circle"),
            nb::arg("r"),
            nb::arg("w"),
            nb::arg("group"))
        .def_rw("circle", &CirclePlaceRule::circle)
        .def_rw("r", &CirclePlaceRule::r)
        .def_rw("w", &CirclePlaceRule::w)
        .def_rw("group", &CirclePlaceRule::group);

    nb::class_<PointAngleRule>(m, "PointAngleRule")
        .def(
            nb::init<Vec2f, float, float, float, int>(),
            nb::arg("pos"),
            nb::arg("a"),
            nb::arg("r"),
            nb::arg("w"),
            nb::arg("group"))
        .def_rw("pos", &PointAngleRule::pos)
        .def_rw("a", &PointAngleRule::a)
        .def_rw("r", &PointAngleRule::r)
        .def_rw("w", &PointAngleRule::w)
        .def_rw("group", &PointAngleRule::group);

    nb::class_<CircleAngleRule>(m, "CircleAngleRule")
        .def(
            nb::init<Circle2f, float, float, float, int>(),
            nb::arg("circle"),
            nb::arg("a"),
            nb::arg("r"),
            nb::arg("w"),
            nb::arg("group"))
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
        .def("size", [](const PlacementRuleSet &s) { return s.size(); })
        .def(
            "append_rule",
            [](PlacementRuleSet &s, nb::handle rule) {
                if (nb::isinstance<PointPlaceRule>(rule)) {
                    append_rule(s, nb::cast<PointPlaceRule>(rule));
                } else if (nb::isinstance<CirclePlaceRule>(rule)) {
                    append_rule(s, nb::cast<CirclePlaceRule>(rule));
                } else if (nb::isinstance<PointAngleRule>(rule)) {
                    append_rule(s, nb::cast<PointAngleRule>(rule));
                } else if (nb::isinstance<CircleAngleRule>(rule)) {
                    append_rule(s, nb::cast<CircleAngleRule>(rule));
                } else {
                    throw nb::type_error(
                        "append_rule: expected PointPlaceRule, CirclePlaceRule, "
                        "PointAngleRule, or CircleAngleRule");
                }
            },
            nb::arg("rule"))
        .def("append_point", &append_point_place_rule)
        .def(
            "append_point_at",
            &append_point_place_rule_at,
            nb::arg("pos"),
            nb::arg("r"),
            nb::arg("w"),
            nb::arg("group"))
        .def("append_circle", &append_circle_place_rule)
        .def(
            "append_circle_at",
            &append_circle_place_rule_at,
            nb::arg("circle"),
            nb::arg("r"),
            nb::arg("w"),
            nb::arg("group"))
        .def("append_point_angle", &append_point_angle_rule)
        .def(
            "append_point_angle_at",
            &append_point_angle_rule_at,
            nb::arg("pos"),
            nb::arg("a"),
            nb::arg("r"),
            nb::arg("w"),
            nb::arg("group"))
        .def("append_circle_angle", &append_circle_angle_rule)
        .def(
            "append_circle_angle_at",
            &append_circle_angle_rule_at,
            nb::arg("circle"),
            nb::arg("a"),
            nb::arg("r"),
            nb::arg("w"),
            nb::arg("group"));

    nb::class_<RuleMutationSettings>(m, "RuleMutationSettings")
        .def(
            nb::init<Circle2f, float, float, float, float, float, float, int>(),
            nb::arg("region"),
            nb::arg("dpos"),
            nb::arg("dw"),
            nb::arg("da"),
            nb::arg("insert_p"),
            nb::arg("remove_p"),
            nb::arg("mutate_p"),
            nb::arg("ngroups"))
        .def_rw("region", &RuleMutationSettings::region)
        .def_rw("dpos", &RuleMutationSettings::dpos)
        .def_rw("dw", &RuleMutationSettings::dw)
        .def_rw("da", &RuleMutationSettings::da)
        .def_rw("insert_p", &RuleMutationSettings::insert_p)
        .def_rw("remove_p", &RuleMutationSettings::remove_p)
        .def_rw("mutate_p", &RuleMutationSettings::mutate_p)
        .def_rw("ngroups", &RuleMutationSettings::ngroups)
        .def_rw("max_inserts_per_type", &RuleMutationSettings::max_inserts_per_type);

    nb::class_<ScoreRulesOptions>(m, "ScoreRulesOptions")
        .def(nb::init<>())
        .def_rw("rule_complexity_penalty", &ScoreRulesOptions::rule_complexity_penalty)
        .def_rw("latest_graph_only", &ScoreRulesOptions::latest_graph_only)
        .def_rw("mean_score_weight", &ScoreRulesOptions::mean_score_weight)
        .def_rw("count_weight", &ScoreRulesOptions::count_weight)
        .def_rw("selection_score_only", &ScoreRulesOptions::selection_score_only)
        .def_rw("select", &ScoreRulesOptions::select);

    nb::class_<ElemGraph>(m, "ElemGraph")
        .def(nb::init<>())
        .def_rw("group_id", &ElemGraph::group_id)
        .def_rw("elems", &ElemGraph::elems)
        .def_rw("coords", &ElemGraph::coords)
        .def_rw("collisions", &ElemGraph::collisions)
        .def(
            "push_elem",
            [](ElemGraph &g, int group_id, const ElemPlace &ep, const Circle2f &circle) {
                g.group_id.push_back(group_id);
                g.elems.push_back(ep);
                g.coords.push_back(circle);
                g.collisions.emplace_back();
            },
            nb::arg("group_id"),
            nb::arg("elem"),
            nb::arg("circle"))
        .def(
            "append_elem",
            [](ElemGraph &g, int group_id, Vec2f pos, Circle2f circle) {
                g.group_id.push_back(group_id);
                g.elems.push_back(ElemPlace{pos, 0.0f});
                g.coords.push_back(circle);
                g.collisions.emplace_back();
            },
            nb::arg("group_id"),
            nb::arg("pos"),
            nb::arg("circle"))
        .def(
            "append_elem_at",
            [](ElemGraph &g, int group_id, float x, float y, float a,
               float cx, float cy, float r_sq) {
                g.group_id.push_back(group_id);
                g.elems.push_back(ElemPlace{Vec2f({x, y}), a});
                g.coords.push_back(Circle2f(Vec2f({cx, cy}), r_sq));
                g.collisions.emplace_back();
            },
            nb::arg("group_id"),
            nb::arg("x"),
            nb::arg("y"),
            nb::arg("a"),
            nb::arg("cx"),
            nb::arg("cy"),
            nb::arg("r_sq"))
        .def(
            "add_collision_pair",
            [](ElemGraph &g, int i, int j) {
                g.collisions[i].push_back(j);
                g.collisions[j].push_back(i);
            },
            nb::arg("i"),
            nb::arg("j"))
        .def(
            "add_collision",
            [](ElemGraph &g, int i, int j) {
                g.collisions[i].push_back(j);
                g.collisions[j].push_back(i);
            },
            nb::arg("i"),
            nb::arg("j"))
        .def(
            "reserve_elems",
            [](ElemGraph &g, std::size_t n) {
                g.group_id.reserve(n);
                g.elems.reserve(n);
                g.coords.reserve(n);
                g.collisions.reserve(n);
            },
            nb::arg("n"));
}
