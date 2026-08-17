#include <nanobind/nanobind.h>
namespace nb = nanobind;
#include <nanobind/stl/tuple.h>
#include <nanobind/stl/vector.h>

#include "bindings.h"
#include "contact_edge.h"
#include "decision_arena.h"
#include "motif_base.h"
#include "node_signature.h"
#include "se2.h"

void bind_elem_graph_decision(nb::module_ &m) {
    nb::class_<Se2>(m, "Se2")
        .def(nb::init<>())
        .def(nb::init<float, float, float>(), nb::arg("x"), nb::arg("y"), nb::arg("a"))
        .def_rw("x", &Se2::x)
        .def_rw("y", &Se2::y)
        .def_rw("a", &Se2::a);

    m.def("se2_invert", &se2_invert, nb::arg("t"));
    m.def("se2_compose", &se2_compose, nb::arg("a"), nb::arg("b"));
    m.def("se2_relative", &se2_relative, nb::arg("ref"), nb::arg("t"));

    nb::enum_<MacroRegion>(m, "MacroRegion")
        .value("Rim", MacroRegion::Rim)
        .value("Void", MacroRegion::Void)
        .value("Sheet", MacroRegion::Sheet)
        .value("Motif", MacroRegion::Motif);

    nb::class_<MacroAction>(m, "MacroAction")
        .def(nb::init<>())
        .def_rw("part_gid", &MacroAction::part_gid)
        .def_rw("region", &MacroAction::region)
        .def_rw("rule_id", &MacroAction::rule_id)
        .def_rw("motif_id", &MacroAction::motif_id);

    nb::class_<DecisionArena>(m, "DecisionArena")
        .def(nb::init<>())
        .def("root_id", &DecisionArena::root_id)
        .def("add_node", &DecisionArena::add_node, nb::arg("parent_id"), nb::arg("action"))
        .def(
            "record_visit",
            &DecisionArena::record_visit,
            nb::arg("id"),
            nb::arg("reward"))
        .def("size", &DecisionArena::size)
        .def("may_expand", &DecisionArena::may_expand, nb::arg("id"), nb::arg("c") = 1.5f, nb::arg("alpha") = 0.5f)
        .def(
            "amaf_record",
            &DecisionArena::amaf_record,
            nb::arg("region"),
            nb::arg("rule_id"),
            nb::arg("motif_id"),
            nb::arg("reward"),
            nb::arg("miss") = false)
        .def(
            "amaf_mean",
            &DecisionArena::amaf_mean,
            nb::arg("region"),
            nb::arg("rule_id"),
            nb::arg("motif_id"))
        .def(
            "amaf_visits",
            &DecisionArena::amaf_visits,
            nb::arg("region"),
            nb::arg("rule_id"),
            nb::arg("motif_id"))
        .def(
            "ucb_score",
            &DecisionArena::ucb_score,
            nb::arg("node_id"),
            nb::arg("parent_visits"),
            nb::arg("ucb_c") = 1.4f)
        .def(
            "visits",
            [](const DecisionArena &a, int32_t id) { return a.node(id).visits; },
            nb::arg("id"))
        .def(
            "total_reward",
            [](const DecisionArena &a, int32_t id) { return a.node(id).total_reward; },
            nb::arg("id"))
        .def(
            "parent_id",
            [](const DecisionArena &a, int32_t id) { return a.node(id).parent_id; },
            nb::arg("id"))
        .def(
            "first_child_id",
            [](const DecisionArena &a, int32_t id) { return a.node(id).first_child_id; },
            nb::arg("id"))
        .def(
            "next_sibling_id",
            [](const DecisionArena &a, int32_t id) { return a.node(id).next_sibling_id; },
            nb::arg("id"))
        .def(
            "action",
            [](const DecisionArena &a, int32_t id) { return a.node(id).action; },
            nb::arg("id"));

    nb::class_<MotifRecord>(m, "MotifRecord")
        .def(nb::init<>())
        .def_rw("gid_a", &MotifRecord::gid_a)
        .def_rw("gid_b", &MotifRecord::gid_b)
        .def_rw("relative", &MotifRecord::relative)
        .def_rw("gci", &MotifRecord::gci)
        .def_rw("compactness", &MotifRecord::compactness)
        .def_rw("accept_count", &MotifRecord::accept_count)
        .def_rw("area_a", &MotifRecord::area_a)
        .def_rw("area_b", &MotifRecord::area_b)
        .def_rw("ttl_remaining", &MotifRecord::ttl_remaining);

    nb::class_<MotifBase>(m, "MotifBase")
        .def(nb::init<>())
        .def(
            "upsert",
            &MotifBase::upsert,
            nb::arg("rec"),
            nb::arg("min_compactness") = 0.f,
            nb::arg("ttl") = 0)
        .def("age", &MotifBase::age, nb::arg("step") = 1)
        .def("truncate", &MotifBase::truncate, nb::arg("max_keep"))
        .def("list_for_inject", &MotifBase::list_for_inject, nb::arg("max_keep") = 4)
        .def("size", &MotifBase::size)
        .def("at", nb::overload_cast<int32_t>(&MotifBase::at), nb::arg("id"))
        .def("credit_accept", &MotifBase::credit_accept, nb::arg("id"), nb::arg("ttl") = 0)
        .def("note_hollow_miss", &MotifBase::note_hollow_miss, nb::arg("id"))
        .def(
            "upsert_contact",
            [](MotifBase &b, const ContactEdge &edge, float min_c, int32_t ttl) {
                MotifRecord rec;
                rec.gid_a = edge.gid_a;
                rec.gid_b = edge.gid_b;
                rec.relative = edge.relative_pose;
                rec.gci = edge.gci;
                rec.compactness = edge.compactness;
                rec.area_a = 1.f;
                rec.area_b = 1.f;
                return b.upsert(rec, min_c, ttl);
            },
            nb::arg("edge"),
            nb::arg("min_compactness") = 0.f,
            nb::arg("ttl") = 0)
        .def(
            "find_exact_id",
            &MotifBase::find_exact_id,
            nb::arg("gid_a"),
            nb::arg("gid_b"),
            nb::arg("relative"),
            nb::arg("area_a") = 1.f,
            nb::arg("area_b") = 1.f)
        .def("moving_median_compactness", &MotifBase::moving_median_compactness)
        .def(
            "find_exact",
            [](const MotifBase &b, int32_t ga, int32_t gb, Se2 rel, float aa, float ab) -> nb::object {
                const MotifRecord *p = b.find_exact(ga, gb, rel, aa, ab);
                if (p == nullptr) {
                    return nb::none();
                }
                return nb::cast(*p);
            },
            nb::arg("gid_a"),
            nb::arg("gid_b"),
            nb::arg("relative"),
            nb::arg("area_a") = 1.f,
            nb::arg("area_b") = 1.f)
        .def(
            "find_nearest",
            [](
                const MotifBase &b,
                int32_t ga,
                int32_t gb,
                Se2 rel,
                float max_key_dist,
                float aa,
                float ab
            ) -> nb::object {
                const MotifRecord *p = b.find_nearest(ga, gb, rel, max_key_dist, aa, ab);
                if (p == nullptr) {
                    return nb::none();
                }
                return nb::cast(*p);
            },
            nb::arg("gid_a"),
            nb::arg("gid_b"),
            nb::arg("relative"),
            nb::arg("max_key_dist") = 5.f,
            nb::arg("area_a") = 1.f,
            nb::arg("area_b") = 1.f)
        .def(
            "find_nearest_id",
            &MotifBase::find_nearest_id,
            nb::arg("gid_a"),
            nb::arg("gid_b"),
            nb::arg("relative"),
            nb::arg("max_key_dist") = 5.f,
            nb::arg("area_a") = 1.f,
            nb::arg("area_b") = 1.f);

    nb::class_<ContactEdge>(m, "ContactEdge")
        .def(nb::init<>())
        .def_rw("gid_a", &ContactEdge::gid_a)
        .def_rw("gid_b", &ContactEdge::gid_b)
        .def_rw("packed_i", &ContactEdge::packed_i)
        .def_rw("packed_j", &ContactEdge::packed_j)
        .def_rw("relative_pose", &ContactEdge::relative_pose)
        .def_rw("contact_score", &ContactEdge::contact_score)
        .def_rw("compactness", &ContactEdge::compactness)
        .def_rw("gci", &ContactEdge::gci);

    m.def(
        "gci_surrogate",
        &gci_surrogate,
        nb::arg("compactness"),
        nb::arg("contact_score"),
        nb::arg("alpha") = 0.5f,
        nb::arg("beta") = 0.5f);
    m.def("clamp01", &clamp01, nb::arg("v"));

    nb::class_<NodeSignature>(m, "NodeSignature")
        .def(nb::init<>())
        .def_rw("rim_fill", &NodeSignature::rim_fill)
        .def_rw("void_fill", &NodeSignature::void_fill);

    m.def("related_distance", &related_distance, nb::arg("a"), nb::arg("b"));
}
