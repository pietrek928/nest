#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include <nanobind/nanobind.h>
namespace nb = nanobind;
#include <nanobind/stl/string.h>
#include <nanobind/stl/tuple.h>
#include <nanobind/stl/vector.h>

#include "bindings.h"
#include "contact_edge.h"
#include "decision_arena.h"
#include "decision_graph.h"
#include "macro_niche_archive.h"
#include "motif_base.h"
#include "node_signature.h"
#include "se2.h"

Se2 se2_from_handle(nb::handle h) {
    if (nb::isinstance<Se2>(h)) {
        return nb::cast<Se2>(h);
    }
    nb::sequence seq = nb::cast<nb::sequence>(h);
    if (nb::len(seq) < 3) {
        throw nb::value_error("SE2 pose needs (x, y, theta)");
    }
    return Se2{
        static_cast<float>(nb::cast<double>(seq[0])),
        static_cast<float>(nb::cast<double>(seq[1])),
        static_cast<float>(nb::cast<double>(seq[2])),
    };
}

void bind_elem_graph_decision(nb::module_ &m) {
    nb::class_<Se2>(m, "Se2")
        .def(nb::init<>())
        .def(nb::init<float, float, float>(), nb::arg("x"), nb::arg("y"), nb::arg("a"))
        .def_rw("x", &Se2::x)
        .def_rw("y", &Se2::y)
        .def_rw("a", &Se2::a)
        .def(
            "__getitem__",
            [](const Se2 &s, int i) -> float {
                if (i < 0) {
                    i += 3;
                }
                if (i == 0) {
                    return s.x;
                }
                if (i == 1) {
                    return s.y;
                }
                if (i == 2) {
                    return s.a;
                }
                throw nb::index_error("Se2 index out of range");
            })
        .def("__len__", [](const Se2 &) { return 3; });

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

    nb::class_<BoardSnapshot>(m, "BoardSnapshot")
        .def(nb::init<>())
        .def(
            "__init__",
            [](BoardSnapshot *self,
               std::vector<int32_t> packed_gids,
               nb::object packed_transforms,
               std::vector<int32_t> remaining_gids,
               uint64_t remaining_mask,
               float coverage,
               int32_t arena_node_id,
               int32_t kiss_pairs,
               float mean_compactness,
               float rim_fill,
               float void_fill,
               std::string free_kind,
               std::vector<int32_t> motif_ids_used,
               nb::object /*telem*/) {
                new (self) BoardSnapshot();
                self->packed_gids = std::move(packed_gids);
                if (!packed_transforms.is_none()) {
                    nb::sequence seq = nb::cast<nb::sequence>(packed_transforms);
                    const Py_ssize_t n = nb::len(seq);
                    self->packed_transforms.reserve(static_cast<std::size_t>(n));
                    for (Py_ssize_t i = 0; i < n; ++i) {
                        self->packed_transforms.push_back(se2_from_handle(seq[i]));
                    }
                }
                if (!remaining_gids.empty()) {
                    board_snapshot_set_remaining(*self, remaining_gids);
                } else {
                    self->remaining_mask = remaining_mask;
                }
                self->coverage = coverage;
                self->arena_node_id = arena_node_id;
                self->kiss_pairs = kiss_pairs;
                self->mean_compactness = mean_compactness;
                self->rim_fill = rim_fill;
                self->void_fill = void_fill;
                self->free_kind = std::move(free_kind);
                self->motif_ids_used = std::move(motif_ids_used);
            },
            nb::kw_only(),
            nb::arg("packed_gids") = std::vector<int32_t>{},
            nb::arg("packed_transforms") = nb::none(),
            nb::arg("remaining_gids") = std::vector<int32_t>{},
            nb::arg("remaining_mask") = uint64_t{0},
            nb::arg("coverage") = 0.f,
            nb::arg("arena_node_id") = 0,
            nb::arg("kiss_pairs") = 0,
            nb::arg("mean_compactness") = 0.f,
            nb::arg("rim_fill") = 0.f,
            nb::arg("void_fill") = 0.f,
            nb::arg("free_kind") = std::string{},
            nb::arg("motif_ids_used") = std::vector<int32_t>{},
            nb::arg("telem") = nb::none())
        .def_rw("packed_gids", &BoardSnapshot::packed_gids)
        .def_prop_rw(
            "packed_transforms",
            [](const BoardSnapshot &s) { return s.packed_transforms; },
            [](BoardSnapshot &s, nb::object obj) {
                s.packed_transforms.clear();
                if (obj.is_none()) {
                    return;
                }
                nb::sequence seq = nb::cast<nb::sequence>(obj);
                const Py_ssize_t n = nb::len(seq);
                s.packed_transforms.reserve(static_cast<std::size_t>(n));
                for (Py_ssize_t i = 0; i < n; ++i) {
                    s.packed_transforms.push_back(se2_from_handle(seq[i]));
                }
            })
        .def_rw("remaining_mask", &BoardSnapshot::remaining_mask)
        .def_prop_rw(
            "remaining_gids",
            [](const BoardSnapshot &s) { return board_snapshot_remaining_gids(s); },
            [](BoardSnapshot &s, const std::vector<int32_t> &gids) {
                board_snapshot_set_remaining(s, gids);
            })
        .def_rw("coverage", &BoardSnapshot::coverage)
        .def_rw("arena_node_id", &BoardSnapshot::arena_node_id)
        .def_rw("kiss_pairs", &BoardSnapshot::kiss_pairs)
        .def_rw("mean_compactness", &BoardSnapshot::mean_compactness)
        .def_rw("rim_fill", &BoardSnapshot::rim_fill)
        .def_rw("void_fill", &BoardSnapshot::void_fill)
        .def_rw("free_kind", &BoardSnapshot::free_kind)
        .def_rw("motif_ids_used", &BoardSnapshot::motif_ids_used)
        .def_prop_ro("n_packed", &BoardSnapshot::n_packed)
        .def_prop_ro("has_remaining", &BoardSnapshot::has_remaining);

    m.attr("NICHE_RING_H") = NICHE_RING_H;

    nb::class_<NicheRow>(m, "NicheRow")
        .def_ro("gid", &NicheRow::gid)
        .def_ro("x", &NicheRow::x)
        .def_ro("y", &NicheRow::y)
        .def_ro("theta", &NicheRow::theta);

    nb::class_<NicheEpisode>(m, "NicheEpisode")
        .def_ro("polarity", &NicheEpisode::polarity)
        .def_ro("void_nest", &NicheEpisode::void_nest)
        .def_ro("score", &NicheEpisode::score)
        .def_ro("rows", &NicheEpisode::rows);

    nb::class_<MacroNicheBucket>(m, "MacroNicheBucket")
        .def_ro("hits", &MacroNicheBucket::hits)
        .def_ro("misses", &MacroNicheBucket::misses)
        .def_ro("best_void_nest", &MacroNicheBucket::best_void_nest)
        .def_ro("ttl_remaining", &MacroNicheBucket::ttl_remaining)
        .def_ro("ring", &MacroNicheBucket::ring)
        .def_prop_ro("miss_rate", &MacroNicheBucket::miss_rate);

    nb::class_<MacroNicheArchive>(m, "MacroNicheArchive")
        .def(nb::init<>())
        .def("get", &MacroNicheArchive::get, nb::arg("key"), nb::rv_policy::reference_internal)
        .def_prop_ro("size", &MacroNicheArchive::size)
        .def_prop_ro("empty", &MacroNicheArchive::empty)
        .def("total_hits", &MacroNicheArchive::total_hits)
        .def_prop_ro(
            "buckets",
            [](MacroNicheArchive &a) -> nb::dict {
                nb::dict out;
                for (auto &kv : a.buckets()) {
                    const auto &key = kv.first;
                    nb::dict bucket;
                    bucket["hits"] = kv.second.hits;
                    bucket["misses"] = kv.second.misses;
                    out[nb::make_tuple(
                        std::get<0>(key), std::get<1>(key), std::get<2>(key))] = bucket;
                }
                return out;
            })
        .def(
            "append_positive",
            [](MacroNicheArchive &arch,
               NicheKey key,
               const std::vector<std::tuple<int32_t, float, float, float>> &rows,
               int32_t void_nest,
               float score,
               int32_t ttl) {
                std::vector<NicheRow> parsed;
                parsed.reserve(rows.size());
                for (const auto &row : rows) {
                    NicheRow r;
                    r.gid = std::get<0>(row);
                    r.x = std::get<1>(row);
                    r.y = std::get<2>(row);
                    r.theta = std::get<3>(row);
                    parsed.push_back(r);
                }
                arch.append_positive(key, parsed, void_nest, score, ttl);
            },
            nb::arg("key"),
            nb::arg("rows"),
            nb::arg("void_nest"),
            nb::arg("score") = 0.f,
            nb::arg("ttl") = 4)
        .def(
            "append_negative",
            &MacroNicheArchive::append_negative,
            nb::arg("key"),
            nb::arg("void_nest") = 0,
            nb::arg("score") = 0.f)
        .def(
            "any_void_miss_rate_high",
            &MacroNicheArchive::any_void_miss_rate_high,
            nb::arg("threshold") = 0.8f)
        .def(
            "active_by_group",
            [](const MacroNicheArchive &arch, int32_t ngroups) -> nb::dict {
                nb::dict out;
                const auto raw = arch.active_by_group(ngroups);
                for (const auto &kv : raw) {
                    nb::list rows;
                    for (const auto &arr : kv.second) {
                        rows.append(nb::make_tuple(arr[0], arr[1], arr[2]));
                    }
                    out[nb::int_(kv.first)] = rows;
                }
                return out;
            },
            nb::arg("ngroups"))
        .def("age", &MacroNicheArchive::age, nb::arg("step") = 1)
        .def("note_place_outcome", &MacroNicheArchive::note_place_outcome, nb::arg("placed"))
        .def_rw("place_fail_streak", &MacroNicheArchive::place_fail_streak)
        .def_rw("max_rows_per_episode", &MacroNicheArchive::max_rows_per_episode)
        .def_prop_rw(
            "last_feed_keys",
            [](const MacroNicheArchive &a) {
                nb::set out;
                for (const auto &t : a.last_feed_keys) {
                    out.add(nb::make_tuple(
                        std::get<0>(t), std::get<1>(t), std::get<2>(t)));
                }
                return out;
            },
            [](MacroNicheArchive &a, nb::object obj) {
                a.last_feed_keys.clear();
                if (obj.is_none()) {
                    return;
                }
                for (nb::handle item : nb::iter(obj)) {
                    nb::sequence seq = nb::cast<nb::sequence>(item);
                    a.last_feed_keys.emplace(
                        nb::cast<double>(seq[0]),
                        nb::cast<double>(seq[1]),
                        nb::cast<double>(seq[2]));
                }
            });

    nb::class_<DecisionArena>(m, "DecisionArena")
        .def(nb::init<>())
        .def("root_id", &DecisionArena::root_id)
        .def("add_node", &DecisionArena::add_node, nb::arg("parent_id"), nb::arg("action"))
        .def(
            "snapshot",
            static_cast<BoardSnapshot &(DecisionArena::*)(int32_t)>(&DecisionArena::snapshot),
            nb::arg("id"),
            nb::rv_policy::reference_internal)
        .def(
            "set_snapshot",
            &DecisionArena::set_snapshot,
            nb::arg("id"),
            nb::arg("snap"))
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
            "amaf_misses",
            &DecisionArena::amaf_misses,
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

    nb::class_<AttachNode>(m, "AttachNode")
        .def(nb::init<>())
        .def_rw("a", &AttachNode::a)
        .def_rw("b", &AttachNode::b)
        .def_rw("realized", &AttachNode::realized);

    nb::class_<MotifJoin>(m, "MotifJoin")
        .def(nb::init<>())
        .def_rw("motif_id", &MotifJoin::motif_id)
        .def_rw("a", &MotifJoin::a)
        .def_rw("b", &MotifJoin::b)
        .def_rw("realized", &MotifJoin::realized);

    nb::class_<DecisionGraph>(m, "DecisionGraph")
        .def(nb::init<>())
        .def(
            "macros",
            static_cast<DecisionArena &(DecisionGraph::*)()>(&DecisionGraph::macros),
            nb::rv_policy::reference_internal)
        .def(
            "poses",
            static_cast<PoseGraph &(DecisionGraph::*)()>(&DecisionGraph::poses),
            nb::rv_policy::reference_internal)
        .def(
            "replace_poses",
            &DecisionGraph::replace_poses,
            nb::arg("g"))
        .def("set_pose_kind", &DecisionGraph::set_pose_kind, nb::arg("i"), nb::arg("kind"))
        .def(
            "set_pose_kinds",
            [](DecisionGraph &dg, const std::vector<int> &kinds) {
                std::vector<uint8_t> out;
                out.reserve(kinds.size());
                for (int k : kinds) {
                    out.push_back(static_cast<uint8_t>(k));
                }
                dg.set_pose_kinds(out);
            },
            nb::arg("kinds"))
        .def("add_attach", &DecisionGraph::add_attach, nb::arg("a"), nb::arg("b"))
        .def(
            "add_motif_join",
            &DecisionGraph::add_motif_join,
            nb::arg("motif_id"),
            nb::arg("a"),
            nb::arg("b"))
        .def(
            "attach_conflicts",
            &DecisionGraph::attach_conflicts,
            nb::arg("i"),
            nb::arg("j"))
        .def("mutex_n", &DecisionGraph::mutex_n)
        .def("attach_n", &DecisionGraph::attach_n)
        .def("motif_n", &DecisionGraph::motif_n)
        .def("kind_tagged_n", &DecisionGraph::kind_tagged_n)
        .def(
            "materialize_selection",
            [](DecisionGraph &dg, const std::vector<int> &selected) {
                std::vector<Tvertex> verts;
                verts.reserve(selected.size());
                for (int v : selected) {
                    verts.push_back(static_cast<Tvertex>(v));
                }
                const MaterializeStats st = dg.materialize_selection(verts);
                nb::dict out;
                out["materialized_attach"] = st.materialized_attach;
                out["materialized_motif"] = st.materialized_motif;
                out["member_hits"] = st.member_hits;
                nb::list kind_survive;
                for (int i = 0; i < 4; ++i) {
                    kind_survive.append(st.kind_count[i]);
                }
                out["kind_survive"] = kind_survive;
                return out;
            },
            nb::arg("selected"))
        .def_prop_ro(
            "pose_kind",
            [](const DecisionGraph &dg) {
                std::vector<int> out;
                out.reserve(dg.pose_kind().size());
                for (uint8_t k : dg.pose_kind()) {
                    out.push_back(static_cast<int>(k));
                }
                return out;
            })
        .def_prop_ro(
            "attach",
            [](const DecisionGraph &dg) { return dg.attach(); })
        .def_prop_ro(
            "motifs",
            [](const DecisionGraph &dg) { return dg.motifs(); });

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
