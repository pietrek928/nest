#include <chrono>
#include <cstdint>
#include <string>
#include <tuple>
#include <vector>

#include <nanobind/nanobind.h>
namespace nb = nanobind;
#include <nanobind/stl/string.h>
#include <nanobind/stl/tuple.h>
#include <nanobind/stl/vector.h>

#include "bindings.h"
#include "decision/mcts_agent.h"
#include "decision_arena.h"
#include "macro_niche_archive.h"
#include "motif_base.h"

nb::dict init_mcts_telem_dict() {
    nb::dict d;
    d["pw_expand"] = 0;
    d["amaf_hits"] = 0;
    d["expand_ms"] = 0.0;
    d["from_shapely_count"] = 0;
    d["amaf_miss"] = 0;
    d["browse_jump"] = 0;
    d["tombstone_n"] = 0;
    d["cohort_sig_amaf_visits"] = 0;
    d["mcts_cohort_macro_n"] = 0;
    return d;
}

nb::dict init_mcts_realized_dict() {
    nb::dict d;
    d["kind"] = nb::make_tuple(0, 0, 0, 0);
    d["attach"] = 0;
    d["member_hits"] = 0;
    d["sel_n"] = 0;
    return d;
}

void sync_telem_to_agent(MctsAgent &agent, const nb::dict &telem) {
    MctsTelem &t = agent.telem;
    auto get_i = [&](const char *k, int32_t def) -> int32_t {
        if (!telem.contains(k)) {
            return def;
        }
        return nb::cast<int32_t>(telem[k]);
    };
    auto get_d = [&](const char *k, double def) -> double {
        if (!telem.contains(k)) {
            return def;
        }
        return nb::cast<double>(telem[k]);
    };
    auto get_f = [&](const char *k, float def) -> float {
        if (!telem.contains(k)) {
            return def;
        }
        return nb::cast<float>(telem[k]);
    };
    t.pw_expand = get_i("pw_expand", t.pw_expand);
    t.amaf_hits = get_i("amaf_hits", t.amaf_hits);
    t.expand_ms = get_d("expand_ms", t.expand_ms);
    t.from_shapely_count = get_i("from_shapely_count", t.from_shapely_count);
    t.amaf_miss = get_i("amaf_miss", t.amaf_miss);
    t.browse_jump = get_i("browse_jump", t.browse_jump);
    t.tombstone_n = get_i("tombstone_n", t.tombstone_n);
    t.cohort_sig_amaf_visits = get_i("cohort_sig_amaf_visits", t.cohort_sig_amaf_visits);
    t.mcts_cohort_macro_n = get_i("mcts_cohort_macro_n", t.mcts_cohort_macro_n);
    t.related_warm = get_i("related_warm", t.related_warm);
    t.nearest_warm = get_i("nearest_warm", t.nearest_warm);
    t.amaf_pick = get_i("amaf_pick", t.amaf_pick);
    t.amaf_survive_scale = get_f("amaf_survive_scale", t.amaf_survive_scale);
}

void sync_telem_from_agent(nb::dict &telem, const MctsAgent &agent) {
    const MctsTelem &t = agent.telem;
    telem["pw_expand"] = t.pw_expand;
    telem["amaf_hits"] = t.amaf_hits;
    telem["expand_ms"] = t.expand_ms;
    telem["from_shapely_count"] = t.from_shapely_count;
    telem["amaf_miss"] = t.amaf_miss;
    telem["browse_jump"] = t.browse_jump;
    telem["tombstone_n"] = t.tombstone_n;
    telem["cohort_sig_amaf_visits"] = t.cohort_sig_amaf_visits;
    telem["mcts_cohort_macro_n"] = t.mcts_cohort_macro_n;
    if (t.related_warm > 0) {
        telem["related_warm"] = t.related_warm;
    }
    if (t.nearest_warm > 0) {
        telem["nearest_warm"] = t.nearest_warm;
    }
    if (t.amaf_pick > 0) {
        telem["amaf_pick"] = t.amaf_pick;
    }
    if (t.amaf_survive_scale > 0.f) {
        telem["amaf_survive_scale"] = t.amaf_survive_scale;
    }
}

void sync_realized_to_agent(MctsAgent &agent, const nb::dict &realized) {
    MctsRealized &r = agent.realized;
    if (realized.contains("kind")) {
        nb::sequence kind = nb::cast<nb::sequence>(realized["kind"]);
        for (int i = 0; i < 4 && i < static_cast<int>(nb::len(kind)); ++i) {
            r.kind[static_cast<std::size_t>(i)] = nb::cast<int32_t>(kind[i]);
        }
    }
    if (realized.contains("attach")) {
        r.attach = nb::cast<int32_t>(realized["attach"]);
    }
    if (realized.contains("member_hits")) {
        r.member_hits = nb::cast<int32_t>(realized["member_hits"]);
    }
    if (realized.contains("sel_n")) {
        r.sel_n = nb::cast<int32_t>(realized["sel_n"]);
    }
    if (realized.contains("materialized_motif")) {
        r.materialized_motif = nb::cast<int32_t>(realized["materialized_motif"]);
    }
    if (realized.contains("survive_motif_n")) {
        r.survive_motif_n = nb::cast<int32_t>(realized["survive_motif_n"]);
    }
    if (realized.contains("macro_survive_n")) {
        r.macro_survive_n = nb::cast<int32_t>(realized["macro_survive_n"]);
    }
    if (realized.contains("proposer_pb")) {
        r.proposer_pb = nb::cast<float>(realized["proposer_pb"]);
    }
    r.survive_by_motif.clear();
    if (realized.contains("survive_by_motif")) {
        nb::dict sm = nb::cast<nb::dict>(realized["survive_by_motif"]);
        for (auto item : sm) {
            r.survive_by_motif[nb::cast<int32_t>(item.first)] = nb::cast<int32_t>(item.second);
        }
    }
}

CohortSigKey cohort_sig_from_py(nb::handle obj) {
    CohortSigKey sig;
    if (obj.is_none() || nb::len(obj) == 0) {
        return sig;
    }
    nb::sequence seq = nb::cast<nb::sequence>(obj);
    if (nb::len(seq) < 2) {
        return sig;
    }
    sig.motif_id = nb::cast<int32_t>(seq[0]);
    sig.leader_gid = nb::cast<int32_t>(seq[1]);
    if (nb::len(seq) >= 3 && !seq[2].is_none()) {
        nb::sequence lk = nb::cast<nb::sequence>(seq[2]);
        if (nb::len(lk) >= 3) {
            sig.key_kind = 1;
            sig.lk_x = nb::cast<float>(lk[0]);
            sig.lk_y = nb::cast<float>(lk[1]);
            sig.lk_a = nb::cast<float>(lk[2]);
        } else {
            sig.key_kind = 0;
        }
    } else {
        sig.key_kind = 0;
    }
    return sig;
}

ActionKey action_key_from_py_tuple(nb::handle obj) {
    nb::sequence seq = nb::cast<nb::sequence>(obj);
    ActionKey key;
    key.region_i = nb::cast<int32_t>(seq[0]);
    key.rule_id = nb::cast<int32_t>(seq[1]);
    key.motif_id = nb::cast<int32_t>(seq[2]);
    if (nb::len(seq) >= 4) {
        key.cohort = cohort_sig_from_py(seq[3]);
    }
    return key;
}

nb::object action_key_to_py(const ActionKey &key) {
    nb::object cohort;
    if (!key.cohort.active()) {
        cohort = nb::make_tuple();
    } else if (key.cohort.key_kind == 1) {
        cohort = nb::make_tuple(
            key.cohort.motif_id,
            key.cohort.leader_gid,
            nb::make_tuple(key.cohort.lk_x, key.cohort.lk_y, key.cohort.lk_a));
    } else {
        cohort = nb::make_tuple(key.cohort.motif_id, key.cohort.leader_gid, nb::none());
    }
    return nb::make_tuple(key.region_i, key.rule_id, key.motif_id, cohort);
}

std::vector<MotifCohortEntry> motif_cohorts_from_py(nb::object obj) {
    std::vector<MotifCohortEntry> out;
    if (obj.is_none()) {
        return out;
    }
    for (nb::handle item : nb::iter(obj)) {
        if (!nb::isinstance<nb::dict>(item)) {
            continue;
        }
        nb::dict c = nb::cast<nb::dict>(item);
        MotifCohortEntry entry;
        entry.motif_id = c.contains("motif_id") ? nb::cast<int32_t>(c["motif_id"]) : -1;
        entry.leader_gid = c.contains("leader_gid") ? nb::cast<int32_t>(c["leader_gid"]) : -1;
        nb::object members = c.contains("member_keys") ? c["member_keys"] : nb::list();
        entry.member_keys_count = static_cast<int32_t>(nb::len(members));
        if (entry.member_keys_count < 2) {
            continue;
        }
        if (c.contains("leader_key") && !c["leader_key"].is_none()) {
            nb::sequence lk_seq = nb::cast<nb::sequence>(c["leader_key"]);
            if (nb::len(lk_seq) >= 3) {
                entry.has_leader_key = true;
                entry.leader_key_x = nb::cast<float>(lk_seq[0]);
                entry.leader_key_y = nb::cast<float>(lk_seq[1]);
                entry.leader_key_theta = nb::cast<float>(lk_seq[2]);
            }
        }
        out.push_back(entry);
    }
    return out;
}

struct MctsAgentHolder {
    MctsAgent agent;
    nb::dict telem;
    nb::dict realized;

    MctsAgentHolder(DecisionArena &arena, MotifBase &motif_base, MacroNicheArchive *niche)
        : agent(arena, motif_base, niche)
        , telem(init_mcts_telem_dict())
        , realized(init_mcts_realized_dict()) {}

    void pull_state() {
        sync_telem_to_agent(agent, telem);
        sync_realized_to_agent(agent, realized);
    }

    void push_state() { sync_telem_from_agent(telem, agent); }
};

void bind_graph_mcts(nb::module_ &m) {
    m.def(
        "coverage_delta",
        [](const BoardSnapshot &child, const BoardSnapshot &parent) {
            return coverage_delta(child, parent);
        },
        nb::arg("child"),
        nb::arg("parent"));

    m.def(
        "path_reward_beats",
        [](const BoardSnapshot &parent,
           const BoardSnapshot &child,
           float base_reward,
           float alt_reward,
           float min_abs_cov,
           float min_reward_eps) {
            return path_reward_beats(
                parent, child, base_reward, alt_reward, min_abs_cov, min_reward_eps);
        },
        nb::arg("parent"),
        nb::arg("child"),
        nb::arg("base_reward"),
        nb::arg("alt_reward"),
        nb::arg("min_abs_cov") = 0.005f,
        nb::arg("min_reward_eps") = 0.01f);

    m.def(
        "leaf_reward",
        [](const BoardSnapshot &snapshot,
           float lam_kiss,
           float lam_comp,
           float lam_void,
           float lam_rim,
           int32_t rule_id,
           int32_t member_hits,
           int32_t materialized_motif,
           int32_t survive_motif_n,
           int32_t macro_survive_n) {
            return leaf_reward(
                snapshot,
                lam_kiss,
                lam_comp,
                lam_void,
                lam_rim,
                rule_id,
                member_hits,
                materialized_motif,
                survive_motif_n,
                macro_survive_n);
        },
        nb::arg("snapshot"),
        nb::arg("lam_kiss") = 0.05f,
        nb::arg("lam_comp") = 0.05f,
        nb::arg("lam_void") = 0.5f,
        nb::arg("lam_rim") = 0.1f,
        nb::arg("rule_id") = -1,
        nb::arg("member_hits") = 0,
        nb::arg("materialized_motif") = 0,
        nb::arg("survive_motif_n") = 0,
        nb::arg("macro_survive_n") = 0);

    m.def(
        "timed_expand_ms",
        [](nb::dict telem, double t0) {
            nb::module_ time_mod = nb::module_::import_("time");
            const double t_now = nb::cast<double>(time_mod.attr("perf_counter")());
            const double prev =
                telem.contains("expand_ms") ? nb::cast<double>(telem["expand_ms"]) : 0.0;
            telem["expand_ms"] = prev + (t_now - t0) * 1000.0;
        },
        nb::arg("telem"),
        nb::arg("t0"));

    nb::class_<MctsAgentHolder>(m, "MctsAgent")
        .def(
            "__init__",
            [](MctsAgentHolder *self,
               DecisionArena &arena,
               MotifBase &motif_base,
               nb::object niche_archive) {
                MacroNicheArchive *niche = nullptr;
                if (!niche_archive.is_none()) {
                    niche = nb::cast<MacroNicheArchive *>(niche_archive);
                }
                new (self) MctsAgentHolder(arena, motif_base, niche);
            },
            nb::arg("arena"),
            nb::arg("motif_base"),
            nb::arg("niche_archive") = nb::none())
        .def_prop_ro(
            "arena",
            [](MctsAgentHolder &h) -> DecisionArena & { return *h.agent.arena; },
            nb::rv_policy::reference_internal)
        .def_prop_ro(
            "motif_base",
            [](MctsAgentHolder &h) -> MotifBase & { return *h.agent.motif_base; },
            nb::rv_policy::reference_internal)
        .def_prop_ro(
            "niche_archive",
            [](MctsAgentHolder &h) -> nb::object {
                if (h.agent.niche_archive == nullptr) {
                    return nb::none();
                }
                return nb::cast(h.agent.niche_archive, nb::rv_policy::reference_internal);
            })
        .def_prop_rw(
            "telem",
            [](MctsAgentHolder &h) -> nb::dict & { return h.telem; },
            [](MctsAgentHolder &h, nb::dict d) { h.telem = std::move(d); })
        .def_prop_rw(
            "realized",
            [](MctsAgentHolder &h) -> nb::dict & { return h.realized; },
            [](MctsAgentHolder &h, nb::dict d) { h.realized = std::move(d); })
        .def_prop_rw(
            "motif_cohorts",
            [](MctsAgentHolder &h) -> nb::object {
                nb::list out;
                for (const MotifCohortEntry &c : h.agent.motif_cohorts) {
                    nb::dict d;
                    d["motif_id"] = c.motif_id;
                    d["leader_gid"] = c.leader_gid;
                    d["member_keys"] = nb::make_tuple(0, 1);
                    if (c.has_leader_key) {
                        d["leader_key"] = nb::make_tuple(
                            c.leader_key_x, c.leader_key_y, c.leader_key_theta);
                    }
                    out.append(d);
                }
                return nb::tuple(out);
            },
            [](MctsAgentHolder &h, nb::object obj) {
                h.agent.motif_cohorts = motif_cohorts_from_py(obj);
            })
        .def_prop_rw(
            "pw_c",
            [](MctsAgentHolder &h) { return h.agent.pw_c; },
            [](MctsAgentHolder &h, float v) { h.agent.pw_c = v; })
        .def_prop_rw(
            "pw_alpha",
            [](MctsAgentHolder &h) { return h.agent.pw_alpha; },
            [](MctsAgentHolder &h, float v) { h.agent.pw_alpha = v; })
        .def_prop_rw(
            "ucb_c",
            [](MctsAgentHolder &h) { return h.agent.ucb_c; },
            [](MctsAgentHolder &h, float v) { h.agent.ucb_c = v; })
        .def_prop_rw(
            "expand_frozen",
            [](MctsAgentHolder &h) { return h.agent.expand_frozen; },
            [](MctsAgentHolder &h, bool v) { h.agent.expand_frozen = v; })
        .def_prop_rw(
            "prior_motif_graph_hit_n",
            [](MctsAgentHolder &h) { return h.agent.prior_motif_graph_hit_n; },
            [](MctsAgentHolder &h, int32_t v) { h.agent.prior_motif_graph_hit_n = v; })
        .def_prop_ro(
            "_tombstoned",
            [](MctsAgentHolder &h) {
                nb::set out;
                for (int32_t nid : h.agent.tombstoned()) {
                    out.add(nid);
                }
                return out;
            })
        .def(
            "may_expand_node",
            [](MctsAgentHolder &h, int32_t node_id) { return h.agent.may_expand_node(node_id); },
            nb::arg("node_id"))
        .def(
            "select_leaf",
            [](MctsAgentHolder &h) {
                h.pull_state();
                const int32_t leaf = h.agent.select_leaf();
                h.push_state();
                return leaf;
            })
        .def(
            "expand",
            [](MctsAgentHolder &h, int32_t parent_id, const MacroAction &action, float reward) {
                h.pull_state();
                const int32_t cid = h.agent.expand(parent_id, action, reward);
                h.push_state();
                return cid;
            },
            nb::arg("parent_id"),
            nb::arg("action"),
            nb::arg("reward"))
        .def(
            "backprop",
            [](MctsAgentHolder &h, int32_t node_id, float reward) {
                h.pull_state();
                h.agent.backprop(node_id, reward);
                h.push_state();
            },
            nb::arg("node_id"),
            nb::arg("reward"))
        .def(
            "note_macro_miss",
            [](MctsAgentHolder &h, nb::object action) {
                h.pull_state();
                if (action.is_none()) {
                    h.agent.note_macro_miss(nullptr);
                } else {
                    const MacroAction &a = nb::cast<const MacroAction &>(action);
                    h.agent.note_macro_miss(&a);
                }
                h.push_state();
            },
            nb::arg("action") = nb::none())
        .def(
            "best_child",
            [](MctsAgentHolder &h, nb::object node_id) {
                if (node_id.is_none()) {
                    return h.agent.best_child();
                }
                return h.agent.best_child(nb::cast<int32_t>(node_id));
            },
            nb::arg("node_id") = nb::none())
        .def("deepest_best_child", [](MctsAgentHolder &h) { return h.agent.deepest_best_child(); })
        .def(
            "ancestor_chain",
            [](MctsAgentHolder &h, int32_t node_id) {
                nb::set out;
                for (int32_t nid : h.agent.ancestor_chain(node_id)) {
                    out.add(nid);
                }
                return out;
            },
            nb::arg("node_id"))
        .def(
            "age_and_tombstone",
            [](MctsAgentHolder &h, int32_t spine_id, int32_t idle_t, bool force) {
                h.pull_state();
                const int32_t dropped = h.agent.age_and_tombstone(spine_id, idle_t, force);
                h.push_state();
                return dropped;
            },
            nb::arg("spine_id"),
            nb::kw_only(),
            nb::arg("idle_t") = 4,
            nb::arg("force") = false)
        .def(
            "remember_related",
            [](MctsAgentHolder &h, const BoardSnapshot &snapshot, bool allow) {
                h.agent.remember_related(snapshot, allow);
            },
            nb::arg("snapshot"),
            nb::kw_only(),
            nb::arg("allow") = true)
        .def(
            "pick_expand_action",
            [](MctsAgentHolder &h,
               const std::vector<int32_t> &remaining_gids,
               nb::object rule_ids,
               nb::object parent_id,
               nb::object snapshot) -> nb::object {
                h.pull_state();
                std::vector<int32_t> rules{0};
                if (!rule_ids.is_none()) {
                    rules = nb::cast<std::vector<int32_t>>(rule_ids);
                }
                int32_t pid = -1;
                if (!parent_id.is_none()) {
                    pid = nb::cast<int32_t>(parent_id);
                }
                const BoardSnapshot *snap_ptr = nullptr;
                BoardSnapshot snap_storage;
                if (!snapshot.is_none()) {
                    snap_storage = nb::cast<BoardSnapshot>(snapshot);
                    snap_ptr = &snap_storage;
                }
                const MacroAction *action = h.agent.pick_expand_action(
                    remaining_gids, rules, pid, snap_ptr);
                h.push_state();
                if (action == nullptr) {
                    return nb::object(nb::none());
                }
                return nb::object(nb::cast(*action));
            },
            nb::arg("remaining_gids"),
            nb::kw_only(),
            nb::arg("rule_ids") = nb::none(),
            nb::arg("parent_id") = nb::none(),
            nb::arg("snapshot") = nb::none())
        .def(
            "_ucb",
            [](MctsAgentHolder &h, int32_t node_id, int32_t parent_visits) {
                h.pull_state();
                const float score = h.agent.ucb(node_id, parent_visits);
                h.push_state();
                return score;
            },
            nb::arg("node_id"),
            nb::arg("parent_visits"))
        .def(
            "_warm_motif_ids",
            [](MctsAgentHolder &h, nb::object snapshot) {
                h.pull_state();
                BoardSnapshot snap_storage;
                const BoardSnapshot *snap_ptr = nullptr;
                if (!snapshot.is_none()) {
                    snap_storage = nb::cast<BoardSnapshot>(snapshot);
                    snap_ptr = &snap_storage;
                }
                const std::vector<int32_t> warm = h.agent.warm_motif_ids(snap_ptr);
                h.push_state();
                return warm;
            },
            nb::arg("snapshot"))
        .def(
            "_action_key",
            [](MctsAgentHolder &h, const MacroAction &action) {
                return action_key_to_py(h.agent.action_key(action));
            },
            nb::arg("action"))
        .def(
            "_amaf_pick_score",
            [](MctsAgentHolder &h, nb::object key, int32_t parent_id, std::string free_kind) {
                h.pull_state();
                const float score = h.agent.amaf_pick_score(
                    action_key_from_py_tuple(key), parent_id, free_kind);
                h.push_state();
                return score;
            },
            nb::arg("key"),
            nb::arg("parent_id"),
            nb::kw_only(),
            nb::arg("free_kind") = std::string{});
}
