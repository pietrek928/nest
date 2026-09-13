#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "decision/action_gen.h"
#include "decision_arena.h"
#include "macro_niche_archive.h"
#include "motif_base.h"
#include "node_signature.h"
#include "se2.h"

constexpr float MCTS_LAMBDA_MISS = 0.5f;
constexpr float MCTS_NEG_INF = -1e30f;
constexpr float MCTS_POS_INF_THRESH = 1e30f;

struct MctsRealized {
    std::array<int32_t, 4> kind{0, 0, 0, 0};
    int32_t attach = 0;
    int32_t member_hits = 0;
    int32_t sel_n = 0;
    int32_t materialized_motif = 0;
    int32_t survive_motif_n = 0;
    int32_t macro_survive_n = 0;
    float proposer_pb = 0.f;
    std::unordered_map<int32_t, int32_t> survive_by_motif;
};

struct MctsTelem {
    int32_t pw_expand = 0;
    int32_t amaf_hits = 0;
    double expand_ms = 0.0;
    int32_t from_shapely_count = 0;
    int32_t amaf_miss = 0;
    int32_t browse_jump = 0;
    int32_t tombstone_n = 0;
    int32_t cohort_sig_amaf_visits = 0;
    int32_t mcts_cohort_macro_n = 0;
    int32_t related_warm = 0;
    int32_t nearest_warm = 0;
    int32_t amaf_pick = 0;
    float amaf_survive_scale = 0.f;
};

struct MotifCohortEntry {
    int32_t motif_id = -1;
    int32_t leader_gid = -1;
    int32_t member_keys_count = 0;
    bool has_leader_key = false;
    float leader_key_x = 0.f;
    float leader_key_y = 0.f;
    float leader_key_theta = 0.f;
};

struct CohortSigKey {
    int32_t motif_id = -1;
    int32_t leader_gid = -1;
    // -1 = inactive, 0 = active without leader_key, 1 = active with leader_key
    int8_t key_kind = -1;
    float lk_x = 0.f;
    float lk_y = 0.f;
    float lk_a = 0.f;

    bool active() const { return key_kind >= 0; }

    bool operator==(const CohortSigKey &o) const {
        if (key_kind != o.key_kind) {
            return false;
        }
        if (key_kind < 0) {
            return true;
        }
        if (motif_id != o.motif_id || leader_gid != o.leader_gid) {
            return false;
        }
        if (key_kind == 0) {
            return true;
        }
        return lk_x == o.lk_x && lk_y == o.lk_y && lk_a == o.lk_a;
    }
};

struct ActionKey {
    int32_t region_i = 0;
    int32_t rule_id = 0;
    int32_t motif_id = -1;
    CohortSigKey cohort{};

    bool operator==(const ActionKey &o) const {
        return region_i == o.region_i && rule_id == o.rule_id && motif_id == o.motif_id
            && cohort == o.cohort;
    }
};

struct ActionKeyHash {
    std::size_t operator()(const ActionKey &k) const {
        std::size_t h = static_cast<std::size_t>(k.region_i);
        h = h * 1315423911u + static_cast<std::size_t>(k.rule_id);
        h = h * 1315423911u + static_cast<std::size_t>(k.motif_id + 2);
        h = h * 1315423911u + static_cast<std::size_t>(k.cohort.key_kind + 2);
        h = h * 1315423911u + static_cast<std::size_t>(k.cohort.motif_id + 2);
        h = h * 1315423911u + static_cast<std::size_t>(k.cohort.leader_gid + 2);
        if (k.cohort.key_kind == 1) {
            h = h * 1315423911u + static_cast<std::size_t>(k.cohort.lk_x * 1000.f);
            h = h * 1315423911u + static_cast<std::size_t>(k.cohort.lk_y * 1000.f);
            h = h * 1315423911u + static_cast<std::size_t>(k.cohort.lk_a * 1000.f);
        }
        return h;
    }
};

struct CohortAmafSlot {
    int32_t visits = 0;
    float reward = 0.f;
    int32_t misses = 0;
};

inline MctsRealized empty_mcts_realized() {
    return MctsRealized{};
}

inline float coverage_delta(const BoardSnapshot &child, const BoardSnapshot &parent) {
    const float pc = std::max(parent.coverage, 1e-9f);
    const float cc = child.coverage;
    return (cc - pc) / pc;
}

inline bool path_reward_beats(
    const BoardSnapshot &parent,
    const BoardSnapshot &child,
    float base_reward,
    float alt_reward,
    float min_abs_cov = 0.005f,
    float min_reward_eps = 0.01f
) {
    if (alt_reward <= base_reward + min_reward_eps) {
        return false;
    }
    const float bc = parent.coverage;
    const float cc = child.coverage;
    if (cc + 1e-9f >= bc + min_abs_cov) {
        return true;
    }
    return coverage_delta(child, parent) >= min_abs_cov / std::max(bc, 1e-9f);
}

inline float leaf_reward(
    const BoardSnapshot &snapshot,
    float lam_kiss = 0.05f,
    float lam_comp = 0.05f,
    float lam_void = 0.5f,
    float lam_rim = 0.1f,
    int32_t rule_id = -1,
    int32_t member_hits = 0,
    int32_t materialized_motif = 0,
    int32_t survive_motif_n = 0,
    int32_t macro_survive_n = 0
) {
    const int32_t packed = std::max(snapshot.n_packed(), 1);
    float void_w = lam_void;
    float rim_w = lam_rim;
    if (snapshot.free_kind == "large_void") {
        void_w = std::max(void_w, 0.75f);
        rim_w = std::min(rim_w, 0.05f);
    }
    float base = snapshot.coverage + void_w * snapshot.void_fill + rim_w * snapshot.rim_fill
        + lam_kiss * (static_cast<float>(snapshot.kiss_pairs) / static_cast<float>(packed))
        + lam_comp * snapshot.mean_compactness;
    if (rule_id > 0) {
        base += 0.02f * std::min(static_cast<float>(rule_id), 3.f);
    }
    const int32_t mh = std::max(member_hits, 0);
    const int32_t mm = std::max(materialized_motif, 0);
    if (mh > 0) {
        base += 0.03f * std::min(static_cast<float>(mh), 12.f) / 12.f;
    }
    if (mm > 0) {
        base += 0.05f * std::min(static_cast<float>(mm), 4.f) / 4.f;
    }
    const int32_t sn = std::max(survive_motif_n, 0);
    const int32_t msn = std::max(macro_survive_n, 0);
    if (sn > 0) {
        base += 0.04f * std::min(static_cast<float>(sn), 8.f) / 8.f;
    }
    if (msn > 0) {
        base += 0.06f * std::min(static_cast<float>(msn), 4.f) / 4.f;
    }
    return base;
}

inline void timed_expand_ms(MctsTelem &telem, double elapsed_ms) {
    telem.expand_ms += elapsed_ms;
}

class MctsAgent {
public:
    DecisionArena *arena = nullptr;
    MotifBase *motif_base = nullptr;
    MacroNicheArchive *niche_archive = nullptr;

    float pw_c = 1.5f;
    float pw_alpha = 0.5f;
    float ucb_c = 1.4f;
    bool expand_frozen = false;

    MctsTelem telem{};
    MctsRealized realized = empty_mcts_realized();
    std::vector<MotifCohortEntry> motif_cohorts;
    // M2b: gate PLACE_COHORT expand; cohorts stay populated for AMAF/soft path.
    bool place_cohort_ready = false;
    int32_t prior_motif_graph_hit_n = 0;

    MctsAgent() = default;

    MctsAgent(
        DecisionArena &arena_ref,
        MotifBase &motif_ref,
        MacroNicheArchive *niche = nullptr
    )
        : arena(&arena_ref), motif_base(&motif_ref), niche_archive(niche) {}

    const std::unordered_set<int32_t> &tombstoned() const { return tombstoned_; }

    CohortSigKey cohort_sig_for_action(const MacroAction &action) const {
        const int32_t mid = action.motif_id;
        if (mid < 0) {
            return CohortSigKey{};
        }
        const int32_t leader = action.part_gid;
        for (const MotifCohortEntry &c : motif_cohorts) {
            if (c.motif_id != mid || c.leader_gid != leader) {
                continue;
            }
            CohortSigKey sig;
            sig.motif_id = mid;
            sig.leader_gid = leader;
            if (c.has_leader_key) {
                sig.key_kind = 1;
                sig.lk_x = c.leader_key_x;
                sig.lk_y = c.leader_key_y;
                sig.lk_a = c.leader_key_theta;
            } else {
                sig.key_kind = 0;
            }
            return sig;
        }
        CohortSigKey sig;
        sig.motif_id = mid;
        sig.leader_gid = leader;
        sig.key_kind = 0;
        return sig;
    }

    ActionKey action_key(const MacroAction &action) const {
        ActionKey key;
        key.region_i = static_cast<int32_t>(action.region);
        key.rule_id = action.rule_id;
        key.motif_id = action.motif_id;
        key.cohort = cohort_sig_for_action(action);
        return key;
    }

    void record_cohort_amaf(const ActionKey &key, float reward, bool miss) {
        if (!key.cohort.active()) {
            return;
        }
        CohortAmafSlot &slot = cohort_amaf_[key];
        slot.visits += 1;
        slot.reward += reward;
        if (miss) {
            slot.misses += 1;
        }
        telem.cohort_sig_amaf_visits = std::max(
            telem.cohort_sig_amaf_visits,
            slot.visits
        );
    }

    bool is_tombstoned(int32_t node_id) const {
        return tombstoned_.count(node_id) > 0;
    }

    bool may_expand_node(int32_t node_id) const {
        if (expand_frozen || arena == nullptr) {
            return false;
        }
        return arena->may_expand(node_id, pw_c, pw_alpha);
    }

    float ucb(int32_t node_id, int32_t parent_visits) {
        if (is_tombstoned(node_id) || arena == nullptr) {
            return MCTS_NEG_INF;
        }
        const MacroAction &action = arena->node(node_id).action;
        const int32_t region_i = static_cast<int32_t>(action.region);
        if (arena->amaf_visits(region_i, action.rule_id, action.motif_id) > 0) {
            telem.amaf_hits += 1;
        }
        float score = arena->ucb_score(node_id, parent_visits, ucb_c);
        if (!std::isfinite(score) || score > MCTS_POS_INF_THRESH) {
            return std::numeric_limits<float>::infinity();
        }
        return score;
    }

    int32_t select_leaf() {
        if (arena == nullptr) {
            return -1;
        }
        int32_t node_id = arena->root_id();
        while (true) {
            const int32_t first = arena->node(node_id).first_child_id;
            if (first < 0) {
                return node_id;
            }
            const int32_t parent_visits = std::max(arena->node(node_id).visits, 1);
            int32_t best_id = first;
            float best_score = MCTS_NEG_INF;
            int32_t child = first;
            bool any_live = false;
            while (child >= 0) {
                if (!is_tombstoned(child)) {
                    any_live = true;
                    const float score = ucb(child, parent_visits);
                    if (score > best_score) {
                        best_score = score;
                        best_id = child;
                    }
                }
                child = arena->node(child).next_sibling_id;
            }
            if (!any_live) {
                return node_id;
            }
            if (may_expand_node(node_id)) {
                return node_id;
            }
            node_id = best_id;
        }
    }

    int32_t expand(int32_t parent_id, const MacroAction &action, float reward) {
        if (arena == nullptr) {
            return parent_id;
        }
        if (expand_frozen) {
            backprop(parent_id, reward);
            return parent_id;
        }
        const int32_t child = arena->add_node(parent_id, action);
        backprop(child, reward);
        telem.pw_expand += 1;
        idle_age_[child] = 0;
        return child;
    }

    void backprop(int32_t node_id, float reward) {
        if (arena == nullptr) {
            return;
        }
        int32_t cur = node_id;
        while (cur >= 0) {
            arena->record_visit(cur, reward);
            idle_age_[cur] = 0;
            const MacroAction &action = arena->node(cur).action;
            const int32_t region_i = static_cast<int32_t>(action.region);
            arena->amaf_record(region_i, action.rule_id, action.motif_id, reward, false);
            record_cohort_amaf(action_key(action), reward, false);
            cur = arena->node(cur).parent_id;
        }
    }

    void note_macro_miss(const MacroAction *action) {
        if (action == nullptr || arena == nullptr) {
            return;
        }
        const int32_t region_i = static_cast<int32_t>(action->region);
        arena->amaf_record(region_i, action->rule_id, action->motif_id, 0.f, true);
        record_cohort_amaf(action_key(*action), 0.f, true);
        telem.amaf_miss += 1;
    }

    int32_t best_child(int32_t node_id = -1) const {
        if (arena == nullptr) {
            return -1;
        }
        const int32_t root = node_id < 0 ? arena->root_id() : node_id;
        const int32_t first = arena->node(root).first_child_id;
        if (first < 0) {
            return root;
        }
        int32_t best_id = root;
        float best_mean = MCTS_NEG_INF;
        bool found = false;
        int32_t child = first;
        while (child >= 0) {
            if (!is_tombstoned(child)) {
                const int32_t visits = arena->node(child).visits;
                if (visits > 0) {
                    const float mean = arena->node(child).total_reward / static_cast<float>(visits);
                    if (mean > best_mean) {
                        best_mean = mean;
                        best_id = child;
                        found = true;
                    }
                }
            }
            child = arena->node(child).next_sibling_id;
        }
        return found ? best_id : root;
    }

    int32_t deepest_best_child() const {
        if (arena == nullptr) {
            return -1;
        }
        int32_t cur = arena->root_id();
        while (true) {
            const int32_t nxt = best_child(cur);
            if (nxt == cur) {
                return cur;
            }
            cur = nxt;
        }
    }

    std::unordered_set<int32_t> ancestor_chain(int32_t node_id) const {
        std::unordered_set<int32_t> out;
        if (arena == nullptr) {
            return out;
        }
        int32_t cur = node_id;
        while (cur >= 0) {
            out.insert(cur);
            cur = arena->node(cur).parent_id;
        }
        return out;
    }

    int32_t age_and_tombstone(int32_t spine_id, int32_t idle_t = 4, bool force = false) {
        if (arena == nullptr) {
            return 0;
        }
        std::unordered_set<int32_t> protect = ancestor_chain(spine_id);
        const std::unordered_set<int32_t> best_chain = ancestor_chain(deepest_best_child());
        protect.insert(best_chain.begin(), best_chain.end());
        protect.insert(arena->root_id());
        const int32_t n = arena->size();
        int32_t dropped = 0;
        for (int32_t nid = 0; nid < n; ++nid) {
            if (protect.count(nid) > 0 || is_tombstoned(nid)) {
                continue;
            }
            const int32_t age = idle_age_[nid] + 1;
            idle_age_[nid] = age;
            const int32_t visits = arena->node(nid).visits;
            if (visits < 2 && (age >= idle_t || force)) {
                tombstoned_.insert(nid);
                dropped += 1;
            }
        }
        telem.tombstone_n = static_cast<int32_t>(tombstoned_.size());
        return dropped;
    }

    std::vector<int32_t> warm_motif_ids(const BoardSnapshot *snapshot) {
        std::vector<int32_t> warm;
        if (snapshot == nullptr || motif_base == nullptr) {
            return warm;
        }
        std::unordered_set<int32_t> seen;

        auto add_mid = [&](int32_t mid) {
            if (mid < 0 || seen.count(mid) > 0 || mid >= motif_base->size()) {
                return;
            }
            seen.insert(mid);
            warm.push_back(mid);
        };

        if (!snapshot->packed_gids.empty()) {
            NodeSignature cur;
            cur.rim_fill = snapshot->rim_fill;
            cur.void_fill = snapshot->void_fill;
            float best_d = 1e9f;
            std::vector<int32_t> best_ids;
            for (const BoardSnapshot &snap : related_snaps_) {
                NodeSignature other;
                other.rim_fill = snap.rim_fill;
                other.void_fill = snap.void_fill;
                const float d = related_distance(cur, other);
                if (d < best_d && !snap.motif_ids_used.empty()) {
                    best_d = d;
                    best_ids = snap.motif_ids_used;
                }
            }
            if (best_d < 0.75f) {
                telem.related_warm += 1;
                for (int32_t mid : best_ids) {
                    add_mid(mid);
                }
            }
        }

        const auto &gids = snapshot->packed_gids;
        const auto &tfs = snapshot->packed_transforms;
        if (motif_base->size() > 0 && gids.size() >= 2 && tfs.size() >= 2) {
            const int32_t ga = gids[gids.size() - 2];
            const int32_t gb = gids[gids.size() - 1];
            const Se2 &ta = tfs[tfs.size() - 2];
            const Se2 &tb = tfs[tfs.size() - 1];
            const Se2 rel = se2_relative(ta, tb);
            const int32_t near_id = motif_base->find_nearest_id(ga, gb, rel);
            if (near_id >= 0) {
                add_mid(near_id);
                telem.nearest_warm += 1;
            }
        }
        return warm;
    }

    void remember_related(const BoardSnapshot &snapshot, bool allow = true) {
        if (!allow) {
            return;
        }
        related_snaps_.push_back(snapshot);
        if (related_snaps_.size() > 32) {
            related_snaps_.erase(related_snaps_.begin(), related_snaps_.begin() + (related_snaps_.size() - 32));
        }
    }

    std::unordered_set<ActionKey, ActionKeyHash> tried_action_keys(int32_t parent_id) const {
        std::unordered_set<ActionKey, ActionKeyHash> tried;
        if (arena == nullptr) {
            return tried;
        }
        int32_t child = arena->node(parent_id).first_child_id;
        while (child >= 0) {
            if (!is_tombstoned(child)) {
                tried.insert(action_key(arena->node(child).action));
            }
            child = arena->node(child).next_sibling_id;
        }
        return tried;
    }

    float amaf_pick_score(
        const ActionKey &key,
        int32_t parent_id,
        const std::string &free_kind = ""
    ) {
        if (arena == nullptr) {
            return MCTS_NEG_INF;
        }
        const int32_t parent_visits = std::max(arena->node(parent_id).visits, 0);
        const int32_t region_i = key.region_i;
        const int32_t rule_id = key.rule_id;
        const int32_t motif_id = key.motif_id;
        int32_t visits = arena->amaf_visits(region_i, rule_id, motif_id);
        float mean = 0.f;
        int32_t misses = 0;
        if (key.cohort.active()) {
            auto it = cohort_amaf_.find(key);
            if (it != cohort_amaf_.end() && it->second.visits > 0) {
                visits = it->second.visits;
                mean = it->second.reward / static_cast<float>(visits);
                misses = it->second.misses;
                telem.cohort_sig_amaf_visits = std::max(
                    telem.cohort_sig_amaf_visits,
                    visits
                );
            } else {
                visits = 0;
            }
        } else if (visits <= 0) {
            visits = 0;
        } else {
            mean = arena->amaf_mean(region_i, rule_id, motif_id);
            misses = arena->amaf_misses(region_i, rule_id, motif_id);
        }
        const float c = ucb_c;
        const float pb = c / std::sqrt(static_cast<float>(parent_visits) + 1.f);
        bool survival_live = realized.survive_motif_n > 0;
        if (!survival_live) {
            for (const auto &kv : realized.survive_by_motif) {
                if (kv.second > 0) {
                    survival_live = true;
                    break;
                }
            }
        }
        const float amaf_scale = survival_live ? 0.65f : 1.f;
        if (survival_live) {
            telem.amaf_survive_scale = amaf_scale;
        }
        float score = 0.f;
        if (visits <= 0) {
            score = pb * amaf_scale;
        } else {
            float miss_term = 0.f;
            if (misses > 0) {
                miss_term = MCTS_LAMBDA_MISS * (static_cast<float>(misses) / static_cast<float>(visits));
            }
            score = (mean - miss_term + pb) * amaf_scale;
        }
        const int32_t void_i = static_cast<int32_t>(MacroRegion::Void);
        const int32_t rim_i = static_cast<int32_t>(MacroRegion::Rim);
        const int32_t motif_i = static_cast<int32_t>(MacroRegion::Motif);
        if (free_kind == "large_void" && region_i == void_i) {
            score += 0.35f;
        }
        if (free_kind == "large_void" && region_i == rim_i) {
            score -= 0.15f;
        }
        const int32_t sel_n = std::max(realized.sel_n, 1);
        if (region_i >= 0 && region_i < 4) {
            score += 0.2f * (static_cast<float>(realized.kind[static_cast<std::size_t>(region_i)]) / static_cast<float>(sel_n));
        }
        const int32_t attach_n = realized.attach;
        const int32_t member_hits = realized.member_hits;
        const int32_t mat_motif = realized.materialized_motif;
        int32_t survive_n = 0;
        if (motif_id >= 0) {
            auto sit = realized.survive_by_motif.find(motif_id);
            if (sit != realized.survive_by_motif.end()) {
                survive_n = sit->second;
            }
        }
        if (survive_n <= 0) {
            survive_n = realized.survive_motif_n;
        }
        if (region_i == void_i || region_i == motif_i || region_i == rim_i) {
            score += 0.1f * (static_cast<float>(attach_n) / static_cast<float>(sel_n));
        }
        if (member_hits > 0 && region_i == motif_i) {
            score += 0.08f * std::min(static_cast<float>(member_hits), 16.f) / 16.f;
        }
        if (mat_motif > 0 && region_i == motif_i) {
            score += 0.1f * std::min(static_cast<float>(mat_motif), 4.f) / 4.f;
        }
        if (survive_n > 0 && (region_i == void_i || region_i == motif_i)) {
            score += 0.12f * (static_cast<float>(survive_n) / static_cast<float>(sel_n));
        }
        if (prior_motif_graph_hit_n > 0 && region_i == motif_i) {
            score += 0.05f * std::min(static_cast<float>(prior_motif_graph_hit_n), 4.f) / 4.f;
        }
        if (motif_id >= 0 && motif_base != nullptr && motif_id < motif_base->size()) {
            const MotifRecord &rec = motif_base->at(motif_id);
            score += 0.04f * std::min(static_cast<float>(rec.accept_count), 8.f);
            score += 0.12f * rec.gci;
        }
        if (niche_archive != nullptr) {
            NicheKey nkey{region_i, rule_id, motif_id};
            auto bit = niche_archive->buckets().find(nkey);
            if (bit == niche_archive->buckets().end()) {
                nkey = NicheKey{region_i, 0, motif_id};
                bit = niche_archive->buckets().find(nkey);
            }
            if (bit != niche_archive->buckets().end()) {
                const MacroNicheBucket &raw = bit->second;
                const int32_t tot = raw.hits + raw.misses;
                if (tot > 0) {
                    score -= 0.2f * (static_cast<float>(raw.misses) / static_cast<float>(tot));
                }
            }
        }
        const float prop_h = std::max(0.f, std::min(1.f, realized.proposer_pb));
        if (prop_h > 0.f) {
            const float den = visits > 0 ? static_cast<float>(visits) + 1.f
                                         : static_cast<float>(parent_visits) + 1.f;
            score += prop_h / den;
        }
        return score;
    }

    const MacroAction *pick_expand_action(
        const std::vector<int32_t> &remaining_gids,
        const std::vector<int32_t> &rule_ids,
        int32_t parent_id,
        const BoardSnapshot *snapshot
    ) {
        static MacroAction fallback{};
        std::vector<int32_t> warm;
        std::string free_kind;
        if (snapshot != nullptr) {
            warm = warm_motif_ids(snapshot);
            free_kind = snapshot->free_kind;
        }
        std::vector<MotifCohortSpec> cohort_specs;
        if (place_cohort_ready) {
            cohort_specs.reserve(motif_cohorts.size());
            for (const MotifCohortEntry &c : motif_cohorts) {
                MotifCohortSpec spec;
                spec.motif_id = c.motif_id;
                spec.leader_gid = c.leader_gid;
                spec.member_keys_count = c.member_keys_count;
                cohort_specs.push_back(spec);
            }
        }
        std::vector<MacroAction> actions = generate_macros(
            remaining_gids,
            rule_ids,
            motif_base,
            true,
            warm,
            free_kind,
            cohort_specs
        );
        int32_t cohort_n = 0;
        if (place_cohort_ready) {
            for (const MacroAction &a : actions) {
                const CohortSigKey sig = cohort_sig_for_action(a);
                if (sig.active() && a.motif_id >= 0) {
                    cohort_n += 1;
                }
            }
            if (cohort_n > 0) {
                telem.mcts_cohort_macro_n = cohort_n;
            }
        } else {
            // M2b telem: no PLACE_COHORT expand → report 0 (do not retain stale peak).
            telem.mcts_cohort_macro_n = 0;
        }
        if (actions.empty()) {
            return nullptr;
        }
        if (parent_id < 0) {
            fallback = actions.front();
            return &fallback;
        }
        const std::unordered_set<ActionKey, ActionKeyHash> tried = tried_action_keys(parent_id);
        std::vector<const MacroAction *> untried;
        untried.reserve(actions.size());
        for (const MacroAction &a : actions) {
            if (tried.count(action_key(a)) == 0) {
                untried.push_back(&a);
            }
        }
        if (untried.empty()) {
            fallback = actions.front();
            return &fallback;
        }
        const MacroAction *best = untried.front();
        float best_score = MCTS_NEG_INF;
        for (const MacroAction *a : untried) {
            const ActionKey key = action_key(*a);
            const float score = amaf_pick_score(key, parent_id, free_kind);
            if (score > best_score) {
                best_score = score;
                best = a;
            }
        }
        telem.amaf_pick += 1;
        const ActionKey best_key = action_key(*best);
        if (arena->amaf_visits(best_key.region_i, best_key.rule_id, best_key.motif_id) > 0) {
            telem.amaf_hits += 1;
        }
        fallback = *best;
        return &fallback;
    }

private:
    std::unordered_set<int32_t> tombstoned_;
    std::unordered_map<int32_t, int32_t> idle_age_;
    std::vector<BoardSnapshot> related_snaps_;
    std::unordered_map<ActionKey, CohortAmafSlot, ActionKeyHash> cohort_amaf_;
};
