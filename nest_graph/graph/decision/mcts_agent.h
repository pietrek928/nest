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
#include "decision_graph.h"
#include "macro_niche_archive.h"
#include "motif_base.h"
#include "node_signature.h"
#include "se2.h"

constexpr float MCTS_LAMBDA_MISS = 0.5f;
/** H2: soft-γ on AMAF + cohort_amaf when free_kind changes. */
constexpr float MCTS_AMAF_FREE_KIND_GAMMA = 0.25f;
/** H2e: GRAVE — use arena AMAF while cohort slot visits below ref (incl 0). */
constexpr int32_t MCTS_AMAF_COHORT_REF = 4;
/** H2e: mean-only MAST age per outer-iter since last_touch. */
constexpr float MCTS_AMAF_AGE_GAMMA = 0.9f;
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
    int32_t extend_scaled_n = 0;
    int32_t extend_dropped_n = 0;
    int32_t conflict_p_soft_n = 0;
    float conflict_p_sum = 0.f;
    int32_t amaf_free_kind_decay = 0;  // H2: times soft-γ fired
    int32_t amaf_cohort_fallback = 0;  // H2e: cohort cold → arena AMAF
    int32_t amaf_age_apply = 0;        // H2e: picks where mean aged
    int32_t amaf_age_max = 0;          // H2e: max age applied
    int32_t amaf_pole_floor = 0;       // L2: multi-pole trailing-region floor fires
    int32_t multi_pole_n = 0;          // L2: mirrored for telem pull/push persistence
    // GE2: Gelly–Silver β telem from ucb()
    float ucb_beta_sum = 0.f;
    int32_t ucb_beta_n = 0;
    float ucb_beta_min = 1.f;
    // GE5: progressive-bias envelope fires (k/(visits+k) < 1)
    int32_t pb_decay_n = 0;
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
    int32_t preset_id = 0;
    CohortSigKey cohort{};

    bool operator==(const ActionKey &o) const {
        return region_i == o.region_i && rule_id == o.rule_id && motif_id == o.motif_id
            && preset_id == o.preset_id && cohort == o.cohort;
    }
};

struct ActionKeyHash {
    std::size_t operator()(const ActionKey &k) const {
        std::size_t h = static_cast<std::size_t>(k.region_i);
        h = h * 1315423911u + static_cast<std::size_t>(k.rule_id);
        h = h * 1315423911u + static_cast<std::size_t>(k.motif_id + 2);
        h = h * 1315423911u + static_cast<std::size_t>(k.preset_id + 2);
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
    int32_t last_touch = 0;
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

/** N0: path accept gate (eligible, cov_ok, base_cov, alt_cov). */
inline std::tuple<bool, bool, float, float> path_accept_eligible(
    const BoardSnapshot &parent,
    const BoardSnapshot &child,
    float base_reward,
    float alt_reward,
    bool path_overlap_ok,
    bool is_motif,
    bool relax_motif_void_fill = false,
    bool fits_part = true,
    float min_cov_delta = 0.005f,
    float min_void_fill_delta = 0.02f
) {
    const float base_cov = parent.coverage;
    const float alt_cov = child.coverage;
    FreeKind free_kind = child.free_kind;
    if (free_kind == FreeKind::Empty) {
        free_kind = parent.free_kind;
    }
    if (relax_motif_void_fill && is_motif && free_kind == FreeKind::LargeVoid) {
        const bool cov_ok = path_overlap_ok && (alt_cov + 1e-9f >= base_cov);
        return {true, cov_ok, base_cov, alt_cov};
    }
    if (free_kind == FreeKind::LargeVoid && !fits_part) {
        const bool cov_ok = path_overlap_ok && (alt_cov + 1e-9f >= base_cov);
        return {true, cov_ok, base_cov, alt_cov};
    }
    if (!path_reward_beats(parent, child, base_reward, alt_reward)) {
        return {false, false, 0.f, 0.f};
    }
    bool cov_ok = false;
    if (free_kind == FreeKind::LargeVoid) {
        const float base_vf = parent.void_fill;
        const float alt_vf = child.void_fill;
        cov_ok = path_overlap_ok && (alt_cov + 1e-9f >= base_cov) &&
                 ((alt_vf - base_vf) + 1e-12f >= min_void_fill_delta);
    } else {
        cov_ok = path_overlap_ok &&
                 (alt_cov + 1e-9f >= base_cov + min_cov_delta);
    }
    return {true, cov_ok, base_cov, alt_cov};
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
    // D1: placeable LargeVoid inflates void_w; Swiss placeable keeps lam_void
    // (high Swiss void_fill would reward staying Swiss). Scale void_fill by fit
    // for LargeVoid and Swiss so hopeless contributes 0.
    const float fit_r = snapshot.void_fit_r;
    if (snapshot.free_kind == FreeKind::LargeVoid && fit_r >= 1.f) {
        void_w = std::max(void_w, 0.75f);
        rim_w = std::min(rim_w, 0.05f);
    }
    float void_fill_term = snapshot.void_fill;
    // GE1: LargeVoid scales by continuous void_fit_ratio (not binary fit_r).
    // Swiss keeps binary fit_r so high Swiss void_fill cannot reward staying Swiss.
    if (snapshot.free_kind == FreeKind::LargeVoid) {
        const float fit_c = snapshot.void_fit_ratio;
        void_fill_term *= std::min(std::max(fit_c, 0.f), 1.f);
    } else if (snapshot.free_kind == FreeKind::SwissCheese) {
        void_fill_term *= std::min(std::max(fit_r, 0.f), 1.f);
    }
    // Layout-only: rule/motif additives live in amaf_pick_score (Gelly–Silver),
    // not backed into the leaf mean (Q286 hollow index / double-count).
    // L1b restore of survive_* re-hit late +0.1678 attractor — keep stripped.
    (void)rule_id;
    (void)member_hits;
    (void)materialized_motif;
    (void)survive_motif_n;
    (void)macro_survive_n;
    return snapshot.coverage + void_w * void_fill_term + rim_w * snapshot.rim_fill
        + lam_kiss * (static_cast<float>(snapshot.kiss_pairs) / static_cast<float>(packed))
        + lam_comp * snapshot.mean_compactness;
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
    /** Densify preset ids for generate_macros (0 = baseline). Empty → {0}. */
    std::vector<int32_t> preset_ids{0};

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
        key.preset_id = action.preset_id;
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
        if (arena != nullptr) {
            slot.last_touch = arena->amaf_outer_iter();
        }
        telem.cohort_sig_amaf_visits = std::max(
            telem.cohort_sig_amaf_visits,
            slot.visits
        );
    }

    void set_outer_iter(int32_t outer_iter) {
        if (arena != nullptr) {
            arena->set_amaf_outer_iter(outer_iter);
        }
    }

    /** L2: pocket / iterative spine pole count (≥2 arms multi-pole AMAF floor). */
    void set_multi_pole_n(int32_t n) {
        multi_pole_n_ = std::max(0, n);
        telem.multi_pole_n = multi_pole_n_;
    }

    int32_t multi_pole_n() const { return multi_pole_n_; }

    /**
     * H2: on BoardSnapshot.free_kind change, soft-scale arena AMAF + cohort_amaf_.
     * Empty kind ignored (unset / inherit). First non-empty seeds without decay.
     */
    void note_free_kind(
        FreeKind kind,
        float gamma = MCTS_AMAF_FREE_KIND_GAMMA
    ) {
        if (kind == FreeKind::Empty) {
            return;
        }
        if (have_last_free_kind_ && kind != last_free_kind_) {
            if (arena != nullptr) {
                arena->amaf_soft_scale(gamma);
            }
            for (auto &kv : cohort_amaf_) {
                CohortAmafSlot &e = kv.second;
                if (e.visits <= 0) {
                    continue;
                }
                const float mean = e.reward / static_cast<float>(e.visits);
                const float miss_rate =
                    static_cast<float>(e.misses) / static_cast<float>(e.visits);
                const int32_t nv = static_cast<int32_t>(
                    std::lround(static_cast<float>(e.visits) * gamma)
                );
                if (nv <= 0) {
                    e.visits = 0;
                    e.reward = 0.f;
                    e.misses = 0;
                    continue;
                }
                e.visits = nv;
                e.reward = mean * static_cast<float>(nv);
                e.misses = std::max(
                    0,
                    static_cast<int32_t>(
                        std::lround(miss_rate * static_cast<float>(nv))
                    )
                );
                if (e.misses > e.visits) {
                    e.misses = e.visits;
                }
            }
            telem.amaf_free_kind_decay += 1;
        }
        last_free_kind_ = kind;
        have_last_free_kind_ = true;
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
        const int32_t av = arena->amaf_visits(
            region_i, action.rule_id, action.motif_id, action.preset_id
        );
        if (av > 0) {
            telem.amaf_hits += 1;
            // Mirror GE2 β for telem (same formula as DecisionArena::ucb_score).
            constexpr float k_amaf = 50.f;
            const float n_vis = static_cast<float>(
                std::max(arena->node(node_id).visits, 1)
            );
            const float beta = std::sqrt(
                k_amaf / (3.f * n_vis + k_amaf + 1e-9f)
            );
            telem.ucb_beta_sum += beta;
            telem.ucb_beta_n += 1;
            telem.ucb_beta_min = std::min(telem.ucb_beta_min, beta);
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
            arena->amaf_record(
                region_i, action.rule_id, action.motif_id, reward, false, action.preset_id
            );
            record_cohort_amaf(action_key(action), reward, false);
            cur = arena->node(cur).parent_id;
        }
    }

    void note_macro_miss(const MacroAction *action) {
        if (action == nullptr || arena == nullptr) {
            return;
        }
        const int32_t region_i = static_cast<int32_t>(action->region);
        arena->amaf_record(
            region_i, action->rule_id, action->motif_id, 0.f, true, action->preset_id
        );
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

    /** GE3: fraction of related_snaps_ with leaf_reward ≤ reward; n = buffer size. */
    std::pair<float, int32_t> rank_related_reward(float reward) const {
        const int32_t n = static_cast<int32_t>(related_snaps_.size());
        if (n <= 0) {
            return {0.5f, 0};
        }
        int32_t le = 0;
        for (const BoardSnapshot &snap : related_snaps_) {
            if (leaf_reward(snap) <= reward + 1e-12f) {
                le += 1;
            }
        }
        return {static_cast<float>(le) / static_cast<float>(n), n};
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
        FreeKind free_kind = FreeKind::Empty,
        float void_fit_r = 0.f,
        float void_fit_ratio = 0.f
    ) {
        if (arena == nullptr) {
            return MCTS_NEG_INF;
        }
        const int32_t parent_visits = std::max(arena->node(parent_id).visits, 0);
        const int32_t region_i = key.region_i;
        const int32_t rule_id = key.rule_id;
        const int32_t motif_id = key.motif_id;
        const int32_t preset_id = key.preset_id;
        int32_t visits = arena->amaf_visits(region_i, rule_id, motif_id, preset_id);
        float mean = 0.f;
        int32_t misses = 0;
        int32_t last_touch = 0;
        if (key.cohort.active()) {
            auto it = cohort_amaf_.find(key);
            const int32_t cohort_visits =
                (it != cohort_amaf_.end() && it->second.visits > 0)
                    ? it->second.visits
                    : 0;
            if (cohort_visits >= MCTS_AMAF_COHORT_REF) {
                visits = cohort_visits;
                mean = it->second.reward / static_cast<float>(visits);
                misses = it->second.misses;
                last_touch = it->second.last_touch;
                telem.cohort_sig_amaf_visits = std::max(
                    telem.cohort_sig_amaf_visits,
                    visits
                );
            } else if (visits > 0) {
                // H2e GRAVE: cold/thin cohort → arena AMAF for same 4-tuple.
                mean = arena->amaf_mean(region_i, rule_id, motif_id, preset_id);
                misses = arena->amaf_misses(region_i, rule_id, motif_id, preset_id);
                last_touch = arena->amaf_last_touch(
                    region_i, rule_id, motif_id, preset_id
                );
                telem.amaf_cohort_fallback += 1;
            } else {
                visits = 0;
            }
        } else if (visits <= 0) {
            visits = 0;
        } else {
            mean = arena->amaf_mean(region_i, rule_id, motif_id, preset_id);
            misses = arena->amaf_misses(region_i, rule_id, motif_id, preset_id);
            last_touch = arena->amaf_last_touch(
                region_i, rule_id, motif_id, preset_id
            );
        }
        // H2e: mean-only MAST age (do not scale miss_term / pb / additives).
        if (visits > 0) {
            const int32_t age = std::max(0, arena->amaf_outer_iter() - last_touch);
            if (age > 0) {
                mean *= std::pow(
                    MCTS_AMAF_AGE_GAMMA, static_cast<float>(age)
                );
                telem.amaf_age_apply += 1;
                telem.amaf_age_max = std::max(telem.amaf_age_max, age);
            }
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
        // L4: drop uniform amaf_survive_scale (dead argmax multiply).
        telem.amaf_survive_scale = survival_live ? 1.f : 0.f;
        float score = 0.f;
        if (visits <= 0) {
            score = pb;
        } else {
            float miss_term = 0.f;
            if (misses > 0) {
                miss_term = MCTS_LAMBDA_MISS * (static_cast<float>(misses) / static_cast<float>(visits));
            }
            score = mean - miss_term + pb;
        }
        const int32_t void_i = static_cast<int32_t>(MacroRegion::Void);
        const int32_t rim_i = static_cast<int32_t>(MacroRegion::Rim);
        const int32_t motif_i = static_cast<int32_t>(MacroRegion::Motif);
        // D: niche-miss soft-scales Void; Motif +0.2 when cohort/MotifBase fuel (plan D).
        const bool motif_fuel =
            (motif_base != nullptr && motif_base->size() > 0)
            || place_cohort_ready
            || !motif_cohorts.empty();
        // D1: placeable (LV or Swiss) Void AMAF; phantom LV soft-down Void.
        const bool placeable = void_placeable(free_kind, void_fit_r);
        const bool phantom = void_phantom(free_kind, void_fit_r);
        if (placeable && region_i == void_i) {
            float void_bonus = 0.35f;
            float miss_rate = 0.f;
            if (visits > 0 && misses > 0) {
                miss_rate = static_cast<float>(misses) / static_cast<float>(visits);
            }
            if (niche_archive != nullptr) {
                NicheKey nkey{region_i, rule_id, motif_id};
                auto bit = niche_archive->buckets().find(nkey);
                if (bit == niche_archive->buckets().end()) {
                    nkey = NicheKey{region_i, 0, motif_id};
                    bit = niche_archive->buckets().find(nkey);
                }
                if (bit != niche_archive->buckets().end()) {
                    miss_rate = std::max(miss_rate, bit->second.miss_rate());
                }
                if (niche_archive->place_fail_streak > 0) {
                    miss_rate = std::max(
                        miss_rate,
                        std::min(1.f, 0.25f * static_cast<float>(niche_archive->place_fail_streak))
                    );
                }
            }
            miss_rate = std::max(0.f, std::min(1.f, miss_rate));
            score += void_bonus * (1.f - miss_rate);
        }
        // Soft Macro↔Void: keep Swiss −0.2 magnitude under phantom (fit_r bit).
        // Continuous conflict_p telem from void_fit_ratio; Letter B consumes p on pole.
        // (×(1−α) and ratio-softened −0.2 both dumped early min below open.)
        if (phantom && region_i == void_i) {
            score -= 0.2f;
            const float p = soft_macro_conflict_p(free_kind, void_fit_ratio);
            telem.extend_scaled_n += 1;
            if (p > 0.f) {
                telem.conflict_p_soft_n += 1;
                telem.conflict_p_sum += p;
            }
        }
        // GE5 degrade: Motif-only PB envelope (full Rim/kind/attach/survive
        // decay dual-min under open). k=8 matches beta_accept. Void/phantom flat.
        constexpr float k_pb_decay = 8.f;
        const float pb_decay =
            visits <= 0 ? 1.f
                        : (k_pb_decay / (static_cast<float>(visits) + k_pb_decay));
        if (placeable && region_i == rim_i) {
            score -= 0.15f;
        }
        if (placeable && region_i == motif_i && motif_fuel) {
            // R2b unify: niche miss soft-scale dual-min under open — flat +0.2.
            score += 0.2f * pb_decay;
            if (pb_decay < 1.f - 1e-6f) {
                telem.pb_decay_n += 1;
            }
        }
        const int32_t sel_n = std::max(realized.sel_n, 1);
        // R2: live Kind term; when multi-pole floor fires, scale trailing
        // regions by 1+0.25·(max_share−my_share). Skip scale when visits==0.
        float kind_term = 0.f;
        if (region_i >= 0 && region_i < 4) {
            kind_term = 0.2f * (
                static_cast<float>(realized.kind[static_cast<std::size_t>(region_i)])
                / static_cast<float>(sel_n)
            );
        }
        if (multi_pole_n_ <= 0 && telem.multi_pole_n > 0) {
            multi_pole_n_ = telem.multi_pole_n;
        }
        if (multi_pole_n_ >= 2 && region_i >= 0 && region_i < 4) {
            int32_t max_k = realized.kind[0];
            int32_t kind_sum = realized.kind[0];
            for (int32_t k = 1; k < 4; ++k) {
                const int32_t kv = realized.kind[static_cast<std::size_t>(k)];
                max_k = std::max(max_k, kv);
                kind_sum += kv;
            }
            if (kind_sum > 0 && max_k > 0) {
                const float max_share =
                    static_cast<float>(max_k) / static_cast<float>(kind_sum);
                const float my_share = static_cast<float>(
                    realized.kind[static_cast<std::size_t>(region_i)]
                ) / static_cast<float>(kind_sum);
                if (max_share >= 0.5f && my_share + 1e-6f < max_share) {
                    // R2 unify: Kind scale dual-min under open — telem-only.
                    telem.amaf_pole_floor += 1;
                }
            }
        }
        score += kind_term;
        const int32_t attach_n = realized.attach;
        const int32_t member_hits = realized.member_hits;
        const int32_t mat_motif = realized.materialized_motif;
        int32_t survive_n = 0;
        if (motif_id >= 0) {
            auto sit = realized.survive_by_motif.find(motif_id);
            if (sit != realized.survive_by_motif.end()) {
                survive_n = sit->second;
            }
            if (survive_n <= 0) {
                survive_n = realized.survive_motif_n;
            }
        }
        // L2: motif_id<0 must not fall through to aggregate survive_motif_n on Void;
        // realized.kind[region] already soft-boosts above.
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
            // L4: Gelly–Silver β(visits) fades accept_count prior; gci stays max quality.
            const float beta_accept =
                visits <= 0 ? 1.f
                            : (8.f / (static_cast<float>(visits) + 8.f));
            score += beta_accept * 0.04f
                * std::min(static_cast<float>(rec.accept_count), 8.f);
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
        // L2: drop preset_id==0 +1e-3 cold-tie bias so online presets compete.
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
        FreeKind free_kind = FreeKind::Empty;
        float void_fit_r = 0.f;
        float void_fit_ratio = 0.f;
        if (snapshot != nullptr) {
            warm = warm_motif_ids(snapshot);
            free_kind = snapshot->free_kind;
            void_fit_r = snapshot->void_fit_r;
            void_fit_ratio = snapshot->void_fit_ratio;
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
            cohort_specs,
            preset_ids,
            void_fit_r
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
            const float score = amaf_pick_score(
                key, parent_id, free_kind, void_fit_r, void_fit_ratio);
            if (score > best_score) {
                best_score = score;
                best = a;
            }
        }
        telem.amaf_pick += 1;
        const ActionKey best_key = action_key(*best);
        bool amaf_hit = arena->amaf_visits(
                best_key.region_i, best_key.rule_id, best_key.motif_id, best_key.preset_id
            )
            > 0;
        if (!amaf_hit && best_key.cohort.active()) {
            auto cit = cohort_amaf_.find(best_key);
            if (cit != cohort_amaf_.end() && cit->second.visits > 0) {
                amaf_hit = true;
            }
        }
        if (amaf_hit) {
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
    FreeKind last_free_kind_ = FreeKind::Empty;
    bool have_last_free_kind_ = false;
    int32_t multi_pole_n_ = 0;
};
