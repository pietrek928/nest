#pragma once

#include <cstdint>
#include <string>
#include <tuple>
#include <unordered_set>
#include <vector>

#include "decision_arena.h"
#include "motif_base.h"

inline PlaceZone region_to_zone(MacroRegion region) {
    switch (region) {
    case MacroRegion::Rim:
        return PlaceZone::ClusterEdge;
    case MacroRegion::Void:
        return PlaceZone::VoidSeek;
    case MacroRegion::Motif:
        return PlaceZone::VoidSeek;  // Q104
    case MacroRegion::Sheet:
        return PlaceZone::InteriorPocket;  // Q98
    default:
        return PlaceZone::InteriorPocket;
    }
}

inline MacroRegion zone_to_region(PlaceZone zone) {
    switch (zone) {
    case PlaceZone::ClusterEdge:
    case PlaceZone::EmptyBorder:
        return MacroRegion::Rim;
    case PlaceZone::VoidSeek:
        return MacroRegion::Void;
    case PlaceZone::InteriorPocket:
    case PlaceZone::BorderGap:
    case PlaceZone::InterCluster:
    default:
        return MacroRegion::Sheet;
    }
}

/** Niche AMAF stays (region, rule=0, motif) — deliberately ignores preset_id/rule_id. */
inline std::tuple<int32_t, int32_t, int32_t> niche_amaf_key(const MacroAction *action) {
    if (action == nullptr) {
        return {static_cast<int32_t>(MacroRegion::Void), 0, -1};
    }
    int32_t motif_id = action->motif_id;
    if (motif_id < 0) {
        motif_id = -1;
    }
    return {static_cast<int32_t>(action->region), 0, motif_id};
}

struct MotifCohortSpec {
    int32_t motif_id = -1;
    int32_t leader_gid = -1;
    int32_t member_keys_count = 0;
};

inline std::vector<MacroAction> generate_macros(
    const std::vector<int32_t> &remaining_gids,
    const std::vector<int32_t> &rule_ids,
    const MotifBase *motif_base,
    bool prefer_motifs,
    const std::vector<int32_t> &warm_motif_ids,
    FreeKind free_kind,
    const std::vector<MotifCohortSpec> &motif_cohorts,
    const std::vector<int32_t> &preset_ids = {}) {
    std::vector<MacroAction> actions;
    if (remaining_gids.empty()) {
        return actions;
    }
    std::unordered_set<int32_t> rem(remaining_gids.begin(), remaining_gids.end());
    std::vector<int32_t> presets = preset_ids;
    if (presets.empty()) {
        presets.push_back(0);
    }

    // Cohort first so Motif×rule_ids can dedupe (region, rule, motif, preset).
    std::vector<MacroAction> cohort_actions;
    std::unordered_set<std::string> seen_cohort;
    std::unordered_set<std::string> cohort_rr_mp;
    for (const MotifCohortSpec &c : motif_cohorts) {
        if (c.member_keys_count < 2) {
            continue;
        }
        if (!rem.count(c.leader_gid)) {
            continue;
        }
        const std::string sig = std::to_string(c.motif_id) + ":" + std::to_string(c.leader_gid);
        if (seen_cohort.count(sig)) {
            continue;
        }
        seen_cohort.insert(sig);
        for (int32_t rid : rule_ids) {
            MacroAction a;
            a.region = MacroRegion::Motif;
            a.motif_id = c.motif_id;
            a.part_gid = c.leader_gid;
            a.rule_id = rid;
            a.preset_id = 0;
            cohort_actions.push_back(a);
            cohort_rr_mp.insert(
                std::to_string(static_cast<int32_t>(MacroRegion::Motif)) + ":"
                + std::to_string(rid) + ":" + std::to_string(c.motif_id) + ":0"
            );
        }
    }

    // C1 / Q286: Motif × rule_ids only for warm/cohort-ready motifs (PW starvation
    // when every Motif×K floods the expand list — early dual dumped below H3).
    std::vector<MacroAction> motif_actions;
    std::unordered_set<int32_t> seen_motifs;
    std::vector<int32_t> motif_rules = rule_ids;
    if (motif_rules.empty()) {
        motif_rules.push_back(0);
    }
    const int32_t rule0 = motif_rules.front();
    // Multi-rule only for warm or cohort motif ids — not every Motif when any cohort exists.
    std::unordered_set<int32_t> warm_set(warm_motif_ids.begin(), warm_motif_ids.end());
    for (const MotifCohortSpec &c : motif_cohorts) {
        if (c.motif_id >= 0) {
            warm_set.insert(c.motif_id);
        }
    }
    if (prefer_motifs && motif_base != nullptr && motif_base->size() > 0) {
        std::vector<int32_t> ordered = warm_motif_ids;
        for (int32_t i = 0; i < motif_base->size(); ++i) {
            ordered.push_back(i);
        }
        for (int32_t mid_i : ordered) {
            if (mid_i < 0 || mid_i >= motif_base->size() || seen_motifs.count(mid_i)) {
                continue;
            }
            seen_motifs.insert(mid_i);
            const MotifRecord &rec = motif_base->at(mid_i);
            // Pair rem preferred; D: single-gid Motif tip when one mate already packed.
            int32_t part = -1;
            if (rem.count(rec.gid_a) && rem.count(rec.gid_b)) {
                part = rec.gid_a;
            } else if (free_kind == FreeKind::LargeVoid && rem.count(rec.gid_a)) {
                part = rec.gid_a;
            } else if (free_kind == FreeKind::LargeVoid && rem.count(rec.gid_b)) {
                part = rec.gid_b;
            }
            if (part < 0) {
                continue;
            }
            const bool multi_rule = warm_set.count(mid_i) > 0;
            // Warm Motif: emit rule_id>0 before rule0 so PW can expand non-zero first.
            std::vector<int32_t> emit_rules;
            if (multi_rule) {
                for (int32_t rid : motif_rules) {
                    if (rid != rule0) {
                        emit_rules.push_back(rid);
                    }
                }
                emit_rules.push_back(rule0);
            } else {
                emit_rules.push_back(rule0);
            }
            for (int32_t rid : emit_rules) {
                const std::string key =
                    std::to_string(static_cast<int32_t>(MacroRegion::Motif)) + ":"
                    + std::to_string(rid) + ":" + std::to_string(mid_i) + ":0";
                if (cohort_rr_mp.count(key)) {
                    continue;
                }
                MacroAction a;
                a.region = MacroRegion::Motif;
                a.motif_id = mid_i;
                a.part_gid = part;
                a.rule_id = rid;
                a.preset_id = 0;
                motif_actions.push_back(a);
            }
        }
    }

    std::vector<MacroRegion> region_order;
    if (free_kind == FreeKind::LargeVoid) {
        region_order = {MacroRegion::Void, MacroRegion::Rim, MacroRegion::Sheet};
    } else {
        region_order = {MacroRegion::Rim, MacroRegion::Void, MacroRegion::Sheet};
    }

    actions.insert(actions.end(), cohort_actions.begin(), cohort_actions.end());
    actions.insert(actions.end(), motif_actions.begin(), motif_actions.end());
    const int32_t rem0 = remaining_gids.front();
    // Letter B: rem0 only here — dual late Δ regressed with Void×rem expand.
    // part_gid still diversifies Motif/cohort macros + cache key as rem0 advances.
    for (MacroRegion region : region_order) {
        for (int32_t rid : rule_ids) {
            for (int32_t pid : presets) {
                MacroAction a;
                a.region = region;
                a.rule_id = rid;
                a.part_gid = rem0;
                a.motif_id = -1;
                a.preset_id = pid;
                actions.push_back(a);
            }
        }
    }
    return actions;
}
