#pragma once

#include <cstdint>
#include <string>
#include <tuple>
#include <unordered_set>
#include <vector>

#include "decision_arena.h"
#include "motif_base.h"

inline std::string region_to_zone(MacroRegion region) {
    switch (region) {
    case MacroRegion::Rim:
        return "cluster_edge";
    case MacroRegion::Void:
        return "void_seek";
    case MacroRegion::Motif:
        return "void_seek";  // Q104
    case MacroRegion::Sheet:
        return "interior_pocket";  // Q98
    default:
        return "interior_pocket";
    }
}

inline MacroRegion zone_to_region(const std::string &zone) {
    if (zone == "cluster_edge" || zone == "empty_border") {
        return MacroRegion::Rim;
    }
    if (zone == "void_seek" || zone == "void") {
        return MacroRegion::Void;
    }
    if (zone == "interior_pocket") {
        return MacroRegion::Sheet;
    }
    return MacroRegion::Sheet;
}

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
    const std::string &free_kind,
    const std::vector<MotifCohortSpec> &motif_cohorts) {
    std::vector<MacroAction> actions;
    if (remaining_gids.empty()) {
        return actions;
    }
    std::unordered_set<int32_t> rem(remaining_gids.begin(), remaining_gids.end());

    std::vector<MacroAction> motif_actions;
    std::unordered_set<int32_t> seen_motifs;
    if (prefer_motifs && motif_base != nullptr && motif_base->size() > 0) {
        std::vector<int32_t> ordered = warm_motif_ids;
        for (int32_t i = 0; i < motif_base->size(); ++i) {
            ordered.push_back(i);
        }
        const int32_t rule0 = rule_ids.empty() ? 0 : rule_ids.front();
        for (int32_t mid_i : ordered) {
            if (mid_i < 0 || mid_i >= motif_base->size() || seen_motifs.count(mid_i)) {
                continue;
            }
            seen_motifs.insert(mid_i);
            const MotifRecord &rec = motif_base->at(mid_i);
            if (rem.count(rec.gid_a) && rem.count(rec.gid_b)) {
                MacroAction a;
                a.region = MacroRegion::Motif;
                a.motif_id = mid_i;
                a.part_gid = rec.gid_a;
                a.rule_id = rule0;
                motif_actions.push_back(a);
            }
        }
    }

    std::vector<MacroAction> cohort_actions;
    std::unordered_set<std::string> seen_cohort;
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
            cohort_actions.push_back(a);
        }
    }

    std::vector<MacroRegion> region_order;
    if (free_kind == "large_void") {
        region_order = {MacroRegion::Void, MacroRegion::Rim, MacroRegion::Sheet};
    } else {
        region_order = {MacroRegion::Rim, MacroRegion::Void, MacroRegion::Sheet};
    }

    actions.insert(actions.end(), motif_actions.begin(), motif_actions.end());
    actions.insert(actions.end(), cohort_actions.begin(), cohort_actions.end());
    const int32_t rem0 = remaining_gids.front();
    for (MacroRegion region : region_order) {
        for (int32_t rid : rule_ids) {
            MacroAction a;
            a.region = region;
            a.rule_id = rid;
            a.part_gid = rem0;
            a.motif_id = -1;
            actions.push_back(a);
        }
    }
    return actions;
}
