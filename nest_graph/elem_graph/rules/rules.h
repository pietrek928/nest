#pragma once

#include <cstdint>
#include <vector>

#include "types/types.h"

typedef struct PointPlaceRule {
    Vec2f pos;
    float r, w;
    Tvertex group;

    PointPlaceRule() = default;
    PointPlaceRule(Vec2f pos_, float r_, float w_, Tvertex group_)
        : pos(pos_), r(r_), w(w_), group(group_) {}
} PointPlaceRule;


typedef struct CirclePlaceRule {
    Circle2f circle;
    float r, w;
    Tvertex group;

    CirclePlaceRule() = default;
    CirclePlaceRule(Circle2f circle_, float r_, float w_, Tvertex group_)
        : circle(circle_), r(r_), w(w_), group(group_) {}
} CirclePlaceRule;


typedef struct PointAngleRule {
    Vec2f pos;
    float a, r, w;
    Tvertex group;

    PointAngleRule() = default;
    PointAngleRule(Vec2f pos_, float a_, float r_, float w_, Tvertex group_)
        : pos(pos_), a(a_), r(r_), w(w_), group(group_) {}
} PointAngleRule;


typedef struct CircleAngleRule {
    Circle2f circle;
    float a, r, w;
    Tvertex group;

    CircleAngleRule() = default;
    CircleAngleRule(Circle2f circle_, float a_, float r_, float w_, Tvertex group_)
        : circle(circle_), a(a_), r(r_), w(w_), group(group_) {}
} CircleAngleRule;


typedef struct PlacementRuleSet {
    std::vector<PointPlaceRule> point_rules;
    std::vector<CirclePlaceRule> circle_rules;
    std::vector<PointAngleRule> point_angle_rules;
    std::vector<CircleAngleRule> circle_angle_rules;
    inline auto size() const {
        return point_rules.size() + circle_rules.size() + point_angle_rules.size() +
               circle_angle_rules.size();
    }
} PlacementRuleSet;

typedef struct RuleMutationSettings {
    Circle2f region;
    float dpos, dw, da;
    float insert_p, remove_p, mutate_p;
    Tvertex ngroups;
    int max_inserts_per_type = 3;

    RuleMutationSettings() = default;
    RuleMutationSettings(
        Circle2f region_,
        float dpos_,
        float dw_,
        float da_,
        float insert_p_,
        float remove_p_,
        float mutate_p_,
        Tvertex ngroups_)
        : region(region_),
          dpos(dpos_),
          dw(dw_),
          da(da_),
          insert_p(insert_p_),
          remove_p(remove_p_),
          mutate_p(mutate_p_),
          ngroups(ngroups_) {}
} RuleMutationSettings;

std::vector<PlacementRuleSet> augment_rules(
    const std::vector<PlacementRuleSet> &rules,
    const RuleMutationSettings &settings,
    std::uint32_t seed = 0);
