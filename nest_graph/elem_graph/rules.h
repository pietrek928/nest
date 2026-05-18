#pragma once

#include <vector>

#include "types.h"

typedef struct PointPlaceRule {
    Vec2f pos;
    float r, w;
    Tvertex group;
} PointPlaceRule;


typedef struct CirclePlaceRule {
    Circle2f circle;
    float r, w;
    Tvertex group;
} CirclePlaceRule;


typedef struct PointAngleRule {
    Vec2f pos;
    float a, r, w;
    Tvertex group;
} PointAngleRule;


typedef struct CircleAngleRule {
    Circle2f circle;
    float a, r, w;
    Tvertex group;
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
} RuleMutationSettings;

std::vector<PlacementRuleSet> augment_rules(
    const std::vector<PlacementRuleSet> &rules,
    const RuleMutationSettings &settings);
