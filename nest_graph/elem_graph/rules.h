#pragma once

#include <vector>

#include "types.h"

typedef struct PointPlaceRule {
    float x, y, r, w;
    Tvertex group;
} PointPlaceRule;


typedef struct BBoxPlaceRule {
    BBox bbox;
    float r, w;
    Tvertex group;
} BBoxPlaceRule;


typedef struct PointAngleRule {
    float x, y, a, r, w;
    Tvertex group;
} PointAngleRule;


typedef struct BBoxAngleRule {
    BBox bbox;
    float a, r, w;
    Tvertex group;
} BBoxAngleRule;


typedef struct PlacementRuleSet {
    std::vector<PointPlaceRule> point_rules;
    std::vector<BBoxPlaceRule> bbox_rules;
    std::vector<PointAngleRule> point_angle_rules;
    std::vector<BBoxAngleRule> bbox_angle_rules;
    inline auto size() const {
        return point_rules.size() + bbox_rules.size() + point_angle_rules.size() +
               bbox_angle_rules.size();
    }
} PlacementRuleSet;

typedef struct RuleMutationSettings {
    BBox box;
    float dpos, dw, da;
    float insert_p, remove_p, mutate_p;
    Tvertex ngroups;
} RuleMutationSettings;

std::vector<PlacementRuleSet> augment_rules(
    const std::vector<PlacementRuleSet> &rules,
    const RuleMutationSettings &settings);
