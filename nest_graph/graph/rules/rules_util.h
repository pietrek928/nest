#pragma once

#include "rules/rules.h"

inline void append_point_place_rule(PlacementRuleSet &s, const PointPlaceRule &r) {
    s.point_rules.push_back(r);
}

inline void append_circle_place_rule(PlacementRuleSet &s, const CirclePlaceRule &r) {
    s.circle_rules.push_back(r);
}

inline void append_point_angle_rule(PlacementRuleSet &s, const PointAngleRule &r) {
    s.point_angle_rules.push_back(r);
}

inline void append_circle_angle_rule(PlacementRuleSet &s, const CircleAngleRule &r) {
    s.circle_angle_rules.push_back(r);
}

inline void append_rule(PlacementRuleSet &s, const PointPlaceRule &r) {
    append_point_place_rule(s, r);
}

inline void append_rule(PlacementRuleSet &s, const CirclePlaceRule &r) {
    append_circle_place_rule(s, r);
}

inline void append_rule(PlacementRuleSet &s, const PointAngleRule &r) {
    append_point_angle_rule(s, r);
}

inline void append_rule(PlacementRuleSet &s, const CircleAngleRule &r) {
    append_circle_angle_rule(s, r);
}

inline void append_point_place_rule_at(
    PlacementRuleSet &s, const Vec2f &pos, float r, float w, Tvertex group) {
    s.point_rules.push_back(PointPlaceRule{pos, r, w, group});
}

inline void append_circle_place_rule_at(
    PlacementRuleSet &s, const Circle2f &circle, float r, float w, Tvertex group) {
    s.circle_rules.push_back(CirclePlaceRule{circle, r, w, group});
}

inline void append_point_angle_rule_at(
    PlacementRuleSet &s, const Vec2f &pos, float a, float r, float w, Tvertex group) {
    s.point_angle_rules.push_back(PointAngleRule{pos, a, r, w, group});
}

inline void append_circle_angle_rule_at(
    PlacementRuleSet &s, const Circle2f &circle, float a, float r, float w, Tvertex group) {
    s.circle_angle_rules.push_back(CircleAngleRule{circle, a, r, w, group});
}
