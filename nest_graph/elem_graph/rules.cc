#include "rules.h"

#include <algorithm>
#include <cmath>
#include <random>
#include <vector>

template <class T>
T adjust_angle(T a) {
    if (a < 0) {
        return a + 2 * M_PI;
    }
    if (a > 2 * M_PI) {
        return a - 2 * M_PI;
    }
    return a;
}

template <class Tgen, class Tdistrib>
Circle2f mutate_random(const Circle2f &c, Tgen gen, Tdistrib &distrib) {
    Circle2f r = c;
    r.c.get_(0) += distrib(gen);
    r.c.get_(1) += distrib(gen);
    r.r_sq += distrib(gen);
    if (r.r_sq < 0.0f) {
        r.r_sq = 0.0f;
    }
    return r;
}

template <class Tgen, class Tdistrib>
Circle2f generate_random_circle(
    Tgen gen, Tdistrib &distrib_x, Tdistrib &distrib_y, Tdistrib &distrib_r
) {
    const float rx = distrib_r(gen);
    const float r_sq = rx * rx;
    return Circle2f(Vec2f({distrib_x(gen), distrib_y(gen)}), r_sq);
}

template <class Tgen, class Tdistrib>
PointPlaceRule mutate_random(
    const PointPlaceRule &p, Tgen gen, Tdistrib &distrib_pos, Tdistrib &distrib_w
) {
    Vec2f pos = p.pos;
    pos.get_(0) += distrib_pos(gen);
    pos.get_(1) += distrib_pos(gen);
    return {
        .pos = pos,
        .r = p.r + distrib_pos(gen),
        .w = p.w + distrib_w(gen),
        .group = p.group
    };
}

template <class Tgen, class Tdistrib, class Tdistrib_group>
PointPlaceRule generate_random_point_place_rule(
    Tgen gen, Tdistrib &distrib_x, Tdistrib &distrib_y,
    Tdistrib &distrib_r, Tdistrib &distrib_w, Tdistrib_group &distrib_group
) {
    return {
        .pos = Vec2f({distrib_x(gen), distrib_y(gen)}),
        .r = distrib_r(gen),
        .w = distrib_w(gen),
        .group = distrib_group(gen)
    };
}

template <class Tgen, class Tdistrib>
CirclePlaceRule mutate_random(
    const CirclePlaceRule &p, Tgen gen, Tdistrib &distrib_pos, Tdistrib &distrib_w
) {
    return {
        .circle = mutate_random(p.circle, gen, distrib_pos),
        .r = p.r + distrib_pos(gen),
        .w = p.w + distrib_w(gen),
        .group = p.group
    };
}

template <class Tgen, class Tdistrib, class Tdistrib_group>
CirclePlaceRule generate_random_circle_place_rule(
    Tgen gen, Tdistrib &distrib_x, Tdistrib &distrib_y,
    Tdistrib &distrib_r, Tdistrib &distrib_w, Tdistrib_group &distrib_group
) {
    return {
        .circle = generate_random_circle(gen, distrib_x, distrib_y, distrib_r),
        .r = distrib_r(gen),
        .w = distrib_w(gen),
        .group = distrib_group(gen)
    };
}

template <class Tgen, class Tdistrib>
PointAngleRule mutate_random(
    const PointAngleRule &p, Tgen gen, Tdistrib &distrib_pos,
    Tdistrib &distrib_a, Tdistrib &distrib_w
) {
    Vec2f pos = p.pos;
    pos.get_(0) += distrib_pos(gen);
    pos.get_(1) += distrib_pos(gen);
    return {
        .pos = pos,
        .a = adjust_angle(p.a + distrib_a(gen)),
        .r = p.r,
        .w = p.w + distrib_w(gen),
        .group = p.group
    };
}

template <class Tgen, class Tdistrib, class Tdistrib_group>
PointAngleRule generate_random_point_angle_rule(
    Tgen gen, Tdistrib &distrib_x, Tdistrib &distrib_y,
    Tdistrib &distrib_r, Tdistrib &distrib_a,
    Tdistrib &distrib_w, Tdistrib_group &distrib_group
) {
    return {
        .pos = Vec2f({distrib_x(gen), distrib_y(gen)}),
        .a = adjust_angle(distrib_a(gen)),
        .r = distrib_r(gen),
        .w = distrib_w(gen),
        .group = distrib_group(gen)
    };
}

template <class Tgen, class Tdistrib>
CircleAngleRule mutate_random(
    const CircleAngleRule &p, Tgen gen,
    Tdistrib &distrib_pos, Tdistrib &distrib_a, Tdistrib &distrib_w
) {
    return {
        .circle = mutate_random(p.circle, gen, distrib_pos),
        .a = adjust_angle(p.a + distrib_a(gen)),
        .r = p.r + distrib_pos(gen),
        .w = p.w + distrib_w(gen),
        .group = p.group
    };
}

template <class Tgen, class Tdistrib, class Tdistrib_group>
CircleAngleRule generate_random_circle_angle_rule(
    Tgen gen, Tdistrib &distrib_x, Tdistrib &distrib_y,
    Tdistrib &distrib_r, Tdistrib &distrib_a,
    Tdistrib &distrib_w, Tdistrib_group &distrib_group
) {
    return {
        .circle = generate_random_circle(gen, distrib_x, distrib_y, distrib_r),
        .a = adjust_angle(distrib_a(gen)),
        .r = distrib_r(gen),
        .w = distrib_w(gen),
        .group = distrib_group(gen)
    };
}

template <class Tgen, class Tdistrib>
PlacementRuleSet mutate_random(
    const PlacementRuleSet &s, Tgen gen, Tdistrib &distrib_select,
    Tdistrib &distrib_pos, Tdistrib &distrib_a, Tdistrib &distrib_w
) {
    PlacementRuleSet r;
    for (const PointPlaceRule &p : s.point_rules) {
        if (distrib_select(gen) < 1.0) {
            r.point_rules.push_back(mutate_random(p, gen, distrib_pos, distrib_w));
        } else {
            r.point_rules.push_back(p);
        }
    }
    for (const CirclePlaceRule &p : s.circle_rules) {
        if (distrib_select(gen) < 1.0) {
            r.circle_rules.push_back(mutate_random(p, gen, distrib_pos, distrib_w));
        } else {
            r.circle_rules.push_back(p);
        }
    }
    for (const PointAngleRule &p : s.point_angle_rules) {
        if (distrib_select(gen) < 1.0) {
            r.point_angle_rules.push_back(mutate_random(p, gen, distrib_pos, distrib_a, distrib_w));
        } else {
            r.point_angle_rules.push_back(p);
        }
    }
    for (const CircleAngleRule &p : s.circle_angle_rules) {
        if (distrib_select(gen) < 1.0) {
            r.circle_angle_rules.push_back(mutate_random(p, gen, distrib_pos, distrib_a, distrib_w));
        } else {
            r.circle_angle_rules.push_back(p);
        }
    }
    return r;
}

template <class Tgen, class Tdistrib, class Tdistrib_group>
PlacementRuleSet insert_random(
    const PlacementRuleSet &s, Tgen gen, Tdistrib &distrib_insert,
    Tdistrib &distrib_pos, Tdistrib &distrib_a,
    Tdistrib &distrib_w, Tdistrib_group &distrib_group
) {
    PlacementRuleSet r = s;
    while (distrib_insert(gen) < 1.0) {
        r.point_rules.push_back(generate_random_point_place_rule(
            gen, distrib_pos, distrib_pos, distrib_pos, distrib_w, distrib_group
        ));
    }
    while (distrib_insert(gen) < 1.0) {
        r.circle_rules.push_back(generate_random_circle_place_rule(
            gen, distrib_pos, distrib_pos, distrib_pos, distrib_w, distrib_group
        ));
    }
    while (distrib_insert(gen) < 1.0) {
        r.point_angle_rules.push_back(generate_random_point_angle_rule(
            gen, distrib_pos, distrib_pos, distrib_pos, distrib_a, distrib_w, distrib_group
        ));
    }
    while (distrib_insert(gen) < 1.0) {
        r.circle_angle_rules.push_back(generate_random_circle_angle_rule(
            gen, distrib_pos, distrib_pos, distrib_pos, distrib_a, distrib_w, distrib_group
        ));
    }
    return r;
}

template <class Tgen, class Tdistrib>
PlacementRuleSet remove_random(
    const PlacementRuleSet &s, Tgen gen, Tdistrib &distrib_remove
) {
    PlacementRuleSet r;
    for (const PointPlaceRule &p : s.point_rules) {
        if (distrib_remove(gen) >= 1.0) {
            r.point_rules.push_back(p);
        }
    }
    for (const CirclePlaceRule &p : s.circle_rules) {
        if (distrib_remove(gen) >= 1.0) {
            r.circle_rules.push_back(p);
        }
    }
    for (const PointAngleRule &p : s.point_angle_rules) {
        if (distrib_remove(gen) >= 1.0) {
            r.point_angle_rules.push_back(p);
        }
    }
    for (const CircleAngleRule &p : s.circle_angle_rules) {
        if (distrib_remove(gen) >= 1.0) {
            r.circle_angle_rules.push_back(p);
        }
    }
    return r;
}

std::vector<PlacementRuleSet> augment_rules(
    const std::vector<PlacementRuleSet> &rules, const RuleMutationSettings &settings
) {
    std::random_device rd;
    std::mt19937 gen(rd());

    std::uniform_real_distribution<float>
        distrib_pos(-settings.dpos, settings.dpos),
        distrib_w(-settings.dw, settings.dw),
        distrib_a(-settings.da, settings.da),
        distrib_insert(0.0, 1.0 / (settings.insert_p + 1e-6)),
        distrib_remove(0.0, 1.0 / (settings.remove_p + 1e-6)),
        distrib_mutate(0.0, 1.0 / (settings.mutate_p + 1e-6));
    std::uniform_int_distribution<Tvertex> distrib_group(0, settings.ngroups - 1);

    std::vector<PlacementRuleSet> r;
    for (const PlacementRuleSet &s : rules) {
        auto new_s = remove_random(s, gen, distrib_remove);
        new_s = insert_random(new_s, gen, distrib_insert, distrib_pos, distrib_a, distrib_w, distrib_group);
        new_s = mutate_random(new_s, gen, distrib_mutate, distrib_pos, distrib_a, distrib_w);
        if (new_s.size() > 0) {
            r.push_back(new_s);
        }
    }
    return r;
}
