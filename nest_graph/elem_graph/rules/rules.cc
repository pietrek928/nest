#include "rules/rules.h"

#include <algorithm>
#include <cmath>
#include <random>
#include <vector>

template <class T>
T adjust_angle(T a) {
    if (a < 0) {
        return a + 2 * static_cast<T>(M_PI);
    }
    if (a > 2 * static_cast<T>(M_PI)) {
        return a - 2 * static_cast<T>(M_PI);
    }
    return a;
}

Vec2f sample_pos_in_region(const Circle2f &region, std::mt19937 &gen) {
    std::uniform_real_distribution<float> ang(0.0f, static_cast<float>(2.0 * M_PI));
    std::uniform_real_distribution<float> rad(0.0f, 1.0f);
    const float r = std::sqrt(rad(gen) * std::max(region.r_sq, 1e-12f));
    const float a = ang(gen);
    return Vec2f({
        region.c[0] + r * std::cos(a),
        region.c[1] + r * std::sin(a),
    });
}

Vec2f clamp_to_region(Vec2f p, const Circle2f &region) {
    Vec2f d = p - region.c;
    const float sq = d.len_sq();
    if (sq <= region.r_sq || sq < 1e-12f) {
        return p;
    }
    const float scale = std::sqrt(region.r_sq / sq) * 0.999f;
    return region.c + d * scale;
}

float default_rule_radius(const Circle2f &region) {
    return std::sqrt(std::max(region.r_sq, 1e-12f)) * 0.05f;
}

template <class Tgen, class Tdistrib>
Circle2f mutate_random_circle(
    const Circle2f &c, const Circle2f &region, Tgen &gen, Tdistrib &distrib_pos
) {
    Circle2f r = c;
    r.c = clamp_to_region(r.c + Vec2f({distrib_pos(gen), distrib_pos(gen)}), region);
    r.r_sq += distrib_pos(gen);
    if (r.r_sq < 0.0f) {
        r.r_sq = 0.0f;
    }
    return r;
}

template <class Tgen, class Tdistrib>
Circle2f generate_random_circle(
    const Circle2f &region, Tgen &gen, Tdistrib &distrib_r
) {
    const float rx = std::abs(distrib_r(gen));
    const float r_sq = rx * rx;
    return Circle2f(sample_pos_in_region(region, gen), r_sq);
}

template <class Tgen, class Tdistrib>
PointPlaceRule mutate_random(
    const PointPlaceRule &p, const Circle2f &region, Tgen &gen,
    Tdistrib &distrib_pos, Tdistrib &distrib_w
) {
    Vec2f pos = clamp_to_region(
        p.pos + Vec2f({distrib_pos(gen), distrib_pos(gen)}), region);
    return PointPlaceRule(
        pos,
        std::max(p.r + distrib_pos(gen), 1e-6f),
        p.w + distrib_w(gen),
        p.group);
}

template <class Tgen, class Tdistrib, class Tdistrib_group>
PointPlaceRule generate_random_point_place_rule(
    const Circle2f &region, Tgen &gen, Tdistrib &distrib_r,
    Tdistrib &distrib_w, Tdistrib_group &distrib_group
) {
    return PointPlaceRule(
        sample_pos_in_region(region, gen),
        std::max(std::abs(distrib_r(gen)), default_rule_radius(region)),
        distrib_w(gen),
        distrib_group(gen));
}

template <class Tgen, class Tdistrib>
CirclePlaceRule mutate_random(
    const CirclePlaceRule &p, const Circle2f &region, Tgen &gen,
    Tdistrib &distrib_pos, Tdistrib &distrib_w
) {
    return CirclePlaceRule(
        mutate_random_circle(p.circle, region, gen, distrib_pos),
        std::max(p.r + distrib_pos(gen), 1e-6f),
        p.w + distrib_w(gen),
        p.group);
}

template <class Tgen, class Tdistrib, class Tdistrib_group>
CirclePlaceRule generate_random_circle_place_rule(
    const Circle2f &region, Tgen &gen, Tdistrib &distrib_r,
    Tdistrib &distrib_w, Tdistrib_group &distrib_group
) {
    return CirclePlaceRule(
        generate_random_circle(region, gen, distrib_r),
        std::max(std::abs(distrib_r(gen)), default_rule_radius(region)),
        distrib_w(gen),
        distrib_group(gen));
}

template <class Tgen, class Tdistrib>
PointAngleRule mutate_random(
    const PointAngleRule &p, const Circle2f &region, Tgen &gen,
    Tdistrib &distrib_pos, Tdistrib &distrib_a, Tdistrib &distrib_w
) {
    Vec2f pos = clamp_to_region(
        p.pos + Vec2f({distrib_pos(gen), distrib_pos(gen)}), region);
    return PointAngleRule(
        pos,
        adjust_angle(p.a + distrib_a(gen)),
        std::max(p.r, 1e-6f),
        p.w + distrib_w(gen),
        p.group);
}

template <class Tgen, class Tdistrib, class Tdistrib_group>
PointAngleRule generate_random_point_angle_rule(
    const Circle2f &region, Tgen &gen, Tdistrib &distrib_r,
    Tdistrib &distrib_a, Tdistrib &distrib_w, Tdistrib_group &distrib_group
) {
    return PointAngleRule(
        sample_pos_in_region(region, gen),
        adjust_angle(distrib_a(gen)),
        std::max(std::abs(distrib_r(gen)), default_rule_radius(region)),
        distrib_w(gen),
        distrib_group(gen));
}

template <class Tgen, class Tdistrib>
CircleAngleRule mutate_random(
    const CircleAngleRule &p, const Circle2f &region, Tgen &gen,
    Tdistrib &distrib_pos, Tdistrib &distrib_a, Tdistrib &distrib_w
) {
    return CircleAngleRule(
        mutate_random_circle(p.circle, region, gen, distrib_pos),
        adjust_angle(p.a + distrib_a(gen)),
        std::max(p.r + distrib_pos(gen), 1e-6f),
        p.w + distrib_w(gen),
        p.group);
}

template <class Tgen, class Tdistrib, class Tdistrib_group>
CircleAngleRule generate_random_circle_angle_rule(
    const Circle2f &region, Tgen &gen, Tdistrib &distrib_r,
    Tdistrib &distrib_a, Tdistrib &distrib_w, Tdistrib_group &distrib_group
) {
    return CircleAngleRule(
        generate_random_circle(region, gen, distrib_r),
        adjust_angle(distrib_a(gen)),
        std::max(std::abs(distrib_r(gen)), default_rule_radius(region)),
        distrib_w(gen),
        distrib_group(gen));
}

template <class Tgen, class Tdistrib>
PlacementRuleSet mutate_random(
    const PlacementRuleSet &s, const Circle2f &region, Tgen &gen,
    Tdistrib &distrib_select, Tdistrib &distrib_pos,
    Tdistrib &distrib_a, Tdistrib &distrib_w
) {
    PlacementRuleSet r;
    r.point_rules.reserve(s.point_rules.size());
    r.circle_rules.reserve(s.circle_rules.size());
    r.point_angle_rules.reserve(s.point_angle_rules.size());
    r.circle_angle_rules.reserve(s.circle_angle_rules.size());

    for (const PointPlaceRule &p : s.point_rules) {
        if (distrib_select(gen) < 1.0f) {
            r.point_rules.push_back(mutate_random(p, region, gen, distrib_pos, distrib_w));
        } else {
            r.point_rules.push_back(p);
        }
    }
    for (const CirclePlaceRule &p : s.circle_rules) {
        if (distrib_select(gen) < 1.0f) {
            r.circle_rules.push_back(mutate_random(p, region, gen, distrib_pos, distrib_w));
        } else {
            r.circle_rules.push_back(p);
        }
    }
    for (const PointAngleRule &p : s.point_angle_rules) {
        if (distrib_select(gen) < 1.0f) {
            r.point_angle_rules.push_back(
                mutate_random(p, region, gen, distrib_pos, distrib_a, distrib_w));
        } else {
            r.point_angle_rules.push_back(p);
        }
    }
    for (const CircleAngleRule &p : s.circle_angle_rules) {
        if (distrib_select(gen) < 1.0f) {
            r.circle_angle_rules.push_back(
                mutate_random(p, region, gen, distrib_pos, distrib_a, distrib_w));
        } else {
            r.circle_angle_rules.push_back(p);
        }
    }
    return r;
}

template <class Tgen>
int capped_inserts(float insert_p, int max_inserts, Tgen &gen) {
    if (insert_p <= 1e-8f || max_inserts <= 0) {
        return 0;
    }
    std::geometric_distribution<int> dist(insert_p);
    int count = 0;
    while (count < max_inserts && dist(gen)) {
        ++count;
    }
    return count;
}

template <class Tgen, class Tdistrib, class Tdistrib_group>
PlacementRuleSet insert_random(
    const PlacementRuleSet &s, const Circle2f &region,
    const RuleMutationSettings &settings, Tgen &gen,
    Tdistrib &distrib_pos, Tdistrib &distrib_a,
    Tdistrib &distrib_w, Tdistrib &distrib_r,
    Tdistrib_group &distrib_group
) {
    PlacementRuleSet r = s;
    const int cap = std::max(settings.max_inserts_per_type, 0);
    const bool had_circle = !s.circle_rules.empty() || !s.circle_angle_rules.empty();

    const int n_point = capped_inserts(settings.insert_p, cap, gen);
    r.point_rules.reserve(r.point_rules.size() + static_cast<size_t>(n_point));
    for (int i = 0; i < n_point; ++i) {
        r.point_rules.push_back(
            generate_random_point_place_rule(region, gen, distrib_r, distrib_w, distrib_group));
    }

    if (had_circle) {
        const int n_circle = capped_inserts(settings.insert_p, cap, gen);
        r.circle_rules.reserve(r.circle_rules.size() + static_cast<size_t>(n_circle));
        for (int i = 0; i < n_circle; ++i) {
            r.circle_rules.push_back(generate_random_circle_place_rule(
                region, gen, distrib_r, distrib_w, distrib_group));
        }
    }

    const int n_pangle = capped_inserts(settings.insert_p, cap, gen);
    r.point_angle_rules.reserve(r.point_angle_rules.size() + static_cast<size_t>(n_pangle));
    for (int i = 0; i < n_pangle; ++i) {
        r.point_angle_rules.push_back(generate_random_point_angle_rule(
            region, gen, distrib_r, distrib_a, distrib_w, distrib_group));
    }

    if (had_circle) {
        const int n_cangle = capped_inserts(settings.insert_p, cap, gen);
        r.circle_angle_rules.reserve(
            r.circle_angle_rules.size() + static_cast<size_t>(n_cangle));
        for (int i = 0; i < n_cangle; ++i) {
            r.circle_angle_rules.push_back(generate_random_circle_angle_rule(
                region, gen, distrib_r, distrib_a, distrib_w, distrib_group));
        }
    }
    return r;
}

template <class Tgen, class Tdistrib>
PlacementRuleSet remove_random(
    const PlacementRuleSet &s, Tgen &gen, Tdistrib &distrib_remove
) {
    PlacementRuleSet r;
    for (const PointPlaceRule &p : s.point_rules) {
        if (distrib_remove(gen) >= 1.0f) {
            r.point_rules.push_back(p);
        }
    }
    for (const CirclePlaceRule &p : s.circle_rules) {
        if (distrib_remove(gen) >= 1.0f) {
            r.circle_rules.push_back(p);
        }
    }
    for (const PointAngleRule &p : s.point_angle_rules) {
        if (distrib_remove(gen) >= 1.0f) {
            r.point_angle_rules.push_back(p);
        }
    }
    for (const CircleAngleRule &p : s.circle_angle_rules) {
        if (distrib_remove(gen) >= 1.0f) {
            r.circle_angle_rules.push_back(p);
        }
    }
    return r;
}

std::vector<PlacementRuleSet> augment_rules(
    const std::vector<PlacementRuleSet> &rules,
    const RuleMutationSettings &settings,
    std::uint32_t seed
) {
    std::mt19937 gen(seed != 0 ? seed : std::random_device{}());

    const Circle2f &region = settings.region;
    const Tvertex ng = std::max<Tvertex>(settings.ngroups, 1);

    std::uniform_real_distribution<float>
        distrib_pos(-settings.dpos, settings.dpos),
        distrib_w(-settings.dw, settings.dw),
        distrib_a(-settings.da, settings.da),
        distrib_r(0.0f, settings.dpos * 0.5f),
        distrib_remove(0.0f, 1.0f / (settings.remove_p + 1e-6f)),
        distrib_mutate(0.0f, 1.0f / (settings.mutate_p + 1e-6f));
    std::uniform_int_distribution<Tvertex> distrib_group(0, ng - 1);

    std::vector<PlacementRuleSet> r;
    r.reserve(rules.size());
    for (const PlacementRuleSet &s : rules) {
        auto new_s = remove_random(s, gen, distrib_remove);
        new_s = insert_random(
            new_s, region, settings, gen, distrib_pos, distrib_a, distrib_w, distrib_r, distrib_group);
        new_s = mutate_random(
            new_s, region, gen, distrib_mutate, distrib_pos, distrib_a, distrib_w);
        if (new_s.size() > 0) {
            r.push_back(std::move(new_s));
        }
    }
    return r;
}
