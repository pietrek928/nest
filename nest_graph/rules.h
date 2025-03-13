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
} PlacementRuleSet;


template<class Tgen, class Tdistrib>
BBox mutate_random(const BBox &b, Tgen gen, const Tdistrib &distrib) {
    return {
        .xstart = b.xstart + distrib(gen),
        .xend = b.xend + distrib(gen),
        .ystart = b.ystart + distrib(gen),
        .yend = b.yend + distrib(gen)
    };
}

template<class Tgen, class Tdistrib>
BBox generate_random(Tgen gen, const Tdistrib &distrib_x, const Tdistrib &distrib_y) {
    auto x1 = distrib_x(gen);
    auto x2 = distrib_x(gen);
    auto y1 = distrib_y(gen);
    auto y2 = distrib_y(gen);

    return {
        .xstart = std::min(x1, x2),
        .xend = std::max(x1, x2),
        .ystart = std::min(y1, y2),
        .yend = std::max(y1, y2)
    };
}

template<class Tgen, class Tdistrib>
PointPlaceRule mutate_random(
    const PointPlaceRule &p, Tgen gen, const Tdistrib &distrib_pos, const Tdistrib &distrib_w
) {
    return {
        .x = p.x + distrib_pos(gen),
        .y = p.y + distrib_pos(gen),
        .r = p.r + distrib_pos(gen),
        .w = p.w + distrib_w(gen),
        .group = p.group
    };
}

template<class Tgen, class Tdistrib, class Tdistrib_group>
PointPlaceRule generate_random(
    Tgen gen, const Tdistrib &distrib_x, const Tdistrib &distrib_y,
    const Tdistrib &distrib_r, const Tdistrib &distrib_w, const Tdistrib_group &distrib_group
) {
    return {
        .x = distrib_x(gen),
        .y = distrib_y(gen),
        .r = distrib_r(gen),
        .w = distrib_w(gen),
        .group = distrib_group(gen)
    };
}

template<class Tgen, class Tdistrib>
BBoxPlaceRule mutate_random(
    const BBoxPlaceRule &p, Tgen gen, const Tdistrib &distrib_pos, const Tdistrib &distrib_w
) {
    return {
        .bbox = mutate_random(p.bbox, gen, distrib_pos),
        .r = p.r + distrib_pos(gen),
        .w = p.w + distrib_w(gen),
        .group = p.group
    };
}

template<class Tgen, class Tdistrib, class Tdistrib_group>
BBoxPlaceRule generate_random(
    Tgen gen, const Tdistrib &distrib_x, const Tdistrib &distrib_y,
    const Tdistrib &distrib_r, const Tdistrib &distrib_w, const Tdistrib_group &distrib_group
) {
    return {
        .bbox = generate_random(gen, distrib_x, distrib_y),
        .r = distrib_r(gen),
        .w = distrib_w(gen),
        .group = distrib_group(gen)
    };
}

template<class Tgen, class Tdistrib>
PointAngleRule mutate_random(
    const PointAngleRule &p, Tgen gen, const Tdistrib &distrib_pos,
    const Tdistrib &distrib_a, const Tdistrib &distrib_w
) {
    return {
        .x = p.x + distrib_pos(gen),
        .y = p.y + distrib_pos(gen),
        .a = p.a + distrib_a(gen),
        .w = p.w + distrib_w(gen),
        .group = p.group
    };
}

template<class Tgen, class Tdistrib, class Tdistrib_group>
PointAngleRule generate_random(
    Tgen gen, const Tdistrib &distrib_x, const Tdistrib &distrib_y,
    const Tdistrib &distrib_r, const Tdistrib &distrib_a,
    const Tdistrib &distrib_w, const Tdistrib_group &distrib_group
) {
    return {
        .x = distrib_x(gen),
        .y = distrib_y(gen),
        .a = distrib_a(gen),
        .w = distrib_w(gen),
        .group = distrib_group(gen)
    };
}
    


template<class Tgen, class Tdistrib>
BBoxAngleRule mutate_random(
    const BBoxAngleRule &p, Tgen gen,
    const Tdistrib &distrib_pos, const Tdistrib &distrib_a, const Tdistrib &distrib_w
) {
    return {
        .bbox = mutate_random(p.bbox, gen, distrib_pos),
        .a = p.a + distrib_a(gen),
        .r = p.r + distrib_pos(gen),
        .w = p.w + distrib_w(gen),
        .group = p.group
    };
}

template<class Tgen, class Tdistrib, class Tdistrib_group>
BBoxAngleRule generate_random(
    Tgen gen, const Tdistrib &distrib_x, const Tdistrib &distrib_y,
    const Tdistrib &distrib_r, const Tdistrib &distrib_a,
    const Tdistrib &distrib_w, const Tdistrib_group &distrib_group
) {
    return {
        .bbox = generate_random(gen, distrib_x, distrib_y),
        .a = distrib_a(gen),
        .r = distrib_r(gen),
        .w = distrib_w(gen),
        .group = distrib_group(gen)
    };
}

template<class Tgen, class Tdistrib>
PlacementRuleSet mutate_random(
    const PlacementRuleSet &s, Tgen gen, const Tdistrib &distrib_select,
    const Tdistrib &distrib_pos, const Tdistrib &distrib_a, const Tdistrib &distrib_w
) {
    PlacementRuleSet r;
    for (const PointPlaceRule &p : s.point_rules) {
        if (distrib_select(gen) < 1.0) {
            r.point_rules.push_back(mutate_random(p, gen, distrib_pos, distrib_w));
        } else {
            r.point_rules.push_back(p);
        }
    }
    for (const BBoxPlaceRule &p : s.bbox_rules) {
        if (distrib_select(gen) < 1.0) {
            r.bbox_rules.push_back(mutate_random(p, gen, distrib_pos, distrib_w));
        } else {
            r.bbox_rules.push_back(p);
        }
    }
    for (const PointAngleRule &p : s.point_angle_rules) {
        if (distrib_select(gen) < 1.0) {
            r.point_angle_rules.push_back(mutate_random(p, gen, distrib_pos, distrib_a, distrib_w));
        } else {
            r.point_angle_rules.push_back(p);
        }
    }
    for (const BBoxAngleRule &p : s.bbox_angle_rules) {
        if (distrib_select(gen) < 1.0) {
            r.bbox_angle_rules.push_back(mutate_random(p, gen, distrib_pos, distrib_a, distrib_w));
        } else {
            r.bbox_angle_rules.push_back(p);
        }
    }
    return r;
}

template<class Tgen, class Tdistrib, class Tdistrib_group>
PlacementRuleSet insert_random(
    const PlacementRuleSet &s, Tgen gen, const Tdistrib &distrib_insert,
    const Tdistrib &distrib_pos, const Tdistrib &distrib_a,
    const Tdistrib &distrib_w, const Tdistrib_group &distrib_group
) {
    PlacementRuleSet r = s;
    while (distrib_insert(gen) < 1.0) {
        r.point_rules.push_back(generate_random(
            gen, distrib_pos, distrib_pos, distrib_pos, distrib_w, distrib_group
        ));
    }
    while (distrib_insert(gen) < 1.0) {
        r.bbox_rules.push_back(generate_random(
            gen, distrib_pos, distrib_pos, distrib_pos, distrib_w, distrib_group
        ));
    }
    while (distrib_insert(gen) < 1.0) {
        r.point_angle_rules.push_back(generate_random(
            gen, distrib_pos, distrib_pos, distrib_pos, distrib_a, distrib_w, distrib_group
        ));
    }
    while (distrib_insert(gen) < 1.0) {
        r.bbox_angle_rules.push_back(generate_random(
            gen, distrib_pos, distrib_pos, distrib_pos, distrib_a, distrib_w, distrib_group
        ));
    }
    return r;
}

template<class Tgen, class Tdistrib>
PlacementRuleSet remove_random(
    const PlacementRuleSet &s, Tgen gen, const Tdistrib &distrib_remove
) {
    PlacementRuleSet r;
    for (const PointPlaceRule &p : s.point_rules) {
        if (distrib_remove(gen) >= 1.0) {
            r.point_rules.push_back(p);
        }
    }
    for (const BBoxPlaceRule &p : s.bbox_rules) {
        if (distrib_remove(gen) >= 1.0) {
            r.bbox_rules.push_back(p);
        }
    }
    for (const PointAngleRule &p : s.point_angle_rules) {
        if (distrib_remove(gen) >= 1.0) {
            r.point_angle_rules.push_back(p);
        }
    }
    for (const BBoxAngleRule &p : s.bbox_angle_rules) {
        if (distrib_remove(gen) >= 1.0) {
            r.bbox_angle_rules.push_back(p);
        }
    }
    return r;
}

