#include <iostream>
#include <vector>
#include "nest_graph/geometry/guide/guide.h"
#include "nest_graph/geometry/solid/solid_geometry.h"

int main() {
    using Vec2 = Vector2<double>;
    std::vector<SolidGeometry<Vec2>> polys;
    
    // Placed poly: 1x1 square at origin
    SolidGeometry<Vec2> p1;
    p1.line_parts.push_back({{{0,0}, {1,0}, {1,1}, {0,1}}, false});
    p1.update_bounds();
    polys.push_back(p1);
    
    // Obstacle: 1x1 square at (3, 0)
    SolidGeometry<Vec2> p2;
    p2.line_parts.push_back({{{3,0}, {4,0}, {4,1}, {3,1}}, false});
    p2.update_bounds();
    polys.push_back(p2);
    
    GuidanceConfig<Vec2> cfg;
    cfg.use_gravity = false;
    cfg.use_target_attractor = false;
    cfg.use_tight_packing = true;
    cfg.use_corner_alignment = false;
    cfg.max_alternative_angles = 1;
    
    auto guidance = evaluate_local_placement<Vec2>(0, polys, {0,0}, cfg);
    
    for (const auto& prop : guidance.propositions) {
        std::cout << "Move: " << prop.move_type 
                  << " | Trans: " << prop.translation[0] << "," << prop.translation[1] 
                  << " | Rot: " << prop.rotation_rad << "\n";
    }
    return 0;
}
