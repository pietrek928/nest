#pragma once

#include "poly.h"

template<class VecType>
inline bool is_point_inside_solid_space(const VecType& pt, const SolidGeometry<VecType>& poly) {
    if (!poly.boundary_rings.empty()) {
        int winding = 0;

        for (const auto& ring : poly.boundary_rings) {
            const int n = static_cast<int>(ring.points.size());
            if (n < 3) {
                continue;
            }
            const int delta = ring.is_subtractive ? -1 : 1;

            for (int i = 0; i < n; ++i) {
                const VecType& v1 = ring.points[i];
                const VecType& v2 = ring.points[(i + 1) % n];

                if (((v1[1] > pt[1]) != (v2[1] > pt[1])) &&
                    (pt[0] < (v2[0] - v1[0]) * (pt[1] - v1[1]) / (v2[1] - v1[1]) + v1[0])) {
                    winding += delta;
                }
            }
        }
        // Odd winding: even counts from parallel slab edges must not read as inside.
        return (winding & 1) != 0;
    }

    // Fallback for manually built geometry without stored rings
    int winding = 0;
    for (size_t part = 0; part < poly.line_parts.size(); ++part) {
        const VecType* pts = poly.get_part_points(part);
        int n = poly.get_part_size(part);
        const int delta = poly.line_parts[part].is_subtractive ? -1 : 1;

        for (int i = 0; i < n - 1; ++i) {
            const VecType& v1 = pts[i];
            const VecType& v2 = pts[i + 1];

            if (((v1[1] > pt[1]) != (v2[1] > pt[1])) &&
                (pt[0] < (v2[0] - v1[0]) * (pt[1] - v1[1]) / (v2[1] - v1[1]) + v1[0])) {
                winding += delta;
            }
        }
    }
    return (winding & 1) != 0;
}
