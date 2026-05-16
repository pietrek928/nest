#pragma once

#include "poly.h"

template<class VecType>
inline bool is_point_inside_solid_space(const VecType& pt, const SolidGeometry<VecType>& poly) {
    int crossings = 0;

    for (size_t part = 0; part < poly.line_parts.size(); ++part) {
        const VecType* pts = poly.get_part_points(part);
        int n = poly.get_part_size(part);

        for (int i = 0; i < n; ++i) {
            const VecType& v1 = pts[i];
            const VecType& v2 = pts[(i + 1) % n];

            if (((v1[1] > pt[1]) != (v2[1] > pt[1])) &&
                (pt[0] < (v2[0] - v1[0]) * (pt[1] - v1[1]) / (v2[1] - v1[1]) + v1[0])) {
                crossings++;
            }
        }
    }
    return (crossings % 2) != 0;
}
