#pragma once

#include <cmath>

#include "poly.h"
#include "convex/contain.h"


template<class VecType>
inline bool narrow_phase_contain(
    const VecType* polyA, int nA,
    const VecType* polyB, int nB,
    int GRADIENT_THRESHOLD = 24
) {
    if (nA + nB > GRADIENT_THRESHOLD) {
        return is_polygon_fully_inside_gradient<VecType>(polyA, nA, polyB, nB);
    } else {
        return is_polygon_fully_inside<VecType>(polyA, nA, polyB, nB);
    }
}

// -------------------------------------------------------------------------
// INVALIDATION & PAIR CHECKS
// -------------------------------------------------------------------------
template<class VecType>
inline bool is_invalidated_by_hole(
    const VecType* ptsA, int nA, const Polygon<VecType>& polyA,
    const VecType* ptsB, int nB, const Polygon<VecType>& polyB
) {
    // Check if Part A is entirely swallowed by a hole in Poly B
    for (size_t hb = 0; hb < polyB.convex_holes.size(); ++hb) {
        if (narrow_phase_contain<VecType>(ptsA, nA, polyB.get_hole_points(hb), polyB.get_hole_size(hb))) {
            return true;
        }
    }

    // Check if Part B is entirely swallowed by a hole in Poly A
    for (size_t ha = 0; ha < polyA.convex_holes.size(); ++ha) {
        if (narrow_phase_contain<VecType>(ptsB, nB, polyA.get_hole_points(ha), polyA.get_hole_size(ha))) {
            return true;
        }
    }

    return false;
}
