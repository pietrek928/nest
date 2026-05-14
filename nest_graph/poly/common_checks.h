#include <cmath>

#include "poly.h"
#include "convex/contain.h"


enum class SweepMode {
    Monopartite, // All vs All (Skips self-collisions)
    Bipartite,   // Set A vs Set B (Skips intra-set collisions)
    Subset       // Active vs All (Skips Static vs Static)
};

template<class VecType>
struct PartSweepElement {
    int poly_idx;          // Original index in the user's vector
    int part_idx;          // Which convex part of the polygon this is
    int group_id;          // 0 = Set A, 1 = Set B (used for bipartite checks)

    typename VecType::Scalar min_proj;
    typename VecType::Scalar max_proj;

    const Polygon<VecType>* poly_ptr; // NEW: Decouples element from its source vector
    const Circle<VecType>* bounds;
};

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
