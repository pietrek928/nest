#pragma once

#include <cstddef>
#include "algo.h"

template<class Tcoord>
struct Poly2DPtr {
    std::size_t size;
    const Vec<2, Tcoord> *coords;
    Vec<3, int> grad_pos;
};

template<class Tgrad, class Tcoord>
void accum_dist_gradients(
    Tgrad *V, Tgrad *M, int Mwidth,
    const std::vector<Poly2DPtr<Tcoord>> &polys
) {
    //
}
