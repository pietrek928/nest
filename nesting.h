#pragma once

#include <cstddef>
#include "algo.h"

template<class Tcoord>
struct Poly2DGradPtr {
    std::size_t size;
    const Vec<2, Diff2<3, Tcoord>> *coords;
    Vec<3, int> grad_pos;
};

template<class Tgrad, class Tcoord>
void accum_dist_gradients(
    Tgrad *V, Tgrad *M, int Mwidth,
    const Poly2DGradPtr<Tcoord> *polys, int n,
    Tgrad dist_thresh
) {
    Tgrad qdist_thresh = dist_thresh * dist_thresh;
    Tgrad grad_val = 0;
    for (int i=0; i<n; i++) {
        for (int j=0; j<n; j++) if (i != j) {
            auto grad = points_line_ring_distance_g3(
                polys[i].coords, polys[i].size,
                polys[j].coords, polys[j].size,
                [=](const auto &qdist) {
                    if (qdist > qdist_thresh) {
                        return (qdist ^ .5 - qdist_thresh) ^ 2;
                    } else {
                        return (qdist_thresh - qdist ^ .5) ^ 2;
                    }
                }
            );
            auto vi_pos = polys[i].grad_pos;
            auto vj_pos = polys[j].grad_pos;
            Vec<6, int> vpos = {
                vi_pos[0], vi_pos[1], vi_pos[2],
                vj_pos[0], vj_pos[1], vj_pos[2],
            };
            grad.get_dx().add_to_vec(V, vpos);
            grad.get_d2x().add_to_mat(M, Mwidth, vpos);
        }
    }
}
