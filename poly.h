#ifndef __POLY_H_
#define __POLY_H_

#include <vector>

#include "bbox.h"
#include "vec.h"

template <int K, class T>
class ConvexPolygon {
    std::vector<Vec<K, T>> pts;

   public:
    bool is_point_inside_brute(const Vec<K, T>& pt_q) {
        auto last_point = pts.back();
        for (auto& pt : pts) {
            if (!is_point_on_left(last_point, pt, pt_q)) {
                return false;
            }
            last_point = pt;
        }
        return true;
    }

    ConvexPolygon() {}
};

#endif /* __POLY_H_ */
