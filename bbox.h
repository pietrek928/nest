#ifndef __BBOX_H_
#define __BBOX_H_

#include "vec.h"

template <int K, class T>
class BBox {
    Vec<K, T> start, end;

   public:
    using Tnum = T;

    BBox() {}

    BBox(const Vec<K, T>& start, const Vec<K, T>& end) : start(start), end(end) {}

    template <int Kmax = 1024>
    inline auto get_pos_start(int k) const {
        return start.template get_pos<Kmax>(k);
    }

    template <int Kmax = 1024>
    inline auto get_pos_end(int k) const {
        return end.template get_pos<Kmax>(k);
    }
};

template <int N, class To1, class To2>
inline bool object_bbox_intersects(const To1& o1, const To2& o2) {
    if (o1.get_pos_end(N - 1) < o2.get_pos_start(N - 1) ||
        o2.get_pos_end(N - 1) < o1.get_pos_start(N - 1)) {
        return false;
    }
    if constexpr (N > 1) {
        return object_bbox_intersects<N - 1>(o1, o2);
    }
    return true;
}

#endif /* __BBOX_H_ */
