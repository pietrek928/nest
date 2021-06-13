#ifndef __ALGO_H_
#define __ALGO_H_

#include "vec.h"


template<int K, class T>
Vec<K, T> triangle_height_vector(
    const Vec<K, T> &A,
    const Vec<K, T> &B,
    const Vec<K, T> &C
) {
    auto ab = A - B;
    auto ac = A - C;

    auto kab = (ab.dp(ac) / ab.qlen());

    return ac - ab * kab;
}

template<class T>
bool is_point_on_left(
    const Vec<2, T> &A,
    const Vec<2, T> &B,
    const Vec<2, T> &C
) {
    auto ab = A - B;
    auto ac = A - C;

    return ab.get_pos(0)*ac.get_pos(1) < ab.get_pos(1)*ac.get_pos(0);
}

template<int K, class T>
T triangle_height_qlen(
    const Vec<K, T> &A,
    const Vec<K, T> &B,
    const Vec<K, T> &C
) {
    return triangle_height_vector(A, B, C).qlen();
}

//template<int K, class T>
//T vector_to_outside(
    //const Vec<K, T> &A,
    //const Vec<K, T> &B,
    //const Vec<K, T> &C
//) {
//}

#endif /* __ALGO_H_ */

