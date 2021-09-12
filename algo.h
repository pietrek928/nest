#ifndef __ALGO_H_
#define __ALGO_H_

#include <algorithm>

#include "diff2.h"
#include "utils.h"
#include "vec.h"

template <int K, class T>
Vec<K, T> triangle_height_vector(
    const Vec<K, T>& A, const Vec<K, T>& B, const Vec<K, T>& C) {
    auto ab = A - B;
    auto ac = A - C;

    auto kab = (ab.dp(ac) / ab.qlen());

    return ac - ab * kab;
}

template <class T>
bool is_point_on_left(
    const Vec<2, T>& A, const Vec<2, T>& B, const Vec<2, T>& C) {
    auto ab = A - B;
    auto ac = A - C;

    return ab.get_pos(0) * ac.get_pos(1) < ab.get_pos(1) * ac.get_pos(0);
}

template <class T>
bool vectors_intersect(
    const Vec<2, T>& A,
    const Vec<2, T>& B,
    const Vec<2, T>& C,
    const Vec<2, T>& D) {
    auto ab = B - A;
    auto ac = C - A;
    auto ad = D - A;

    auto ca = -ac;
    auto cb = B - C;
    auto cd = C - C;

    return (cross2d(ac, ab) * cross2d(ab, ad) < 0) &&
           (cross2d(ca, cd) * cross2d(cd, cb) < 0);
}

template <int K, class T>
T triangle_height_qlen(
    const Vec<K, T>& A, const Vec<K, T>& B, const Vec<K, T>& C) {
    return triangle_height_vector(A, B, C).qlen();
}

template <class T>
class PolygonTransformer {
    Vec<2, T> d;
    T ca, sa;

    PolygonTransformer(T dx, T dy, T a)
        : d(dx, dy), ca(std::cos(a)), sa(std::sin(a)) {}

    auto transform(const Vec<2, T>& v) {
        return Vec<2, T>(v[0] * ca - v[1] * sa, v[0] * sa + v[1] * ca) + d;
    }
};

template <class T>
class PolygonTransformerGrad {
    Vec<2, T> d;
    Diff2<3, T> ca, sa;

    PolygonTransformerGrad(T dx, T dy, T a)
        : d(dx, dy), ca(std::cos(a)), sa(std::sin(a)) {
        auto cs = Diff2<3, T>::from_var(a, 2).cossin();
        ca = cs.cos;
        sa = cs.sin;
    }

    auto transform(const Vec<2, T>& v) {
        return Vec<2, T>(
                   v[0] * (T)ca - v[1] * (T)sa, v[0] * (T)sa + v[1] * (T)ca) +
               d;
    }

    auto transform_grad(const Vec<2, T>& v) {
        return Vec<2, Diff2<3, T>>(
            v[0] * ca - v[1] * sa + Diff2<3, T>::from_var(d[0], 0),
            v[0] * sa + v[1] * ca + Diff2<3, T>::from_var(d[1], 1));
    }
};

template <class Tv>
class PolygonIterator {
    const Tv* points;
    int n, i;

    int next_item(int i) {
        return (i + 1) % n;
        // i++;
        // if (i == n) {  // TODO: unlikely
        //     return n - 1;
        // }
        // return i;
    }

    int prev_item(int i) {
        return (i + (n - 1)) % n;
        // if (!i) {  // TODO: unlikely
        //     return n - 1;
        // }
        // return i - 1;
    }

   public:
    Tv pt_cur, pt_next, pt_prev;

    PolygonIterator(const Tv* points, int n) : points(points), n(n) {
        go_to(0);
    }

    PolygonIterator(const Tv* points, int n, int start_pos)
        : points(points), n(n) {
        go_to(start_pos);
    }

    inline auto count() {
        return n;
    }

    inline auto get_pos() {
        return i;
    }

    inline void go_to(int new_i) {
        i = new_i;
        pt_prev = points[prev_item(i)];
        pt_cur = points[i];
        pt_next = points[next_item(i)];
    }

    inline void go_next() {
        i = next_item(i);
        rotl(pt_prev, pt_cur, pt_next, points[next_item(i)]);
    }

    inline void go_prev() {
        i = prev_item(i);
        rotl(pt_next, pt_cur, pt_prev, points[prev_item(i)]);
    }
};

template <class Tv>
inline void walk_to_nearest_points(
    PolygonIterator<Tv>& p1, PolygonIterator<Tv>& p2) {
    while (true) {
        auto p1_prev_dist = p1.pt_prev.qdist(p2.pt_cur);
        auto p1_next_dist = p1.pt_next.qdist(p2.pt_cur);
        auto cur_dist = p1.pt_cur.qdist(p2.pt_cur);
        cout << "-" << endl;
        if (p1_prev_dist < p1_next_dist) {
            if (p1_prev_dist < cur_dist) {
                p1.go_prev();
                continue;
            }
        } else {
            if (p1_next_dist < cur_dist) {
                p1.go_next();
                continue;
            }
        }
        auto p2_prev_dist = p2.pt_prev.qdist(p1.pt_cur);
        auto p2_next_dist = p2.pt_next.qdist(p1.pt_cur);
        cout << "-" << endl;
        if (p2_prev_dist < p2_next_dist) {
            if (p2_prev_dist < cur_dist) {
                p2.go_prev();
                continue;
            }
        } else {
            if (p2_next_dist < cur_dist) {
                p2.go_next();
                continue;
            }
        }
        return;
    }
}

#if 0
template <class Tv>
inline bool walk_to_nearest_edge(
    PolygonIterator<Tv>& p1, PolygonIterator<Tv>& p2) {
    while (true) {
        auto bc = p1.pt_prev - p1.pt_cur;
        auto ba = p1.pt_next - p1.pt_cur;
        auto d = p2.pt_cur - p1.pt_cur;

        auto cc = bc.dp(d);
        auto ca = ba.dp(d);

        if (cc < 0) {
            auto la = ca.qlen();
            if (< la) {
                auto sa = cross2d(bc, d);
            }
        }
        break;
    }
    return false;
}
#endif

template <class T>
inline bool convex_polygons_intersect(
    PolygonIterator<Vec<2, T>>& p1,
    PolygonIterator<Vec<2, T>>& p2,
    const bool direction = false) {
    int op_limit = p1.count() + p2.count() + 16;
    auto d = p1.pt_cur - p2.pt_cur;
    do {
        auto op_limit_old = op_limit;
        {
            auto d2 = p1.pt_next - p2.pt_cur;
            while (turns_direction(d, d2, direction)) {
                p1.go_next();
                d = d2;
                d2 = p1.pt_next - p2.pt_cur;
                if (--op_limit <= 0) {
                    break;
                }
            }
        }
        while (turns_direction(d, p2.pt_next - p2.pt_cur, !direction)) {
            p2.go_next();
            d = p1.pt_cur - p2.pt_cur;
            if (--op_limit <= 0) {
                break;
            }
        }
        {
            auto d2 = p1.pt_prev - p2.pt_cur;
            while (turns_direction(d, d2, direction)) {
                p1.go_prev();
                d = d2;
                d2 = p1.pt_prev - p2.pt_cur;
                if (--op_limit <= 0) {
                    break;
                }
            }
        }
        while (turns_direction(d, p2.pt_prev - p2.pt_cur, !direction)) {
            p2.go_prev();
            d = p1.pt_cur - p2.pt_cur;
            if (--op_limit <= 0) {
                break;
            }
        }
        if (op_limit == op_limit_old) {
            // all conditions met - polygons do not intersect
            return false;
        }
    } while (op_limit > 0);
    return true;
}

template <class T>
inline bool convex_polygons_local_qdist(
    PolygonIterator<Vec<2, T>>& p1, PolygonIterator<Vec<2, T>>& p2, T& qdist) {
    bool cont = false;

    auto d21 = p1.pt_cur - p2.pt_cur;
    auto d21_qlen = d21.qlen();
    if (d21_qlen < qdist) {
        qdist = d21_qlen;
        cont = true;
    }

    {
        cout << "*" << endl;
        auto dprev = p2.pt_prev - p2.pt_cur;
        auto mprev = d21.dp(dprev);
        if (mprev > 0) {
            cout << "." << endl;
            mprev *= mprev;
            auto dprev_qlen = dprev.qlen();
            auto qdist_p = mprev / dprev_qlen;
            if (qdist_p < dprev_qlen) {
                auto qdist2 = d21_qlen - qdist_p;
                if (qdist2 < qdist) {
                    qdist = qdist2;
                    p2.go_prev();
                    return true;
                }
            }
        }
    }
    {
        cout << "*" << endl;
        auto dnext = p2.pt_next - p2.pt_cur;
        auto mnext = d21.dp(dnext);
        if (mnext > 0) {
            cout << "." << endl;
            mnext *= mnext;
            auto dnext_qlen = dnext.qlen();
            auto qdist_p = mnext / dnext_qlen;
            if (qdist_p < dnext_qlen) {
                auto qdist2 = d21_qlen - qdist_p;
                if (qdist2 < qdist) {
                    qdist = qdist2;
                    p2.go_next();
                    return true;
                }
            }
        }
    }
    return cont;
}

template <class T>
inline auto convex_polygons_qdist(
    PolygonIterator<Vec<2, T>>& p1, PolygonIterator<Vec<2, T>>& p2) {
    walk_to_nearest_points(p1, p2);

    T qdist = 1e9;

    while (convex_polygons_local_qdist(p1, p2, qdist) ||
           convex_polygons_local_qdist(p2, p1, qdist))
        ;
    return qdist;
}

#endif /* __ALGO_H_ */
