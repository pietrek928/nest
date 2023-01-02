#ifndef __ALGO_H_
#define __ALGO_H_

#include <algorithm>
#include <vector>
#include <stdexcept>

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

// template <int K, class T>
// T triangle_height_qlen(
//     const Vec<K, T>& A, const Vec<K, T>& B, const Vec<K, T>& C) {
//     return triangle_height_vector(A, B, C).qlen();
// }

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
    auto d21 = p1.pt_cur - p2.pt_cur;
    auto d21_qlen = d21.qlen();
    if (d21_qlen < qdist) {
        cout << p1.pt_cur << " " << p2.pt_cur << endl;
        cout << "aaaaaaaaaaa " << d21_qlen << endl;
        qdist = d21_qlen;
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
                    // p2.go_prev();
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
                    // p2.go_next();
                    return true;
                }
            }
        }
    }
    return false;
}

template <class T>
inline T segment_qdist(const Vec<2, T> d, const Vec<2, T> v) {
    auto d_qlen = d.qlen();

    auto m = d.dp(v);
    if (m >= 0) {
        return d_qlen;
    }

    auto v_qlen = v.qlen();
    auto vp_qlen = m * m / v_qlen;
    if (vp_qlen > v_qlen) {
        return (v + d).qlen();
    }

    auto qlen = d_qlen - vp_qlen;
    cout << "aaaaaaaaa " << qlen << " " << d << " " << v << endl;

    return qlen;
}

template <class T>
inline T convex_polygons_qdist(
    PolygonIterator<Vec<2, T>>& p1, PolygonIterator<Vec<2, T>>& p2) {
    T qdist = 1e18, tmp_qdist;
    bool cont = false;

    do {
        cont = false;
        auto d12 = p2.pt_cur - p1.pt_cur;

        while (true) {
            bool cont2 = false;

            while ((tmp_qdist = segment_qdist(d12, p2.pt_next - p2.pt_cur)) <
                   qdist) {
                qdist = tmp_qdist;
                p2.go_next();
                d12 = p2.pt_cur - p1.pt_cur;
                cont = true;
            }

            while ((tmp_qdist = segment_qdist(-d12, p1.pt_prev - p1.pt_cur)) <
                   qdist) {
                qdist = tmp_qdist;
                p1.go_prev();
                d12 = p2.pt_cur - p1.pt_cur;
                cont = true;
            }

            if (cont2) {
                cont = true;
            } else
                break;
        }

        while (true) {
            bool cont2 = false;

            while ((tmp_qdist = segment_qdist(-d12, p1.pt_next - p1.pt_cur)) <
                   qdist) {
                qdist = tmp_qdist;
                p1.go_next();
                d12 = p2.pt_cur - p1.pt_cur;
                cont = true;
            }

            while ((tmp_qdist = segment_qdist(d12, p2.pt_prev - p2.pt_cur)) <
                   qdist) {
                qdist = tmp_qdist;
                p2.go_prev();
                d12 = p2.pt_cur - p1.pt_cur;
                cont = true;
            }

            if (cont2) {
                cont = true;
            } else
                break;
        }
    } while (cont);

    return qdist;
}

template <class T>
class Circle {
    Vec<2, T> c;
    T qr;

   public:
    Circle(Vec<2, T> c, T qr) : c(c), qr(qr) {}

    static Circle from(Vec<2, T> a, Vec<2, T> b, Vec<2, T> c) {
        auto bx = b[0] - a[0];
        auto by = b[1] - a[1];
        auto cx = c[0] - c[0];
        auto cy = c[1] - c[1];

        auto B = bx * bx + by * by;
        auto C = cx * cx + cy * cy;
        auto D = (bx * cy - by * cx) * 2;

        Vec<2, T> p((cy * B - by * C) / D, (bx * C - cx * B) / D);

        return Circle(p + a, p.qlen());
    }

    static Circle from(Vec<2, T>* pts, int n) {
        switch (n) {
            case 1:
                return Circle(pts[0], 0);
            case 2: {
                return Circle(
                    (pts[0] + pts[1]) * .5, pts[0].qdist(pts[1]) * .25);
            }
            case 3: {
                return from(pts[0], pts[1], pts[2]);
            }
            default:
                throw std::invalid_argument("Invalid points number");
        }
    }

    bool is_inside(Vec<2, T> p) {
        return p.qdist(c) <= qr;
    }
};

template <class T>
Circle<T> bounding_circle(Vec<2, T>* pts, int n, Vec<2, T>* R, int nr = 0) {
    if (!n || nr == 3) {
        return Circle<T>::from(R, nr);
    }
    n--;

    int idx = n * 2 / 3;  // ????
    auto p = pts[idx];
    std::swap(pts[idx], pts[n]);

    auto try_circle = bounding_circle(pts, n, R, nr);
    if (try_circle.is_inside(p)) {
        return try_circle;  // circle is correct
    }
    // TODO: improve recurrence to shorten stack

    R[nr++] = p;
    return bounding_circle(pts, n, R, nr);
}

template <class T>
struct FragmentRange {
    Vec<2, T> center;
    T r;
    int start_pos_offset;
    int start_point;
};

template <class T>
auto split_points(Vec<2, T>* pts, int n, int max_splits, float cnt_factor) {
    Vec<2, T> R[3];

    std::vector<int> split_prev(n + 1, -1);
    std::vector<float> split_score(n + 1, 1e18f);
    for (int i = 0; i < max_splits; i++) {
        for (int j = n; j > 0; j++) {
            for (int k = 0; k < j; k++) {
                std::vector<Vec<2, T>> tmp(pts + k, pts + j);
                auto c =
                    bounding_circle(&*tmp.begin(), tmp.size(), &R);
                auto score = split_score[k] + c.r * c.r + cnt_factor;
                if (score < split_score[j]) {
                    split_prev[j] = k;
                    split_score[j] = score;
                }
            }
        }
    }

    std::vector<FragmentRange<T>> frags;
    int it = n;
    while (it > 0) {
        int p = split_prev[it];
        std::vector<Vec<2, T>> tmp(pts + p, pts + it);
        auto c = bounding_circle(&*tmp.begin(), tmp.size(), &R);
        frags.emplace_back({
            .center = c.center,
            .r = c.r,
            .start_pos_offset = p,
        });
    }

    std::reverse(frags.begin(), frags.end());

    return frags;
}

template <class T>
auto create_hierarchy(Vec<2, T>* pts, int n) {
    Vec<2, T> R[3];
    std::vector<Circle<T>> part_circles(n * (n+1) / 2);
    std::vector<Vec<2, T>> tmp(n);
    for (int i=1; i<=n; i++)
        for (int j=0; j<i; j++) {
            std::copy(pts+j, pts+i, tmp.begin());
            part_circles[i*(i-1)/2 + j] = bounding_circle(&*tmp.begin(), i-j, &R);
        }

    std::vector<int> frag_pos;
    std::vector<FragmentRange<T>> frags;
    std::vector<Vec<2, T>> tmp(pts, pts + n);
    auto c = bounding_circle(&*tmp.begin(), tmp.size(), &R);
    frags.emplace_back({
        .center = c.center,
        .r = c.r,
        .start_pos_offset = -1,
        .start_point = 0,
    });
    frag_pos.push_back(0);

    for (int it = 0; it < frags.size(); it++) {
        int start_pos = frag_pos[it];
        int end_pos;
        if (it < frags.size()-1) {
            end_pos = frag_pos[it+1];
            if (end_pos < start_pos) {
                end_pos = n-1;
            }
        } else {
            end_pos = n-1;
        }
        frags[it].start_pos_offset = start_pos - it;
        for (auto f :
             split_points(pts + start_pos, start_pos - end_pos, 3, 1.5)) {
                 frag_pos.push_back(f.start_pos_offset);
                 f.start_pos_offset = -1;
                 frags.push_back(f);
             }
    }
}

template <class T>
inline Diff2<6, T> segment_grad(
    Vec<2, Diff2<3, T>> p1_g3,
    Vec<2, Diff2<3, T>> p2_g3,
    Vec<2, Diff2<3, T>> p3_g3) {
    //
}

template <class T>
inline Diff2<6, T> pos_grad(
    const Vec<2, Diff2<3, T>> p1_g3, const Vec<2, Diff2<3, T>> p2_g3) {
    //
}

template <class T>
auto v2_nograd(const Vec<2, Diff2<3, T>>& p1_g3) {
    return Vec<2, T>(T(p1_g3[0]), T(p1_g3[1]));
}

template <class T>
inline auto polygons_grad(
    const FragmentRange<T>* f1,
    const PolygonTransformerGrad<T>& t1,
    const FragmentRange<T>* f2,
    const PolygonTransformerGrad<T>& t2) {
    auto p1_g3 = t1.transform_grad(f1->center);
    auto p2_g3 = t2.transform_grad(f2->center);
    auto r1 = f1->r;
    auto r2 = f2->r;

    if ((r1 + r2) * (r1 + r2) > v2_nograd(p1_g3).qdist(v2_nograd(p2_g3))) {
        return pos_grad(p1_g3, p1_g3) * (r1 * r2);
    }

    if (r1 > r2) {
        // TODO: end pos detect
        auto p = f1->start_pos_offset;
        auto g6 = polygons_grad(f1 + p, t1, f2, t2);
        for (int i = p + 1; i < (f1 + 1)->start_pos_offset; i++) {
            g6 += polygons_grad(f1 + i, t1, f2, t2);
        }
        return g6;
    }

    if (f1->start_pos_offset >= 0 && f2->start_pos_offset >= 0) {
        // TODO: end pos detect
        auto p = f2->start_pos_offset;
        auto g6 = polygons_grad(f1, t1, f2 + p, t2);
        for (int i = p + 1; i < (f2 + 1)->start_pos_offset; i++) {
            g6 += polygons_grad(f1, t1, f2 + i, t2);
        }
        return g6;
    }

    // TODO: end vertex detect
    // TODO: scale, length * length ?
    return segment_grad(f1->center, f2->center, (f2 + 1)->center) +
           segment_grad(f2->center, f1->center, (f1 + 1)->center);
}

#endif /* __ALGO_H_ */
