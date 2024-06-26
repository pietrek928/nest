#ifndef __ALGO_H_
#define __ALGO_H_

#include <algorithm>
#include <vector>
#include <stdexcept>

// for debug
#include <iostream>
using namespace std;

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


template<class T>
void transform_points(
    Vec<2, T> *out, const Vec<2, T> *points, int n,
    const Vec<2, T> pos, T a
) {
    T ca = std::cos(a);
    T sa = std::sin(a);
    for (int i=0; i<n; i++) {
        const auto &v = points[i];
        out[i].get_(0) = v[0] * ca - v[1] * sa + pos[0];
        out[i].get_(1) = v[0] * sa + v[1] * ca + pos[1];
    }
}

template<class T>
void transform_points_g3(
    Vec<2, Diff2<3, T>> *out, const Vec<2, T> *points, int n,
    Vec<2, T> pos, T a
) {
    auto cs = Diff2<3, T>::from_var(a, 2).cossin();
    for (int i=0; i<n; i++) {
        const auto &v = points[i];
        out[i].get_(0) = v[0] * cs.cos - v[1] * cs.sin + Diff2<3, T>::from_var(pos[0], 0);
        out[i].get_(1) = v[0] * cs.sin + v[1] * cs.cos + Diff2<3, T>::from_var(pos[1], 1);
    }
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
inline T segment_qdist(const Vec<2, T> vp1, const Vec<2, T> v21) {
    auto v21_qlen = v21.qlen();

    auto m = v21.dp(vp1);
    if (m >= 0) {
        return vp1.qlen();
    }

    auto vp1_qlen = vp1.qlen();
    auto vT_qlen = m * m / vp1_qlen;
    if (vT_qlen > v21_qlen) {
        return (vp1 - v21).qlen();
    }

    auto qlen = vp1_qlen - vT_qlen;
    cout << "aaaaaaaaa " << qlen << " " << v21 << " " << vp1 << endl;

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

#if 0

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

#endif

template <class T>
inline bool convex_polygons_intersect(
    const Vec<2, T> *poly1, int n1,
    const Vec<2, T> *poly2, int n2,
    bool direction = false,
    int it1 = 0, int it2 = 0
) {
    int op_limit = n1 + n2 + 16;
    auto p1 = poly1[it1];
    auto p2 = poly2[it2];
    auto d = p1 - p2;
    do {
        auto op_limit_old = op_limit;
        {
            auto it1_next = it1 < n1 - 1 ? it1 + 1 : 0;
            auto p1_next = poly1[it1_next];
            auto d_next = p1_next - p2;
            while (turns_direction(d, d_next, direction)) {
                it1 = it1_next;
                d = d_next;
                it1_next = it1 < n1 - 1 ? it1 + 1 : 0;
                p1_next = poly1[it1_next];
                d_next = p1_next - p2;
                if (--op_limit <= 0) {
                    break;
                }
            }
        }
        {
            auto it2_next = it2 < n2 - 1 ? it2 + 1 : 0;
            auto p2_next = poly2[it2_next];
            while (turns_direction(d, p2_next - p2, !direction)) {
                it2 = it2_next;
                p2 = p2_next;
                d = p1 - p2;
                if (--op_limit <= 0) {
                    break;
                }
            }
        }
        {
            auto it1_prev = it1 ? it1 - 1 : n1 - 1;
            auto p1_prev = poly1[it1_prev];
            auto d_prev = p1_prev - p2;
            while (turns_direction(d, d_prev, direction)) {
                d = d_prev;
                it1 = it1_prev;
                p1 = p1_prev;
                d_prev = p1_prev - p2;
                if (--op_limit <= 0) {
                    break;
                }
            }
        }
        {
            auto it2_prev = it2 ? it2 - 1 : n2 - 1;
            auto p2_prev = poly2[it2_prev];
            while (turns_direction(d, p2_prev - p2, !direction)) {
                it2 = it2_prev;
                p2 = p2_prev;
                d = p1 - p2;
                if (--op_limit <= 0) {
                    break;
                }
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
inline T convex_line_rings_qdist(
    const Vec<2, T> *ring1, unsigned int n1,
    const Vec<2, T> *ring2, unsigned int n2,
    unsigned int it1 = 0, unsigned int it2 = 0
) {
    T qdist = 1e18, tmp_qdist;
    bool cont = false;

    auto p1 = ring1[it1];
    auto p2 = ring2[it2];

    do {
        cont = false;

        bool moved_next;
        do {
            bool moved_next = false;

            {
                auto it2_next = it2 < n2 - 1 ? it2 + 1 : 0;
                auto p2_next = ring2[it2_next];
                while ((tmp_qdist = segment_qdist(p2 - p1, p2_next - p2)) <
                    qdist) {
                    qdist = tmp_qdist;
                    it2 = it2_next;
                    p2 = p2_next;
                    it2_next = it2 < n2 - 1 ? it2 + 1 : 0;
                    p2_next = ring2[it2_next];
                    moved_next = true;
                }
            }

            {
                auto it1_next = it1 < n1 - 1 ? it1 + 1 : 0;
                auto p1_next = ring1[it1_next];
                while ((tmp_qdist = segment_qdist(p1 - p2, p1_next - p1)) <
                    qdist) {
                    qdist = tmp_qdist;
                    it1 = it1_next;
                    p1 = p1_next;
                    it1_next = it1 < n1 - 1 ? it1 + 1 : 0;
                    p1_next = ring1[it1_next];
                    moved_next = true;
                }
            }

            if (moved_next) {
                cont = true;
            }
        } while (moved_next);

        bool moved_prev;
        do {
            bool moved_prev = false;

            {
                auto it2_prev = it2 ? it2 - 1 : n2 - 1;
                auto p2_prev = ring2[it2_prev];
                while ((tmp_qdist = segment_qdist(p2 - p1, p2_prev - p2)) <
                    qdist) {
                    qdist = tmp_qdist;
                    it2 = it2_prev;
                    p2 = p2_prev;
                    it2_prev = it2 ? it2 - 1 : n2 - 1;
                    p2_prev = ring2[it2_prev];
                    moved_prev = true;
                }
            }

            {
                auto it1_prev = it1 ? it1 - 1 : n1 - 1;
                auto p1_prev = ring1[it1_prev];
                while ((tmp_qdist = segment_qdist(p1 - p2, p1_prev - p1)) <
                    qdist) {
                    qdist = tmp_qdist;
                    it1 = it1_prev;
                    p1 = p1_prev;
                    it1_prev = it1 ? it1 - 1 : n1 - 1;
                    p1_prev = ring1[it1_prev];
                    moved_prev = true;
                }
            }

            if (moved_prev) {
                cont = true;
            }
        } while (moved_prev);
    } while (cont);

    return qdist;
}

template <class T>
inline Diff2<6, T> segment_qdist_grad_g6(
    Vec<2, Diff2<3, T>> p,
    Vec<2, Diff2<3, T>> s1,
    Vec<2, Diff2<3, T>> s2) {

    Vec<2, Diff2<6, T>> p_g6, s1_g6, s2_g6;
    p_g6.get_(0).add_mapped(p[0], {0, 1, 2});
    p_g6.get_(1).add_mapped(p[1], {0, 1, 2});
    s1_g6.get_(0).add_mapped(s1[0], {3, 4, 5});
    s1_g6.get_(1).add_mapped(s1[1], {3, 4, 5});
    s2_g6.get_(0).add_mapped(s2[0], {3, 4, 5});
    s2_g6.get_(1).add_mapped(s2[1], {3, 4, 5});

    return segment_qdist(s1_g6 - p_g6, s1_g6 - s2_g6);
}

template <class T>
inline Diff2<6, T> pos_dist_grad_g6(
    const Vec<2, Diff2<3, T>> p1_g3, const Vec<2, Diff2<3, T>> p2_g3) {
    Vec<2, Diff2<6, T>> v_g6;
    v_g6.get_(0).add_mapped(p1_g3[0], {0, 1, 2});
    v_g6.get_(1).add_mapped(p1_g3[1], {0, 1, 2});
    v_g6.get_(0).add_mapped(-p2_g3[0], {3, 4, 5});
    v_g6.get_(1).add_mapped(-p2_g3[1], {3, 4, 5});
    return v_g6.qlen();
}

template <class T>
auto v2_nograd(const Vec<2, Diff2<3, T>>& p1_g3) {
    return v<T>(T(p1_g3[0]), T(p1_g3[1]));
}


template <class T, typename Fqdist_transform>
inline auto points_line_string_distance_g3(
    const Vec<2, Diff2<3, T>> *points, int npoints,
    const Vec<2, Diff2<3, T>> *line_string, int nline,
    const Fqdist_transform&& qdist_transform
) {
    Diff2<6, T> ret_dist;

    int line_pos = 0;
    for (int i = 0; i < npoints; i++) {
        auto pt_nograd = v2_nograd(points[i]);
        T qdist = pt_nograd.qdist(v2_nograd(line_string[line_pos]));

        if (line_pos) {
            T qdist_prev = pt_nograd.qdist(v2_nograd(line_string[line_pos - 1]));
            while (qdist_prev < qdist) {
                line_pos--;
                qdist = qdist_prev;
                if (line_pos) {
                    qdist_prev = pt_nograd.qdist(v2_nograd(line_string[line_pos - 1]));
                } else {
                    qdist_prev = 1e18;
                    break;
                }
            }
        }

        T qdist_next;
        if (line_pos < nline - 1) {
            qdist_next = pt_nograd.qdist(v2_nograd(line_string[line_pos + 1]));
            while (qdist_next < qdist) {
                line_pos++;
                qdist = qdist_next;
                if (line_pos < nline - 1) {
                    qdist_next = pt_nograd.qdist(v2_nograd(line_string[line_pos + 1]));
                } else {
                    qdist_next = 1e18;
                    break;
                }
            }
        } else {
            qdist_next = 1e18;
        }

        T qdist_prev;
        if (line_pos) {
            qdist_prev = pt_nograd.qdist(v2_nograd(line_string[line_pos - 1]));
        } else {
            qdist_prev = 1e18;
        }
        auto pt3 = line_string[qdist_prev < qdist_next ? line_pos-1 : line_pos+1];

        ret_dist += qdist_transform(
            segment_qdist_grad_g6(points[i], line_string[line_pos], pt3)
        );
    }

    return ret_dist;
}

template <class T, typename Fqdist_transform>
inline auto points_line_ring_distance_g3(
    const Vec<2, Diff2<3, T>> *points, int npoints,
    const Vec<2, Diff2<3, T>> *line_ring, int npoly,
    const Fqdist_transform&& qdist_transform
) {
    Diff2<6, T> ret_dist;

    int line_pos = 0;
    for (int i = 0; i < npoints; i++) {
        auto pt_nograd = v2_nograd(points[i]);
        T qdist = pt_nograd.qdist(v2_nograd(line_ring[line_pos]));

        {
            int prev_pos = line_pos ? line_pos - 1 : npoly - 1;
            T qdist_prev = pt_nograd.qdist(v2_nograd(line_ring[prev_pos]));
            while (qdist_prev < qdist) {
                line_pos = prev_pos;
                qdist = qdist_prev;
                prev_pos = line_pos ? line_pos - 1 : npoly - 1;
                qdist_prev = pt_nograd.qdist(v2_nograd(line_ring[prev_pos]));
            }
        }

        int next_pos = line_pos < npoly - 1 ? line_pos + 1 : 0;
        T qdist_next = pt_nograd.qdist(v2_nograd(line_ring[next_pos]));
        while (qdist_next < qdist) {
            line_pos = next_pos;
            qdist = qdist_next;
            next_pos = line_pos < npoly - 1 ? line_pos + 1 : 0;
            qdist_next = pt_nograd.qdist(v2_nograd(line_ring[next_pos]));
        }

        int prev_pos = line_pos ? line_pos - 1 : npoly - 1;
        T qdist_prev = pt_nograd.qdist(v2_nograd(line_ring[prev_pos]));
        auto pt3 = line_ring[qdist_prev < qdist_next ? prev_pos : next_pos];

        ret_dist += qdist_transform(
            segment_qdist_grad_g6(points[i], line_ring[line_pos], pt3)
        );
    }

    return ret_dist;
}

#endif /* __ALGO_H_ */
