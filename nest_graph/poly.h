#include <algorithm>
#include <cmath>
#include <tuple>

#include "vec.h"
#include "geom_obj.h"


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
    // cout << "aaaaaaaaa " << qlen << " " << v21 << " " << vp1 << endl;

    return qlen;
}


template <class T>
inline auto convex_polygons_intersect(
    const Vec<2, T> *poly1, int n1,
    const Vec<2, T> *poly2, int n2,
    bool direction = false,
    int it1 = 0, int it2 = 0
) {
    typedef struct {
        bool intersect;
        int it1, it2;
    } ret_t;

    int op_limit = n1 + n2 + 16;
    int turn_limit = 16;  // ?????
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
            return ret_t{
                .intersect = false,
                .it1 = it1,
                .it2 = it2,
            };
        }
    } while (op_limit > 0 && --turn_limit);

    return ret_t{
        .intersect = true,
        .it1 = it1,
        .it2 = it2,
    };
}


template <class T>
inline T convex_line_rings_qdist(
    const Vec<2, T> *ring1, unsigned int n1,
    const Vec<2, T> *ring2, unsigned int n2,
    unsigned int it1 = 0, unsigned int it2 = 0
) {
    typedef struct {
        T qdist;
        unsigned int it1, it2;
    } ret_t;

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
                while ((tmp_qdist = segment_qdist(p2 - p1, p2_next - p2)) < qdist) {
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
                while ((tmp_qdist = segment_qdist(p1 - p2, p1_next - p1)) < qdist) {
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
                while ((tmp_qdist = segment_qdist(p2 - p1, p2_prev - p2)) < qdist) {
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
                while ((tmp_qdist = segment_qdist(p1 - p2, p1_prev - p1)) < qdist) {
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

    return ret_t{
        .qdist = qdist,
        .it1 = it1,
        .it2 = it2,
    };
}


template <class T>
inline T convex_line_strings_qdist(
    const Vec<2, T> *string1, unsigned int n1,
    const Vec<2, T> *string2, unsigned int n2,
    unsigned int it1 = 0, unsigned int it2 = 0
) {
    typedef struct {
        T qdist;
        unsigned int it1, it2;
    } ret_t;

    T qdist = 1e18, tmp_qdist;
    bool cont = false;

    auto p1 = string1[it1];
    auto p2 = string2[it2];

    do {
        cont = false;

        bool moved_next;
        do {
            bool moved_next = false;
            {
                auto it2_next = it2 + 1;
                while (it2_next < n2) {
                    auto p2_next = string2[it2_next];
                    auto tmp_qdist = segment_qdist(p2 - p1, p2_next - p2);
                    if (tmp_qdist >= qdist) {
                        break;
                    }
                    qdist = tmp_qdist;
                    it2 = it2_next;
                    p2 = p2_next;
                    it2_next = it2 + 1;
                    moved_next = true;
                }
            }

            {
                auto it1_next = it1 + 1;
                while (it1_next < n1) {
                    auto p1_next = string1[it1_next];
                    auto tmp_qdist = segment_qdist(p1 - p2, p1_next - p1);
                    if (tmp_qdist >= qdist) {
                        break;
                    }
                    qdist = tmp_qdist;
                    it1 = it1_next;
                    p1 = p1_next;
                    it1_next = it1 + 1;
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
                auto it2_prev = it2 - 1;
                while (it2_prev >= 0) {
                    auto p2_prev = string2[it2_prev];
                    auto tmp_qdist = segment_qdist(p2 - p1, p2_prev - p2);
                    if (tmp_qdist >= qdist) {
                        break;
                    }
                    qdist = tmp_qdist;
                    it2 = it2_prev;
                    p2 = p2_prev;
                    it2_prev = it2 - 1;
                    moved_prev = true;
                }
            }

            {
                auto it1_prev = it1 - 1;
                while (it1_prev >= 0) {
                    auto p1_prev = string1[it1_prev];
                    auto tmp_qdist = segment_qdist(p1 - p2, p1_prev - p1);
                    if (tmp_qdist >= qdist) {
                        break;
                    }
                    qdist = tmp_qdist;
                    it1 = it1_prev;
                    p1 = p1_prev;
                    it1_prev = it1 - 1;
                    moved_prev = true;
                }
            }

            if (moved_prev) {
                cont = true;
            }
        } while (moved_prev);
    } while (cont);

    return ret_t{
        .qdist = qdist,
        .it1 = it1,
        .it2 = it2,
    };
}


template <class T>
inline bool polygons_intersect(
    const Polygon<T> &p1, const Polygon<T> &p2
) {
    for (int i=0; i<p1.circles.size(); i++) {
        const auto &c1 = p1.circles[i];
        auto r1 = std::sqrt(c1.square_radius());
        for (int j=0; j<p2.circles.size(); j++) {
            const auto &c2 = p2.circles[j];
            auto dist = std::sqrt(c1.center.qdist(c2.center));
            auto r2 = std::sqrt(c2.square_radius());
            if (dist <= r1 + r2) {
                const auto *conv1_pts = &p1.points[p1.convex_ends[i]];
                auto conv1_n = p1.convex_ends[i+1] - p1.convex_ends[i];
                const auto *conv2_pts = &p2.points[p2.convex_ends[j]];
                auto conv2_n = p2.convex_ends[j+1] - p2.convex_ends[j];

                if (convex_polygons_intersect(
                    conv1_pts, conv1_n, conv2_pts, conv2_n
                )) {
                    return true;
                }
            }
        }
    }
    return false;
}

template <class T>
inline T polygons_distance(
    const Polygon<T> &p1, const Polygon<T> &p2
) {
    // TODO: pass buffer here
    std::vector<std::tuple<float, int, int>> buffer;
    for (int i=0; i<p1.circles.size(); i++) {
        const auto &c1 = p1.circles[i];
        auto r1 = std::sqrt(c1.square_radius());
        for (int j=0; j<p2.circles.size(); j++) {
            const auto &c2 = p2.circles[j];
            auto dist = std::sqrt(c1.center.qdist(c2.center));
            auto r2 = std::sqrt(c2.square_radius());
            buffer.emplace_back(dist - r1 - r2, i, j);
        }
    }
    std::sort(buffer.begin(), buffer.end());

    T dist = 1e18;
    for (const auto &t : buffer) {
        if (std::get<0>(t) >= dist) {
            break;
        }
        auto p1_pos = std::get<1>(t);
        auto p2_pos = std::get<2>(t);
        const auto *conv1_pts = &p1.points[p1.convex_ends[p1_pos]];
        auto conv1_n = p1.convex_ends[p1_pos+1] - p1.convex_ends[p1_pos];
        const auto *conv2_pts = &p2.points[p2.convex_ends[p2_pos]];
        auto conv2_n = p2.convex_ends[p2_pos+1] - p2.convex_ends[p2_pos];

        auto res = convex_line_rings_qdist(
            conv1_pts, conv1_n, conv2_pts, conv2_n
        );
        auto new_dist = std::sqrt(res.qdist);
        if (new_dist < dist) {
            dist = new_dist;
        }
    }
    return dist;
}
