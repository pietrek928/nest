#include <algorithm>
#include <cstring>
#include <vector>
#include <stdexcept>

#include "vec.h"


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
std::vector<Circle<T>> all_bounding_circles(Vec<2, T>* pts, int n) {
    std::vector<Vec<2, T>> pts_round(pts, pts + n);
    std::vector<Vec<2, T>> pts_tmp(n);
    Vec<2, T> R[3];
    pts_round.emplace(pts, pts + n);
    std::vector<Circle<T>> result(n * n);

    for (int i=0; i<n; i++) {
        for (int j=1; j<=n; j++) {
            std::memcpy(pts_tmp.data(), pts_round.data() + i, j * sizeof(T));
            result[i * n + j - 1] = bounding_circle(pts_tmp.data(), j, R);
        }
    }

    return result;
}

template <class T, typename Tscore_func>
std::pair<int, std::vector<int>> find_best_split(
    Circle<T> *pts_circles, int n, float pow, Tscore_func && radius_func
) {
    std::vector<int> best_split_pos(n * n);
    std::vector<T> best_split_score(n * n);
    for (int i=1; i<=n; i++) {
        for (int p1=0; p1<n; p1++) {
            T score = radius_func(pts_circles[(i-1) * n + p1].qr);
            int score_split = i;
            for (int l1=1; l1<n; l1++) {
                int p2 = (p1 + l1) % n;
                int l2 = n - l1;
                T s1 = best_split_score(pts_circles[(l1-1) * n + p1].qr);
                T s2 = radius_func(pts_circles[(l2-1) * n + p2].qr);
                if (s1 + s2 < score) {
                    score = s1 + s2;
                    score_split = l2;
                }
            }
            best_split_pos[i * n + p1] = score_split;
            best_split_score[i * n + p1] = score;
        }
    }

    int start_pos = 0;
    T best_score = best_split_score[n * (n-1)];
    for (int i=1; i<n; i++) {
        if (best_split_score[n * (n-1) + i] < best_score) {
            best_score = best_split_score[n * (n-1) + i];
            start_pos = i;
        }
    }

    std::vector<int> splits;
    int l = n;
    while (l) {
        int s = best_split_pos[(l-1) * l + start_pos];
        splits.push_back(s);
        l -= s;
    }
    std::reverse(splits.begin(), splits.end());
    return std::make_pair(start_pos, splits);
}
