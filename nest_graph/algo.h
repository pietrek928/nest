#include <algorithm>
#include <cstring>
#include <vector>
#include <stdexcept>

#include "vec.h"
#include "geom_obj.h"


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

template<class T>
inline T &circle_pos(T *A, int n, int npoints, int pos) {
    return A[n * (npoints-2) + pos];
}

template <class T>
std::vector<Circle<T>> convex_bounding_circles(Vec<2, T>* pts, int n) {
    std::vector<Vec<2, T>> pts_round(pts, pts + n);
    std::vector<Vec<2, T>> pts_tmp(n);
    Vec<2, T> R[3];
    pts_round.emplace(pts, pts + n);
    std::vector<Circle<T>> result(n * (n-2), Circle<T>(Vec<2, T>(0, 0), 1e18));

    for (int i=0; i<n; i++) {
        circle_pos(pts_tmp.data(), n, 2, i) = Circle<T>::from(pts_round[i], pts_round[i+1]);

        int last_direction = -1;
        for (int j=3; j<=n; j++) {
            int direction = turns_right(pts[i+j-3]-pts[i+j-1], pts[i+j-2]-pts[i+j-1]);
            if (direction != last_direction) {
                if (last_direction == -1) {
                    last_direction = direction;
                } else {
                    break;
                }
            }
            std::memcpy(pts_tmp.data(), pts_round.data() + i, j * sizeof(T));
            circle_pos(pts_tmp.data(), n, j, i) = bounding_circle(pts_tmp.data(), j, R);
        }
    }

    return result;
}

template <class T, typename Tscore_func>
std::pair<int, std::vector<int>> find_best_split(
    Circle<T> *pts_circles, int n, float pow, Tscore_func && circle_weight
) {
    std::vector<int> best_split_pos(n * (n-2));
    std::vector<T> best_split_score(n * (n-2));
    for (int i=2; i<=n; i++) {
        for (int p1=0; p1<n; p1++) {
            T score = circle_weight(circle_pos(pts_circles, n, i, p1).qr, i);
            int score_split = i;
            for (int l1=2; l1<n; l1++) {
                int p2 = (p1 + l1 - 1) % n;
                int l2 = n - l1 + 1;
                T s1 = best_split_score(circle_pos(pts_circles, n, l1, p1).qr);
                T s2 = circle_weight(circle_pos(pts_circles, n, l2, p2).qr, l2);
                if (s1 + s2 < score) {
                    score = s1 + s2;
                    score_split = l2;
                }
            }
            circle_pos(best_split_pos.data(), n, i, p1) = score_split;
            circle_pos(best_split_score.data(), n, i, p1) = score;
        }
    }

    int start_pos = 0;
    T best_score = circle_pos(best_split_score.data(), n, n, 0);
    for (int i=1; i<n; i++) {
        if (circle_pos(best_split_score.data(), n, n, i) < best_score) {
            best_score = circle_pos(best_split_score.data(), n, n, i);
            start_pos = i;
        }
    }

    std::vector<int> splits;
    int l = n;
    while (l >= 2) {
        int s = circle_pos(best_split_pos.data(), n, l, start_pos);
        splits.push_back(s);
        l -= s-1;
    }
    std::reverse(splits.begin(), splits.end());
    return std::make_pair(start_pos, splits);
}

template<class T>
Polygon<T> &&polygon_from_split(Vec<2, T> *pts, int n, int start_pos, int *convex_lengths, int nconvex) {
    std::vector<Vec<2, T>> points(pts + start_pos, pts + n);
    points.insert(points.end(), pts, pts + start_pos);
    points.push_back(points[0]);

    std::vector<int> convex_ends(nconvex+1);
    int it = 0;
    convex_ends.push_back(it);
    for (int i=0; i<nconvex; i++) {
        it += convex_lengths[i]-1;
        convex_ends.push_back(it);
    }

    std::vector<Circle<T>> circles(convex_ends.size()-1);
    for (int i=0; i<convex_ends.size()-1; i++) {
        circles[i] = bounding_circle(
            points.data() + convex_ends[i], convex_ends[i+1] - convex_ends[i] + 1
        );
    }

    return Polygon<T>(points, convex_ends, circles);
}

template<class T>
int find_mid_angle_point(Vec<2, T> *pts, int n) {
    if (n <= 3) {
        return n >> 1;
    }
    auto vm = pts[1] - pts[0] - pts[n-2] + pts[n-1];

    int last_direction = -1;
    for (int it=0; it<n-1; it++) {
        int direction = turns_right(pts[it+1]-pts[it], vm);
        if (direction != last_direction) {
            if (last_direction == -1) {
                last_direction = direction;
            } else {
                return it;
            }
        }
    }
    return n-1;
}
