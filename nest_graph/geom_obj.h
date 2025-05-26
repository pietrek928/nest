#include <stdexcept>
#include <vector>

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

template<class T>
class Polygon {
    std::vector<Vec<2, T>> points;
    std::vector<int> convex_ends;
    std::vector<Circle<T>> circles;

    public:
    Polygon(
        const std::vector<Vec<2, T>> &points,
        const std::vector<int> &convex_ends,
        const std::vector<Circle<T>> &circles)
        : points(points), convex_ends(convex_ends), circles(circles) {}
};
