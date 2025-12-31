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

    static Circle from(Vec<2, T> a, Vec<2, T> b) {
        return Circle((a + b) * .5, (a - b).qlen() * .5);
    }

    static Circle from(Vec<2, T>* pts, int n) {
        switch (n) {
            case 1:
                return Circle(pts[0], 0);
            case 2: {
                return from(pts[0], pts[1])
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

    auto center() const {
        return c;
    }

    auto square_radius() const {
        return qr;
    }
};

template <class T>
class FragmentBoundingCircle {
    Vec<2, T> c;
    T r;
    int start_pos, mid_angle_pos;

    public:
    FragmentBoundingCircle(Vec<2, T> c, T r, int start_pos, int mid_angle_pos)
        : c(c), r(r), start_pos(start_pos), mid_angle_pos(mid_angle_pos) {}
};

// TODO: deeper convex parts hierarchy
template<class T>
class Polygon {
    public:

    std::vector<Vec<2, T>> points;
    std::vector<FragmentBoundingCircle<T>> circles;

    Polygon(
        const std::vector<Vec<2, T>> &points,
        const std::vector<FragmentBoundingCircle<T>> &circles)
        : points(points), circles(circles) {}
};
