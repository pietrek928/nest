#include "python_converters.h"

#include <nanobind/stl/tuple.h>

void append_coords(nb::handle coords, std::vector<Vec2d> &out) {
    for (nb::handle pt : nb::iter(coords)) {
        double x = 0.0;
        double y = 0.0;
        if (read_xy(pt, x, y)) {
            out.push_back(Vec2d({x, y}));
        }
    }
}

bool read_xy(nb::handle pt, double &x, double &y) {
    if (nb::isinstance<nb::tuple>(pt)) {
        if (nb::len(pt) < 2) {
            return false;
        }
        x = nb::cast<double>(pt[0]);
        y = nb::cast<double>(pt[1]);
        return true;
    }
    if (nb::isinstance<nb::sequence>(pt)) {
        if (nb::len(pt) < 2) {
            return false;
        }
        x = nb::cast<double>(pt[0]);
        y = nb::cast<double>(pt[1]);
        return true;
    }
    return false;
}

bool read_transform(nb::handle t, double &x, double &y, double &angle) {
    if (nb::isinstance<nb::tuple>(t) || nb::isinstance<nb::sequence>(t)) {
        if (nb::len(t) < 3) {
            return false;
        }
        x = nb::cast<double>(t[0]);
        y = nb::cast<double>(t[1]);
        angle = nb::cast<double>(t[2]);
        return true;
    }
    return false;
}

nb::tuple vec2d_to_tuple(const Vec2d &v) {
    return nb::make_tuple(v[0], v[1]);
}

Vec2d vec2d_from_tuple(nb::handle o) {
    double x = 0.0;
    double y = 0.0;
    if (!read_xy(o, x, y)) {
        throw nb::type_error("expected a length-2 tuple or sequence for Vec2");
    }
    return Vec2d({x, y});
}

void points_from_iterable(nb::handle points, std::vector<Vec2d> &out) {
    append_coords(points, out);
}

std::vector<Vec2d> ring_from_coords(nb::handle coords_iterable) {
    std::vector<Vec2d> out;
    points_from_iterable(coords_iterable, out);
    if (out.size() >= 2) {
        const Vec2d &a = out.front();
        const Vec2d &b = out.back();
        if (a[0] == b[0] && a[1] == b[1]) {
            out.pop_back();
        }
    }
    return out;
}
