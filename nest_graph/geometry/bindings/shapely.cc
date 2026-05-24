#include "shapely.h"

#include <Python.h>

#include <cmath>
#include <string>

PyObject *steal_attr(PyObject *obj, const char *name) {
    PyObject *value = PyObject_GetAttrString(obj, name);
    if (!value) {
        throw nb::python_error();
    }
    return value;
}

bool nb_is_empty(nb::handle geom) {
    PyObject *value = steal_attr(geom.ptr(), "is_empty");
    const bool empty = PyObject_IsTrue(value) != 0;
    Py_DECREF(value);
    return empty;
}

bool geom_type_is(nb::handle geom, const char *name) {
    PyObject *value = steal_attr(geom.ptr(), "geom_type");
    const bool match =
        PyUnicode_Check(value)
        && PyUnicode_CompareWithASCIIString(value, name) == 0;
    Py_DECREF(value);
    return match;
}

void append_coords(nb::handle coords, std::vector<Vec2d> &out) {
    for (nb::handle pt : nb::iter(coords)) {
        double x = 0.0;
        double y = 0.0;
        if (read_xy(pt, x, y)) {
            out.push_back(Vec2d({x, y}));
        }
    }
}

void collect_polygon_rings(
    nb::handle poly,
    std::vector<std::vector<Vec2d>> &outers,
    std::vector<std::vector<Vec2d>> &holes
) {
    PyObject *exterior = steal_attr(poly.ptr(), "exterior");
    PyObject *exterior_coords = steal_attr(exterior, "coords");
    Py_DECREF(exterior);

    std::vector<Vec2d> outer = ring_from_coords(exterior_coords);
    Py_DECREF(exterior_coords);
    if (outer.size() >= 3) {
        outers.push_back(std::move(outer));
    }

    PyObject *interiors = steal_attr(poly.ptr(), "interiors");
    for (nb::handle interior : nb::iter(interiors)) {
        PyObject *coords = steal_attr(interior.ptr(), "coords");
        std::vector<Vec2d> hole = ring_from_coords(coords);
        Py_DECREF(coords);
        if (hole.size() >= 3) {
            holes.push_back(std::move(hole));
        }
    }
    Py_DECREF(interiors);
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

void collect_from_shapely(
    nb::handle geom,
    std::vector<std::vector<Vec2d>> &outers,
    std::vector<std::vector<Vec2d>> &holes
) {
    if (nb_is_empty(geom)) {
        return;
    }

    if (geom_type_is(geom, "Polygon")) {
        collect_polygon_rings(geom, outers, holes);
        return;
    }
    if (geom_type_is(geom, "MultiPolygon")) {
        PyObject *geoms = steal_attr(geom.ptr(), "geoms");
        for (nb::handle part : nb::iter(geoms)) {
            collect_polygon_rings(part, outers, holes);
        }
        Py_DECREF(geoms);
        return;
    }
    if (geom_type_is(geom, "GeometryCollection")) {
        PyObject *geoms = steal_attr(geom.ptr(), "geoms");
        for (nb::handle part : nb::iter(geoms)) {
            collect_from_shapely(part, outers, holes);
        }
        Py_DECREF(geoms);
        return;
    }
}
