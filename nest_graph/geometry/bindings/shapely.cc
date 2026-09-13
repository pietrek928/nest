#include "shapely.h"

#include <Python.h>

#include "python_converters.h"

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

std::string geom_type_string(nb::handle geom) {
    PyObject *value = steal_attr(geom.ptr(), "geom_type");
    std::string out;
    if (PyUnicode_Check(value)) {
        const char *s = PyUnicode_AsUTF8(value);
        if (s != nullptr) {
            out = s;
        }
    }
    Py_DECREF(value);
    return out;
}

bool geom_type_is(nb::handle geom, const char *name) {
    return geom_type_string(geom) == name;
}

void collect_polygon_rings(
    nb::handle poly,
    std::vector<std::vector<Vec2d>> &outers,
    std::vector<std::vector<Vec2d>> &holes,
    bool include_holes
) {
    PyObject *exterior = steal_attr(poly.ptr(), "exterior");
    PyObject *exterior_coords = steal_attr(exterior, "coords");
    Py_DECREF(exterior);

    std::vector<Vec2d> outer = ring_from_coords(exterior_coords);
    Py_DECREF(exterior_coords);
    // Closed polygon shells need ≥3 vertices after close-pop.
    if (outer.size() >= 3) {
        outers.push_back(std::move(outer));
    }

    if (!include_holes) {
        return;
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

void collect_from_shapely(
    nb::handle geom,
    std::vector<std::vector<Vec2d>> &outers,
    std::vector<std::vector<Vec2d>> &holes,
    bool include_holes
) {
    if (nb_is_empty(geom)) {
        return;
    }

    const std::string gtype = geom_type_string(geom);
    if (gtype == "Polygon") {
        collect_polygon_rings(geom, outers, holes, include_holes);
        return;
    }
    if (gtype == "MultiPolygon") {
        PyObject *geoms = steal_attr(geom.ptr(), "geoms");
        for (nb::handle part : nb::iter(geoms)) {
            collect_polygon_rings(part, outers, holes, include_holes);
        }
        Py_DECREF(geoms);
        return;
    }
    if (gtype == "GeometryCollection") {
        PyObject *geoms = steal_attr(geom.ptr(), "geoms");
        for (nb::handle part : nb::iter(geoms)) {
            collect_from_shapely(part, outers, holes, include_holes);
        }
        Py_DECREF(geoms);
        return;
    }
}
