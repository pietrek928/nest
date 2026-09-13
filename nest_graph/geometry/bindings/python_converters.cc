#include "python_converters.h"

#include <Python.h>
#include <cstring>

#include <nanobind/stl/tuple.h>

static int g_coord_copy_buffer_n = 0;
static int g_coord_copy_iter_n = 0;

int coord_copy_buffer_n() { return g_coord_copy_buffer_n; }
int coord_copy_iter_n() { return g_coord_copy_iter_n; }
void reset_coord_copy_telem() {
    g_coord_copy_buffer_n = 0;
    g_coord_copy_iter_n = 0;
}

bool try_append_from_xy_attr(nb::handle coords, std::vector<Vec2d> &out) {
    if (!nb::hasattr(coords, "xy")) {
        return false;
    }
    nb::object xy_obj = coords.attr("xy");
    if (!nb::isinstance<nb::tuple>(xy_obj) && !nb::isinstance<nb::sequence>(xy_obj)) {
        return false;
    }
    if (nb::len(xy_obj) < 2) {
        return false;
    }
    nb::handle xs = xy_obj[0];
    nb::handle ys = xy_obj[1];
    const Py_ssize_t nx = PySequence_Size(xs.ptr());
    const Py_ssize_t ny = PySequence_Size(ys.ptr());
    if (nx < 0 || ny < 0 || nx != ny) {
        PyErr_Clear();
        return false;
    }
    out.reserve(out.size() + static_cast<std::size_t>(nx));
    for (Py_ssize_t i = 0; i < nx; ++i) {
        nb::object xi = nb::steal(PySequence_GetItem(xs.ptr(), i));
        nb::object yi = nb::steal(PySequence_GetItem(ys.ptr(), i));
        if (!xi || !yi) {
            PyErr_Clear();
            out.clear();
            return false;
        }
        const double x = nb::cast<double>(xi);
        const double y = nb::cast<double>(yi);
        out.push_back(Vec2d({x, y}));
    }
    ++g_coord_copy_buffer_n;
    return true;
}

bool try_append_from_buffer(nb::handle coords, std::vector<Vec2d> &out) {
    Py_buffer view;
    std::memset(&view, 0, sizeof(view));
    if (PyObject_GetBuffer(coords.ptr(), &view, PyBUF_ND | PyBUF_FORMAT | PyBUF_STRIDES)
        != 0) {
        PyErr_Clear();
        return false;
    }
    bool ok = false;
    if (view.ndim == 2 && view.shape != nullptr && view.shape[1] == 2
        && view.format != nullptr
        && (std::strcmp(view.format, "d") == 0 || std::strcmp(view.format, "f") == 0)) {
        const Py_ssize_t n = view.shape[0];
        const Py_ssize_t s0 = view.strides[0];
        const Py_ssize_t s1 = view.strides[1];
        const char *base = static_cast<const char *>(view.buf);
        out.reserve(out.size() + static_cast<std::size_t>(n));
        if (std::strcmp(view.format, "d") == 0) {
            for (Py_ssize_t i = 0; i < n; ++i) {
                const double x = *reinterpret_cast<const double *>(base + i * s0);
                const double y = *reinterpret_cast<const double *>(base + i * s0 + s1);
                out.push_back(Vec2d({x, y}));
            }
        } else {
            for (Py_ssize_t i = 0; i < n; ++i) {
                const double x = static_cast<double>(
                    *reinterpret_cast<const float *>(base + i * s0));
                const double y = static_cast<double>(
                    *reinterpret_cast<const float *>(base + i * s0 + s1));
                out.push_back(Vec2d({x, y}));
            }
        }
        ok = true;
        ++g_coord_copy_buffer_n;
    }
    PyBuffer_Release(&view);
    return ok;
}

void append_coords(nb::handle coords, std::vector<Vec2d> &out) {
    if (try_append_from_xy_attr(coords, out)) {
        return;
    }
    if (try_append_from_buffer(coords, out)) {
        return;
    }
    const Py_ssize_t n = PySequence_Size(coords.ptr());
    if (n < 0) {
        PyErr_Clear();
    } else if (n > 0) {
        out.reserve(out.size() + static_cast<std::size_t>(n));
    }
    for (nb::handle pt : nb::iter(coords)) {
        double x = 0.0;
        double y = 0.0;
        if (read_xy(pt, x, y)) {
            out.push_back(Vec2d({x, y}));
        }
    }
    ++g_coord_copy_iter_n;
}

bool read_xy(nb::handle pt, double &x, double &y) {
    if (nb::isinstance<nb::tuple>(pt) || nb::isinstance<nb::sequence>(pt)) {
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

std::vector<double> sample_fracs_from_python(nb::object sample_fracs) {
    std::vector<double> fracs;
    if (!sample_fracs.is_none()) {
        for (nb::handle f : nb::iter(sample_fracs)) {
            fracs.push_back(static_cast<double>(nb::cast<double>(f)));
        }
    }
    if (fracs.empty()) {
        fracs = {0.1, 0.5};
    }
    return fracs;
}
