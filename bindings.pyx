import numpy as np
cimport numpy as cnp

cimport bindings
from bindings cimport Vec2f, Vec2d

cnp.import_array()

cdef cnp.ndarray[cnp.float64_t, ndim=2] c_transform_points_2f(
    cnp.ndarray[cnp.float64_t, ndim=2] pts, float dx, float dy, float a
):
    cdef int items_count = 1
    for i in range(pts.ndim-1):
        items_count *= pts.shape[i]

    cdef const Vec2f *pts_data = <Vec2f *> pts.data
    cdef cnp.ndarray[cnp.float64_t, ndim=2] out_array = cnp.PyArray_EMPTY(pts.ndim, pts.shape, pts.descr.type_num, 0)
    cdef Vec2f *out_data = <Vec2f *> out_array.data
    bindings.transform_points_2f(out_data, pts_data, items_count, Vec2f(dx, dy), a)
    return out_array


cdef cnp.ndarray[cnp.float64_t, ndim=2] c_transform_points_2d(
    cnp.ndarray[cnp.float64_t, ndim=2] pts, float dx, float dy, float a
):
    cdef int items_count = 1
    for i in range(pts.ndim-1):
        items_count *= pts.shape[i]

    cdef const Vec2d *pts_data = <Vec2d *> pts.data
    cdef cnp.ndarray[cnp.float64_t, ndim=2] out_array = cnp.PyArray_EMPTY(pts.ndim, pts.shape, pts.descr.type_num, 0)
    cdef Vec2d *out_data = <Vec2d *> out_array.data
    bindings.transform_points_2d(out_data, pts_data, items_count, Vec2d(dx, dy), a)
    return out_array


def transform_points(cnp.ndarray pts, dx, dy, a):
    cdef int vec_size = pts.shape[pts.ndim-1]
    cdef dtype = pts.dtype

    if vec_size == 2:
        if dtype == np.float32:
            return c_transform_points_2f(pts, dx, dy, a)
        elif dtype == np.float64:
            return c_transform_points_2d(pts, dx, dy, a)

    raise ValueError(f"Unsupported dtype={dtype} and vec_size={vec_size}")
