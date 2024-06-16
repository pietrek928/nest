import numpy as np
cimport numpy as cnp

cimport bindings
from bindings cimport Vec2f, Vec2d, Vec2f_g3, Vec2d_g3, Vec2_g3_size
from libcpp.vector cimport vector

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


cdef cnp.ndarray[cnp.float32_t, ndim=3] c_transform_points_g3_2f(
    cnp.ndarray[cnp.float32_t, ndim=2] pts, float dx, float dy, float a
):
    cdef int items_count = 1
    cdef vector[cnp.npy_intp] out_dims
    for i in range(pts.ndim-1):
        items_count *= pts.shape[i]
        out_dims.push_back(pts.shape[i])
    out_dims.push_back(2)
    out_dims.push_back(Vec2_g3_size())

    cdef const Vec2f *pts_data = <Vec2f *> pts.data
    cdef cnp.ndarray[cnp.float32_t, ndim=3] out_array = cnp.PyArray_EMPTY(out_dims.size(), &out_dims[0], pts.descr.type_num, 0)
    cdef Vec2f_g3 *out_data = <Vec2f_g3 *> out_array.data
    bindings.transform_points_g3_2f(out_data, pts_data, items_count, Vec2f(dx, dy), a)
    return out_array


cdef cnp.ndarray[cnp.float64_t, ndim=3] c_transform_points_g3_2d(
    cnp.ndarray[cnp.float64_t, ndim=2] pts, float dx, float dy, float a
):
    cdef int items_count = 1
    cdef vector[cnp.npy_intp] out_dims
    for i in range(pts.ndim-1):
        items_count *= pts.shape[i]
        out_dims.push_back(pts.shape[i])
    out_dims.push_back(2)
    out_dims.push_back(Vec2_g3_size())

    cdef const Vec2d *pts_data = <Vec2d *> pts.data
    cdef cnp.ndarray[cnp.float64_t, ndim=3] out_array = cnp.PyArray_EMPTY(out_dims.size(), &out_dims[0], pts.descr.type_num, 0)
    cdef Vec2d_g3 *out_data = <Vec2d_g3 *> out_array.data
    bindings.transform_points_g3_2d(out_data, pts_data, items_count, Vec2d(dx, dy), a)
    return out_array


def transform_points_g3(cnp.ndarray pts, dx, dy, a):
    cdef int vec_size = pts.shape[pts.ndim-1]
    cdef dtype = pts.dtype

    if vec_size == 2:
        if dtype == np.float32:
            return c_transform_points_g3_2f(pts, dx, dy, a)
        elif dtype == np.float64:
            return c_transform_points_g3_2d(pts, dx, dy, a)

    raise ValueError(f"Unsupported dtype={dtype} and vec_size={vec_size}")
