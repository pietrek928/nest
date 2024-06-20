import numpy as np
cimport numpy as cnp

cimport bindings
from bindings cimport Vec2f, v2f, Vec2d, v2d, Vec2f_g3, Vec2d_g3, Vec2_g3_size, g3_size, f_g6, d_g6, g6_size
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
    bindings.transform_points_2f(out_data, pts_data, items_count, v2f(dx, dy), a)
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
    bindings.transform_points_2d(out_data, pts_data, items_count, v2d(dx, dy), a)
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
    bindings.transform_points_g3_2f(out_data, pts_data, items_count, v2f(dx, dy), a)
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
    bindings.transform_points_g3_2d(out_data, pts_data, items_count, v2d(dx, dy), a)
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


cdef cnp.ndarray[cnp.float32_t, ndim=1] c_points_line_string_distance_2f_g3(
    cnp.ndarray[cnp.float32_t, ndim=3] points, cnp.ndarray[cnp.float32_t, ndim=3] line_string
):
    cdef int points_count = points.shape[0]
    cdef int line_string_count = line_string.shape[0]

    cdef const Vec2f_g3 *points_data = <Vec2f_g3 *> points.data
    cdef const Vec2f_g3 *line_string_data = <Vec2f_g3 *> line_string.data
    cdef f_g6 result = bindings.points_line_string_distance_2f_g3(points_data, points_count, line_string_data, line_string_count)
    return cnp.PyArray_SimpleNewFromData(1, [g6_size()], np.float32, <void*>&result)


cdef cnp.ndarray[cnp.float64_t, ndim=1] c_points_line_string_distance_2d_g3(
    cnp.ndarray[cnp.float64_t, ndim=3] points, cnp.ndarray[cnp.float64_t, ndim=3] line_string
):
    cdef int points_count = points.shape[0]
    cdef int line_string_count = line_string.shape[0]

    cdef const Vec2d_g3 *points_data = <Vec2d_g3 *> points.data
    cdef const Vec2d_g3 *line_string_data = <Vec2d_g3 *> line_string.data
    cdef d_g6 result = bindings.points_line_string_distance_2d_g3(points_data, points_count, line_string_data, line_string_count)
    return cnp.PyArray_SimpleNewFromData(1, [g6_size()], np.float64, <void*>&result)


def points_line_string_distance_g3(cnp.ndarray points, cnp.ndarray line_string):
    cdef int elem_size = points.shape[points.ndim-1]
    cdef int vec_size = points.shape[points.ndim-2]
    cdef dtype = points.dtype

    if vec_size == 2 and elem_size == g3_size():
        if dtype == np.float32:
            return c_points_line_string_distance_2f_g3(points, line_string)
        elif dtype == np.float64:
            return c_points_line_string_distance_2d_g3(points, line_string)

    raise ValueError(f"Unsupported dtype={dtype} and vec_size={vec_size}, elem_size={elem_size}")
