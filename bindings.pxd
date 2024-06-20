cdef extern from "bindings.h":
    cdef cppclass Vec2f:
        pass
    Vec2f v2f(float, float)
    cdef cppclass Vec2d:
        pass
    Vec2d v2d(double, double)
    cdef cppclass Vec3f:
        pass
    Vec3f v3f(float, float, float)
    cdef cppclass Vec3d:
        pass
    Vec3d v3d(double, double, double)

    cdef cppclass Vec2f_g3:
        pass
    cdef cppclass Vec2d_g3:
        pass
    unsigned int Vec2_g3_size()

    unsigned int g3_size()
    cdef cppclass f_g6:
        pass
    cdef cppclass d_g6:
        pass
    unsigned int g6_size()

    cdef void transform_points_2f(
        Vec2f *out, const Vec2f *points, int n,
        const Vec2f pos, float a
    )
    cdef void transform_points_2d(
        Vec2d *out, const Vec2d *points, int n,
        const Vec2d pos, double a
    )

    cdef void transform_points_g3_2f(
        Vec2f_g3 *out, const Vec2f *points, int n,
        const Vec2f pos, float a
    )
    cdef void transform_points_g3_2d(
        Vec2d_g3 *out, const Vec2d *points, int n,
        const Vec2d pos, double a
    )

    cdef f_g6 points_line_string_distance_2f_g3(
        const Vec2f_g3 *points, int npoints,
        const Vec2f_g3 *line_string, int nline
    )
    cdef d_g6 points_line_string_distance_2d_g3(
        const Vec2d_g3 *points, int npoints,
        const Vec2d_g3 *line_string, int nline
    )
    cdef f_g6 points_line_ring_distance_2f_g3(
        const Vec2f_g3 *points, int npoints,
        const Vec2f_g3 *line_ring, int nline
    )
    cdef d_g6 points_line_ring_distance_2d_g3(
        const Vec2d_g3 *points, int npoints,
        const Vec2d_g3 *line_ring, int nline
    )
