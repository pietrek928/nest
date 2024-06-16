cdef extern from "bindings.h":
    cdef cppclass Vec2f:
        Vec2f(float, float)
    cdef cppclass Vec2d:
        Vec2d(double, double)
    cdef cppclass Vec3f:
        Vec3f(float, float, float)
    cdef cppclass Vec3d:
        Vec3d(double, double, double)

    cdef cppclass Vec2f_g3:
        pass
    cdef cppclass Vec2d_g3:
        pass
    unsigned int Vec2_g3_size()

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
