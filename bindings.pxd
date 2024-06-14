cdef extern from "vec.h":
    cdef cppclass Vec2f:
        Vec2f(float, float)
    cdef cppclass Vec2d:
        Vec2d(double, double)
    cdef cppclass Vec3f:
        Vec3f(float, float, float)
    cdef cppclass Vec3d:
        Vec3d(double, double, double)

cdef extern from "algo.h":
    cdef void transform_points_2f(
        Vec2f *out, const Vec2f *points, int n,
        const Vec2f pos, float a
    )
    cdef void transform_points_2d(
        Vec2d *out, const Vec2d *points, int n,
        const Vec2d pos, double a
    )

    # cdef const int n3 = 3
    # ctypedef Vec[n3, float] Vec3f
    # ctypedef Vec[n3, double] Vec3d

    # cdef void transform_points_3f(
    #     Vec3f *out, const Vec3f *points, int n,
    #     const Vec3f pos, float a
    # )
