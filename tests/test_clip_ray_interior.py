"""Q283: native clip_ray_interior parity vs Shapely LineString intersection."""

import math

from shapely import LineString, box

from nest_graph.geometry import Geometry
from tests.fixtures.shapes import c_shape, l_shape, square_with_hole


def _shapely_ray_samples(
    region,
    ox: float,
    oy: float,
    dx: float,
    dy: float,
    max_t: float,
    fracs=(0.1, 0.5),
) -> list[tuple[float, float]]:
    ray_len = math.hypot(dx, dy)
    if ray_len <= 0.0 or max_t <= 0.0:
        return []
    ux, uy = dx / ray_len, dy / ray_len
    ex = ox + ux * max_t
    ey = oy + uy * max_t
    ray = LineString([(ox, oy), (ex, ey)])
    valid = ray.intersection(region)
    if valid.is_empty:
        return []
    out: list[tuple[float, float]] = []
    segments = [valid] if valid.geom_type == "LineString" else list(valid.geoms)
    for seg in segments:
        if seg.geom_type != "LineString" or seg.length <= 0:
            continue
        for frac in fracs:
            pt = seg.interpolate(float(frac), normalized=True)
            out.append((float(pt.x), float(pt.y)))
    return out


def _assert_clip_parity(region_poly, ox, oy, dx, dy, max_t, *, tol=1e-4):
    g = Geometry.from_shapely(region_poly)
    shapely_pts = _shapely_ray_samples(region_poly, ox, oy, dx, dy, max_t)
    native_pts = list(g.clip_ray_interior((ox, oy), (dx, dy), max_t, (0.1, 0.5)))
    assert len(native_pts) == len(shapely_pts)
    for (nx, ny), (sx, sy) in zip(
        sorted(native_pts), sorted(shapely_pts), strict=True,
    ):
        assert math.isclose(nx, sx, abs_tol=tol)
        assert math.isclose(ny, sy, abs_tol=tol)


def test_clip_ray_l_shape_interior():
    region = l_shape().buffer(0)
    _assert_clip_parity(region, 1.0, 1.0, 8.0, 0.0, 20.0)


def test_clip_ray_c_shape_concave():
    region = c_shape().buffer(0)
    # Ray through void pocket (not from boundary vertex).
    _assert_clip_parity(region, 4.0, 5.0, 1.0, 0.0, 8.0)


def test_clip_ray_c_shape_boundary_ray():
    """Boundary-origin rays may omit thin wall segments (native interior-only)."""
    region = c_shape().buffer(0)
    g = Geometry.from_shapely(region)
    dx, dy = 0.0, 6.0
    max_t = 20.0
    native_pts = list(g.clip_ray_interior((2.0, 2.0), (dx, dy), max_t, (0.1, 0.5)))
    assert native_pts
    for x, y in native_pts:
        assert region.contains(__import__("shapely").Point(x, y))


def test_clip_ray_hole_annulus():
    region = square_with_hole()
    g = Geometry.from_shapely(region)
    # Ray through hole center should miss interior shell samples in hole.
    native_pts = list(g.clip_ray_interior((5.0, 5.0), (10.0, 0.0), 15.0, (0.5,)))
    for x, y in native_pts:
        assert region.contains(__import__("shapely").Point(x, y))


def test_clip_ray_miss_exterior():
    region = box(0, 0, 10, 10)
    g = Geometry.from_shapely(region)
    dx, dy = 10.0, 0.0
    max_t = math.hypot(dx, dy)
    fracs = (0.5,)
    native_pts = list(g.clip_ray_interior((-5.0, 5.0), (dx, dy), max_t, fracs))
    shapely_pts = _shapely_ray_samples(region, -5.0, 5.0, dx, dy, max_t, fracs=fracs)
    assert len(native_pts) == len(shapely_pts)


def test_clip_ray_interval_march_regression():
    """Fixed t×max_t along full ray can lie outside; segment midpoint must hit."""
    region = box(0, 0, 10, 10)
    g = Geometry.from_shapely(region)
    # Short segment inside from (1,1) toward (2,2) — interior samples exist.
    dx, dy = 1.0, 1.0
    max_t = math.hypot(dx, dy)
    native_pts = list(g.clip_ray_interior((1.0, 1.0), (dx, dy), max_t, (0.1, 0.5)))
    assert len(native_pts) >= 1
    shapely_pts = _shapely_ray_samples(region, 1.0, 1.0, dx, dy, max_t)
    assert len(native_pts) == len(shapely_pts)
