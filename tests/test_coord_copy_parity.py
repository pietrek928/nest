"""Letter B: ring_from_coords buffer vs iterable parity + coord_copy telem."""

import math
import time

import numpy as np
from shapely.geometry import Polygon, box

from nest_graph.geometry import (
    Geometry,
    coord_copy_buffer_n,
    coord_copy_iter_n,
    reset_coord_copy_telem,
)
from tests.fixtures.shapes import c_shape, l_shape, square_with_hole


def _ring_tuples(poly: Polygon) -> list[tuple[float, float]]:
    return [(float(x), float(y)) for x, y in poly.exterior.coords]


def _assert_area_close(a: Geometry, b: Geometry, *, tol: float = 1e-6) -> None:
    aa = float(a.area())
    bb = float(b.area())
    assert math.isclose(aa, bb, rel_tol=0.0, abs_tol=tol), (aa, bb)


def test_coord_copy_from_shapely_uses_buffer_path():
    reset_coord_copy_telem()
    g = Geometry.from_shapely(box(0, 0, 4, 3))
    assert float(g.area()) > 0.0
    assert coord_copy_buffer_n() >= 1
    assert coord_copy_iter_n() == 0


def test_coord_copy_list_tuples_uses_iter_path():
    ring = _ring_tuples(box(0, 0, 4, 3))
    reset_coord_copy_telem()
    g = Geometry.from_ring(ring)
    assert float(g.area()) > 0.0
    assert coord_copy_iter_n() >= 1


def test_coord_copy_ndarray_uses_buffer_path():
    ring = np.asarray(_ring_tuples(box(0, 0, 4, 3)), dtype=np.float64)
    # Drop closing duplicate if present for Nx2 buffer ingest.
    if len(ring) >= 2 and np.allclose(ring[0], ring[-1]):
        ring = ring[:-1]
    reset_coord_copy_telem()
    g = Geometry.from_ring(ring)
    assert float(g.area()) > 0.0
    assert coord_copy_buffer_n() >= 1


def test_ring_parity_l_shape_buffer_vs_iterable():
    poly = l_shape().buffer(0)
    tuples = _ring_tuples(poly)
    arr = np.asarray(tuples[:-1] if tuples[0] == tuples[-1] else tuples, dtype=np.float64)
    reset_coord_copy_telem()
    g_iter = Geometry.from_ring(tuples)
    n_iter = coord_copy_iter_n()
    reset_coord_copy_telem()
    g_buf = Geometry.from_ring(arr)
    n_buf = coord_copy_buffer_n()
    assert n_iter >= 1
    assert n_buf >= 1
    _assert_area_close(g_iter, g_buf)
    _assert_area_close(g_iter, Geometry.from_shapely(poly), tol=1e-4)


def test_ring_parity_c_shape_buffer_vs_iterable():
    poly = c_shape().buffer(0)
    tuples = _ring_tuples(poly)
    arr = np.asarray(tuples[:-1] if tuples[0] == tuples[-1] else tuples, dtype=np.float64)
    g_iter = Geometry.from_ring(tuples)
    g_buf = Geometry.from_ring(arr)
    _assert_area_close(g_iter, g_buf)
    _assert_area_close(g_iter, Geometry.from_shapely(poly), tol=1e-4)


def test_ring_parity_hole_from_shapely():
    poly = square_with_hole()
    reset_coord_copy_telem()
    g = Geometry.from_shapely(poly)
    assert coord_copy_buffer_n() >= 1
    # Usable area should be outer - hole (~100 - 16 for default fixture).
    assert float(g.area()) < float(Geometry.from_shapely(box(0, 0, 10, 10)).area())


def test_from_shapely_microbench_buffer_path_dominates():
    poly = l_shape().buffer(0)
    reset_coord_copy_telem()
    n = 40
    t0 = time.perf_counter()
    for _ in range(n):
        Geometry.from_shapely(poly)
    ms = (time.perf_counter() - t0) * 1000.0
    assert coord_copy_buffer_n() >= n
    assert coord_copy_iter_n() == 0
    # Soft ceiling: catch pathological regressions only.
    assert ms < 5000.0, f"from_shapely microbench too slow: {ms:.1f}ms for {n}"
