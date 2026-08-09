"""Polylabel: C++ Mapbox vs Shapely oracle."""

import math

import pytest
from shapely.ops import polylabel as shapely_polylabel

from nest_graph.geometry import Geometry, polylabel, polylabel_rings
from tests.fixtures.shapes import nest_triangle_board, simple_square, unit_square


def _rings_from_poly(poly):
    rings = [list(poly.exterior.coords)]
    for interior in poly.interiors:
        rings.append(list(interior.coords))
    return rings


@pytest.mark.parametrize(
    "factory",
    [simple_square, unit_square, nest_triangle_board],
    ids=["simple_square", "unit_square", "triangle"],
)
def test_polylabel_matches_shapely_oracle(factory):
    poly = factory()
    precision = 0.1
    sx = float(shapely_polylabel(poly, tolerance=precision).x)
    sy = float(shapely_polylabel(poly, tolerance=precision).y)

    rx, ry, rr = polylabel_rings(_rings_from_poly(poly), precision)
    gx, gy, gr = polylabel(Geometry.from_shapely(poly), precision)

    assert math.hypot(rx - sx, ry - sy) <= precision * 2.0 + 1e-6
    assert math.hypot(gx - sx, gy - sy) <= precision * 2.0 + 1e-6
    assert math.isclose(rx, gx, abs_tol=1e-9)
    assert math.isclose(ry, gy, abs_tol=1e-9)
    assert rr > 0.0 and gr > 0.0
    # Distance should match distance to exterior within precision.
    from shapely import Point

    assert abs(rr - float(Point(rx, ry).distance(poly.exterior))) <= precision + 1e-6


def test_void_topology_prefers_cpp_polylabel():
    from nest_graph.propose import void_topology

    poly = simple_square()
    pt = void_topology.polylabel(poly, tolerance=0.1)
    assert abs(float(pt.x) - 1.0) < 0.2
    assert abs(float(pt.y) - 1.0) < 0.2
