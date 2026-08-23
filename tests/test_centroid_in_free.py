"""FreeCentroidPredicate oracle: distance SoT, no buffer(-margin)."""

import math

from shapely.geometry import Point, Polygon, box

from nest_graph.geometry import Geometry
from nest_graph.propose.void_selection import (
    FreeCentroidPredicate,
    centroid_in_free,
    xy_in_free,
)


def _oracle_covers(free: Polygon, x: float, y: float, margin: float) -> bool:
    p = Point(x, y)
    if not free.covers(p):
        return False
    if margin <= 1e-12:
        return True
    return float(p.distance(free.boundary)) >= margin - 1e-9


def test_margin_zero_boundary_covers():
    free = box(0, 0, 10, 10)
    pred = FreeCentroidPredicate(free, 0.0)
    assert pred.covers_anchor(0.0, 5.0)
    assert pred.covers_anchor(10.0, 5.0)
    assert not pred.covers_anchor(11.0, 5.0)


def test_margin_positive_interior():
    free = box(0, 0, 10, 10)
    margin = 1.0
    pred = FreeCentroidPredicate(free, margin)
    assert pred.covers_anchor(5.0, 5.0)
    assert not pred.covers_anchor(0.5, 5.0)
    assert _oracle_covers(free, 5.0, 5.0, margin)


def test_donut_hole_excluded():
    outer = box(0, 0, 10, 10)
    hole = box(3, 3, 7, 7)
    free = outer.difference(hole)
    pred = FreeCentroidPredicate(free, 0.0)
    assert pred.covers_anchor(1.0, 1.0)
    assert not pred.covers_anchor(5.0, 5.0)


def test_multipolygon():
    a = box(0, 0, 4, 4)
    b = box(6, 0, 10, 4)
    free = a.union(b)
    pred = FreeCentroidPredicate(free, 0.0)
    assert pred.covers_anchor(2.0, 2.0)
    assert pred.covers_anchor(8.0, 2.0)
    assert not pred.covers_anchor(5.0, 2.0)


def test_narrow_corridor_margin():
    free = box(0, 0, 10, 1.0)
    margin = 0.3
    pred = FreeCentroidPredicate(free, margin)
    assert pred.covers_anchor(5.0, 0.5)
    assert not pred.covers_anchor(0.1, 0.5)


def test_reflex_vertex_concave_void():
    # L-shaped void: reflex at (5,5)
    free = Polygon(
        [(0, 0), (10, 0), (10, 5), (5, 5), (5, 10), (0, 10), (0, 0)],
    )
    margin = 0.5
    pred = FreeCentroidPredicate(free, margin)
    for x, y in [(1.0, 1.0), (1.0, 8.0), (8.0, 1.0)]:
        assert pred.covers_anchor(x, y) == _oracle_covers(free, x, y, margin)
    # Near reflex: clearance to (5,5) corner is ~0.28
    assert not pred.covers_anchor(4.85, 4.85)


def test_covers_part_matches_centroid_in_free():
    free = box(0, 0, 10, 10)
    part = box(4, 4, 6, 6)
    margin = 0.25
    pred = FreeCentroidPredicate(free, margin)
    assert pred.covers_part(part) == centroid_in_free(part, free, interior_margin=margin)


def test_covers_anchor_matches_xy_in_free():
    free = box(0, 0, 10, 10)
    pred = FreeCentroidPredicate(free, 0.0)
    assert pred.covers_anchor(3.0, 3.0) == xy_in_free(3.0, 3.0, free)


def test_native_boundary_clearance_parity():
    free = box(0, 0, 10, 10)
    geom = Geometry.from_shapely(free)
    pred = FreeCentroidPredicate.from_shapely(free, 1.0)
    for x, y in [(5.0, 5.0), (0.5, 5.0), (9.5, 9.5)]:
        shapely_ok = _oracle_covers(free, x, y, 1.0)
        native_ok = pred.covers_anchor(x, y)
        assert native_ok == shapely_ok
        if shapely_ok:
            assert abs(float(geom.boundary_clearance(x, y)) - float(Point(x, y).distance(free.boundary))) < 1e-6


def test_empty_free_poly():
    pred = FreeCentroidPredicate(None, 0.0)
    assert not pred.covers_anchor(1.0, 1.0)
    pred2 = FreeCentroidPredicate(Polygon(), 0.0)
    assert not pred2.covers_anchor(1.0, 1.0)
