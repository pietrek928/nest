"""Expanded Geometry oracle matrix (Letter T): factories, kinds, Scene, faults."""

import math

import pytest
from shapely.geometry import (
    GeometryCollection,
    LineString,
    LinearRing,
    MultiLineString,
    MultiPoint,
    MultiPolygon,
    Point,
    Polygon,
    box,
)

from nest_graph.geometry import (
    Geometry,
    StaticCollisionScene,
    aabb_tree_query_box,
    batch_packing_clear,
    find_polygon_intersections,
)
from tests.fixtures.shapes import donut, l_shape, simple_square, square_with_hole
from tests.geometry_oracle import assert_packing_collide, shapely_packing_collide


def test_factories_area_and_contains():
    ring = [(0, 0), (2, 0), (2, 2), (0, 2)]
    g_conv = Geometry.from_convex_polygon(ring)
    g_ring = Geometry.from_ring(ring)
    g_rings = Geometry.from_rings([ring])
    g_shapely = Geometry.from_shapely(Polygon(ring))
    g_outline = Geometry.from_shapely_outline(Polygon(ring))
    g_wkt = Geometry.from_string(Polygon(ring).wkt)
    for g in (g_conv, g_ring, g_rings, g_shapely, g_outline, g_wkt):
        assert math.isclose(g.area(), 4.0, abs_tol=1e-9)
        assert g.contains_point(1.0, 1.0)
        assert not g.contains_point(5.0, 5.0)


def test_mutators_donut_matches_from_shapely():
    outer = [(0, 0), (10, 0), (10, 10), (0, 10)]
    hole = [(3, 3), (7, 3), (7, 7), (3, 7)]
    g = Geometry()
    g.append_convex_poly(outer)
    g.append_convex_hole(hole)
    g.finalize()
    ref = Geometry.from_shapely(donut())
    assert math.isclose(g.area(), ref.area(), rel_tol=1e-5)
    assert g.contains_point(1.0, 1.0) == ref.contains_point(1.0, 1.0)
    assert g.contains_point(5.0, 5.0) is False


def test_boundary_rings_subtractive_flags():
    g = Geometry.from_shapely(square_with_hole())
    rings = g.boundary_rings()
    assert len(rings) >= 2
    flags = [bool(is_sub) for _coords, is_sub in rings]
    assert any(flags), "donut/hole must mark a subtractive ring"
    coords, offsets, subtractive = g.boundary_rings_flat()
    assert len(offsets) >= 3
    assert any(subtractive)


def test_kinds_filled_stroke_point_collection():
    assert Geometry.from_shapely(simple_square()).area() > 0
    assert Geometry.from_shapely(donut()).area() > 0
    mp = MultiPolygon([box(0, 0, 1, 1), box(3, 3, 4, 4)])
    assert Geometry.from_shapely(mp).area() > 0
    stroke = Geometry.from_shapely(LineString([(0, 0), (1, 0), (1, 1)]))
    assert stroke.area() == 0.0
    assert Geometry.from_shapely(MultiLineString([[(0, 0), (1, 0)]])).area() == 0.0
    assert Geometry.from_shapely(Point(0, 0)).area() == 0.0
    assert Geometry.from_shapely(MultiPoint([(0, 0), (1, 1)])).area() == 0.0
    gc = GeometryCollection([box(0, 0, 1, 1), LineString([(2, 0), (3, 0)])])
    g = Geometry.from_shapely(gc)
    assert g.area() > 0


def test_linearring_stays_filled():
    ring = LinearRing([(0, 0), (2, 0), (2, 2), (0, 2)])
    g = Geometry.from_shapely(ring)
    assert g.area() > 0


def test_metrics_distance_bounds_circle():
    a = simple_square()
    b = box(5, 0, 7, 2)
    ga = Geometry.from_shapely(a)
    gb = Geometry.from_shapely(b)
    assert math.isclose(ga.distance(gb), a.distance(b), rel_tol=1e-4, abs_tol=1e-5)
    xmin, ymin, xmax, ymax = ga.bounds()
    assert xmax > xmin and ymax > ymin
    cx, cy, r = ga.get_bounding_circle()
    assert r > 0
    assert math.hypot(cx - 1, cy - 1) <= r + 1e-6


def test_contact_kiss_vs_penetrating():
    a = box(0, 0, 2, 2)
    kiss = box(2, 0, 4, 2)
    pen = box(1, 1, 3, 3)
    assert shapely_packing_collide(a, kiss) is False
    assert_packing_collide(a, kiss, expect=False)
    assert_packing_collide(a, pen, expect=True)


def test_transform_translate_rotate_apply():
    g = Geometry.from_shapely(simple_square())
    t = g.translate((1.0, 2.0))
    r = g.rotate(math.pi / 2, origin=(0.0, 0.0))
    at = g.apply_transform(1.0, 2.0, 0.0)
    # aabb = vertex box; bounds() is circle LTRB.
    assert math.isclose(t.aabb()[0], 1.0, abs_tol=1e-6)
    assert math.isclose(at.aabb()[0], 1.0, abs_tol=1e-6)
    assert r.area() > 0


def test_batch_packing_clear_and_intersections():
    part = Geometry.from_shapely(box(0, 0, 1, 1))
    obs = [Geometry.from_shapely(box(3, 3, 4, 4))]
    flags = batch_packing_clear(part, [(0.0, 0.0, 0.0)], obs)
    assert list(flags) == [1] or list(flags) == [True] or flags[0]
    hits = find_polygon_intersections(
        [Geometry.from_shapely(box(0, 0, 2, 2)), Geometry.from_shapely(box(1, 1, 3, 3))]
    )
    assert len(hits) >= 1


def test_scene_kiss_ok_margin_may_fail():
    a = Geometry.from_shapely(box(0, 0, 1, 1))
    b = Geometry.from_shapely(box(1.05, 0, 2.05, 1))
    assert Geometry.are_valid([a, b])
    scene = StaticCollisionScene.build([a])
    assert scene.query_any_violation(b, min_dist=0.1) is True
    assert scene.query_any_violation(b, min_dist=0.0) is False


def test_aabb_tree_smoke():
    boxes = [(0.0, 0.0, 1.0, 1.0), (5.0, 5.0, 6.0, 6.0), (0.5, 0.5, 1.5, 1.5)]
    hits = aabb_tree_query_box(boxes, -0.1, -0.1, 1.2, 1.2)
    assert 0 in hits
    assert 2 in hits
    assert 1 not in hits


def test_fault_nan_bowtie_nested_shells():
    nan_g = Geometry.from_convex_polygon([(float("nan"), 0.0), (1.0, 0.0), (1.0, 1.0)])
    assert nan_g.is_valid() is False
    bowtie = Polygon([(0, 0), (2, 2), (2, 0), (0, 2)])
    assert bowtie.is_valid is False
    assert Geometry.from_shapely(bowtie).is_valid() is False
    nested = MultiPolygon([box(0, 0, 4, 4), box(1, 1, 2, 2)])
    # Nested shells → invalid (crossing alone may miss containment nesting).
    assert Geometry.from_shapely(nested).is_valid() is False


def test_wkt_roundtrip_matrix():
    for shapely_g in (
        Point(1, 2),
        LineString([(0, 0), (1, 0)]),
        simple_square(),
        donut(),
        MultiPolygon([box(0, 0, 1, 1), box(3, 0, 4, 1)]),
    ):
        g = Geometry.from_string(shapely_g.wkt)
        g2 = Geometry.from_string(g.to_string())
        assert math.isclose(g.area(), g2.area(), rel_tol=1e-5, abs_tol=1e-9)


def test_l_shape_packing_self_ok():
    g = Geometry.from_shapely(l_shape())
    assert g.is_valid()
    assert Geometry.are_valid([g])
