"""Geometry.is_valid / are_valid vs Shapely topology + packing Penetrating SoT."""

import pytest
from shapely.geometry import LineString, MultiPolygon, Point, Polygon, box

from nest_graph.geometry import Geometry, StaticCollisionScene
from tests.fixtures.shapes import donut, simple_square


def test_valid_square_and_donut():
    assert Geometry.from_shapely(simple_square()).is_valid()
    assert Geometry.from_shapely(donut()).is_valid()
    assert simple_square().is_valid
    assert donut().is_valid


def test_bowtie_invalid():
    bowtie = Polygon([(0, 0), (2, 2), (2, 0), (0, 2)])
    assert bowtie.is_valid is False
    assert Geometry.from_shapely(bowtie).is_valid() is False


def test_hole_outside_shell_invalid():
    outer = [(0, 0), (4, 0), (4, 4), (0, 4)]
    hole = [(5, 5), (6, 5), (6, 6), (5, 6)]
    poly = Polygon(outer, [hole])
    assert poly.is_valid is False
    assert Geometry.from_shapely(poly).is_valid() is False


def test_disjoint_multipolygon_valid():
    mp = MultiPolygon([box(0, 0, 1, 1), box(3, 3, 4, 4)])
    assert mp.is_valid
    assert Geometry.from_shapely(mp).is_valid()


def test_open_line_and_point_valid():
    assert Geometry.from_shapely(LineString([(0, 0), (1, 0), (1, 1)])).is_valid()
    assert Geometry.from_shapely(Point(0, 0)).is_valid()


def test_are_valid_disjoint_and_kiss():
    a = Geometry.from_shapely(box(0, 0, 2, 2))
    b = Geometry.from_shapely(box(3, 0, 5, 2))
    kiss = Geometry.from_shapely(box(2, 0, 4, 2))
    assert Geometry.are_valid([a, b])
    assert Geometry.are_valid([a, kiss])  # edge kiss is not Penetrating


def test_are_valid_penetrating_fails():
    a = Geometry.from_shapely(box(0, 0, 2, 2))
    b = Geometry.from_shapely(box(1, 1, 3, 3))
    assert Geometry.are_valid([a, b]) is False


def test_are_valid_rejects_stroke():
    stroke = Geometry.from_shapely(LineString([(0, 0), (1, 0)]))
    filled = Geometry.from_shapely(box(0, 0, 1, 1))
    with pytest.raises(ValueError, match="Point/Stroke"):
        Geometry.are_valid([filled, stroke])


def test_are_valid_distinct_from_scene_clearance():
    a = Geometry.from_shapely(box(0, 0, 1, 1))
    b = Geometry.from_shapely(box(1.05, 0, 2.05, 1))
    assert Geometry.are_valid([a, b])
    scene = StaticCollisionScene.build([a])
    # Packing independence passes; Scene margin can still violate.
    assert scene.query_any_violation(b, min_dist=0.1) is True
