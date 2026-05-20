"""Regression tests for Geometry intersection vs Shapely (placement paths)."""

import numpy as np
import pytest

from nest_graph.build_graph import make_polygon_graph
from nest_graph.geometry import Geometry
from nest_graph.utils import transform_poly


def _geom_intersects(p1, t1, p2, t2) -> bool:
    g1 = Geometry.from_convex_polygon(list(p1.exterior.coords)[:-1]).apply_transform(t1)
    g2 = Geometry.from_convex_polygon(list(p2.exterior.coords)[:-1]).apply_transform(t2)
    return g1.intersects(g2)


def _geom_intersects_shapely_base(p1, t1, p2, t2) -> bool:
    g1 = Geometry.from_shapely(p1).apply_transform(t1)
    g2 = Geometry.from_shapely(p2).apply_transform(t2)
    return g1.intersects(g2)


def test_intersects_regression_cases_match_shapely(t1, t2, rect_poly, tri_poly):
    shapely_hit = transform_poly(rect_poly, t1).intersects(transform_poly(tri_poly, t2))
    assert shapely_hit
    assert _geom_intersects(rect_poly, t1, tri_poly, t2)


def test_intersects_any_matches_pairwise(
    rect_poly, tri_poly, first_regression_intersect_transform
):
    t1, t2 = first_regression_intersect_transform
    g1 = Geometry.from_convex_polygon(list(rect_poly.exterior.coords)[:-1]).apply_transform(t1)
    g2 = Geometry.from_convex_polygon(list(tri_poly.exterior.coords)[:-1]).apply_transform(t2)
    assert g1.intersects(g2) == g1.intersects_any([g2])


@pytest.mark.parametrize("seed", [0, 1, 7, 42, 99])
def test_intersects_fuzz_sample_matches_shapely(seed: int, rect_poly, tri_poly):
    rng = np.random.default_rng(seed)
    mismatches = 0
    for _ in range(400):
        t1 = rng.uniform(-0.35, 0.35, 3) * [1.0, 1.0, 2 * np.pi]
        t2 = rng.uniform(-0.35, 0.35, 3) * [1.0, 1.0, 2 * np.pi]
        sh = transform_poly(rect_poly, t1).intersects(transform_poly(tri_poly, t2))
        g = _geom_intersects(rect_poly, t1, tri_poly, t2)
        if sh != g:
            mismatches += 1
    assert mismatches == 0, f"seed={seed} had {mismatches} Shapely/Geometry mismatches"


@pytest.mark.parametrize("seed", [0, 1, 7, 42, 99])
def test_from_shapely_convex_fuzz_matches_shapely(seed: int, rect_poly, tri_poly):
    """Placement bases use from_shapely; convex parts must still match Shapely intersects."""
    rng = np.random.default_rng(seed)
    mismatches = 0
    for _ in range(400):
        t1 = rng.uniform(-0.35, 0.35, 3) * [1.0, 1.0, 2 * np.pi]
        t2 = rng.uniform(-0.35, 0.35, 3) * [1.0, 1.0, 2 * np.pi]
        sh = transform_poly(rect_poly, t1).intersects(transform_poly(tri_poly, t2))
        g = _geom_intersects_shapely_base(rect_poly, t1, tri_poly, t2)
        if sh != g:
            mismatches += 1
    assert mismatches == 0, f"seed={seed} had {mismatches} Shapely/Geometry mismatches"


def test_from_convex_polygon_inflates_concave_notch_intersection(l_shape_raw, notch_square):
    """from_convex_polygon on a concave outline is not the same solid as the Shapely polygon."""
    host = Geometry.from_convex_polygon(list(l_shape_raw.exterior.coords)[:-1])
    guest = Geometry.from_shapely(notch_square)
    assert not l_shape_raw.intersects(notch_square)
    assert not Geometry.from_shapely(l_shape_raw).intersects(guest)
    assert host.intersects(guest), "convex-path host falsely fills the reflex notch"


def test_from_convex_polygon_concave_pair_false_positive(
    l_shape_poly, concave_placement_transforms
):
    t1, t2 = concave_placement_transforms
    assert not transform_poly(l_shape_poly, t1).intersects(transform_poly(l_shape_poly, t2))
    base = Geometry.from_convex_polygon(list(l_shape_poly.exterior.coords)[:-1])
    assert base.apply_transform(t1).intersects(base.apply_transform(t2))


def test_from_shapely_concave_pair_matches_shapely(l_shape_poly, concave_placement_transforms):
    t1, t2 = concave_placement_transforms
    assert not transform_poly(l_shape_poly, t1).intersects(transform_poly(l_shape_poly, t2))
    base = Geometry.from_shapely(l_shape_poly)
    assert not base.apply_transform(t1).intersects(base.apply_transform(t2))


def test_make_polygon_graph_no_overlaps_without_collision_edge(nest_board, rect_poly, tri_poly):
    rng = np.random.default_rng(7)
    t0 = rng.uniform(-0.35, 0.35, (60, 3)) * [1.0, 1.0, 2 * np.pi]
    t1 = rng.uniform(-0.35, 0.35, (60, 3)) * [1.0, 1.0, 2 * np.pi]
    graph, polys, _gid, _trans = make_polygon_graph(nest_board, [(rect_poly, t0), (tri_poly, t1)])

    for i in range(len(polys)):
        for j in range(i + 1, len(polys)):
            if polys[i].intersects(polys[j]):
                assert j in graph._obj.collisions[i]
                assert i in graph._obj.collisions[j]
