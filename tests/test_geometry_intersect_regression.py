"""Regression tests for Geometry intersection vs Shapely (placement paths)."""

import numpy as np
import pytest

from nest_graph.geometry import Geometry
from nest_graph.utils import transform_poly
from tests.geometry_oracle import shapely_packing_collide


def test_intersects_regression_cases_match_shapely(t1, t2, rect_poly, tri_poly):
    s1 = transform_poly(rect_poly, t1)
    s2 = transform_poly(tri_poly, t2)
    assert shapely_packing_collide(s1, s2)
    g1 = Geometry.from_convex_polygon(list(rect_poly.exterior.coords)[:-1]).apply_transform(t1)
    g2 = Geometry.from_convex_polygon(list(tri_poly.exterior.coords)[:-1]).apply_transform(t2)
    assert g1.intersects(g2)


def test_intersects_any_matches_pairwise(
    rect_poly, tri_poly, first_regression_intersect_transform
):
    t1, t2 = first_regression_intersect_transform
    g1 = Geometry.from_convex_polygon(list(rect_poly.exterior.coords)[:-1]).apply_transform(t1)
    g2 = Geometry.from_convex_polygon(list(tri_poly.exterior.coords)[:-1]).apply_transform(t2)
    assert g1.intersects(g2) == g1.intersects_any([g2])


def _fuzz_packing_mismatches(seed: int, rect_poly, tri_poly, *, n: int, use_shapely_base: bool) -> int:
    rng = np.random.default_rng(seed)
    base_a = (
        Geometry.from_shapely(rect_poly)
        if use_shapely_base
        else Geometry.from_convex_polygon(list(rect_poly.exterior.coords)[:-1])
    )
    base_b = (
        Geometry.from_shapely(tri_poly)
        if use_shapely_base
        else Geometry.from_convex_polygon(list(tri_poly.exterior.coords)[:-1])
    )
    mismatches = 0
    for _ in range(n):
        t1 = rng.uniform(-0.35, 0.35, 3) * [1.0, 1.0, 2 * np.pi]
        t2 = rng.uniform(-0.35, 0.35, 3) * [1.0, 1.0, 2 * np.pi]
        want = shapely_packing_collide(
            transform_poly(rect_poly, t1), transform_poly(tri_poly, t2)
        )
        got = bool(base_a.apply_transform(t1).intersects(base_b.apply_transform(t2)))
        if want != got:
            mismatches += 1
    return mismatches


def test_intersects_fuzz_sample_matches_shapely_smoke(rect_poly, tri_poly):
    assert _fuzz_packing_mismatches(0, rect_poly, tri_poly, n=400, use_shapely_base=False) == 0


def test_from_shapely_convex_fuzz_matches_shapely_smoke(rect_poly, tri_poly):
    assert _fuzz_packing_mismatches(0, rect_poly, tri_poly, n=400, use_shapely_base=True) == 0


@pytest.mark.parametrize("seed", [1, 7, 42, 99])
def test_intersects_fuzz_sample_matches_shapely_extra_seeds(seed: int, rect_poly, tri_poly):
    assert _fuzz_packing_mismatches(seed, rect_poly, tri_poly, n=200, use_shapely_base=False) == 0


@pytest.mark.parametrize("seed", [1, 7, 42, 99])
def test_from_shapely_convex_fuzz_matches_shapely_extra_seeds(seed: int, rect_poly, tri_poly):
    assert _fuzz_packing_mismatches(seed, rect_poly, tri_poly, n=200, use_shapely_base=True) == 0

def test_from_convex_polygon_inflates_concave_notch_intersection(l_shape_raw, notch_square):
    host = Geometry.from_convex_polygon(list(l_shape_raw.exterior.coords)[:-1])
    guest = Geometry.from_shapely(notch_square)
    assert not l_shape_raw.intersects(notch_square)
    assert not Geometry.from_shapely(l_shape_raw).intersects(guest)
    assert host.intersects(guest), "convex-path host falsely fills the reflex notch"


def test_from_shapely_concave_pair_matches_shapely(l_shape_poly, concave_placement_transforms):
    t1, t2 = concave_placement_transforms
    assert not transform_poly(l_shape_poly, t1).intersects(transform_poly(l_shape_poly, t2))
    base = Geometry.from_shapely(l_shape_poly)
    assert not base.apply_transform(t1).intersects(base.apply_transform(t2))
