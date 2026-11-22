"""Shapely↔Geometry parity helpers for Python geometry tests.

Packing collide SoT is Penetrating / interior overlap only. Shapely ``intersects``
is True on shared-edge kisses; use ``shapely_packing_collide`` for packing asserts.
"""

import math
from typing import Sequence

from shapely import Point
from shapely.geometry.base import BaseGeometry

from nest_graph.geometry import (
    Geometry,
    find_polygon_distances,
    find_polygon_intersections,
)
from nest_graph.placement.common import is_pose_clear
from nest_graph.utils import transform_poly


def as_geometry(poly: BaseGeometry) -> Geometry:
    return Geometry.from_shapely(poly)


def shapely_packing_collide(a: BaseGeometry, b: BaseGeometry) -> bool:
    """True iff interiors overlap (Touch / shared boundary is not a packing hit)."""
    if not a.intersects(b):
        return False
    if a.touches(b):
        return False
    return True


def assert_packing_collide(
    a: BaseGeometry,
    b: BaseGeometry,
    *,
    expect: bool | None = None,
    ga: Geometry | None = None,
    gb: Geometry | None = None,
) -> None:
    ga = ga if ga is not None else as_geometry(a)
    gb = gb if gb is not None else as_geometry(b)
    want = shapely_packing_collide(a, b) if expect is None else expect
    got = bool(ga.intersects(gb))
    assert got is want, (
        f"packing collide mismatch: expected={want} geometry={got} "
        f"shapely_intersects={a.intersects(b)} touches={a.touches(b)}"
    )
    assert bool(gb.intersects(ga)) is want


def assert_distance_close(
    a: BaseGeometry,
    b: BaseGeometry,
    *,
    ga: Geometry | None = None,
    gb: Geometry | None = None,
    rel: float = 1e-4,
    abs_: float = 1e-5,
) -> None:
    ga = ga if ga is not None else as_geometry(a)
    gb = gb if gb is not None else as_geometry(b)
    shapely_dist = float(a.distance(b))
    our_dist = float(ga.distance(gb))
    packing = bool(ga.intersects(gb))
    if shapely_dist == 0.0:
        if a.touches(b) and not shapely_packing_collide(a, b):
            assert packing is False, "kiss must not be packing collide"
            assert our_dist <= max(abs_, 1e-4), (
                f"kiss distance expected ~0, got {our_dist}"
            )
            return
        assert packing is True or our_dist <= max(abs_, 1e-4), (
            f"expected overlap/contact, packing={packing} dist={our_dist}"
        )
        return
    assert packing is False
    assert math.isclose(our_dist, shapely_dist, rel_tol=rel, abs_tol=abs_), (
        f"distance mismatch: shapely={shapely_dist} geometry={our_dist}"
    )


def assert_batch_packing_collide(polys: Sequence[BaseGeometry]) -> None:
    geoms = [as_geometry(p) for p in polys]
    hits = find_polygon_intersections(geoms)
    our_pairs: set[tuple[int, int]] = set()
    for h in hits:
        if isinstance(h, tuple):
            our_pairs.add(tuple(sorted((int(h[0]), int(h[1])))))
        else:
            our_pairs.add(tuple(sorted((h.polyA_idx, h.polyB_idx))))
    mismatches = []
    for i in range(len(polys)):
        for j in range(i + 1, len(polys)):
            want = shapely_packing_collide(polys[i], polys[j])
            got = (i, j) in our_pairs
            if want != got:
                mismatches.append((i, j, want, got))
    assert not mismatches, f"packing batch mismatches: {mismatches}"


# Back-compat names used by older tests; packing-aware (not raw Shapely intersects).
assert_batch_intersects_match_shapely = assert_batch_packing_collide
assert_intersects_matches_shapely = assert_packing_collide


def assert_batch_distances_match_shapely(
    polys: Sequence[BaseGeometry],
    *,
    aura: float = 100.0,
    rel: float = 1e-4,
    abs_: float = 1e-5,
) -> None:
    geoms = [as_geometry(p) for p in polys]
    results = find_polygon_distances(geoms, aura=aura)
    our = {}
    for r in results:
        our[tuple(sorted((r.polyA_idx, r.polyB_idx)))] = r
    mismatches = []
    for i in range(len(polys)):
        for j in range(i + 1, len(polys)):
            shapely_dist = float(polys[i].distance(polys[j]))
            pair = (i, j)
            packing = shapely_packing_collide(polys[i], polys[j])
            if shapely_dist == 0.0 and packing:
                if pair not in our or not our[pair].intersect:
                    if pair in our:
                        our_dist = math.sqrt(our[pair].distance_sq)
                        if our_dist > 1e-4:
                            mismatches.append((i, j, "expected packing intersect", our_dist))
            elif shapely_dist == 0.0 and not packing:
                if pair in our and our[pair].intersect:
                    mismatches.append((i, j, "kiss marked intersect", True))
            elif pair in our:
                our_dist = math.sqrt(our[pair].distance_sq)
                if not math.isclose(our_dist, shapely_dist, rel_tol=rel, abs_tol=abs_):
                    mismatches.append((i, j, shapely_dist, our_dist))
    assert not mismatches, f"distance mismatches: {mismatches}"


def assert_pose_packing_collide(
    p1: BaseGeometry,
    t1,
    p2: BaseGeometry,
    t2,
) -> None:
    s1 = transform_poly(p1, t1)
    s2 = transform_poly(p2, t2)
    g1 = as_geometry(p1).apply_transform(t1)
    g2 = as_geometry(p2).apply_transform(t2)
    want = shapely_packing_collide(s1, s2)
    assert bool(g1.intersects(g2)) is want


# Back-compat alias
assert_pose_intersects_matches_shapely = assert_pose_packing_collide


def assert_contains_point_matches(
    poly: BaseGeometry,
    x: float,
    y: float,
    *,
    g: Geometry | None = None,
) -> None:
    g = g if g is not None else as_geometry(poly)
    assert bool(g.contains_point(x, y)) == bool(poly.contains(Point(x, y)))


def assert_area_matches_shapely(
    poly: BaseGeometry,
    *,
    g: Geometry | None = None,
    rel: float = 1e-9,
    abs_: float = 1e-9,
) -> None:
    g = g if g is not None else as_geometry(poly)
    assert math.isclose(float(g.area()), float(poly.area), rel_tol=rel, abs_tol=abs_), (
        f"area mismatch: shapely={poly.area} geometry={g.area()}"
    )


def assert_scene_margin_vs_packing(
    candidate: BaseGeometry,
    obstacles: Sequence[BaseGeometry],
    min_dist: float,
) -> None:
    """gap in (0, min_dist): packing clear, is_pose_clear False."""
    cand_g = as_geometry(candidate)
    obs_g = [as_geometry(o) for o in obstacles]
    packing_hit = any(cand_g.intersects(o) for o in obs_g)
    assert packing_hit is False
    assert is_pose_clear(cand_g, [], obs_g, float(min_dist)) is False
