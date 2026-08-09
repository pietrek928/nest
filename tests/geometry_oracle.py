"""Shapely↔Geometry parity helpers for Python geometry tests."""

import math
from typing import Sequence

from shapely.geometry.base import BaseGeometry

from nest_graph.geometry import (
    Geometry,
    find_polygon_distances,
    find_polygon_intersections,
)
from nest_graph.utils import transform_poly


def as_geometry(poly: BaseGeometry) -> Geometry:
    return Geometry.from_shapely(poly)


def assert_intersects_matches_shapely(
    a: BaseGeometry,
    b: BaseGeometry,
    *,
    ga: Geometry | None = None,
    gb: Geometry | None = None,
) -> None:
    ga = ga if ga is not None else as_geometry(a)
    gb = gb if gb is not None else as_geometry(b)
    shapely_hit = bool(a.intersects(b))
    our_hit = bool(ga.intersects(gb))
    assert our_hit == shapely_hit, (
        f"intersects mismatch: shapely={shapely_hit} geometry={our_hit}"
    )


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
    if shapely_dist == 0.0:
        assert our_dist <= max(abs_, 1e-4), (
            f"expected contact/intersect, geometry dist={our_dist}"
        )
        return
    assert math.isclose(our_dist, shapely_dist, rel_tol=rel, abs_tol=abs_), (
        f"distance mismatch: shapely={shapely_dist} geometry={our_dist}"
    )


def assert_batch_intersects_match_shapely(polys: Sequence[BaseGeometry]) -> None:
    geoms = [as_geometry(p) for p in polys]
    hits = find_polygon_intersections(geoms)
    our_pairs = set()
    for h in hits:
        if isinstance(h, tuple):
            our_pairs.add(tuple(sorted((int(h[0]), int(h[1])))))
        else:
            our_pairs.add(tuple(sorted((h.polyA_idx, h.polyB_idx))))
    mismatches = []
    for i in range(len(polys)):
        for j in range(i + 1, len(polys)):
            shapely_hit = bool(polys[i].intersects(polys[j]))
            our_hit = (i, j) in our_pairs
            if shapely_hit != our_hit:
                mismatches.append((i, j, shapely_hit, our_hit))
    assert not mismatches, f"intersect mismatches: {mismatches}"


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
            if shapely_dist == 0.0:
                if pair in our and not our[pair].intersect:
                    our_dist = math.sqrt(our[pair].distance_sq)
                    if our_dist > 1e-4:
                        mismatches.append((i, j, "expected intersect", our_dist))
            elif pair in our:
                our_dist = math.sqrt(our[pair].distance_sq)
                if not math.isclose(our_dist, shapely_dist, rel_tol=rel, abs_tol=abs_):
                    mismatches.append((i, j, shapely_dist, our_dist))
    assert not mismatches, f"distance mismatches: {mismatches}"


def assert_pose_intersects_matches_shapely(
    p1: BaseGeometry,
    t1,
    p2: BaseGeometry,
    t2,
) -> None:
    s1 = transform_poly(p1, t1)
    s2 = transform_poly(p2, t2)
    g1 = as_geometry(p1).apply_transform(t1)
    g2 = as_geometry(p2).apply_transform(t2)
    assert bool(g1.intersects(g2)) == bool(s1.intersects(s2))


def assert_contains_point_matches(
    poly: BaseGeometry,
    x: float,
    y: float,
    *,
    g: Geometry | None = None,
) -> None:
    g = g if g is not None else as_geometry(poly)
    from shapely import Point

    assert bool(g.contains_point(x, y)) == bool(poly.contains(Point(x, y)))


def assert_pose_clear_matches_shapely(
    candidate: BaseGeometry,
    sheet: BaseGeometry,
    obstacles: Sequence[BaseGeometry],
    min_dist: float,
    *,
    cand_g: Geometry | None = None,
    board_g: Geometry | None = None,
    obs_g: list[Geometry] | None = None,
) -> None:
    """Compare Geometry pose_clear_geoms to Shapely contain + min distance."""
    from nest_graph.propose.placement_common import pose_clear_geoms

    cand_g = cand_g if cand_g is not None else as_geometry(candidate)
    board_g = board_g if board_g is not None else as_geometry(sheet)
    obs_g = obs_g if obs_g is not None else [as_geometry(o) for o in obstacles]

    our = pose_clear_geoms(cand_g, board_g, obs_g, min_dist)

    if candidate is None or candidate.is_empty:
        assert our is False
        return
    if not sheet.contains(candidate) and not sheet.buffer(1e-9).contains(candidate):
        assert our is False
        return
    if not obstacles:
        assert our is True
        return
    for o in obstacles:
        if candidate.intersects(o) and candidate.intersection(o).area > 1e-12:
            assert our is False
            return
        if float(candidate.distance(o)) + 1e-12 < float(min_dist):
            assert our is False
            return
    assert our is True
