"""Native voronoi_vertices (Delaunay dual) + densify/batch-contains propose path."""

import inspect
import math

import numpy as np
import pytest
from shapely.geometry import Point, box
from shapely.ops import unary_union

from nest_graph import geometry as geom
from nest_graph.placement.geo import densify_points, propose_placements_voronoi
from nest_graph.propose.context import search_region_for_placement


def _sites(*xy):
    return [(float(x), float(y)) for x, y in xy]


def test_voronoi_empty_small_inputs():
    assert geom.voronoi_vertices([]) == []
    assert geom.voronoi_vertices([(0.0, 0.0)]) == []
    assert geom.voronoi_vertices([(0.0, 0.0), (1.0, 0.0)]) == []


def test_voronoi_collinear_empty():
    assert geom.voronoi_vertices(_sites((0, 0), (1, 0), (2, 0), (3, 0))) == []


def test_voronoi_equilateral_center():
    s = math.sqrt(3.0) / 2.0
    v = geom.voronoi_vertices(_sites((0, 0), (1, 0), (0.5, s)))
    assert len(v) == 1
    assert v[0][0] == pytest.approx(0.5, abs=1e-9)
    assert v[0][1] == pytest.approx(s / 3.0, abs=1e-9)


def test_voronoi_square_one_center():
    v = geom.voronoi_vertices(_sites((0, 0), (1, 0), (1, 1), (0, 1)))
    assert len(v) == 1
    assert v[0][0] == pytest.approx(0.5, abs=1e-9)
    assert v[0][1] == pytest.approx(0.5, abs=1e-9)


def test_voronoi_lattice_6x6_unique():
    sites = [(float(i), float(j)) for i in range(6) for j in range(6)]
    v = geom.voronoi_vertices(sites)
    assert len(v) == 25


def test_voronoi_lex_sorted_stable():
    sites = _sites((0, 0), (2, 0), (2, 2), (0, 2), (1, 1))
    a = geom.voronoi_vertices(sites)
    b = geom.voronoi_vertices(sites)
    assert a == b
    for i in range(1, len(a)):
        assert a[i - 1] <= a[i]


def test_voronoi_shuffled_sites_identical():
    a = _sites((0, 0), (2, 0), (2, 2), (0, 2), (1, 1))
    b = _sites((1, 1), (0, 2), (2, 2), (0, 0), (2, 0))
    assert geom.voronoi_vertices(a) == geom.voronoi_vertices(b)


def test_voronoi_nan_inf_ignored():
    clean = geom.voronoi_vertices(_sites((0, 0), (1, 0), (0.5, 0.8)))
    dirty = geom.voronoi_vertices(
        [
            (float("nan"), 0.0),
            (0.0, 0.0),
            (1.0, 0.0),
            (float("inf"), 1.0),
            (0.5, 0.8),
            (0.0, float("-inf")),
        ]
    )
    assert dirty == clean


def test_voronoi_all_negative():
    s = math.sqrt(3.0) / 2.0
    v = geom.voronoi_vertices(_sites((-100, -100), (-99, -100), (-99.5, -100 + s)))
    assert len(v) == 1
    assert v[0][0] == pytest.approx(-99.5, abs=1e-9)


def test_voronoi_equidistant_to_three_sites():
    sites = np.array([[0.0, 0.0], [2.0, 0.0], [2.0, 2.0], [0.0, 2.0], [1.0, 1.0]])
    verts = geom.voronoi_vertices([(float(x), float(y)) for x, y in sites])
    assert verts
    for vx, vy in verts:
        d = np.sqrt((sites[:, 0] - vx) ** 2 + (sites[:, 1] - vy) ** 2)
        d.sort()
        assert d[2] == pytest.approx(d[0], abs=1e-5)


def test_voronoi_scipy_parity_random():
    scipy = pytest.importorskip("scipy")
    from scipy.spatial import Voronoi

    rng = np.random.default_rng(0)
    pts = rng.uniform(0.0, 10.0, size=(40, 2))
    sites = [(float(x), float(y)) for x, y in pts]
    ours = geom.voronoi_vertices(sites)
    vor = Voronoi(pts)
    ref = {
        (round(float(x), 6), round(float(y), 6))
        for x, y in vor.vertices
        if np.isfinite(x) and np.isfinite(y)
    }
    ok = [(round(x, 6), round(y, 6)) for x, y in ours]
    hit = sum(1 for k in ok if k in ref)
    assert hit == len(ok)


def test_voronoi_in_region_l_shape():
    region = box(0, 0, 3, 3).difference(box(1, 1, 3, 3))
    region_g = geom.Geometry.from_shapely(region)
    sites = region_g.sample_boundary_rings(0.35, exteriors_only=False)
    verts = geom.voronoi_vertices(sites)
    flags = region_g.contains_points_batch(verts)
    kept = [(x, y) for (x, y), ok in zip(verts, flags, strict=True) if ok]
    assert len(kept) >= 1
    for x, y in kept:
        assert region.contains(Point(x, y))


def test_voronoi_holed_free_rejects_island_interior():
    sheet = box(0, 0, 10, 10)
    island = box(4, 4, 6, 6)
    free = sheet.difference(island)
    region_g = geom.Geometry.from_shapely(free)
    sites = region_g.sample_boundary_rings(0.8, exteriors_only=False)
    verts = geom.voronoi_vertices(sites)
    flags = region_g.contains_points_batch(verts)
    for (x, y), ok in zip(verts, flags, strict=True):
        if ok:
            assert not island.contains(Point(x, y))


def test_voronoi_annulus_corridor_medial():
    free = box(0, 0, 20, 20).difference(box(6, 6, 14, 14))
    region_g = geom.Geometry.from_shapely(free)
    sites = region_g.sample_boundary_rings(1.0, exteriors_only=False)
    verts = geom.voronoi_vertices(sites)
    flags = region_g.contains_points_batch(verts)
    kept = [(x, y) for (x, y), ok in zip(verts, flags, strict=True) if ok]
    assert kept
    # At least one medial-ish vertex in the corridor (clearance ~3 to hole).
    assert any(abs(math.hypot(x - 10.0, y - 10.0) - 7.0) < 1.5 for x, y in kept)


def test_densify_closed_multipolygon_no_crash():
    a = box(0, 0, 2, 2)
    b = box(3, 3, 5, 5)
    mp = densify_points(unary_union([a, b]), 0.5)
    sites = [(float(g.x), float(g.y)) for g in mp.geoms]
    verts = geom.voronoi_vertices(sites)
    assert all(math.isfinite(x) and math.isfinite(y) for x, y in verts)


def test_sample_boundary_rings_native():
    g = geom.Geometry.from_shapely(box(0, 0, 10, 4))
    pts = g.sample_boundary_rings(1.0, exteriors_only=True)
    assert len(pts) >= 8
    flags = g.contains_points_batch(pts)
    # Boundary samples may sit on edge; batch matches single contains_point.
    for (x, y), ok in zip(pts, flags, strict=True):
        assert bool(ok) == bool(g.contains_point(float(x), float(y)))


def test_geo_module_has_no_shapely_voronoi_import():
    import nest_graph.placement.geo as geo_mod

    src = inspect.getsource(geo_mod)
    assert "voronoi_diagram" not in src
    assert "ops.voronoi" not in src
    assert "voronoi_vertices" in src


def test_propose_placements_voronoi_emits_inside_region():
    from nest_graph.config import ProposeConfig
    from nest_graph.propose.geometry import ProposeGeometry

    sheet = box(0, 0, 40, 40)
    obstacle = box(5, 5, 15, 15)
    part = box(0, 0, 2, 2)
    cfg = ProposeConfig()
    propose_geom = ProposeGeometry(
        sheet, obstacle, part, 0.1, propose_cfg=cfg
    )
    poses = propose_placements_voronoi(
        obstacle,
        part,
        sheet,
        0.1,
        use_free_region=True,
        num_angles=4,
        top_n=8,
        densify_divisor=20.0,
        max_sites=32,
        propose_geom=propose_geom,
        pt_push=sheet.centroid,
    )
    assert len(poses) > 0
    assert len(poses) <= 8
    region = search_region_for_placement(
        obstacle, sheet, sheet, 0.1, use_free_region=True
    )
    region_g = propose_geom.region_geometry(region)
    assert region_g is not None
    for x, y, _th in poses:
        assert region_g.contains_point(float(x), float(y))


def test_propose_voronoi_max_sites_top_n_nonpositive():
    from nest_graph.config import ProposeConfig
    from nest_graph.propose.geometry import ProposeGeometry

    sheet = box(0, 0, 20, 20)
    obstacle = box(2, 2, 6, 6)
    part = box(0, 0, 1, 1)
    propose_geom = ProposeGeometry(
        sheet, obstacle, part, 0.1, propose_cfg=ProposeConfig()
    )
    assert (
        propose_placements_voronoi(
            obstacle,
            part,
            sheet,
            0.1,
            use_free_region=True,
            max_sites=0,
            propose_geom=propose_geom,
            pt_push=sheet.centroid,
        )
        == []
    )
    assert (
        propose_placements_voronoi(
            obstacle,
            part,
            sheet,
            0.1,
            use_free_region=True,
            top_n=0,
            propose_geom=propose_geom,
            pt_push=sheet.centroid,
        )
        == []
    )
