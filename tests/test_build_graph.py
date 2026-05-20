import os
import subprocess
import sys

import numpy as np
import pytest
from shapely.geometry import Point, Polygon

from nest_graph.build_graph import (
    Geometry,
    _base_geometries,
    _poly_and_transforms,
    _rule_region,
    improve_rules,
    make_polygon_graph,
    make_polygon_matrix,
    placement_board_score,
    select_polygons_from_edges,
)
from nest_graph.elem_graph import PlacementRuleSet
from nest_graph.utils import transform_poly




def _collision_edges(graph, n_nodes: int) -> set[tuple[int, int]]:
    edges = set()
    for i in range(n_nodes):
        for j in graph._obj.collisions[i]:
            a, b = min(i, j), max(i, j)
            edges.add((a, b))
    return edges


def _make_polygon_graph_shapely(b, polygons):
    selected_polys = []
    selected_group_id = []
    selected_transform = []
    graph_edges = []

    for i, item in enumerate(polygons):
        if len(item) == 2:
            p, transforms = item
        else:
            p, _w, transforms = item
        for t in transforms:
            poly_t = transform_poly(p, t)
            if not b.contains(poly_t):
                continue
            n = len(selected_polys)
            for j in range(n):
                if poly_t.intersects(selected_polys[j]):
                    graph_edges.append((min(n, j), max(n, j)))
            selected_polys.append(poly_t)
            selected_group_id.append(i)
            selected_transform.append(t)

    return selected_polys, selected_group_id, selected_transform, graph_edges


def test_make_polygon_graph_no_hidden_overlaps(nest_board, rect_poly, tri_poly, small_transforms):
    """Accepted placements must record collisions for every Shapely overlap."""
    transforms = (
        small_transforms(24, seed=11),
        small_transforms(24, seed=12),
    )
    graph, polys, _gid, _trans = make_polygon_graph(
        nest_board, [(rect_poly, transforms[0]), (tri_poly, transforms[1])]
    )
    for i in range(len(polys)):
        for j in range(i + 1, len(polys)):
            if polys[i].intersects(polys[j]):
                assert j in graph._obj.collisions[i]
                assert i in graph._obj.collisions[j]


def test_make_polygon_graph_accepts_disjoint_concave_pair(
    nest_board_large, l_shape_poly, concave_placement_transforms
):
    """Regression: inflated from_convex_polygon solids rejected valid L+L placements."""
    t1, t2 = concave_placement_transforms
    transforms = np.stack([t1, t2])
    graph, polys, _gid, _trans = make_polygon_graph(
        nest_board_large, [(l_shape_poly, transforms)]
    )
    assert len(polys) == 2
    assert not polys[0].intersects(polys[1])
    assert len(graph._obj.collisions[0]) == 0


def test_make_polygon_graph_main_iteration_scale(nest_board, rect_poly, tri_poly):
    """First main()-style iteration should build a dense graph, not a tiny independent set."""
    rng = np.random.default_rng(0)
    history = np.zeros((1, 3))
    s0 = np.concatenate([
        rng.uniform(-0.35, 0.35, (256, 3)) * [1.5, 1.5, 2 * np.pi],
        history,
    ])
    s1 = np.concatenate([
        rng.uniform(-0.35, 0.35, (256, 3)) * [1.5, 1.5, 2 * np.pi],
        history,
    ])
    graph, polys, _gid, _trans = make_polygon_graph(
        nest_board, [(rect_poly, s0), (tri_poly, s1)]
    )
    edges = sum(len(graph._obj.collisions[i]) for i in range(len(polys))) // 2
    assert len(polys) > 50
    assert edges > 100


def test_make_polygon_graph_keeps_overlapping_placements(nest_board, rect_poly, small_transforms):
    """Graph must list board-valid transforms even when they overlap (collision edges, not drop)."""
    transforms = small_transforms(32, seed=20)
    graph, polys, _gid, _trans = make_polygon_graph(nest_board, [(rect_poly, transforms)])
    legacy_polys, _, _, legacy_edges = _make_polygon_graph_shapely(
        nest_board, [(rect_poly, transforms)]
    )
    assert len(polys) == len(legacy_polys)
    assert len(polys) >= 1
    assert len(legacy_edges) > 0
    assert sum(len(graph._obj.collisions[i]) for i in range(len(polys))) > 0


def test_make_polygon_graph_collision_parity(nest_board, rect_poly, tri_poly, small_transforms):
    transforms = (
        small_transforms(12, seed=1),
        small_transforms(12, seed=2),
    )
    polygons = [(rect_poly, transforms[0]), (tri_poly, transforms[1])]

    graph, polys, group_id, transform = make_polygon_graph(nest_board, polygons)
    legacy_polys, legacy_gid, legacy_t, legacy_edges = _make_polygon_graph_shapely(
        nest_board, polygons
    )

    assert len(polys) == len(legacy_polys)
    assert set(map(tuple, transform)) == set(map(tuple, legacy_t))
    assert len(graph._obj.elems) == len(legacy_polys)
    assert _collision_edges(graph, len(polys)) == set(legacy_edges)


def test_make_polygon_graph_elem_graph_fields(nest_board, rect_poly, small_transforms):
    transforms = (small_transforms(8, seed=3),)
    graph, polys, _gid, _t = make_polygon_graph(nest_board, [(rect_poly, transforms[0])])
    board_geom = Geometry.from_shapely(nest_board)

    for i, ep in enumerate(graph._obj.elems):
        circle = graph._obj.coords[i]
        assert circle.r_sq > 0
        assert board_geom.contains_point(ep.pos.x, ep.pos.y)
    assert len(polys) == len(graph._obj.elems)


def _select_polygons_from_edges_reference(b, polygons):
    """Mirror of select_polygons_from_edges for parity checks."""
    board_geom = Geometry.from_shapely(b)
    result = [[] for _ in range(len(polygons))]
    scored = []
    bases = _base_geometries(polygons)
    for i, item in enumerate(polygons):
        _p, transforms, w = _poly_and_transforms(item)
        base = bases[i]
        for t in transforms:
            placed = base.apply_transform(t)
            score = placement_board_score(b, board_geom, placed)
            if score > 0:
                scored.append((
                    placed.center()[0] + w + np.random.rand() * 1e-4,
                    i,
                    t,
                    placed,
                ))
    scored.sort(key=lambda x: x[0])

    selected_geoms = []
    for _, pnum, t, placed in scored:
        if placed.intersects_any(selected_geoms):
            continue
        selected_geoms.append(placed)
        result[pnum].append(t)
    return tuple(np.array(tt) if tt else np.array([]).reshape(0, 3) for tt in result)


def test_select_polygons_from_edges_parity(nest_board, rect_poly, tri_poly, small_transforms):
    np.random.seed(42)
    polygons = [
        (rect_poly, 1.0, small_transforms(16, seed=4)),
        (tri_poly, 0.5, small_transforms(16, seed=5)),
    ]
    got = select_polygons_from_edges(nest_board, polygons)
    np.random.seed(42)
    ref = _select_polygons_from_edges_reference(nest_board, polygons)
    for g, r in zip(got, ref):
        assert {tuple(x) for x in g} == {tuple(x) for x in r}
        assert len(g) == len(r)


def _make_polygon_matrix_shapely(b, polygons):
    selected = []
    group_weights = []
    for item in polygons:
        if len(item) == 2:
            p, transforms = item
            w = 1.0
        else:
            p, w, transforms = item
        for t in transforms:
            poly_t = transform_poly(p, t)
            if b.contains(poly_t):
                selected.append(poly_t)
                group_weights.append(w)

    n = len(selected)
    M = np.zeros((n, n), dtype=np.float32)
    for i in range(n):
        for j in range(i + 1, n):
            if selected[i].intersects(selected[j]):
                M[i, j] = M[j, i] = 1
        M[i, i] = -group_weights[i]
    return M, selected


def test_make_polygon_matrix_parity(nest_board, rect_poly, tri_poly, small_transforms):
    polygons = [
        (rect_poly, 1.0, small_transforms(12, seed=6)),
        (tri_poly, 0.5, small_transforms(12, seed=7)),
    ]
    M, selected = make_polygon_matrix(nest_board, polygons)
    M_ref, selected_ref = _make_polygon_matrix_shapely(nest_board, polygons)

    assert M.shape == M_ref.shape
    assert np.array_equal(M != 0, M_ref != 0)
    assert np.allclose(np.diag(M), np.diag(M_ref))
    assert len(selected) == len(selected_ref)


def test_build_graph_module_smoke():
    """Import must not segfault (regression for extension load / import side effects)."""
    proc = subprocess.run(
        [sys.executable, '-c', 'import nest_graph.build_graph'],
        cwd=os.path.dirname(os.path.dirname(__file__)),
        capture_output=True,
        timeout=30,
    )
    assert proc.returncode == 0, proc.stderr.decode()[-2000:]


def test_improve_rules_region_from_board(nest_board):
    region = _rule_region(nest_board)
    xmin, ymin, xmax, ymax = nest_board.bounds
    assert region.center.x == pytest.approx(0.5 * (xmin + xmax))
    assert region.center.y == pytest.approx(0.5 * (ymin + ymax))

    rules = [PlacementRuleSet()]
    improved = improve_rules([], rules, 1, nest_board)
    assert len(improved) >= 1

    assert region.radius > 0


def test_placement_board_score_positive_inside(nest_board, rect_poly):
    board_geom = Geometry.from_shapely(nest_board)
    base = Geometry.from_shapely(rect_poly)
    placed = base.apply_transform((0.5, 0.5, 0.0))
    score = placement_board_score(nest_board, board_geom, placed)
    assert score > 0
