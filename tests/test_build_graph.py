import numpy as np
import pytest
from rtree.index import Index
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
from nest_graph.utils import normalize_poly, transform_poly


BOARD = Polygon([(0, 0), (1.2, 0), (0, 1.1)])
RECT = normalize_poly(Polygon([(0, 0), (0.1, 0), (0.1, 0.1), (0, 0.1)]))
TRI = normalize_poly(Polygon([(0, 0), (0.15, 0), (0, 0.07)]))


def _small_transforms(n: int, seed: int = 0) -> np.ndarray:
    rng = np.random.default_rng(seed)
    return rng.uniform(-0.3, 0.3, (n, 3)) * [1.0, 1.0, 2 * np.pi]


def _collision_edges(graph, n_nodes: int) -> set[tuple[int, int]]:
    edges = set()
    for i in range(n_nodes):
        for j in graph._obj.collisions[i]:
            a, b = min(i, j), max(i, j)
            edges.add((a, b))
    return edges


def _make_polygon_graph_shapely(b, polygons):
    idx = Index()
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
            candidates = [j for j in idx.intersection(poly_t.bounds)]
            if any(poly_t.intersects(selected_polys[j]) for j in candidates):
                continue
            n = len(selected_polys)
            for j in candidates:
                if poly_t.intersects(selected_polys[j]):
                    graph_edges.append((min(n, j), max(n, j)))
            selected_polys.append(poly_t)
            selected_group_id.append(i)
            selected_transform.append(t)
            idx.insert(n, poly_t.bounds)

    return selected_polys, selected_group_id, selected_transform, graph_edges


def test_make_polygon_graph_collision_parity():
    transforms = (
        _small_transforms(12, seed=1),
        _small_transforms(12, seed=2),
    )
    polygons = [(RECT, transforms[0]), (TRI, transforms[1])]

    graph, polys, group_id, transform = make_polygon_graph(BOARD, polygons)
    legacy_polys, legacy_gid, legacy_t, legacy_edges = _make_polygon_graph_shapely(
        BOARD, polygons
    )

    assert len(polys) == len(legacy_polys)
    assert set(map(tuple, transform)) == set(map(tuple, legacy_t))
    assert len(graph._obj.elems) == len(legacy_polys)
    assert _collision_edges(graph, len(polys)) == set(legacy_edges)


def test_make_polygon_graph_elem_graph_fields():
    transforms = (_small_transforms(8, seed=3),)
    graph, polys, _gid, _t = make_polygon_graph(BOARD, [(RECT, transforms[0])])
    board_geom = Geometry.from_shapely(BOARD)

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

    idx = Index()
    selected_geoms = []
    for _, pnum, t, placed in scored:
        bbox = placed.bounds()
        candidates = [selected_geoms[j] for j in idx.intersection(bbox)]
        if placed.intersects_any(candidates):
            continue
        n = len(selected_geoms)
        idx.insert(n, bbox)
        selected_geoms.append(placed)
        result[pnum].append(t)
    return tuple(np.array(tt) if tt else np.array([]).reshape(0, 3) for tt in result)


def test_select_polygons_from_edges_parity():
    np.random.seed(42)
    polygons = [
        (RECT, 1.0, _small_transforms(16, seed=4)),
        (TRI, 0.5, _small_transforms(16, seed=5)),
    ]
    got = select_polygons_from_edges(BOARD, polygons)
    np.random.seed(42)
    ref = _select_polygons_from_edges_reference(BOARD, polygons)
    for g, r in zip(got, ref):
        assert {tuple(x) for x in g} == {tuple(x) for x in r}
        assert len(g) == len(r)


def _make_polygon_matrix_shapely(b, polygons):
    idx = Index()
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
    for i, poly in enumerate(selected):
        for j in idx.intersection(poly.bounds):
            if poly.intersects(selected[j]):
                M[i, j] = M[j, i] = 1
        idx.insert(i, poly.bounds)
        M[i, i] = -group_weights[i]
    return M, selected


def test_make_polygon_matrix_parity():
    polygons = [
        (RECT, 1.0, _small_transforms(12, seed=6)),
        (TRI, 0.5, _small_transforms(12, seed=7)),
    ]
    M, selected = make_polygon_matrix(BOARD, polygons)
    M_ref, selected_ref = _make_polygon_matrix_shapely(BOARD, polygons)

    assert M.shape == M_ref.shape
    assert np.array_equal(M != 0, M_ref != 0)
    assert np.allclose(np.diag(M), np.diag(M_ref))
    assert len(selected) == len(selected_ref)


def test_improve_rules_region_from_board():
    region = _rule_region(BOARD)
    xmin, ymin, xmax, ymax = BOARD.bounds
    assert region.center.x == pytest.approx(0.5 * (xmin + xmax))
    assert region.center.y == pytest.approx(0.5 * (ymin + ymax))

    rules = [PlacementRuleSet()]
    improved = improve_rules([], rules, 1, BOARD)
    assert len(improved) >= 1

    assert region.radius > 0


def test_placement_board_score_positive_inside():
    board_geom = Geometry.from_shapely(BOARD)
    base = Geometry.from_convex_polygon(list(RECT.exterior.coords)[:-1])
    placed = base.apply_transform((0.5, 0.5, 0.0))
    score = placement_board_score(BOARD, board_geom, placed)
    assert score > 0
