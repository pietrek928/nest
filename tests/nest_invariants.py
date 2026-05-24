"""Shared assertions for nest graph selection invariants."""

from typing import Sequence

import numpy as np
from shapely.geometry.base import BaseGeometry

from nest_graph.build_graph import Geometry, find_polygon_intersections_bipartite
from nest_graph.elem_graph import ElemGraph
from nest_graph.utils import transform_poly


def assert_selected_graph_independent(graph: ElemGraph, selected: Sequence[int]) -> None:
    sel = set(selected)
    n = len(graph.group_id)
    for v in sel:
        assert 0 <= v < n
        for u in graph.collisions[v]:
            assert u not in sel or u == v


def assert_selected_shapely_disjoint(polys: Sequence[BaseGeometry], selected: Sequence[int]) -> None:
    for a, i in enumerate(selected):
        for j in selected[a + 1 :]:
            assert not polys[i].intersects(polys[j]), f"Shapely overlap: {i}, {j}"


def assert_selected_geometry_disjoint(
    node_geoms: Sequence[Geometry],
    transforms: Sequence[np.ndarray],
    selected: Sequence[int],
) -> None:
    placed = [node_geoms[i].apply_transform(transforms[i]) for i in selected]
    for a in range(len(placed)):
        for b in range(a + 1, len(placed)):
            hits = find_polygon_intersections_bipartite([placed[a]], [placed[b]])
            assert not hits, f"Geometry overlap: selected[{a}], selected[{b}]"


def assert_selected_non_overlapping(
    graph: ElemGraph,
    polys: Sequence[BaseGeometry],
    selected: Sequence[int],
    *,
    bases: Sequence[Geometry] | None = None,
    transforms: Sequence[np.ndarray] | None = None,
) -> None:
    assert_selected_graph_independent(graph, selected)
    assert_selected_shapely_disjoint(polys, selected)
    if bases is not None and transforms is not None:
        assert_selected_geometry_disjoint(bases, transforms, selected)
