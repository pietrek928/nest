import time

import pytest

from nest_graph.graph import Circle, PoseGraph, Vec2


def test_append_elem_many_fast():
    graph = PoseGraph()
    t0 = time.perf_counter()
    for i in range(200):
        graph.append_elem(i % 2, Vec2(x=float(i), y=float(i)), Circle.from_center_radius(i, i, 1.0))
    elapsed = time.perf_counter() - t0
    assert len(graph.elems) == 200
    assert len(graph.group_id) == 200
    assert len(graph.coords) == 200
    assert elapsed < 0.5


def test_add_collision_pair_symmetric():
    graph = PoseGraph()
    for i in range(3):
        graph.append_elem(0, Vec2(x=0.0, y=0.0), Circle.from_center_radius(0.0, 0.0, 1.0))
    graph.add_collision(0, 1)
    graph.add_collision(1, 2)
    assert 1 in graph.collisions[0]
    assert 0 in graph.collisions[1]
    assert 2 in graph.collisions[1]
    assert 1 in graph.collisions[2]


def test_pose_graph_row_accessors():
    graph = PoseGraph()
    for i in range(3):
        graph.append_elem(0, Vec2(x=0.0, y=0.0), Circle.from_center_radius(0.0, 0.0, 1.0))
    graph.add_collision(0, 1)
    graph.add_collision(1, 2)
    graph.add_attract(0, 2, 1.5)
    assert int(graph.size()) == 3
    assert int(graph.edge_count()) == 2
    assert int(graph.degree(1)) == 2
    assert list(graph.collision_row(0)) == [1]
    assert int(graph.attract_edge_count()) == 1
    row = list(graph.attract_row(0))
    assert len(row) == 1
    assert int(row[0].target) == 2
    assert float(row[0].w) == pytest.approx(1.5)


def test_reserve_elems_then_append():
    graph = PoseGraph()
    graph.reserve_elems(4)
    for i in range(4):
        graph.append_elem(i, Vec2(x=i, y=i), Circle.from_center_radius(i, i, 0.5))
    assert len(graph.elems) == 4
