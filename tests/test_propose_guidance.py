import math

import numpy as np
import pytest
from shapely.affinity import translate
from shapely.geometry import Point, Polygon

from nest_graph.config import ProposeConfig
from nest_graph.propose import (
    ProposeGeometry,
    evaluate_ray_placement,
    proposed_transforms_for_groups,
)
from nest_graph.placement_scene import (
    board_placement_valid,
    best_proposition,
    proposition_translation,
)
from nest_graph.geometry import Geometry


def _unit_square_poly():
    return Polygon([(0, 0), (1, 0), (1, 1), (0, 1)])


@pytest.fixture
def board():
    return Polygon([(0, 0), (10, 0), (10, 10), (0, 10)])


@pytest.fixture
def propose_geom(board):
    return ProposeGeometry(
        board,
        translate(_unit_square_poly(), 3, 0),
        _unit_square_poly(),
        min_dist=0.05,
        propose_cfg=ProposeConfig(),
    )


def test_placement_guidance_separated(board, propose_geom):
    placed = propose_geom.placed_at((0.5, 0.5, 0.0))
    g = propose_geom.placement_guidance(placed, (0.5, 0.5), board.centroid)
    assert g.is_penetrating is False
    assert float(g.clearance) < float("inf")
    assert float(g.clearance) > 0.0
    assert len(g.propositions) >= 1


def test_placement_guidance_overlap(board):
    base = translate(_unit_square_poly(), 3, 0)
    geom = ProposeGeometry(
        board, base, _unit_square_poly(), min_dist=0.05, propose_cfg=ProposeConfig(),
    )
    placed = geom.placed_at((3.3, 0.1, 0.0))
    push = board.centroid
    g = geom.placement_guidance(placed, (3.3, 0.1), push)
    assert g.is_penetrating is True
    prop = best_proposition(g)
    tx, ty = proposition_translation(prop)
    assert math.hypot(tx, ty) > 1e-9


def test_attraction_unit_matches_ejection(board):
    base = translate(_unit_square_poly(), 3, 0)
    geom = ProposeGeometry(
        board, base, _unit_square_poly(), min_dist=0.05, propose_cfg=ProposeConfig(),
    )
    placed = geom.placed_at((3.3, 0.1, 0.0))
    push = board.centroid
    g = geom.placement_guidance(placed, (3.3, 0.1), push)
    attr = geom.attraction_unit(placed, push, (3.3, 0.1))
    prop = best_proposition(g)
    tx, ty = proposition_translation(prop)
    eject_u = np.array([tx, ty], dtype=np.float64)
    eject_u = eject_u / (np.linalg.norm(eject_u) + 1e-12)
    assert np.dot(attr, eject_u) > 0.99


def test_evaluate_ray_placement_nudge_moves_part(board):
    """Guidance-driven nudge should change pose; empty base yields a valid scored placement."""
    part = _unit_square_poly()
    geom = ProposeGeometry(
        board, Polygon(), part, min_dist=0.05, propose_cfg=ProposeConfig(),
    )
    push = board.centroid
    params = np.array([1.0, 1.0, 0.0, 0.0])
    score, settled = evaluate_ray_placement(
        params,
        Polygon(),
        part,
        board,
        0.0,
        board.centroid,
        push,
        0.05,
        nudge_iters=4,
        pull_factor=0.08,
        propose_geom=geom,
        ray_dirs=8,
    )
    assert score < 1e6
    assert settled[0] != params[0] or settled[1] != params[1]
    placed = geom.placed_at(tuple(settled))
    assert geom.valid(placed, push, (settled[0], settled[1]))


def test_proposed_transforms_board_valid(nest_board, rect_poly, tri_poly, build_graph_config):
    cfg = build_graph_config
    proposals = proposed_transforms_for_groups(
        nest_board,
        [(rect_poly, 0), (tri_poly, 1)],
        [],
        [],
        cfg.propose,
        min_dist=cfg.board_min_dist(),
        pt_push=Point(nest_board.centroid),
    )
    for group_id, part in ((0, rect_poly), (1, tri_poly)):
        arr = proposals[group_id]
        assert arr.ndim == 2 and arr.shape[1] == 3
        part_geom = Geometry.from_shapely(part)
        for t in arr:
            placed = part_geom.apply_transform(t)
            assert board_placement_valid(nest_board, part_geom, placed)


def test_propose_geometry_validation(board):
    geom = ProposeGeometry(
        board, Polygon(), _unit_square_poly(), min_dist=0.001, propose_cfg=ProposeConfig(),
    )
    placed = geom.placed_at((0.5, 0.5, 0.0))
    push = board.centroid
    assert geom.footprint_clear_of_voids(placed)
    assert not geom.hits_base(placed)
