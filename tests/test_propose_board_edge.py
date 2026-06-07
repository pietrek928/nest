import math

import numpy as np
from shapely.geometry import Point, Polygon

from nest_graph.config import BuildGraphConfig, ProposeConfig
from nest_graph.propose import (
    ProposeGeometry,
    propose_coords_with_strategy,
    propose_placements_board_edge,
    propose_placements_sheet_edge,
)
from nest_graph.propose.placements_edge import _board_edge_snap_seeds
from nest_graph.propose.placements_guidance import propose_placements_board_edge_guidance_cast
from nest_graph.utils import normalize_poly, transform_poly


def _triangle_board() -> Polygon:
    return Polygon([(0, 0), (1.2, 0), (0, 1.1)])


def _rect_part():
    return normalize_poly(Polygon([(0, 0), (0.12, 0), (0.12, 0.08), (0, 0.08)]))


def test_board_edge_produces_tight_border_placements():
    board = _triangle_board()
    rect = _rect_part()
    cfg = BuildGraphConfig()
    min_dist = cfg.board_min_dist()
    geom = ProposeGeometry(board, Polygon(), rect, min_dist, propose_cfg=cfg.propose)
    pt_push = board.centroid

    coords = propose_placements_board_edge(
        rect,
        board,
        Polygon(),
        min_dist=min_dist,
        propose_cfg=cfg.propose,
        propose_geom=geom,
        pt_push=pt_push,
        top_n=16,
    )
    assert len(coords) > 0
    dists = [transform_poly(rect, c).distance(board.exterior) for c in coords]
    assert min(dists) < min_dist * 2.0


def test_board_edge_hybrid_tighter_than_snap_only():
    board = _triangle_board()
    rect = _rect_part()
    cfg = BuildGraphConfig()
    min_dist = cfg.board_min_dist()
    geom = ProposeGeometry(board, Polygon(), rect, min_dist, propose_cfg=cfg.propose)
    pt_push = board.centroid

    snap_only = propose_placements_board_edge(
        rect, board, Polygon(),
        min_dist=min_dist,
        propose_cfg=cfg.propose,
        propose_geom=geom,
        pt_push=pt_push,
        guidance_refine=False,
        top_n=16,
    )
    hybrid = propose_placements_board_edge(
        rect, board, Polygon(),
        min_dist=min_dist,
        propose_cfg=cfg.propose,
        propose_geom=geom,
        pt_push=pt_push,
        guidance_refine=True,
        top_n=16,
    )
    assert snap_only and hybrid
    snap_min = min(transform_poly(rect, c).distance(board.exterior) for c in snap_only)
    hybrid_min = min(transform_poly(rect, c).distance(board.exterior) for c in hybrid)
    assert hybrid_min <= snap_min * 1.05 + 1e-6


def test_board_edge_beats_sheet_edge_bbox_on_triangle():
    board = _triangle_board()
    rect = _rect_part()
    cfg = BuildGraphConfig()
    min_dist = cfg.board_min_dist()
    geom = ProposeGeometry(board, Polygon(), rect, min_dist, propose_cfg=cfg.propose)
    pt_push = board.centroid

    board_coords = propose_placements_board_edge(
        rect, board, Polygon(),
        min_dist=min_dist,
        propose_cfg=cfg.propose,
        propose_geom=geom,
        pt_push=pt_push,
        top_n=16,
    )
    sheet_coords = propose_placements_sheet_edge(
        rect, board, min_dist,
        propose_geom=geom,
        pt_push=pt_push,
        top_n=16,
    )
    assert board_coords
    board_min = min(transform_poly(rect, c).distance(board.exterior) for c in board_coords)
    if sheet_coords:
        sheet_min = min(transform_poly(rect, c).distance(board.exterior) for c in sheet_coords)
        assert board_min <= sheet_min * 1.1 + 1e-6


def test_board_edge_guidance_cast_emits_candidates():
    board = _triangle_board()
    rect = _rect_part()
    cfg = BuildGraphConfig()
    min_dist = cfg.board_min_dist()
    geom = ProposeGeometry(board, Polygon(), rect, min_dist, propose_cfg=cfg.propose)
    pt_push = board.centroid

    seeds = _board_edge_snap_seeds(
        rect, board, Polygon(),
        min_dist=min_dist,
        propose_geom=geom,
        pt_push=pt_push,
        num_angles=8,
        samples_per_edge=8,
        top_n=8,
    )
    assert seeds
    refined = propose_placements_board_edge_guidance_cast(
        seeds, pt_push, geom, cfg.propose, min_dist=min_dist, top_n=8,
    )
    assert isinstance(refined, list)


def test_propose_coords_with_board_edge_only():
    board = _triangle_board()
    rect = _rect_part()
    cfg = ProposeConfig(
        max_proposals=12,
        candidate_pool=32,
        use_board_edge_seeds=True,
        board_edge_guidance_refine=True,
        use_border_focus=True,
        use_border_edge_seeds=False,
        use_ribbon_seeds=False,
        use_voronoi=False,
        use_group_edge_seeds=False,
        use_guidance_propositions=False,
    )
    min_dist = BuildGraphConfig().board_min_dist()
    coords = propose_coords_with_strategy(
        Polygon(), rect, board, cfg,
        min_dist=min_dist, pt_push=board.centroid,
    )
    assert len(coords) >= 4


def test_default_config_enables_board_edge():
    assert BuildGraphConfig().propose.use_board_edge_seeds is True
    assert BuildGraphConfig().propose.board_edge_guidance_refine is True
