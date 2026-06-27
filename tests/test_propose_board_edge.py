import math

import numpy as np
from shapely.geometry import LineString, Point, Polygon

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
    from nest_graph.propose.context import _placement_contact_error

    assert board_coords
    board_err = min(
        _placement_contact_error(transform_poly(rect, c), board, min_dist, None)
        for c in board_coords
    )
    if sheet_coords:
        sheet_err = min(
            _placement_contact_error(transform_poly(rect, c), board, min_dist, None)
            for c in sheet_coords
        )
        assert board_err <= sheet_err * 1.1 + 1e-6


def _edge_label_for_anchor(board: Polygon, anchor: Point) -> str:
    coords = list(board.exterior.coords)
    best_i = 0
    best_dist = float("inf")
    for i in range(len(coords) - 1):
        seg = LineString([coords[i], coords[i + 1]])
        dist = seg.distance(anchor)
        if dist < best_dist:
            best_dist = dist
            best_i = i
    return ("bottom", "hypotenuse", "left")[best_i]


def test_board_edge_snap_seeds_cover_all_triangle_edges():
    """Stratified seed pick must reserve slots on the long hypotenuse."""
    board = _triangle_board()
    rect = _rect_part()
    cfg = BuildGraphConfig()
    min_dist = cfg.board_min_dist(first_pass=True)
    propose_cfg = cfg.first_pass_propose_config()
    geom = ProposeGeometry(board, Polygon(), rect, min_dist, propose_cfg=propose_cfg)
    seeds = _board_edge_snap_seeds(
        rect,
        board,
        Polygon(),
        min_dist=min_dist,
        propose_geom=geom,
        pt_push=board.centroid,
        num_angles=propose_cfg.first_pass_num_angles,
        samples_per_edge=propose_cfg.board_edge_samples_per_edge,
        top_n=propose_cfg.first_pass_max_proposals,
    )
    by_edge = {
        _edge_label_for_anchor(board, anchor)
        for _, anchor, _ in seeds
    }
    assert "hypotenuse" in by_edge
    assert len(seeds) >= propose_cfg.first_pass_max_proposals // 3


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


def test_snap_coords_cpp_matches_shapely():
    from nest_graph.propose.placement_common import _snap_coords_along_exterior

    board = _triangle_board()
    rect = _rect_part()
    cfg = BuildGraphConfig()
    min_dist = cfg.board_min_dist()
    geom = ProposeGeometry(board, Polygon(), rect, min_dist, propose_cfg=cfg.propose)
    anchors = [
        Point(0.3, 0.0),
        Point(0.0, 0.4),
        Point(0.55, 0.55),
    ]
    angles = [0.0, math.pi / 4, math.pi / 2]
    from nest_graph.propose.placement_common import _inward_at_contact

    for contact in anchors:
        for angle in angles:
            snap_contact, inward = _inward_at_contact(board, contact)
            shapely_coords = _snap_coords_along_exterior(
                rect,
                board,
                snap_contact,
                inward,
                angle,
                min_dist,
                container=board,
                propose_geom=None,
            )
            cpp_coords = _snap_coords_along_exterior(
                rect,
                board,
                snap_contact,
                inward,
                angle,
                min_dist,
                container=board,
                propose_geom=geom,
            )
            if shapely_coords is None:
                assert cpp_coords is None
                continue
            assert cpp_coords is not None
            for i in range(3):
                assert math.isclose(shapely_coords[i], cpp_coords[i], abs_tol=0.07)


def test_snap_coords_multipolygon_focal_uses_cpp():
    from shapely.geometry import MultiPolygon

    from nest_graph.propose.placement_common import (
        _inward_at_contact,
        _outline_ring_geom,
        _snap_coords_along_exterior,
    )

    board = Polygon([(0, 0), (10, 0), (10, 10), (0, 10)])
    rect = _rect_part()
    cfg = BuildGraphConfig()
    min_dist = cfg.board_min_dist()
    geom = ProposeGeometry(board, Polygon(), rect, min_dist, propose_cfg=cfg.propose)
    placed_a = Polygon([(1, 1), (2, 1), (2, 2), (1, 2)])
    placed_b = Polygon([(4, 1), (5, 1), (5, 2), (4, 2)])
    focal = MultiPolygon([placed_a, placed_b])
    focal_ring = _outline_ring_geom(focal)
    assert focal_ring is not None
    contact = Point(2.5, 1.0)
    snap_contact, inward = _inward_at_contact(focal, contact)
    inward = (-inward[0], -inward[1])
    coords = _snap_coords_along_exterior(
        rect,
        focal,
        snap_contact,
        inward,
        0.0,
        min_dist,
        container=board,
        propose_geom=geom,
        boundary_ring_geom=focal_ring,
    )
    assert coords is not None
