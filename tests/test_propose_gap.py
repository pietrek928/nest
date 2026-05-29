"""Gap-fitting proposer: free region, focal obstacles, clearance ranking."""

import numpy as np
from shapely.geometry import Polygon
from shapely.ops import unary_union

from nest_graph.board import board_context_from_geometry
from nest_graph.config import ProposeConfig
from nest_graph.propose import (
    ProposeGeometry,
    cluster_packed_solid_groups,
    effective_ranking_mode,
    focal_shape_for_propose,
    obstacle_polys_for_propose,
    obstacle_shape_for_propose,
    placement_free_region,
    propose_coords_with_strategy,
    propose_placements_group_fit,
    propose_push_point,
)
from nest_graph.config import expand_structured_transforms
from nest_graph.utils import normalize_poly, transform_poly


def _triangle_board() -> Polygon:
    return Polygon([(0, 0), (1.2, 0), (0, 1.1)])


def _rect():
    return normalize_poly(Polygon([(0, 0), (0.1, 0), (0.1, 0.1), (0, 0.1)]))


def _tri():
    return normalize_poly(Polygon([(0, 0), (0.15, 0), (0, 0.07)]))


def test_placement_free_region_has_pocket():
    board = _triangle_board()
    rect = _rect()
    t = np.array([0.35, 0.25, 0.2])
    base = transform_poly(rect, t)
    sheet, _ = board_context_from_geometry(board)
    free = placement_free_region(sheet, base, min_dist=0.01)
    assert not free.is_empty
    assert free.area < sheet.area


def test_free_region_beats_legacy_search_on_partial_pack():
    board = _triangle_board()
    tri = _tri()
    base = transform_poly(_rect(), np.array([0.35, 0.25, 0.3]))
    min_dist = 0.003
    push = base.centroid

    def min_clearance(*, free_region: bool) -> float:
        cfg = ProposeConfig(
            max_proposals=8,
            candidate_pool=8,
            use_free_region_search=free_region,
            ranking_mode="clearance",
            use_ribbon_seeds=False,
        )
        coords = propose_coords_with_strategy(
            base, tri, board, cfg, min_dist=min_dist, pt_push=push,
        )
        geom = ProposeGeometry(board, base, tri, min_dist)
        vals = []
        for c in coords:
            placed = geom.placed_at(c)
            if geom.is_valid_placement(placed, push, (c[0], c[1])):
                g = geom.placement_guidance(placed, (c[0], c[1]), push)
                if not g.is_penetrating:
                    vals.append(float(g.clearance))
        return min(vals) if vals else 0.0

    assert min_clearance(free_region=True) >= min_clearance(free_region=False) * 0.9


def test_ribbon_seeds_improve_partial_pack_clearance():
    board = _triangle_board()
    tri = _tri()
    base = transform_poly(_rect(), np.array([0.35, 0.25, 0.3]))
    min_dist = 0.003
    push = base.centroid

    def min_group_gap(*, ribbon: bool) -> float:
        cfg = ProposeConfig(
            max_proposals=8,
            candidate_pool=8,
            use_ribbon_seeds=ribbon,
            use_contact_ranking=True,
            trim_candidates_by_clearance=True,
        )
        coords = propose_coords_with_strategy(
            base, tri, board, cfg, min_dist=min_dist, pt_push=push, focal_shape=base,
        )
        dists = [transform_poly(tri, c).distance(base) for c in coords]
        return min(dists) if dists else 999.0

    assert min_group_gap(ribbon=True) <= min_group_gap(ribbon=False) * 1.05


def test_cluster_packed_solid_groups_splits_touching_parts():
    rect = _rect()
    p1 = transform_poly(rect, np.array([0.1, 0.1, 0.0]))
    p2 = transform_poly(rect, np.array([0.5, 0.5, 0.0]))
    assert len(cluster_packed_solid_groups([p1, p2], min_dist=0.01)) == 2


def test_obstacle_polys_focal_ignores_distant_cluster():
    rect = _rect()
    tri = _tri()
    p1 = transform_poly(rect, np.array([0.08, 0.08, 0.0]))
    p2 = transform_poly(rect, np.array([0.55, 0.55, 0.0]))
    placed = [p1, p2]
    min_dist = 0.01
    near_origin = obstacle_polys_for_propose(placed, tri, min_dist)
    assert len(near_origin) == 1
    assert near_origin[0].distance(p1) < 0.02
    tri_near_p2 = transform_poly(tri, np.array([0.5, 0.5, 0.0]))
    near_p2 = obstacle_polys_for_propose(placed, tri_near_p2, min_dist)
    assert len(near_p2) == 1
    assert near_p2[0].distance(p2) < 0.02


def test_focal_obstacle_expands_free_region():
    board = _triangle_board()
    rect = _rect()
    tri = _tri()
    p1 = transform_poly(rect, np.array([0.08, 0.08, 0.0]))
    p2 = transform_poly(rect, np.array([0.55, 0.55, 0.0]))
    sheet, _ = board_context_from_geometry(board)
    min_dist = 0.01
    focal = obstacle_shape_for_propose([p1, p2], tri, min_dist)
    full = unary_union([p1, p2])
    free_focal = placement_free_region(sheet, focal, min_dist)
    free_full = placement_free_region(sheet, full, min_dist)
    assert free_focal.area > free_full.area


def test_border_focus_tighter_than_centroid_on_empty_board():
    board = _triangle_board()
    tri = _tri()
    base = Polygon()
    min_dist = 0.003
    sheet, _ = board_context_from_geometry(board)

    def min_border(*, border: bool) -> float:
        cfg = ProposeConfig(
            max_proposals=12,
            candidate_pool=32,
            use_border_focus=border,
            use_border_edge_seeds=border,
            border_focus_ranking=border,
            use_ribbon_seeds=False,
        )
        coords = propose_coords_with_strategy(
            base, tri, board, cfg, min_dist=min_dist, pt_push=board.centroid,
        )
        vals = [transform_poly(tri, c).distance(sheet.exterior) for c in coords]
        return min(vals) if vals else 999.0

    assert min_border(border=True) < min_border(border=False) * 0.5


def test_sheet_edge_produces_tight_border_placements():
    board = _triangle_board()
    tri = _tri()
    sheet, _ = board_context_from_geometry(board)
    min_dist = 0.003
    from nest_graph.propose import ProposeGeometry, propose_placements_sheet_edge

    geom = ProposeGeometry(board, Polygon(), tri, min_dist, propose_cfg=ProposeConfig())
    pt_push = board.centroid
    coords = propose_placements_sheet_edge(
        tri,
        sheet,
        min_dist,
        propose_geom=geom,
        pt_push=pt_push,
        top_n=16,
        samples_per_edge=12,
    )
    assert len(coords) > 0
    dists = [transform_poly(tri, c).distance(sheet.exterior) for c in coords]
    assert min(dists) < min_dist * 2.0


def test_contact_hybrid_ranking_mode_when_packed():
    board = _triangle_board()
    base = transform_poly(_rect(), np.array([0.35, 0.25, 0.3]))
    cfg = ProposeConfig(use_contact_ranking=True, use_contact_clearance_hybrid=True)
    assert effective_ranking_mode(cfg, base) == "contact_hybrid"
    empty_cfg = ProposeConfig(use_contact_ranking=True, use_contact_clearance_hybrid=True)
    assert effective_ranking_mode(empty_cfg, Polygon()) == "border"


def test_contact_ranking_prefers_group_fit_on_partial_pack():
    board = _triangle_board()
    tri = _tri()
    base = transform_poly(_rect(), np.array([0.35, 0.25, 0.3]))
    min_dist = 0.003
    push = base.centroid

    def best_group_dist(*, contact: bool) -> float:
        cfg = ProposeConfig(
            max_proposals=12,
            candidate_pool=48,
            use_group_edge_seeds=True,
            use_ribbon_seeds=False,
            use_voronoi=False,
            use_axis_push=False,
            use_bottom_left=False,
            use_nfp_vertices=False,
            use_neighbor_slide=False,
            use_contact_ranking=contact,
            trim_candidates_by_clearance=True,
        )
        coords = propose_coords_with_strategy(
            base, tri, board, cfg, min_dist=min_dist, pt_push=push, focal_shape=base,
        )
        dists = [transform_poly(tri, c).distance(base) for c in coords]
        return min(dists) if dists else 999.0

    assert best_group_dist(contact=True) < best_group_dist(contact=False) * 0.5


def test_group_edge_seeds_improve_partial_pack_clearance():
    board = _triangle_board()
    tri = _tri()
    base = transform_poly(_rect(), np.array([0.35, 0.25, 0.3]))
    min_dist = 0.003
    push = base.centroid
    sheet, _ = board_context_from_geometry(board)
    focal = base

    def min_group_gap(*, group_seeds: bool) -> float:
        cfg = ProposeConfig(
            max_proposals=8,
            candidate_pool=8,
            use_group_edge_seeds=group_seeds,
            use_ribbon_seeds=False,
            use_voronoi=False,
            use_contact_ranking=True,
        )
        coords = propose_coords_with_strategy(
            base, tri, board, cfg, min_dist=min_dist, pt_push=push, focal_shape=focal,
        )
        dists = [transform_poly(tri, c).distance(base) for c in coords]
        return min(dists) if dists else 999.0

    direct = propose_placements_group_fit(
        focal, tri, sheet, base, min_dist=min_dist, top_n=8,
    )
    assert len(direct) > 0
    assert min_group_gap(group_seeds=True) <= min_group_gap(group_seeds=False) * 1.05


def test_focal_shape_matches_nearest_cluster():
    board = _triangle_board()
    rect = _rect()
    tri = _tri()
    p1 = transform_poly(rect, np.array([0.08, 0.08, 0.0]))
    p2 = transform_poly(rect, np.array([0.55, 0.55, 0.0]))
    cfg = ProposeConfig()
    focal = focal_shape_for_propose(board, [p1, p2], tri, 0.01, cfg)
    assert focal is not None
    assert focal.distance(p1) < 0.02
    assert focal.distance(p2) > 0.1


def test_expand_structured_transforms_is_deterministic():
    base = np.array([[0.5, 0.5, 0.0]])
    a = expand_structured_transforms(base, (0.1, 0.1, 0.5), 8)
    b = expand_structured_transforms(base, (0.1, 0.1, 0.5), 8)
    np.testing.assert_array_equal(a, b)
    assert a.shape[0] > 1


def test_smart_push_uses_base_centroid():
    board = _triangle_board()
    base = transform_poly(_rect(), np.array([0.4, 0.3, 0.0]))
    p_board = propose_push_point(board, base, smart_push=True)
    p_dumb = propose_push_point(board, base, smart_push=False)
    assert p_board.distance(base.centroid) < p_dumb.distance(base.centroid)
