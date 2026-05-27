"""Board sheet conversion and unified placement validity."""

import numpy as np
import pytest
from shapely.geometry import Point, Polygon

from nest_graph.board import board_sheet_from_outline, board_void_geometries
from nest_graph.build_graph import make_polygon_graph
from nest_graph.config import RulesConfig
from nest_graph.geometry import Geometry, GuidanceConfig
from nest_graph.placement_scene import (
    PlacementScene,
    board_placement_valid,
    build_placement_scene,
    is_valid_placement,
)
from nest_graph.utils import transform_poly


def test_board_sheet_equivalent_to_outline(nest_board, rect_poly, small_transforms):
    """Triangle outline validity matches sheet + void obstacle model."""
    sheet = board_sheet_from_outline(nest_board)
    void_geoms = board_void_geometries(sheet)
    part = Geometry.from_shapely(rect_poly)
    guidance_cfg = GuidanceConfig()
    transforms = small_transforms(24, seed=7)

    for t in transforms:
        placed = part.apply_transform(t)
        legacy = nest_board.contains(transform_poly(rect_poly, t))
        scene = PlacementScene(sheet, void_geoms, [], part)
        cx, cy = placed.center()
        unified = is_valid_placement(scene, placed, (cx, cy), 0.0, guidance_cfg)
        assert unified == legacy


def test_void_penetrating_in_guidance(nest_board):
    sheet = board_sheet_from_outline(nest_board)
    void_geoms = board_void_geometries(sheet)
    assert void_geoms, "triangle sheet should have corner voids"

    part = Geometry.from_convex_polygon([(0, 0), (0.05, 0), (0.05, 0.05), (0, 0.05)])
    placed = part.apply_transform((1.0, 0.5, 0.0))
    assert not nest_board.contains(transform_poly(
        Polygon([(0, 0), (0.05, 0), (0.05, 0.05), (0, 0.05)]), (1.0, 0.5, 0.0)
    ))
    scene = PlacementScene(sheet, void_geoms, [], part)
    g = scene.guidance(placed, placed.center(), GuidanceConfig())
    assert g.is_penetrating is True


def test_rules_config_board_sheet_polygon():
    rules = RulesConfig()
    outline = rules.board_polygon()
    sheet = rules.board_sheet_polygon()
    assert sheet.area >= outline.area
    assert len(sheet.interiors) >= 1


def test_user_board_holes():
    outline = Polygon([(0, 0), (10, 0), (10, 10), (0, 10)])
    keepout = [(4, 4), (6, 4), (6, 6), (4, 6)]
    part = Geometry.from_convex_polygon([(0, 0), (0.5, 0), (0.5, 0.5), (0, 0.5)])
    placed = part.apply_transform((5.0, 5.0, 0.0))
    scene = build_placement_scene(outline, part, user_holes=(keepout,))
    cx, cy = placed.center()
    assert not is_valid_placement(scene, placed, (cx, cy), 0.0, GuidanceConfig())


def test_proposals_respect_board_min_dist(nest_board, rect_poly, build_graph_config):
    from nest_graph.config import ProposeConfig
    from nest_graph.placement_scene import guidance_config_for_scene, is_valid_placement
    from nest_graph.propose import proposed_transforms_for_groups

    cfg = build_graph_config
    min_dist = cfg.board_min_dist()
    eps_ratio = cfg.propose.placement_clearance_epsilon_ratio
    scene_cfg = guidance_config_for_scene(
        min_dist, board_bounds=nest_board.bounds, epsilon_ratio=eps_ratio
    )
    proposals = proposed_transforms_for_groups(
        nest_board,
        [(rect_poly, 0)],
        [],
        [],
        ProposeConfig(max_proposals=4, candidate_pool=4, use_point_cloud=False),
        min_dist=min_dist,
    )
    part = Geometry.from_shapely(rect_poly)
    from nest_graph.placement_scene import build_placement_scene

    scene = build_placement_scene(nest_board, part)
    margin = min_dist + min_dist * eps_ratio
    for t in proposals[0]:
        placed = part.apply_transform(t)
        g = scene.guidance(placed, placed.center(), scene_cfg)
        assert not g.is_penetrating
        if min_dist > 0:
            assert float(g.clearance) >= margin - 1e-9


def test_donut_board_rejects_hole_overlap(nest_board_donut, rect_poly):
    part = Geometry.from_shapely(rect_poly)
    placed = part.apply_transform((5.0, 5.0, 0.0))
    assert not board_placement_valid(nest_board_donut, part, placed)

    graph, polys, _, _ = make_polygon_graph(
        nest_board_donut,
        [(rect_poly, np.array([[5.0, 5.0, 0.0]]))],
    )
    assert len(polys) == 0
