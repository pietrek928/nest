import math

import pytest
from shapely.affinity import translate
from shapely.geometry import Point, Polygon

from nest_graph.config import ProposeConfig
from nest_graph.geometry import Geometry, GuidanceConfig, evaluate_local_placement
from nest_graph.placement_scene import guidance_config_for_propose
from nest_graph.propose import (
    ProposeGeometry,
    propose_placements_guidance_propositions,
    propose_coords_with_strategy,
)


def _unit_square():
    return Geometry.from_convex_polygon([(0, 0), (1, 0), (1, 1), (0, 1)])


def test_diversity_filter_returns_distinct_propositions():
    a = _unit_square()
    b = _unit_square().translate(0.3, 0.1)
    cfg = GuidanceConfig()
    cfg.search_radius = 2.0
    cfg.minimum_placing_distance = 0.05
    cfg.max_propositions = 5
    g = evaluate_local_placement(0, [a, b], (0.0, 0.0), cfg)
    assert len(g.propositions) >= 1
    seen: set[tuple[float, float, float]] = set()
    for prop in g.propositions:
        tx, ty = prop.translation
        key = (round(tx, 3), round(ty, 3), round(float(prop.rotation_rad), 3))
        assert key not in seen
        seen.add(key)


def test_guidance_config_scaled_for_small_board():
    board = Polygon([(0, 0), (1.2, 0), (0, 1.1)])
    cfg = guidance_config_for_propose(
        board.centroid,
        min_dist=0.01,
        board_bounds=board.bounds,
        max_propositions=5,
        use_tight_packing=True,
        enable_grid_exploration=True,
        diversity_dist_ratio=4.0,
    )
    assert cfg.diversity_distance_threshold < 1.0
    assert cfg.use_hole_seeking is True


def test_proposition_seeds_partial_pack():
    board = Polygon([(0, 0), (10, 0), (10, 10), (0, 10)])
    base = translate(Polygon([(0, 0), (1, 0), (1, 1), (0, 1)]), 3, 3)
    part = Polygon([(0, 0), (0.5, 0), (0.5, 0.5), (0, 0.5)])
    min_dist = 0.05
    cfg = ProposeConfig(
        max_proposals=8,
        candidate_pool=16,
        use_guidance_propositions=True,
        guidance_max_propositions=5,
        use_ribbon_seeds=False,
        use_voronoi=False,
    )
    geom = ProposeGeometry(board, base, part, min_dist, propose_cfg=cfg)
    seeds = [(5.0, 5.0, 0.0), (6.0, 5.0, 0.0)]
    push = base.centroid
    expanded = propose_placements_guidance_propositions(
        seeds, push, geom, cfg, min_dist=min_dist,
    )
    assert expanded
    for coords in expanded:
        placed = geom.placed_at(coords)
        assert geom.valid(placed, push, (coords[0], coords[1]))


def test_propose_with_guidance_propositions_smoke():
    board = Polygon([(0, 0), (10, 0), (10, 10), (0, 10)])
    base = translate(Polygon([(0, 0), (1, 0), (1, 1), (0, 1)]), 2, 2)
    part = Polygon([(0, 0), (0.4, 0), (0.4, 0.4), (0, 0.4)])
    cfg = ProposeConfig(
        max_proposals=6,
        candidate_pool=12,
        use_guidance_propositions=True,
        use_voronoi=False,
        use_point_cloud=False,
    )
    coords = propose_coords_with_strategy(
        base, part, board, cfg,
        min_dist=0.05,
        pt_push=Point(base.centroid),
    )
    assert coords
    geom = ProposeGeometry(board, base, part, 0.05, propose_cfg=cfg)
    for c in coords:
        placed = geom.placed_at(c)
        assert geom.valid(placed, Point(base.centroid), (c[0], c[1]))
