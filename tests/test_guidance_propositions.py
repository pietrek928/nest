import math

import pytest
from shapely.affinity import translate
from shapely.geometry import Point, Polygon

from nest_graph.config import ProposeConfig
from nest_graph.geometry import Geometry, GuidanceConfig, evaluate_local_placement
from nest_graph.placement_scene import (
    guidance_config_for_propose,
    guidance_config_for_scene,
)
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


def test_border_focus_scene_config_does_not_force_sw():
    board = Polygon([(0, 0), (1.2, 0), (0, 1.1)])
    cfg = guidance_config_for_scene(
        0.01,
        pt_push=board.centroid,
        board_bounds=board.bounds,
        for_propose=True,
        border_focus=True,
    )
    assert cfg.use_gravity is True
    # Unset in factory; C++ default may still be SW until per-seed apply.


def test_placement_guidance_rim_gravity_follows_nearest_edge():
    board = Polygon([(0, 0), (4, 0), (0, 3)])
    part = Polygon([(0, 0), (0.2, 0), (0.2, 0.2), (0, 0.2)])
    geom = ProposeGeometry(
        board, Polygon(), part, 0.02, border_focus=True,
    )
    # Near the hypotenuse (4,0)–(0,3): inward is into the triangle, gravity toward outline.
    placed = geom.placed_at((1.4, 0.9, 0.0))
    geom.placement_guidance(placed, (1.4, 0.9), board.centroid, border_focus=True)
    gx, gy = geom._propose_guidance_cfg(board.centroid, border_focus=True).gravity_vector
    # Mutated on the cached cfg by placement_guidance.
    assert abs(gx + 1.0) > 0.05 or abs(gy + 1.0) > 0.05


def test_tight_pass_rim_band_uses_rim_gravity_not_pole():
    """Rim seeds must not pole-walk into a hole; interior seeds keep pole unit."""
    from nest_graph.propose.context import part_extents
    from nest_graph.propose.placement_perimeter import edge_inward_at_point
    from nest_graph.propose.placements_guidance import _merged_guidance_propositions

    board = Polygon([(0, 0), (4, 0), (0, 3)])
    part = Polygon([(0, 0), (0.3, 0), (0.3, 0.3), (0, 0.3)])
    min_dist = 0.02
    geom = ProposeGeometry(board, Polygon(), part, min_dist, border_focus=False)
    xy = (2.0, 0.12)
    pole = Point(1.2, 1.0)
    info = edge_inward_at_point(board, Point(*xy))
    assert info is not None
    anchor, _ = info
    dist = math.hypot(xy[0] - float(anchor.x), xy[1] - float(anchor.y))
    _, part_max = part_extents(part)
    assert dist <= max(4.0 * min_dist, part_max)
    placed = geom.placed_at((xy[0], xy[1], 0.0))
    props, _g = _merged_guidance_propositions(geom, placed, xy, pole, 0.0)
    assert isinstance(props, list)
    cfg = GuidanceConfig()
    geom._apply_rim_gravity(cfg, xy)
    pole_dx, pole_dy = float(pole.x) - xy[0], float(pole.y) - xy[1]
    plen = math.hypot(pole_dx, pole_dy)
    gx, gy = cfg.gravity_vector
    # Rim inward at the base is not the same as pole-at-centroid.
    assert abs(gx - pole_dx / plen) > 0.05 or abs(gy - pole_dy / plen) > 0.05


def test_void_densify_pole_gravity_skips_rim_band():
    """Densify flag: tight merged pass uses pole unit even on rim seeds."""
    from nest_graph.propose.placements_guidance import _merged_guidance_propositions

    assert ProposeConfig().void_densify_pole_gravity is False
    board = Polygon([(0, 0), (4, 0), (0, 3)])
    part = Polygon([(0, 0), (0.3, 0), (0.3, 0.3), (0, 0.3)])
    cfg = ProposeConfig(void_densify_pole_gravity=True)
    geom = ProposeGeometry(
        board, Polygon(), part, 0.02, border_focus=False, propose_cfg=cfg,
    )
    xy = (2.0, 0.12)
    pole = Point(1.2, 1.0)
    placed = geom.placed_at((xy[0], xy[1], 0.0))
    called = {"rim": 0}
    orig = geom._apply_rim_gravity

    def _spy(tight_cfg, seed_xy):
        called["rim"] += 1
        return orig(tight_cfg, seed_xy)

    geom._apply_rim_gravity = _spy  # type: ignore[method-assign]
    props, _g = _merged_guidance_propositions(geom, placed, xy, pole, 0.0)
    assert isinstance(props, list)
    assert called["rim"] == 0


def test_guidance_propositions_expand_smoke():
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
