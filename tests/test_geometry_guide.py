import math
import sys

import pytest

from nest_graph.geometry import Geometry, GuidanceConfig, evaluate_local_placement
from nest_graph.placement_scene import (
    best_proposition,
    placement_clearance_epsilon,
    propositions_by_tier,
    proposition_translation,
)


def _unit_square():
    return Geometry.from_convex_polygon([(0, 0), (1, 0), (1, 1), (0, 1)])


def test_guidance_config_defaults():
    cfg = GuidanceConfig()
    assert cfg.use_target_attractor is False
    assert cfg.use_gravity is True
    assert cfg.minimum_placing_distance == pytest.approx(1e-6)
    assert cfg.max_alternative_angles == 3
    assert cfg.slide_escape_multiplier == pytest.approx(0.8)
    assert cfg.use_hole_seeking is True
    assert cfg.use_tight_packing is True
    assert cfg.use_corner_alignment is True
    assert cfg.max_hole_size_ratio == pytest.approx(4.0)
    assert cfg.max_propositions == 6
    assert cfg.enable_grid_exploration is True
    assert cfg.target_angle_rad == pytest.approx(0.0)
    assert cfg.attraction_weight == pytest.approx(0.1)
    assert cfg.alignment_weight == pytest.approx(0.5)
    assert cfg.search_radius == pytest.approx(25.0)
    assert cfg.target_position == (0.0, 0.0)
    assert cfg.gravity_vector == (-1.0, -1.0)


def test_evaluate_single_polygon_gravity():
    poly = _unit_square()
    cfg = GuidanceConfig()
    cfg.minimum_placing_distance = 0.0
    cfg.search_radius = 50.0
    guidance = evaluate_local_placement(0, [poly], (0.0, 0.0), cfg)
    assert guidance.is_penetrating is False
    assert len(guidance.propositions) >= 1
    prop = best_proposition(guidance)
    tx, ty = proposition_translation(prop)
    assert prop.move_type == "Free Space Default"
    assert tx == pytest.approx(-50.0)
    assert ty == pytest.approx(-50.0)


def test_evaluate_target_attractor():
    poly = _unit_square()
    distant = _unit_square().translate(20.0, 0.0)
    cfg = GuidanceConfig()
    cfg.use_target_attractor = True
    cfg.use_gravity = False
    cfg.use_hole_seeking = False
    cfg.use_tight_packing = False
    cfg.enable_grid_exploration = False
    cfg.minimum_placing_distance = 0.0
    cfg.search_radius = 25.0
    cfg.target_position = (10.0, 0.0)
    cfg.attraction_weight = 0.2

    guidance = evaluate_local_placement(0, [poly, distant], (0.0, 0.0), cfg)
    assert guidance.is_penetrating is False
    prop = best_proposition(guidance)
    tx, ty = proposition_translation(prop)
    assert tx == pytest.approx(2.0)
    assert ty == pytest.approx(0.0)


def test_evaluate_penetrating_overlap():
    a = _unit_square()
    b = _unit_square().translate(0.3, 0.1)
    cfg = GuidanceConfig()
    cfg.search_radius = 2.0
    cfg.minimum_placing_distance = 0.05

    guidance = evaluate_local_placement(0, [a, b], (0.0, 0.0), cfg)
    assert guidance.is_penetrating is True
    tiers = propositions_by_tier(guidance)
    assert tiers["ejection"] or tiers["slide"]
    prop = best_proposition(guidance)
    tx, ty = proposition_translation(prop)
    assert math.hypot(tx, ty) > 1e-9


def test_evaluate_separated_clearance():
    a = _unit_square()
    b = _unit_square().translate(3.0, 0.0)
    cfg = GuidanceConfig()
    cfg.search_radius = 5.0
    cfg.minimum_placing_distance = 0.0

    guidance = evaluate_local_placement(0, [a, b], (0.0, 0.0), cfg)
    assert guidance.is_penetrating is False
    assert guidance.clearance < sys.float_info.max
    assert guidance.clearance > 0.0


def test_exact_neighbor_snap_when_separated():
    a = _unit_square()
    b = _unit_square().translate(1.2, 0.0)
    cfg = GuidanceConfig()
    cfg.search_radius = 5.0
    cfg.minimum_placing_distance = 0.0
    cfg.use_tight_packing = True
    cfg.use_gravity = False
    cfg.enable_grid_exploration = False

    guidance = evaluate_local_placement(0, [a, b], (0.0, 0.0), cfg)
    assert guidance.is_penetrating is False
    snap_types = [
        p.move_type for p in guidance.propositions
        if "Exact Neighbor Snap" in (p.move_type or "")
    ]
    assert snap_types
    prop = next(
        p for p in guidance.propositions
        if "Exact Neighbor Snap" in (p.move_type or "")
    )
    tx, ty = proposition_translation(prop)
    assert math.hypot(tx, ty) == pytest.approx(0.2, abs=0.05)


def test_gravity_scales_down_near_obstacle():
    a = _unit_square()
    b = _unit_square().translate(1.05, 0.0)
    cfg = GuidanceConfig()
    cfg.search_radius = 5.0
    cfg.minimum_placing_distance = 0.0
    cfg.use_gravity = True
    cfg.use_target_attractor = False
    cfg.enable_grid_exploration = False

    guidance = evaluate_local_placement(0, [a, b], (0.0, 0.0), cfg)
    assert guidance.is_penetrating is False
    assert guidance.clearance < 0.5
    prop = best_proposition(guidance)
    assert prop is not None
    tx, ty = proposition_translation(prop)
    assert math.hypot(tx, ty) < math.hypot(-1.0, -1.0) * cfg.search_radius


def test_floor_walk_when_grid_exploration_on():
    a = _unit_square()
    b = _unit_square().translate(1.2, 0.0)
    cfg = GuidanceConfig()
    cfg.search_radius = 5.0
    cfg.minimum_placing_distance = 0.0
    cfg.use_gravity = True
    cfg.use_tight_packing = False
    cfg.use_corner_alignment = False
    cfg.enable_grid_exploration = True

    guidance = evaluate_local_placement(0, [a, b], (0.0, 0.0), cfg)
    assert guidance.is_penetrating is False
    walks = [
        p.move_type for p in guidance.propositions
        if "Floor Walk" in (p.move_type or "")
    ]
    assert walks


def test_tight_packing_proposition_when_separated():
    a = _unit_square()
    b = _unit_square().translate(1.2, 0.0)
    cfg = GuidanceConfig()
    cfg.search_radius = 5.0
    cfg.minimum_placing_distance = 0.0
    cfg.use_tight_packing = True
    cfg.use_gravity = False
    cfg.enable_grid_exploration = False

    guidance = evaluate_local_placement(0, [a, b], (0.0, 0.0), cfg)
    assert guidance.is_penetrating is False
    tiers = propositions_by_tier(guidance)
    assert tiers["pack"]


def test_placement_clearance_epsilon():
    assert placement_clearance_epsilon(0.0) == pytest.approx(1e-6)
    assert placement_clearance_epsilon(0.1) == pytest.approx(0.005)
