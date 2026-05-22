import math
import sys

import pytest

from nest_graph.geometry import Geometry, GuidanceConfig, evaluate_local_placement


def _unit_square():
    return Geometry.from_convex_polygon([(0, 0), (1, 0), (1, 1), (0, 1)])


def test_guidance_config_defaults():
    cfg = GuidanceConfig()
    assert cfg.use_target_attractor is False
    assert cfg.use_gravity is True
    assert cfg.target_angle_rad == pytest.approx(0.0)
    assert cfg.attraction_weight == pytest.approx(0.1)
    assert cfg.alignment_weight == pytest.approx(0.5)
    assert cfg.search_radius == pytest.approx(5.0)
    assert cfg.target_position == (0.0, 0.0)
    assert cfg.gravity_vector == (-1.0, -1.0)


def test_evaluate_single_polygon_gravity():
    poly = _unit_square()
    guidance = evaluate_local_placement(0, [poly], (0.0, 0.0))
    assert guidance.is_penetrating is False
    assert guidance.suggested_translation == (-1.0, -1.0)


def test_evaluate_target_attractor():
    poly = _unit_square()
    cfg = GuidanceConfig()
    cfg.use_target_attractor = True
    cfg.use_gravity = False
    cfg.target_position = (10.0, 0.0)
    cfg.attraction_weight = 0.2

    guidance = evaluate_local_placement(0, [poly], (0.0, 0.0), cfg)
    assert guidance.is_penetrating is False
    assert guidance.suggested_translation[0] == pytest.approx(2.0)
    assert guidance.suggested_translation[1] == pytest.approx(0.0)


def test_evaluate_penetrating_overlap():
    a = _unit_square()
    # Asymmetric overlap so distance sweep reports a non-zero MTV (axis-aligned
    # half-offset often yields intersect=True with penetration_sq=0).
    b = _unit_square().translate(0.3, 0.1)
    cfg = GuidanceConfig()
    cfg.search_radius = 2.0

    guidance = evaluate_local_placement(0, [a, b], (0.0, 0.0), cfg)
    assert guidance.is_penetrating is True
    ex, ey = guidance.ejection_vector
    assert math.hypot(ex, ey) > 1e-9


def test_evaluate_separated_clearance():
    a = _unit_square()
    b = _unit_square().translate(3.0, 0.0)
    cfg = GuidanceConfig()
    cfg.search_radius = 5.0

    guidance = evaluate_local_placement(0, [a, b], (0.0, 0.0), cfg)
    assert guidance.is_penetrating is False
    assert guidance.clearance < sys.float_info.max
    assert guidance.clearance > 0.0
