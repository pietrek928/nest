"""Unit tests for late border saturation void override and hull rim gate."""

from shapely.geometry import box

from nest_graph.build_graph import NestState
from nest_graph.propose.context import late_border_saturation_info as _late_border_saturation_info
from nest_graph.config import BuildGraphConfig, ProposeConfig
from nest_graph.utils import transform_poly


def _cfg(**propose_kw) -> BuildGraphConfig:
    cfg = BuildGraphConfig()
    if propose_kw:
        cfg.propose = cfg.propose.model_copy(update=propose_kw)
    return cfg


def _state(polys) -> NestState:
    return NestState(
        polys=list(polys),
        group_id=[0] * len(polys),
        transform=[(0.0, 0.0, 0.0)] * len(polys),
        selected_indices=list(range(len(polys))),
    )


def test_late_sat_defaults():
    p = ProposeConfig()
    assert p.late_border_void_override_ratio == 2.5
    assert p.late_border_void_release_ratio == 1.5
    assert p.late_border_hull_threshold == 0.4
    assert p.void_attractor_rule_weight == 16.0
    assert p.attract_contact_weight == 8.0


def test_large_void_overrides_late_sat():
    sheet = box(0, 0, 10, 10)
    part = box(0, 0, 1, 1)
    placed = [transform_poly(part, (1.0, 1.0, 0.0))]
    info = _late_border_saturation_info(_cfg(), _state(placed), sheet)
    assert info.free_kind == "large_void"
    assert info.sat_override is True
    assert info.active is False
    assert info.outline_cov < 0.35


def test_override_disabled_keeps_sat_on_large_void():
    sheet = box(0, 0, 10, 10)
    part = box(0, 0, 1, 1)
    placed = [transform_poly(part, (1.0, 1.0, 0.0))]
    info = _late_border_saturation_info(
        _cfg(late_border_void_override_ratio=0.0),
        _state(placed),
        sheet,
    )
    assert info.free_kind == "large_void"
    assert info.sat_override is False
    assert info.active is True


def test_hysteresis_holds_override_after_entry():
    """Once unlocked, hold override while ratio still above release (1.5)."""
    sheet = box(0, 0, 10, 10)
    part = box(0, 0, 1, 1)
    placed = [transform_poly(part, (1.0, 1.0, 0.0))]
    # Entry off via huge override threshold; hold via had_void_override + release.
    info = _late_border_saturation_info(
        _cfg(late_border_void_override_ratio=1000.0),
        _state(placed),
        sheet,
        had_void_override=True,
    )
    assert info.sat_override is True
    assert info.active is False


def test_hysteresis_disabled_when_release_zero():
    sheet = box(0, 0, 10, 10)
    part = box(0, 0, 1, 1)
    placed = [transform_poly(part, (1.0, 1.0, 0.0))]
    info = _late_border_saturation_info(
        _cfg(
            late_border_void_override_ratio=1000.0,
            late_border_void_release_ratio=0.0,
        ),
        _state(placed),
        sheet,
        had_void_override=True,
    )
    assert info.sat_override is False
    assert info.active is True


def test_swiss_cheese_no_void_override():
    sheet = box(0, 0, 3, 3)
    # Near-full obstacle → swiss_cheese, no large_void override.
    placed = [box(0.2, 0.2, 2.8, 2.8)]
    info = _late_border_saturation_info(_cfg(), _state(placed), sheet)
    assert info.free_kind == "swiss_cheese"
    assert info.sat_override is False
    # Rim progress near 1 for a near-full pack hull → Exp3 ends sat.
    assert info.rim_progress >= 0.4
    assert info.active is False


def test_hull_threshold_zero_disables_rim_gate():
    sheet = box(0, 0, 3, 3)
    placed = [box(0.2, 0.2, 2.8, 2.8)]
    info = _late_border_saturation_info(
        _cfg(
            late_border_void_override_ratio=0.0,
            late_border_hull_threshold=0.0,
            place_border_coverage_threshold=0.99,
        ),
        _state(placed),
        sheet,
    )
    assert info.sat_override is False
    assert info.outline_cov < 0.99
    assert info.active is True
