"""T0 smoke: propose emit / tightness / perimeter fixtures are wired."""

import pytest

from nest_graph.placement.pattern import emit_packing_clear
from nest_graph.propose.ranking import pack_tightness_cost
from tests.fixtures.propose_emit import (
    EMIT_CLEAR_POSE_CASES,
    EMIT_PENETRATE_CASES,
    make_propose_geom,
    tightness_pack_geoms,
)


@pytest.mark.parametrize(
    "label,x,y,th,expect",
    EMIT_CLEAR_POSE_CASES,
    ids=[c[0] for c in EMIT_CLEAR_POSE_CASES],
)
def test_emit_clear_cases_table(label, x, y, th, expect):
    geom = make_propose_geom()
    assert emit_packing_clear(geom, (x, y, th)) is expect


@pytest.mark.parametrize(
    "label,x,y,th,expect",
    EMIT_PENETRATE_CASES,
    ids=[c[0] for c in EMIT_PENETRATE_CASES],
)
def test_emit_penetrate_cases_table(label, x, y, th, expect):
    from shapely.geometry import box

    geom = make_propose_geom(packed=[box(9.0, 9.0, 12.0, 12.0)])
    assert emit_packing_clear(geom, (x, y, th)) is expect


def test_tightness_pack_fixture_finite():
    geoms, outline, min_dist = tightness_pack_geoms(n=3, gap=0.15)
    cost = pack_tightness_cost(geoms, outline, min_dist)
    assert cost >= 0.0
    assert cost < float("inf")


def test_perimeter_board_fixture_builds(perimeter_board_setup):
    geom, base, part = perimeter_board_setup
    assert geom is not None
    assert not base.is_empty
    assert float(part.area) > 0.0
