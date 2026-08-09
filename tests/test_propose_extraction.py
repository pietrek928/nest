"""Unit tests for the propose modules extracted out of build_graph.

Covers propose/first_pass_border.py, propose/void_selection.py, the staged
collect helpers in propose/pipeline.py, and late_border re-homing on context.py.
"""

import numpy as np
import pytest
from shapely.geometry import Point, box

from nest_graph.config import BuildGraphConfig, ProposeConfig
from nest_graph.propose import first_pass_border_coords
from nest_graph.propose.context import (
    late_border_saturation_active,
    late_border_saturation_info,
    propose_push_point,
)
from nest_graph.propose.first_pass_border import (
    FIRST_PASS_BORDER_PROPOSERS,
    border_kiss_indices,
    nest_outline_boundary,
    outline_anchor_inward,
)
from nest_graph.propose.geometry import ProposeGeometry
from nest_graph.propose.pipeline import (
    _CollectState,
    _collect_candidates,
    collect_propose_candidates,
)
from nest_graph.propose.placement_common import is_pose_clear
from nest_graph.propose.types import (
    PackedProposeExtras,
    PocketStats,
    make_propose_context,
)
from nest_graph.propose.void_selection import (
    boost_border_scores,
    count_props_near_pole,
    format_prop_accept,
    pin_nest_void_independent,
    transform_row_key,
    void_attractor_radius,
)
from nest_graph.proposer_names import ProposerName
from nest_graph.utils import transform_poly


def _cfg() -> BuildGraphConfig:
    return BuildGraphConfig()


class _FakeGraph:
    """Minimal duck-typed ElemGraph for pin repair (collisions + group_id)."""

    def __init__(self, n: int, collisions: dict[int, list[int]]):
        self.group_id = [0] * n
        self.collisions = [collisions.get(i, []) for i in range(n)]


def test_transform_row_key_matches_round4():
    key = transform_row_key(np.array([1.234567, -2.0, 0.5]))
    assert key == (1.2346, -2.0, 0.5)


def test_void_attractor_radius_is_sheet_aware():
    # Legacy local radius is too sharp; sheet diag term must dominate.
    assert void_attractor_radius(0.01, 10.0, 0.5) == pytest.approx(2.5)
    assert void_attractor_radius(0.01, 0.0, 0.5) == pytest.approx(0.5)


def test_boost_border_scores_favors_flush_nodes():
    outline = box(0, 0, 10, 10)
    min_dist = 0.1
    flush = box(0.1, 0.1, 1.1, 1.1)
    inner = box(4.0, 4.0, 5.0, 5.0)
    scores = [0.0, 0.0]
    boost_border_scores([flush, inner], scores, outline, min_dist, weight=8.0)
    assert scores[0] > scores[1]


def test_format_prop_accept_orders_by_emit_and_limits():
    line = format_prop_accept(
        {"pocket_fit": 9, "raycast": 3},
        {"pocket_fit": 5},
        {"pocket_fit": 2},
        {"pocket_fit": 1},
        limit=1,
    )
    assert line == "pocket_fit:e9/p5/n2/r1"


def test_count_props_near_pole_uses_radius():
    props = [np.array([[0.0, 0.0, 0.0], [5.0, 5.0, 0.0]])]
    assert count_props_near_pole(props, Point(0.0, 0.0), 1.0) == 1
    assert count_props_near_pole(props, Point(0.0, 0.0), 0.0) == 0
    assert count_props_near_pole(props, None, 1.0) == 0


def test_pin_nest_void_independent_respects_collisions():
    free = box(0, 0, 10, 10)
    polys = [box(1, 1, 2, 2), box(3, 3, 4, 4), box(5, 5, 6, 6)]
    # Node 1 collides with the already-selected node 0 → must stay out.
    graph = _FakeGraph(3, {0: [1], 1: [0], 2: []})
    stats: dict = {}
    out = pin_nest_void_independent(
        graph, [0, 1, 2], [0], polys, free, stats_out=stats,
    )
    assert set(out) == {0, 2}
    assert stats["pin_candidates"] == 2
    assert stats["pin_added"] == 1
    assert stats["pin_blocked_collision"] == 1
    assert stats["pin_ms"] >= 0.0


def test_pin_skips_nodes_outside_free_poly():
    free = box(0, 0, 3, 3)
    polys = [box(1, 1, 2, 2), box(8, 8, 9, 9)]
    graph = _FakeGraph(2, {0: [], 1: []})
    stats: dict = {}
    out = pin_nest_void_independent(
        graph, [0, 1], [], polys, free, stats_out=stats,
    )
    assert out == [0]
    assert stats["pin_candidates"] == 1


def test_late_border_saturation_lives_on_context():
    from nest_graph.build_graph import NestState

    sheet = box(0, 0, 10, 10)
    placed = [transform_poly(box(0, 0, 1, 1), (1.0, 1.0, 0.0))]
    state = NestState(
        polys=placed,
        group_id=[0],
        transform=[(1.0, 1.0, 0.0)],
        selected_indices=[0],
    )
    info = late_border_saturation_info(_cfg(), state, sheet)
    assert info.free_kind == "large_void"
    assert info.active is False
    assert late_border_saturation_active(_cfg(), state, sheet) is False


def test_late_border_saturation_info_handles_no_state():
    info = late_border_saturation_info(_cfg(), None, box(0, 0, 10, 10))
    assert info == (False, 0.0, False, 0.0, "")


def test_nest_outline_boundary_and_anchor_inward():
    outline = box(0, 0, 10, 10)
    ring = nest_outline_boundary(outline)
    assert ring is not None and not ring.is_empty
    anchor, inward = outline_anchor_inward(box(0.0, 0.0, 1.0, 1.0), outline)
    assert anchor is not None
    assert abs(float(np.hypot(inward[0], inward[1])) - 1.0) < 1e-6


def test_border_kiss_indices_finds_flush_only():
    outline = box(0, 0, 10, 10)
    min_dist = 0.1
    polys = [box(0.1, 0.1, 1.1, 1.1), box(4.0, 4.0, 5.0, 5.0)]
    assert border_kiss_indices(polys, outline, min_dist) == [0]


def test_first_pass_border_coords_are_clear_and_deduped():
    cfg = _cfg()
    board = box(0, 0, 10, 10)
    part = box(0, 0, 1, 1)
    min_dist = cfg.board_min_dist(first_pass=True)
    placed = [transform_poly(part, (0.5, 0.5, 0.0))]
    coords = first_pass_border_coords(
        cfg, board, part, placed, min_dist=min_dist,
    )
    assert coords, "border augment must emit candidates on an open sheet"
    keys = {(round(c[0], 3), round(c[1], 3), round(c[2], 2)) for c in coords}
    assert len(keys) == len(coords)
    for c in coords[:12]:
        assert is_pose_clear(transform_poly(part, c), board, placed, min_dist)


def test_first_pass_border_coords_on_empty_pack():
    cfg = _cfg()
    board = box(0, 0, 10, 10)
    part = box(0, 0, 1, 1)
    coords = first_pass_border_coords(
        cfg, board, part, [], min_dist=cfg.board_min_dist(first_pass=True),
    )
    assert coords
    assert FIRST_PASS_BORDER_PROPOSERS == frozenset({
        ProposerName.BOARD_EDGE,
        ProposerName.GROUP_FIT,
        ProposerName.NEIGHBOR_SLIDE,
    })


def _ctx_and_extras(propose_cfg: ProposeConfig, *, packed):
    board = box(0, 0, 10, 10)
    part = box(0, 0, 1, 1)
    min_dist = 0.05
    obstacle = packed[0] if packed else box(0, 0, 0, 0)
    push = propose_push_point(
        board, obstacle, smart_push=propose_cfg.smart_push_target,
        min_dist=min_dist, use_border_focus=True,
    )
    geom = ProposeGeometry(
        board, obstacle, part, min_dist,
        epsilon_ratio=propose_cfg.placement_clearance_epsilon_ratio,
        propose_cfg=propose_cfg,
    )
    ctx = make_propose_context(
        base_shape=obstacle,
        shape_to_place=part,
        sheet=board,
        propose_cfg=propose_cfg,
        min_dist=min_dist,
        pt_push=push,
        propose_geom=geom,
        border_focus=True,
    )
    extras = PackedProposeExtras(packed_polys=packed, pocket_stats=PocketStats())
    return ctx, extras


def test_collect_stage_order_is_static_per_mode():
    """Both modes keep snipers first; only builder/explorer order differs."""
    propose_cfg = ProposeConfig()
    packed = [transform_poly(box(0, 0, 1, 1), (0.5, 0.5, 0.0))]
    for mode in ("cascade", "free"):
        ctx, extras = _ctx_and_extras(propose_cfg, packed=packed)
        counts: dict[str, int] = {}
        ctx.proposer_counts = counts
        out = _collect_candidates(ctx, extras, mode=mode)
        assert out, f"mode={mode} produced no candidates"
        names = list(counts)
        if ProposerName.POCKET_FIT in names:
            assert names.index(ProposerName.POCKET_FIT) == 0


def test_collect_state_tracks_keys_and_skips():
    state = _CollectState(
        proposer_counts={}, proposer_keys={}, cascade_stats_out={},
    )
    state.ext("board_edge", [(1.0, 2.0, 0.0), (1.0, 2.0, 0.0)])
    # Duplicate keys are claimed once so funnel counts do not double-count.
    assert state.proposer_counts["board_edge"] == 2
    assert len(state.proposer_keys["board_edge"]) == 1
    state.mark_skip("snipers", ["raycasting", "voronoi"])
    assert state.cascade_stats_out["cascade_stopped_after"] == "snipers"
    assert state.cascade_stats_out["cascade_skipped_proposers"] == [
        "raycasting", "voronoi",
    ]


def test_collect_state_explorer_cap_scales():
    state = _CollectState(
        proposer_counts=None, proposer_keys=None, cascade_stats_out=None,
    )
    assert state.explorer_cap(20) == 20
    state.explorer_scale = 0.35
    assert state.explorer_cap(20) == 7
    state.skip_explorers = True
    assert state.explorer_cap(20) == 0


def test_collect_propose_candidates_wrapper_reports_pocket_stats():
    propose_cfg = ProposeConfig()
    board = box(0, 0, 10, 10)
    part = box(0, 0, 1, 1)
    min_dist = 0.05
    packed = [transform_poly(part, (0.5, 0.5, 0.0))]
    geom = ProposeGeometry(
        board, packed[0], part, min_dist,
        epsilon_ratio=propose_cfg.placement_clearance_epsilon_ratio,
        propose_cfg=propose_cfg,
    )
    tags: list[str] = []
    out = collect_propose_candidates(
        packed[0],
        part,
        board,
        propose_cfg,
        min_dist=min_dist,
        pt_push=Point(9.5, 9.5),
        propose_geom=geom,
        packed_polys=packed,
        pocket_tags_out=tags,
        border_focus=True,
    )
    assert out
    assert all(len(c) == 3 for c in out)
