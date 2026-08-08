"""OOS void-fill: proposer telemetry, native void_seek, pole ranking, P3 pin."""

import numpy as np
from shapely.geometry import Point, Polygon
from shapely.ops import unary_union

from nest_graph.board import board_sheet_from_outline
from nest_graph.build_graph import (
    _count_selected_by_proposer,
    _format_prop_accept,
    _pin_nest_void_independent,
    _transform_row_key,
)
from nest_graph.config import ProposeConfig
from nest_graph.elem_graph import ElemGraph
from nest_graph.propose.context import classify_propose_zone
from nest_graph.propose.geometry import ProposeGeometry
from nest_graph.propose.pipeline import _extend_counted, _proposal_key
from nest_graph.propose.ranking import _score_placement_contact_hybrid
from nest_graph.utils import transform_poly


def test_proposal_key_matches_transform_row_key_round4():
    row = np.asarray([1.23456, -9.87654, 0.123456], dtype=np.float64)
    assert _proposal_key(tuple(row)) == _transform_row_key(row)
    assert _proposal_key(row) == (round(1.23456, 4), round(-9.87654, 4), round(0.123456, 4))


def test_extend_counted_first_writer_wins():
    candidates: list[tuple[float, float, float]] = []
    proposer_keys: dict[str, set[tuple[float, float, float]]] = {}
    claimed: set[tuple[float, float, float]] = set()
    a = (1.0, 2.0, 0.0)
    b = (1.00004, 2.0, 0.0)  # same round-4 key as a
    _extend_counted(
        candidates, None, "pocket_fit", [a],
        proposer_keys=proposer_keys, claimed_keys=claimed,
    )
    _extend_counted(
        candidates, None, "raycast", [b, (3.0, 4.0, 0.0)],
        proposer_keys=proposer_keys, claimed_keys=claimed,
    )
    assert _proposal_key(a) in proposer_keys["pocket_fit"]
    assert _proposal_key(a) not in proposer_keys.get("raycast", set())
    assert _proposal_key((3.0, 4.0, 0.0)) in proposer_keys["raycast"]
    nest = _count_selected_by_proposer(
        [np.asarray(a), np.asarray((3.0, 4.0, 0.0))],
        [0, 1],
        proposer_keys,
    )
    assert nest == {"pocket_fit": 1, "raycast": 1}
    snip = _format_prop_accept(
        {"pocket_fit": 2, "raycast": 5},
        {"pocket_fit": 1, "raycast": 3},
        nest,
        {"pocket_fit": 1},
    )
    assert "pocket_fit:e2/p1/n1/r1" in snip
    assert "raycast:e5/p3/n1/r0" in snip


def test_oos1_exterior_large_free_void_seek_despite_packed_near_border():
    board = Polygon([(0, 0), (20, 0), (20, 20), (0, 20)])
    part = Polygon([(0, 0), (1, 0), (1, 1), (0, 1)])
    # Rim seeds leave a large open bay toward the center/right.
    placed = [
        transform_poly(part, (1.0, 1.0, 0.0)),
        transform_poly(part, (3.0, 1.0, 0.0)),
        transform_poly(part, (1.0, 3.0, 0.0)),
        transform_poly(part, (1.0, 19.0, 0.0)),
        transform_poly(part, (19.0, 1.0, 0.0)),
    ]
    obstacle = unary_union(placed)
    free_area = float(board.difference(obstacle).area)
    assert free_area / float(part.area) > 2.5
    zone = classify_propose_zone(
        board,
        obstacle,
        part,
        min_dist=0.05,
        propose_cfg=ProposeConfig(),
        selected_polys=placed,
    )
    assert zone == "void_seek"


def test_oos1_narrow_mouth_still_not_void_seek_when_override_off():
    outline = Polygon([(0, 0), (12, 0), (12, 12), (0, 12)])
    hole = ((5, 5), (7, 5), (7, 7), (5, 7), (5, 5))
    sheet = board_sheet_from_outline(outline, user_holes=(hole,))
    rect = Polygon([(0, 0), (2.5, 0), (2.5, 2.5), (0, 2.5)])
    placed = [
        transform_poly(rect, (1.5, 1.5, 0.0)),
        transform_poly(rect, (10.5, 1.5, 0.0)),
        transform_poly(rect, (1.5, 10.5, 0.0)),
    ]
    obstacle = unary_union(placed)
    zone = classify_propose_zone(
        outline,
        obstacle,
        rect,
        min_dist=0.05,
        propose_cfg=ProposeConfig(late_border_void_override_ratio=0.0),
        selected_polys=placed,
        user_holes=(hole,),
        sheet=sheet,
    )
    assert zone != "void_seek"


def test_oos4_void_pole_bonus_prefers_deeper_and_larger():
    sheet = Polygon([(0, 0), (10, 0), (10, 10), (0, 10)])
    part_small = Polygon([(0, 0), (1, 0), (1, 1), (0, 1)])
    part_large = Polygon([(0, 0), (2, 0), (2, 2), (0, 2)])
    # Empty obstacle keeps contact scores finite so pole delta is visible.
    obstacle = Polygon()
    pole = Point(8.0, 8.0)
    cfg_on = ProposeConfig(void_rank_pole_weight=8.0)
    cfg_off = ProposeConfig(void_rank_pole_weight=0.0)
    geom_small = ProposeGeometry(
        sheet, obstacle, part_small, 0.05, epsilon_ratio=0.0, propose_cfg=cfg_on,
    )
    geom_large = ProposeGeometry(
        sheet, obstacle, part_large, 0.05, epsilon_ratio=0.0, propose_cfg=cfg_on,
    )
    near = (7.5, 7.5, 0.0)
    far = (4.0, 4.0, 0.0)
    push = Point(5.0, 5.0)

    def bonus(geom, part, coords, cfg):
        return _score_placement_contact_hybrid(
            coords, part, geom, push, 0.05, None, 0.0,
            propose_cfg=cfg, void_pole=pole,
        )

    near_bonus = bonus(geom_small, part_small, near, cfg_on) - bonus(
        geom_small, part_small, near, cfg_off,
    )
    far_bonus = bonus(geom_small, part_small, far, cfg_on) - bonus(
        geom_small, part_small, far, cfg_off,
    )
    assert near_bonus > 0.0
    assert near_bonus > far_bonus
    large_bonus = bonus(geom_large, part_large, near, cfg_on) - bonus(
        geom_large, part_large, near, cfg_off,
    )
    assert large_bonus > near_bonus


def test_p3_pin_adds_independent_nest_void_only():
    from nest_graph.elem_graph import Circle, Vec2

    graph = ElemGraph()
    for i in range(3):
        graph.append_elem(0, Vec2(x=float(i), y=0.0), Circle.from_center_radius(float(i), 0.0, 0.1))
    graph.add_collision(0, 1)
    polys = [
        Polygon([(0, 0), (1, 0), (1, 1), (0, 1)]),
        # Centroid in free; collides with refine node 0 via graph edge.
        Polygon([(4.5, 4.5), (5.5, 4.5), (5.5, 5.5), (4.5, 5.5)]),
        Polygon([(5, 5), (6, 5), (6, 6), (5, 6)]),
    ]
    free = Polygon([(4, 4), (8, 4), (8, 8), (4, 8)])
    nest = [0, 2]
    refine = [0]
    scores = [1.0, 0.5, 2.0]
    pinned = _pin_nest_void_independent(graph, nest, refine, polys, free, scores)
    assert 2 in pinned
    assert 0 in pinned
    # Nest void that collides with refine is skipped (blocked_collision).
    pin_stats: dict = {}
    pinned2 = _pin_nest_void_independent(
        graph, [1, 2], [0], polys, free, scores, stats_out=pin_stats,
    )
    assert 1 not in pinned2
    assert 2 in pinned2
    assert pin_stats["pin_candidates"] == 2
    assert pin_stats["pin_added"] == 1
    assert pin_stats["pin_blocked_collision"] == 1
    assert pin_stats["pin_ms"] >= 0.0
