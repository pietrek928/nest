import numpy as np
from shapely.geometry import Point, Polygon

from nest_graph.config import BuildGraphConfig, ProposeConfig
from nest_graph.propose import proposed_transforms_for_groups
from nest_graph.propose.placements_batch import (
    _batch_pack_pair_order,
    _parts_by_group,
    augment_batch_pack_proposals,
)
from nest_graph.utils import transform_poly


def test_batch_pack_adds_proposals_on_empty_board(
    nest_board, rect_poly, tri_poly, build_graph_config,
):
    cfg = build_graph_config
    cfg_off = cfg.model_copy(deep=True)
    cfg_off.propose = ProposeConfig(max_proposals=16, use_batch_pack=False)
    cfg_on = cfg.model_copy(deep=True)
    cfg_on.propose = ProposeConfig(
        max_proposals=16,
        use_batch_pack=True,
        batch_pack_anchor_seeds=4,
        batch_pack_follow_proposals=6,
        batch_pack_max_pairs=8,
    )

    min_dist = cfg.board_min_dist()
    parts = [(rect_poly, 0), (tri_poly, 1)]
    kwargs = dict(
        board=nest_board,
        parts=parts,
        selected_polys=[],
        selected_indices=[],
        min_dist=min_dist,
        pt_push=Point(nest_board.centroid),
    )
    off = proposed_transforms_for_groups(**kwargs, propose_cfg=cfg_off.propose)
    on = proposed_transforms_for_groups(**kwargs, propose_cfg=cfg_on.propose)

    assert on[0].shape[0] >= off[0].shape[0]
    assert on[1].shape[0] >= off[1].shape[0]
    assert on[0].shape[0] > off[0].shape[0] or on[1].shape[0] > off[1].shape[0]


def test_batch_pack_pairs_are_mutually_valid(
    nest_board, rect_poly, tri_poly, build_graph_config,
):
    cfg = build_graph_config
    cfg.propose = ProposeConfig(
        max_proposals=12,
        use_batch_pack=True,
        batch_pack_anchor_seeds=3,
        batch_pack_follow_proposals=4,
        batch_pack_max_pairs=6,
    )
    min_dist = cfg.board_min_dist()
    parts = [(rect_poly, 0), (tri_poly, 1)]
    per_group = proposed_transforms_for_groups(
        nest_board, parts, [], [], cfg.propose, min_dist=min_dist,
    )
    parts_by_group = _parts_by_group(parts)
    pairs = _batch_pack_pair_order(
        nest_board,
        0,
        1,
        parts_by_group,
        Polygon(),
        per_group,
        cfg.propose,
        min_dist=min_dist,
        placed=[],
    )
    assert pairs
    for coords_a, coords_b, _score in pairs[:3]:
        pa = transform_poly(rect_poly, coords_a)
        pb = transform_poly(tri_poly, coords_b)
        assert nest_board.contains(pa)
        assert nest_board.contains(pb)
        assert not pa.intersects(pb)
        assert pa.distance(pb) >= min_dist - 1e-5


def test_batch_pack_augment_returns_per_group(
    nest_board, rect_poly, tri_poly, build_graph_config,
):
    cfg = build_graph_config
    cfg.propose.use_batch_pack = True
    cfg.propose.batch_pack_anchor_seeds = 3
    min_dist = cfg.board_min_dist()
    parts = [(rect_poly, 0), (tri_poly, 1)]
    per_group = proposed_transforms_for_groups(
        nest_board, parts, [], [], cfg.propose, min_dist=min_dist,
    )
    extra = augment_batch_pack_proposals(
        nest_board, parts, [], per_group, cfg.propose, min_dist=min_dist,
    )
    assert extra[0]
    assert extra[1]


def test_default_config_enables_batch_pack():
    assert BuildGraphConfig().propose.use_batch_pack is True
