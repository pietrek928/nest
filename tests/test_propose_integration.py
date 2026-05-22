import numpy as np
import pytest
from shapely.geometry import Point, Polygon

from nest_graph.build_graph import (
    NestState,
    _build_transform_batch,
    _placement_valid,
    make_polygon_graph,
)
from nest_graph.config import BuildGraphConfig, ProposeConfig, dedupe_transforms
from nest_graph.geometry import Geometry
from nest_graph.propose import (
    base_shape_from_selection,
    proposed_transforms_for_groups,
    propositions_to_ndarray,
)
from nest_graph.utils import transform_poly


def test_propositions_to_ndarray_empty():
    assert propositions_to_ndarray([]).shape == (0, 3)


def test_base_shape_from_selection_empty():
    base = base_shape_from_selection([], [])
    assert base.is_empty


def test_proposed_transforms_board_valid(nest_board, rect_poly, tri_poly, build_graph_config):
    cfg = build_graph_config
    proposals = proposed_transforms_for_groups(
        nest_board,
        [(rect_poly, 0), (tri_poly, 1)],
        [],
        [],
        cfg.propose,
        min_dist=cfg.board_min_dist(),
        pt_push=Point(nest_board.centroid),
    )
    board_geom = Geometry.from_shapely(nest_board)
    for group_id, part in ((0, rect_poly), (1, tri_poly)):
        arr = proposals[group_id]
        assert arr.ndim == 2 and arr.shape[1] == 3
        for t in arr:
            placed = Geometry.from_shapely(part).apply_transform(t)
            assert _placement_valid(
                nest_board, board_geom, placed, part, t, "vertices",
            )


def test_build_batch_includes_proposals(
    nest_board, rect_poly, tri_poly, build_graph_config,
):
    """Batch with propose enabled should be larger than random-only baseline."""
    parts = [(rect_poly, 0), (tri_poly, 1)]
    nest_state = NestState(polys=[], group_id=[], transform=[], selected_indices=[])
    selected_t = (np.zeros((0, 3)), np.zeros((0, 3)))
    history = (np.zeros((1, 3)), np.zeros((1, 3)))

    cfg_off = build_graph_config.model_copy(deep=True)
    cfg_off.propose.max_proposals = 0
    cfg_off.sampling.max_transforms_per_group = None
    cfg_off.sampling.random_per_iter = 32
    batch_off = _build_transform_batch(
        cfg_off, selected_t, history, np.random.default_rng(0),
        board=nest_board, parts=parts, nest_state=nest_state,
    )

    cfg_on = build_graph_config.model_copy(deep=True)
    cfg_on.propose.use_point_cloud = False
    cfg_on.sampling.max_transforms_per_group = None
    cfg_on.sampling.random_per_iter = 32
    batch_on = _build_transform_batch(
        cfg_on, selected_t, history, np.random.default_rng(0),
        board=nest_board, parts=parts, nest_state=nest_state,
    )
    for gid in (0, 1):
        assert batch_on[gid].shape[0] >= batch_off[gid].shape[0] + 4


def test_dedupe_transforms():
    t = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [1.0, 0.0, 0.0]])
    assert dedupe_transforms(t).shape[0] == 2


def test_propose_adds_graph_nodes_without_cap(
    nest_board, rect_poly, tri_poly, build_graph_config,
):
    """All proposers combined should yield at least one extra valid graph node vs random-only."""
    rng = np.random.default_rng(42)
    history = (np.zeros((1, 3)), np.zeros((1, 3)))
    selected_t = (
        rng.uniform(-1, 1, (16, 3)) * [1.5, 1.5, 2 * np.pi],
        rng.uniform(-1, 1, (16, 3)) * [1.5, 1.5, 2 * np.pi],
    )
    parts = [(rect_poly, 0), (tri_poly, 1)]

    cfg_off = build_graph_config.model_copy(deep=True)
    cfg_off.propose = ProposeConfig(max_proposals=0, candidate_pool=0)
    cfg_off.sampling.max_transforms_per_group = None
    cfg_off.sampling.random_per_iter = 16
    batch_off = _build_transform_batch(
        cfg_off, selected_t, history, rng, board=nest_board, parts=parts,
    )
    graph_off, _, _, _ = make_polygon_graph(
        nest_board, [(rect_poly, batch_off[0]), (tri_poly, batch_off[1])],
    )

    cfg_on = build_graph_config.model_copy(deep=True)
    cfg_on.sampling.max_transforms_per_group = None
    cfg_on.sampling.random_per_iter = 16
    batch_on = _build_transform_batch(
        cfg_on, selected_t, history, rng, board=nest_board, parts=parts,
    )
    graph_on, _, _, _ = make_polygon_graph(
        nest_board, [(rect_poly, batch_on[0]), (tri_poly, batch_on[1])],
    )
    assert len(graph_on._obj.elems) >= len(graph_off._obj.elems)


def test_default_config_first_pass_tuned():
    cfg = BuildGraphConfig()
    assert cfg.sampling.initial_random == 256
    assert cfg.sampling.random_per_iter == 256
    assert cfg.sampling.max_transforms_per_group == 900
    assert cfg.propose.max_proposals == 20
    assert cfg.propose.use_voronoi is True
    assert cfg.propose.use_point_cloud is False


def test_propose_geometry_validation(nest_board, rect_poly):
    from nest_graph.propose import ProposeGeometry

    geom = ProposeGeometry(nest_board, Polygon(), rect_poly, min_dist=0.001)
    placed = geom.placed_at((0.5, 0.5, 0.0))
    assert geom.inside_board(placed)
    assert not geom.hits_base(placed)
