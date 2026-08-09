"""Post-flatten audit gates: full_packed reuse, ranking SoT, cascade, expand proposers."""

from unittest.mock import MagicMock, patch

import numpy as np
import pytest
from shapely import Point, Polygon, box

from nest_graph.config import ProposeConfig
from nest_graph.geometry import Geometry
from nest_graph.propose.geometry import ProposeGeometry
from nest_graph.propose.pipeline import (
    _count_fast_valid_seeds,
    _normalize_proposers,
    pre_filter_candidates,
    propose_coords_from_candidates,
)
from nest_graph.propose.placements_selection_expand import (
    propose_placements_history_expand,
    propose_placements_selection_expand,
)
from nest_graph.propose.ranking import (
    _rank_score_for_mode,
    _score_placement_border,
)
from nest_graph.proposer_names import ProposerName


def _unit_square():
    return Polygon([(0, 0), (1, 0), (1, 1), (0, 1)])


def test_pre_filter_drops_full_packed_collision():
    sheet = box(0, 0, 20, 20)
    part = _unit_square()
    packed = Geometry.from_shapely(box(9.5, 9.5, 10.5, 10.5))
    geom = ProposeGeometry(
        sheet,
        Polygon(),
        part,
        0.05,
        propose_cfg=ProposeConfig(),
        full_packed_geoms=[packed],
    )
    # Candidate centered on packed solid → placed part intersects packed.
    bad = (10.0, 10.0, 0.0)
    ok = (2.0, 2.0, 0.0)
    out = pre_filter_candidates(
        [bad, ok],
        geom,
        Point(0, 0),
        0.05,
        0.05,
        trim_by_validity=False,
    )
    keys = {(round(c[0], 4), round(c[1], 4), round(c[2], 4)) for c in out}
    assert (10.0, 10.0, 0.0) not in keys
    assert (2.0, 2.0, 0.0) in keys


def test_from_candidates_rejects_part_mismatch_and_dual_args():
    sheet = box(0, 0, 10, 10)
    part_a = _unit_square()
    part_b = Polygon([(0, 0), (0.5, 0), (0.5, 0.5), (0, 0.5)])
    geom = ProposeGeometry(sheet, Polygon(), part_a, 0.05, propose_cfg=ProposeConfig())
    with pytest.raises(AssertionError, match="part mismatch"):
        propose_coords_from_candidates(
            Polygon(),
            part_b,
            sheet,
            ProposeConfig(max_proposals=2, candidate_pool=4),
            min_dist=0.05,
            pt_push=Point(0, 0),
            candidates=[(1.0, 1.0, 0.0)],
            rank_mode="clearance",
            propose_geom=geom,
        )
    with pytest.raises(AssertionError, match="Cannot pass packed"):
        propose_coords_from_candidates(
            Polygon(),
            part_a,
            sheet,
            ProposeConfig(max_proposals=2, candidate_pool=4),
            min_dist=0.05,
            pt_push=Point(0, 0),
            candidates=[(1.0, 1.0, 0.0)],
            rank_mode="clearance",
            propose_geom=geom,
            full_packed_geoms=[Geometry.from_shapely(part_a)],
        )


def test_cast_rank_boost_via_mocked_feedback():
    sheet = box(0, 0, 10, 10)
    part = _unit_square()
    geom = ProposeGeometry(sheet, Polygon(), part, 0.1, propose_cfg=ProposeConfig())
    cfg = ProposeConfig(cast_rank_boost=0.35)
    coords = (2.0, 2.0, 0.0)

    cast_prop = MagicMock()
    cast_prop.move_type = "cast_slide"
    feedback = MagicMock()
    feedback.clearance = 1.0
    feedback.__iter__ = lambda self: iter([cast_prop])

    with patch(
        "nest_graph.propose.ranking._placement_feedback",
        return_value=feedback,
    ), patch(
        "nest_graph.propose.ranking.best_proposition",
        return_value=cast_prop,
    ), patch(
        "nest_graph.propose.ranking.is_cast_move",
        return_value=True,
    ), patch(
        "nest_graph.propose.ranking.outline_standoff_distance",
        return_value=0.1,
    ):
        with_boost = _score_placement_border(
            coords, part, geom, Point(0, 0), 0.1, cfg,
        )
        without = _score_placement_border(
            coords, part, geom, Point(0, 0), 0.1, None,
        )
    assert with_boost == pytest.approx(without + cfg.cast_rank_boost * 0.1)


def test_count_fast_valid_seeds_fail_closed_on_runtime_error():
    sheet = box(0, 0, 10, 10)
    part = _unit_square()
    geom = ProposeGeometry(sheet, Polygon(), part, 0.05, propose_cfg=ProposeConfig())
    with patch(
        "nest_graph.propose.pipeline.batch_valid_flags",
        side_effect=RuntimeError("native boom"),
    ):
        n = _count_fast_valid_seeds(
            [(1.0, 1.0, 0.0), (2.0, 2.0, 0.0)],
            geom,
            limit=2,
            pt_push=Point(0, 0),
        )
    assert n == 0


def test_normalize_proposers_enum_and_str():
    mixed = {ProposerName.BOARD_EDGE, "group_fit"}
    assert _normalize_proposers(mixed) == {"board_edge", "group_fit"}
    assert _normalize_proposers({ProposerName.GROUP_FIT}) == {"group_fit"}


def test_selection_and_history_expand_emit():
    rng = np.random.default_rng(0)
    seeds = np.asarray([[1.0, 2.0, 0.1]], dtype=np.float64)
    sel = propose_placements_selection_expand(seeds, n=1, rng=rng)
    hist = propose_placements_history_expand(seeds, n=1, rng=rng)
    assert len(sel) > 0
    assert len(hist) > 0
    assert all(len(c) == 3 for c in sel)


def test_rank_score_for_mode_receives_propose_cfg_on_border():
    """Contract: SoT border path uses propose_cfg (cast boost plumbing)."""
    sheet = box(0, 0, 10, 10)
    part = _unit_square()
    geom = ProposeGeometry(sheet, Polygon(), part, 0.1, propose_cfg=ProposeConfig())
    cfg = ProposeConfig(cast_rank_boost=0.5)
    coords = (2.0, 2.0, 0.0)
    cast_prop = MagicMock()
    cast_prop.move_type = "cast_slide"
    feedback = MagicMock()
    feedback.clearance = 1.0

    with patch(
        "nest_graph.propose.ranking._placement_feedback",
        return_value=feedback,
    ), patch(
        "nest_graph.propose.ranking.best_proposition",
        return_value=cast_prop,
    ), patch(
        "nest_graph.propose.ranking.is_cast_move",
        return_value=True,
    ), patch(
        "nest_graph.propose.ranking.outline_standoff_distance",
        return_value=0.1,
    ):
        score = _rank_score_for_mode(
            coords,
            rank_mode="border",
            base_shape=Polygon(),
            shape_to_place=part,
            boundary=sheet,
            propose_geom=geom,
            propose_cfg=cfg,
            pt_push=Point(0, 0),
            min_dist=0.1,
            focal_shape=None,
        )
        score_none = _score_placement_border(
            coords, part, geom, Point(0, 0), 0.1, None,
        )
    assert score == pytest.approx(score_none + 0.5 * 0.1)


def test_build_graph_transform_selection_is_reexport():
    import inspect
    from nest_graph.build_graph import transform_selection

    src = inspect.getsource(transform_selection)
    assert "selection_expand_arrays" in src
    assert "(0.1, 0.1, 1.5)" not in src
