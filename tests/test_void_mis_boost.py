"""Unit tests for void-island MIS boost experiment helpers."""

import numpy as np
from shapely.geometry import Point, box

from nest_graph.propose.void_selection import (
    boost_void_island_scores as _boost_void_island_scores,
)
from nest_graph.config import ProposeConfig
from nest_graph.utils import transform_poly


def test_gravity_compaction_enabled_by_default():
    assert ProposeConfig().enable_gravity_compaction is True
    assert ProposeConfig().void_island_score_boost == 64.0
    assert ProposeConfig().enable_void_large_hijack is True
    assert ProposeConfig().pocket_score_boost == 50.0
    assert ProposeConfig().small_part_void_score_boost == 40.0
    assert ProposeConfig().use_open_void_pocket is True
    assert ProposeConfig().enable_cluster_repack is True
    assert ProposeConfig().enable_local_se2 is True


def test_boost_keyed_and_small_part_scores():
    from nest_graph.propose.void_selection import (
        boost_keyed_proposal_scores as _boost_keyed_proposal_scores,
        boost_small_part_scores as _boost_small_part_scores,
    )

    scores = [1.0, 2.0, 3.0]
    n = _boost_keyed_proposal_scores(
        [0, 1, 0],
        [(1.0, 2.0, 0.0), (3.0, 4.0, 0.0), (1.0, 2.0, 0.1)],
        scores,
        {0: {(1.0, 2.0, 0.0)}},
        weight=50.0,
    )
    assert n == 1
    assert scores[0] == 51.0
    assert scores[1] == 2.0
    scores2 = [0.0, 0.0]
    n2 = _boost_small_part_scores([0, 1], scores2, [10.0, 2.0], weight=40.0)
    assert n2 == 1
    assert scores2[1] > scores2[0]
    assert scores2[0] == 0.0


def test_boost_void_island_scores_only_inside_free():
    free = box(5, 5, 10, 10)
    polys = [
        transform_poly(box(0, 0, 1, 1), (6.5, 6.5, 0.0)),  # inside free
        transform_poly(box(0, 0, 1, 1), (1.0, 1.0, 0.0)),  # outside
    ]
    scores = [1.0, 2.0]
    # No pole → flat factor 1.0 (legacy path).
    n = _boost_void_island_scores(polys, scores, free, weight=64.0)
    assert n == 1
    assert scores[0] == 65.0
    assert scores[1] == 2.0


def test_boost_void_island_distance_to_pole():
    free = box(0, 0, 10, 10)
    pole = Point(5.0, 5.0)
    # transform_poly shifts by translation; unit box centroid is (+0.5,+0.5).
    near = transform_poly(box(0, 0, 1, 1), (4.5, 4.5, 0.0))
    far = transform_poly(box(0, 0, 1, 1), (8.5, 8.5, 0.0))
    scores = [0.0, 0.0]
    n = _boost_void_island_scores(
        [near, far],
        scores,
        free,
        weight=64.0,
        pole=pole,
        sheet_diag=10.0 * np.sqrt(2.0),
    )
    assert n == 2
    assert scores[0] > scores[1]
    assert abs(scores[0] - 64.0) < 1e-6  # centroid on pole


def test_boost_void_island_zero_weight_noop():
    free = box(0, 0, 10, 10)
    polys = [transform_poly(box(0, 0, 1, 1), (1.0, 1.0, 0.0))]
    scores = [1.0]
    n = _boost_void_island_scores(polys, scores, free, weight=0.0)
    assert n == 0
    assert scores[0] == 1.0
