"""Recency-ordered history trim and graph-valid carry caps."""

import numpy as np

from nest_graph.config import cap_graph_valid_carry, trim_history
from nest_graph.propose.transform_batch import graph_valid_carry_by_group as _graph_valid_carry_by_group


def test_trim_history_keeps_newest_selected_not_lexicographic_tail():
    # Lexicographic unique would prefer high-x rows; recency must keep selected.
    history = np.asarray(
        [
            [9.0, 0.0, 0.0],
            [8.0, 0.0, 0.0],
            [0.1, 0.0, 0.0],
            [0.2, 0.0, 0.0],
        ],
        dtype=np.float64,
    )
    selected = np.asarray([[0.05, 0.0, 0.0], [0.06, 0.0, 0.0]], dtype=np.float64)
    out = trim_history(history, selected, history_max=3)
    assert out.shape == (3, 3)
    keys = {(round(r[0], 4), round(r[1], 4), round(r[2], 4)) for r in out}
    assert (0.05, 0.0, 0.0) in keys
    assert (0.06, 0.0, 0.0) in keys
    # Oldest low-priority history dropped before selected.
    assert (0.2, 0.0, 0.0) not in keys or (0.1, 0.0, 0.0) not in keys


def test_trim_history_dedupes_round4_and_prefers_selected_row():
    history = np.asarray([[1.00004, 2.0, 0.0]], dtype=np.float64)
    selected = np.asarray([[1.00001, 2.0, 0.0]], dtype=np.float64)
    out = trim_history(history, selected, history_max=8)
    assert out.shape == (1, 3)
    # Selected row wins when round-4 keys collide.
    assert abs(float(out[0, 0]) - 1.00001) < 1e-9


def test_cap_graph_valid_carry_hard_cap_and_dedupe():
    rows = np.asarray(
        [[float(i), 0.0, 0.0] for i in range(20)]
        + [[0.0, 0.0, 0.0]],
        dtype=np.float64,
    )
    out = cap_graph_valid_carry(rows, max_keep=5)
    assert out.shape[0] == 5
    assert float(out[0, 0]) == 0.0


def test_graph_valid_carry_by_group_buckets():
    group_id = [0, 1, 0, 1]
    transform = [
        (0.0, 0.0, 0.0),
        (1.0, 0.0, 0.0),
        (0.1, 0.0, 0.0),
        (1.1, 0.0, 0.0),
    ]
    carry = _graph_valid_carry_by_group(group_id, transform, ngroups=2, max_keep=10)
    assert len(carry) == 2
    assert carry[0].shape[0] == 2
    assert carry[1].shape[0] == 2
