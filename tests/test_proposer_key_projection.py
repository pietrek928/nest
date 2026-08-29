"""Proposer key projection + graph survivor matching (Q358)."""

import numpy as np

from nest_graph.propose.motif_keys import (
    count_proposer_on_selection,
    proposer_survivors_on_graph,
)
from nest_graph.propose.transform_batch import project_proposer_keys
from nest_graph.utils import transform_row_key


def test_project_proposer_keys_angle_snap():
    raw = {(10.0, 20.0, 0.37)}
    pk = project_proposer_keys({"cluster_copy": raw}, [[0.0, 90.0]])
    proj = pk["cluster_copy"]
    assert len(proj) >= 1
    snapped = transform_row_key(np.array([10.0, 20.0, 0.0]))
    assert snapped in proj


def test_proposer_survivors_match_graph_only():
    group_id = [0, 0, 1]
    transform = [
        (1.0, 2.0, 0.0),
        (5.0, 6.0, 0.0),
        (9.0, 9.0, 0.0),
    ]
    keys = {
        "cluster_copy": {
            transform_row_key(transform[0]),
            transform_row_key(transform[2]),
            (99.0, 99.0, 0.0),
        },
    }
    surv = proposer_survivors_on_graph(group_id, transform, keys)
    assert transform_row_key(transform[0]) in surv[0]
    assert transform_row_key(transform[2]) in surv[1]
    assert 0 not in surv or transform_row_key(transform[1]) not in surv.get(0, ())
    g, n = count_proposer_on_selection(
        group_id, transform, [0, 2], keys, "cluster_copy",
    )
    assert g == 2
    assert n == 2
