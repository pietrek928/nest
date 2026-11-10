"""P1: batch_packing_clear matches scalar emit_packing_clear (Penetrating SoT)."""

import pytest

from nest_graph.placement.pattern import (
    emit_packing_clear,
    emit_packing_clear_batch,
)
from tests.fixtures.propose_emit import (
    EMIT_CLEAR_POSE_CASES,
    EMIT_PENETRATE_CASES,
    emit_board_empty,
    emit_board_with_block,
    make_propose_geom,
)


@pytest.mark.parametrize(
    "label,x,y,th,expect",
    EMIT_CLEAR_POSE_CASES,
    ids=[c[0] for c in EMIT_CLEAR_POSE_CASES],
)
def test_emit_packing_clear_batch_matches_scalar_empty(label, x, y, th, expect):
    geom = emit_board_empty()
    batch = emit_packing_clear_batch(geom, [(x, y, th)])
    assert batch == [expect]
    assert batch[0] is emit_packing_clear(geom, (x, y, th))


@pytest.mark.parametrize(
    "label,x,y,th,expect",
    EMIT_PENETRATE_CASES,
    ids=[c[0] for c in EMIT_PENETRATE_CASES],
)
def test_emit_packing_clear_batch_matches_scalar_block(label, x, y, th, expect):
    geom = emit_board_with_block()
    batch = emit_packing_clear_batch(geom, [(x, y, th)])
    assert batch == [expect]
    assert batch[0] is emit_packing_clear(geom, (x, y, th))


def test_emit_packing_clear_batch_empty_obstacles():
    geom = make_propose_geom()
    rows = [(10.0, 10.0, 0.0), (11.0, 11.0, 0.5)]
    assert emit_packing_clear_batch(geom, rows, obstacles=[]) == [True, True]


def test_emit_packing_clear_batch_multi_row_parity():
    geom = emit_board_with_block()
    rows = [(c[1], c[2], c[3]) for c in EMIT_PENETRATE_CASES]
    batch = emit_packing_clear_batch(geom, rows)
    scalar = [emit_packing_clear(geom, r) for r in rows]
    assert batch == scalar


def test_prune_transforms_vs_packed_uses_batch():
    import numpy as np
    from shapely.geometry import box

    from nest_graph.geometry import Geometry
    from nest_graph.propose.transform_batch import prune_transforms_vs_packed

    base = Geometry.from_shapely(box(0, 0, 1, 1))
    packed = [Geometry.from_shapely(box(9, 9, 12, 12))]
    arr = np.array(
        [[5.0, 5.0, 0.0], [10.5, 10.5, 0.0], [8.0, 10.5, 0.0]],
        dtype=np.float64,
    )
    out = prune_transforms_vs_packed(arr, base, packed)
    assert out.shape[0] == 2
    assert float(out[0, 0]) == pytest.approx(5.0)
