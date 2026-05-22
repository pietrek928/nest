import numpy as np

from nest_graph.build_graph import transform_shuffle_mix
from nest_graph.config import shuffle_transforms


def test_shuffle_transforms_changes_order():
    rng = np.random.default_rng(0)
    t = np.arange(12, dtype=float).reshape(4, 3)
    out = shuffle_transforms(t, rng)
    assert out.shape == t.shape
    assert {tuple(row) for row in out} == {tuple(row) for row in t}
    assert not np.array_equal(out, t)


def test_transform_shuffle_mix_from_selection():
    rng = np.random.default_rng(1)
    sel = np.array([[0.1, 0.2, 0.3], [0.4, 0.5, 0.6]])
    hist = np.array([[0.0, 0.0, 0.0]])
    out = transform_shuffle_mix(sel, hist, 8, rng, (0.1, 0.1, 0.2))
    assert out.shape == (8, 3)
