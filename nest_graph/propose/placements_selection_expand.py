"""Selection / history multi-scale jitter proposers (random near-seed expands).

These emit new SE2 candidates from packed selection or history rows — the same
role as other proposers, not packed-pose editors (``SelectionEditCtx``).
"""

from typing import Sequence

import numpy as np


def transforms_around(
    p: np.ndarray,
    s: tuple[float, float, float],
    n: int,
    rng: np.random.Generator,
) -> np.ndarray:
    """Jitter each row of ``p`` by scale ``s`` for ``n`` repeats."""
    if p is None or getattr(p, "shape", (0,))[0] == 0 or n <= 0:
        return np.zeros((0, 3), dtype=np.float64)
    sx, sy, sa = s
    return np.concatenate([
        p + rng.uniform(-1, 1, (p.shape[0], 3)) * [sx, sy, sa]
        for _ in range(n)
    ])


def propose_placements_selection_expand(
    seeds: np.ndarray | Sequence,
    *,
    n: int,
    rng: np.random.Generator,
) -> list[tuple[float, float, float]]:
    """Multi-scale XY/θ jitter around selection (or window) transform rows."""
    arr = np.asarray(seeds, dtype=np.float64)
    if arr.ndim != 2 or arr.shape[0] == 0 or arr.shape[1] < 3:
        return []
    parts = [
        transforms_around(arr, (0.1, 0.1, 1.5), n, rng),
        transforms_around(arr, (0.1, 0.1, 0), n, rng),
        transforms_around(arr, (0, 0, 1.5), n, rng),
        transforms_around(arr, (0.05, 0.05, 1), n, rng),
        transforms_around(arr, (0.05, 0.05, 0), n, rng),
        transforms_around(arr, (0, 0, 1), n, rng),
        transforms_around(arr, (0.01, 0.01, 0.01), n, rng),
        transforms_around(arr, (0.01, 0.01, 0), n, rng),
        transforms_around(arr, (0, 0, 0.01), n, rng),
        transforms_around(arr, (0.001, 0.001, 0.001), n, rng),
        transforms_around(arr, (0.001, 0.001, 0), n, rng),
        transforms_around(arr, (0, 0, 0.001), n, rng),
    ]
    merged = np.concatenate(parts, axis=0) if parts else np.zeros((0, 3))
    return [
        (float(r[0]), float(r[1]), float(r[2]))
        for r in merged
    ]


def propose_placements_history_expand(
    seeds: np.ndarray | Sequence,
    *,
    n: int,
    rng: np.random.Generator,
) -> list[tuple[float, float, float]]:
    """Coarser multi-scale jitter around history transform rows."""
    arr = np.asarray(seeds, dtype=np.float64)
    if arr.ndim != 2 or arr.shape[0] == 0 or arr.shape[1] < 3:
        return []
    parts = [
        transforms_around(arr, (0.05, 0.05, 0.1), n, rng),
        transforms_around(arr, (0.05, 0.05, 0), n, rng),
        transforms_around(arr, (0, 0, 0.1), n, rng),
    ]
    merged = np.concatenate(parts, axis=0) if parts else np.zeros((0, 3))
    return [
        (float(r[0]), float(r[1]), float(r[2]))
        for r in merged
    ]


def selection_expand_arrays(
    seeds: np.ndarray,
    n: int,
    rng: np.random.Generator,
):
    """Yield ndarray chunks (compat with former ``transform_selection`` generator)."""
    scales = (
        (0.1, 0.1, 1.5),
        (0.1, 0.1, 0),
        (0, 0, 1.5),
        (0.05, 0.05, 1),
        (0.05, 0.05, 0),
        (0, 0, 1),
        (0.01, 0.01, 0.01),
        (0.01, 0.01, 0),
        (0, 0, 0.01),
        (0.001, 0.001, 0.001),
        (0.001, 0.001, 0),
        (0, 0, 0.001),
    )
    for scale in scales:
        yield transforms_around(seeds, scale, n, rng)


def history_expand_arrays(
    seeds: np.ndarray,
    n: int,
    rng: np.random.Generator,
):
    """Yield ndarray chunks (compat with former ``transform_history`` generator)."""
    scales = (
        (0.05, 0.05, 0.1),
        (0.05, 0.05, 0),
        (0, 0, 0.1),
    )
    for scale in scales:
        yield transforms_around(seeds, scale, n, rng)
