"""Selection / history multi-scale jitter proposers (random near-seed expands).

These emit new SE2 candidates from packed selection or history rows — the same
role as other proposers, not packed-pose editors (``SelectionEditCtx``).
"""

from typing import Sequence

import numpy as np

# One SoT for collect emit + mixer ``transform_selection`` / ``transform_history``.
SELECTION_EXPAND_SCALES: tuple[tuple[float, float, float], ...] = (
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

HISTORY_EXPAND_SCALES: tuple[tuple[float, float, float], ...] = (
    (0.05, 0.05, 0.1),
    (0.05, 0.05, 0),
    (0, 0, 0.1),
)


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


def interleave_scale_chunks(chunks: Sequence[np.ndarray]) -> np.ndarray:
    """Round-robin rows across scale chunks so a head-cap keeps fine jitter."""
    cleaned = [np.asarray(c, dtype=np.float64) for c in chunks if getattr(c, "shape", (0,))[0] > 0]
    if not cleaned:
        return np.zeros((0, 3), dtype=np.float64)
    if len(cleaned) == 1:
        return cleaned[0]
    max_n = max(int(c.shape[0]) for c in cleaned)
    rows: list[np.ndarray] = []
    for i in range(max_n):
        for c in cleaned:
            if i < int(c.shape[0]):
                rows.append(c[i])
    return np.stack(rows, axis=0) if rows else np.zeros((0, 3), dtype=np.float64)


def _expand_rows(
    seeds: np.ndarray,
    scales: Sequence[tuple[float, float, float]],
    n: int,
    rng: np.random.Generator,
    *,
    interleave: bool,
) -> np.ndarray:
    chunks = [transforms_around(seeds, scale, n, rng) for scale in scales]
    if interleave:
        return interleave_scale_chunks(chunks)
    parts = [c for c in chunks if c.shape[0] > 0]
    return np.concatenate(parts, axis=0) if parts else np.zeros((0, 3), dtype=np.float64)


def _append_unique(
    out: list[tuple[float, float, float]],
    seen: set[tuple[float, float, float]],
    rows: np.ndarray,
    *,
    limit: int,
) -> None:
    for r in rows:
        if len(out) >= limit:
            return
        t = (float(r[0]), float(r[1]), float(r[2]))
        if t in seen:
            continue
        seen.add(t)
        out.append(t)


def stratified_expand_tuples(
    seeds: np.ndarray | Sequence,
    scales: Sequence[tuple[float, float, float]],
    *,
    n: int,
    rng: np.random.Generator,
    max_items: int | None = None,
    coarse_scales: int = 3,
) -> list[tuple[float, float, float]]:
    """Coarse-first fill; tiny fine tail when saturated; interleave remainder if not.

    Late packs have many seeds so the head-cap is almost pure coarse (S0 winner).
    Inject only ``max(2, cap//8)`` fine (0.01/0.001) slots when saturated; when
    coarse undershoots the cap, fill the rest from interleaved all-scales.
    """
    arr = np.asarray(seeds, dtype=np.float64)
    if arr.ndim != 2 or arr.shape[0] == 0 or arr.shape[1] < 3:
        return []
    n_coarse = max(1, min(int(coarse_scales), len(scales)))
    coarse = _expand_rows(arr, scales[:n_coarse], n, rng, interleave=False)
    if max_items is None or int(max_items) <= 0:
        interleaved = _expand_rows(arr, scales, n, rng, interleave=True)
        if coarse.shape[0] and interleaved.shape[0]:
            merged = np.concatenate([coarse, interleaved], axis=0)
        else:
            merged = coarse if coarse.shape[0] else interleaved
        return [(float(r[0]), float(r[1]), float(r[2])) for r in merged]

    cap = int(max_items)
    out: list[tuple[float, float, float]] = []
    seen: set[tuple[float, float, float]] = set()
    _append_unique(out, seen, coarse, limit=cap)

    if len(out) >= cap:
        # Saturated: keep coarse head, swap a small fine reserve into the tail.
        fine_n = min(max(2, cap // 8), cap)
        fine_scales = scales[6:] if len(scales) > 6 else scales[n_coarse:]
        if not fine_scales:
            return out[:cap]
        fine = _expand_rows(arr, fine_scales, n, rng, interleave=True)
        keep = out[: max(0, cap - fine_n)]
        seen_k = set(keep)
        _append_unique(keep, seen_k, fine, limit=cap)
        return keep

    interleaved = _expand_rows(arr, scales, n, rng, interleave=True)
    _append_unique(out, seen, interleaved, limit=cap)
    return out


def propose_placements_selection_expand(
    seeds: np.ndarray | Sequence,
    *,
    n: int,
    rng: np.random.Generator,
    max_items: int | None = None,
) -> list[tuple[float, float, float]]:
    """Multi-scale XY/θ jitter around selection (or window) transform rows."""
    return stratified_expand_tuples(
        seeds,
        SELECTION_EXPAND_SCALES,
        n=n,
        rng=rng,
        max_items=max_items,
    )


def propose_placements_history_expand(
    seeds: np.ndarray | Sequence,
    *,
    n: int,
    rng: np.random.Generator,
    max_items: int | None = None,
) -> list[tuple[float, float, float]]:
    """Coarser multi-scale jitter around history transform rows."""
    return stratified_expand_tuples(
        seeds,
        HISTORY_EXPAND_SCALES,
        n=n,
        rng=rng,
        max_items=max_items,
        coarse_scales=min(2, len(HISTORY_EXPAND_SCALES)),
    )


def selection_expand_arrays(
    seeds: np.ndarray,
    n: int,
    rng: np.random.Generator,
):
    """Yield ndarray chunks (compat with former ``transform_selection`` generator)."""
    for scale in SELECTION_EXPAND_SCALES:
        yield transforms_around(seeds, scale, n, rng)


def history_expand_arrays(
    seeds: np.ndarray,
    n: int,
    rng: np.random.Generator,
):
    """Yield ndarray chunks (compat with former ``transform_history`` generator)."""
    for scale in HISTORY_EXPAND_SCALES:
        yield transforms_around(seeds, scale, n, rng)
