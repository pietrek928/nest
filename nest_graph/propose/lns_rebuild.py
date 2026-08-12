"""Ruin-and-recreate (LNS) on plateau: spatial destroy at void frontier."""

import math
from typing import Sequence

import numpy as np
from shapely.geometry import Point
from shapely.geometry.base import BaseGeometry


def void_frontier_destroy_indices(
    polys: Sequence[BaseGeometry],
    selected: Sequence[int],
    *,
    pole: Point | None,
    void_poly: BaseGeometry | None,
    destroy_fraction: float = 0.25,
    max_destroy: int = 12,
) -> list[int]:
    """Pick selected indices near the void frontier for spatial destroy."""
    if not selected:
        return []
    frac = min(max(float(destroy_fraction), 0.05), 0.75)
    n_destroy = max(1, min(int(round(len(selected) * frac)), int(max_destroy)))
    scored: list[tuple[float, int]] = []
    for idx in selected:
        if idx < 0 or idx >= len(polys):
            continue
        poly = polys[idx]
        if poly is None or getattr(poly, "is_empty", True):
            continue
        c = poly.centroid
        dist = 0.0
        if void_poly is not None and not getattr(void_poly, "is_empty", True):
            try:
                dist = float(poly.distance(void_poly))
            except Exception:
                dist = float("inf")
        elif pole is not None and not pole.is_empty:
            dist = float(math.hypot(c.x - pole.x, c.y - pole.y))
        else:
            dist = float(c.x + c.y)
        scored.append((dist, int(idx)))
    scored.sort(key=lambda x: x[0])
    return [i for _d, i in scored[:n_destroy]]


def apply_lns_destroy(
    selected: Sequence[int],
    destroy_idxs: Sequence[int],
) -> list[int]:
    """Return selection with destroyed indices removed."""
    drop = set(int(i) for i in destroy_idxs)
    return [int(i) for i in selected if int(i) not in drop]


def lns_accept(
    *,
    old_count: int,
    old_area: float,
    old_cov: float,
    new_count: int,
    new_area: float,
    new_cov: float,
    temperature: float = 0.0,
    rng: np.random.Generator | None = None,
) -> bool:
    """Accept if lex (count, area, cov) improves; SA may accept mild regressions early."""
    if new_count > old_count:
        return True
    if new_count < old_count:
        if temperature <= 0.0:
            return False
    else:
        if new_area > old_area + 1e-9:
            return True
        if new_area < old_area - 1e-9 and temperature <= 0.0:
            return False
        if new_cov >= old_cov - 1e-6:
            return True
        if temperature <= 0.0:
            return False
    # Mild regression under temperature.
    delta = (old_cov - new_cov) + 0.01 * (old_area - new_area)
    if delta <= 0.0:
        return True
    if rng is None:
        rng = np.random.default_rng()
    return bool(rng.random() < math.exp(-delta / max(temperature, 1e-6)))
