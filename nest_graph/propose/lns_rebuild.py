"""LNS accept helper. Void-kNN destroy was replaced by 3b contact-CC re-nest."""

import math
from typing import Sequence

import numpy as np


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
    delta = (old_cov - new_cov) + 0.01 * (old_area - new_area)
    if delta <= 0.0:
        return True
    if rng is None:
        rng = np.random.default_rng()
    return bool(rng.random() < math.exp(-delta / max(temperature, 1e-6)))
