"""Shared placement clearance constants (no propose/scene dependencies)."""

PLACEMENT_EPSILON_RATIO = 0.05


def placement_clearance_epsilon(
    min_dist: float,
    *,
    ratio: float = PLACEMENT_EPSILON_RATIO,
) -> float:
    return max(1e-6, min_dist * ratio)
