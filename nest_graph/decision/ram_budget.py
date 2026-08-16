"""Process RSS budget bands for PoseGraph / MCTS taper (default 2GB)."""

import os
import resource
from dataclasses import dataclass


def rss_mb() -> float:
    """Linux: ru_maxrss is KB; return peak RSS in MiB."""
    usage = resource.getrusage(resource.RUSAGE_SELF)
    return float(usage.ru_maxrss) / 1024.0


def ram_budget_mb() -> float:
    return float(os.environ.get("NEST_RAM_BUDGET_MB", "2048") or 2048)


@dataclass(slots=True)
class RamBandState:
    band: str  # ok | pressure | critical
    rss_mb: float
    budget_mb: float
    graphs_window: int | None
    transforms_scale: float
    n_sims: int
    expand_frozen: bool
    force_tombstone: bool


def evaluate_ram_band(
    *,
    default_graphs_window: int = 24,
    default_n_sims: int = 4,
) -> RamBandState:
    budget = ram_budget_mb()
    rss = rss_mb()
    ratio = rss / max(budget, 1.0)
    if ratio < 0.75:
        return RamBandState(
            band="ok",
            rss_mb=rss,
            budget_mb=budget,
            graphs_window=None,
            transforms_scale=1.0,
            n_sims=int(default_n_sims),
            expand_frozen=False,
            force_tombstone=False,
        )
    if ratio < 1.0:
        return RamBandState(
            band="pressure",
            rss_mb=rss,
            budget_mb=budget,
            graphs_window=min(8, int(default_graphs_window)),
            transforms_scale=0.7,
            n_sims=max(2, int(default_n_sims) // 2),
            expand_frozen=False,
            force_tombstone=True,
        )
    return RamBandState(
        band="critical",
        rss_mb=rss,
        budget_mb=budget,
        graphs_window=1,
        transforms_scale=0.5,
        n_sims=0,
        expand_frozen=True,
        force_tombstone=True,
    )
