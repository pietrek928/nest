"""Pack iteration context dataclasses."""

from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any, Callable, Sequence

from nest_graph.propose.heavy_polish import PolishBudget

if TYPE_CHECKING:
    from nest_graph.propose.selection_compose import ComposedSelection


@dataclass
class PackIterCtx:
    """One outer-iter pack profile + shared graph state."""

    graph: Any
    polys: list
    group_id: list
    transform: list
    part_areas: Sequence[float]
    part_bases: dict
    cfg: Any
    sel: Any
    propose_stats: dict
    dg: Any = None
    sheet: Any = None
    min_dist: float = 0.0
    rule_sets: list = field(default_factory=list)
    active_rules: Any = None
    scores: list = field(default_factory=list)
    free_info: Any = None
    free_poly: Any = None
    void_geoms: Sequence | None = None
    packed_geoms: list = field(default_factory=list)
    packed_group_id: Sequence | None = None
    packed_transform: Sequence | None = None
    sheet_diag: float = 0.0
    sheet_area: float = 0.0
    ngroups: int = 0
    is_last_leaf: bool = False
    near_last: bool = False
    refine_seed: int = 0
    rim_reject: float = 0.02
    first_pass: bool = False
    enable_3b: bool = True
    locked_seed: Sequence | None = None
    native_geoms_fn: Callable | None = None


@dataclass
class RefinePackBox:
    """Mutable compose+refine scratch (replaces nested closures)."""

    composed: "ComposedSelection | None" = None
    budget: PolishBudget | None = None
    selected: list = field(default_factory=list)
    coverage: float = 0.0


@dataclass
class MidPackStagesResult:
    selected_polys: list
    polys: list
    transform: list
    group_id: list
    candidate_geoms: list | None
    pin_stats: dict
    selected_nest: list
    refine_scores: list
    free_poly: Any
    free_info: Any
    n_void_nest: int
    boost_hits: dict
    polish_budget: PolishBudget


@dataclass
class FirstPassBorderResult:
    graph: Any
    polys: list
    group_id: list
    transform: list
    selected_polys: list[int]
    free_info: Any | None = None
    old_len: int = 0


@dataclass
class VoidLeakOrchResult:
    void_elite_by_group: dict
    leak_dict: dict
    void_leak: str
    had_void_override: bool
    n_void_graph: int
    outline_cov: float
    proposer_keys: dict
    niche_telem: dict
    prev_void_nest: int


__all__ = [
    "FirstPassBorderResult",
    "MidPackStagesResult",
    "PackIterCtx",
    "RefinePackBox",
    "VoidLeakOrchResult",
]
