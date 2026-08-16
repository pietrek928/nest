"""Compose nest/DFS selection scores and choose the nest seed.

Shared by ``build_graph`` mid-pack and ``nesting_evaluator`` so geom/void/border
policy (G22/G24) cannot drift. Propose ranking stays in ``ranking.py``.
"""

from dataclasses import dataclass, field
from typing import Any, Sequence

from shapely import Point
from shapely.geometry.base import BaseGeometry

from nest_graph.elem_graph import (
    PlacementRuleSet,
    SelectMode,
    SelectOptions,
    nest_by_scores,
)
from nest_graph.geometry import Geometry
from nest_graph.propose.context import should_use_border_focus
from nest_graph.propose.void_selection import (
    apply_void_centroid_score_term,
    apply_void_selection_boosts,
    boost_border_scores,
    count_selected_in_free,
    transform_row_key,
    void_attractor_radius,
)
from nest_graph.propose.block_replace import (
    _packing_independent,
    _sel_area,
    lex_count_area_better,
    try_block_cohort_swap,
)
from nest_graph.propose.motif_lock import (
    _key_index_map,
    sequential_accept_motif_cohorts,
)
from nest_graph.propose.first_pass_border import border_kiss_indices
from nest_graph.board import board_context_from_geometry
from shapely.geometry import Polygon
import math


def dual_nest_for(free_info: Any, *, do_heavy: bool) -> bool:
    """Q105: dual lex when heavy leaf OR large_void basin."""
    if bool(do_heavy):
        return True
    return free_info is not None and getattr(free_info, "kind", None) == "large_void"


def kiss_lock_subset(
    polys: Sequence,
    outline: BaseGeometry,
    min_dist: float,
    candidates: Sequence[int],
    *,
    max_n: int = 3,
) -> list[int]:
    """Top kiss-outline anchors among ``candidates`` (L2: ≤3 keystones)."""
    if not candidates or max_n <= 0:
        return []
    cand_set = {int(i) for i in candidates}
    kiss = [
        int(i)
        for i in border_kiss_indices(list(polys), outline, float(min_dist))
        if int(i) in cand_set
    ]
    return kiss[: max(0, int(max_n))]


def sheet_diag_from(sheet) -> float:
    if sheet is None or getattr(sheet, "is_empty", True):
        return 0.0
    minx, miny, maxx, maxy = sheet.bounds
    return float(math.hypot(maxx - minx, maxy - miny))


def _map_incumbent_indices(
    *,
    group_id: Sequence[int],
    transform: Sequence,
    packed_group_id: Sequence[int] | None,
    packed_transform: Sequence | None,
    graph,
) -> list[int]:
    """Map last packed (gid, key) into this graph; empty if not packing-independent."""
    if not packed_group_id or packed_transform is None:
        return []
    key_map = _key_index_map(group_id, transform)
    idxs: list[int] = []
    for gid, tr in zip(packed_group_id, packed_transform, strict=False):
        ix = key_map.get((int(gid), transform_row_key(tr)))
        if ix is not None:
            idxs.append(int(ix))
    if not idxs or not _packing_independent(idxs, graph):
        return []
    return idxs


def _nest_with_locks(
    graph,
    scores: Sequence[float],
    locked: Sequence[int],
    *,
    group_id: Sequence[int] | None = None,
    part_areas: Sequence[float] | None = None,
    dual: bool = True,
) -> list[int]:
    nest_opts = SelectOptions()
    if locked:
        nest_opts.local_swap = False
        nest_opts.mode = SelectMode.greedy_score
        nest_opts.locked_indices = [int(i) for i in locked]
        return list(nest_by_scores(graph, scores, nest_opts)) if scores else []
    # Q105: dual=False → cheap local_swap=False; dual=True → lex on/off (heavy OR large_void).
    if not dual:
        nest_opts.local_swap = False
        nest_opts.mode = SelectMode.greedy_score
        return list(nest_by_scores(graph, scores, nest_opts)) if scores else []
    nest_opts.local_swap = True
    cand_on = list(nest_by_scores(graph, scores, nest_opts)) if scores else []
    if group_id is None or part_areas is None:
        return cand_on
    nest_off = SelectOptions()
    nest_off.local_swap = False
    cand_off = list(nest_by_scores(graph, scores, nest_off)) if scores else []
    if _lex_pick_better(
        best=cand_on,
        cand=cand_off,
        group_id=group_id,
        part_areas=part_areas,
    ):
        return cand_off
    return cand_on


def nest_border_kiss_selection(
    graph,
    polys: Sequence,
    outline: BaseGeometry,
    min_dist: float,
    scores: Sequence[float],
    *,
    locked: Sequence[int] | None = None,
) -> list[int]:
    """Greedy MIS on outline-kiss nodes; optional locks (Ua: first+mid kiss SoT)."""
    border = set(border_kiss_indices(polys, outline, min_dist))
    n = len(scores)
    nest_scores = [0.0] * n
    for i in border:
        if 0 <= i < n:
            nest_scores[i] = float(scores[i])
    locked_list = [int(i) for i in (locked or ()) if 0 <= int(i) < n]
    for i in locked_list:
        nest_scores[i] = max(nest_scores[i], float(scores[i]), 1.0)
    if not border and not locked_list:
        return []
    return _nest_with_locks(graph, nest_scores, locked_list, dual=False)


def _lex_pick_better(
    *,
    best: Sequence[int],
    cand: Sequence[int],
    group_id: Sequence[int],
    part_areas: Sequence[float],
) -> bool:
    return lex_count_area_better(
        old_count=len(best),
        old_area=_sel_area(best, group_id, part_areas),
        new_count=len(cand),
        new_area=_sel_area(cand, group_id, part_areas),
    )


@dataclass
class ComposedSelection:
    """Result of score compose + nest seed (before DFS / pin)."""

    scores: list[float]
    refine_scores: list[float]
    selected_nest: list[int]
    nest_rules: list
    refine_rules: Any
    free_info: Any
    free_poly: Any
    sheet_diag: float
    geom_w: float
    use_nest_by_scores: bool
    boost_hits: dict[str, int] = field(default_factory=dict)
    geom_stats: dict = field(default_factory=dict)
    void_r: float = 0.0
    n_void_nest: int = 0


def active_rule_set(rule_sets: list[PlacementRuleSet]) -> PlacementRuleSet:
    if not rule_sets:
        return PlacementRuleSet()
    return rule_sets[0]


def compose_and_nest_selection(
    *,
    graph,
    rule_sets: list,
    active_rules,
    scores: list[float],
    polys: list,
    group_id: Sequence[int],
    transform: Sequence,
    candidate_geoms: list[Geometry] | None,
    packed_geoms: list[Geometry],
    part_areas: Sequence[float],
    free_info,
    cfg,
    selection,
    first_pass: bool,
    outline: BaseGeometry,
    min_dist: float,
    sheet_area: float,
    sheet_diag: float,
    propose_stats: dict | None,
    ngroups: int,
    packed_group_id: Sequence[int] | None = None,
    packed_transform: Sequence | None = None,
    dual_nest: bool = True,
    void_geoms: Sequence | None = None,
    locked_seed: Sequence[int] | None = None,
) -> ComposedSelection:
    """Apply void/geom boosts, pick nest seed, prepare refine_scores (G22/G24).

    ``dual_nest`` (Q105): False → cheap ``local_swap=False``; True → lex on/off.
    build_graph sets True on heavy leaf or large_void. ``void_geoms`` from board
    prep (Ua); ``locked_seed`` optional kiss/motif locks — unlocked dual is always
    beamed when ``dual_nest`` (L2; locks kill local_swap inside one nest call).
    """
    free_poly = free_info.target_poly
    nest_rules = rule_sets
    refine_rules = active_rules
    scores = list(scores)
    sel = selection
    geom_w = float(getattr(cfg.propose, "selection_geom_weight", 0.0) or 0.0)
    void_r = void_attractor_radius(
        min_dist, sheet_diag, cfg.rules.place_rule_radius,
    )
    use_nest_by_scores = bool(scores)

    geom_stats: dict = {}
    boost_hits = apply_void_selection_boosts(
        polys=polys,
        group_id=group_id,
        transform=transform,
        scores=scores,
        free_info=free_info,
        free_poly=free_poly,
        part_areas=part_areas,
        propose_stats=propose_stats,
        cfg=cfg,
        sheet_diag=sheet_diag,
        void_r=void_r,
        candidate_geoms=candidate_geoms if geom_w > 0.0 else None,
        packed_geoms=packed_geoms,
        outline=outline,
        min_dist=min_dist,
        sheet_area=sheet_area,
        geom_stats_out=geom_stats,
    )
    if geom_stats and propose_stats is not None:
        propose_stats["geom_ms"] = geom_stats.get("geom_ms", 0.0)
        propose_stats["geom_share"] = geom_stats.get("geom_share", 0.0)

    # L1: void-centroid term on nest scores before MIS (refine shares same helper).
    void_term = float(getattr(cfg.propose, "void_island_score_boost", 0.0) or 0.0) * 0.25
    nest_void_hits = apply_void_centroid_score_term(
        polys,
        scores,
        free_info=free_info,
        free_poly=free_poly,
        void_term=void_term,
    )
    if propose_stats is not None:
        propose_stats["nest_void_term_hits"] = int(nest_void_hits)

    n_graph = len(graph.elems) if hasattr(graph, "elems") else len(getattr(graph, "collisions", []))
    if len(scores) != n_graph:
        raise AssertionError(
            f"selection scores length {len(scores)} != graph size {n_graph}"
        )

    locked_motif: list[int] = []
    lock_sets: list[list[int]] = []
    void_geoms_list: list = list(void_geoms) if void_geoms is not None else []
    if bool(getattr(cfg.propose, "enable_motif_sequential_accept", False)):
        cohorts = (propose_stats or {}).get("motif_cohorts") or []
        if not void_geoms_list:
            try:
                _sheet, void_geoms_list = board_context_from_geometry(outline)
                void_geoms_list = list(void_geoms_list or [])
            except Exception:
                void_geoms_list = []
        _combined, seq_telem = sequential_accept_motif_cohorts(
            graph=graph,
            scores=scores,
            group_id=group_id,
            transform=transform,
            cohorts=cohorts,
            candidate_geoms=candidate_geoms,
            void_geoms=void_geoms_list,
            packed_geoms=packed_geoms,
            min_dist=float(min_dist),
            pole=getattr(free_info, "target_pt", None),
            max_accept=int(
                getattr(cfg.propose, "motif_sequential_accept_max", 3) or 3
            ),
            rcl_top_k=10,
        )
        del _combined
        lock_sets = [list(s) for s in (seq_telem.get("motif_lock_sets") or [])][:4]
        boost_hits = dict(boost_hits)
        boost_hits["motif_sequential"] = int(seq_telem.get("motif_sequential_full", 0))
        if propose_stats is not None:
            propose_stats.update(seq_telem)

    seed_lock = [int(i) for i in (locked_seed or ()) if 0 <= int(i) < n_graph]
    # L2: always beam unlocked dual when dual_nest (locks kill dual inside one call).
    selected_nest = _nest_with_locks(
        graph,
        scores,
        [],
        group_id=group_id,
        part_areas=part_areas,
        dual=bool(dual_nest),
    )
    beam_unlocked = 1
    if seed_lock:
        locked_cand = _nest_with_locks(
            graph,
            scores,
            seed_lock,
            group_id=group_id,
            part_areas=part_areas,
            dual=bool(dual_nest),
        )
        if _lex_pick_better(
            best=selected_nest,
            cand=locked_cand,
            group_id=group_id,
            part_areas=part_areas,
        ):
            selected_nest = locked_cand
            locked_motif = list(seed_lock)
    for lock in lock_sets:
        cand = _nest_with_locks(
            graph,
            scores,
            lock,
            group_id=group_id,
            part_areas=part_areas,
            dual=bool(dual_nest),
        )
        if _lex_pick_better(
            best=selected_nest,
            cand=cand,
            group_id=group_id,
            part_areas=part_areas,
        ):
            selected_nest = cand
            locked_motif = list(lock)

    incumbent = _map_incumbent_indices(
        group_id=group_id,
        transform=transform,
        packed_group_id=packed_group_id,
        packed_transform=packed_transform,
        graph=graph,
    )
    incumbent_hold = 0
    if incumbent and not _lex_pick_better(
        best=incumbent,
        cand=selected_nest,
        group_id=group_id,
        part_areas=part_areas,
    ):
        # Void override: allow cand when it colonizes more free space and is
        # not a collapse (count ≥ 0.9× incumbent).
        void_override = False
        if (
            free_info is not None
            and getattr(free_info, "kind", None) == "large_void"
            and free_poly is not None
            and not getattr(free_poly, "is_empty", True)
            and len(incumbent) > 0
            and len(selected_nest) >= int(0.9 * len(incumbent))
        ):
            void_cand = count_selected_in_free(polys, selected_nest, free_poly)
            void_inc = count_selected_in_free(polys, incumbent, free_poly)
            void_override = void_cand > void_inc
        if not void_override:
            selected_nest = list(incumbent)
            locked_motif = []
            incumbent_hold = 1
    if propose_stats is not None:
        propose_stats["incumbent_hold"] = int(incumbent_hold)
        propose_stats["motif_locked"] = list(locked_motif)
        propose_stats["motif_beam_sets"] = int(len(lock_sets))
        propose_stats["uh_beam_unlocked"] = int(beam_unlocked)
        propose_stats["compose_beam_unlocked"] = int(beam_unlocked)

    if (
        not first_pass
        and bool(getattr(cfg.propose, "enable_block_replace", True))
        and locked_motif
    ):
        void_geoms_swap = void_geoms_list
        selected_nest, locked_motif, swap_telem = try_block_cohort_swap(
            graph=graph,
            scores=scores,
            selected=selected_nest,
            locked_motif=locked_motif,
            cohorts=(propose_stats or {}).get("motif_cohorts") or [],
            candidate_geoms=candidate_geoms,
            void_geoms=void_geoms_swap,
            group_id=group_id,
            transform=transform,
            part_areas=part_areas,
            min_dist=float(min_dist),
        )
        boost_hits = dict(boost_hits)
        boost_hits["block_cohort_accepted"] = int(swap_telem.get("block_cohort_accepted", 0))
        if propose_stats is not None:
            propose_stats.update(swap_telem)
            propose_stats["motif_locked"] = list(locked_motif)


    refine_scores = list(scores)
    # Term already on nest scores under large_void; refine inherits via copy.
    if (
        first_pass
        and should_use_border_focus(Polygon(), cfg.propose)
        and free_info.kind != "large_void"
        and geom_w <= 0.0  # G22: skip border boost when selection geom on
    ):
        boost_border_scores(
            polys, refine_scores, outline, min_dist,
            weight=cfg.propose.border_selection_score_boost,
        )

    return ComposedSelection(
        scores=scores,
        refine_scores=refine_scores,
        selected_nest=selected_nest,
        nest_rules=nest_rules,
        refine_rules=refine_rules,
        free_info=free_info,
        free_poly=free_poly,
        sheet_diag=sheet_diag,
        geom_w=geom_w,
        use_nest_by_scores=use_nest_by_scores,
        boost_hits=boost_hits,
        geom_stats=geom_stats,
        void_r=void_r,
        n_void_nest=count_selected_in_free(polys, selected_nest, free_poly),
    )
