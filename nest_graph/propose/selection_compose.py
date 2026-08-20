"""Compose nest/DFS selection scores and choose the nest seed.

Shared by ``build_graph`` mid-pack and ``nesting_evaluator`` so geom/void/border
policy (G22/G24) cannot drift. Propose ranking stays in ``ranking.py``.
"""

from dataclasses import dataclass, field
from typing import Any, Sequence

import numpy as np
from shapely import Point
from shapely.geometry.base import BaseGeometry

from nest_graph.elem_graph import (
    PlacementRuleSet,
    SelectMode,
    SelectOptions,
    nest_by_scores,
)
from nest_graph.geometry import Geometry
from nest_graph.propose.context import outline_coverage_ratio, should_use_border_focus
from nest_graph.propose.void_selection import (
    apply_void_centroid_score_term,
    apply_void_selection_boosts,
    boost_border_scores,
    centroid_in_free,
    colonize_void_onto_base,
    count_graph_in_free,
    count_selected_in_free,
    pose_key_to_index,
    transform_row_key,
    void_attractor_radius,
    void_core_then_rim,
    _sel_area,
)
from nest_graph.propose.block_replace import (
    _packing_independent,
    lex_count_area_better,
    try_block_cohort_swap,
)
from nest_graph.propose.motif_lock import sequential_accept_motif_cohorts
from nest_graph.propose.first_pass_border import border_kiss_indices
from nest_graph.propose.motif_keys import resolve_motif_keys
from nest_graph.board import board_context_from_geometry
from nest_graph.decision.epoch import bind_epoch
from shapely.geometry import Polygon
import math


def _motif_key_density(
    sel: Sequence[int],
    transform: Sequence,
    motif_key_set: set,
) -> float:
    """Fraction of selection poses whose transform key is in the motif set (Q187/Q198)."""
    if not sel or not motif_key_set:
        return 0.0
    hits = 0
    for i in sel:
        ii = int(i)
        if ii < 0 or ii >= len(transform):
            continue
        if transform_row_key(np.asarray(transform[ii], dtype=np.float64)) in motif_key_set:
            hits += 1
    return float(hits) / float(len(sel))


def dual_nest_for(free_info: Any, *, last_leaf: bool = False, do_heavy: bool | None = None) -> bool:
    """Q105: dual lex when last leaf OR large_void basin."""
    if do_heavy is not None:
        last_leaf = bool(do_heavy)
    if bool(last_leaf):
        return True
    return free_info is not None and getattr(free_info, "kind", None) == "large_void"


def free_basin_is_fat(
    free_poly: BaseGeometry | None,
    part_areas: Sequence[float] | None,
    *,
    erode: float = 0.04,
) -> bool:
    """True when eroded free still holds ≥2× smallest catalog part (stampable basin)."""
    if free_poly is None or getattr(free_poly, "is_empty", True):
        return False
    try:
        min_part = max(
            min((float(a) for a in (part_areas or [0.01])), default=0.01),
            0.01,
        )
        return float(free_poly.buffer(-float(erode)).area) > 2.0 * min_part
    except Exception:
        return False


def prefer_void_core_selection(
    *,
    rim_sel: Sequence[int],
    void_sel: Sequence[int],
    polys: list,
    free_poly: BaseGeometry,
    group_id: Sequence[int],
    part_areas: Sequence[float],
) -> bool:
    """Hybrid: take void-first when it wins area, or soft-keeps area with more void.

    Fat basin (early large_void): softer floors so void packs before rim ribbons
    the residual. Thin basin: require near area parity.
    """
    if not void_sel:
        return False
    rim_area = float(_sel_area(rim_sel, group_id, part_areas))
    vf_area = float(_sel_area(void_sel, group_id, part_areas))
    if vf_area > rim_area + 1e-12:
        return True
    vf_void = count_selected_in_free(polys, void_sel, free_poly)
    rim_void = count_selected_in_free(polys, rim_sel, free_poly)
    if vf_void <= rim_void:
        return False
    fat = free_basin_is_fat(free_poly, part_areas)
    if fat and vf_void >= 8 and vf_area + 1e-12 >= 0.90 * rim_area:
        return True
    area_floor = 0.90 if fat else 0.94
    count_floor = 0.85 if fat else 0.90
    return (
        vf_area + 1e-12 >= area_floor * rim_area
        and len(void_sel) >= int(count_floor * max(len(rim_sel), 1))
    )


def compose_nest_kwargs(
    *,
    graph,
    rule_sets: list,
    active_rules,
    scores: list[float],
    polys: list,
    group_id: Sequence[int],
    transform: Sequence,
    candidate_geoms: list | None,
    packed_geoms: list,
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
    last_leaf: bool = False,
    void_geoms: Sequence | None = None,
    locked_seed: Sequence[int] | None = None,
    dg=None,
) -> dict:
    """One kwargs dict for ``compose_and_nest_selection`` (Uh + mid + evaluator)."""
    return {
        "graph": graph,
        "rule_sets": rule_sets,
        "active_rules": active_rules,
        "scores": scores,
        "polys": polys,
        "group_id": group_id,
        "transform": transform,
        "candidate_geoms": candidate_geoms,
        "packed_geoms": packed_geoms,
        "part_areas": part_areas,
        "free_info": free_info,
        "cfg": cfg,
        "selection": selection,
        "first_pass": first_pass,
        "outline": outline,
        "min_dist": min_dist,
        "sheet_area": sheet_area,
        "sheet_diag": sheet_diag,
        "propose_stats": propose_stats,
        "ngroups": ngroups,
        "packed_group_id": packed_group_id,
        "packed_transform": packed_transform,
        "dual_nest": dual_nest_for(free_info, last_leaf=last_leaf),
        "void_geoms": void_geoms,
        "locked_seed": locked_seed,
        "dg": dg,
    }


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
    key_map = pose_key_to_index(group_id, transform)
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
    dg=None,
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
        dg=dg,
    )
    if geom_stats and propose_stats is not None:
        propose_stats["geom_ms"] = geom_stats.get("geom_ms", 0.0)
        propose_stats["geom_share"] = geom_stats.get("geom_share", 0.0)

    # L1: void-centroid term on nest scores before MIS (refine shares same helper).
    # Q118: deepen island boost under large_void / void_seek.
    void_scale = 2.0
    mcts_zone = str((propose_stats or {}).get("mcts_zone") or "")
    if mcts_zone == "void_seek":
        void_scale = 2.5
    if free_info is not None and getattr(free_info, "kind", None) == "large_void":
        void_scale = max(void_scale, 4.0)
    void_term = float(getattr(cfg.propose, "void_island_score_boost", 0.0) or 0.0) * void_scale
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

    # Beam void-core MIS before incumbent hold so fat-basin void packs can
    # survive S0 via the same void_override path (one prefer helper).
    if (
        free_info is not None
        and getattr(free_info, "kind", None) == "large_void"
        and free_poly is not None
        and not getattr(free_poly, "is_empty", True)
    ):
        interior_m = float(min_dist) * 0.25
        core_stats: dict = {}
        void_first = void_core_then_rim(
            graph,
            polys,
            free_poly,
            scores,
            interior_margin=interior_m,
            stats_out=core_stats,
        )
        if not void_first and interior_m > 1e-12:
            void_first = void_core_then_rim(
                graph,
                polys,
                free_poly,
                scores,
                interior_margin=0.0,
                stats_out=core_stats,
            )
        if propose_stats is not None:
            propose_stats.update(core_stats)
        if void_first and prefer_void_core_selection(
            rim_sel=selected_nest,
            void_sel=void_first,
            polys=polys,
            free_poly=free_poly,
            group_id=group_id,
            part_areas=part_areas,
        ):
            selected_nest = list(void_first)
            if propose_stats is not None:
                propose_stats["void_core_accepted"] = 1
                propose_stats["void_core_fat_free"] = int(
                    free_basin_is_fat(free_poly, part_areas)
                )
        elif propose_stats is not None:
            propose_stats["void_core_accepted"] = 0
            propose_stats["void_core_fat_free"] = int(
                free_basin_is_fat(free_poly, part_areas)
            )

    incumbent = _map_incumbent_indices(
        group_id=group_id,
        transform=transform,
        packed_group_id=packed_group_id,
        packed_transform=packed_transform,
        graph=graph,
    )
    incumbent_hold = 0
    void_override_flag = 0
    if propose_stats is not None:
        propose_stats["incumbent_mapped"] = int(len(incumbent))
    if incumbent and not _lex_pick_better(
        best=incumbent,
        cand=selected_nest,
        group_id=group_id,
        part_areas=part_areas,
    ):
        # Soft void override when MIS already fills free with near area parity.
        void_override = False
        if (
            free_info is not None
            and getattr(free_info, "kind", None) == "large_void"
            and free_poly is not None
            and not getattr(free_poly, "is_empty", True)
            and len(incumbent) > 0
        ):
            void_cand = count_selected_in_free(polys, selected_nest, free_poly)
            void_inc = count_selected_in_free(polys, incumbent, free_poly)
            void_gain = int(void_cand) - int(void_inc)
            cand_area = float(_sel_area(selected_nest, group_id, part_areas))
            inc_area = float(_sel_area(incumbent, group_id, part_areas))
            fat_free = free_basin_is_fat(free_poly, part_areas)
            if void_inc <= 2 and void_gain >= 8:
                area_ok = cand_area + 1e-12 >= (0.88 if fat_free else 0.93) * inc_area
                count_ok = len(selected_nest) >= int(0.75 * len(incumbent))
            else:
                area_floor = 0.90 if fat_free else 0.97
                area_ok = cand_area + 1e-12 >= area_floor * inc_area
                count_ok = len(selected_nest) >= int(0.9 * len(incumbent))
                if (
                    not count_ok
                    and area_ok
                    and void_gain >= 3
                    and len(selected_nest) >= int(0.85 * len(incumbent))
                ):
                    count_ok = True
            void_override = bool(void_gain > 0 and area_ok and count_ok)
            # Fat basin / void-core beam: skip hold so rim incumbent does not
            # restore a hollow pack after void-first MIS.
            if (
                not void_override
                and fat_free
                and void_gain >= 5
                and cand_area + 1e-12 >= 0.85 * inc_area
            ):
                void_override = True
            if (
                not void_override
                and int((propose_stats or {}).get("void_core_accepted", 0) or 0) > 0
                and void_gain >= 1
                and cand_area + 1e-12 >= 0.92 * inc_area
            ):
                void_override = True
            if void_override:
                drop_allow = 0.10 if fat_free else (
                    0.08 if (void_inc <= 2 and void_gain >= 8) else (
                        0.06 if void_gain >= 3 else 0.02
                    )
                )
                try:
                    cov_cand = float(outline_coverage_ratio(
                        [polys[i] for i in selected_nest if 0 <= int(i) < len(polys)],
                        outline,
                        float(min_dist),
                    ))
                    cov_inc = float(outline_coverage_ratio(
                        [polys[i] for i in incumbent if 0 <= int(i) < len(polys)],
                        outline,
                        float(min_dist),
                    ))
                    if cov_cand + 1e-9 < cov_inc - drop_allow:
                        void_override = False
                except Exception:
                    pass
            if void_override:
                void_override_flag = 1
            else:
                # Q187/Q198: motif soft override — key-hit fraction (not MotifBase GCI).
                # Q199: if incumbent has 0 key-hit (stringy rim), allow cand with dens>0.
                # Q220/Q221: must also pass void_override coverage drop_allow (OR accept).
                motif_override = False
                if bool(getattr(cfg.propose, "enable_inward_bridge", True)):
                    motif_keys_map = resolve_motif_keys(
                        propose_stats,
                        densify=(propose_stats or {}).get("densify_stats"),
                    )
                    motif_key_set: set = set()
                    for raw_set in motif_keys_map.values():
                        for raw in raw_set or ():
                            motif_key_set.add(
                                tuple(raw) if not isinstance(raw, tuple) else raw
                            )
                    if motif_key_set and transform is not None:
                        dens_cand = _motif_key_density(
                            selected_nest, transform, motif_key_set,
                        )
                        dens_inc = _motif_key_density(
                            incumbent, transform, motif_key_set,
                        )
                        count_ok = len(selected_nest) >= len(incumbent)
                        if count_ok and (
                            dens_cand > dens_inc + 1e-12
                            or (dens_inc <= 1e-12 and dens_cand > 1e-12)
                        ):
                            motif_override = True
                            drop_allow = 0.10 if fat_free else (
                                0.08 if (void_inc <= 2 and void_gain >= 8) else (
                                    0.06 if void_gain >= 3 else 0.02
                                )
                            )
                            try:
                                cov_cand = float(outline_coverage_ratio(
                                    [
                                        polys[i] for i in selected_nest
                                        if 0 <= int(i) < len(polys)
                                    ],
                                    outline,
                                    float(min_dist),
                                ))
                                cov_inc = float(outline_coverage_ratio(
                                    [
                                        polys[i] for i in incumbent
                                        if 0 <= int(i) < len(polys)
                                    ],
                                    outline,
                                    float(min_dist),
                                ))
                                if cov_cand + 1e-9 < cov_inc - drop_allow:
                                    motif_override = False
                            except Exception:
                                motif_override = False
                if motif_override:
                    void_override_flag = 1
                    if propose_stats is not None:
                        propose_stats["motif_override"] = 1
                else:
                    selected_nest = list(incumbent)
                    locked_motif = []
                    incumbent_hold = 1
                    if propose_stats is not None:
                        propose_stats["motif_override"] = 0
    # One colonize walk onto the held/MIS base (rim density + void pins).
    if (
        free_info is not None
        and getattr(free_info, "kind", None) == "large_void"
        and free_poly is not None
        and not getattr(free_poly, "is_empty", True)
    ):
        interior_m = float(min_dist) * 0.25
        void_base = count_selected_in_free(
            polys, selected_nest, free_poly, interior_margin=interior_m,
        )
        n_void_graph = count_graph_in_free(
            polys, free_poly, interior_margin=interior_m,
        )
        use_margin = interior_m
        if n_void_graph <= void_base:
            void_base = count_selected_in_free(polys, selected_nest, free_poly)
            n_void_graph = count_graph_in_free(polys, free_poly)
            use_margin = 0.0
        if n_void_graph > void_base:
            colonize_stats: dict = {}
            colonized = colonize_void_onto_base(
                graph,
                selected_nest,
                polys,
                free_poly,
                scores,
                stats_out=colonize_stats,
                interior_margin=use_margin,
                max_rim_drop=20,
                group_id=group_id,
                part_areas=part_areas,
            )
            if propose_stats is not None:
                propose_stats.update(colonize_stats)
            pinned_n = int(colonize_stats.get("colonize_pinned", 0) or 0)
            if pinned_n > 0 or len(colonized) != len(selected_nest):
                base_area = float(_sel_area(selected_nest, group_id, part_areas))
                col_area = float(_sel_area(colonized, group_id, part_areas))
                if col_area > base_area + 1e-12 or (
                    col_area + 1e-12 >= base_area
                    and len(colonized) >= len(selected_nest)
                ):
                    selected_nest = colonized
                elif propose_stats is not None:
                    propose_stats["colonize_area_reject"] = 1
            # Keep colonized free-core pins locked through refine so DFS cannot
            # shed void fill (R0 soft unify: budget/hold, not Motif→DFS).
            void_pins = [
                int(i) for i in selected_nest
                if 0 <= int(i) < len(polys)
                and centroid_in_free(
                    polys[int(i)], free_poly, interior_margin=use_margin,
                )
            ][:12]
            if void_pins:
                locked_motif = list(dict.fromkeys(list(locked_motif) + void_pins))
            # void_core already beamed before hold (prefer_void_core_selection).
            # Colonize is the rim↔void hybrid pin walk on the held/MIS base.
    if propose_stats is not None:
        propose_stats["incumbent_hold"] = int(incumbent_hold)
        propose_stats["void_override"] = int(void_override_flag)
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
    # Q175 exception: first-pass border boost is refine-only rim bias (nest already ran).
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
