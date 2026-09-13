"""Compose nest/DFS selection scores and choose the nest seed.

Shared by ``build_graph`` mid-pack and ``nesting_evaluator`` so geom/void/border
policy (G22/G24) cannot drift. Propose ranking stays in ``ranking.py``.
"""

from dataclasses import dataclass, field
from typing import Any, Sequence

import numpy as np
from shapely import Point
from shapely.geometry.base import BaseGeometry

from nest_graph.graph import (
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
    apply_motif_packing_score_steer,
    boost_border_scores,
    colonize_void_onto_base,
    count_proposer_on_selection,
    count_graph_in_free,
    count_selected_in_free,
    FreeCentroidPredicate,
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
from nest_graph.propose.motif_lock import (
    compose_motif_pipeline,
    hybrid_compose_pick,
    hollow_cc_score_steer,
    motif_join_lock_sets,
    pin_if_in_nest,
)
from nest_graph.propose.first_pass_border import border_kiss_indices
from nest_graph.propose.motif_keys import merge_motif_cohorts, resolve_motif_keys
from nest_graph.board import board_context_from_geometry
from nest_graph.pack.epoch import bind_epoch
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


def active_rule_set(
    rule_sets: list[PlacementRuleSet],
    rule_id: int = 0,
) -> PlacementRuleSet:
    if not rule_sets:
        return PlacementRuleSet()
    rid = int(rule_id)
    if 0 <= rid < len(rule_sets):
        return rule_sets[rid]
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
    on_plateau = bool((propose_stats or {}).get("on_plateau", False))
    if on_plateau and mcts_zone == "void_seek":
        void_scale = max(void_scale, 5.0)
    if on_plateau and free_info is not None and getattr(free_info, "kind", None) == "large_void":
        void_scale = max(void_scale, 5.0)
    void_scale = min(float(void_scale), 5.0)
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
        propose_stats["void_scale_applied"] = float(void_scale)

    n_graph = len(graph.elems) if hasattr(graph, "elems") else len(getattr(graph, "collisions", []))
    if len(scores) != n_graph:
        raise AssertionError(
            f"selection scores length {len(scores)} != graph size {n_graph}"
        )

    locked_motif: list[int] = []
    lock_sets: list[list[int]] = []
    void_geoms_list: list = list(void_geoms) if void_geoms is not None else []
    merged_cohorts = merge_motif_cohorts(
        (propose_stats or {}).get("motif_cohorts"),
        ((propose_stats or {}).get("densify_stats") or {}).get("motif_cohorts"),
    )
    if bool(getattr(cfg.propose, "enable_motif_sequential_accept", False)):
        if not void_geoms_list:
            try:
                _sheet, void_geoms_list = board_context_from_geometry(outline)
                void_geoms_list = list(void_geoms_list or [])
            except Exception:
                void_geoms_list = []
        pre_result = compose_motif_pipeline(
            "pre_nest",
            graph=graph,
            scores=scores,
            group_id=group_id,
            transform=transform,
            cohorts=merged_cohorts,
            candidate_geoms=candidate_geoms,
            void_geoms=void_geoms_list,
            packed_geoms=packed_geoms,
            min_dist=float(min_dist),
            pole=getattr(free_info, "target_pt", None),
            max_accept=int(
                getattr(cfg.propose, "motif_sequential_accept_max", 3) or 3
            ),
            rcl_top_k=10,
            large_void=bool(
                free_info is not None and getattr(free_info, "kind", None) == "large_void"
            ),
        )
        lock_sets = list(pre_result.lock_sets)
        boost_hits = dict(boost_hits)
        boost_hits["motif_sequential"] = int(
            pre_result.telem.get("motif_sequential_full", 0)
        )
        if propose_stats is not None:
            propose_stats.update(pre_result.telem)
            boost_idxs = pre_result.boost_idxs
            motif_w = float(getattr(cfg.propose, "motif_score_boost", 0.0) or 0.0)
            if boost_idxs and motif_w > 0.0:
                n_boost = apply_motif_packing_score_steer(
                    scores, boost_idxs, motif_weight=motif_w,
                )
                propose_stats["motif_packing_score_boost_n"] = int(n_boost)
                boost_hits["motif_packing_soft"] = int(n_boost)

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
    motif_beam_trials = 0
    motif_beam_wins = 0
    beamed_sigs: set[tuple[int, ...]] = set()
    pk_merged: dict[str, set[tuple[float, float, float]]] = {}
    if propose_stats is not None:
        pk = propose_stats.get("proposer_keys") or {}
        dens_pk = (propose_stats.get("densify_stats") or {}).get("proposer_keys") or {}
        pk_merged = dict(dens_pk)
        for name, keys in pk.items():
            pk_merged.setdefault(name, set()).update(keys or ())

    def _beam_locks(locks: Sequence[Sequence[int]]) -> None:
        nonlocal selected_nest, locked_motif, motif_beam_trials, motif_beam_wins
        for lock in locks:
            sig = tuple(sorted(int(i) for i in lock))
            if len(sig) < 2 or sig in beamed_sigs:
                continue
            beamed_sigs.add(sig)
            motif_beam_trials += 1
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
                motif_beam_wins += 1
                if propose_stats is not None:
                    propose_stats["motif_lock_source"] = "beam"

    _beam_locks(lock_sets)
    if propose_stats is not None:
        union_raw = propose_stats.get("motif_union_lock_idxs")
        if (
            isinstance(union_raw, (list, tuple))
            and len(union_raw) >= 2
            and locked_motif
            and tuple(sorted(int(i) for i in locked_motif))
            == tuple(sorted(int(i) for i in union_raw))
        ):
            propose_stats["motif_union_beam_win"] = 1

    graph_to_nest_hollow = False
    nest_void_ratio = 1.0
    if (
        free_info is not None
        and getattr(free_info, "kind", None) == "large_void"
        and free_poly is not None
        and not getattr(free_poly, "is_empty", True)
    ):
        n_void_graph_pre = count_graph_in_free(polys, free_poly)
        n_void_nest_pre = count_selected_in_free(polys, selected_nest, free_poly)
        if n_void_graph_pre > 0:
            nest_void_ratio = float(n_void_nest_pre) / float(n_void_graph_pre)
        graph_to_nest_hollow = bool(
            n_void_graph_pre > 20
            and n_void_nest_pre <= max(1, int(0.25 * n_void_graph_pre))
        )
        if propose_stats is not None:
            propose_stats["nest_void_ratio"] = float(nest_void_ratio)
            propose_stats["graph_to_nest_hollow"] = int(graph_to_nest_hollow)
        if pk_merged:
            cc_g, cc_n = count_proposer_on_selection(
                group_id, transform, selected_nest, pk_merged, "cluster_copy",
            )
            if propose_stats is not None:
                propose_stats["cluster_copy_graph_n"] = int(cc_g)
                propose_stats["cluster_copy_nest_n"] = int(cc_n)
            if graph_to_nest_hollow and not void_geoms_list:
                try:
                    _sheet, void_geoms_list = board_context_from_geometry(outline)
                    void_geoms_list = list(void_geoms_list or [])
                except Exception:
                    void_geoms_list = []
            post_result = compose_motif_pipeline(
                "post_hollow",
                graph=graph,
                scores=scores,
                group_id=group_id,
                transform=transform,
                cohorts=merged_cohorts,
                candidate_geoms=candidate_geoms,
                void_geoms=void_geoms_list,
                packed_geoms=packed_geoms,
                min_dist=float(min_dist),
                selected_nest=selected_nest,
                polys=polys,
                sheet=outline,
                proposer_keys=pk_merged,
                cc_graph_n=int(cc_g),
                cc_nest_n=int(cc_n),
                graph_to_nest_hollow=graph_to_nest_hollow,
                pole=getattr(free_info, "target_pt", None),
                free_poly=free_poly,
            )
            hollow_locks = post_result.lock_sets
            hollow_telem = post_result.telem
            if propose_stats is not None:
                propose_stats.update(hollow_telem)
            join_lock_fallback = False
            if not hollow_locks and dg is not None:
                hollow_locks = motif_join_lock_sets(dg, graph, max_locks=4)
                join_lock_fallback = bool(hollow_locks)
                if hollow_locks and propose_stats is not None:
                    propose_stats["hollow_join_lock_sets"] = int(len(hollow_locks))
            cc_pair_won = False
            pick_telem: dict = {}
            void_scene_locks = bool(
                (hollow_telem or {}).get("hollow_lock_void_hit", 0)
            )
            # M1: soft-try sequential union under hybrid_compose_pick even if lex beam lost.
            if (
                graph_to_nest_hollow
                and propose_stats is not None
                and not locked_motif
            ):
                union_raw = propose_stats.get("motif_union_lock_idxs")
                if isinstance(union_raw, (list, tuple)) and len(union_raw) >= 2:
                    union_lock = [int(i) for i in union_raw]
                    union_sig = tuple(sorted(union_lock))
                    propose_stats["motif_union_hollow_tried"] = int(
                        propose_stats.get("motif_union_hollow_tried", 0) or 0
                    ) + 1
                    if union_sig in beamed_sigs:
                        propose_stats["motif_union_hollow_skip_beamed"] = int(
                            propose_stats.get("motif_union_hollow_skip_beamed", 0) or 0
                        ) + 1
                    a_orig_u = float(_sel_area(selected_nest, group_id, part_areas))
                    vf_orig_u = count_selected_in_free(
                        polys, selected_nest, free_poly,
                    )
                    score_use_u = list(scores)
                    if void_scene_locks and void_term > 0.0:
                        for i in union_lock:
                            ix = int(i)
                            if 0 <= ix < len(score_use_u):
                                score_use_u[ix] = float(score_use_u[ix]) + float(void_term)
                    cc_nest_u = _nest_with_locks(
                        graph,
                        score_use_u,
                        union_lock,
                        group_id=group_id,
                        part_areas=part_areas,
                        dual=bool(dual_nest),
                    )
                    _cg2_u, cc_n2_u = count_proposer_on_selection(
                        group_id, transform, cc_nest_u, pk_merged, "cluster_copy",
                    )
                    a_cc_u = float(_sel_area(cc_nest_u, group_id, part_areas))
                    vf_cc_u = count_selected_in_free(
                        polys, cc_nest_u, free_poly,
                    )
                    lex_win_u = _lex_pick_better(
                        best=selected_nest,
                        cand=cc_nest_u,
                        group_id=group_id,
                        part_areas=part_areas,
                    )
                    if hybrid_compose_pick(
                        graph=graph,
                        lock=union_lock,
                        area_cand=a_cc_u,
                        area_orig=a_orig_u,
                        void_cand=int(vf_cc_u),
                        void_orig=int(vf_orig_u),
                        cc_n2=int(cc_n2_u),
                        lex_better=lex_win_u,
                        telem=pick_telem,
                        join_prefer=bool(void_scene_locks),
                        void_scene=bool(void_scene_locks),
                        count_cand=len(cc_nest_u),
                        count_orig=len(selected_nest),
                    ):
                        selected_nest = cc_nest_u
                        locked_motif = list(union_lock)
                        cc_pair_won = True
                        beamed_sigs.add(union_sig)
                        motif_beam_trials += 1
                        motif_beam_wins += 1
                        propose_stats["motif_union_hollow_win"] = 1
                        propose_stats["motif_lock_source"] = "union_hollow"
            if hollow_locks and not cc_pair_won:
                a_orig = float(_sel_area(selected_nest, group_id, part_areas))
                vf_orig = count_selected_in_free(
                    polys, selected_nest, free_poly,
                )
                for lock in hollow_locks:
                    sig = tuple(sorted(int(i) for i in lock))
                    if sig in beamed_sigs:
                        continue
                    # MotifJoin fallback: if pair already in MIS, pin for refine
                    # without re-nest area penalty (hybrid_pick_reject_area).
                    if join_lock_fallback:
                        pinned = pin_if_in_nest(
                            lock,
                            selected_nest,
                            graph,
                            propose_stats=propose_stats,
                            source="join_in_nest",
                        )
                        if pinned is not None:
                            locked_motif = pinned
                            cc_pair_won = True
                            beamed_sigs.add(sig)
                            motif_beam_trials += 1
                            motif_beam_wins += 1
                            break
                    score_use = list(scores)
                    if void_scene_locks and void_term > 0.0:
                        for i in lock:
                            ix = int(i)
                            if 0 <= ix < len(score_use):
                                score_use[ix] = float(score_use[ix]) + float(void_term)
                    cc_nest = _nest_with_locks(
                        graph,
                        score_use,
                        lock,
                        group_id=group_id,
                        part_areas=part_areas,
                        dual=bool(dual_nest),
                    )
                    _cg2, cc_n2 = count_proposer_on_selection(
                        group_id, transform, cc_nest, pk_merged, "cluster_copy",
                    )
                    a_cc = float(_sel_area(cc_nest, group_id, part_areas))
                    vf_cc = count_selected_in_free(
                        polys, cc_nest, free_poly,
                    )
                    lex_win = _lex_pick_better(
                        best=selected_nest,
                        cand=cc_nest,
                        group_id=group_id,
                        part_areas=part_areas,
                    )
                    if hybrid_compose_pick(
                        graph=graph,
                        lock=lock,
                        area_cand=a_cc,
                        area_orig=a_orig,
                        void_cand=int(vf_cc),
                        void_orig=int(vf_orig),
                        cc_n2=int(cc_n2),
                        lex_better=lex_win,
                        telem=pick_telem,
                        join_prefer=bool(join_lock_fallback or void_scene_locks),
                        void_scene=bool(void_scene_locks),
                        count_cand=len(cc_nest),
                        count_orig=len(selected_nest),
                    ):
                        selected_nest = cc_nest
                        locked_motif = list(lock)
                        cc_pair_won = True
                        beamed_sigs.add(sig)
                        motif_beam_trials += 1
                        motif_beam_wins += 1
                        if void_scene_locks and score_use is not scores:
                            scores[:] = score_use
                        if propose_stats is not None:
                            propose_stats.update(pick_telem)
                            propose_stats["cluster_copy_pair_lock"] = 1
                            propose_stats["cluster_copy_nest_n"] = int(cc_n2)
                            propose_stats["compose_motif_hold"] = 1
                            propose_stats["motif_lock_source"] = (
                                "hybrid_void_scene"
                                if void_scene_locks
                                else ("hybrid_join" if join_lock_fallback else "hybrid_pick")
                            )
                        break
                    beamed_sigs.add(sig)
                    motif_beam_trials += 1
                if propose_stats is not None and pick_telem:
                    for k, v in pick_telem.items():
                        if k.startswith("hybrid_pick_"):
                            propose_stats[k] = int(propose_stats.get(k, 0) or 0) + int(v)
                if not cc_pair_won and dg is not None:
                    for lock in motif_join_lock_sets(dg, graph, max_locks=4):
                        pinned = pin_if_in_nest(
                            lock,
                            selected_nest,
                            graph,
                            propose_stats=propose_stats,
                            source="join_in_nest",
                        )
                        if pinned is not None:
                            locked_motif = pinned
                            cc_pair_won = True
                            break
                if not cc_pair_won:
                    # Q377: when pick=0 on hollow, unlocked re-nest @ 0.88× + void rise.
                    pick_wins = int(pick_telem.get("hybrid_pick_wins", 0) or 0)
                    cc_keys = pk_merged.get("cluster_copy") or set()
                    emit_scores = list(scores)
                    motif_w = float(getattr(cfg.propose, "motif_score_boost", 0.0) or 0.0)
                    n_steer = hollow_cc_score_steer(
                        emit_scores,
                        transform,
                        cc_keys,
                        motif_weight=motif_w,
                        void_term=float(void_term),
                    )
                    if n_steer > 0 or pick_wins == 0:
                        score_use = emit_scores if n_steer > 0 else scores
                        cc_nest = _nest_with_locks(
                            graph,
                            score_use,
                            [],
                            group_id=group_id,
                            part_areas=part_areas,
                            dual=bool(dual_nest),
                        )
                        if not _packing_independent(cc_nest, graph):
                            cc_nest = []
                        _cg2, cc_n2 = count_proposer_on_selection(
                            group_id, transform, cc_nest, pk_merged, "cluster_copy",
                        ) if cc_nest else (0, 0)
                        a_cc = float(_sel_area(cc_nest, group_id, part_areas)) if cc_nest else 0.0
                        vf_cc = count_selected_in_free(
                            polys, cc_nest, free_poly,
                        ) if cc_nest else 0
                        void_rise = int(vf_cc) > int(vf_orig)
                        lex_ok = bool(cc_nest) and lex_count_area_better(
                            old_count=len(selected_nest),
                            old_area=a_orig,
                            new_count=len(cc_nest),
                            new_area=a_cc,
                        )
                        # Q377: unlocked void arms void soft floor in hybrid_compose_pick.
                        unlock_telem: dict = {}
                        accept_unlock = bool(cc_nest) and hybrid_compose_pick(
                            graph=graph,
                            lock=[],
                            lock_len=0,
                            unlocked_void=bool(pick_wins == 0 and void_rise),
                            area_cand=a_cc,
                            area_orig=a_orig,
                            void_cand=int(vf_cc),
                            void_orig=int(vf_orig),
                            cc_n2=int(cc_n2),
                            lex_better=lex_ok,
                            telem=unlock_telem,
                        )
                        if accept_unlock:
                            selected_nest = cc_nest
                            if n_steer > 0:
                                scores[:] = emit_scores
                            if propose_stats is not None:
                                propose_stats.update(unlock_telem)
                                propose_stats["cluster_copy_nest_retry"] = 1
                                propose_stats["cluster_copy_nest_n"] = int(cc_n2)
                                propose_stats["hollow_void_bridge"] = int(
                                    pick_wins == 0 and void_rise
                                )
                                if n_steer > 0:
                                    propose_stats["motif_packing_score_boost_n"] = int(
                                        propose_stats.get("motif_packing_score_boost_n", 0) or 0
                                    ) + int(n_steer)
                            # Pin MotifJoin / Scene hollow on unlocked nest.
                            if not locked_motif:
                                if dg is not None:
                                    for lock in motif_join_lock_sets(
                                        dg, graph, max_locks=4,
                                    ):
                                        pinned = pin_if_in_nest(
                                            lock,
                                            selected_nest,
                                            graph,
                                            propose_stats=propose_stats,
                                            source="join_after_unlock",
                                        )
                                        if pinned is not None:
                                            locked_motif = pinned
                                            break
                                if not locked_motif and void_scene_locks:
                                    for lock in hollow_locks:
                                        pinned = pin_if_in_nest(
                                            lock,
                                            selected_nest,
                                            graph,
                                            propose_stats=propose_stats,
                                            source="void_scene_in_nest",
                                        )
                                        if pinned is not None:
                                            locked_motif = pinned
                                            break

    # Beam void-core MIS before incumbent hold so fat-basin void packs can
    # survive S0 via the same void_override path (one prefer helper).
    free_predicate: FreeCentroidPredicate | None = None
    if (
        free_info is not None
        and getattr(free_info, "kind", None) == "large_void"
        and free_poly is not None
        and not getattr(free_poly, "is_empty", True)
    ):
        interior_m = float(min_dist) * 0.25
        free_predicate = FreeCentroidPredicate.from_shapely(
            free_poly,
            interior_m,
        )
        core_stats: dict = {}
        void_seed = (
            list(locked_motif)
            if graph_to_nest_hollow and locked_motif
            else None
        )
        void_first = void_core_then_rim(
            graph,
            polys,
            free_poly,
            scores,
            interior_margin=interior_m,
            stats_out=core_stats,
            predicate=free_predicate,
            seed_core=void_seed,
        )
        if not void_first and interior_m > 1e-12:
            void_first = void_core_then_rim(
                graph,
                polys,
                free_poly,
                scores,
                interior_margin=0.0,
                stats_out=core_stats,
                predicate=free_predicate.with_margin(0.0),
                seed_core=void_seed,
            )
        if propose_stats is not None:
            propose_stats.update(core_stats)
        take_void_core = False
        if void_first:
            vf_void = count_selected_in_free(polys, void_first, free_poly)
            rim_void = count_selected_in_free(polys, selected_nest, free_poly)
            vf_area = float(_sel_area(void_first, group_id, part_areas))
            rim_area = float(_sel_area(selected_nest, group_id, part_areas))
            if graph_to_nest_hollow and vf_void > rim_void and (
                vf_area + 1e-12 >= 0.87 * rim_area
                or (vf_void >= rim_void + 5 and vf_area + 1e-12 >= 0.84 * rim_area)
            ):
                take_void_core = True
            elif prefer_void_core_selection(
                rim_sel=selected_nest,
                void_sel=void_first,
                polys=polys,
                free_poly=free_poly,
                group_id=group_id,
                part_areas=part_areas,
            ):
                take_void_core = True
        if take_void_core:
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
                # Q342: hollow + plateau + large_void — area near-tie + void gain.
                hollow_miss = bool((propose_stats or {}).get("hollow_miss", False))
                on_plateau = bool((propose_stats or {}).get("on_plateau", False))
                if (
                    (hollow_miss or graph_to_nest_hollow)
                    and on_plateau
                    and void_gain > 0
                    and cand_area + 1e-12 >= 0.99 * inc_area
                ):
                    void_override = True
                    void_override_flag = 1
            if (
                not void_override
                and graph_to_nest_hollow
                and void_gain >= 1
                and cand_area + 1e-12 >= (
                    0.82 if void_gain >= 5 else 0.86
                ) * inc_area
                and len(selected_nest) >= int(0.70 * len(incumbent))
            ):
                void_override = True
                void_override_flag = 1
            if not void_override:
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
                    if (
                        int((propose_stats or {}).get("motif_sequential_full", 0) or 0) <= 0
                        and not int((propose_stats or {}).get("cluster_copy_pair_lock", 0) or 0)
                        and not locked_motif
                    ):
                        locked_motif = []
                    elif locked_motif:
                        if propose_stats is not None:
                            propose_stats["compose_motif_hold"] = 1
                    incumbent_hold = 1
                    if propose_stats is not None:
                        propose_stats["motif_override"] = 0
    # One colonize walk onto the held/MIS base (rim density + void pins).
    if (
        free_predicate is not None
        and free_poly is not None
        and not getattr(free_poly, "is_empty", True)
    ):
        interior_m = float(min_dist) * 0.25
        on_plateau_void_col = bool((propose_stats or {}).get("on_plateau", False)) and (
            free_info is not None and getattr(free_info, "kind", None) == "large_void"
        )
        # Q344: plateau + large_void colonize margin 1 (margin 0 on hollow hurt area).
        if on_plateau_void_col:
            interior_m = min(1.0, float(min_dist))
        void_base = count_selected_in_free(
            polys, selected_nest, free_poly, interior_margin=interior_m,
            predicate=free_predicate,
        )
        n_void_graph = count_graph_in_free(
            polys, free_poly, interior_margin=interior_m,
            predicate=free_predicate,
        )
        use_margin = interior_m
        pred_use = free_predicate
        if n_void_graph <= void_base:
            pred_zero = free_predicate.with_margin(0.0)
            void_base = count_selected_in_free(
                polys, selected_nest, free_poly, predicate=pred_zero,
            )
            n_void_graph = count_graph_in_free(
                polys, free_poly, predicate=pred_zero,
            )
            use_margin = 0.0
            pred_use = pred_zero
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
                predicate=pred_use,
            )
            if propose_stats is not None:
                propose_stats.update(colonize_stats)
            pinned_n = int(colonize_stats.get("colonize_pinned", 0) or 0)
            if pinned_n > 0 or len(colonized) != len(selected_nest):
                # Q375: colonize accept via lex only (no flat-area part inflate).
                if lex_count_area_better(
                    old_count=len(selected_nest),
                    old_area=float(_sel_area(selected_nest, group_id, part_areas)),
                    new_count=len(colonized),
                    new_area=float(_sel_area(colonized, group_id, part_areas)),
                ):
                    selected_nest = colonized
                    # Q374: do not union colonize pins into motif_locked — only
                    # pick/beam/block_swap are MotifJoin-worthy (colonize is density).
                elif propose_stats is not None:
                    propose_stats["colonize_area_reject"] = 1
        elif propose_stats is not None and on_plateau_void_col:
            propose_stats["colonize_skipped"] = int(
                propose_stats.get("colonize_skipped", 0) or 0
            ) + 1
    if propose_stats is not None:
        propose_stats["incumbent_hold"] = int(incumbent_hold)
        propose_stats["void_override"] = int(void_override_flag)
        propose_stats["motif_locked"] = list(locked_motif)
        compose_sz = len(locked_motif)
        propose_stats["motif_compose_accepted_size"] = int(compose_sz)
        propose_stats["lock_n_compose"] = int(compose_sz)
        propose_stats["refine_lock_n"] = int(compose_sz)
        propose_stats["motif_beam_sets"] = int(len(lock_sets))
        propose_stats["motif_beam_trials"] = int(motif_beam_trials)
        propose_stats["motif_beam_wins"] = int(motif_beam_wins)
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
            compose_sz = len(locked_motif)
            propose_stats["motif_compose_accepted_size"] = int(compose_sz)
            propose_stats["lock_n_compose"] = int(compose_sz)
            propose_stats["refine_lock_n"] = int(compose_sz)
            if int(swap_telem.get("block_cohort_accepted", 0) or 0) > 0:
                propose_stats["motif_lock_source"] = str(
                    propose_stats.get("motif_lock_source") or ""
                ) + "+block_swap"


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
