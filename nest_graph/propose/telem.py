"""Propose/build_graph telemetry helpers (void_leak + motif_telem)."""

from typing import Any, Mapping, Sequence

import numpy as np
from dataclasses import dataclass

from nest_graph.propose.void_selection import (
    centroid_in_free,
    count_graph_in_free,
    count_props_in_free,
    count_props_near_pole,
    count_selected_by_proposer,
    count_selected_in_free,
    format_prop_accept,
    void_pole_near_radius,
    zones_have_void_hijack,
)


def format_void_leak_line(
    *,
    free_kind: str,
    max_void_ratio: float,
    n_void_props: int,
    n_props_pole: int,
    hijack: int | bool,
    n_void_graph: int,
    n_void_nest: int,
    n_void_refine: int,
    nest_refine_delta: int,
    border_only: bool,
    outline_cov: float,
    sat_override: int | bool,
    rim_progress: float,
    on_plateau: bool,
    zone_snip: str,
    pf_em: int,
    pf_att: int,
    pf_sel: int,
    pf_surv: int,
    pocket_key_hits: int,
    motif_key_hits: int,
    island_hits: int,
    small_hits: int,
    pin_stats: Mapping[str, Any],
    elite_seeded: int,
    elite_next: int,
    cc_e: int,
    cc_p: int,
    n_patterns: int,
    sp_e: int,
    sp_p: int,
    fsc_e: int,
    fsc_p: int,
    propose_stats: Mapping[str, Any],
    densify: Mapping[str, Any],
    densify_a: int,
    densify_f: int,
    densify_reason: str | None,
    skip_snip: str,
    attract_edges: int,
    attract_pairs_selected: int,
    prop_accept: str,
) -> str:
    """One-line void_leak print string (keys unchanged)."""
    reason = f" reason={densify_reason}" if densify_reason else ""
    pocket = f" pocket_skip=[{skip_snip}]" if skip_snip else ""
    prop = f" prop_accept {prop_accept}" if prop_accept else ""
    return (
        f"void_leak free={free_kind} ratio={max_void_ratio:.1f} "
        f"props={n_void_props} props_pole={n_props_pole} hijack={hijack} "
        f"graph={n_void_graph} "
        f"nest={n_void_nest} refine={n_void_refine} delta={nest_refine_delta} "
        f"border_only={bool(border_only)} "
        f"outline_cov={outline_cov:.3f} sat_override={sat_override} "
        f"rim={rim_progress:.3f} plateau={int(on_plateau)} "
        f"zones=[{zone_snip}] "
        f"force_zone={propose_stats.get('dg_force_zone') or '-'} "
        f"en_prop={int(propose_stats.get('enabled_proposers_n', 0))} "
        f"pocket={pf_em}/{pf_att} sel={pf_sel} valid={pf_surv}% "
        f"key_boost={pocket_key_hits} motif_boost={motif_key_hits} "
        f"island_boost={island_hits} "
        f"small_boost={small_hits} "
        f"pin={pin_stats.get('pin_added', 0)}/"
        f"{pin_stats.get('pin_candidates', 0)}"
        f"(block={pin_stats.get('pin_blocked_collision', 0)},"
        f"{pin_stats.get('pin_ms', 0.0):.1f}ms) "
        f"elite={elite_seeded}->{elite_next} "
        f"arch_elite={int(propose_stats.get('archive_elite_n', 0))} "
        f"browse={int(propose_stats.get('browse_jump', 0))} "
        f"amaf_hits={int(propose_stats.get('amaf_hits', 0) or (propose_stats.get('mcts') or {}).get('amaf_hits', 0) or 0)} "
        f"amaf_miss={int(propose_stats.get('amaf_miss', 0))} "
        f"kind_survive={int(propose_stats.get('kind_survive', 0))} "
        f"motif_credit={int(propose_stats.get('motif_nest_credit', 0))} "
        f"hollow={int(propose_stats.get('hollow_miss', 0))} "
        f"cluster_copy={cc_e}/{cc_p} patterns={n_patterns} "
        f"side_pack={sp_e}/{sp_p} cloud={fsc_e}/{fsc_p} "
        f"ray={int((densify.get('emitted_by_proposer') or {}).get('raycasting', 0))}/"
        f"{int((densify.get('pool_by_proposer') or {}).get('raycasting', 0))} "
        f"voronoi={int((densify.get('emitted_by_proposer') or {}).get('voronoi', 0))}/"
        f"{int((densify.get('pool_by_proposer') or {}).get('voronoi', 0))} "
        f"erosion={int((densify.get('emitted_by_proposer') or {}).get('erosion', 0))}/"
        f"{int((densify.get('pool_by_proposer') or {}).get('erosion', 0))} "
        f"carry={int(propose_stats.get('carry_n', 0))}/"
        f"{int(propose_stats.get('carry_n_next', 0))} "
        f"hist_q={int(propose_stats.get('hist_q', 0))} "
        f"gvalid={int(propose_stats.get('graph_valid_n', 0))} "
        f"mix_props={int(propose_stats.get('mix_props', 0))} "
        f"rim_skip={int(propose_stats.get('rim_skip', 0))} "
        f"sel_kept={int(propose_stats.get('sel_kept', 0))} "
        f"incumbent_hold={int(propose_stats.get('incumbent_hold', 0))} "
        f"incumbent_mapped={int(propose_stats.get('incumbent_mapped', 0))} "
        f"void_override={int(propose_stats.get('void_override', 0))} "
        f"colonize={int(propose_stats.get('colonize_pinned', 0))}/"
        f"{int(propose_stats.get('colonize_blocked', 0))} "
        f"nest_void_term_hits={int(propose_stats.get('nest_void_term_hits', 0))} "
        f"void_refine_hold={int(propose_stats.get('void_refine_hold', 0))} "
        f"dfs_passes={int(propose_stats.get('dfs_passes', 0))} "
        f"dfs_mode={propose_stats.get('dfs_mode', '-')} "
        f"refine_ms={float(propose_stats.get('refine_ms', 0.0) or 0.0):.1f} "
        f"densify={densify_a}/{densify_f}"
        f"{reason}{pocket}"
        f"{_motif_skip_snip(propose_stats, n_patterns)}"
        f" cascade={densify.get('cascade_stopped_after', 'none')}"
        f" nms={int(densify.get('nms_kept', 0))}/{int(densify.get('nms_dropped', 0))}"
        f" attract={attract_edges}/{attract_pairs_selected}"
        f" 3a={int(propose_stats.get('block_cohort_accepted', 0))}/"
        f"{int(propose_stats.get('block_cohort_related', 0))}"
        f" 3b={int(propose_stats.get('block_hole_accepted', 0))}/"
        f"{int(propose_stats.get('block_hole_emit_in_hull', 0))}"
        f"{prop}"
    )


def _motif_skip_snip(propose_stats: Mapping[str, Any], n_patterns: int) -> str:
    """Named motif skip keys when patterns>0 (T1/M0)."""
    if int(n_patterns) <= 0:
        return ""
    skip = propose_stats.get("pocket_skip") or propose_stats.get("motif_skip") or {}
    if not isinstance(skip, Mapping):
        densify = propose_stats.get("densify_stats") or {}
        skip = densify.get("pocket_skip_map") or {}
    if not isinstance(skip, Mapping):
        return ""
    keys = (
        "no_rels",
        "collide",
        "motif_collide",
        "leader_fail",
        "no_anchors",
        "fallback_leader",
        "full_motif_clear",
    )
    parts = []
    for k in keys:
        v = int(skip.get(k, 0) or skip.get(f"motif_{k}", 0) or 0)
        if v > 0:
            parts.append(f"{k}={v}")
    if not parts:
        return ""
    return f" motif_skip=[{','.join(parts)}]"


def build_void_leak_dict(
    *,
    free_kind: str,
    max_void_ratio: float,
    n_void_props: int,
    n_props_pole: int,
    hijack: bool,
    n_void_graph: int,
    n_void_nest: int,
    n_void_refine: int,
    nest_refine_delta: int,
    outline_cov: float,
    sat_override: bool,
    rim_progress: float,
    plateau: Any,
    pin_all_blocked_streak: int,
    propose_stats: Mapping[str, Any],
    zones: Sequence,
    pf_em: int,
    pf_att: int,
    pf_sel: int,
    pin_stats: Mapping[str, Any],
    pf_surv: int,
    pocket_key_hits: int,
    motif_key_hits: int,
    island_hits: int,
    small_hits: int,
    densify_f: int,
    densify_a: int,
    densify_reason: Any,
    pocket_skip: Sequence,
    emitted_bp: dict,
    pool_bp: dict,
    nest_bp: dict,
    refine_bp: dict,
    pocket_by_tag: dict,
    prop_accept: str,
    densify: Mapping[str, Any],
    elite_seeded: int,
    elite_next: int,
    cc_e: int,
    cc_p: int,
    sp_e: int,
    sp_p: int,
    fsc_e: int,
    fsc_p: int,
    n_patterns: int,
    attract_edges: int,
    attract_pairs_selected: int,
    attract_bonus: float,
) -> dict:
    """void_leak dict for propose_stats (keys unchanged)."""
    return {
        "free_kind": free_kind,
        "max_void_ratio": float(max_void_ratio),
        "props": n_void_props,
        "props_pole": n_props_pole,
        "hijack": bool(hijack),
        "graph": n_void_graph,
        "nest": n_void_nest,
        "refine": n_void_refine,
        "nest_refine_delta": nest_refine_delta,
        "border_only": bool(propose_stats.get("border_only", False)),
        "outline_cov": outline_cov,
        "sat_override": bool(sat_override),
        "rim_progress": rim_progress,
        "on_plateau": bool(plateau.on_plateau),
        "plateau_streak": int(plateau.streak),
        "pin_all_blocked_streak": int(pin_all_blocked_streak),
        "rim_saturated_skip": bool(propose_stats.get("rim_saturated_skip", False)),
        "mix_props": int(propose_stats.get("mix_props", 0)),
        "rim_skip": int(propose_stats.get("rim_skip", 0)),
        "sel_kept": int(propose_stats.get("sel_kept", 0)),
        "incumbent_hold": int(propose_stats.get("incumbent_hold", 0)),
        "incumbent_mapped": int(propose_stats.get("incumbent_mapped", 0)),
        "void_override": int(propose_stats.get("void_override", 0)),
        "colonize_pinned": int(propose_stats.get("colonize_pinned", 0)),
        "colonize_blocked": int(propose_stats.get("colonize_blocked", 0)),
        "colonize_rim_drop": int(propose_stats.get("colonize_rim_drop", 0)),
        "colonize_candidates": int(propose_stats.get("colonize_candidates", 0)),
        "colonize_area_reject": int(propose_stats.get("colonize_area_reject", 0)),
        "nest_void_term_hits": int(propose_stats.get("nest_void_term_hits", 0)),
        "void_refine_hold": int(propose_stats.get("void_refine_hold", 0)),
        "dfs_passes": int(propose_stats.get("dfs_passes", 0)),
        "dfs_mode": str(propose_stats.get("dfs_mode", "")),
        "refine_ms": float(propose_stats.get("refine_ms", 0.0) or 0.0),
        "zones_used": list(zones),
        "pocket_fit_emitted": pf_em,
        "pocket_fit_attempts": pf_att,
        "pocket_fit_selected": pf_sel,
        "pin_candidates": int(pin_stats.get("pin_candidates", 0)),
        "pin_added": int(pin_stats.get("pin_added", 0)),
        "pin_blocked_collision": int(pin_stats.get("pin_blocked_collision", 0)),
        "pin_ms": float(pin_stats.get("pin_ms", 0.0)),
        "pocket_fit_survival_pct": pf_surv,
        "pocket_key_boost_hits": pocket_key_hits,
        "motif_key_boost_hits": motif_key_hits,
        "void_island_boost_hits": island_hits,
        "small_part_boost_hits": small_hits,
        "densify_fired": densify_f,
        "densify_accepted": densify_a,
        "densify_reason": densify_reason,
        "pocket_skip": list(pocket_skip),
        "emitted_by_proposer": emitted_bp,
        "pool_by_proposer": pool_bp,
        "nest_by_proposer": nest_bp,
        "refine_by_proposer": refine_bp,
        "pocket_by_tag": pocket_by_tag,
        "prop_accept": prop_accept,
        "cascade_stopped_after": densify.get("cascade_stopped_after", "none"),
        "cascade_skipped_proposers": list(
            densify.get("cascade_skipped_proposers") or []
        ),
        "nms_kept": int(densify.get("nms_kept", 0)),
        "nms_dropped": int(densify.get("nms_dropped", 0)),
        "conflict_penalty_applied": int(
            densify.get("conflict_penalty_applied", 0)
        ),
        "multi_pole_count": int(densify.get("multi_pole_count", 0) or 0),
        "void_elite_seeded": elite_seeded,
        "void_elite_archived": elite_next,
        "cluster_copy_emitted": cc_e,
        "cluster_copy_pool": cc_p,
        "side_pack_emitted": sp_e,
        "side_pack_pool": sp_p,
        "free_space_cloud_emitted": fsc_e,
        "free_space_cloud_pool": fsc_p,
        "cluster_patterns": n_patterns,
        "carry_n": int(propose_stats.get("carry_n", 0)),
        "carry_n_next": int(propose_stats.get("carry_n_next", 0)),
        "hist_niche_n": int(propose_stats.get("hist_niche_n", 0)),
        "hist_q": int(propose_stats.get("hist_q", 0)),
        "graph_valid_n": int(propose_stats.get("graph_valid_n", 0)),
        "attract_edges": attract_edges,
        "attract_pairs_selected": attract_pairs_selected,
        "attract_bonus": attract_bonus,
        "attach_n": int(propose_stats.get("attach_n", 0)),
        "kind_n": int(propose_stats.get("kind_n", 0)),
        "mutex_n": int(propose_stats.get("mutex_n", 0)),
        "kind_keys_n": sum(
            len(v) for v in (propose_stats.get("kind_keys") or {}).values()
        ),
        "member_hits": int(propose_stats.get("member_hits", 0)),
        "materialized_attach": int(propose_stats.get("materialized_attach", 0)),
        "kind_survive": int(
            propose_stats.get("kind_survive", 0)
            or (propose_stats.get("mcts") or {}).get("kind_survive", 0)
            or 0
        ),
        "motif_nest_credit": int(propose_stats.get("motif_nest_credit", 0)),
        "amaf_hits": int(
            propose_stats.get("amaf_hits", 0)
            or (propose_stats.get("mcts") or {}).get("amaf_hits", 0)
            or 0
        ),
        "block_cohort_accepted": int(propose_stats.get("block_cohort_accepted", 0)),
        "block_cohort_related": int(propose_stats.get("block_cohort_related", 0)),
        "block_hole_accepted": int(propose_stats.get("block_hole_accepted", 0)),
        "block_hole_emit_in_hull": int(propose_stats.get("block_hole_emit_in_hull", 0)),
        "block_hole_tried": int(propose_stats.get("block_hole_tried", 0) or 0),
        "keep_history_on_sterile": bool(
            propose_stats.get("keep_history_on_sterile", False)
        ),
        # R2/R3 letter telem (Q186–Q192).
        "cluster_copy_mix_floor_hits": int(
            propose_stats.get("cluster_copy_mix_floor_hits", 0) or 0
        ),
        "motif_override": int(propose_stats.get("motif_override", 0) or 0),
        "plateau_props_boost": int(propose_stats.get("plateau_props_boost", 0) or 0),
        "run_3b": int(propose_stats.get("run_3b", 0) or 0),
        "refine_rejected": int(bool(propose_stats.get("refine_rejected", False))),
        "motif_refine_hits": int(propose_stats.get("motif_refine_hits", 0) or 0),
        "accepted_patterns_archived": int(
            propose_stats.get("accepted_patterns_archived", 0)
            or propose_stats.get("motif_library_n", 0)
            or 0
        ),
        "inward_bridge_attempt": int(
            propose_stats.get("inward_bridge_attempt", 0)
            or (propose_stats.get("densify_stats") or {}).get("inward_bridge_attempt", 0)
            or 0
        ),
        # F0/Q211 repair + repack stamp telem (aliases until RepairCohort).
        "repair_mode": int(propose_stats.get("repair_mode", 0) or 0),
        "repair_patterns_n": int(propose_stats.get("repair_patterns_n", 0) or 0),
        "repack_motif_accepted": int(
            (propose_stats.get("repack") or {}).get("motif_accepted", 0)
            or propose_stats.get("repack_motif_accepted", 0)
            or 0
        ),
        "repack_pattern_fallback": int(
            (propose_stats.get("repack") or {}).get("pattern_fallback", 0)
            or propose_stats.get("repack_pattern_fallback", 0)
            or 0
        ),
        "repack_accepted": int(
            (propose_stats.get("repack") or {}).get("accepted", 0)
            or propose_stats.get("repack_accepted", 0)
            or 0
        ),
        "repack_attempted": int(
            (propose_stats.get("repack") or {}).get("attempted", 0)
            or propose_stats.get("repack_attempted", 0)
            or 0
        ),
    }


def build_motif_telem(
    *,
    skip_map: Mapping[str, int],
    motif_refine_n: int,
    propose_stats: Mapping[str, Any],
    archive_len: int,
) -> dict:
    """motif_telem dict (keys unchanged until M2 renames)."""
    motif_telem = {
        "full_motif_clear": int(skip_map.get("motif_full_motif_clear", 0)),
        "fallback_leader": int(skip_map.get("motif_fallback_leader", 0)),
        "lattice_anchors_added": int(skip_map.get("motif_lattice_anchors_added", 0)),
        "lattice_anchors_kept": int(skip_map.get("motif_lattice_anchors_kept", 0)),
        "motif_key_boost_hits": 0,
        "motif_refine_hits": int(motif_refine_n),
        "accepted_patterns_n": int(propose_stats.get("accepted_patterns_n", 0)),
        "accepted_patterns_archived": int(archive_len),
    }
    if isinstance(propose_stats.get("void_leak"), dict):
        motif_telem["motif_key_boost_hits"] = int(
            propose_stats["void_leak"].get("motif_key_boost_hits", 0)
        )
    return motif_telem


def classify_void_funnel(
    *,
    n_void_props: int,
    n_void_graph: int,
    n_void_nest: int,
    n_void_refine: int,
    densify_a: int,
    densify_f: int,
    pin_added: int,
    pin_cands: int,
) -> dict:
    """R0: name ONE bottleneck stage from void funnel ratios."""
    stages = {
        "props": int(n_void_props),
        "graph": int(n_void_graph),
        "nest": int(n_void_nest),
        "refine": int(n_void_refine),
        "densify": int(densify_a),
        "pin": int(pin_added),
    }
    bottleneck = "ok"
    # V1: props sterile with hollow graph → props_empty even if densify fired.
    if int(n_void_props) <= 0 and int(n_void_nest) <= 0 and int(n_void_graph) > 0:
        bottleneck = "props_empty"
    elif n_void_props <= 0 and densify_f <= 0:
        bottleneck = "props_empty"
    elif n_void_props > 0 and n_void_graph <= max(1, int(0.15 * n_void_props)):
        bottleneck = "props_to_graph"
    elif n_void_graph > 0 and n_void_nest <= max(0, int(0.25 * n_void_graph)):
        bottleneck = "graph_to_nest"
    elif n_void_nest > 0 and n_void_refine < n_void_nest:
        bottleneck = "nest_to_refine"
    elif densify_f > 0 and densify_a <= 0:
        bottleneck = "densify_reject"
    elif pin_cands > 0 and pin_added <= 0:
        bottleneck = "pin_blocked"
    elif n_void_refine <= 2 and n_void_props > 10:
        bottleneck = "residual_fill"
    return {
        "funnel_stages": stages,
        "bottleneck": bottleneck,
        "densify_fired": int(densify_f),
        "pin_cands": int(pin_cands),
    }


def classify_board_edge_corners(
    *,
    emitted_bp: Mapping[str, Any],
    pool_bp: Mapping[str, Any],
    nest_bp: Mapping[str, Any],
    refine_bp: Mapping[str, Any],
) -> dict:
    """R0: board_edge / sheet_corners survival through pool→nest→refine."""
    be_e = int(emitted_bp.get("board_edge", 0))
    be_p = int(pool_bp.get("board_edge", 0))
    be_n = int(nest_bp.get("board_edge", 0))
    be_r = int(refine_bp.get("board_edge", 0))
    sc_e = int(emitted_bp.get("sheet_corners", 0))
    sc_p = int(pool_bp.get("sheet_corners", 0))
    sc_n = int(nest_bp.get("sheet_corners", 0))
    sc_r = int(refine_bp.get("sheet_corners", 0))
    return {
        "board_edge_emitted": be_e,
        "board_edge_pool": be_p,
        "board_edge_nest": be_n,
        "board_edge_refine": be_r,
        "sheet_corners_emitted": sc_e,
        "sheet_corners_pool": sc_p,
        "sheet_corners_nest": sc_n,
        "sheet_corners_refine": sc_r,
        "corner_in": be_e + sc_e,
        "corner_kept": be_r + sc_r,
    }


def _proposer_e_p_n_r(
    name: str,
    *,
    emitted_bp: Mapping[str, Any],
    pool_bp: Mapping[str, Any],
    nest_bp: Mapping[str, Any],
    refine_bp: Mapping[str, Any],
) -> tuple[int, int, int, int]:
    return (
        int(emitted_bp.get(name, 0)),
        int(pool_bp.get(name, 0)),
        int(nest_bp.get(name, 0)),
        int(refine_bp.get(name, 0)),
    )


def classify_inward_explorers(
    *,
    emitted_bp: Mapping[str, Any],
    pool_bp: Mapping[str, Any],
    nest_bp: Mapping[str, Any],
    refine_bp: Mapping[str, Any],
) -> dict:
    """R0: raycasting / voronoi / erosion survival (edge→center funnel)."""
    ray_e, ray_p, ray_n, ray_r = _proposer_e_p_n_r(
        "raycasting",
        emitted_bp=emitted_bp,
        pool_bp=pool_bp,
        nest_bp=nest_bp,
        refine_bp=refine_bp,
    )
    vor_e, vor_p, vor_n, vor_r = _proposer_e_p_n_r(
        "voronoi",
        emitted_bp=emitted_bp,
        pool_bp=pool_bp,
        nest_bp=nest_bp,
        refine_bp=refine_bp,
    )
    ero_e, ero_p, ero_n, ero_r = _proposer_e_p_n_r(
        "erosion",
        emitted_bp=emitted_bp,
        pool_bp=pool_bp,
        nest_bp=nest_bp,
        refine_bp=refine_bp,
    )
    return {
        "raycasting_emitted": ray_e,
        "raycasting_pool": ray_p,
        "raycasting_nest": ray_n,
        "raycasting_refine": ray_r,
        "voronoi_emitted": vor_e,
        "voronoi_pool": vor_p,
        "voronoi_nest": vor_n,
        "voronoi_refine": vor_r,
        "erosion_emitted": ero_e,
        "erosion_pool": ero_p,
        "erosion_nest": ero_n,
        "erosion_refine": ero_r,
        "inward_emitted": ray_e + vor_e + ero_e,
        "inward_pool": ray_p + vor_p + ero_p,
        "inward_kept": ray_r + vor_r + ero_r,
    }


def assemble_void_leak(
    *,
    free_kind: str,
    max_void_ratio: float,
    n_void_props: int,
    n_props_pole: int,
    hijack: int | bool,
    n_void_graph: int,
    n_void_nest: int,
    n_void_refine: int,
    nest_refine_delta: int,
    outline_cov: float,
    sat_override: int | bool,
    rim_progress: float,
    plateau: Any,
    pin_all_blocked_streak: int,
    propose_stats: Mapping[str, Any],
    zones: Sequence,
    pf_em: int,
    pf_att: int,
    pf_sel: int,
    pin_stats: Mapping[str, Any],
    pf_surv: int,
    pocket_key_hits: int,
    motif_key_hits: int,
    island_hits: int,
    small_hits: int,
    densify_f: int,
    densify_a: int,
    densify_reason: Any,
    pocket_skip: Sequence,
    emitted_bp: dict,
    pool_bp: dict,
    nest_bp: dict,
    refine_bp: dict,
    pocket_by_tag: dict,
    prop_accept: str,
    densify: Mapping[str, Any],
    elite_seeded: int,
    elite_next: int,
    cc_e: int,
    cc_p: int,
    sp_e: int,
    sp_p: int,
    fsc_e: int,
    fsc_p: int,
    n_patterns: int,
    attract_edges: int,
    attract_pairs_selected: int,
    attract_bonus: float,
    zone_snip: str,
    skip_snip: str,
) -> tuple[str, dict]:
    """One void_leak line + dict (funnel/corners attached). Niche credit stays outside."""
    line = format_void_leak_line(
        free_kind=free_kind,
        max_void_ratio=float(max_void_ratio),
        n_void_props=n_void_props,
        n_props_pole=n_props_pole,
        hijack=hijack,
        n_void_graph=n_void_graph,
        n_void_nest=n_void_nest,
        n_void_refine=n_void_refine,
        nest_refine_delta=nest_refine_delta,
        border_only=bool(propose_stats.get("border_only", False)),
        outline_cov=outline_cov,
        sat_override=sat_override,
        rim_progress=rim_progress,
        on_plateau=bool(getattr(plateau, "on_plateau", False)),
        zone_snip=zone_snip,
        pf_em=pf_em,
        pf_att=pf_att,
        pf_sel=pf_sel,
        pf_surv=pf_surv,
        pocket_key_hits=pocket_key_hits,
        motif_key_hits=motif_key_hits,
        island_hits=island_hits,
        small_hits=small_hits,
        pin_stats=pin_stats,
        elite_seeded=elite_seeded,
        elite_next=elite_next,
        cc_e=cc_e,
        cc_p=cc_p,
        n_patterns=n_patterns,
        sp_e=sp_e,
        sp_p=sp_p,
        fsc_e=fsc_e,
        fsc_p=fsc_p,
        propose_stats=propose_stats,
        densify=densify,
        densify_a=densify_a,
        densify_f=densify_f,
        densify_reason=densify_reason,
        skip_snip=skip_snip,
        attract_edges=attract_edges,
        attract_pairs_selected=attract_pairs_selected,
        prop_accept=prop_accept,
    )
    leak = build_void_leak_dict(
        free_kind=free_kind,
        max_void_ratio=float(max_void_ratio),
        n_void_props=n_void_props,
        n_props_pole=n_props_pole,
        hijack=bool(hijack),
        n_void_graph=n_void_graph,
        n_void_nest=n_void_nest,
        n_void_refine=n_void_refine,
        nest_refine_delta=nest_refine_delta,
        outline_cov=outline_cov,
        sat_override=bool(sat_override),
        rim_progress=rim_progress,
        plateau=plateau,
        pin_all_blocked_streak=pin_all_blocked_streak,
        propose_stats=propose_stats,
        zones=zones,
        pf_em=pf_em,
        pf_att=pf_att,
        pf_sel=pf_sel,
        pin_stats=pin_stats,
        pf_surv=pf_surv,
        pocket_key_hits=pocket_key_hits,
        motif_key_hits=motif_key_hits,
        island_hits=island_hits,
        small_hits=small_hits,
        densify_f=densify_f,
        densify_a=densify_a,
        densify_reason=densify_reason,
        pocket_skip=pocket_skip,
        emitted_bp=emitted_bp,
        pool_bp=pool_bp,
        nest_bp=nest_bp,
        refine_bp=refine_bp,
        pocket_by_tag=pocket_by_tag,
        prop_accept=prop_accept,
        densify=densify,
        elite_seeded=elite_seeded,
        elite_next=elite_next,
        cc_e=cc_e,
        cc_p=cc_p,
        sp_e=sp_e,
        sp_p=sp_p,
        fsc_e=fsc_e,
        fsc_p=fsc_p,
        n_patterns=n_patterns,
        attract_edges=attract_edges,
        attract_pairs_selected=attract_pairs_selected,
        attract_bonus=attract_bonus,
    )
    funnel = classify_void_funnel(
        n_void_props=n_void_props,
        n_void_graph=n_void_graph,
        n_void_nest=n_void_nest,
        n_void_refine=n_void_refine,
        densify_a=densify_a,
        densify_f=densify_f,
        pin_added=int(pin_stats.get("pin_added", 0)),
        pin_cands=int(pin_stats.get("pin_candidates", 0)),
    )
    corners = classify_board_edge_corners(
        emitted_bp=emitted_bp,
        pool_bp=pool_bp,
        nest_bp=nest_bp,
        refine_bp=refine_bp,
    )
    inward = classify_inward_explorers(
        emitted_bp=emitted_bp,
        pool_bp=pool_bp,
        nest_bp=nest_bp,
        refine_bp=refine_bp,
    )
    leak["funnel"] = funnel
    leak["corners"] = corners
    leak["inward"] = inward
    return line, leak


@dataclass
class VoidLeakGatherCtx:
    """Inputs for void-leak assembly (build_graph + evaluator)."""

    graph: Any
    polys: list
    group_id: list
    transform: list
    free_info: Any
    free_poly: Any
    selected_nest: Sequence
    selected_polys: Sequence
    refine_scores: Sequence[float]
    propose_stats: dict
    plateau: Any
    pin_stats: Mapping[str, Any]
    pin_all_blocked_streak: int
    n_void_nest: int
    boost_hits: Mapping[str, int]
    void_pole_near_diag_ratio: float = 0.25
    proposer_counts: Mapping[str, int] | None = None
    sheet_diag: float = 0.0
    void_elite_by_group: dict | None = None
    void_elite_count_fn: Any = None
    mcts_telem: dict | None = None
    mcts_runner: Any = None


def gather_void_leak_inputs(ctx: VoidLeakGatherCtx) -> tuple[str, dict]:
    """Compute void-leak inputs and call assemble_void_leak (R1)."""
    propose_stats = ctx.propose_stats
    proposer_counts = ctx.proposer_counts or {}
    proposed_map = propose_stats.get("proposed_by_group") or {}
    proposed_list = (
        [proposed_map[g] for g in sorted(proposed_map)]
        if proposed_map else None
    )
    pole_radius = void_pole_near_radius(
        float(ctx.sheet_diag), float(ctx.void_pole_near_diag_ratio),
    )
    target_pt = getattr(ctx.free_info, "target_pt", None)
    n_props_pole = count_props_near_pole(proposed_list, target_pt, pole_radius)
    n_void_refine = count_selected_in_free(
        ctx.polys, ctx.selected_polys, ctx.free_poly,
    )
    n_void_props = count_props_in_free(proposed_list, ctx.free_poly)
    n_void_graph = count_graph_in_free(ctx.polys, ctx.free_poly)
    zones = propose_stats.get("zones_used") or []
    hijack = int(zones_have_void_hijack(zones))
    outline_cov = float(propose_stats.get("outline_cov", 0.0))
    sat_override = int(bool(propose_stats.get("sat_override", False)))
    rim_progress = float(propose_stats.get("rim_progress", 0.0))
    zone_snip = ",".join(str(z) for z in zones[:4])
    pf_em = int(
        proposer_counts.get("_pocket_fit_emitted", 0)
        or proposer_counts.get("pocket_fit", 0)
    )
    pf_att = int(proposer_counts.get("_pocket_fit_attempted", 0))
    pf_sel = int(proposer_counts.get("_pocket_fit_selected", 0))
    pf_surv = int(proposer_counts.get("_pocket_fit_survival_pct", -1))
    densify = propose_stats.get("densify_stats") or {}
    densify_f = int(densify.get("fired", proposer_counts.get("_densify_fired", 0)))
    densify_a = int(
        densify.get("accepted", proposer_counts.get("_densify_accepted", 0))
    )
    densify_reason = densify.get("densify_reason")
    hijack_over = int(densify.get("void_hijack_over_mcts", 0) or 0)
    if hijack_over and ctx.mcts_telem is not None:
        ctx.mcts_telem["void_hijack_over_mcts"] = int(
            ctx.mcts_telem.get("void_hijack_over_mcts", 0)
        ) + hijack_over
    pocket_skip = densify.get("pocket_skip") or propose_stats.get("pocket_skip") or []
    if isinstance(pocket_skip, str):
        pocket_skip = [pocket_skip]
    skip_snip = ",".join(str(s) for s in list(pocket_skip)[:4])
    boost_hits = ctx.boost_hits or {}
    pocket_key_hits = int(boost_hits.get("pocket_keys", 0))
    motif_key_hits = int(boost_hits.get("motif_keys", 0))
    small_hits = int(boost_hits.get("small_part", 0))
    island_hits = int(boost_hits.get("void_island", 0))
    nest_refine_delta = int(ctx.n_void_nest) - int(n_void_refine)
    proposer_keys = propose_stats.get("proposer_keys") or densify.get("proposer_keys") or {}
    emitted_bp = dict(propose_stats.get("emitted_by_proposer") or {})
    for name, n in (densify.get("emitted_by_proposer") or {}).items():
        emitted_bp[name] = max(int(emitted_bp.get(name, 0)), int(n))
    if not emitted_bp and densify.get("emitted_by_proposer"):
        emitted_bp = dict(densify.get("emitted_by_proposer") or {})
    pool_bp = dict(propose_stats.get("pool_by_proposer") or {})
    for name, n in (densify.get("pool_by_proposer") or {}).items():
        pool_bp[name] = max(int(pool_bp.get(name, 0)), int(n))
    if not pool_bp and densify.get("pool_by_proposer"):
        pool_bp = dict(densify.get("pool_by_proposer") or {})
    nest_bp = count_selected_by_proposer(
        ctx.transform, ctx.selected_nest, proposer_keys,
    )
    refine_bp = count_selected_by_proposer(
        ctx.transform, ctx.selected_polys, proposer_keys,
    )
    prop_accept = format_prop_accept(emitted_bp, pool_bp, nest_bp, refine_bp)
    pocket_by_tag = dict(
        propose_stats.get("pocket_by_tag") or densify.get("pocket_by_tag") or {}
    )
    elite_seeded = int(propose_stats.get("void_elite_seeded", 0))
    elite_next = 0
    if ctx.void_elite_by_group is not None and ctx.void_elite_count_fn is not None:
        elite_next = int(ctx.void_elite_count_fn(ctx.void_elite_by_group))
    cc_e = int(emitted_bp.get("cluster_copy", 0))
    cc_p = int(pool_bp.get("cluster_copy", 0))
    if (cc_e > 0 or cc_p > 0) and ctx.mcts_telem is not None:
        ctx.mcts_telem["place_motif_ok"] = int(
            ctx.mcts_telem.get("place_motif_ok", 0)
        ) + 1
    sp_e = int(emitted_bp.get("side_pack", 0))
    sp_p = int(pool_bp.get("side_pack", 0))
    fsc_e = int(emitted_bp.get("free_space_cloud", 0))
    fsc_p = int(pool_bp.get("free_space_cloud", 0))
    n_patterns = int(densify.get("cluster_patterns", 0))
    attract_edges = int(propose_stats.get("attract_edges", 0) or 0)
    if attract_edges <= 0:
        attract_edges = sum(len(row) for row in ctx.graph.attract) // 2
    sel_set = set(int(i) for i in ctx.selected_polys)
    attract_pairs_selected = 0
    attract_bonus = 0.0
    for i in sel_set:
        if i < 0 or i >= len(ctx.graph.attract):
            continue
        for e in ctx.graph.attract[i]:
            j = int(e.target)
            if j > i and j in sel_set:
                attract_pairs_selected += 1
                attract_bonus += float(e.w)
    if ctx.mcts_runner is not None and getattr(ctx.mcts_runner, "agent", None) is not None:
        agent = ctx.mcts_runner.agent
        if ctx.mcts_telem is not None:
            ctx.mcts_telem["amaf_hits"] = int(agent.telem.get("amaf_hits", 0) or 0)
            ctx.mcts_telem["amaf_miss"] = int(agent.telem.get("amaf_miss", 0) or 0)
        propose_stats["amaf_hits"] = int(agent.telem.get("amaf_hits", 0) or 0)
        propose_stats["amaf_miss"] = int(agent.telem.get("amaf_miss", 0) or 0)
    if ctx.mcts_telem is not None:
        propose_stats["browse_jump"] = int(ctx.mcts_telem.get("browse_jump", 0) or 0)
        propose_stats["motif_nest_credit"] = int(
            ctx.mcts_telem.get("motif_nest_credit", 0) or 0
        )
    if ctx.mcts_runner is not None:
        propose_stats["niche_hits"] = int(ctx.mcts_runner.niche_archive.total_hits())
    return assemble_void_leak(
        free_kind=ctx.free_info.kind,
        max_void_ratio=float(ctx.free_info.max_void_ratio),
        n_void_props=n_void_props,
        n_props_pole=n_props_pole,
        hijack=hijack,
        n_void_graph=n_void_graph,
        n_void_nest=int(ctx.n_void_nest),
        n_void_refine=n_void_refine,
        nest_refine_delta=nest_refine_delta,
        outline_cov=outline_cov,
        sat_override=sat_override,
        rim_progress=rim_progress,
        plateau=ctx.plateau,
        pin_all_blocked_streak=int(ctx.pin_all_blocked_streak),
        propose_stats=propose_stats,
        zones=zones,
        pf_em=pf_em,
        pf_att=pf_att,
        pf_sel=pf_sel,
        pin_stats=ctx.pin_stats,
        pf_surv=pf_surv,
        pocket_key_hits=pocket_key_hits,
        motif_key_hits=motif_key_hits,
        island_hits=island_hits,
        small_hits=small_hits,
        densify_f=densify_f,
        densify_a=densify_a,
        densify_reason=densify_reason,
        pocket_skip=pocket_skip,
        emitted_bp=emitted_bp,
        pool_bp=pool_bp,
        nest_bp=nest_bp,
        refine_bp=refine_bp,
        pocket_by_tag=pocket_by_tag,
        prop_accept=prop_accept,
        densify=densify,
        elite_seeded=elite_seeded,
        elite_next=elite_next,
        cc_e=cc_e,
        cc_p=cc_p,
        sp_e=sp_e,
        sp_p=sp_p,
        fsc_e=fsc_e,
        fsc_p=fsc_p,
        n_patterns=n_patterns,
        attract_edges=attract_edges,
        attract_pairs_selected=attract_pairs_selected,
        attract_bonus=attract_bonus,
        zone_snip=zone_snip,
        skip_snip=skip_snip,
    )


def archive_void_elite_transforms(
    *,
    selected_nest: Sequence[int],
    selected_refine: Sequence[int],
    polys: Sequence,
    transforms: Sequence,
    group_ids: Sequence[int],
    free_poly,
    scores: Sequence[float] | None,
    max_keep: int = 32,
    enabled: bool = True,
) -> dict[int, list[np.ndarray]]:
    """Top-scoring nest-void losers (missing from refine) → next-iter elite seeds."""
    if not enabled or free_poly is None or getattr(free_poly, "is_empty", True):
        return {}
    refine_set = set(int(i) for i in selected_refine)
    scored: list[tuple[float, int]] = []
    for v in selected_nest:
        vi = int(v)
        if vi in refine_set:
            continue
        if not centroid_in_free(polys[vi], free_poly):
            continue
        sc_v = float(scores[vi]) if scores is not None and vi < len(scores) else 0.0
        scored.append((sc_v, vi))
    scored.sort(key=lambda x: x[0], reverse=True)
    next_elite: dict[int, list[np.ndarray]] = {}
    for _sc, v in scored[: max(int(max_keep), 0)]:
        gid = int(group_ids[v]) if v < len(group_ids) else 0
        row = np.asarray(transforms[v], dtype=np.float64).reshape(3)
        next_elite.setdefault(gid, []).append(row)
    return next_elite


def void_elite_count(archive: dict[int, list[np.ndarray]] | None) -> int:
    if not archive:
        return 0
    return sum(len(v) for v in archive.values())
