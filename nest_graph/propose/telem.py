"""Propose/build_graph telemetry helpers (void_leak + motif_telem)."""

from typing import Any, Mapping, Sequence


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
        f"pocket={pf_em}/{pf_att} sel={pf_sel} valid={pf_surv}% "
        f"key_boost={pocket_key_hits} motif_boost={motif_key_hits} "
        f"island_boost={island_hits} "
        f"small_boost={small_hits} "
        f"pin={pin_stats.get('pin_added', 0)}/"
        f"{pin_stats.get('pin_candidates', 0)}"
        f"(block={pin_stats.get('pin_blocked_collision', 0)},"
        f"{pin_stats.get('pin_ms', 0.0):.1f}ms) "
        f"elite={elite_seeded}->{elite_next} "
        f"cluster_copy={cc_e}/{cc_p} patterns={n_patterns} "
        f"side_pack={sp_e}/{sp_p} cloud={fsc_e}/{fsc_p} "
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
        "block_cohort_accepted": int(propose_stats.get("block_cohort_accepted", 0)),
        "block_cohort_related": int(propose_stats.get("block_cohort_related", 0)),
        "block_hole_accepted": int(propose_stats.get("block_hole_accepted", 0)),
        "block_hole_emit_in_hull": int(propose_stats.get("block_hole_emit_in_hull", 0)),
        "keep_history_on_sterile": bool(
            propose_stats.get("keep_history_on_sterile", False)
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
    # Prefer earliest collapse in the funnel.
    if n_void_props <= 0 and densify_f <= 0:
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
