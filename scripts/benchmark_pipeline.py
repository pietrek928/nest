#!/usr/bin/env python3
"""End-to-end nesting quality benchmark using parameterized fixtures."""

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

from nest_graph.config import (
    BuildGraphConfig,
    DfsMode,
    ProposeAblation,
    ProposeConfig,
)
from scripts.nesting_evaluator import NestingPipelineEvaluator, metrics_meet_floors
from scripts.nesting_fixtures import get_all_cases, resolve_cases


def _build_cfg(
    propose_name: ProposeAblation | str,
    dfs_mode: DfsMode | str,
) -> BuildGraphConfig:
    cfg = BuildGraphConfig()
    cfg.selection.dfs_mode = DfsMode(dfs_mode)
    propose = ProposeAblation(propose_name)

    match propose:
        case ProposeAblation.SHIPPED:
            pass
        case ProposeAblation.LOCAL_COMPACT:
            cfg.propose = ProposeConfig.local_compact_profile()
        case ProposeAblation.NO_VOID_BOOST:
            cfg.propose = cfg.propose.model_copy(update={
                "void_island_score_boost": 0.0,
                "void_attractor_rule_weight": 0.0,
            })
        case ProposeAblation.VOID_BOOST_HIGH:
            cfg.propose = cfg.propose.model_copy(update={
                "void_island_score_boost": 128.0,
                "pocket_score_boost": 100.0,
                "void_attractor_rule_weight": 48.0,
            })
        case ProposeAblation.NO_VOID_YIELD_DENSIFY:
            cfg.propose = cfg.propose.model_copy(update={
                "enable_void_yield_densify_accept": False,
            })
        case ProposeAblation.NO_VOID_POLE_CLEAR:
            cfg.propose = cfg.propose.model_copy(update={
                "enable_void_pole_clear_densify": False,
            })
        case ProposeAblation.NO_FREE_SPACE_CLOUD:
            cfg.propose = cfg.propose.model_copy(update={
                "use_free_space_cloud": False,
            })
        case ProposeAblation.NO_GREEDY_NEST:
            # Alias of shipped: void_greedy_nest_seed was removed.
            pass
        case ProposeAblation.NO_CONTACT_ATTRACT:
            cfg.propose = cfg.propose.model_copy(update={
                "attract_contact_weight": 0.0,
            })
        case ProposeAblation.NO_POCKET_FIT:
            cfg.propose = cfg.propose.model_copy(update={
                "use_pocket_fit": False,
            })
        case ProposeAblation.NO_SMALL_PREFER:
            cfg.propose = cfg.propose.model_copy(update={
                "small_part_void_score_boost": 0.0,
            })
        case ProposeAblation.NO_POCKET_MIS_BOOST:
            cfg.propose = cfg.propose.model_copy(update={
                "pocket_score_boost": 0.0,
            })
        case ProposeAblation.NO_CLUSTER_REPACK:
            cfg.propose = cfg.propose.model_copy(update={
                "enable_cluster_repack": False,
            })
        case ProposeAblation.NO_CLUSTER_COPY:
            cfg.propose = cfg.propose.model_copy(update={
                "use_cluster_copy": False,
            })
        case ProposeAblation.NO_SIDE_PACK:
            cfg.propose = cfg.propose.model_copy(update={
                "use_side_pack": False,
            })
        case ProposeAblation.NO_OPEN_VOID_POCKET:
            cfg.propose = cfg.propose.model_copy(update={
                "use_open_void_pocket": False,
            })
        case ProposeAblation.NO_LOCAL_SE2:
            cfg.propose = cfg.propose.model_copy(update={
                "enable_local_se2": False,
            })
        case ProposeAblation.NO_CLUSTER_RELOCATE:
            cfg.propose = cfg.propose.model_copy(update={
                "enable_cluster_relocate": False,
            })
        case ProposeAblation.NO_DENSIFY_HIJACK:
            cfg.propose = cfg.propose.model_copy(update={
                "densify_on_void_hijack": False,
            })
        case ProposeAblation.NO_VOID_ELITE:
            cfg.propose = cfg.propose.model_copy(update={
                "enable_void_elite_archive": False,
                "stratified_void_elite_quota": 0,
            })
        case ProposeAblation.NO_KEEP_HIST_STERILE:
            cfg.propose = cfg.propose.model_copy(update={
                "keep_history_on_void_sterile": False,
            })
        case ProposeAblation.NO_UNIFIED_RESERVE:
            cfg.propose = cfg.propose.model_copy(update={
                "unified_void_reserve": False,
            })
        case ProposeAblation.NO_VOID_CONTACT_HYBRID:
            cfg.propose = cfg.propose.model_copy(update={
                "void_seek_contact_hybrid": False,
            })
        case ProposeAblation.NO_MOTIF_TOPO_ANCHORS:
            cfg.propose = cfg.propose.model_copy(update={
                "motif_use_topo_anchors": False,
            })
        case ProposeAblation.NO_OVERRIDE:
            cfg.propose = cfg.propose.model_copy(update={
                "late_border_void_override_ratio": 0.0,
                "late_border_void_release_ratio": 0.0,
            })
        case ProposeAblation.NO_VOID_HIJACK:
            cfg.propose = cfg.propose.model_copy(update={
                "enable_void_large_hijack": False,
                "void_island_score_boost": 0.0,
                "void_attractor_rule_weight": 0.0,
            })
        case ProposeAblation.NO_VOID_RANK:
            cfg.propose = cfg.propose.model_copy(update={
                "void_rank_pole_weight": 0.0,
            })
        case ProposeAblation.VOID_PSO:
            # OOS-3 gated: only enable when props_pole≈0 after OOS-1+4; off by default.
            cfg.propose = cfg.propose.model_copy(update={
                "use_point_cloud": True,
                "use_guidance_walk": True,
            })
        case ProposeAblation.LEAN_VOID_COMBO:
            cfg.propose = cfg.propose.model_copy(update={
                "propose_cascade_short_circuit": True,
                "use_pose_nms": False,
                "use_multi_pole_void": True,
                "use_conflict_degree_rank": False,
                "use_ema_proposer_scales": False,
            })
        case ProposeAblation.CASCADE_ONLY:
            cfg.propose = cfg.propose.model_copy(update={
                "propose_cascade_short_circuit": True,
            })
        case ProposeAblation.NMS_ONLY:
            cfg.propose = cfg.propose.model_copy(update={
                "use_pose_nms": True,
            })
        case ProposeAblation.CONFLICT_DEGREE_RANK:
            cfg.propose = cfg.propose.model_copy(update={
                "use_conflict_degree_rank": True,
            })
        case ProposeAblation.MULTI_POLE_VOID:
            cfg.propose = cfg.propose.model_copy(update={
                "use_multi_pole_void": True,
            })
        case ProposeAblation.EMA_SCALES:
            cfg.propose = cfg.propose.model_copy(update={
                "use_ema_proposer_scales": True,
            })
        case ProposeAblation.NO_EDGE_FREE:
            cfg.propose = cfg.propose.model_copy(update={
                "edge_free_weight": 0.0,
                "selection_geom_weight": 0.0,
            })
        case ProposeAblation.NO_EDGE_FREE_RANK:
            cfg.propose = cfg.propose.model_copy(update={
                "edge_free_weight": 0.0,
            })
        case ProposeAblation.NO_SELECTION_GEOM:
            cfg.propose = cfg.propose.model_copy(update={
                "selection_geom_weight": 0.0,
            })
        case ProposeAblation.NEST_BY_GRAPH_ONLY:
            cfg.propose = cfg.propose.model_copy(update={
                "selection_geom_weight": 0.0,
            })
        case ProposeAblation.GREEDY_NEST_ONLY:
            cfg.propose = cfg.propose.model_copy(update={
                "selection_geom_weight": 0.0,
            })
        case ProposeAblation.NO_SEARCH_BUDGET:
            cfg.selection = cfg.selection.model_copy(update={
                "score_rules_latest_graph_only": False,
                "enable_plateau_budget_taper": False,
            })
            cfg.propose = cfg.propose.model_copy(update={
                "prune_colliding_transforms": False,
                "rim_saturated_skip_emitters": False,
                "pin_all_blocked_skip_after": 0,
                "stop_elite_archive_when_pin_blocked": False,
            })
        case ProposeAblation.NO_MATE_SYNTH:
            cfg.propose = cfg.propose.model_copy(update={
                "enable_mate_synth": False,
                "motif_score_boost": 0.0,
                "cascade_min_motif_pocket_emit": 0,
            })
        case ProposeAblation.NO_LEX_REFINE:
            cfg.selection = cfg.selection.model_copy(update={
                "refine_lexicographic_area": False,
            })
            cfg.propose = cfg.propose.model_copy(update={
                "refine_rim_drop_reject": 0.0,
            })
        case ProposeAblation.NO_LNS_REBUILD:
            cfg.propose = cfg.propose.model_copy(update={
                "enable_lns_rebuild": False,
            })
            cfg.selection = cfg.selection.model_copy(update={
                "refine_explore_shuffle": False,
                "plateau_beam_width": cfg.selection.dfs_refine_beam_width,
                "plateau_max_stagnant_passes": cfg.selection.dfs_refine_max_stagnant_passes,
            })
        case ProposeAblation.NO_BLOCK_REPLACE:
            cfg.propose = cfg.propose.model_copy(update={
                "enable_block_replace": False,
            })
        case ProposeAblation.NO_INCUMBENT_LOOP:
            cfg.propose = cfg.propose.model_copy(update={
                "enable_incumbent_loop": False,
            })
        case ProposeAblation.NO_PATTERN_PROPAGATE:
            cfg.propose = cfg.propose.model_copy(update={
                "enable_accepted_pattern_archive": False,
                "enable_motif_lattice": False,
                "enable_motif_mirror_anchors": True,
                "enable_motif_sequential_accept": False,
                "motif_score_boost": 0.0,
            })
        case ProposeAblation.NO_MOTIF_SEQUENTIAL_ACCEPT:
            cfg.propose = cfg.propose.model_copy(update={
                "enable_motif_sequential_accept": False,
            })
        case ProposeAblation.NO_PROPOSE:
            cfg.propose = ProposeConfig(
                use_neighbor_slide=False,
                use_voronoi=False,
                use_point_cloud=False,
                use_guidance_walk=False,
                use_ribbon_seeds=False,
                use_group_edge_seeds=False,
                use_border_edge_seeds=False,
                use_guidance_propositions=False,
            )
    return cfg


def _parse_cfg_value(raw: str):
    s = raw.strip()
    low = s.lower()
    if low in ("true", "yes", "1"):
        return True
    if low in ("false", "no", "0"):
        return False
    try:
        if "." in s:
            return float(s)
        return int(s)
    except ValueError:
        return s


def _apply_cfg_overrides(cfg: BuildGraphConfig, specs: list[str] | None) -> BuildGraphConfig:
    """Mute/retune existing ProposeConfig (or SelectionConfig) fields. No new flags."""
    if not specs:
        return cfg
    propose_upd: dict = {}
    selection_upd: dict = {}
    for spec in specs:
        if "=" not in spec:
            raise ValueError(f"cfg override must be key=value got {spec!r}")
        key, raw = spec.split("=", 1)
        key = key.strip()
        val = _parse_cfg_value(raw)
        if key.startswith("selection."):
            selection_upd[key.split(".", 1)[1]] = val
        elif key.startswith("propose."):
            propose_upd[key.split(".", 1)[1]] = val
        else:
            propose_upd[key] = val
    if propose_upd:
        cfg.propose = cfg.propose.model_copy(update=propose_upd)
    if selection_upd:
        cfg.selection = cfg.selection.model_copy(update=selection_upd)
    return cfg


def _parse_matrix(specs: list[str]) -> dict[str, list[str]]:
    """Parse axis=v1,v2 into dict."""
    out: dict[str, list[str]] = {}
    for spec in specs:
        if "=" not in spec:
            raise ValueError(f"matrix entry must be axis=v1,v2 got {spec!r}")
        axis, vals = spec.split("=", 1)
        out[axis.strip()] = [v.strip() for v in vals.split(",") if v.strip()]
    return out


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cases", nargs="*", help="Case names / family prefixes / tags")
    parser.add_argument("--tags", nargs="*", help="Tags to run (e.g. dense, void)")
    parser.add_argument("--family", type=str, default=None, help="Filter by family name")
    parser.add_argument("--all", action="store_true", help="Run all cases (overrides default tight_pack)")
    parser.add_argument(
        "--matrix",
        nargs="*",
        help="Expand axes e.g. pitch=1.6,1.8 (reserved; currently filters only)",
    )
    parser.add_argument(
        "--cfg",
        nargs="*",
        default=[],
        help=(
            "ProposeConfig/SelectionConfig overrides key=value to mute other stages "
            "while gating the letter's component (e.g. enable_lns_rebuild=false). "
            "Prefix selection. for SelectionConfig."
        ),
    )
    parser.add_argument(
        "--propose",
        nargs="*",
        default=[ProposeAblation.SHIPPED],
        type=ProposeAblation,
        choices=list(ProposeAblation),
    )
    parser.add_argument(
        "--dfs-modes",
        nargs="*",
        default=[DfsMode.MERGED_LOOSE_TIGHT],
        type=DfsMode,
        choices=list(DfsMode),
    )
    parser.add_argument("--seeds", type=int, nargs="*", default=[0])
    parser.add_argument("--gate", action="store_true", help="Fail if case floors / baselines not met")
    parser.add_argument(
        "--always-heavy",
        action="store_true",
        help="E0 ablation: DFS/post-pack every iter (default Q69 last-iter only)",
    )
    parser.add_argument("--update-baselines", action="store_true", help="Update docs/nesting_baselines.json")
    parser.add_argument(
        "--baselines",
        type=Path,
        default=Path("docs/nesting_baselines.json"),
    )
    args = parser.parse_args()

    if args.matrix:
        _parse_matrix(args.matrix)  # validate syntax; expansion reserved for CaseSpec knobs

    if args.all:
        cases_to_run = get_all_cases()
    elif args.cases or args.tags or args.family:
        cases_to_run = resolve_cases(args.cases, tags=args.tags, family=args.family)
    else:
        cases_to_run = resolve_cases(tags=["tight_pack"])

    if not cases_to_run:
        print("No cases matched.")
        sys.exit(1)

    n_runs = len(cases_to_run) * len(args.propose) * len(args.dfs_modes) * len(args.seeds)
    if n_runs > 40 and not args.force:
        print(f"Refusing {n_runs} runs (>40); pass --force to override.")
        sys.exit(2)

    results = []
    baselines: dict = {}
    if args.baselines.exists():
        baselines = json.loads(args.baselines.read_text())

    gate_fail = False
    for case in cases_to_run:
        for propose in args.propose:
            for dfs_mode in args.dfs_modes:
                cfg = _apply_cfg_overrides(_build_cfg(propose, dfs_mode), args.cfg)
                if args.cfg:
                    print(f"  cfg overrides: {args.cfg}", flush=True)
                evaluator = NestingPipelineEvaluator(
                    case, cfg, always_heavy_polish=bool(args.always_heavy),
                )

                case_metrics = []
                for seed in args.seeds:
                    print(
                        f"Running {case.name} | {propose} | {dfs_mode} | seed={seed}...",
                        flush=True,
                    )
                    metrics = evaluator.run_full_pipeline(seed)
                    case_metrics.append(metrics)
                    print(
                        f"  -> parts={metrics.parts_final} area={metrics.area_coverage:.3f} "
                        f"kiss_s={metrics.kiss_seed:.2f} kiss_o={metrics.kiss_outline:.2f} "
                        f"auc={metrics.density_auc:.2f} ok={metrics.independent_ok} "
                        f"free={metrics.free_kind}/{metrics.largest_free_over_part:.1f} "
                        f"void={metrics.void_props}/{metrics.void_graph}/"
                        f"{metrics.void_selected_nest}/{metrics.void_selected_refine} "
                        f"time={metrics.time_s:.2f}s"
                    )
                    leak = (evaluator.last_result or {}).get("void_leak") or {}
                    ebp = leak.get("emitted_by_proposer") or {}
                    pbp = leak.get("pool_by_proposer") or {}
                    inward = leak.get("inward") or {}
                    funnel = leak.get("funnel") or {}
                    if (
                        leak.get("niche_pos") is not None
                        or leak.get("contact_grg_upserts")
                        or leak.get("dfs_passes")
                        or leak.get("refine_ms")
                        or leak.get("free_space_cloud_emitted")
                        or leak.get("cluster_copy_emitted")
                        or ebp.get("history_expand")
                        or leak.get("rim_progress") is not None
                        or inward
                    ):
                        print(
                            f"     letter telem niche_pos={leak.get('niche_pos', 0)} "
                            f"niche_rescue={leak.get('niche_rescue', 0)} "
                            f"contact_grg_upserts={leak.get('contact_grg_upserts', 0)} "
                            f"pin={leak.get('pin_added', 0)}/{leak.get('pin_candidates', 0)} "
                            f"dfs_passes={leak.get('dfs_passes', 0)} "
                            f"refine_ms={float(leak.get('refine_ms', 0.0) or 0.0):.1f} "
                            f"history_expand={int(ebp.get('history_expand', 0) or 0)} "
                            f"cluster_copy={int(leak.get('cluster_copy_emitted', ebp.get('cluster_copy', 0)) or 0)} "
                            f"free_space_cloud={int(leak.get('free_space_cloud_emitted', ebp.get('free_space_cloud', 0)) or 0)} "
                            f"rim={float(leak.get('rim_progress', 0.0) or 0.0):.3f} "
                            f"rim_sat={int(bool(leak.get('rim_saturated_skip', False)))} "
                            f"side_pack={int(leak.get('side_pack_emitted', ebp.get('side_pack', 0)) or 0)}/"
                            f"{int(pbp.get('side_pack', 0) or 0)} "
                            f"ray={int(inward.get('raycasting_emitted', ebp.get('raycasting', 0)) or 0)}/"
                            f"{int(inward.get('raycasting_pool', pbp.get('raycasting', 0)) or 0)} "
                            f"voronoi={int(inward.get('voronoi_emitted', ebp.get('voronoi', 0)) or 0)}/"
                            f"{int(inward.get('voronoi_pool', pbp.get('voronoi', 0)) or 0)} "
                            f"erosion={int(inward.get('erosion_emitted', ebp.get('erosion', 0)) or 0)}/"
                            f"{int(inward.get('erosion_pool', pbp.get('erosion', 0)) or 0)} "
                            f"peak_ray={int((leak.get('inward_peak') or {}).get('raycasting', 0) or 0)} "
                            f"peak_ero={int((leak.get('inward_peak') or {}).get('erosion', 0) or 0)} "
                            f"peak_sp={int((leak.get('inward_peak') or {}).get('side_pack', 0) or 0)} "
                            f"inward_ray_keys={int(leak.get('inward_ray_keys', 0) or 0)} "
                            f"inward_att={int(leak.get('inward_bridge_attempt', 0) or 0)} "
                            f"mix_floor={int(leak.get('cluster_copy_mix_floor_hits', 0) or 0)} "
                            f"motif_ov={int(leak.get('motif_override', 0) or 0)} "
                            f"plat_boost={int(leak.get('plateau_props_boost', 0) or 0)} "
                            f"run_3b={int(leak.get('run_3b', 0) or 0)} "
                            f"3b_ok={int(leak.get('block_hole_accepted', 0) or 0)} "
                            f"3b_try={int(leak.get('block_hole_tried', 0) or 0)} "
                            f"3b_hull={int(leak.get('block_hole_emit_in_hull', 0) or 0)} "
                            f"repair={int(leak.get('repair_mode', 0) or 0)}/"
                            f"{int(leak.get('repair_patterns_n', 0) or 0)} "
                            f"repack={int(leak.get('repack_attempted', 0) or 0)}/"
                            f"{int(leak.get('repack_accepted', 0) or 0)}/"
                            f"{int(leak.get('repack_motif_accepted', 0) or 0)}/"
                            f"{int(leak.get('repack_pattern_fallback', 0) or 0)} "
                            f"arch_n={int(leak.get('accepted_patterns_archived', 0) or 0)} "
                            f"motif_ref={int(leak.get('motif_refine_hits', 0) or 0)} "
                            f"restore={int(leak.get('refine_rejected', 0) or 0)} "
                            f"bottleneck={funnel.get('bottleneck', '-')} "
                            f"zones={leak.get('zones_used', [])} "
                            f"densify={leak.get('densify_reason')} "
                            f"attach_n={leak.get('attach_n', 0)} "
                            f"kind_keys={leak.get('kind_keys_n', leak.get('kind_n', 0))} "
                            f"member_hits={leak.get('member_hits', 0)} "
                            f"mat_motif={leak.get('materialized_motif', 0)} "
                            f"mat_attach={leak.get('materialized_attach', 0)} "
                            f"compose_sz={leak.get('motif_compose_accepted_size', 0)} "
                            f"place_coh={leak.get('place_cohort_ready', 0)}/"
                            f"{leak.get('place_cohort_specs_n', 0)}/"
                            f"{leak.get('mcts_cohort_macro_n', 0)}/"
                            f"{leak.get('place_cohort_ready_hard', 0)} "
                            f"coh_sig={leak.get('motif_cohort_sig', 0)} "
                            f"cache_inv_coh={leak.get('cache_invalidate_cohort', 0)} "
                            f"union_n={leak.get('motif_union_lock_n', 0)} "
                            f"union_win={leak.get('motif_union_beam_win', 0)}/"
                            f"{leak.get('motif_union_hollow_win', 0)} "
                            f"grow_rej={leak.get('grow_reject_first_obstacle', 0)}/"
                            f"{leak.get('grow_reject_later_obstacle', 0)}/"
                            f"{leak.get('grow_reject_later_glue', 0)}/"
                            f"{leak.get('grow_reject_geom_missing', 0)}/"
                            f"{leak.get('grow_reject_none_cg', 0)} "
                            f"grow_1st={leak.get('grow_reject_first_void', 0)}/"
                            f"{leak.get('grow_reject_first_packed', 0)}/"
                            f"{leak.get('grow_reject_first_both', 0)} "
                            f"grow_tier={leak.get('grow_reject_first_seed', 0)}/"
                            f"{leak.get('grow_reject_first_mapped', 0)}/"
                            f"{leak.get('grow_reject_first_unmapped', 0)} "
                            f"grow_cls={leak.get('grow_classify_seed_n', 0)}/"
                            f"{leak.get('grow_classify_mapped_n', 0)}/"
                            f"{leak.get('grow_classify_unmapped_n', 0)} "
                            f"soft_inc={leak.get('soft_incumbent_n', 0)} "
                            f"grow_ord={leak.get('grow_order_tier_n', 0)} "
                            f"grow_touch={leak.get('grow_member_touch_n', 0)} "
                            f"full_clear={leak.get('full_motif_clear', 0)} "
                            f"fb_lead={leak.get('fallback_leader', 0)} "
                            f"coemit={leak.get('motif_coemit_followers', 0)} "
                            f"foreign_fail={leak.get('foreign_clear_fail_sheet', 0)}/"
                            f"{leak.get('foreign_clear_fail_obs', 0)}/"
                            f"{leak.get('foreign_clear_fail_empty', 0)} "
                            f"soft_fgn={leak.get('soft_foreign_incumbent_n', 0)} "
                            f"pick={leak.get('hybrid_pick_trials', 0)}/"
                            f"{leak.get('hybrid_pick_wins', 0)}/"
                            f"{leak.get('hybrid_pick_reject_indep', 0)}/"
                            f"{leak.get('hybrid_pick_reject_area', 0)}/"
                            f"{leak.get('hybrid_pick_join_soft', 0)}/"
                            f"{leak.get('hybrid_pick_survive_soft', 0)}/"
                            f"{leak.get('hybrid_pick_sticky_soft', 0)} "
                            f"lead_fail={leak.get('leader_fail', 0)} "
                            f"lat={leak.get('lattice_anchors_added', 0)}/"
                            f"{leak.get('lattice_anchors_kept', 0)} "
                            f"coh_amaf={leak.get('cohort_sig_amaf_visits', 0)} "
                            f"lock_fp={leak.get('cache_key_lock_fp', 0)} "
                            f"beam_won={leak.get('beamed_won_n', 0)} "
                            f"hollow_retry={leak.get('hollow_lock_retry_n', 0)} "
                            f"bonded={leak.get('motif_lock_bonded_max', 0)} "
                            f"w1_sot={leak.get('w1_obstacle_sot', 0)}/"
                            f"{leak.get('w1_packed_rebuilt_n', 0)} "
                            f"path_cred={leak.get('path_credit_n', 0)} "
                            f"soft={leak.get('macro_path_motif_soft', 0)} "
                            f"stitch={leak.get('star_stitch_n', 0)} "
                            f"beam={leak.get('motif_beam_sets', 0)} "
                            f"seq_full={leak.get('motif_sequential_full', 0)} "
                            f"seq_miss={leak.get('motif_sequential_skipped_missing', 0)} "
                            f"seq_clear={leak.get('motif_sequential_clear_fail', 0)} "
                            f"seq_pack={leak.get('motif_sequential_packing_clear', 0)} "
                            f"pair_restore={leak.get('archive_pin_pair_restore_n', 0)} "
                            f"pack_boost={leak.get('motif_packing_score_boost_n', 0)} "
                            f"compose_hold={leak.get('compose_motif_hold', 0)} "
                            f"join_n={leak.get('motif_join_n', 0)} "
                            f"graph_hit={leak.get('motif_graph_hit_n', 0)} "
                            f"cohorts_n={leak.get('motif_cohorts_n', 0)} "
                            f"kind_survive={leak.get('kind_survive', 0)} "
                            f"motif_credit={leak.get('motif_nest_credit', leak.get('motif_credit', 0))} "
                            f"amaf_hits={leak.get('amaf_hits', 0)} "
                            f"mcts_rid={leak.get('mcts_rule_id', 0)} "
                            f"rid_amaf={leak.get('rule_id_amaf_visits', 0)} "
                            f"arch_mix={leak.get('archive_mix_floor_hits', 0)} "
                            f"cc_mix={leak.get('cluster_copy_mix_floor_hits', 0)} "
                            f"mix_skip={leak.get('archive_mix_skip_reason', '-')} "
                            f"pin_surv={leak.get('archive_mix_pin_survive_n', 0)} "
                            f"follower_miss={leak.get('motif_graph_follower_miss_n', 0)} "
                            f"void_scale={float(leak.get('void_scale_applied', 0.0) or 0.0):.1f} "
                            f"mp_cand={leak.get('macro_path_candidate', 0)} "
                            f"arch_reject={leak.get('archive_mix_reject_n', 0)} "
                            f"arch_origin={leak.get('archive_ref_origin_n', 0)} "
                            f"ref_sc={leak.get('refine_score_accept', 0)} "
                            f"mp_acc={leak.get('macro_path_accept', 0)} "
                            f"mp_dep={leak.get('macro_swap_depth', 0)} "
                            f"path_x={leak.get('path_extend_n', 0)} "
                            f"p_mac={leak.get('path_step_macro', 0)} "
                            f"p_join={leak.get('path_step_join', 0)} "
                            f"surv_m={leak.get('survive_motif_n', 0)} "
                            f"path_up={leak.get('path_contact_upserts', 0)} "
                            f"cheap_d={float(leak.get('cheap_outer_reward_delta', 0.0) or 0.0):.3f} "
                            f"prop_ms={float(leak.get('propose_ms', 0.0) or 0.0):.1f} "
                            f"compose_ms={float(leak.get('compose_ms', 0.0) or 0.0):.1f} "
                            f"mpg={float(leak.get('mpg_ms', 0.0) or 0.0):.1f}/"
                            f"{int(leak.get('graph_valid_n', 0) or 0)} "
                            f"mpg_x={float(leak.get('mpg_xform_ms', 0.0) or 0.0):.1f}/"
                            f"{float(leak.get('mpg_valid_ms', 0.0) or 0.0):.1f}/"
                            f"{float(leak.get('mpg_edge_ms', 0.0) or 0.0):.1f} "
                            f"dfs={float(leak.get('dfs_loose_ms', 0.0) or 0.0):.1f}/"
                            f"{float(leak.get('finalize_ms', 0.0) or 0.0):.1f} "
                            f"ray={float(leak.get('ray_ms', 0.0) or 0.0):.1f} "
                            f"path_ms={float(leak.get('replay_from_ancestor_ms', 0.0) or 0.0):.1f} "
                            f"path_skip={int(leak.get('path_probe_skip', 0) or 0)}"
                        )
                    diag = (evaluator.last_result or {}).get("void_leak", {}).get("diag") or {}
                    if diag:
                        print(
                            f"     diag cov={diag.get('cov_final', 0):.3f}/"
                            f"{diag.get('cov_peak', 0):.3f}/"
                            f"seed={diag.get('cov_seed', 0):.3f} "
                            f"bn={diag.get('bottleneck', '-')} "
                            f"void={diag.get('void_props', 0)}/"
                            f"{diag.get('void_graph', 0)}/"
                            f"{diag.get('void_nest', 0)}/"
                            f"{diag.get('void_refine', 0)} "
                            f"nest_r={float(diag.get('nest_graph_ratio', 0) or 0):.2f} "
                            f"nvr_min={float(diag.get('nest_void_ratio_min', 1) or 1):.2f} "
                            f"hollow_iters={int(diag.get('graph_to_nest_hollow_iters', 0) or 0)} "
                            f"hold={int(diag.get('incumbent_hold', 0) or 0)} "
                            f"ovr={int(diag.get('void_override', 0) or 0)} "
                            f"beam={int(diag.get('motif_beam_wins', 0) or 0)}/"
                            f"{int(diag.get('motif_beam_trials', 0) or 0)} "
                            f"scene_sz={int(diag.get('motif_scene_max_sz', 0) or 0)} "
                            f"f_miss={float(diag.get('follower_miss_pct', 0) or 0):.0f}% "
                            f"hollow_rn={int(diag.get('hollow_renest', 0) or 0)} "
                            f"regress={float(diag.get('cov_regress', 0) or 0):.3f} "
                            f"best={int(diag.get('best_pack_restore', 0) or 0)} "
                            f"brej={int(diag.get('best_pack_reject', 0) or 0)} "
                            f"cc={int(diag.get('cluster_copy_graph_n', 0) or 0)}/"
                            f"{int(diag.get('cluster_copy_nest_n', 0) or 0)} "
                            f"{diag.get('hybrid_diag', '')} "
                            f"| {diag.get('prop_survival', '')}"
                        )

                avg_parts = np.mean([m.parts_final for m in case_metrics])
                avg_area = np.mean([m.area_coverage for m in case_metrics])
                avg_parts_delta = np.mean([m.parts_delta for m in case_metrics])
                avg_area_delta = np.mean([m.area_coverage_delta for m in case_metrics])
                avg_kiss = np.mean([m.kiss_seed for m in case_metrics])
                avg_kiss_o = np.mean([m.kiss_outline for m in case_metrics])
                avg_kiss_st = np.mean([m.kiss_standoff for m in case_metrics])
                avg_auc = np.mean([m.density_auc for m in case_metrics])
                avg_time = np.mean([m.time_s for m in case_metrics])
                all_indep = all(m.independent_ok for m in case_metrics)

                row = {
                    "case": case.name,
                    "family": case.family,
                    "demand": case.demand_ratio,
                    "propose": str(propose),
                    "dfs_mode": str(dfs_mode),
                    "parts": float(avg_parts),
                    "area": float(avg_area),
                    "parts_delta": float(avg_parts_delta),
                    "area_delta": float(avg_area_delta),
                    "kiss": float(avg_kiss),
                    "kiss_outline": float(avg_kiss_o),
                    "kiss_standoff": float(avg_kiss_st),
                    "density_auc": float(avg_auc),
                    "time": float(avg_time),
                    "independent_ok": all_indep,
                }
                results.append(row)

                if args.gate:
                    fails = metrics_meet_floors(case_metrics[0], case.floors)
                    key = f"{case.name}|{propose}|{dfs_mode}"
                    if key in baselines:
                        b = baselines[key]
                        if avg_parts + 1e-9 < b.get("parts", 0) * 0.9:
                            fails.append("baseline_parts")
                        if avg_area + 1e-9 < b.get("area", 0) * 0.9:
                            fails.append("baseline_area")
                    if fails:
                        gate_fail = True
                        print(f"  GATE FAIL {case.name}: {fails}")

                if args.update_baselines:
                    key = f"{case.name}|{propose}|{dfs_mode}"
                    if not all_indep:
                        print(f"  SKIP baseline update {case.name}: independent_ok=false")
                    elif fails:
                        print(f"  SKIP baseline update {case.name}: gate fails {fails}")
                    else:
                        prev = baselines.get(key) or {}
                        area_ok = float(avg_area) >= float(prev.get("area", 0.0) or 0.0) * 0.999
                        parts_ok = float(avg_parts) >= float(prev.get("parts", 0.0) or 0.0) * 0.999
                        if prev and not (area_ok and parts_ok):
                            print(
                                f"  SKIP baseline update {case.name}: "
                                f"no gain vs baseline area/parts"
                            )
                        else:
                            baselines[key] = {
                                "parts": float(avg_parts),
                                "area": float(avg_area),
                                "time": float(avg_time),
                                "kiss": float(avg_kiss),
                                "independent_ok": bool(all_indep),
                            }

    print(
        "\n| Case | Demand | Propose | DFS | Parts | Area | ΔParts | KissS | KissO | AUC | Time | Indep |"
    )
    print("|------|--------|---------|-----|-------|------|--------|-------|-------|-----|------|-------|")
    for r in results:
        print(
            f"| {r['case']} | {r['demand']:.2f} | {r['propose']} | {r['dfs_mode']} | "
            f"{r['parts']:.1f} | {r['area']:.3f} | {r['parts_delta']:.1f} | "
            f"{r['kiss']:.2f} | {r['kiss_outline']:.2f} | {r['density_auc']:.2f} | "
            f"{r['time']:.2f}s | {r['independent_ok']} |"
        )

    if args.update_baselines:
        args.baselines.parent.mkdir(parents=True, exist_ok=True)
        args.baselines.write_text(json.dumps(baselines, indent=2, sort_keys=True) + "\n")
        print(f"Wrote baselines to {args.baselines}")

    if args.gate and gate_fail:
        sys.exit(1)


if __name__ == "__main__":
    main()
