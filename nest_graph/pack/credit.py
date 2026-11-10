"""Void-elite archive, niche credit, and MCTS iter finalize."""

from dataclasses import replace
from typing import Any, Sequence

import numpy as np

from nest_graph.graph import FreeKind, MacroAction, MacroRegion
from nest_graph.pack.ctx import VoidLeakOrchResult
from nest_graph.pack.epoch import realize_selection
from nest_graph.pack.execute import note_mcts_multi_pole
from nest_graph.pack.motif_credit import (
    credit_motif_on_nest_survival,
    credit_void_niche_from_iter,
    niche_amaf_key,
)
from nest_graph.propose.pattern_archive import age_motif_library, note_motif_hollow_miss
from nest_graph.propose.telem import (
    VoidLeakGatherCtx,
    build_motif_telem,
    archive_void_elite_transforms,
    format_dg_expand_funnel,
    gather_void_leak_inputs,
    merge_phase_gate_telem,
    void_elite_count,
)
from nest_graph.free_kind_util import free_kind_token, is_large_void
from nest_graph.propose.void_selection import (
    count_props_near_pole,
    transform_row_key,
    void_pole_near_radius,
)
from nest_graph.telem_log import write_telem


def run_void_leak_and_niche_credit(
    ctx: VoidLeakGatherCtx,
    *,
    mcts_action: Any | None,
    cfg_propose: Any,
    prev_void_nest: int,
    pin_cands: int,
    pin_added: int,
    proposed_list: Sequence | None,
    stratified_void_elite_quota: int = 15,
    print_funnel: bool = True,
) -> VoidLeakOrchResult:
    """Void-elite archive + gather_void_leak + niche credit (build_graph + evaluator)."""
    graph = ctx.graph
    polys = ctx.polys
    group_id = ctx.group_id
    transform = ctx.transform
    free_info = ctx.free_info
    free_poly = ctx.free_poly
    selected_nest = ctx.selected_nest
    selected_polys = ctx.selected_polys
    refine_scores = ctx.refine_scores
    propose_stats = ctx.propose_stats
    plateau = ctx.plateau
    pin_stats = ctx.pin_stats
    pin_all_blocked_streak = ctx.pin_all_blocked_streak
    n_void_nest = ctx.n_void_nest
    boost_hits = ctx.boost_hits
    void_pole_near_diag_ratio = float(ctx.void_pole_near_diag_ratio)
    proposer_counts = ctx.proposer_counts or {}
    sheet_diag = float(ctx.sheet_diag)
    mcts_telem = ctx.mcts_telem if ctx.mcts_telem is not None else {}
    mcts_runner = ctx.mcts_runner
    sheet = ctx.sheet
    n_props_pole = count_props_near_pole(
        proposed_list,
        getattr(free_info, "target_pt", None),
        void_pole_near_radius(
            float(sheet_diag),
            float(void_pole_near_diag_ratio),
        ),
    )
    archive_enabled = bool(getattr(cfg_propose, "enable_void_elite_archive", True))
    if (
        archive_enabled
        and bool(getattr(cfg_propose, "stop_elite_archive_when_pin_blocked", True))
        and pin_cands > 0
        and pin_added == 0
    ):
        densify_keep = int(
            (propose_stats.get("densify_stats") or {}).get("accepted", 0) or 0
        ) > 0
        if not densify_keep and int(n_props_pole) <= 0:
            archive_enabled = False
    void_elite_by_group = archive_void_elite_transforms(
        selected_nest=selected_nest,
        selected_refine=selected_polys,
        polys=polys,
        transforms=transform,
        group_ids=group_id,
        free_poly=free_poly,
        scores=refine_scores,
        max_keep=int(stratified_void_elite_quota),
        enabled=archive_enabled,
    )
    ctx = replace(
        ctx,
        void_elite_by_group=void_elite_by_group,
        void_elite_count_fn=void_elite_count,
        proposer_counts=proposer_counts,
        mcts_telem=mcts_telem,
    )
    void_leak, leak_dict = gather_void_leak_inputs(ctx)
    had_void_override = bool(
        int(leak_dict.get("hijack", 0))
        or propose_stats.get("sat_override", False)
    )
    n_void_graph = int(leak_dict.get("graph", 0))
    outline_cov = float(leak_dict.get("outline_cov", 0.0))
    proposer_keys = (
        propose_stats.get("proposer_keys")
        or (propose_stats.get("densify_stats") or {}).get("proposer_keys")
        or {}
    )
    propose_stats["void_leak"] = leak_dict
    propose_stats["max_void_ratio"] = float(leak_dict.get("max_void_ratio", 0.0) or 0.0)
    propose_stats["void_fill"] = float(leak_dict.get("void_fill", 0.0) or 0.0)
    # GE2: push agent MctsTelem (ucb_beta_*) into mcts_telem before funnel print.
    push = getattr(mcts_runner, "push_state", None)
    if callable(push):
        push()
    if isinstance(mcts_telem, dict) and getattr(mcts_runner, "agent", None) is not None:
        at = getattr(mcts_runner.agent, "telem", None)
        if isinstance(at, dict):
            for _k in (
                "ucb_beta_mean",
                "ucb_beta_min",
                "ucb_beta_n",
                "ucb_beta_sum",
                "pb_decay_n",
            ):
                if _k in at:
                    mcts_telem[_k] = at[_k]
    # Wp: merge phase-gate keys before print so build_graph void_leak shows
    # place_coh / coh_sig / union (finalize_iter_mcts also merges later).
    merge_phase_gate_telem(leak_dict, mcts_telem, propose_stats)
    funnel = leak_dict["funnel"]
    corners = leak_dict["corners"]
    inward = leak_dict.get("inward") or {}
    propose_stats["r0_bottleneck"] = funnel["bottleneck"]
    # P0: path_probe_budget hollow thrift reads this next iter.
    mcts_telem["last_graph_to_nest"] = int(
        str(funnel.get("bottleneck") or "") == "graph_to_nest"
    )
    # Hollow basin flag for next-iter telem (not bn==graph_to_nest alone).
    mcts_telem["last_graph_to_nest_hollow"] = int(
        propose_stats.get("graph_to_nest_hollow", 0) or 0
    )
    if print_funnel:
        write_telem(
            f"r0 bottleneck={funnel['bottleneck']} "
            f"funnel={funnel['funnel_stages']} "
            f"corners_in={corners['corner_in']} kept={corners['corner_kept']} "
            f"inward_e={int(inward.get('inward_emitted', 0))} "
            f"pool={int(inward.get('inward_pool', 0))} "
            f"kept={int(inward.get('inward_kept', 0))} "
            f"ray={int(inward.get('raycasting_emitted', 0))}/"
            f"{int(inward.get('raycasting_pool', 0))} "
            f"voronoi={int(inward.get('voronoi_emitted', 0))}/"
            f"{int(inward.get('voronoi_pool', 0))} "
            f"erosion={int(inward.get('erosion_emitted', 0))}/"
            f"{int(inward.get('erosion_pool', 0))}"
        )
        motif_wire = (
            f" place_coh={int(leak_dict.get('place_cohort_ready', 0) or 0)}/"
            f"{int(leak_dict.get('place_cohort_specs_n', 0) or 0)}/"
            f"{int(leak_dict.get('mcts_cohort_macro_n', 0) or 0)}"
            f" motif_hit={int(leak_dict.get('motif_hit', 0) or 0)}"
            f" path_tip={int(leak_dict.get('path_tip_apply', 0) or 0)}"
            f" path_join={int(leak_dict.get('path_join_signal', 0) or 0)}"
            f" coh_sig={int(leak_dict.get('motif_cohort_sig', 0) or 0)}"
            f" cache_inv_coh={int(leak_dict.get('cache_invalidate_cohort', 0) or 0)}"
            f" union_n={int(leak_dict.get('motif_union_lock_n', 0) or 0)}"
            f" union_win={int(leak_dict.get('motif_union_beam_win', 0) or 0)}/"
            f"{int(leak_dict.get('motif_union_hollow_win', 0) or 0)}"
            f" compose_sz={int(leak_dict.get('motif_compose_accepted_size', 0) or 0)}"
        )
        grow_rej = leak_dict.get("grow_reject_first_obstacle")
        if grow_rej is not None or "grow_reject_later_glue" in leak_dict:
            motif_wire += (
                f" grow_rej="
                f"{int(leak_dict.get('grow_reject_first_obstacle', 0) or 0)}/"
                f"{int(leak_dict.get('grow_reject_later_obstacle', 0) or 0)}/"
                f"{int(leak_dict.get('grow_reject_later_glue', 0) or 0)}/"
                f"{int(leak_dict.get('grow_reject_geom_missing', 0) or 0)}/"
                f"{int(leak_dict.get('grow_reject_none_cg', 0) or 0)}"
            )
        write_telem(f"{void_leak}{motif_wire}")
        write_telem(format_dg_expand_funnel(leak_dict))
    bottleneck = str(funnel.get("bottleneck") or "")
    large_void = is_large_void(free_info)
    hollow = bool(
        large_void
        and (bottleneck == "graph_to_nest" or int(n_void_nest) <= 0)
    )
    propose_stats["hollow_miss"] = hollow
    amaf_key = niche_amaf_key(mcts_action)
    # H0: correlate niche AMAF tuple with 3a/3b outcomes in void_leak.
    try:
        _reg, _rule, _motif = amaf_key
        propose_stats["niche_amaf_region"] = int(_reg)
        propose_stats["niche_amaf_rule"] = int(_rule)
        propose_stats["niche_amaf_motif"] = int(_motif)
    except (TypeError, ValueError):
        propose_stats["niche_amaf_region"] = -1
        propose_stats["niche_amaf_rule"] = 0
        propose_stats["niche_amaf_motif"] = -1
    leak_dict["niche_amaf_region"] = int(propose_stats["niche_amaf_region"])
    leak_dict["niche_amaf_rule"] = int(propose_stats["niche_amaf_rule"])
    leak_dict["niche_amaf_motif"] = int(propose_stats["niche_amaf_motif"])
    # H2: gainful 3a/3b must not poison niche AMAF (path misses stay elsewhere).
    hole_gain = (
        int(propose_stats.get("block_hole_accepted", 0) or 0) > 0
        and (
            int(propose_stats.get("block_hole_dcount", 0) or 0) > 0
            or float(propose_stats.get("block_hole_darea", 0.0) or 0.0) > 0.0
        )
    )
    cohort_gain = (
        int(propose_stats.get("block_cohort_accepted", 0) or 0) > 0
        and (
            int(propose_stats.get("block_cohort_dcount", 0) or 0) > 0
            or float(propose_stats.get("block_cohort_darea", 0.0) or 0.0) > 0.0
        )
    )
    propose_stats["hole_expand_gain"] = int(hole_gain or cohort_gain)
    leak_dict["hole_expand_gain"] = int(propose_stats["hole_expand_gain"])
    if hole_gain or cohort_gain:
        hollow = False
        propose_stats["hollow_miss"] = False
    # Post-wise niche GLS (3b lex|no_gain → note_macro_miss/append_negative)
    # dual-missed early med 0.582 — parked; gainful 3a/3b still clear hollow.
    if hollow and mcts_runner.agent is not None:
        mcts_runner.agent.note_macro_miss(mcts_action)
        if (
            mcts_action is not None
            and mcts_action.region == MacroRegion.Motif
            and int(mcts_action.motif_id) >= 0
        ):
            note_motif_hollow_miss(
                mcts_runner.motif_base, int(mcts_action.motif_id),
            )
    niche_telem = credit_void_niche_from_iter(
        mcts_runner.niche_archive,
        free_kind=free_kind_token(getattr(free_info, "kind", None)),
        bottleneck=bottleneck,
        n_void_nest=int(n_void_nest),
        n_void_graph=int(n_void_graph),
        prev_void_nest=int(prev_void_nest),
        polys=polys,
        transform=transform,
        group_id=group_id,
        selected=selected_nest,
        free_poly=free_poly,
        outline_cov=float(outline_cov),
        ttl=int(getattr(cfg_propose, "accepted_pattern_ttl", 4) or 4),
        max_seed=int(getattr(cfg_propose, "max_proposals", 64) or 64),
        amaf_key=amaf_key,
        proposer_keys=proposer_keys,
    )
    propose_stats.update(niche_telem)
    if isinstance(propose_stats.get("void_leak"), dict):
        propose_stats["void_leak"]["niche_pos"] = int(
            niche_telem.get("niche_pos", 0)
        )
        propose_stats["void_leak"]["niche_rescue"] = int(
            niche_telem.get("niche_rescue", 0)
        )
    new_prev = int(n_void_nest)
    feed = mcts_runner.niche_archive.last_feed_keys
    if feed and large_void:
        nest_keys = {
            transform_row_key(np.asarray(transform[i], dtype=np.float64))
            for i in selected_nest
            if 0 <= int(i) < len(transform)
        }
        placed = bool(feed & nest_keys)
        mcts_runner.niche_archive.note_place_outcome(placed)
    mcts_runner.niche_archive.last_feed_keys = set()
    return VoidLeakOrchResult(
        void_elite_by_group=void_elite_by_group,
        leak_dict=leak_dict,
        void_leak=void_leak,
        had_void_override=had_void_override,
        n_void_graph=n_void_graph,
        outline_cov=outline_cov,
        proposer_keys=proposer_keys,
        niche_telem=niche_telem,
        prev_void_nest=new_prev,
    )


def finalize_iter_mcts(
    runner,
    *,
    selected_polys: Sequence[int],
    group_id: Sequence[int],
    transform: Sequence,
    propose_stats: dict,
    mcts_telem: dict | None,
    motif_keys: dict | None,
    motif_ttl: int,
    credit_motif: bool,
    refine_bp: dict | None = None,
    emitted_bp: dict | None = None,
) -> None:
    """Post-pack DG materialize + Kind/Attach credit (Q165 outer leaf only)."""
    if runner.agent is not None and refine_bp is not None and emitted_bp is not None:
        tot_emit = float(sum(int(v) for v in emitted_bp.values()) or 0)
        tot_surv = float(sum(int(v) for v in refine_bp.values()) or 0)
        if tot_emit > 0.0:
            prop_h = min(1.0, tot_surv / tot_emit)
            if int(propose_stats.get("cache_hit", 0) or 0) > 0 and int(
                propose_stats.get("proposal_count", 0) or 0
            ) <= 0:
                prop_h *= 0.25
            runner.agent.realized["proposer_pb"] = float(prop_h)
    mat = realize_selection(runner.dg, selected_polys, propose_stats)
    if runner.agent is not None:
        sc = mat.get("survive_by_motif") or {}
        runner.agent.realized["survive_by_motif"] = dict(sc)
        runner.agent.realized["survive_motif_n"] = int(sum(int(v) for v in sc.values()))
        runner.agent.realized["materialized_motif"] = int(
            propose_stats.get("materialized_motif", 0) or 0
        )
        mid = int(getattr(getattr(runner, "mcts_action", None), "motif_id", -1) or -1)
        if sc and refine_bp is not None and emitted_bp is not None:
            motif_names = (
                "cluster_copy", "motif", "pocket_fit", "pattern", "archive_mix",
            )
            tagged_e = sum(int(emitted_bp.get(n, 0) or 0) for n in motif_names)
            tagged_r = sum(int(refine_bp.get(n, 0) or 0) for n in motif_names)
            if tagged_e > 0:
                prop_h = min(1.0, float(tagged_r) / float(tagged_e))
                if int(propose_stats.get("cache_hit", 0) or 0) > 0 and int(
                    propose_stats.get("proposal_count", 0) or 0
                ) <= 0:
                    prop_h *= 0.25
                runner.agent.realized["proposer_pb"] = float(prop_h)
        credit_motif_on_nest_survival(
            runner.motif_base,
            selected_polys=selected_polys,
            group_id=group_id,
            transform=transform,
            motif_keys=motif_keys,
            ttl=int(motif_ttl),
            telem=mcts_telem,
            realized_out=runner.agent.realized,
            kind_survive=propose_stats.get("kind_survive_hist"),
            materialized_attach=int(propose_stats.get("materialized_attach", 0) or 0),
            member_hits=int(propose_stats.get("member_hits", 0) or 0),
            credit_motif=bool(credit_motif),
            survive_by_motif=dict(sc),
            upsert_patterns=(
                (propose_stats.get("repack") or {}).get("upsert_patterns")
            ),
        )
        # L2: always overwrite macro_survive_n (Void/Rim used to keep prior Motif value).
        if mid >= 0:
            runner.agent.realized["macro_survive_n"] = int(sc.get(mid, 0) or 0)
        else:
            kind = runner.agent.realized.get("kind") or (0, 0, 0, 0)
            action = getattr(runner, "mcts_action", None)
            ri = -1
            if action is not None:
                reg = getattr(action, "region", None)
                ri = int(getattr(reg, "value", reg) if reg is not None else -1)
            if 0 <= ri < 4:
                runner.agent.realized["macro_survive_n"] = int(kind[ri])
            else:
                runner.agent.realized["macro_survive_n"] = 0
        # L2: after kind credit, refresh poles from stats + touch Void AMAF pick
        # so floor telem lands when expand skipped pick. No forced pole invent.
        note_mcts_multi_pole(runner.agent, propose_stats=propose_stats)
        if int(getattr(runner.agent, "multi_pole_n", 0) or 0) < 2:
            n_poles = int(propose_stats.get("multi_pole_count", 0) or 0)
            if n_poles < 2 and isinstance(propose_stats.get("void_leak"), dict):
                n_poles = int(
                    propose_stats["void_leak"].get("multi_pole_count", 0) or 0
                )
            if n_poles >= 2:
                setter = getattr(runner.agent, "set_multi_pole_n", None)
                if setter is not None:
                    setter(int(n_poles))
        if int(getattr(runner.agent, "multi_pole_n", 0) or 0) >= 2:
            touch = MacroAction()
            touch.region = MacroRegion.Void
            touch.rule_id = 0
            touch.motif_id = -1
            key = runner.agent._action_key(touch)
            runner.agent._amaf_pick_score(
                key, 0, free_kind=FreeKind.Empty
            )
            floor_n = int(runner.agent.telem.get("amaf_pole_floor", 0) or 0)
            if mcts_telem is not None:
                mcts_telem["amaf_pole_floor"] = floor_n
                mcts_telem["multi_pole_n"] = int(
                    getattr(runner.agent, "multi_pole_n", 0) or 0
                )
            propose_stats["amaf_pole_floor"] = floor_n
    if isinstance(propose_stats.get("void_leak"), dict):
        leak = propose_stats["void_leak"]
        leak["kind_survive"] = int(propose_stats.get("kind_survive", 0) or 0)
        leak["materialized_attach"] = int(
            propose_stats.get("materialized_attach", 0) or 0
        )
        leak["member_hits"] = int(propose_stats.get("member_hits", 0) or 0)
        leak["materialized_motif"] = int(
            propose_stats.get("materialized_motif", 0) or 0
        )
        leak["survive_motif_n"] = max(
            int(leak.get("survive_motif_n", 0) or 0),
            int(propose_stats.get("survive_motif_n", 0) or 0),
            int(
                (runner.agent.realized or {}).get("survive_motif_n", 0)
                if runner.agent is not None
                else 0
            ),
        )
        # L2: assign, do not max-mask stale Motif tip values onto Void tips.
        leak["macro_survive_n"] = int(
            (runner.agent.realized or {}).get("macro_survive_n", 0)
            if runner.agent is not None
            else 0
        )
        leak["motif_compose_accepted_size"] = int(
            propose_stats.get("motif_compose_accepted_size", 0) or 0
        )
        leak["motif_beam_sets"] = int(propose_stats.get("motif_beam_sets", 0) or 0)
        leak["motif_join_n"] = max(
            int(leak.get("motif_join_n", 0) or 0),
            int(propose_stats.get("motif_join_n", 0) or 0),
        )
        leak["motif_graph_hit_n"] = max(
            int(leak.get("motif_graph_hit_n", 0) or 0),
            int(propose_stats.get("motif_graph_hit_n", 0) or 0),
        )
        cohorts = propose_stats.get("motif_cohorts") or ()
        leak["motif_cohorts_n"] = max(
            int(leak.get("motif_cohorts_n", 0) or 0),
            int(len(cohorts)),
        )
        merge_phase_gate_telem(leak, mcts_telem, propose_stats)



def age_iter_archives(
    *,
    mcts_runner: Any,
    mcts_parent_id: int,
    mcts_telem: dict,
    propose_stats: dict,
    cfg_propose: Any,
    pattern_accept: bool,
    motif_refine_n: int,
) -> dict:
    """Q142: Motif/niche/arena age + motif telem (build_graph-only sibling of finalize)."""
    if bool(getattr(cfg_propose, "enable_accepted_pattern_archive", True)):
        if not pattern_accept:
            age_motif_library(mcts_runner.motif_base, 1)
        mcts_runner.niche_archive.age(1)
        if mcts_runner.agent is not None:
            mcts_runner.agent.age_and_tombstone(
                spine_id=int(mcts_parent_id),
                idle_t=4,
                force=bool(
                    str(mcts_telem.get("ram_band") or "") in ("pressure", "critical")
                ),
            )
    lib_n = int(mcts_runner.motif_base.size())
    propose_stats["motif_refine_hits"] = int(motif_refine_n)
    propose_stats["accepted_patterns_archived"] = lib_n
    propose_stats["motif_library_n"] = lib_n
    propose_stats["motif_base_seed_n"] = int(
        mcts_telem.get("motif_base_seed_n", 0) or 0
    )
    propose_stats["pattern_accept"] = bool(pattern_accept)
    densify_full = propose_stats.get("densify_stats") or {}
    skip_map: dict[str, int] = {}
    densify_skips = densify_full.get("pocket_skip") or []
    if isinstance(densify_skips, dict):
        skip_map = {str(k): int(v) for k, v in densify_skips.items()}
    elif isinstance(densify_skips, list):
        for s in densify_skips:
            skip_map[str(s)] = skip_map.get(str(s), 0) + 1
    for k, v in (densify_full.get("motif_skip") or {}).items():
        skip_map[str(k)] = int(v)
    motif_telem = build_motif_telem(
        skip_map=skip_map,
        motif_refine_n=motif_refine_n,
        propose_stats=propose_stats,
        archive_len=lib_n,
    )
    motif_telem["motif_library_n"] = lib_n
    motif_telem["motif_library_inject"] = int(
        propose_stats.get("accepted_patterns_n", 0)
    )
    if isinstance(propose_stats.get("void_leak"), dict):
        propose_stats["void_leak"]["motif_telem"] = motif_telem
        for _mk in (
            "full_motif_clear",
            "fallback_leader",
            "leader_fail",
            "collide",
            "lattice_anchors_added",
            "lattice_anchors_kept",
            "foreign_clear_fail_sheet",
            "foreign_clear_fail_obs",
            "foreign_clear_fail_empty",
            "soft_foreign_incumbent_n",
        ):
            propose_stats["void_leak"][_mk] = max(
                int(propose_stats["void_leak"].get(_mk, 0) or 0),
                int(motif_telem.get(_mk, 0) or 0),
            )
    propose_stats["motif_telem"] = motif_telem
    return motif_telem

__all__ = [
    "age_iter_archives",
    "finalize_iter_mcts",
    "run_void_leak_and_niche_credit",
]
