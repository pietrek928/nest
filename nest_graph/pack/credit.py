"""Void-elite archive, niche credit, and MCTS iter finalize."""

from typing import Any, Sequence

import numpy as np

from nest_graph.graph import MacroRegion
from nest_graph.pack.ctx import VoidLeakOrchResult
from nest_graph.pack.epoch import realize_selection
from nest_graph.pack.motif_credit import (
    credit_motif_on_nest_survival,
    credit_void_niche_from_iter,
    niche_amaf_key,
)
from nest_graph.propose.pattern_archive import note_motif_hollow_miss
from nest_graph.propose.telem import (
    VoidLeakGatherCtx,
    archive_void_elite_transforms,
    format_dg_expand_funnel,
    gather_void_leak_inputs,
    merge_phase_gate_telem,
    void_elite_count,
)
from nest_graph.propose.void_selection import (
    count_props_near_pole,
    transform_row_key,
    void_pole_near_radius,
)


def run_void_leak_and_niche_credit(
    *,
    graph: Any,
    polys: list,
    group_id: list,
    transform: list,
    free_info: Any,
    free_poly: Any,
    selected_nest: Sequence[int],
    selected_polys: Sequence[int],
    refine_scores: Sequence[float],
    propose_stats: dict,
    plateau: Any,
    pin_stats: dict,
    pin_all_blocked_streak: int,
    n_void_nest: int,
    boost_hits: dict,
    void_pole_near_diag_ratio: float,
    proposer_counts: dict,
    sheet_diag: float,
    mcts_telem: dict,
    mcts_runner: Any,
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
    void_leak, leak_dict = gather_void_leak_inputs(
        VoidLeakGatherCtx(
            graph=graph,
            polys=polys,
            group_id=group_id,
            transform=transform,
            free_info=free_info,
            free_poly=free_poly,
            selected_nest=selected_nest,
            selected_polys=selected_polys,
            refine_scores=refine_scores,
            propose_stats=propose_stats,
            plateau=plateau,
            pin_stats=pin_stats,
            pin_all_blocked_streak=pin_all_blocked_streak,
            n_void_nest=n_void_nest,
            boost_hits=boost_hits,
            void_pole_near_diag_ratio=float(void_pole_near_diag_ratio),
            proposer_counts=proposer_counts,
            sheet_diag=float(sheet_diag),
            void_elite_by_group=void_elite_by_group,
            void_elite_count_fn=void_elite_count,
            mcts_telem=mcts_telem,
            mcts_runner=mcts_runner,
        )
    )
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
    # Wp: merge phase-gate keys before print so build_graph void_leak shows
    # place_coh / coh_sig / union (finalize_iter_mcts also merges later).
    merge_phase_gate_telem(leak_dict, mcts_telem, propose_stats)
    funnel = leak_dict["funnel"]
    corners = leak_dict["corners"]
    inward = leak_dict.get("inward") or {}
    propose_stats["r0_bottleneck"] = funnel["bottleneck"]
    if print_funnel:
        print(
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
        print(f"{void_leak}{motif_wire}")
        print(format_dg_expand_funnel(leak_dict))
    bottleneck = str(funnel.get("bottleneck") or "")
    large_void = str(getattr(free_info, "kind", "") or "") == "large_void"
    hollow = bool(
        large_void
        and (bottleneck == "graph_to_nest" or int(n_void_nest) <= 0)
    )
    propose_stats["hollow_miss"] = hollow
    amaf_key = niche_amaf_key(mcts_action)
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
        free_kind=str(getattr(free_info, "kind", "") or ""),
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
        if mid >= 0:
            runner.agent.realized["macro_survive_n"] = int(sc.get(mid, 0) or 0)
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
        leak["macro_survive_n"] = max(
            int(leak.get("macro_survive_n", 0) or 0),
            int(
                (runner.agent.realized or {}).get("macro_survive_n", 0)
                if runner.agent is not None
                else 0
            ),
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


__all__ = [
    "finalize_iter_mcts",
    "run_void_leak_and_niche_credit",
]
