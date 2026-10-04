"""Macro-tier augmenting path (Q296–Q300 / Q384–Q386)."""

import time
from collections.abc import Sequence
from typing import Any, Callable

from nest_graph.cohort_specs import generate_macros, motif_cohort_specs
from nest_graph.graph import (
    BoardSnapshot,
    MacroRegion,
    PathKind,
    leaf_reward,
    node_macro,
    path_reward_beats,
    rank_motif_join_neighbors,
    region_to_zone,
    sibling_macro_actions,
    survive_rank,
)


def ancestors(runner: Any, node_id: int) -> list[int]:
    """Parent-walk from root to node_id (Q304)."""
    return list(runner.arena.ancestors(int(node_id)))


def path_probe_budget(
    *,
    on_plateau: bool,
    parent_free_hint: bool,
    agent: Any,
    tip_action: Any,
    beam: int,
    max_depth: int,
    path_carved_streak: int = 0,
    incumbent_hold: bool = False,
    pick_empty: bool = False,
) -> tuple[bool, int, int]:
    """D1/G1/P1/P: run/shrink macro_increase_path from place_cohort_ready + Motif tip.

    Returns ``(run, beam, max_depth)``. P1: when ready=0 and tip is not Motif but
    ``parent_free_hint`` (large_void), shrink-run with beam/4 (not skip). Motif tip
    with ready=0 still shrinks beam/2. G1: ready but macros idle → mild shrink.
    P: when ``path_carved_streak≥2`` and hold/plateau, further thrift — skip when
    ready∧pick_empty∧non-Motif tip; else shrink beyond P1. Motif tip fuel kept.
    """
    tip_motif = (
        tip_action is not None
        and getattr(tip_action, "region", None) == MacroRegion.Motif
    )
    if not (bool(on_plateau) or bool(parent_free_hint)):
        # P: Motif tip alone still probes (was dead behind free_hint-only gate).
        if tip_motif:
            return True, max(1, int(beam) // 2), max(1, int(max_depth) - 1)
        return False, int(beam), int(max_depth)
    ready = bool(getattr(agent, "place_cohort_ready", False)) if agent is not None else False
    telem = getattr(agent, "telem", None) if agent is not None else None
    macros_n = 0
    if isinstance(telem, dict):
        macros_n = int(telem.get("mcts_cohort_macro_n", 0) or 0)
    elif telem is not None:
        macros_n = int(getattr(telem, "mcts_cohort_macro_n", 0) or 0)
    if ready:
        # G1: ready but PLACE_COHORT macros idle and tip not Motif → mild shrink
        # (never skip while ready — dual miss when skip-on-collapse).
        if macros_n <= 0 and not tip_motif:
            run, b, d = (
                True,
                max(2, int(beam) // 2),
                max(2, int(max_depth) - 1),
            )
        else:
            run, b, d = True, max(1, int(beam)), max(1, int(max_depth))
    elif tip_motif:
        run, b, d = True, max(1, int(beam) // 2), max(1, int(max_depth) - 1)
    elif bool(parent_free_hint):
        # P1: large_void free hint — shrink-run with tight budget (beam/4).
        # Full beam/2 on every free-hint iter missed early dual; skip left late idle.
        run, b, d = (
            True,
            max(1, int(beam) // 4),
            max(1, int(max_depth) - 2),
        )
    else:
        return False, int(beam), int(max_depth)
    # P: idle carve streak thrift (one gate; Motif tip budget preserved).
    idle_thrift = (
        int(path_carved_streak) >= 2
        and (bool(incumbent_hold) or bool(on_plateau))
        and not tip_motif
    )
    if idle_thrift:
        if ready and bool(pick_empty):
            return False, int(b), int(d)
        return (
            True,
            max(1, int(b) // 2),
            max(1, int(d) - 1),
        )
    return bool(run), int(b), int(d)


def _realized_dict(agent: Any) -> dict:
    return dict(getattr(agent, "realized", None) or {})


def _place_cohort_raw(agent: Any):
    """M2b: raw cohort dicts only when readiness sticky is green."""
    if not bool(getattr(agent, "place_cohort_ready", False)):
        return ()
    return getattr(agent, "motif_cohorts", None) or ()


def _place_cohort_specs(agent: Any) -> list:
    """M2b: PLACE_COHORT macro specs only when readiness sticky is green."""
    return motif_cohort_specs(_place_cohort_raw(agent))


def _sibling_actions(
    agent: Any,
    blocked_action: Any,
    remaining_gids: tuple[int, ...],
    *,
    rule_ids: tuple[int, ...],
    snapshot: BoardSnapshot | None,
) -> list[Any]:
    free_kind = str(getattr(snapshot, "free_kind", "") or "") if snapshot else ""
    return list(
        sibling_macro_actions(
            [int(g) for g in remaining_gids],
            [int(r) for r in rule_ids],
            agent.motif_base,
            True,
            [int(m) for m in agent._warm_motif_ids(snapshot)] if snapshot else [],
            free_kind,
            _place_cohort_specs(agent),
            blocked_action,
        )
    )


def _path_neighbors_ranked(runner: Any, macro_node_id: int, agent: Any) -> list[Any]:
    """Structural MotifJoin neighbors via DecisionGraph.neighbors (Q378/Q386)."""
    dg = getattr(runner, "dg", None)
    if dg is None:
        return []
    realized = _realized_dict(agent)
    sc = realized.get("survive_by_motif") or {}
    try:
        return list(
            rank_motif_join_neighbors(
                dg,
                int(macro_node_id),
                {int(k): int(v) for k, v in dict(sc).items()},
            )
        )
    except Exception:
        return []


def _pose_validate_hops(
    dg: Any, join_nbrs: list[Any], *, cap: int
) -> list[Any]:
    """E2: Pose validate as second hop from Join/Attach beam (cap ~2*beam)."""
    if dg is None or not join_nbrs or cap <= 0:
        return []
    out: list[Any] = []
    seen: set[int] = set()
    for join_step in join_nbrs:
        try:
            for nbr in dg.neighbors(join_step):
                if getattr(nbr, "kind", None) != PathKind.Pose:
                    continue
                pid = int(getattr(nbr, "id", -1))
                if pid in seen:
                    continue
                seen.add(pid)
                out.append(nbr)
                if len(out) >= int(cap):
                    return out
        except Exception:
            continue
    return out



def record_path_edge_census(
    runner: Any,
    macro_node_id: int,
    telem: dict,
    *,
    motif_locked: Sequence | None = None,
) -> None:
    """E0/E2: PathKind neighbor census + pose hop-2 (telem only; no path mutate)."""
    dg = getattr(runner, "dg", None)
    telem.setdefault("path_nbr_macro", 0)
    telem.setdefault("path_nbr_join", 0)
    telem.setdefault("path_nbr_attach", 0)
    telem.setdefault("path_nbr_pose", 0)
    telem.setdefault("path_conflicts_cut", 0)
    telem.setdefault("path_lock_join_overlap", 0)
    telem.setdefault("motif_join_from_base", 0)
    # Also mirror path_step_* when present (probe telem).
    telem["path_nbr_macro"] = max(
        int(telem.get("path_nbr_macro", 0) or 0),
        int(telem.get("path_step_macro", 0) or 0),
    )
    telem["path_nbr_join"] = max(
        int(telem.get("path_nbr_join", 0) or 0),
        int(telem.get("path_step_join", 0) or 0),
    )
    telem["path_nbr_attach"] = max(
        int(telem.get("path_nbr_attach", 0) or 0),
        int(telem.get("path_step_attach", 0) or 0),
    )
    if dg is None:
        return
    n_macro = n_join = n_attach = 0
    join_nbrs: list[Any] = []
    try:
        for nbr in dg.neighbors(node_macro(int(macro_node_id))):
            kind = getattr(nbr, "kind", None)
            if kind == PathKind.Macro:
                n_macro += 1
            elif kind == PathKind.MotifJoin:
                n_join += 1
                join_nbrs.append(nbr)
            elif kind == PathKind.Attach:
                n_attach += 1
                join_nbrs.append(nbr)
    except Exception:
        pass
    pose_n = len(
        _pose_validate_hops(dg, join_nbrs, cap=max(2 * max(len(join_nbrs), 1), 1))
    )
    telem["path_nbr_macro"] = max(int(telem.get("path_nbr_macro", 0) or 0), n_macro)
    telem["path_nbr_join"] = max(int(telem.get("path_nbr_join", 0) or 0), n_join)
    telem["path_nbr_attach"] = max(int(telem.get("path_nbr_attach", 0) or 0), n_attach)
    telem["path_nbr_pose"] = max(int(telem.get("path_nbr_pose", 0) or 0), pose_n)
    # E4: Join/Attach endpoints ∩ prior motif_locked pose indices (telem).
    lock_idxs = {
        int(i) for i in (motif_locked or ()) if isinstance(i, (int, float)) and int(i) >= 0
    }
    overlap = 0
    if lock_idxs:
        for nbr in join_nbrs:
            a = int(getattr(nbr, "a", -1))
            b = int(getattr(nbr, "b", -1))
            if a in lock_idxs or b in lock_idxs:
                overlap += 1
    telem["path_lock_join_overlap"] = max(
        int(telem.get("path_lock_join_overlap", 0) or 0), int(overlap)
    )


def _validate_join_bonus(
    agent: Any, join_step: Any, baseline: float, telem: dict | None = None
) -> float:
    mid = int(getattr(join_step, "motif_id", -1) or -1)
    survive = float(
        _realized_dict(agent).get("survive_by_motif", {}).get(mid, 0) or 0
    )
    # W: soft-scale by MotifBase accept_count + motif_adj_hits (Wang adj fuel).
    accept_scale = 1.0
    mb = getattr(agent, "motif_base", None)
    if mb is not None and mid >= 0:
        try:
            if int(mb.size()) > mid:
                rec = mb.at(mid)
                ac = float(getattr(rec, "accept_count", 0) or 0)
                accept_scale = 1.0 + 0.05 * min(ac, 8.0)
        except Exception:
            accept_scale = 1.0
    adj_hits = 0.0
    src = telem if isinstance(telem, dict) else getattr(agent, "telem", None)
    try:
        adj_hits = float((src or {}).get("motif_adj_hits", 0) or 0)
    except Exception:
        adj_hits = 0.0
    wang_scale = accept_scale * (1.0 + 0.02 * min(adj_hits, 8.0))
    bonus = (0.02 + 0.01 * survive) * wang_scale
    # A: Attach parity — slight soft bump (same validate-only path).
    if getattr(join_step, "kind", None) == PathKind.Attach:
        bonus *= 0.85
    if isinstance(telem, dict):
        telem["path_wang_scale"] = float(
            max(float(telem.get("path_wang_scale", 0.0) or 0.0), wang_scale)
        )
    return float(baseline) + bonus


def macro_increase_path(
    runner: Any,
    *,
    leaf_id: int,
    baseline_reward: float,
    execute_fn: Callable[..., BoardSnapshot] | None,
    rule_ids: tuple[int, ...] = (0,),
    max_depth: int = 3,
    beam: int = 4,
    telem: dict | None = None,
    overlap_ok_fn: Callable[..., bool] | None = None,
    prefer_motif_return: bool = False,
    motif_hold_tip: bool = False,
    force_motif_action: Any | None = None,
) -> tuple[Any | None, float, BoardSnapshot | None]:
    """Plateau sibling swap + cheap replay from ancestor snapshot (Q297/Q305/Q384).

    Returns ``(best_action, best_reward, best_snap)``. MotifJoin/Pose steps are
    validate-only (Q386); only Macro siblings call ``execute_fn``.
    P: ``prefer_motif_return`` (build_graph) surfaces Motif soft tip telem without
    early evaluator Motif-prefer regress.
    M: ``motif_hold_tip`` — on hold/plateau/rim large_void, track Motif on cov
    non-regress (not path_reward_beats); pair with prefer_motif_return.
    ``force_motif_action`` prepends a Motif Macro when generate_macros has none.
    """
    agent = getattr(runner, "agent", None)
    if agent is None or execute_fn is None:
        return None, float(baseline_reward), None
    t0 = time.perf_counter()
    telem = telem if telem is not None else agent.telem
    # T: probe body entered (distinct from path_tip_apply / soft Motif credit).
    telem["path_ran"] = 1
    telem["macro_swap_attempts"] = int(telem.get("macro_swap_attempts", 0) or 0)
    path = ancestors(runner, int(leaf_id))
    telem["policy_path_len"] = int(len(path))
    best_action: Any | None = None
    best_reward = float(baseline_reward)
    best_snap: BoardSnapshot | None = None
    best_motif_action: Any | None = None
    best_motif_reward = float(baseline_reward)
    best_motif_snap: BoardSnapshot | None = None
    attempts = 0
    accept = 0
    swap_depth = 0
    path_step_macro = 0
    path_step_join = 0
    path_extend_n = 0
    macro_chain_accept = 0
    path_type_hist = [0, 0, 0, 0]
    realized = _realized_dict(agent)
    # Depth 0: when leaf is root-only, still PW-expand alternate macros.
    depth_lo = 0 if len(path) <= 1 else 1
    depth_hi = max(len(path), 1)
    for depth in range(depth_lo, min(depth_hi, max(int(max_depth), 1) + 1)):
        if len(path) <= 1:
            anc_id = int(path[0])
            block_node = int(path[0])
            blocked = None
        else:
            anc_id = int(path[-(depth + 1)] if depth < len(path) else path[0])
            block_node = int(path[-depth])
            try:
                blocked = runner.arena.action(block_node)
            except Exception:
                continue
            if blocked is None:
                continue
        anc_snap = runner.snapshot_at(anc_id, missing_ok=True)
        if anc_snap is None or not anc_snap.remaining_gids:
            continue
        rem = tuple(int(g) for g in anc_snap.remaining_gids)
        if blocked is None:
            siblings = generate_macros(
                rem,
                rule_ids=tuple(int(r) for r in rule_ids),
                motif_base=agent.motif_base,
                prefer_motifs=True,
                warm_motif_ids=agent._warm_motif_ids(anc_snap),
                free_kind=str(getattr(anc_snap, "free_kind", "") or ""),
                motif_cohorts=_place_cohort_raw(agent),
            )
        else:
            siblings = _sibling_actions(
                agent,
                blocked,
                rem,
                rule_ids=tuple(int(r) for r in rule_ids),
                snapshot=anc_snap,
            )
        # M: hold tip needs Motif in the beam (late rem often lacks Motif pair gids).
        if motif_hold_tip and not any(
            getattr(a, "region", None) == MacroRegion.Motif for a in siblings
        ):
            motif_extra = [
                a
                for a in generate_macros(
                    rem,
                    rule_ids=tuple(int(r) for r in rule_ids),
                    motif_base=agent.motif_base,
                    prefer_motifs=True,
                    warm_motif_ids=agent._warm_motif_ids(anc_snap),
                    free_kind=str(getattr(anc_snap, "free_kind", "") or ""),
                    motif_cohorts=_place_cohort_raw(agent),
                )
                if getattr(a, "region", None) == MacroRegion.Motif
            ]
            if not motif_extra and force_motif_action is not None:
                motif_extra = [force_motif_action]
            siblings = list(motif_extra) + list(siblings)
        siblings.sort(key=lambda a: -survive_rank(a, realized))
        beam_n = max(int(beam), 1)
        if motif_hold_tip:
            motifs = [
                a
                for a in siblings
                if getattr(a, "region", None) == MacroRegion.Motif
            ]
            non = [
                a
                for a in siblings
                if getattr(a, "region", None) != MacroRegion.Motif
            ]
            siblings = (motifs[:1] + non)[:beam_n]
        else:
            siblings = siblings[:beam_n]
        join_nbrs = _path_neighbors_ranked(runner, block_node, agent)[:beam]
        # Q386: MotifJoin/Attach neighbors validate-only (no pack).
        # E2 pose hop-2 is census-only (record_path_edge_census); active walk cancelled.
        path_step_attach = 0
        for join_step in join_nbrs:
            path_extend_n += 1
            kind = getattr(join_step, "kind", None)
            if kind == PathKind.Attach:
                path_step_attach += 1
                path_type_hist[int(getattr(PathKind.Attach, "value", 2))] += 1
            else:
                path_step_join += 1
                path_type_hist[int(getattr(PathKind.MotifJoin, "value", 1))] += 1
            dg = getattr(runner, "dg", None)
            if dg is not None and hasattr(dg, "realized") and dg.realized(join_step):
                scored = _validate_join_bonus(
                    agent, join_step, baseline_reward, telem=telem
                )
                if scored > best_reward:
                    best_reward = scored
                    macro_chain_accept += 1
        telem["path_step_attach"] = int(
            telem.get("path_step_attach", 0) or 0
        ) + int(path_step_attach)
        for alt in siblings:
            attempts += 1
            path_extend_n += 1
            path_step_macro += 1
            path_type_hist[int(getattr(PathKind.Macro, "value", 0))] += 1
            zone = region_to_zone(alt.region)
            try:
                child_snap = execute_fn(
                    anc_snap,
                    zone=zone,
                    action=alt,
                    patterns=[],
                )
            except Exception:
                continue
            # Q384: overlap on **replay** snapshot (after execute updates pack_cache).
            if overlap_ok_fn is not None:
                try:
                    ok = bool(overlap_ok_fn(child_snap))
                except TypeError:
                    ok = bool(overlap_ok_fn())
                if not ok:
                    telem["macro_path_overlap_skip"] = int(
                        telem.get("macro_path_overlap_skip", 0) or 0
                    ) + 1
                    try:
                        agent.note_macro_miss(alt)
                    except Exception:
                        pass
                    continue
            reward = leaf_reward(
                child_snap,
                rule_id=int(getattr(alt, "rule_id", 0) or 0),
                survive_motif_n=int(realized.get("survive_motif_n", 0) or 0),
                macro_survive_n=int(realized.get("macro_survive_n", 0) or 0),
                member_hits=int(realized.get("member_hits", 0) or 0),
                materialized_motif=int(realized.get("materialized_motif", 0) or 0),
            )
            reward += 0.01 * survive_rank(alt, realized)
            # P: Motif tip telem is via build_graph Motif soft path_accept_apply;
            # do not inflate Motif path reward (early parts regress under +0.02/+0.08).
            # M4b: Macro execute + MotifJoin validate chain bonus.
            chain_bonus = 0.0
            for join_step in join_nbrs:
                dg = getattr(runner, "dg", None)
                if dg is not None and hasattr(dg, "realized") and dg.realized(join_step):
                    chain_bonus = max(
                        chain_bonus,
                        0.015 + 0.005 * survive_rank(alt, realized),
                    )
            if chain_bonus > 0.0:
                reward += chain_bonus
                path_step_join += 1
                macro_chain_accept += 1
            anc_base = leaf_reward(
                anc_snap,
                survive_motif_n=int(realized.get("survive_motif_n", 0) or 0),
                macro_survive_n=int(realized.get("macro_survive_n", 0) or 0),
            )
            if path_reward_beats(
                anc_snap,
                child_snap,
                base_reward=anc_base,
                alt_reward=reward,
            ) and reward > best_reward:
                best_reward = float(reward)
                best_action = alt
                best_snap = child_snap
                swap_depth = int(depth)
                accept += 1
            # M: Motif on large_void — hold/plateau tip tracks cov non-regress
            # (unify with relax_motif_void_fill); else require path_reward_beats.
            # Prefer-return gated by prefer_motif_return (build_graph phantom gate).
            if getattr(alt, "region", None) == MacroRegion.Motif:
                free_k = str(getattr(child_snap, "free_kind", "") or "")
                if not free_k:
                    free_k = str(getattr(anc_snap, "free_kind", "") or "")
                child_cov = float(getattr(child_snap, "coverage", 0.0) or 0.0)
                anc_cov = float(getattr(anc_snap, "coverage", 0.0) or 0.0)
                motif_cov_ok = child_cov + 1e-9 >= anc_cov
                motif_beats = path_reward_beats(
                    anc_snap,
                    child_snap,
                    base_reward=anc_base,
                    alt_reward=reward,
                )
                take_motif = False
                if free_k == "large_void" and motif_cov_ok:
                    if motif_hold_tip or motif_beats:
                        take_motif = (
                            best_motif_action is None or reward > best_motif_reward
                        )
                elif motif_beats and reward > best_motif_reward:
                    take_motif = True
                if take_motif:
                    best_motif_reward = float(reward)
                    best_motif_action = alt
                    best_motif_snap = child_snap
                    if accept == 0:
                        accept = 1
    telem["macro_swap_attempts"] = int(
        telem.get("macro_swap_attempts", 0) or 0
    ) + int(attempts)
    telem["macro_path_accept"] = int(telem.get("macro_path_accept", 0) or 0) + int(
        accept > 0
    )
    telem["macro_swap_depth"] = max(
        int(telem.get("macro_swap_depth", 0) or 0),
        int(swap_depth),
    )
    telem["macro_path_beam_n"] = int(telem.get("macro_path_beam_n", 0) or 0) + int(
        min(int(beam), max(attempts, 0))
    )
    telem["path_step_macro"] = int(telem.get("path_step_macro", 0) or 0) + int(path_step_macro)
    telem["path_step_join"] = int(telem.get("path_step_join", 0) or 0) + int(path_step_join)
    telem["path_extend_n"] = int(telem.get("path_extend_n", 0) or 0) + int(path_extend_n)
    telem["macro_chain_accept"] = int(telem.get("macro_chain_accept", 0) or 0) + int(
        macro_chain_accept
    )
    # T0: MotifJoin-on-accept is a soft signal, not a real tip install.
    # path_tip_apply stays reserved for path_accept_apply tip_install only.
    if accept > 0 and path_step_join > 0:
        telem["path_join_signal"] = int(telem.get("path_join_signal", 0) or 0) + 1
        telem["macro_path_motif_soft"] = int(
            telem.get("macro_path_motif_soft", 0) or 0
        ) + 1
    prev_hist = list(telem.get("path_type_hist") or [0, 0, 0, 0])
    prev_hist = (prev_hist + [0, 0, 0, 0])[:4]
    telem["path_type_hist"] = [
        int(prev_hist[i]) + int(path_type_hist[i]) for i in range(4)
    ]
    # G0: per-call overwrite (not +=) so dg_funnel path_ms is this probe only.
    telem["replay_from_ancestor_ms"] = (time.perf_counter() - t0) * 1000.0
    # T: Motif soft track vs prefer-return discard / empty alt.
    telem["path_motif_tracked"] = int(best_motif_action is not None)
    if best_motif_action is not None and not prefer_motif_return:
        telem["path_motif_discarded"] = int(
            telem.get("path_motif_discarded", 0) or 0
        ) + 1
    # P: surface Motif path candidate for accept (tip_install telem); outer tip swap
    # is gated in build_graph (Motif soft does not thrash late Δ). Early evaluator
    # keeps Void path winners (prefer_motif_return=False) to hold floors.
    if prefer_motif_return and best_motif_action is not None:
        return best_motif_action, float(best_motif_reward), best_motif_snap
    if best_action is None and best_motif_action is None:
        telem["path_alt_none"] = int(telem.get("path_alt_none", 0) or 0) + 1
    return best_action, best_reward, best_snap


def path_accept_eligible(
    *,
    alt_action: Any,
    path_accept_snap: Any,
    parent_snap: Any,
    base_reward: float,
    alt_reward: float,
    path_overlap_ok: bool,
    min_cov_delta: float = 0.005,
    min_void_fill_delta: float = 0.02,
    relax_motif_void_fill: bool = False,
    fits_part: bool = True,
) -> tuple[bool, bool, float, float]:
    """Shared path-accept gate (Dp1/P2/P): reward + cov + overlap (+ void on large_void).

    Returns ``(eligible, cov_ok, base_cov, alt_cov)``.
    P2: on ``free_kind==large_void``, ``cov_ok`` if coverage non-regress **and**
    ``void_fill`` Δ ≥ ``min_void_fill_delta`` (overlap still mandatory).
    P: when ``relax_motif_void_fill`` and Motif on large_void, drop void_fill Δ —
    overlap + coverage non-regress only (build_graph late tip path).
    F: void_fill Δ branch only when ``fits_part`` (phantom large_void → cov+delta).
    Non-void unchanged (cov + min_cov_delta).
    """
    if alt_action is None or path_accept_snap is None:
        return False, False, 0.0, 0.0
    is_motif = getattr(alt_action, "region", None) == MacroRegion.Motif
    base_cov = float(getattr(parent_snap, "coverage", 0.0) or 0.0)
    alt_cov = float(getattr(path_accept_snap, "coverage", 0.0) or 0.0)
    free_kind = str(getattr(path_accept_snap, "free_kind", "") or "")
    if not free_kind:
        free_kind = str(getattr(parent_snap, "free_kind", "") or "")
    # P: Motif on large_void — drop void_fill Δ; require overlap + cov non-regress
    # (do not re-gate vs outer parent reward; macro_increase_path already filtered).
    if relax_motif_void_fill and is_motif and free_kind == "large_void":
        cov_ok = bool(path_overlap_ok) and alt_cov + 1e-9 >= base_cov
        return True, cov_ok, base_cov, alt_cov
    if not path_reward_beats(
        parent_snap,
        path_accept_snap,
        base_reward=float(base_reward),
        alt_reward=float(alt_reward),
    ):
        return False, False, 0.0, 0.0
    if free_kind == "large_void" and bool(fits_part):
        base_vf = float(getattr(parent_snap, "void_fill", 0.0) or 0.0)
        alt_vf = float(getattr(path_accept_snap, "void_fill", 0.0) or 0.0)
        cov_ok = (
            bool(path_overlap_ok)
            and alt_cov + 1e-9 >= base_cov
            and (alt_vf - base_vf) + 1e-12 >= float(min_void_fill_delta)
        )
    else:
        cov_ok = (
            bool(path_overlap_ok)
            and alt_cov + 1e-9 >= base_cov + float(min_cov_delta)
        )
    return True, cov_ok, base_cov, alt_cov


def path_accept_apply(
    *,
    mode: str,
    cov_ok: bool,
    path_overlap_ok: bool,
    alt_action: Any,
    enable_macro_path_replay: bool,
    mutate_motif_base_on_path: bool,
    telem: dict,
    free_kind: str = "",
) -> dict[str, Any]:
    """Dp1 apply decision: tip-install vs credit-only vs Motif-soft.

    ``mode``:
      - ``build_graph``: Motif soft tip-install without replay flag
      - ``evaluator``: tip-install only when enable_macro_path_replay
    ``free_kind`` reserved for late-gated Void tip (P1 trial regressed early).
    Returns dict with tip_install / credit / motif_soft / upsert / skip flags.
    """
    _ = free_kind
    out = {
        "tip_install": False,
        "credit": False,
        "motif_soft": False,
        "upsert": False,
        "overlap_skip": False,
        "cov_skip": False,
    }
    if not path_overlap_ok:
        out["overlap_skip"] = True
        return out
    is_motif = getattr(alt_action, "region", None) == MacroRegion.Motif
    apply_replay = bool(enable_macro_path_replay)
    # P: never tip-install on cov_skip without coverage check (banned early 0.522).
    if not cov_ok:
        out["cov_skip"] = True
        if mode == "build_graph" and (apply_replay or is_motif):
            out["credit"] = True
            out["motif_soft"] = bool(is_motif and not apply_replay)
        return out
    if mode == "evaluator":
        # Evaluator: tip-install only with enable_macro_path_replay (early Motif soft
        # tip-install regressed area). Late build_graph Motif soft proves path_tip.
        if apply_replay:
            out["tip_install"] = True
            out["credit"] = True
            out["upsert"] = bool(mutate_motif_base_on_path)
        else:
            out["credit"] = True
            out["motif_soft"] = bool(is_motif)
        telem["path_tip_apply"] = int(telem.get("path_tip_apply", 0) or 0) + int(
            out["tip_install"]
        )
        return out
    # build_graph: Motif soft OR replay → tip install; else credit-only (Q351).
    if apply_replay or is_motif:
        out["tip_install"] = True
        out["credit"] = True
        out["motif_soft"] = bool(is_motif and not apply_replay)
        out["upsert"] = bool(mutate_motif_base_on_path)
    else:
        out["credit"] = True
    telem["path_tip_apply"] = int(telem.get("path_tip_apply", 0) or 0) + int(
        out["tip_install"]
    )
    return out


__all__ = [
    "ancestors",
    "macro_increase_path",
    "path_accept_apply",
    "path_accept_eligible",
    "path_probe_budget",
    "record_path_edge_census",
]
