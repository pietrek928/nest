"""Macro-tier augmenting path (Q296–Q300 / Q384–Q386)."""

import time
from collections.abc import Sequence
from typing import Any, Callable

from nest_graph.cohort_specs import generate_macros, motif_cohort_specs
from nest_graph.free_kind_util import free_kind_from_any, free_kind_token, is_large_void
from nest_graph.graph import (
    BoardSnapshot,
    FreeKind,
    MacroRegion,
    PathKind,
    leaf_reward,
    node_macro,
    path_accept_eligible_native,
    path_reward_beats,
    rank_motif_join_neighbors,
    region_to_zone,
    sibling_macro_actions,
    survive_rank,
    wang_edge_pair_from_relative,
)
from nest_graph.place_zone_util import place_zone_token
from nest_graph.pack.cache_key import (
    cheap_lock_fingerprint,
    cheap_pack_cache_key,
    macro_motif_id,
)
from nest_graph.pack.cheap import (
    pack_cache_overlap_ok,
    peek_isolation_telem,
    proven_compose_row,
    with_isolated_pack_cache,
)
from nest_graph.pack.ctx import PathProbeResult
from nest_graph.pack.execute import (
    mcts_rule_ids,
    motif_tip_for_path_probe,
    path_accept_contact_upsert,
    record_mcts_expand,
)
from nest_graph.pack.slave_pack import _motif_inject_patterns
from nest_graph.propose.telem import record_overlap_reject


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
    graph_to_nest_hollow: bool = False,
) -> tuple[bool, int, int]:
    """D1/G1/P1/P: run/shrink macro_increase_path from place_cohort_ready + Motif tip.

    Returns ``(run, beam, max_depth)``. P1: when ready=0 and tip is not Motif but
    ``parent_free_hint`` (large_void), shrink-run with beam/4 (not skip). Motif tip
    with ready=0 still shrinks beam/2. G1: ready but macros idle → mild shrink.
    P/T: when ``path_carved_streak≥2`` and hold/plateau, further thrift — skip when
    ready∧pick_empty; else shrink beyond P1 (Motif tips thrifted too once streak honest).
    P0: graph_to_nest + hold∨plateau + carved≥2 → hard (1, 1) after idle thrift
    (caps path_refine/Attach flood; not Motif tip /4).
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
    # P/T: idle carve streak thrift (one gate; Motif included once streak is honest).
    idle_thrift = (
        int(path_carved_streak) >= 2
        and (bool(incumbent_hold) or bool(on_plateau))
    )
    if idle_thrift:
        # Letter C: never full-skip — ready∧pick_empty skip left late path_ms≈0
        # with path_ran still set from prior iters; always shrink-run instead.
        b = max(1, int(b) // 2)
        d = max(1, int(d) - 1)
    # P0: hollow MIS stall — hard-cap path packs + Attach validate beam.
    if (
        bool(graph_to_nest_hollow)
        and (bool(incumbent_hold) or bool(on_plateau))
        and int(path_carved_streak) >= 2
    ):
        return True, 1, 1
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


def _agent_preset_ids(agent: Any) -> list[int]:
    """Letter B: path generate_macros must see densify preset_ids (not default [0])."""
    raw = getattr(agent, "preset_ids", None) if agent is not None else None
    if not raw:
        return [0]
    return [int(x) for x in raw]


def _sibling_actions(
    agent: Any,
    blocked_action: Any,
    remaining_gids: tuple[int, ...],
    *,
    rule_ids: tuple[int, ...],
    snapshot: BoardSnapshot | None,
) -> list[Any]:
    free_kind = (
        free_kind_from_any(getattr(snapshot, "free_kind", FreeKind.Empty))
        if snapshot is not None
        else FreeKind.Empty
    )
    void_fit_r = (
        float(getattr(snapshot, "void_fit_r", 0.0) or 0.0)
        if snapshot is not None
        else 0.0
    )
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
            _agent_preset_ids(agent),
            void_fit_r,
        )
    )


def _path_neighbors_ranked(
    runner: Any, macro_node_id: int, agent: Any, telem: dict | None = None
) -> list[Any]:
    """Structural MotifJoin neighbors via DecisionGraph.neighbors (Q378/Q386)."""
    dg = getattr(runner, "dg", None)
    if dg is None:
        return []
    realized = _realized_dict(agent)
    sc = realized.get("survive_by_motif") or {}
    mb = getattr(agent, "motif_base", None) if agent is not None else None
    if mb is None:
        mb = getattr(runner, "motif_base", None)
    try:
        ranked = rank_motif_join_neighbors(
            dg,
            int(macro_node_id),
            {int(k): int(v) for k, v in dict(sc).items()},
            mb,
        )
        # L2 binding returns (steps, wang_rank_flip_n).
        if isinstance(ranked, tuple) and len(ranked) == 2:
            steps, flips = ranked
            if telem is not None:
                telem["wang_rank_flip_n"] = max(
                    int(telem.get("wang_rank_flip_n", 0) or 0), int(flips)
                )
            return list(steps)
        return list(ranked)
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



def _record_motif_join_identity_census(
    runner: Any,
    telem: dict,
    *,
    join_nbrs: Sequence[Any],
) -> None:
    """L0: MotifJoin labeled/unique/stack/stale/prefer-gap census (telem only).

    Prefer ``dg.motifs()`` as SoT — ``path_nbr_join`` may only mirror
    ``path_step_join`` when ``neighbors()`` is empty at census time.
    """
    dg = getattr(runner, "dg", None)
    agent = getattr(runner, "agent", None)
    mb = getattr(agent, "motif_base", None) if agent is not None else None
    if mb is None:
        mb = getattr(runner, "motif_base", None)

    motifs: list[Any] = []
    if dg is not None:
        try:
            motifs_fn = getattr(dg, "motifs", None)
            motifs = list(motifs_fn() if callable(motifs_fn) else (motifs_fn or []))
        except Exception:
            motifs = []

    join_only = [
        n for n in join_nbrs if getattr(n, "kind", None) == PathKind.MotifJoin
    ]
    # Mid list: path Join nbrs first (probe SoT), else dg.motifs().
    mid_list: list[int] = [int(macro_motif_id(nbr)) for nbr in join_only]
    if not mid_list:
        for m in motifs:
            raw = getattr(m, "motif_id", -1)
            mid = -1 if raw is None else int(raw)
            mid_list.append(mid)

    labeled = 0
    mid_set: set[int] = set()
    for mid in mid_list:
        if mid >= 0:
            mid_set.add(mid)
        if mb is None or mid < 0:
            continue
        try:
            if mid >= int(mb.size()):
                continue
            rec = mb.at(mid)
            ea_raw = getattr(rec, "edge_a", -1)
            eb_raw = getattr(rec, "edge_b", -1)
            ea = int(ea_raw) if ea_raw is not None else -1
            eb = int(eb_raw) if eb_raw is not None else -1
            if ea >= 0 or eb >= 0:
                labeled += 1
        except Exception:
            continue
    telem["path_join_labeled_n"] = max(
        int(telem.get("path_join_labeled_n", 0) or 0), int(labeled)
    )
    telem["path_join_mid_unique_n"] = max(
        int(telem.get("path_join_mid_unique_n", 0) or 0), int(len(mid_set))
    )

    # Stack: unordered pose pairs with >1 MotifJoin in dg.motifs().
    pair_counts: dict[tuple[int, int], int] = {}
    for m in motifs:
        a = int(getattr(m, "a", -1))
        b = int(getattr(m, "b", -1))
        if a < 0 or b < 0 or a == b:
            continue
        lo, hi = (a, b) if a < b else (b, a)
        pair_counts[(lo, hi)] = int(pair_counts.get((lo, hi), 0)) + 1
    stack_n = sum(1 for c in pair_counts.values() if int(c) > 1)
    telem["join_stack_n"] = max(int(telem.get("join_stack_n", 0) or 0), int(stack_n))

    # Stale: MotifRecord gid pair ≠ join pose gids.
    stale_n = 0
    poses = None
    if dg is not None:
        poses_attr = getattr(dg, "poses", None)
        try:
            poses = poses_attr() if callable(poses_attr) else poses_attr
        except Exception:
            poses = None
    pose_gids = getattr(poses, "group_id", None) if poses is not None else None
    if mb is not None and pose_gids is not None:
        for m in motifs:
            raw = getattr(m, "motif_id", -1)
            mid = -1 if raw is None else int(raw)
            if mid < 0:
                continue
            try:
                if mid >= int(mb.size()):
                    continue
                a = int(getattr(m, "a", -1))
                b = int(getattr(m, "b", -1))
                if a < 0 or b < 0 or a >= len(pose_gids) or b >= len(pose_gids):
                    continue
                ga, gb = int(pose_gids[a]), int(pose_gids[b])
                rec = mb.at(mid)
                ra_raw = getattr(rec, "gid_a", -1)
                rb_raw = getattr(rec, "gid_b", -1)
                ra = -1 if ra_raw is None else int(ra_raw)
                rb = -1 if rb_raw is None else int(rb_raw)
                if ra < 0 or rb < 0:
                    continue
                if {ra, rb} != {ga, gb}:
                    stale_n += 1
            except Exception:
                continue
    telem["stale_join_mid_n"] = max(
        int(telem.get("stale_join_mid_n", 0) or 0), int(stale_n)
    )

    # Prefer gap: path upsert fired but prefer_mid still tip-only (no path mid).
    upsert_n = int(telem.get("path_contact_upserts", 0) or 0)
    path_prefer = telem.get("path_upsert_prefer_mid", None)
    path_prefer_i = -1 if path_prefer is None else int(path_prefer)
    tip_prefer = telem.get("prefer_motif_id", None)
    if tip_prefer is None and isinstance(getattr(agent, "telem", None), dict):
        tip_prefer = agent.telem.get("prefer_motif_id")
    tip_i = -1 if tip_prefer is None else int(tip_prefer)
    gap = 1 if upsert_n > 0 and path_prefer_i < 0 else 0
    telem["path_upsert_prefer_gap"] = max(
        int(telem.get("path_upsert_prefer_gap", 0) or 0), int(gap)
    )
    if tip_i >= 0:
        telem["prefer_motif_id"] = int(tip_i)

    # Survive-count ties among join mids (id-ascending would decide).
    survive = {}
    if agent is not None:
        survive = _realized_dict(agent).get("survive_by_motif") or {}
    counts = [
        0 if mid < 0 else int(survive.get(mid, 0) or 0) for mid in mid_list
    ]
    tie_n = 0
    for i in range(len(counts)):
        for j in range(i + 1, len(counts)):
            if counts[i] == counts[j]:
                tie_n += 1
    telem["wang_rank_tie_n"] = max(
        int(telem.get("wang_rank_tie_n", 0) or 0), int(tie_n)
    )


def record_path_edge_census(
    runner: Any,
    macro_node_id: int,
    telem: dict,
    *,
    motif_locked: Sequence | None = None,
    beam_cap: int = 0,
) -> None:
    """E0/E2: PathKind neighbor census + pose hop-2 (telem only; no path mutate).

    P: ``beam_cap`` limits Join/Attach conflict pairs (same order of magnitude as
    path probe beam); full neighbor counts still recorded for telem.
    """
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
    # P: cap conflict scan to probe beam (reuse ranked Join slice when available).
    cap = int(beam_cap) if int(beam_cap) > 0 else 0
    if cap > 0 and len(join_nbrs) > cap:
        ranked = _path_neighbors_ranked(
            runner, int(macro_node_id), getattr(runner, "agent", None), telem
        )
        if ranked:
            ranked_ids = {int(getattr(n, "id", -1)) for n in ranked[:cap]}
            capped = [
                n for n in join_nbrs if int(getattr(n, "id", -1)) in ranked_ids
            ]
            if capped:
                join_nbrs_cut = capped[:cap]
            else:
                join_nbrs_cut = join_nbrs[:cap]
        else:
            join_nbrs_cut = join_nbrs[:cap]
    else:
        join_nbrs_cut = join_nbrs
    pose_n = len(
        _pose_validate_hops(
            dg, join_nbrs_cut, cap=max(2 * max(len(join_nbrs_cut), 1), 1)
        )
    )
    telem["path_nbr_macro"] = max(int(telem.get("path_nbr_macro", 0) or 0), n_macro)
    telem["path_nbr_join"] = max(int(telem.get("path_nbr_join", 0) or 0), n_join)
    telem["path_nbr_attach"] = max(int(telem.get("path_nbr_attach", 0) or 0), n_attach)
    telem["path_nbr_pose"] = max(int(telem.get("path_nbr_pose", 0) or 0), pose_n)
    # F0/P: pairwise Join/Attach conflicts on beam-capped list.
    cut_n = 0
    if hasattr(dg, "conflicts"):
        for i in range(len(join_nbrs_cut)):
            for j in range(i + 1, len(join_nbrs_cut)):
                try:
                    if bool(dg.conflicts(join_nbrs_cut[i], join_nbrs_cut[j])):
                        cut_n += 1
                except Exception:
                    continue
    telem["path_conflicts_cut"] = max(
        int(telem.get("path_conflicts_cut", 0) or 0), int(cut_n)
    )
    # E4: Join/Attach endpoints ∩ prior motif_locked pose indices (telem).
    lock_idxs = {
        int(i) for i in (motif_locked or ()) if isinstance(i, (int, float)) and int(i) >= 0
    }
    overlap = 0
    if lock_idxs:
        for nbr in join_nbrs:
            a, b = _join_endpoints(nbr)
            if a in lock_idxs or b in lock_idxs:
                overlap += 1
                # F2: census-side soft telem (same ov as lock_ov; no reward mutate).
                agent = getattr(runner, "agent", None)
                if agent is not None and isinstance(telem, dict):
                    telem["motif_locked"] = list(lock_idxs)
                    _validate_join_bonus(agent, nbr, 0.0, telem=telem)
    telem["path_lock_join_overlap"] = max(
        int(telem.get("path_lock_join_overlap", 0) or 0), int(overlap)
    )
    # Mirror MotifBase→Join attribution from bind_epoch when present on agent telem.
    base_j = 0
    agent = getattr(runner, "agent", None)
    src = getattr(agent, "telem", None) if agent is not None else None
    if isinstance(src, dict):
        base_j = int(src.get("motif_join_from_base", 0) or 0)
    telem["motif_join_from_base"] = max(
        int(telem.get("motif_join_from_base", 0) or 0), int(base_j)
    )
    # L0: MotifJoin identity / stack / Wang-tie census (telem only).
    # Prefer Join steps captured during macro_increase_path probe; fall back to
    # neighbors() / ranked beam (path_nbr_join may only mirror path_step_join).
    census_joins = [
        n for n in join_nbrs if getattr(n, "kind", None) == PathKind.MotifJoin
    ]
    seen = telem.get("path_join_steps_seen") or []
    if isinstance(seen, list) and seen:
        census_joins = list(seen) + census_joins
    if not census_joins:
        ranked = _path_neighbors_ranked(
            runner, int(macro_node_id), getattr(runner, "agent", None), telem
        )
        census_joins = [
            n for n in ranked if getattr(n, "kind", None) == PathKind.MotifJoin
        ]
    _record_motif_join_identity_census(runner, telem, join_nbrs=census_joins)


def _join_endpoints(step: Any) -> tuple[int, int]:
    """Pose endpoints of a MotifJoin/Attach PathNode (0 is valid — no `or -1`)."""
    ra = getattr(step, "a", -1)
    rb = getattr(step, "b", -1)
    return (
        int(ra) if ra is not None else -1,
        int(rb) if rb is not None else -1,
    )


def _validate_join_bonus(
    agent: Any, join_step: Any, baseline: float, telem: dict | None = None
) -> float:
    # motif_id=0 is valid — never ``x or -1``.
    mid = int(macro_motif_id(join_step))
    survive = float(
        _realized_dict(agent).get("survive_by_motif", {}).get(mid, 0) or 0
    )
    # W3: per-join MotifRecord fuel; soft residual adj when edges unlabeled.
    accept_scale = 1.0
    edge_fuel = 0.0
    odd_octant = False
    mb = getattr(agent, "motif_base", None)
    if mb is not None and mid >= 0:
        try:
            if int(mb.size()) > mid:
                rec = mb.at(mid)
                ac = float(getattr(rec, "accept_count", 0) or 0)
                accept_scale = 1.0 + 0.05 * min(ac, 8.0)
                ea_raw = getattr(rec, "edge_a", -1)
                eb_raw = getattr(rec, "edge_b", -1)
                ea = int(ea_raw) if ea_raw is not None else -1
                eb = int(eb_raw) if eb_raw is not None else -1
                if ea < 0 and eb < 0:
                    # Lazy fill from stored relative (same helper as upsert).
                    rel = getattr(rec, "relative", None)
                    if rel is not None:
                        ea, eb = wang_edge_pair_from_relative(rel)
                        ea = int(ea)
                        eb = int(eb)
                if ea >= 0 and eb >= 0:
                    edge_fuel += 1.0
                    odd_octant = (ea % 2 == 1) or (eb % 2 == 1)
                if bool(getattr(rec, "flush", False)):
                    edge_fuel += 1.0
        except Exception:
            accept_scale = 1.0
    src = telem if isinstance(telem, dict) else getattr(agent, "telem", None)
    wang_scale = accept_scale * (1.0 + 0.02 * min(edge_fuel, 8.0))
    # Residual soft adj when MotifRecord edges still idle after lazy fill.
    if edge_fuel <= 0.0 and isinstance(src, dict):
        try:
            adj = float(src.get("motif_adj_hits", 0) or 0)
        except Exception:
            adj = 0.0
        if adj > 0.0:
            wang_scale = max(wang_scale, 1.0 + 0.02 * min(adj, 8.0))
    # F2: soft-scale by Join/Attach endpoints ∩ sticky motif_locked.
    lock_idxs: set[int] = set()
    if isinstance(src, dict):
        raw_lock = src.get("motif_locked") or ()
        if isinstance(raw_lock, (list, tuple)):
            lock_idxs = {
                int(i) for i in raw_lock
                if isinstance(i, (int, float)) and int(i) >= 0
            }
    a, b = _join_endpoints(join_step)
    ov = (1 if a in lock_idxs else 0) + (1 if b in lock_idxs else 0)
    lock_scale = 1.0 + 0.06 * float(ov)
    # Soft odd-octant bump when lock overlap ≥1 (hypotenuse stair).
    if odd_octant and ov >= 1:
        wang_scale *= 1.03
    # Partial-endpoint soft when Wang fuel is idle (accept/edge flat).
    wang_idle = wang_scale <= 1.0 + 1e-9
    if wang_idle and ov == 1:
        lock_scale = max(lock_scale, 1.08)
    elif wang_idle and ov >= 2:
        lock_scale = max(lock_scale, 1.12)
    bonus = (0.02 + 0.01 * survive) * wang_scale * lock_scale
    # A: Attach parity — slight soft bump (same validate-only path).
    if getattr(join_step, "kind", None) == PathKind.Attach:
        bonus *= 0.85
    if isinstance(telem, dict):
        telem["path_wang_scale"] = float(
            max(float(telem.get("path_wang_scale", 0.0) or 0.0), wang_scale)
        )
        if ov > 0:
            telem["path_lock_join_soft"] = int(
                telem.get("path_lock_join_soft", 0) or 0
            ) + 1
            telem["path_lock_join_overlap"] = max(
                int(telem.get("path_lock_join_overlap", 0) or 0), int(ov)
            )
            # F4: winning Join motif_id for Motif tip force.
            if mid >= 0:
                prev_raw = telem.get("path_lock_join_motif_id", -1)
                prev = -1 if prev_raw is None else int(prev_raw)
                prev_ov = int(telem.get("path_lock_join_overlap", 0) or 0)
                if prev < 0 or ov >= prev_ov:
                    telem["path_lock_join_motif_id"] = int(mid)
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
        path_presets = _agent_preset_ids(agent)
        void_fit_anc = float(getattr(anc_snap, "void_fit_r", 0.0) or 0.0)
        if blocked is None:
            siblings = generate_macros(
                rem,
                rule_ids=tuple(int(r) for r in rule_ids),
                motif_base=agent.motif_base,
                prefer_motifs=True,
                warm_motif_ids=agent._warm_motif_ids(anc_snap),
                free_kind=free_kind_from_any(getattr(anc_snap, "free_kind", FreeKind.Empty)),
                motif_cohorts=_place_cohort_raw(agent),
                preset_ids=path_presets,
                void_fit_r=void_fit_anc,
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
                    free_kind=free_kind_from_any(
                        getattr(anc_snap, "free_kind", FreeKind.Empty)
                    ),
                    motif_cohorts=_place_cohort_raw(agent),
                    preset_ids=path_presets,
                    void_fit_r=void_fit_anc,
                )
                if getattr(a, "region", None) == MacroRegion.Motif
            ]
            if not motif_extra and force_motif_action is not None:
                motif_extra = [force_motif_action]
            siblings = list(motif_extra) + list(siblings)
        # H3: after survive_rank, soft AMAF tie-break (hole-expand credit → path beam).
        # P1: void_fit_r / phantom omitted on path — placeable + phantom demotes dumped
        # early med (Letter A r1). Soft conflict_p consumes on UCB expand only.
        amaf_scores: dict[int, float] = {}
        for i_alt, alt_sc in enumerate(siblings):
            try:
                key = agent._action_key(alt_sc)
                amaf_scores[i_alt] = float(
                    agent._amaf_pick_score(
                        key,
                        int(anc_id),
                        free_kind=FreeKind.Empty,
                    )
                )
            except Exception:
                amaf_scores[i_alt] = 0.0
        order = sorted(
            range(len(siblings)),
            key=lambda i: (
                -survive_rank(siblings[i], realized),
                -float(amaf_scores.get(i, 0.0)),
            ),
        )
        siblings = [siblings[i] for i in order]
        telem["path_amaf_tiebreak"] = int(telem.get("path_amaf_tiebreak", 0) or 0) + 1
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
        # Q386: MotifJoin/Attach neighbors validate-only (no pack).
        join_nbrs = _path_neighbors_ranked(runner, block_node, agent, telem)[
            : max(int(beam), 1)
        ]
        path_step_attach_d = 0
        dg = getattr(runner, "dg", None)
        for join_step in join_nbrs:
            path_extend_n += 1
            kind = getattr(join_step, "kind", None)
            if kind == PathKind.Attach:
                path_step_attach_d += 1
                path_type_hist[int(getattr(PathKind.Attach, "value", 2))] += 1
            else:
                path_step_join += 1
                path_type_hist[int(getattr(PathKind.MotifJoin, "value", 1))] += 1
            # L0: accumulate Join PathNodes seen during probe for identity census.
            if telem is not None and kind == PathKind.MotifJoin:
                seen = telem.setdefault("path_join_steps_seen", [])
                if isinstance(seen, list):
                    seen.append(join_step)
            # F2: lock_scale inside _validate_join_bonus; accept when realized.
            scored = _validate_join_bonus(
                agent, join_step, baseline_reward, telem=telem
            )
            if (
                dg is not None
                and hasattr(dg, "realized")
                and bool(dg.realized(join_step))
                and scored > best_reward
            ):
                best_reward = scored
                macro_chain_accept += 1
        telem["path_step_attach"] = int(
            telem.get("path_step_attach", 0) or 0
        ) + int(path_step_attach_d)
        for alt in siblings:
            attempts += 1
            path_extend_n += 1
            path_step_macro += 1
            path_type_hist[int(getattr(PathKind.Macro, "value", 0))] += 1
            zone = place_zone_token(region_to_zone(alt.region))
            try:
                path_patterns = (
                    _motif_inject_patterns(agent.motif_base, alt)
                    if agent.motif_base is not None
                    else []
                )
                child_snap = execute_fn(
                    anc_snap,
                    zone=zone,
                    action=alt,
                    patterns=path_patterns,
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
            # M4b: Macro execute + MotifJoin validate chain bonus (reuse join_nbrs).
            chain_bonus = 0.0
            for join_step in join_nbrs:
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
                free_k = free_kind_from_any(
                    getattr(child_snap, "free_kind", FreeKind.Empty)
                )
                if free_k == FreeKind.Empty:
                    free_k = free_kind_from_any(
                        getattr(anc_snap, "free_kind", FreeKind.Empty)
                    )
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
                if is_large_void(free_k) and motif_cov_ok:
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
    # F2 leaf soft telem is via record_path_edge_census (lock_ov SoT).
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
    N0: delegates to C++ ``path_accept_eligible_native`` beside ``path_reward_beats``.
    """
    if alt_action is None or path_accept_snap is None or parent_snap is None:
        return False, False, 0.0, 0.0
    is_motif = getattr(alt_action, "region", None) == MacroRegion.Motif
    eligible, cov_ok, base_cov, alt_cov = path_accept_eligible_native(
        parent_snap,
        path_accept_snap,
        float(base_reward),
        float(alt_reward),
        bool(path_overlap_ok),
        bool(is_motif),
        bool(relax_motif_void_fill),
        bool(fits_part),
        float(min_cov_delta),
        float(min_void_fill_delta),
    )
    return bool(eligible), bool(cov_ok), float(base_cov), float(alt_cov)


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
        "adopt": False,
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
        # S2: Motif soft never adopts (tip thrash); Void credit-only / hard carve do.
        out["adopt"] = (not bool(out["motif_soft"])) and (
            bool(out["tip_install"]) or not is_motif
        )
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
    # S2: cov_ok ∧ ¬motif_soft ∧ (tip_install ∨ region ≠ Motif).
    out["adopt"] = (not bool(out["motif_soft"])) and (
        bool(out["tip_install"]) or not is_motif
    )
    telem["path_tip_apply"] = int(telem.get("path_tip_apply", 0) or 0) + int(
        out["tip_install"]
    )
    return out


def note_path_accept_warm(runner: Any, snap: BoardSnapshot, action: Any) -> None:
    """Q390: remember_related + warm bump on path accept."""
    agent = getattr(runner, "agent", None)
    if agent is None:
        return
    agent.remember_related(snap, allow=True)
    mid = int(getattr(action, "motif_id", -1) or -1)
    if mid >= 0:
        agent.telem["related_warm"] = int(agent.telem.get("related_warm", 0) or 0) + 1
        agent.telem["nearest_warm"] = int(agent.telem.get("nearest_warm", 0) or 0) + 1


__all__ = [
    "ancestors",
    "macro_increase_path",
    "note_path_accept_warm",
    "path_accept_apply",
    "path_accept_eligible",
    "path_probe_budget",
    "record_path_edge_census",
    "run_outer_path_probe",
]


def run_outer_path_probe(
    *,
    mcts_runner: Any,
    nest_state: Any,
    history: Any,
    mcts_parent_id: int,
    mcts_action: Any,
    tip_action: Any,
    parent_snap: Any,
    parent_free_hint: bool,
    path_carved_streak: int,
    mcts_telem: dict,
    pack_cache: dict,
    cfg: Any,
    rule_sets: Any,
    part_bases: dict,
    part_areas: Sequence[float],
    p_sheet: Any,
    plateau: Any,
    nest_state_from_proven_compose: Callable[..., Any],
    history_from_nest_state: Callable[..., Any],
) -> PathProbeResult:
    """Telem reset + Motif tip + macro path probe + adopt (build_graph outer loop)."""
    _pack_cache = pack_cache
    # G0: per-iter path telem (skip never clears cumulative lie).
    mcts_telem["replay_from_ancestor_ms"] = 0.0
    mcts_telem["path_probe_skip"] = 0
    mcts_telem["motif_tip_pre_path"] = 0
    mcts_telem["path_ran"] = 0
    mcts_telem["path_cache_miss"] = 0
    mcts_telem["path_carved"] = 0
    mcts_telem["path_hard_carve"] = 0
    mcts_telem["path_geom_dropped"] = 0
    mcts_telem["path_compose_retained"] = 0
    mcts_telem["path_nest_adopt"] = 0
    mcts_telem["path_root_advance"] = 0
    mcts_telem["path_alt_none"] = 0
    mcts_telem["path_motif_discarded"] = 0
    mcts_telem["path_motif_tracked"] = 0
    mcts_telem["macro_path_cov_skip"] = 0
    mcts_telem["path_motif_force_tip"] = 0
    mcts_telem["motif_hold_tip"] = 0
    # P0: per-iter path wall + cch (pack_cache mid clear must not wipe).
    mcts_telem["path_compose_ms"] = 0.0
    mcts_telem["path_refine_ms"] = 0.0
    mcts_telem["cheap_cache_hit"] = 0
    mcts_telem["cheap_cache_lookup"] = 0
    _pack_cache["path_compose_ms"] = 0.0
    _pack_cache["path_refine_ms"] = 0.0
    # Snapshot multi-sim lookups before path (path adds on top).
    mcts_telem["cheap_cache_hit"] = int(_pack_cache.get("cache_hit_n", 0) or 0)
    mcts_telem["cheap_cache_lookup"] = int(
        _pack_cache.get("cache_lookup_n", 0) or 0
    )
    # P: per-iter census reset (was cumulative max → false nbr/cut growth).
    mcts_telem["path_nbr_macro"] = 0
    mcts_telem["path_nbr_join"] = 0
    mcts_telem["path_nbr_attach"] = 0
    mcts_telem["path_nbr_pose"] = 0
    mcts_telem["path_conflicts_cut"] = 0
    mcts_telem["path_lock_join_overlap"] = 0
    mcts_telem["path_lock_join_soft"] = 0
    # T: path sees prior-iter fits (set after compose); expose lag explicitly.
    mcts_telem["path_fits_lag"] = int(mcts_telem.get("last_fits_part", 1) or 0)
    # P: Motif fuel → Motif tip for path_probe_budget only (beam/2); outer tip
    # stays multi_sim/AMAF until ensure_motif_tip_for_fuel below.
    path_tip_for_budget = motif_tip_for_path_probe(
        tip_action,
        motif_base=mcts_runner.motif_base,
        remaining_gids=(
            parent_snap.remaining_gids if parent_snap is not None else None
        ),
        seed_n=int(mcts_telem.get("motif_base_seed_n", 0) or 0),
    )
    if (
        path_tip_for_budget is not None
        and getattr(path_tip_for_budget, "region", None) == MacroRegion.Motif
        and (
            tip_action is None
            or getattr(tip_action, "region", None) != MacroRegion.Motif
        )
    ):
        mcts_telem["motif_tip_pre_path"] = 1
    # I1b/D1/P1: probe on plateau or large_void; P1 shrink-runs (beam/4) when
    # place_cohort_ready=0 and tip is not Motif but free_hint. Motif tip → beam/2.
    # P: thrift when path_carved_streak≥2 on hold/plateau (skip ready∧pick_empty).
    # P0: graph_to_nest hollow → hard (1,1) after idle thrift (not Motif/4).
    graph_to_nest_hollow = bool(
        int(mcts_telem.get("last_graph_to_nest", 0) or 0)
    )
    run_path, path_beam, path_depth = path_probe_budget(
        on_plateau=bool(plateau.on_plateau),
        parent_free_hint=bool(parent_free_hint),
        agent=getattr(mcts_runner, "agent", None),
        tip_action=path_tip_for_budget,
        beam=int(getattr(cfg.propose, "macro_path_beam", 4) or 4),
        max_depth=int(getattr(cfg.propose, "macro_path_max_depth", 3) or 3),
        path_carved_streak=int(path_carved_streak),
        incumbent_hold=bool(
            int(mcts_telem.get("last_incumbent_hold", 0) or 0)
        ),
        pick_empty=int(
            mcts_telem.get("hybrid_pick_trials", 0) or 0
        )
        <= 0,
        graph_to_nest_hollow=graph_to_nest_hollow,
    )
    if not run_path:
        mcts_telem["path_probe_skip"] = 1
    # C: no pre-probe Motif soft tip_install (telem inflation without carve).
    if (
        run_path
        and mcts_runner.agent is not None
        and mcts_runner.execute_fn is not None
        and _pack_cache.get("ready")
    ):
        path = ancestors(mcts_runner, int(mcts_parent_id))
        mcts_telem["policy_path_len"] = int(len(path))
        mcts_telem["macro_swap_attempts"] = int(
            mcts_telem.get("macro_swap_attempts", 0) or 0
        )
        base_cov = float(getattr(parent_snap, "coverage", 0.0) or 0.0)
        realized = getattr(
            getattr(mcts_runner, "agent", None), "realized", None
        ) or {}
        base_r = leaf_reward(
            parent_snap,
            survive_motif_n=int(realized.get("survive_motif_n", 0) or 0),
            macro_survive_n=int(realized.get("macro_survive_n", 0) or 0),
            member_hits=int(realized.get("member_hits", 0) or 0),
            materialized_motif=int(
                realized.get("materialized_motif", 0) or 0
            ),
        )
        path_overlap_ok = False
        agent_pc = getattr(mcts_runner, "agent", None)
        # M: hold∧rim≥0.9∧large_void∧fuel∧late-density → Motif cov tip.
        # Fuel: join/compose/ready OR motif_hit/seed (late join often 0).
        rim_hi = float(mcts_telem.get("last_rim_progress", 0.0) or 0.0) >= 0.9
        fuel_ok = (
            int(mcts_telem.get("path_join_signal", 0) or 0) > 0
            or len(_pack_cache.get("motif_locked") or ()) >= 2
            or int(mcts_telem.get("motif_compose_accepted_size", 0) or 0)
            >= 2
            or bool(getattr(agent_pc, "place_cohort_ready", False))
            or int(mcts_telem.get("motif_hit", 0) or 0) > 0
            or int(mcts_telem.get("motif_base_seed_n", 0) or 0) > 0
        )
        motif_hold_tip = (
            int(mcts_telem.get("last_incumbent_hold", 0) or 0) > 0
            and rim_hi
            and bool(parent_free_hint)
            and fuel_ok
            and int(mcts_telem.get("last_nest_n", 0) or 0) >= 60
        )
        mcts_telem["motif_hold_tip"] = int(motif_hold_tip)
        mcts_telem["motif_hold_rim"] = int(rim_hi)
        mcts_telem["motif_hold_fuel"] = int(fuel_ok)
        force_motif = None
        if motif_hold_tip:
            # F4: path_lock_join_motif_id telem retained; forcing Join mid
            # tip dumped late med below F3 — Motif tip stays MotifBase.at(0).
            force_motif = motif_tip_for_path_probe(
                path_tip_for_budget,
                motif_base=mcts_runner.motif_base,
                remaining_gids=(
                    parent_snap.remaining_gids
                    if parent_snap is not None
                    else None
                ),
                seed_n=int(mcts_telem.get("motif_base_seed_n", 0) or 0),
            )
        # F2: sticky lock visible to _validate_join_bonus during path probe.
        mcts_telem["motif_locked"] = list(_pack_cache.get("motif_locked") or ())
        path_proven_compose: dict | None = None
        with with_isolated_pack_cache(_pack_cache):
            alt_action, alt_reward, path_accept_snap = macro_increase_path(
                mcts_runner,
                leaf_id=int(mcts_parent_id),
                baseline_reward=base_r,
                execute_fn=mcts_runner.execute_fn,
                rule_ids=mcts_rule_ids(rule_sets),
                beam=int(path_beam),
                max_depth=int(path_depth),
                telem=mcts_telem,
                overlap_ok_fn=lambda _s=None: pack_cache_overlap_ok(_pack_cache),
                prefer_motif_return=bool(motif_hold_tip),
                motif_hold_tip=bool(motif_hold_tip),
                force_motif_action=force_motif,
            )
            path_overlap_ok = bool(pack_cache_overlap_ok(_pack_cache))
            # S1: stash compose row before isolation restore (S2 adopt consumes).
            path_proven_compose = proven_compose_row(_pack_cache, alt_action)
            # P0: stamp path wall + cch before isolation restore (peek).
            _path_telem = peek_isolation_telem(_pack_cache)
            mcts_telem["path_compose_ms"] = float(
                _path_telem.get("path_compose_ms", 0.0) or 0.0
            )
            mcts_telem["path_refine_ms"] = float(
                _path_telem.get("path_refine_ms", 0.0) or 0.0
            )
            mcts_telem["cheap_cache_hit"] = int(
                _path_telem.get("cache_hit_n", 0) or 0
            )
            mcts_telem["cheap_cache_lookup"] = int(
                _path_telem.get("cache_lookup_n", 0) or 0
            )
        # E0/F0: census after probe (outside pack_cache isolation — telem only).
        mcts_telem["motif_join_from_base"] = max(
            int(mcts_telem.get("motif_join_from_base", 0) or 0),
            int(
                (_pack_cache.get("propose_stats") or {}).get(
                    "motif_join_from_base", 0
                )
                or 0
            ),
        )
        record_path_edge_census(
            mcts_runner,
            int(mcts_parent_id),
            mcts_telem,
            motif_locked=_pack_cache.get("motif_locked"),
            beam_cap=max(4, int(path_beam) * 2),
        )
        # M/T: hold tip armed but Motif execute sterile → telem only.
        # Do not bump path_carved / tip_apply / clear streak (defeats idle_thrift).
        if (
            motif_hold_tip
            and force_motif is not None
            and (
                alt_action is None
                or getattr(alt_action, "region", None) != MacroRegion.Motif
            )
        ):
            mcts_telem["path_motif_force_tip"] = 1
        if alt_action is None:
            mcts_telem["path_alt_none"] = int(
                mcts_telem.get("path_alt_none", 0) or 0
            ) + 1
        eligible, cov_ok, _base_cov, _alt_cov = path_accept_eligible(
            alt_action=alt_action,
            path_accept_snap=path_accept_snap,
            parent_snap=parent_snap,
            base_reward=base_r,
            alt_reward=float(alt_reward),
            path_overlap_ok=path_overlap_ok,
            relax_motif_void_fill=True,
            fits_part=bool(
                int(mcts_telem.get("last_fits_part", 1) or 0) > 0
            ),
        )
        if eligible:
            mcts_telem["macro_path_candidate"] = 1
            decision = path_accept_apply(
                mode="build_graph",
                cov_ok=cov_ok,
                path_overlap_ok=path_overlap_ok,
                alt_action=alt_action,
                enable_macro_path_replay=bool(
                    getattr(cfg.propose, "enable_macro_path_replay", False)
                ),
                mutate_motif_base_on_path=bool(
                    getattr(cfg.propose, "mutate_motif_base_on_path", False)
                ),
                telem=mcts_telem,
                free_kind=free_kind_token(
                    getattr(parent_snap, "free_kind", FreeKind.Empty)
                ),
            )
            # S1: retain compose row on accept (geom_drop after adopt attempt).
            path_accept_hit = (
                path_accept_snap is not None
                and not decision.get("overlap_skip")
                and (
                    bool(decision.get("credit"))
                    or bool(decision.get("tip_install"))
                )
            )
            if path_accept_hit and path_proven_compose is not None:
                mcts_telem["path_compose_retained"] = int(
                    mcts_telem.get("path_compose_retained", 0) or 0
                ) + 1
            if decision["overlap_skip"]:
                mcts_telem["macro_path_overlap_skip"] = int(
                    mcts_telem.get("macro_path_overlap_skip", 0) or 0
                ) + 1
                agent = getattr(mcts_runner, "agent", None)
                if (
                    agent is not None
                    and getattr(alt_action, "region", None)
                    != MacroRegion.Motif
                ):
                    agent.note_macro_miss(alt_action)
                cheap_by_key = _pack_cache.get("cheap_compose_by_key")
                if isinstance(cheap_by_key, dict):
                    _ppid = _pack_cache.get("pack_preset_id")
                    cheap_by_key.pop(
                        cheap_pack_cache_key(
                            place_zone_token(
                                region_to_zone(
                                    getattr(alt_action, "region", None)
                                )
                            ),
                            alt_action,
                            compose_sz=len(
                                _pack_cache.get("motif_locked") or ()
                            ),
                            cohort_sig=int(
                                _pack_cache.get("motif_cohort_sig", 0) or 0
                            ),
                            lock_fp=cheap_lock_fingerprint(
                                _pack_cache.get("motif_locked")
                            ),
                            pack_preset_id=(
                                int(_ppid) if _ppid is not None else None
                            ),
                        ),
                        None,
                    )
                record_overlap_reject(mcts_telem, stage="path")
            elif decision["cov_skip"]:
                mcts_telem["macro_path_cov_skip"] = 1
                if path_accept_hit:
                    mcts_telem["path_geom_dropped"] = int(
                        mcts_telem.get("path_geom_dropped", 0) or 0
                    ) + 1
                if decision["motif_soft"]:
                    mcts_telem["macro_path_motif_soft"] = 1
                if decision["credit"] and not decision.get("motif_soft"):
                    note_path_accept_warm(
                        mcts_runner, path_accept_snap, alt_action
                    )
                    mcts_telem["path_credit_n"] = int(
                        mcts_telem.get("path_credit_n", 0) or 0
                    ) + 1
                elif decision["credit"]:
                    mcts_telem["path_credit_n"] = int(
                        mcts_telem.get("path_credit_n", 0) or 0
                    ) + 1
            else:
                # R1: hard carve = tip_install ∧ ¬motif_soft (streak clear + idle).
                hard_carve = bool(decision["tip_install"]) and not bool(
                    decision.get("motif_soft")
                )
                adopted = False
                if (
                    bool(decision.get("adopt"))
                    and path_proven_compose is not None
                    and nest_state is not None
                    and path_accept_snap is not None
                    and alt_action is not None
                ):
                    adopted_ns = nest_state_from_proven_compose(
                        path_proven_compose,
                        seed_count=int(nest_state.seed_count or 0),
                        pack_cache=_pack_cache,
                    )
                    if adopted_ns is not None:
                        pre_parent = int(mcts_parent_id)
                        nest_state = adopted_ns
                        history = history_from_nest_state(nest_state)
                        ps_adopt = _pack_cache.get("propose_stats")
                        if not isinstance(ps_adopt, dict):
                            ps_adopt = {}
                        new_parent = record_mcts_expand(
                            mcts_runner,
                            parent_id=pre_parent,
                            action=alt_action,
                            child_snap=path_accept_snap,
                            nest_state=nest_state,
                            part_bases=part_bases,
                            part_areas=part_areas,
                            min_dist=float(
                                cfg.board_min_dist_for(p_sheet)
                            ),
                            t_expand0=time.perf_counter(),
                            mcts_telem=mcts_telem,
                            propose_stats=ps_adopt,
                            motif_min_compactness=float(
                                getattr(
                                    cfg.propose,
                                    "motif_min_compactness",
                                    0.35,
                                )
                                or 0.35
                            ),
                            motif_ttl=int(
                                getattr(
                                    cfg.propose,
                                    "accepted_pattern_ttl",
                                    4,
                                )
                                or 4
                            ),
                            motif_max_keep=int(
                                getattr(
                                    cfg.propose,
                                    "accepted_pattern_max",
                                    4,
                                )
                                or 4
                            ),
                            upsert_contacts=False,
                        )
                        mcts_parent_id = int(new_parent)
                        mcts_telem["path_nest_adopt"] = int(
                            mcts_telem.get("path_nest_adopt", 0) or 0
                        ) + 1
                        if int(new_parent) != pre_parent:
                            mcts_telem["path_root_advance"] = int(
                                mcts_telem.get("path_root_advance", 0)
                                or 0
                            ) + 1
                        adopted = True
                if path_accept_hit and not adopted:
                    mcts_telem["path_geom_dropped"] = int(
                        mcts_telem.get("path_geom_dropped", 0) or 0
                    ) + 1
                if decision["tip_install"]:
                    mcts_telem["macro_path_accept"] = int(
                        mcts_telem.get("macro_path_accept", 0) or 0
                    ) + 1
                    # S2: non-Motif adopt skips force_zone; Motif soft tip kept.
                    # P3 parked: always-install dumped late Δ; enable only when
                    # A0 shows path_nest_adopt vs dg_force_zone mismatch hurt.
                    if not adopted or bool(decision.get("motif_soft")):
                        mcts_action = alt_action
                    mcts_telem["path_carved"] = int(
                        mcts_telem.get("path_carved", 0) or 0
                    ) + 1
                    if hard_carve:
                        path_carved_streak = 0
                        mcts_telem["path_carved_streak"] = 0
                        mcts_telem["path_hard_carve"] = int(
                            mcts_telem.get("path_hard_carve", 0) or 0
                        ) + 1
                if decision["motif_soft"]:
                    mcts_telem["macro_path_motif_soft"] = 1
                if decision["credit"] and not decision.get("motif_soft"):
                    note_path_accept_warm(
                        mcts_runner, path_accept_snap, alt_action
                    )
                    mcts_telem["path_credit_n"] = int(
                        mcts_telem.get("path_credit_n", 0) or 0
                    ) + 1
                elif decision["credit"]:
                    mcts_telem["path_credit_n"] = int(
                        mcts_telem.get("path_credit_n", 0) or 0
                    ) + 1
                if decision["upsert"] and path_accept_snap is not None:
                    # J: Q389 upsert on path-proven snap (incl. Motif soft tip).
                    # Pre-probe soft tip never reaches this block.
                    path_accept_contact_upsert(
                        mcts_runner,
                        path_accept_snap,
                        part_bases=part_bases,
                        min_dist=float(
                            cfg.board_min_dist_for(p_sheet)
                        ),
                        motif_min_compactness=float(
                            getattr(
                                cfg.propose, "motif_min_compactness", 0.35
                            )
                            or 0.35
                        ),
                        motif_ttl=int(
                            getattr(cfg.propose, "accepted_pattern_ttl", 4)
                            or 4
                        ),
                        motif_max_keep=int(
                            getattr(cfg.propose, "accepted_pattern_max", 4)
                            or 4
                        ),
                        pack_cache=_pack_cache,
                        telem=mcts_telem,
                    )
        # T/R1: idle streak uses hard carve (soft tip must not freeze thrift).
        if int(mcts_telem.get("path_hard_carve", 0) or 0) <= 0:
            path_carved_streak += 1
        mcts_telem["path_carved_streak"] = int(path_carved_streak)
    elif run_path and not _pack_cache.get("ready"):
        mcts_telem["path_cache_miss"] = 1
        path_carved_streak += 1
        mcts_telem["path_carved_streak"] = int(path_carved_streak)
    return PathProbeResult(
        nest_state=nest_state,
        history=history,
        mcts_parent_id=int(mcts_parent_id),
        mcts_action=mcts_action,
        path_carved_streak=int(path_carved_streak),
    )

