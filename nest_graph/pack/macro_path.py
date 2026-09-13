"""Macro-tier augmenting path (Q296–Q300 / Q384–Q386)."""

import time
from typing import Any, Callable

from nest_graph.cohort_specs import generate_macros, motif_cohort_specs
from nest_graph.graph import (
    BoardSnapshot,
    PathKind,
    leaf_reward,
    path_reward_beats,
    rank_motif_join_neighbors,
    region_to_zone,
    sibling_macro_actions,
    survive_rank,
)


def ancestors(runner: Any, node_id: int) -> list[int]:
    """Parent-walk from root to node_id (Q304)."""
    return list(runner.arena.ancestors(int(node_id)))


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


def _validate_join_bonus(agent: Any, join_step: Any, baseline: float) -> float:
    mid = int(getattr(join_step, "motif_id", -1) or -1)
    bonus = 0.02 + 0.01 * float(
        _realized_dict(agent).get("survive_by_motif", {}).get(mid, 0) or 0
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
) -> tuple[Any | None, float, BoardSnapshot | None]:
    """Plateau sibling swap + cheap replay from ancestor snapshot (Q297/Q305/Q384).

    Returns ``(best_action, best_reward, best_snap)``. MotifJoin/Pose steps are
    validate-only (Q386); only Macro siblings call ``execute_fn``.
    """
    agent = getattr(runner, "agent", None)
    if agent is None or execute_fn is None:
        return None, float(baseline_reward), None
    t0 = time.perf_counter()
    telem = telem if telem is not None else agent.telem
    telem["macro_swap_attempts"] = int(telem.get("macro_swap_attempts", 0) or 0)
    path = ancestors(runner, int(leaf_id))
    telem["policy_path_len"] = int(len(path))
    best_action: Any | None = None
    best_reward = float(baseline_reward)
    best_snap: BoardSnapshot | None = None
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
        siblings.sort(key=lambda a: -survive_rank(a, realized))
        siblings = siblings[: max(int(beam), 1)]
        join_nbrs = _path_neighbors_ranked(runner, block_node, agent)[:beam]
        # Q386: MotifJoin neighbors validate-only (no pack).
        for join_step in join_nbrs:
            path_extend_n += 1
            path_step_join += 1
            path_type_hist[int(getattr(PathKind.MotifJoin, "value", 1))] += 1
            dg = getattr(runner, "dg", None)
            if dg is not None and hasattr(dg, "realized") and dg.realized(join_step):
                scored = _validate_join_bonus(agent, join_step, baseline_reward)
                if scored > best_reward:
                    best_reward = scored
                    macro_chain_accept += 1
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
    prev_hist = list(telem.get("path_type_hist") or [0, 0, 0, 0])
    prev_hist = (prev_hist + [0, 0, 0, 0])[:4]
    telem["path_type_hist"] = [
        int(prev_hist[i]) + int(path_type_hist[i]) for i in range(4)
    ]
    telem["replay_from_ancestor_ms"] = float(
        telem.get("replay_from_ancestor_ms", 0.0) or 0.0
    ) + (time.perf_counter() - t0) * 1000.0
    return best_action, best_reward, best_snap


__all__ = ["ancestors", "macro_increase_path"]
