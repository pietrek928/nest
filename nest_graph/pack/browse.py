"""Outer-loop browse tip selection (Q132) — policy parent, not geometry board."""

from collections import Counter
from typing import Any, Sequence

from nest_graph.free_kind_util import is_large_void
from nest_graph.graph import BoardSnapshot
from nest_graph.pack.execute import run_mcts_multi_sim
from nest_graph.utils import transform_row_key

BROWSE_K = 3


def should_browse_tip(
    *,
    iter_idx: int,
    n_iters: int,
    is_last_leaf: bool,
    large_void: bool = False,
    incumbent_hold: bool = False,
) -> bool:
    if is_last_leaf:
        return True
    # E: hold + large_void every iter (not only BROWSE_K).
    if bool(large_void) and bool(incumbent_hold):
        return True
    return (int(iter_idx) % int(BROWSE_K)) == (int(BROWSE_K) - 1)


def packed_gids_compatible(
    tip_snap: BoardSnapshot | None,
    nest_packed: set[int],
    nest_tf_keys: Sequence[tuple[float, float, float]] | None = None,
) -> bool:
    """Tip gids ⊆ nest packed groups; when nest_tf_keys set, Se2 multiset ⊆ too (S4)."""
    if tip_snap is None:
        return False
    tip = set(int(g) for g in (tip_snap.packed_gids or ()))
    if not tip:
        return True
    if not tip.issubset(nest_packed):
        return False
    if nest_tf_keys is None:
        return True
    tip_tfs = list(getattr(tip_snap, "packed_transforms", None) or ())
    if not tip_tfs:
        return True
    tip_keys = Counter(transform_row_key(t) for t in tip_tfs)
    nest_keys = Counter(nest_tf_keys)
    for k, n in tip_keys.items():
        if int(nest_keys.get(k, 0)) < int(n):
            return False
    return True


def choose_browse_parent(
    runner: Any,
    *,
    spine_id: int,
    spine_snap: BoardSnapshot,
    nest_packed_gids: set[int],
    do_browse: bool,
    mcts_telem: dict,
    nest_tf_keys: Sequence[tuple[float, float, float]] | None = None,
) -> tuple[int, BoardSnapshot, bool]:
    """Return (parent_id, parent_snap, jumped).

    On browse: prefer select_leaf if expandable else deepest best_child.
    Incompatible tip packed set → spine fallback.
    """
    agent = runner.agent
    spine = int(spine_id)
    mcts_telem["spine_id"] = spine
    mcts_telem.setdefault("browse_tf_mismatch", 0)
    if agent is None or not do_browse:
        mcts_telem["browse_leaf_id"] = spine
        mcts_telem["browse_jump"] = 0
        mcts_telem["browse_incompatible"] = 0
        return spine, spine_snap, False

    leaf = int(agent.select_leaf())
    if not agent.may_expand_node(leaf):
        leaf = int(agent.deepest_best_child())
    tip_snap = runner.snapshot_at(leaf, spine_snap)
    # S4: Se2 multiset telem only — hard-reject stays gid subset (late Δ safe).
    gid_ok = packed_gids_compatible(tip_snap, nest_packed_gids, nest_tf_keys=None)
    tf_ok = packed_gids_compatible(
        tip_snap, nest_packed_gids, nest_tf_keys=nest_tf_keys
    )
    if gid_ok and not tf_ok:
        mcts_telem["browse_tf_mismatch"] = int(
            mcts_telem.get("browse_tf_mismatch", 0) or 0
        ) + 1
    if not gid_ok:
        mcts_telem["browse_leaf_id"] = spine
        mcts_telem["browse_jump"] = 0
        mcts_telem["browse_incompatible"] = 1
        return spine, spine_snap, False

    jumped = int(leaf) != spine
    mcts_telem["browse_leaf_id"] = int(leaf)
    mcts_telem["browse_jump"] = int(jumped)
    mcts_telem["browse_incompatible"] = 0
    if jumped:
        agent.telem["browse_jump"] = int(agent.telem.get("browse_jump", 0)) + 1
    parent_snap = tip_snap if tip_snap is not None else spine_snap
    return int(leaf), parent_snap, jumped


def hold_browse_tip(
    *,
    parent_free_hint: bool,
    spine_snap: Any,
    plateau_on: bool,
    mcts_telem: dict,
) -> bool:
    """Inline hold_browse predicate (build_graph multi-sim tip unlock)."""
    return (
        (
            bool(parent_free_hint)
            or is_large_void(spine_snap)
        )
        and (
            bool(plateau_on)
            or (
                int(mcts_telem.get("last_incumbent_hold", 0) or 0) > 0
                and float(mcts_telem.get("last_rim_progress", 0.0) or 0.0) >= 0.9
            )
        )
    )


def install_iter_tip(
    *,
    mcts_runner: Any,
    spine_id: int,
    spine_snap: BoardSnapshot,
    nest_packed: set[int],
    nest_tf_keys: Sequence[tuple[float, float, float]],
    do_browse: bool,
    hold_browse: bool,
    pack_cache: dict,
    mcts_telem: dict,
    mcts_n_sims: int,
    is_last_leaf: bool,
    rule_ids: tuple[int, ...],
    sync_cohorts_fn,
) -> tuple[int, BoardSnapshot, Any | None]:
    """Browse parent + optional multi-sim tip install. Returns (parent_id, snap, tip_action)."""
    browse_parent_id, parent_snap, _jumped = choose_browse_parent(
        mcts_runner,
        spine_id=spine_id,
        spine_snap=spine_snap,
        nest_packed_gids=nest_packed,
        do_browse=do_browse,
        mcts_telem=mcts_telem,
        nest_tf_keys=nest_tf_keys,
    )
    tip_action = None
    _prev_ps = pack_cache.get("propose_stats")
    sync_cohorts_fn(
        mcts_runner,
        _prev_ps if isinstance(_prev_ps, dict) else None,
    )
    if isinstance(_prev_ps, dict):
        pack_cache["motif_cohort_sig"] = int(
            _prev_ps.get("motif_cohort_sig", 0) or 0
        )
    if pack_cache.get("ready") and not is_last_leaf and int(mcts_n_sims) > 0:
        tip_action, tip_leaf = run_mcts_multi_sim(
            mcts_runner,
            spine_snap,
            n_sims=int(mcts_n_sims),
            parent_id=int(spine_id),
            execute_fn=mcts_runner.execute_fn,
            mcts_telem=mcts_telem,
            rule_ids=rule_ids,
        )
        cheap_t = pack_cache.get("last_execute_telem") or {}
        for k in ("uh_ran", "compose_ran", "refine_ran", "cluster_copy", "cache_hit"):
            if k in cheap_t:
                mcts_telem[k] = cheap_t[k]
        if (do_browse or hold_browse) and tip_leaf is not None:
            tip_snap = mcts_runner.snapshot_at(int(tip_leaf), missing_ok=True)
            if tip_snap is not None and nest_tf_keys:
                gid_ok = packed_gids_compatible(
                    tip_snap, nest_packed, nest_tf_keys=None
                )
                tf_ok = packed_gids_compatible(
                    tip_snap, nest_packed, nest_tf_keys=nest_tf_keys
                )
                if gid_ok and not tf_ok:
                    mcts_telem["browse_tf_mismatch"] = int(
                        mcts_telem.get("browse_tf_mismatch", 0) or 0
                    ) + 1
            if tip_snap is not None and packed_gids_compatible(
                tip_snap, nest_packed, nest_tf_keys=nest_tf_keys
            ):
                if int(tip_leaf) != int(spine_id):
                    browse_parent_id = int(tip_leaf)
                    parent_snap = tip_snap
                    mcts_telem["browse_leaf_id"] = int(tip_leaf)
                    mcts_telem["browse_jump"] = 1
                    agent = getattr(mcts_runner, "agent", None)
                    if agent is not None:
                        agent.telem["browse_jump"] = int(
                            agent.telem.get("browse_jump", 0)
                        ) + 1
                    if hold_browse and not do_browse:
                        mcts_telem["browse_hold_tip"] = 1
    return int(browse_parent_id), parent_snap, tip_action

