"""Outer-loop browse tip selection (Q132) — policy parent, not geometry board."""

from collections import Counter
from typing import Any, Sequence

from nest_graph.graph import BoardSnapshot
from nest_graph.utils import transform_row_key

BROWSE_K = 3


def should_browse_tip(*, iter_idx: int, n_iters: int, is_last_leaf: bool) -> bool:
    if is_last_leaf:
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
