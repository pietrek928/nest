"""Outer-loop browse tip selection (Q132) — policy parent, not geometry board."""

from typing import Any

from nest_graph.decision.types import BoardSnapshot

BROWSE_K = 3


def should_browse_tip(*, iter_idx: int, n_iters: int, is_last_leaf: bool) -> bool:
    if is_last_leaf:
        return True
    return (int(iter_idx) % int(BROWSE_K)) == (int(BROWSE_K) - 1)


def packed_gids_compatible(tip_snap: BoardSnapshot | None, nest_packed: set[int]) -> bool:
    """Tip snap gids must be subset of current NestState packed groups (policy-only jump)."""
    if tip_snap is None:
        return False
    tip = set(int(g) for g in (tip_snap.packed_gids or ()))
    if not tip:
        return True
    return tip.issubset(nest_packed)


def choose_browse_parent(
    runner: Any,
    *,
    spine_id: int,
    spine_snap: BoardSnapshot,
    nest_packed_gids: set[int],
    do_browse: bool,
    mcts_telem: dict,
) -> tuple[int, BoardSnapshot, bool]:
    """Return (parent_id, parent_snap, jumped).

    On browse: prefer select_leaf if expandable else deepest best_child.
    Incompatible tip packed set → spine fallback.
    """
    agent = runner.agent
    spine = int(spine_id)
    mcts_telem["spine_id"] = spine
    if agent is None or not do_browse:
        mcts_telem["browse_leaf_id"] = spine
        mcts_telem["browse_jump"] = 0
        mcts_telem["browse_incompatible"] = 0
        return spine, spine_snap, False

    leaf = int(agent.select_leaf())
    if not agent.may_expand_node(leaf):
        leaf = int(agent.deepest_best_child())
    tip_snap = runner.snapshot_at(leaf, spine_snap)
    if not packed_gids_compatible(tip_snap, nest_packed_gids):
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
