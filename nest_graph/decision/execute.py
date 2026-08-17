"""One-iter execute helpers for Macro-MCTS (snapshot + expand bookkeep)."""

import time
from dataclasses import dataclass
from typing import Any, Callable, Sequence

from nest_graph.decision.mcts import leaf_reward
from nest_graph.decision.slave_pack import upsert_from_contacts
from nest_graph.decision.types import BoardSnapshot
from nest_graph.propose.context import prep_free_space, void_ratio_threshold
from nest_graph.propose.heavy_polish import (
    freeze_improve_rules,
    run_improve_rules_rounds,
    should_freeze_improve_rules,
)


def prep_selection_freeze(
    sel_iter,
    *,
    freeze_cheap_expand: bool,
    on_plateau: bool,
    plateau_streak: int,
    flat_iters: int,
    enable_incumbent_loop: bool,
):
    """One freeze gate for cheap expand / plateau; returns (sel_iter, reason)."""
    freeze, reason = should_freeze_improve_rules(
        freeze_cheap_expand=freeze_cheap_expand,
        on_plateau=on_plateau,
        plateau_streak=plateau_streak,
        flat_iters=flat_iters,
        enable_incumbent_loop=enable_incumbent_loop,
    )
    return freeze_improve_rules(sel_iter, freeze=freeze), reason


@dataclass(slots=True)
class SelectionFreePrep:
    free_result: Any
    packed_geoms: list
    packed_group_id: list[int] | None
    packed_transform: list | None
    packed_shapely: list
    void_thr: float
    mean_part: float

    @property
    def free_info(self):
        """Analysis view (unwraps FreeSpaceSnapshot when snapshot=True)."""
        r = self.free_result
        return getattr(r, "analysis", r)

    @property
    def free_snap(self):
        """Full snapshot when snapshot=True; else None."""
        r = self.free_result
        return r if hasattr(r, "analysis") else None


def schedule_prep_selection_free(
    *,
    phase: str,
    sheet,
    part_areas: Sequence[float],
    min_dist: float,
    cfg_propose,
    packed_shapely: Sequence | None = None,
    pack_geoms: Sequence | None = None,
    packed_group_id: Sequence[int] | None = None,
    packed_transform: Sequence | None = None,
    prior: SelectionFreePrep | None = None,
    selection_changed: bool = True,
    run_uh: bool = False,
    nest_state=None,
) -> SelectionFreePrep | None:
    """Hybrid free-prep schedule (G3): Uh when needed; reuse mid when safe.

    phase: ``uh`` | ``mid`` | ``post``.
    """
    if phase == "uh" and not run_uh:
        return None
    if (
        phase == "post"
        and prior is not None
        and not selection_changed
        and prior.free_info is not None
        and prior.free_snap is not None
    ):
        return prior
    snap = phase == "post"
    return prep_selection_free(
        sheet=sheet,
        part_areas=part_areas,
        min_dist=min_dist,
        cfg_propose=cfg_propose,
        nest_state=nest_state,
        packed_shapely=packed_shapely,
        pack_geoms=pack_geoms,
        packed_group_id=packed_group_id,
        packed_transform=packed_transform,
        snapshot=snap,
    )


def prep_selection_free(
    *,
    sheet,
    part_areas: Sequence[float],
    min_dist: float,
    cfg_propose,
    nest_state=None,
    packed_shapely: Sequence | None = None,
    pack_geoms: Sequence | None = None,
    packed_group_id: Sequence[int] | None = None,
    packed_transform: Sequence | None = None,
    snapshot: bool = False,
) -> SelectionFreePrep:
    """One free/board prep for mid-pack, post-DFS, and post-rim (Ua)."""
    mean_part = float(sum(part_areas) / len(part_areas)) if part_areas else 1.0
    void_thr = void_ratio_threshold(cfg_propose)
    geoms: list = list(pack_geoms) if pack_geoms is not None else []
    shapely_list: list = list(packed_shapely) if packed_shapely is not None else []
    gids: list[int] | None = (
        [int(g) for g in packed_group_id] if packed_group_id is not None else None
    )
    tfs: list | None = list(packed_transform) if packed_transform is not None else None
    if nest_state is not None and nest_state.selected_indices:
        if not shapely_list:
            shapely_list = [
                nest_state.polys[i] for i in nest_state.selected_indices
            ]
        if not geoms:
            native = nest_state.native_geoms
            geoms = [
                native[i] for i in nest_state.selected_indices if i < len(native)
            ]
        if gids is None or tfs is None:
            seed_n = int(nest_state.seed_count or 0)
            gids = []
            tfs = []
            for i in nest_state.selected_indices:
                if int(i) < seed_n:
                    continue
                if int(i) >= len(nest_state.group_id) or int(i) >= len(
                    nest_state.transform
                ):
                    continue
                gids.append(int(nest_state.group_id[i]))
                tfs.append(nest_state.transform[i])
    free_result = prep_free_space(
        sheet,
        shapely_list,
        mean_part,
        min_dist,
        void_ratio_threshold=void_thr,
        pack_geoms=geoms or None,
        snapshot=snapshot,
    )
    return SelectionFreePrep(
        free_result=free_result,
        packed_geoms=geoms,
        packed_group_id=gids,
        packed_transform=tfs,
        packed_shapely=shapely_list,
        void_thr=void_thr,
        mean_part=mean_part,
    )


def board_snapshot_from_selection(
    *,
    selected_polys: Sequence[int],
    group_id: Sequence[int],
    transform: Sequence,
    ngroups: int,
    coverage_pct: float,
    propose_stats: dict,
    mcts_action: Any | None,
    mcts_telem: dict,
    arena_node_id: int = 0,
) -> BoardSnapshot:
    """Thin BoardSnapshot ledger from a nest selection (no polygon blobs)."""
    packed_gids = tuple(
        int(group_id[i]) for i in selected_polys if int(i) < len(group_id)
    )
    packed_tf = tuple(
        tuple(float(x) for x in transform[i][:3])
        for i in selected_polys
        if int(i) < len(transform)
    )
    packed_set = set(packed_gids)
    leak = propose_stats.get("void_leak") if isinstance(propose_stats.get("void_leak"), dict) else {}
    free_kind = str(
        leak.get("free_kind")
        or propose_stats.get("post_rim_free_kind")
        or propose_stats.get("free_kind")
        or ""
    )
    # Dg: honest fills — rim from rim_progress; void from free ratio (not outline_cov).
    rim_fill = float(propose_stats.get("rim_progress", leak.get("rim_progress", 0.0)) or 0.0)
    max_void_ratio = float(leak.get("max_void_ratio", 0.0) or 0.0)
    # Map void ratio (>threshold ≈ large) to a 0..1 "still-open" fill proxy: high ratio → low fill.
    void_fill = float(max(0.0, min(1.0, 1.0 - (max_void_ratio / 10.0))))
    if free_kind == "large_void" and max_void_ratio <= 0.0:
        void_fill = 0.0
    elif free_kind and free_kind != "large_void" and max_void_ratio <= 0.0:
        void_fill = float(propose_stats.get("outline_cov", 0.0) or 0.0)
    return BoardSnapshot(
        packed_gids=packed_gids,
        packed_transforms=packed_tf,
        remaining_gids=tuple(g for g in range(int(ngroups)) if g not in packed_set),
        coverage=float(coverage_pct) / 100.0,
        arena_node_id=int(arena_node_id),
        kiss_pairs=int(propose_stats.get("kiss_pairs", 0) or 0),
        mean_compactness=float(propose_stats.get("mean_compactness", 0.0) or 0.0),
        rim_fill=rim_fill,
        void_fill=void_fill,
        free_kind=free_kind,
        motif_ids_used=(
            (int(mcts_action.motif_id),)
            if mcts_action is not None and int(mcts_action.motif_id) >= 0
            else ()
        ),
        telem=dict(mcts_telem),
    )


def record_mcts_expand(
    runner: Any,
    *,
    parent_id: int,
    action: Any,
    child_snap: BoardSnapshot,
    nest_state: Any | None,
    part_bases: dict,
    part_areas: Sequence[float],
    min_dist: float,
    t_expand0: float,
    mcts_telem: dict,
    propose_stats: dict,
    motif_min_compactness: float = 0.35,
    motif_ttl: int = 4,
    motif_max_keep: int = 0,
) -> int:
    """Expand/backprop + ContactGRG MotifBase upsert (Q93); returns new parent id."""
    del part_areas  # areas come from contact solids
    reward = leaf_reward(child_snap)
    agent = runner.agent
    if agent is None:
        return int(parent_id)
    if agent.may_expand_node(parent_id):
        child_id = agent.expand(parent_id, action, reward)
        mcts_telem["pw_expand"] = int(mcts_telem["pw_expand"]) + 1
    else:
        child_id = parent_id
        agent.backprop(parent_id, reward)
    child_snap.arena_node_id = int(child_id)
    runner.snapshots[int(child_id)] = child_snap
    # Q138: positive-only related warm (skip hollow-miss snaps).
    hollow = bool(propose_stats.get("hollow_miss", False))
    agent.remember_related(child_snap, allow=not hollow)

    packed_gids = child_snap.packed_gids
    packed_tf = child_snap.packed_transforms
    if len(packed_gids) >= 2 and len(packed_tf) >= 2 and part_bases:
        geoms = []
        gids_ok = []
        tfs_ok = []
        for gid, tf in zip(packed_gids, packed_tf):
            base = part_bases.get(int(gid))
            if base is None:
                continue
            try:
                geoms.append(base.apply_transform(float(tf[0]), float(tf[1]), float(tf[2])))
                gids_ok.append(int(gid))
                tfs_ok.append(
                    (float(tf[0]), float(tf[1]), float(tf[2]))
                )
            except Exception:
                continue
        if len(geoms) >= 2:
            upsert_from_contacts(
                runner.motif_base,
                geoms,
                gids_ok,
                tfs_ok,
                gap=float(min_dist),
                min_compactness=float(motif_min_compactness),
                ttl=int(motif_ttl),
                max_keep=int(motif_max_keep),
                telem=mcts_telem,
            )
    mcts_telem["expand_ms"] = float(mcts_telem.get("expand_ms", 0.0)) + (
        time.perf_counter() - t_expand0
    ) * 1000.0
    propose_stats["mcts"] = dict(mcts_telem)
    propose_stats["mcts"]["arena_size"] = int(runner.arena.size())
    propose_stats["mcts"]["motif_library"] = int(runner.motif_base.size())
    propose_stats["mcts"].update(
        {
            k: agent.telem.get(k)
            for k in ("amaf_hits", "related_warm", "from_shapely_count")
            if k in agent.telem
        }
    )
    return int(child_id)


def make_execute_fn(
    pack_fn: Callable[..., BoardSnapshot],
) -> Callable[..., BoardSnapshot]:
    """Wrap a pack body as MacroMctsRunner / cheap_expand_slave execute_fn."""

    def execute_fn(parent: BoardSnapshot, *, zone=None, action=None, patterns=None):
        return pack_fn(parent, zone=zone, action=action, patterns=patterns or [])

    return execute_fn


def run_mcts_multi_sim(
    runner: Any,
    parent_snap: BoardSnapshot,
    *,
    n_sims: int,
    parent_id: int,
    execute_fn: Callable[..., BoardSnapshot] | None,
    mcts_telem: dict,
) -> tuple[Any | None, int]:
    """Q107: K cheap expands; return (tip MacroAction, tip leaf id).

    Populates arena/AMAF so the following outer ``pick_expand_action`` is informed.
    Tip action is from the final ``select_leaf`` (UCB), not a leftover last expand.
    """
    from nest_graph.decision.slave_pack import cheap_expand_slave

    agent = runner.agent
    tip_leaf = int(parent_id)
    tip_action = None
    if agent is None or int(n_sims) <= 0 or agent.expand_frozen:
        mcts_telem["multi_sim"] = 0
        return None, tip_leaf
    for _ in range(max(int(n_sims), 1)):
        leaf = int(agent.select_leaf())
        tip_leaf = leaf
        leaf_snap = runner.snapshots.get(leaf) or parent_snap
        if not leaf_snap.remaining_gids:
            agent.backprop(leaf, float(leaf_snap.coverage))
            continue
        action = agent.pick_expand_action(
            leaf_snap.remaining_gids,
            rule_ids=(0,),
            parent_id=leaf,
            snapshot=leaf_snap,
        )
        if action is None:
            break
        tip_action = action
        if not agent.may_expand_node(leaf):
            agent.backprop(leaf, float(leaf_snap.coverage))
            continue
        result = cheap_expand_slave(
            leaf_snap,
            action,
            motif_base=runner.motif_base,
            execute_fn=execute_fn,
            telem=agent.telem,
        )
        child = agent.expand(leaf, action, result.reward)
        result.snapshot.arena_node_id = int(child)
        runner.snapshots[int(child)] = result.snapshot
        mcts_telem["pw_expand"] = int(mcts_telem.get("pw_expand", 0)) + 1
        tip_leaf = int(child)
        tip_action = action
    # Prefer deepest best_child tip when expandable leaf unavailable
    if tip_action is None:
        tip_leaf = int(agent.deepest_best_child())
        tip_snap = runner.snapshots.get(tip_leaf) or parent_snap
        if tip_snap.remaining_gids:
            tip_action = agent.pick_expand_action(
                tip_snap.remaining_gids,
                rule_ids=(0,),
                parent_id=tip_leaf,
                snapshot=tip_snap,
            )
    mcts_telem["multi_sim"] = int(n_sims)
    mcts_telem["expand_ms"] = float(agent.telem.get("expand_ms", 0.0) or 0.0)
    mcts_telem["tip_leaf"] = int(tip_leaf)
    return tip_action, int(tip_leaf)


def run_pack_stages(
    *,
    rim_only: bool = False,
    heavy: bool = False,
    run_improve_fn: Callable[..., Any] | None = None,
    rim_fn: Callable[..., Any] | None = None,
    compose_fn: Callable[..., Any] | None = None,
    refine_fn: Callable[..., Any] | None = None,
    post_pack_fn: Callable[..., Any] | None = None,
    uh_void_fn: Callable[..., Any] | None = None,
) -> dict:
    """One pack body for loop + execute_fn (Ub).

    ``rim_only``: kiss nest (+ optional Uh). ``heavy``: last-leaf post_pack/3b.
    Refine runs whenever ``refine_fn`` is set (mid or last budget).
    """
    telem = {
        "execute_wired": 1,
        "rim_only": int(rim_only),
        "mcts_heavy": int(heavy),
        "improve_ran": 0,
        "rim_ran": 0,
        "compose_ran": 0,
        "uh_ran": 0,
        "refine_ran": 0,
        "post_pack_ran": 0,
    }
    if run_improve_fn is not None:
        run_improve_fn()
        telem["improve_ran"] = 1
    if rim_only:
        if rim_fn is not None:
            rim_fn()
            telem["rim_ran"] = 1
        if uh_void_fn is not None:
            uh_void_fn()
            telem["uh_ran"] = 1
        return telem
    if compose_fn is not None:
        compose_fn()
        telem["compose_ran"] = 1
    if refine_fn is not None:
        refine_fn()
        telem["refine_ran"] = 1
    if heavy and post_pack_fn is not None:
        post_pack_fn()
        telem["post_pack_ran"] = 1
    return telem


def run_pack_body(
    *,
    do_heavy_polish: bool | None = None,
    run_post_pack: bool | None = None,
    run_improve_fn: Callable[..., Any],
    compose_fn: Callable[..., Any] | None = None,
    refine_fn: Callable[..., Any] | None = None,
    post_pack_fn: Callable[..., Any] | None = None,
) -> dict:
    """Backward-compatible wrapper → ``run_pack_stages`` (Ub)."""
    heavy = bool(run_post_pack) if run_post_pack is not None else bool(do_heavy_polish)
    return run_pack_stages(
        rim_only=False,
        heavy=heavy,
        run_improve_fn=run_improve_fn,
        compose_fn=compose_fn,
        refine_fn=refine_fn,
        post_pack_fn=post_pack_fn,
    )
