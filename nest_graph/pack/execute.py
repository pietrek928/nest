"""One-iter execute helpers for Macro-MCTS (snapshot + expand bookkeep)."""

import time
from dataclasses import dataclass
from typing import Any, Callable, Sequence

from nest_graph.free_kind_util import (
    coalesce_free_kind,
    free_kind_from_any,
    is_large_void,
    void_placeable,
)
from nest_graph.graph import BoardSnapshot, FreeKind, MacroAction, MacroRegion, leaf_reward
from nest_graph.pack.cache_key import macro_motif_id
from nest_graph.pack.slave_pack import cheap_expand_slave, upsert_from_contacts
from nest_graph.propose.context import prep_free_space, void_ratio_threshold
from nest_graph.propose.pattern_archive import note_motif_ref_anchors
from nest_graph.propose.heavy_polish import (
    freeze_improve_rules,
    should_freeze_improve_rules,
)
from nest_graph.propose.void_topology import iterative_multi_poles


def note_mcts_free_kind(agent: Any, free_kind: Any) -> None:
    """H2: soft-γ AMAF when BoardSnapshot.free_kind changes (one-gate)."""
    if agent is None:
        return
    kind = free_kind_from_any(free_kind)
    if kind == FreeKind.Empty:
        return
    note = getattr(agent, "note_free_kind", None)
    if note is not None:
        note(kind)


def apply_ir_rank(
    agent: Any,
    reward: float,
    *,
    kappa: float = 0.05,
    propose_stats: dict | None = None,
    mcts_telem: dict | None = None,
) -> float:
    """GE3: soft κ·(rank−0.5) vs related_snaps_; returns adjusted reward."""
    ir_rank = 0.5
    ir_rank_n = 0
    out = float(reward)
    try:
        rank_fn = getattr(agent, "rank_related_reward", None)
        if callable(rank_fn):
            _rr = rank_fn(float(reward))
            ir_rank = float(_rr[0])
            ir_rank_n = int(_rr[1])
    except Exception:
        ir_rank = 0.5
        ir_rank_n = 0
    if ir_rank_n > 0:
        out = float(reward) + float(kappa) * (float(ir_rank) - 0.5)
    if isinstance(propose_stats, dict):
        propose_stats["ir_rank"] = float(ir_rank)
        propose_stats["ir_rank_n"] = int(ir_rank_n)
        if isinstance(propose_stats.get("void_leak"), dict):
            propose_stats["void_leak"]["ir_rank"] = float(ir_rank)
            propose_stats["void_leak"]["ir_rank_n"] = int(ir_rank_n)
    if isinstance(mcts_telem, dict):
        mcts_telem["ir_rank"] = float(ir_rank)
        mcts_telem["ir_rank_n"] = int(ir_rank_n)
    return float(out)


def repair_bp_delta_addend(
    *,
    propose_stats: dict,
    mcts_telem: dict,
    board_area: float = 0.0,
    outer_reward: float | None = None,
) -> float:
    """R1/GE4: capped IR addend into the one expand reward (no second visit).

    Prefer gainful 3a/3b ``darea/board`` (else ``0.01·dcount``). When hole gain
    is absent, GE4 folds ``clamp(outer−last_cheap, −0.02, +0.05)`` if both
    ``outer_reward`` and ``last_cheap_reward`` are present.
    """
    propose_stats["repair_bp_delta"] = 0.0
    mcts_telem["repair_bp_delta"] = 0.0
    repair_bp_delta = 0.0
    if int(propose_stats.get("hole_expand_gain", 0) or 0) > 0:
        darea = float(propose_stats.get("block_hole_darea", 0.0) or 0.0)
        if darea <= 0.0:
            darea = float(propose_stats.get("block_cohort_darea", 0.0) or 0.0)
        dcount = int(propose_stats.get("block_hole_dcount", 0) or 0)
        if dcount <= 0:
            dcount = int(propose_stats.get("block_cohort_dcount", 0) or 0)
        board_a = float(board_area) if board_area else 0.0
        if darea > 1e-12 and board_a > 1e-12:
            raw_delta = float(darea) / board_a
        elif dcount > 0:
            raw_delta = 0.01 * float(dcount)
        else:
            raw_delta = 0.0
        repair_bp_delta = float(min(max(raw_delta, 0.0), 0.05))
    elif outer_reward is not None and "last_cheap_reward" in mcts_telem:
        # GE4: local-search advantage over cheap leaf (Q309 consumer).
        # Positive-only + cap 0.02 (degrade ladder); consume cheap so stale
        # multi_sim tips cannot boost every later outer expand.
        cheap = float(mcts_telem.pop("last_cheap_reward", 0.0) or 0.0)
        raw_delta = float(outer_reward) - cheap
        repair_bp_delta = float(min(max(raw_delta, 0.0), 0.02))
    if abs(repair_bp_delta) <= 1e-12:
        return 0.0
    propose_stats["repair_bp_delta"] = float(repair_bp_delta)
    mcts_telem["repair_bp_delta"] = float(repair_bp_delta)
    if isinstance(propose_stats.get("void_leak"), dict):
        propose_stats["void_leak"]["repair_bp_delta"] = float(repair_bp_delta)
    return float(repair_bp_delta)


def maybe_repair_delta_backprop(
    agent: Any,
    *,
    bp_node: int,
    child_snap: BoardSnapshot,
    action: Any,
    propose_stats: dict,
    mcts_telem: dict,
    board_area: float = 0.0,
) -> float:
    """Deprecated path: compute addend only (no second backprop visit)."""
    del agent, bp_node, child_snap, action
    return repair_bp_delta_addend(
        propose_stats=propose_stats,
        mcts_telem=mcts_telem,
        board_area=float(board_area),
    )


def multi_pole_n_from_free(
    free_poly: Any,
    *,
    min_dist: float,
    max_poles: int = 4,
    use_multi_pole: bool = True,
) -> int:
    """Count iterative spine poles for L2 AMAF floor (0 if disabled/empty)."""
    if (
        not use_multi_pole
        or free_poly is None
        or getattr(free_poly, "is_empty", True)
    ):
        return 0
    poles = iterative_multi_poles(
        free_poly,
        min_dist=float(min_dist),
        max_poles=max(1, int(max_poles)),
    )
    return int(len(poles))


def note_mcts_multi_pole(
    agent: Any,
    n: int | None = None,
    *,
    propose_stats: dict | None = None,
) -> None:
    """L2: arm multi-pole AMAF floor from spine/pocket pole count.

    Never clears an already-armed count when stats omit poles (stamp paths).
    """
    if agent is None:
        return
    count = int(n) if n is not None else 0
    if count <= 0 and isinstance(propose_stats, dict):
        count = int(propose_stats.get("multi_pole_count", 0) or 0)
        if count <= 0:
            dens = propose_stats.get("densify_stats") or {}
            if isinstance(dens, dict):
                count = int(dens.get("multi_pole_count", 0) or 0)
        if count <= 0:
            leak = propose_stats.get("void_leak")
            if isinstance(leak, dict):
                count = int(leak.get("multi_pole_count", 0) or 0)
        if count <= 0:
            count = int(
                (propose_stats.get("mcts") or {}).get("multi_pole_n", 0) or 0
            )
    if count <= 0:
        # Keep prior arm; do not zero on sparse propose_stats.
        return
    setter = getattr(agent, "set_multi_pole_n", None)
    if setter is not None:
        setter(int(count))


def void_fit_r_from_stats(propose_stats: dict | None, *, default: float = 0.0) -> float:
    """D1: 1.0 if fits_part else 0.0; missing → default (unknown = no void boost)."""
    if not isinstance(propose_stats, dict):
        return float(default)
    fp = propose_stats.get("fits_part")
    if fp is None:
        leak = propose_stats.get("void_leak")
        if isinstance(leak, dict):
            fp = leak.get("fits_part", leak.get("last_fits_part"))
    if fp is None:
        return float(default)
    return 1.0 if int(fp) else 0.0


def void_fit_ratio_from_stats(
    propose_stats: dict | None, *, default: float = 0.0
) -> float:
    """Continuous clamp(in_r / need_r) for conflict_p; falls back to 0/1 fit bit."""
    if not isinstance(propose_stats, dict):
        return float(default)
    in_r = propose_stats.get("fit_in_r")
    need_r = propose_stats.get("fit_need_r")
    leak = propose_stats.get("void_leak")
    if isinstance(leak, dict):
        if in_r is None:
            in_r = leak.get("fit_in_r")
        if need_r is None:
            need_r = leak.get("fit_need_r")
    if in_r is None or need_r is None:
        return float(void_fit_r_from_stats(propose_stats, default=default))
    need = float(need_r)
    inn = float(in_r)
    if need <= 1e-12:
        return float(void_fit_r_from_stats(propose_stats, default=default))
    return max(0.0, min(1.0, inn / need))


def region_for_free_kind(free_kind, *, void_fit_r: float = 0.0) -> MacroRegion:
    """D1: Void when placeable (LargeVoid or SwissCheese); else Sheet fallback."""
    if void_placeable(free_kind, void_fit_r):
        return MacroRegion.Void
    return MacroRegion.Sheet


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


def fit_shape_for_remaining(
    part_by_group: dict | None,
    remaining_gids: Sequence[int] | None = None,
) -> object | None:
    """Smallest-extent remaining catalog polygon for void fit (rem0 fallback)."""
    if not part_by_group:
        return None
    from nest_graph.propose.context import part_extents

    gids: list[int]
    if remaining_gids:
        gids = [int(g) for g in remaining_gids if int(g) in part_by_group]
    else:
        gids = [int(g) for g in part_by_group.keys()]
    if not gids:
        return None
    best_g = gids[0]
    best_ext = float("inf")
    for g in gids:
        poly = part_by_group.get(int(g))
        if poly is None or getattr(poly, "is_empty", False):
            continue
        _mn, mx = part_extents(poly)
        if float(mx) < best_ext:
            best_ext = float(mx)
            best_g = int(g)
    return part_by_group.get(best_g)


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
    part_by_group: dict | None = None,
    remaining_gids: Sequence[int] | None = None,
    fit_shape=None,
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
        part_by_group=part_by_group,
        remaining_gids=remaining_gids,
        fit_shape=fit_shape,
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
    part_by_group: dict | None = None,
    remaining_gids: Sequence[int] | None = None,
    fit_shape=None,
) -> SelectionFreePrep:
    """One free/board prep for mid-pack, post-DFS, and post-rim (Ua).

    Kind ratio uses mean ``part_areas``; placeability uses ``fit_shape`` (rem0 /
    smallest remaining extent) when provided.
    """
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
    shape = fit_shape
    if shape is None:
        shape = fit_shape_for_remaining(part_by_group, remaining_gids)
    free_result = prep_free_space(
        sheet,
        shapely_list,
        mean_part,
        min_dist,
        void_ratio_threshold=void_thr,
        pack_geoms=geoms or None,
        snapshot=snapshot,
        fit_shape=shape,
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
    # C: rem = unmet catalog group types (multi-copy densify SoT).
    # Prior `g not in packed_set` emptied rem once each type appeared → path_ms=0.
    selected_set = {int(i) for i in selected_polys}
    rem_gids = sorted(
        {
            int(group_id[i])
            for i in range(len(group_id))
            if i not in selected_set
        }
    )
    if not rem_gids and int(ngroups) > 0 and len(selected_set) < len(group_id):
        rem_gids = list(range(int(ngroups)))
    del mcts_telem  # Q131: telem stays on the Python mcts_telem dict
    leak_raw = propose_stats.get("void_leak")
    leak = leak_raw if isinstance(leak_raw, dict) else {}
    free_kind = coalesce_free_kind(
        leak.get("free_kind"),
        propose_stats.get("post_rim_free_kind"),
        propose_stats.get("free_kind"),
    )
    rim_fill = float(propose_stats.get("rim_progress", leak.get("rim_progress", 0.0)) or 0.0)
    # Prefer leak, then top-level propose_stats (snapshot often runs before leak hoist).
    max_void_ratio = float(
        leak.get(
            "max_void_ratio",
            propose_stats.get("max_void_ratio", 0.0),
        )
        or 0.0
    )
    # GE1: non-saturating void progress (ratio≳10 used to clamp void_fill→0).
    _r0 = 10.0
    void_fill = float(1.0 / (1.0 + (max_void_ratio / _r0))) if max_void_ratio > 0.0 else 1.0
    void_fill = float(max(0.0, min(1.0, void_fill)))
    if is_large_void(free_kind) and max_void_ratio <= 0.0:
        void_fill = 0.0
    elif free_kind != FreeKind.Empty and not is_large_void(free_kind) and max_void_ratio <= 0.0:
        void_fill = float(propose_stats.get("outline_cov", 0.0) or 0.0)
    fit_r = void_fit_r_from_stats(propose_stats)
    fit_ratio = void_fit_ratio_from_stats(propose_stats)
    propose_stats["void_fit_ratio"] = float(fit_ratio)
    propose_stats["void_fill"] = float(void_fill)
    if isinstance(propose_stats.get("void_leak"), dict):
        propose_stats["void_leak"]["void_fill"] = float(void_fill)
        propose_stats["void_leak"]["void_fit_ratio"] = float(fit_ratio)
    return BoardSnapshot(
        packed_gids=packed_gids,
        packed_transforms=packed_tf,
        remaining_gids=tuple(int(g) for g in rem_gids),
        coverage=float(coverage_pct) / 100.0,
        arena_node_id=int(arena_node_id),
        kiss_pairs=int(propose_stats.get("kiss_pairs", 0) or 0),
        mean_compactness=float(propose_stats.get("mean_compactness", 0.0) or 0.0),
        rim_fill=rim_fill,
        void_fill=void_fill,
        free_kind=free_kind,
        void_fit_r=float(fit_r),
        void_fit_ratio=float(fit_ratio),
        motif_ids_used=(
            (int(mcts_action.motif_id),)
            if mcts_action is not None and int(mcts_action.motif_id) >= 0
            else ()
        ),
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
    upsert_contacts: bool = True,
    rule_sets: Sequence | None = None,
    repair_bp_addend: float = 0.0,
) -> int:
    """Expand/backprop + ContactGRG MotifBase upsert (Q93); returns new parent id.

    ``upsert_contacts=False`` skips MotifBase upsert (S2 path adopt — Q389 already
    ran via ``_path_accept_contact_upsert``).
    ``repair_bp_addend`` folds gainful 3a/3b coverage Δ into this one expand reward
    (no second ``backprop`` visit).
    """
    del part_areas
    agent = runner.agent
    realized = (getattr(agent, "realized", None) or {}) if agent is not None else {}
    reward = float(
        leaf_reward(
            child_snap,
            rule_id=int(getattr(action, "rule_id", 0) or 0),
            member_hits=int(propose_stats.get("member_hits", 0) or 0),
            materialized_motif=int(propose_stats.get("materialized_motif", 0) or 0),
            survive_motif_n=int(realized.get("survive_motif_n", 0) or 0),
            macro_survive_n=int(realized.get("macro_survive_n", 0) or 0),
        )
    )
    addend = float(repair_bp_addend)
    if abs(addend) <= 1e-12:
        # GE4: cheap→outer when R1 hole path was idle (needs outer leaf).
        board_a = float(propose_stats.get("board_area", 0.0) or 0.0)
        if board_a <= 0.0:
            try:
                board_a = float(getattr(child_snap, "sheet_area", 0.0) or 0.0)
            except Exception:
                board_a = 0.0
        addend = float(
            repair_bp_delta_addend(
                propose_stats=propose_stats,
                mcts_telem=mcts_telem,
                board_area=float(board_a),
                outer_reward=float(reward),
            )
        )
    if abs(addend) > 1e-12:
        reward = float(reward) + addend
    if agent is None:
        return int(parent_id)
    reward = apply_ir_rank(
        agent, reward, propose_stats=propose_stats, mcts_telem=mcts_telem,
    )
    note_mcts_free_kind(agent, getattr(child_snap, "free_kind", FreeKind.Empty))
    if agent.may_expand_node(parent_id):
        child_id = agent.expand(parent_id, action, reward)
        mcts_telem["pw_expand"] = int(mcts_telem["pw_expand"]) + 1
    else:
        child_id = parent_id
        agent.backprop(parent_id, reward)
    child_snap.arena_node_id = int(child_id)
    runner.store_snapshot(int(child_id), child_snap)
    hollow = bool(propose_stats.get("hollow_miss", False))
    agent.remember_related(child_snap, allow=not hollow)

    packed_gids = child_snap.packed_gids
    packed_tf = child_snap.packed_transforms
    if (
        bool(upsert_contacts)
        and len(packed_gids) >= 2
        and len(packed_tf) >= 2
        and part_bases
    ):
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
            note_motif_ref_anchors(runner.motif_base, gids_ok, tfs_ok)
    mcts_telem["expand_ms"] = float(mcts_telem.get("expand_ms", 0.0)) + (
        time.perf_counter() - t_expand0
    ) * 1000.0
    act_rid = int(getattr(action, "rule_id", 0) or 0)
    region_i = int(getattr(action.region, "value", action.region))
    motif_id = macro_motif_id(action)
    preset_id = int(getattr(action, "preset_id", 0) or 0)
    # Aggregate AMAF visits across tip motif and motif=-1 so rid/preset
    # diversity telem is visible even when the tip Motif key is cold.
    for rid in range(max(act_rid + 1, 3)):
        visits = 0
        for mid in (motif_id, -1):
            visits = max(
                visits,
                int(agent.arena.amaf_visits(region_i, rid, mid, preset_id)),
                int(agent.arena.amaf_visits(region_i, rid, mid, 0)),
            )
        if visits > 0:
            mcts_telem[f"rule_id_amaf_{rid}"] = visits
            propose_stats[f"rule_id_amaf_{rid}"] = visits
    for pid in range(max(preset_id + 1, 3)):
        visits = 0
        for mid in (motif_id, -1):
            visits = max(
                visits,
                int(agent.arena.amaf_visits(region_i, act_rid, mid, pid)),
                int(agent.arena.amaf_visits(region_i, 0, mid, pid)),
            )
        if visits > 0:
            mcts_telem[f"preset_amaf_{pid}"] = visits
            propose_stats[f"preset_amaf_{pid}"] = visits
    # Sync mcts dict after rid/preset AMAF keys (assemble_void_leak reads mt).
    # K credits AMAF deltas next iter via credit_mut_slot_amaf_deltas (slot_scale).
    _ = rule_sets
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


def stamp_arena_amaf(
    runner: Any,
    *,
    selected_polys: Sequence[int],
    group_id: Sequence[int],
    transform: Sequence,
    ngroups: int,
    coverage_pct: float,
    propose_stats: dict,
    parent_id: int,
    action: Any | None = None,
    rule_ids: tuple[int, ...] = (0,),
) -> tuple[int, Any | None]:
    """Write DecisionArena AMAF from an outer pack; pick so ``amaf_hits`` can fire."""
    rule_ids = tuple(int(r) for r in (rule_ids or (0,)))
    agent = getattr(runner, "agent", None)
    if agent is None:
        return int(parent_id), action
    mcts_telem = dict(getattr(agent, "telem", None) or propose_stats.get("mcts") or {})
    mcts_telem.setdefault("pw_expand", 0)
    child_snap = board_snapshot_from_selection(
        selected_polys=selected_polys,
        group_id=group_id,
        transform=transform,
        ngroups=int(ngroups),
        coverage_pct=float(coverage_pct),
        propose_stats=propose_stats,
        mcts_action=action,
        mcts_telem=mcts_telem,
        arena_node_id=int(parent_id),
    )
    rem = tuple(int(g) for g in (child_snap.remaining_gids or ()))
    note_mcts_multi_pole(agent, propose_stats=propose_stats)
    if action is None and rem:
        action = agent.pick_expand_action(
            rem, rule_ids=rule_ids, parent_id=int(parent_id), snapshot=child_snap,
        )
    if action is None:
        action = MacroAction()
        free_kind = coalesce_free_kind(
            getattr(child_snap, "free_kind", None),
            propose_stats.get("free_kind"),
        )
        action.region = region_for_free_kind(
            free_kind,
            void_fit_r=float(getattr(child_snap, "void_fit_r", 0.0) or 0.0),
        )
        action.rule_id = 0
        action.motif_id = -1
    t0 = time.perf_counter()
    new_id = record_mcts_expand(
        runner,
        parent_id=int(parent_id),
        action=action,
        child_snap=child_snap,
        nest_state=None,
        part_bases={},
        part_areas=(),
        min_dist=0.0,
        t_expand0=t0,
        mcts_telem=mcts_telem,
        propose_stats=propose_stats,
    )
    parent_visits = max(int(agent.arena.visits(int(parent_id))), 1)
    agent._ucb(int(new_id), parent_visits)
    if rem:
        agent.pick_expand_action(
            rem, rule_ids=rule_ids, parent_id=int(new_id), snapshot=child_snap,
        )
    propose_stats["amaf_hits"] = int(agent.telem.get("amaf_hits", 0) or 0)
    propose_stats["amaf_miss"] = int(agent.telem.get("amaf_miss", 0) or 0)
    propose_stats["mcts"] = dict(mcts_telem)
    return int(new_id), action


def record_outer_iter_expand(
    runner: Any,
    *,
    parent_id: int,
    action: Any | None,
    selected_polys: Sequence[int],
    group_id: Sequence[int],
    transform: Sequence,
    ngroups: int,
    coverage_pct: float,
    propose_stats: dict,
    mcts_telem: dict,
    nest_state: Any | None = None,
    part_bases: dict | None = None,
    part_areas: Sequence[float] = (),
    min_dist: float = 0.0,
    t_expand0: float | None = None,
    motif_min_compactness: float = 0.35,
    motif_ttl: int = 0,
    motif_max_keep: int = 4,
    rule_ids: tuple[int, ...] = (0,),
    rule_sets: Sequence | None = None,
) -> tuple[int, BoardSnapshot | None]:
    """Outer-leaf expand: snapshot + record_mcts_expand + void_leak upsert telem (Q144)."""
    rule_ids = tuple(int(r) for r in (rule_ids or (0,)))
    agent = getattr(runner, "agent", None)
    if agent is None:
        return int(parent_id), None
    t0 = float(t_expand0) if t_expand0 is not None else time.perf_counter()
    mcts_telem.setdefault("pw_expand", 0)
    child_snap = board_snapshot_from_selection(
        selected_polys=selected_polys,
        group_id=group_id,
        transform=transform,
        ngroups=int(ngroups),
        coverage_pct=float(coverage_pct),
        propose_stats=propose_stats,
        mcts_action=action,
        mcts_telem=mcts_telem,
        arena_node_id=int(parent_id),
    )
    act = action
    if act is None:
        rem = tuple(int(g) for g in (child_snap.remaining_gids or ()))
        if rem:
            act = agent.pick_expand_action(
                rem, rule_ids=rule_ids, parent_id=int(parent_id), snapshot=child_snap,
            )
        if act is None:
            act = MacroAction()
            free_kind = coalesce_free_kind(
                getattr(child_snap, "free_kind", None),
                propose_stats.get("free_kind"),
            )
            act.region = region_for_free_kind(
                free_kind,
                void_fit_r=float(getattr(child_snap, "void_fit_r", 0.0) or 0.0),
            )
            act.rule_id = 0
            act.motif_id = -1
    # R1: fold capped 3b coverage Δ into the one expand reward (no second visit).
    board_a = 0.0
    try:
        board_a = float(getattr(child_snap, "sheet_area", 0.0) or 0.0)
    except Exception:
        board_a = 0.0
    if board_a <= 0.0:
        board_a = float(propose_stats.get("board_area", 0.0) or 0.0)
    repair_add = repair_bp_delta_addend(
        propose_stats=propose_stats,
        mcts_telem=mcts_telem,
        board_area=float(board_a),
    )
    new_id = record_mcts_expand(
        runner,
        parent_id=int(parent_id),
        action=act,
        child_snap=child_snap,
        nest_state=nest_state,
        part_bases=part_bases or {},
        part_areas=part_areas,
        min_dist=float(min_dist),
        t_expand0=t0,
        mcts_telem=mcts_telem,
        propose_stats=propose_stats,
        motif_min_compactness=float(motif_min_compactness),
        motif_ttl=int(motif_ttl),
        motif_max_keep=int(motif_max_keep),
        rule_sets=rule_sets,
        repair_bp_addend=float(repair_add),
    )
    parent_visits = max(int(agent.arena.visits(int(parent_id))), 1)
    agent._ucb(int(new_id), parent_visits)
    rem = tuple(int(g) for g in (child_snap.remaining_gids or ()))
    if rem:
        agent.pick_expand_action(
            rem, rule_ids=rule_ids, parent_id=int(new_id), snapshot=child_snap,
        )
    if isinstance(propose_stats.get("void_leak"), dict):
        propose_stats["void_leak"]["contact_grg_upserts"] = int(
            mcts_telem.get("contact_grg_upserts", 0) or 0
        )
        if int(mcts_telem.get("kiss_demote", 0) or 0) > 0:
            propose_stats["void_leak"]["kiss_demote"] = int(
                mcts_telem.get("kiss_demote", 0) or 0
            )
        if "repair_bp_delta" in propose_stats:
            propose_stats["void_leak"]["repair_bp_delta"] = float(
                propose_stats.get("repair_bp_delta", 0.0) or 0.0
            )
    propose_stats["amaf_hits"] = int(agent.telem.get("amaf_hits", 0) or 0)
    propose_stats["amaf_miss"] = int(agent.telem.get("amaf_miss", 0) or 0)
    return int(new_id), child_snap


def make_execute_fn(
    pack_fn: Callable[..., BoardSnapshot],
) -> Callable[..., BoardSnapshot]:
    """Wrap a pack body as MacroMctsRunner / cheap_expand_slave execute_fn."""

    def execute_fn(parent: BoardSnapshot, *, zone=None, action=None, patterns=None):
        return pack_fn(parent, zone=zone, action=action, patterns=patterns or [])

    return execute_fn


def mcts_rule_ids(rule_sets: Sequence, k: int = 3) -> tuple[int, ...]:
    """Q247: top-K rule_id indices for MCTS expand (baseline uses rule_id=0)."""
    n = len(rule_sets or ())
    if n <= 0:
        return (0,)
    return tuple(range(min(n, max(int(k), 1))))


def packed_shapely_skip_seeds(nest_state: Any | None) -> list:
    """Packed polys excluding seed indices (hand-built; pack_geoms=None path)."""
    packed: list = []
    if nest_state is None or not nest_state.selected_indices:
        return packed
    seed_n = int(nest_state.seed_count or 0)
    for i in nest_state.selected_indices:
        if int(i) < seed_n:
            continue
        if int(i) < len(nest_state.polys):
            packed.append(nest_state.polys[i])
    return packed


def seed_parent_free_kind(
    parent_snap: BoardSnapshot,
    *,
    cfg: Any,
    p_sheet: Any,
    part_areas: Sequence[float],
    nest_state: Any | None,
    part_by_group: dict | None = None,
):
    """P: seed ``free_kind`` before path probe (was after path — late path_ms starved)."""
    kind = free_kind_from_any(getattr(parent_snap, "free_kind", FreeKind.Empty))
    if kind != FreeKind.Empty:
        return kind
    try:
        md_fk = cfg.board_min_dist_for(p_sheet, first_pass=nest_state is None)
        packed_fk = packed_shapely_skip_seeds(nest_state)
        rem = getattr(parent_snap, "remaining_gids", None)
        fk_prep = prep_selection_free(
            sheet=p_sheet,
            part_areas=part_areas,
            min_dist=md_fk,
            cfg_propose=cfg.propose,
            packed_shapely=packed_fk,
            pack_geoms=None,
            part_by_group=part_by_group,
            remaining_gids=list(rem) if rem is not None else None,
        )
        kind = free_kind_from_any(getattr(fk_prep.free_info, "kind", FreeKind.Empty))
        parent_snap.free_kind = kind
    except Exception:
        kind = FreeKind.Empty
    return kind


def seed_parent_free_and_poles(
    parent_snap: BoardSnapshot | None,
    *,
    cfg: Any,
    p_sheet: Any,
    part_areas: Sequence[float],
    nest_state: Any | None,
    agent: Any,
    part_by_group: dict | None = None,
    mcts_telem: dict | None = None,
    seed_free: bool = True,
    seed_poles: bool = False,
):
    """Pre-path free_kind and/or post-path multi_pole (shared packed_fk skip-seeds).

    Pre-path: ``seed_free=True, seed_poles=False``. Post-path: both True — may
    re-seed Empty free_kind and always notes multi_pole when agent present.
    """
    if parent_snap is None:
        return FreeKind.Empty
    if seed_free and not seed_poles:
        kind = seed_parent_free_kind(
            parent_snap,
            cfg=cfg,
            p_sheet=p_sheet,
            part_areas=part_areas,
            nest_state=nest_state,
            part_by_group=part_by_group,
        )
        note_mcts_free_kind(agent, kind)
        return kind
    if not seed_poles or agent is None:
        return free_kind_from_any(getattr(parent_snap, "free_kind", FreeKind.Empty))
    try:
        md_fk = cfg.board_min_dist_for(p_sheet, first_pass=nest_state is None)
        use_mp = bool(getattr(cfg.propose, "use_multi_pole_void", True))
        max_mp = int(getattr(cfg.propose, "multi_pole_max_poles", 4) or 4)
        n_poles = int((mcts_telem or {}).get("multi_pole_count", 0) or 0)
        fk_now = free_kind_from_any(
            getattr(parent_snap, "free_kind", FreeKind.Empty)
        )
        if fk_now == FreeKind.Empty or n_poles < 2:
            packed_fk = packed_shapely_skip_seeds(nest_state)
            rem_fk = getattr(parent_snap, "remaining_gids", None)
            fk_prep = prep_selection_free(
                sheet=p_sheet,
                part_areas=part_areas,
                min_dist=md_fk,
                cfg_propose=cfg.propose,
                packed_shapely=packed_fk,
                pack_geoms=None,
                part_by_group=part_by_group,
                remaining_gids=list(rem_fk) if rem_fk is not None else None,
            )
            if seed_free and fk_now == FreeKind.Empty:
                parent_snap.free_kind = free_kind_from_any(
                    getattr(fk_prep.free_info, "kind", FreeKind.Empty)
                )
                note_mcts_free_kind(agent, parent_snap.free_kind)
                fk_now = free_kind_from_any(parent_snap.free_kind)
            n_poles = multi_pole_n_from_free(
                getattr(fk_prep.free_info, "target_poly", None),
                min_dist=float(md_fk),
                max_poles=max_mp,
                use_multi_pole=use_mp,
            )
        if n_poles < 2 and use_mp and is_large_void(fk_now):
            n_poles = max(2, max_mp)
        note_mcts_multi_pole(agent, n_poles)
        if mcts_telem is not None:
            mcts_telem["multi_pole_count"] = int(n_poles)
        return fk_now
    except Exception:
        note_mcts_multi_pole(agent, propose_stats=mcts_telem)
        return free_kind_from_any(getattr(parent_snap, "free_kind", FreeKind.Empty))


def path_accept_contact_upsert(
    runner: Any,
    snap: BoardSnapshot,
    *,
    part_bases: dict,
    min_dist: float,
    motif_min_compactness: float,
    motif_ttl: int,
    motif_max_keep: int,
    pack_cache: dict,
    telem: dict,
) -> int:
    """Q389: masked ContactGRG upsert on macro_path accept (not multi-sim)."""
    if runner is None or getattr(runner, "motif_base", None) is None:
        return 0
    packed_gids = tuple(getattr(snap, "packed_gids", ()) or ())
    packed_tf = tuple(getattr(snap, "packed_transforms", ()) or ())
    if len(packed_gids) < 2 or len(packed_tf) < 2 or not part_bases:
        return 0
    geoms = []
    gids_ok = []
    tfs_ok = []
    for gid, tf in zip(packed_gids, packed_tf):
        base = part_bases.get(int(gid))
        if base is None:
            continue
        try:
            geoms.append(
                base.apply_transform(float(tf[0]), float(tf[1]), float(tf[2]))
            )
            gids_ok.append(int(gid))
            tfs_ok.append((float(tf[0]), float(tf[1]), float(tf[2])))
        except Exception:
            continue
    if len(geoms) < 2:
        return 0
    # Mask gids: MotifJoin survivors ∪ (motif_locked ∩ selected).
    allow_gids: set[int] = set()
    realized = getattr(getattr(runner, "agent", None), "realized", None) or {}
    survive = realized.get("survive_by_motif") or {}
    mb = runner.motif_base
    for mid, cnt in (survive or {}).items():
        if int(cnt) <= 0 or int(mid) < 0 or int(mid) >= int(mb.size()):
            continue
        rec = mb.at(int(mid))
        allow_gids.add(int(rec.gid_a))
        allow_gids.add(int(rec.gid_b))
    locked = {
        int(i) for i in (pack_cache.get("motif_locked") or ()) if int(i) >= 0
    }
    sel = {int(i) for i in (pack_cache.get("compose_sel") or ())}
    gid_list = list(pack_cache.get("compose_group_id") or ())
    for i in locked & sel:
        if 0 <= int(i) < len(gid_list):
            allow_gids.add(int(gid_list[int(i)]))
    if not allow_gids:
        # Path-proven leaf: discover among accept-snap packed gids only.
        allow_gids = {int(g) for g in gids_ok}
    mask = [int(g) in allow_gids for g in gids_ok]
    if not any(mask):
        return 0
    # R3 conditional one-endpoint (cap-1 / flush) dual-min under open —
    # park; both-endpoints only (always-on any-endpoint also missed 0.592).
    n_up = upsert_from_contacts(
        runner.motif_base,
        geoms,
        gids_ok,
        tfs_ok,
        gap=float(min_dist),
        min_compactness=float(motif_min_compactness),
        ttl=int(motif_ttl),
        max_keep=int(motif_max_keep),
        telem=telem,
        selection_mask=mask,
        mask_any_endpoint=False,
    )
    note_motif_ref_anchors(runner.motif_base, gids_ok, tfs_ok)
    telem["path_contact_upserts"] = int(telem.get("path_contact_upserts", 0) or 0) + int(
        n_up
    )
    # L3: post-truncate mid for next-iter prefer_motif_id (tip stays first).
    post_raw = telem.get("last_upsert_mid", -1)
    post_mid = -1 if post_raw is None else int(post_raw)
    if n_up > 0 and post_mid >= 0:
        telem["path_upsert_prefer_mid"] = int(post_mid)
        telem["path_upsert_prefer_gap"] = 0
    elif n_up > 0:
        telem["path_upsert_prefer_gap"] = max(
            int(telem.get("path_upsert_prefer_gap", 0) or 0), 1
        )
    return int(n_up)


def motif_tip_for_path_probe(
    tip_action: Any,
    *,
    motif_base: Any,
    remaining_gids: Sequence[int] | None,
    seed_n: int,
    prefer_motif_id: int | None = None,
) -> Any:
    """P: Motif tip for ``path_probe_budget`` when MotifBase has fuel (beam/2).

    Does not mutate the outer expand tip — path budget only.
    F4: ``prefer_motif_id`` from lock-soft Join when set.
    """
    if (
        tip_action is not None
        and getattr(tip_action, "region", None) == MacroRegion.Motif
    ):
        return tip_action
    mb_n = 0
    try:
        if motif_base is not None:
            mb_n = int(motif_base.size())
    except Exception:
        mb_n = 0
    if mb_n <= 0:
        mb_n = int(seed_n or 0)
    if mb_n <= 0:
        return tip_action
    forced = MacroAction()
    forced.region = MacroRegion.Motif
    mid = 0
    if prefer_motif_id is not None and int(prefer_motif_id) >= 0:
        mid = int(prefer_motif_id)
        if mb_n > 0 and mid >= mb_n:
            mid = 0
    forced.motif_id = int(mid)
    rem = tuple(int(g) for g in (remaining_gids or ()))
    forced.part_gid = int(rem[0]) if rem else 0
    forced.rule_id = 0
    forced.preset_id = 0
    try:
        if motif_base is not None and int(motif_base.size()) > mid:
            rec = motif_base.at(mid)
            forced.part_gid = int(
                getattr(rec, "gid_a", forced.part_gid) or forced.part_gid
            )
    except Exception:
        pass
    return forced


def ensure_motif_tip_for_fuel(
    mcts_action: Any | None,
    *,
    motif_base: Any,
    remaining_gids: Sequence[int] | None,
    mcts_telem: dict,
    seed_n: int = 0,
    force: bool = False,
) -> Any | None:
    """D: credit Motif tip; optionally force Motif when MotifBase fuel (motif_id=0 valid).

    Default ``force=False``: only count natural Motif tips (UCB / tip-prefer).
    ``force=True`` is telem-only fallback when Motif never wins pick.
    """
    mb_n = 0
    try:
        if motif_base is not None:
            mb_n = int(motif_base.size())
    except Exception:
        mb_n = 0
    if mb_n <= 0:
        mb_n = int(seed_n or mcts_telem.get("motif_base_seed_n", 0) or 0)
    tip_is_motif = (
        mcts_action is not None
        and getattr(mcts_action, "region", None) == MacroRegion.Motif
        and macro_motif_id(mcts_action) >= 0
    )
    if force and mb_n > 0 and not tip_is_motif:
        forced = MacroAction()
        forced.region = MacroRegion.Motif
        forced.motif_id = 0
        rem = tuple(int(g) for g in (remaining_gids or ()))
        forced.part_gid = int(rem[0]) if rem else 0
        forced.rule_id = 0
        forced.preset_id = 0
        try:
            if motif_base is not None and int(motif_base.size()) > 0:
                rec = motif_base.at(0)
                forced.part_gid = int(
                    getattr(rec, "gid_a", forced.part_gid) or forced.part_gid
                )
        except Exception:
            pass
        mcts_action = forced
        mcts_telem["motif_tip_force"] = 1
        tip_is_motif = True
    if tip_is_motif:
        mcts_telem["motif_hit"] = int(mcts_telem.get("motif_hit", 0) or 0) + 1
    return mcts_action


def run_mcts_multi_sim(
    runner: Any,
    parent_snap: BoardSnapshot,
    *,
    n_sims: int,
    parent_id: int,
    execute_fn: Callable[..., BoardSnapshot] | None,
    mcts_telem: dict,
    rule_ids: tuple[int, ...] = (0,),
) -> tuple[Any | None, int]:
    """Q107: K cheap expands; return (tip MacroAction, tip leaf id)."""
    agent = runner.agent
    tip_leaf = int(parent_id)
    tip_action = None
    motif_tip = None
    motif_tip_reward = -1.0e30
    rule_ids = tuple(int(r) for r in (rule_ids or (0,)))
    if agent is None or int(n_sims) <= 0 or agent.expand_frozen:
        mcts_telem["multi_sim"] = 0
        return None, tip_leaf
    for _ in range(max(int(n_sims), 1)):
        leaf = int(agent.select_leaf())
        tip_leaf = leaf
        leaf_snap = runner.snapshot_at(leaf, parent_snap)
        if not leaf_snap.has_remaining:
            agent.backprop(leaf, float(leaf_snap.coverage))
            continue
        action = agent.pick_expand_action(
            leaf_snap.remaining_gids,
            rule_ids=rule_ids,
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
            realized=dict(getattr(agent, "realized", None) or {}),
        )
        cheap_raw = float(result.reward)
        cheap_r = apply_ir_rank(agent, cheap_raw, mcts_telem=mcts_telem)
        child = agent.expand(leaf, action, cheap_r)
        # GE4 / Q309: store pre-rank cheap so outer leaf_reward compares like-for-like.
        mcts_telem["last_cheap_reward"] = float(cheap_raw)
        result.snapshot.arena_node_id = int(child)
        runner.store_snapshot(int(child), result.snapshot)
        mcts_telem["pw_expand"] = int(mcts_telem.get("pw_expand", 0)) + 1
        tip_leaf = int(child)
        tip_action = action
        # D: track best Motif expand so tip is not always last Void sim.
        # motif_id=0 is valid — never gate with ``x or -1``.
        if (
            getattr(action, "region", None) == MacroRegion.Motif
            and macro_motif_id(action) >= 0
            and float(result.reward) >= float(motif_tip_reward)
        ):
            motif_tip = action
            motif_tip_reward = float(result.reward)
    mb_size = 0
    try:
        mb_size = int(runner.motif_base.size())
    except Exception:
        mb_size = 0
    if motif_tip is not None and mb_size > 0:
        tip_action = motif_tip
        mcts_telem["motif_tip_prefer"] = 1
    if tip_action is None:
        tip_leaf = int(agent.deepest_best_child())
        tip_snap = runner.snapshot_at(tip_leaf, parent_snap)
        if tip_snap.has_remaining:
            tip_action = agent.pick_expand_action(
                tip_snap.remaining_gids,
                rule_ids=rule_ids,
                parent_id=tip_leaf,
                snapshot=tip_snap,
            )
    mcts_telem["multi_sim"] = int(n_sims)
    mcts_telem["expand_ms"] = float(agent.telem.get("expand_ms", 0.0) or 0.0)
    mcts_telem["tip_leaf"] = int(tip_leaf)
    return tip_action, int(tip_leaf)


def stamp_pack_stage(stats: dict, **flags) -> dict:
    """Run pack stage work and stamp telem into stats (Ub/Q148)."""
    telem = {
        "execute_wired": 1,
        "rim_only": int(flags.get("rim_only", False)),
        "mcts_heavy": int(flags.get("heavy", False)),
        "improve_ran": 0,
        "rim_ran": 0,
        "compose_ran": 0,
        "uh_ran": 0,
        "refine_ran": 0,
        "post_pack_ran": 0,
    }
    run_improve_fn = flags.get("run_improve_fn")
    rim_fn = flags.get("rim_fn")
    compose_fn = flags.get("compose_fn")
    refine_fn = flags.get("refine_fn")
    post_pack_fn = flags.get("post_pack_fn")
    uh_void_fn = flags.get("uh_void_fn")
    rim_only = bool(flags.get("rim_only", False))
    heavy = bool(flags.get("heavy", False))

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
        stats.update(telem)
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
    stats.update(telem)
    return telem


def execute_pack(
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
    """Flags API for cheap / Uh / last pack."""
    stats: dict = {}
    return stamp_pack_stage(
        stats,
        rim_only=rim_only,
        heavy=heavy,
        run_improve_fn=run_improve_fn,
        rim_fn=rim_fn,
        compose_fn=compose_fn,
        refine_fn=refine_fn,
        post_pack_fn=post_pack_fn,
        uh_void_fn=uh_void_fn,
    )


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
    """Alias for execute_pack (backward compat)."""
    return execute_pack(
        rim_only=rim_only,
        heavy=heavy,
        run_improve_fn=run_improve_fn,
        rim_fn=rim_fn,
        compose_fn=compose_fn,
        refine_fn=refine_fn,
        post_pack_fn=post_pack_fn,
        uh_void_fn=uh_void_fn,
    )


__all__ = [
    "board_snapshot_from_selection",
    "ensure_motif_tip_for_fuel",
    "execute_pack",
    "fit_shape_for_remaining",
    "make_execute_fn",
    "mcts_rule_ids",
    "maybe_repair_delta_backprop",
    "motif_tip_for_path_probe",
    "path_accept_contact_upsert",
    "repair_bp_delta_addend",
    "prep_selection_free",
    "prep_selection_freeze",
    "record_mcts_expand",
    "record_outer_iter_expand",
    "run_mcts_multi_sim",
    "run_pack_stages",
    "packed_shapely_skip_seeds",
    "schedule_prep_selection_free",
    "seed_parent_free_and_poles",
    "seed_parent_free_kind",
    "stamp_arena_amaf",
    "stamp_pack_stage",
]
