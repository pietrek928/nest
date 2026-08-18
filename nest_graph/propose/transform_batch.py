"""Propose transform-batch assembly (moved from build_graph)."""

from typing import Sequence

import numpy as np
from shapely.geometry.base import BaseGeometry
from shapely.geometry import Polygon

from nest_graph.config import (
    BuildGraphConfig,
    ProposeConfig,
    cap_graph_valid_carry,
    dedupe_transforms,
    expand_structured_transforms,
    subsample_transforms_stratified,
)
from nest_graph.elem_graph import PlacementRuleSet
from nest_graph.geometry import Geometry
from nest_graph.propose.context import sheet_has_narrow_corridor
from nest_graph.propose.pipeline import (
    border_edge_transforms_for_group,
    collect_propose_batch_for_nest,
)
from nest_graph.propose.placements_selection_expand import (
    history_expand_arrays,
    selection_expand_arrays,
)
from nest_graph.propose.void_selection import transform_row_key


def rim_sat_proposer_updates(mcts_zone: str) -> dict:
    """Q145: rim-sat mute. ``use_side_pack`` always; ``use_history_expand`` only Rim/Sheet.

    Void/Motif map to ``void_seek`` (Q104). ``cluster_copy`` is never muted here.
    """
    updates: dict = {"use_side_pack": False}
    zone = str(mcts_zone or "")
    if zone in ("cluster_edge", "interior_pocket"):
        updates["use_history_expand"] = False
    return updates


def prune_transforms_vs_packed(
    transforms: np.ndarray,
    base: Geometry | None,
    packed: Sequence[Geometry] | None,
) -> np.ndarray:
    """Drop expand_rest rows that penetrate locked packed solids."""
    if (
        transforms is None
        or transforms.size == 0
        or base is None
        or not packed
    ):
        return transforms
    keep: list[np.ndarray] = []
    for row in np.asarray(transforms, dtype=np.float64).reshape(-1, 3):
        placed = base.apply_transform(row)
        if placed is None:
            continue
        if placed.intersects_any(list(packed)):
            continue
        keep.append(row)
    if not keep:
        return np.zeros((0, 3), dtype=np.float64)
    return np.asarray(keep, dtype=np.float64).reshape(-1, 3)


def project_angles_to_allowed(
    transforms: np.ndarray,
    allowed: Sequence[float],
) -> np.ndarray:
    """Snap angle column to nearest allowed grain angle (mod 2π)."""
    if transforms.size == 0 or not allowed:
        return transforms
    out = np.asarray(transforms, dtype=np.float64).copy()
    allowed_arr = np.asarray(allowed, dtype=np.float64)
    two_pi = 2.0 * np.pi
    angles = np.mod(out[:, 2], two_pi)
    allowed_mod = np.mod(allowed_arr, two_pi)
    diffs = angles[:, None] - allowed_mod[None, :]
    diffs = (diffs + np.pi) % two_pi - np.pi
    nearest = np.argmin(np.abs(diffs), axis=1)
    out[:, 2] = allowed_mod[nearest]
    return out


def allowed_for_gid(
    group_allowed_angles: Sequence[tuple[float, ...] | None] | tuple,
    gid: int,
) -> Sequence[float] | None:
    if not group_allowed_angles or gid < 0 or gid >= len(group_allowed_angles):
        return None
    return group_allowed_angles[gid]


def project_row_key(
    row,
    allowed: Sequence[float] | None,
) -> tuple[float, float, float]:
    arr = np.asarray(row, dtype=np.float64).reshape(-1)
    if arr.size < 3:
        padded = list(map(float, arr.tolist())) + [0.0] * (3 - int(arr.size))
        return transform_row_key(padded)
    row3 = arr[:3].reshape(1, 3)
    if allowed:
        row3 = project_angles_to_allowed(row3, allowed)
    return transform_row_key(row3[0])


def graph_valid_carry_by_group(
    group_id: Sequence[int],
    transform: Sequence,
    ngroups: int,
    max_keep: int,
) -> tuple[np.ndarray, ...]:
    """Board-valid graph transforms per group (make_polygon_graph survivors)."""
    buckets: list[list[np.ndarray]] = [[] for _ in range(max(int(ngroups), 1))]
    for gid, t in zip(group_id, transform, strict=True):
        g = int(gid)
        if g < 0 or g >= len(buckets):
            continue
        buckets[g].append(np.asarray(t, dtype=np.float64).reshape(3))
    out: list[np.ndarray] = []
    for rows in buckets:
        if not rows:
            out.append(np.zeros((0, 3), dtype=np.float64))
            continue
        stacked = np.asarray(rows, dtype=np.float64).reshape(-1, 3)
        out.append(cap_graph_valid_carry(stacked, max_keep))
    return tuple(out)


def window_selected_transforms(
    selection_window: list[tuple[np.ndarray, ...]] | None,
    ngroups: int = 2,
) -> tuple[np.ndarray, ...]:
    """Merge per-iteration nest selections from the graph window (deduped per group)."""
    if not selection_window:
        return tuple(np.zeros((0, 3)) for _ in range(ngroups))
    out = []
    for i in range(ngroups):
        parts = [w[i] for w in selection_window if len(w) > i and w[i].shape[0] > 0]
        if parts:
            out.append(dedupe_transforms(np.concatenate(parts, axis=0)))
        else:
            out.append(np.zeros((0, 3)))
    return tuple(out)


def prepend_group_transforms(
    phase1: np.ndarray,
    batch: np.ndarray,
) -> np.ndarray:
    if phase1.shape[0] == 0:
        return batch
    if batch.shape[0] == 0:
        return phase1
    return dedupe_transforms(np.concatenate([phase1, batch], axis=0))


def transform_shuffle_mix(
    sel: np.ndarray,
    hist: np.ndarray,
    count: int,
    rng: np.random.Generator,
    scale: tuple[float, float, float],
) -> np.ndarray:
    """Resample shuffled selection/history rows with fresh jitter."""
    parts = [arr for arr in (sel, hist) if arr.shape[0] > 0]
    if not parts or count <= 0:
        return np.zeros((0, 3))
    merged = np.concatenate(parts)
    rng.shuffle(merged)
    if merged.shape[0] >= count:
        picked = merged[:count]
    else:
        extra = rng.integers(0, merged.shape[0], size=count - merged.shape[0])
        picked = np.concatenate([merged, merged[extra]])
    jitter = rng.uniform(-1, 1, (picked.shape[0], 3)) * scale
    return picked + jitter


def transform_selection(s, n, rng: np.random.Generator):
    """Expand selected transforms for the next graph batch (selection_expand proposer)."""
    yield from selection_expand_arrays(s, n, rng)


def transform_history(h, n, rng: np.random.Generator):
    """Expand history transforms for the next graph batch (history_expand proposer)."""
    yield from history_expand_arrays(h, n, rng)


def build_transform_batch(
    cfg: BuildGraphConfig,
    selected_t: tuple[np.ndarray, ...],
    history: tuple[np.ndarray, ...],
    rng: np.random.Generator,
    *,
    board: BaseGeometry | None = None,
    parts: list[tuple[Polygon, int]] | None = None,
    nest_state=None,
    selection_window: list[tuple[np.ndarray, ...]] | None = None,
    first_pass: bool = False,
    border_saturation: bool = False,
    rules: PlacementRuleSet | None = None,
    proposer_counts_out: dict[str, int] | None = None,
    propose_stats_out: dict | None = None,
    propose_feedback=None,
    group_allowed_angles: Sequence[tuple[float, ...] | None] | tuple = (),
    void_elite_t: tuple[np.ndarray, ...] | None = None,
    keep_history_on_sterile: bool = False,
    part_bases: dict[int, Geometry] | None = None,
    graph_valid_carry: tuple[np.ndarray, ...] | None = None,
    archived_patterns: Sequence | None = None,
) -> tuple[np.ndarray, ...]:
    del keep_history_on_sterile  # history always kept; sterile boosts hist_q instead
    sc = cfg.sampling
    scale = sc.transform_scale
    propose_by_group: dict[int, np.ndarray] = {}
    border_pin_by_group: dict[int, np.ndarray] = {}
    empty_sheet = (
        nest_state is None
        or not nest_state.selected_indices
    )
    rim_progress = float(
        (propose_stats_out or {}).get("rim_progress", 0.0) or 0.0
    )
    rim_thr = float(getattr(cfg.propose, "rim_saturated_threshold", 0.9) or 0.9)
    rim_sat = (
        rim_thr > 0.0
        and rim_progress >= rim_thr
        and not empty_sheet
    )
    zones_used: list[str] = []
    densify_stats: dict = {}
    full_packed_geoms = None
    if (
        board is not None
        and parts is not None
        and cfg.propose.max_proposals > 0
    ):
        polys = nest_state.polys if nest_state is not None else []
        selected = nest_state.selected_indices if nest_state is not None else []
        min_dist = cfg.board_min_dist_for(board, first_pass=first_pass)
        propose_cfg = (
            cfg.first_pass_propose_config() if first_pass else cfg.propose
        )
        if rim_sat:
            mcts_zone = ""
            if propose_stats_out is not None:
                mcts_zone = str(propose_stats_out.get("mcts_zone") or "")
            propose_cfg = propose_cfg.model_copy(
                update=rim_sat_proposer_updates(mcts_zone),
            )
            if propose_stats_out is not None:
                propose_stats_out["rim_saturated_skip"] = True
        seeded = bool(nest_state is not None and nest_state.seed_count > 0)
        empty_border_only = (
            empty_sheet and cfg.propose.first_pass_empty_border_only
        )
        if empty_border_only and parts and sheet_has_narrow_corridor(
            board, parts[0][0], min_dist,
        ):
            empty_border_only = False
        border_only = (
            empty_border_only
            or (border_saturation and cfg.propose.first_pass_border_pack)
        )
        zones_used: list[str] = []
        pocket_keys_raw: dict[int, set[tuple[float, float, float]]] = {}
        densify_stats: dict = {}
        if nest_state is not None and selected:
            native = nest_state.native_geoms
            full_packed_geoms = [
                native[i] for i in selected if 0 <= i < len(native)
            ]
            if len(full_packed_geoms) != len(selected):
                full_packed_geoms = None
        force_zone = None
        if propose_stats_out is not None:
            fz = propose_stats_out.get("mcts_zone")
            if fz:
                force_zone = str(fz)
        propose_by_group, batch_stats = collect_propose_batch_for_nest(
            board,
            parts,
            polys,
            selected,
            propose_cfg,
            min_dist=min_dist,
            border_only=border_only,
            use_full_packed_obstacle=(
                cfg.propose.use_full_packed_obstacle and not empty_sheet
            ),
            rules=rules,
            proposer_counts_out=proposer_counts_out,
            propose_feedback=propose_feedback,
            packed_group_ids=(
                nest_state.group_id if nest_state is not None else None
            ),
            packed_transforms=(
                nest_state.transform if nest_state is not None else None
            ),
            group_allowed_angles=group_allowed_angles,
            user_holes=cfg.rules.board_holes,
            seeded=seeded,
            full_packed_geoms=full_packed_geoms,
            archived_patterns=archived_patterns,
            force_zone=force_zone,
        )
        zones_used = list(batch_stats.get("zones_used") or [])
        pocket_keys_raw = dict(batch_stats.get("pocket_keys_raw") or {})
        densify_stats = dict(batch_stats.get("densify_stats") or {})
        # Project angles before keying so MIS boost matches graph transforms.
        proposal_keys: dict[int, set[tuple[float, float, float]]] = {}
        pocket_keys: dict[int, set[tuple[float, float, float]]] = {}
        for gid, arr in propose_by_group.items():
            projected = arr
            if group_allowed_angles and gid < len(group_allowed_angles):
                allowed = group_allowed_angles[gid]
                if allowed is not None:
                    projected = project_angles_to_allowed(arr, allowed)
            proposal_keys[gid] = {transform_row_key(r) for r in projected}
            raw = pocket_keys_raw.get(gid) or set()
            if raw and group_allowed_angles and gid < len(group_allowed_angles):
                allowed = group_allowed_angles[gid]
                if allowed is not None and len(raw) > 0:
                    raw_arr = np.asarray(list(raw), dtype=np.float64)
                    proj = project_angles_to_allowed(raw_arr, allowed)
                    pocket_keys[gid] = {transform_row_key(r) for r in proj}
                else:
                    pocket_keys[gid] = set(raw)
            else:
                pocket_keys[gid] = set(raw)
        if propose_stats_out is not None:
            propose_stats_out.update(batch_stats)
            propose_stats_out["proposal_keys"] = proposal_keys
            propose_stats_out["pocket_keys"] = pocket_keys
            densify = propose_stats_out.get("densify_stats") or densify_stats
            motif_raw = (densify or {}).get("motif_keys") or {}
            motif_keys: dict[int, set[tuple[float, float, float]]] = {}
            for gid, raw in motif_raw.items():
                if not raw:
                    motif_keys[int(gid)] = set()
                    continue
                if group_allowed_angles and int(gid) < len(group_allowed_angles):
                    allowed = group_allowed_angles[int(gid)]
                    if allowed is not None and len(raw) > 0:
                        raw_arr = np.asarray(list(raw), dtype=np.float64)
                        proj = project_angles_to_allowed(raw_arr, allowed)
                        motif_keys[int(gid)] = {transform_row_key(r) for r in proj}
                    else:
                        motif_keys[int(gid)] = {
                            transform_row_key(np.asarray(r, dtype=np.float64))
                            for r in raw
                        }
                else:
                    motif_keys[int(gid)] = {
                        transform_row_key(np.asarray(r, dtype=np.float64))
                        for r in raw
                    }
            propose_stats_out["motif_keys"] = motif_keys
            propose_stats_out["motif_cohorts"] = list(
                (densify or {}).get("motif_cohorts") or densify_stats.get("motif_cohorts") or []
            )
            projected_cohorts: list[dict] = []
            for cohort in propose_stats_out["motif_cohorts"]:
                if not isinstance(cohort, dict):
                    continue
                out_c = dict(cohort)
                lg = int(cohort.get("leader_gid", -1))
                allowed_l = allowed_for_gid(group_allowed_angles, lg)
                if cohort.get("leader_key") is not None:
                    out_c["leader_key"] = project_row_key(
                        cohort["leader_key"], allowed_l,
                    )
                members = []
                for item in cohort.get("member_keys") or []:
                    if isinstance(item, (list, tuple)) and len(item) == 2:
                        gid_m, key_m = int(item[0]), item[1]
                        members.append((
                            gid_m,
                            project_row_key(
                                key_m, allowed_for_gid(group_allowed_angles, gid_m),
                            ),
                        ))
                out_c["member_keys"] = members
                projected_cohorts.append(out_c)
            propose_stats_out["motif_cohorts"] = projected_cohorts
            sniper_raw = (densify or {}).get("sniper_keys") or {}
            sniper_proj: dict[int, set[tuple[float, float, float]]] = {}
            for gid, raw in sniper_raw.items():
                allowed = allowed_for_gid(group_allowed_angles, int(gid))
                sniper_proj[int(gid)] = {
                    project_row_key(r, allowed) for r in (raw or ())
                }
            propose_stats_out["sniper_keys"] = sniper_proj
            raw_pairs = (densify or {}).get("batch_pack_pairs") or []
            proj_pairs = []
            for rec in raw_pairs:
                if rec is None or len(rec) < 4:
                    continue
                ca, cb, ga, gb = rec[0], rec[1], int(rec[2]), int(rec[3])
                proj_pairs.append((
                    project_row_key(ca, allowed_for_gid(group_allowed_angles, ga)),
                    project_row_key(cb, allowed_for_gid(group_allowed_angles, gb)),
                    ga,
                    gb,
                ))
            propose_stats_out["batch_pack_pairs"] = proj_pairs
            propose_stats_out["zones_used"] = zones_used
            propose_stats_out["densify_stats"] = densify_stats
            propose_stats_out["proposed_by_group"] = {
                gid: np.asarray(arr, dtype=np.float64)
                for gid, arr in propose_by_group.items()
            }
            propose_stats_out["border_only"] = bool(border_only)
        if empty_sheet and cfg.propose.use_board_edge_seeds:
            for part_poly, group_id in parts:
                border_pin_by_group[group_id] = border_edge_transforms_for_group(
                    board,
                    part_poly,
                    Polygon(),
                    propose_cfg,
                    min_dist=min_dist,
                )

    window_t = window_selected_transforms(
        selection_window, ngroups=max(len(selected_t), 2),
    )

    def _mix_group_transform_batch(
        group_id: int,
        sel: np.ndarray,
        hist: np.ndarray,
        window: np.ndarray,
    ) -> np.ndarray:
        batch_parts: list[np.ndarray] = []
        pinned = border_pin_by_group.get(group_id, np.zeros((0, 3)))
        proposed = propose_by_group.get(group_id, np.zeros((0, 3)))
        border_batch = (
            empty_sheet
            and cfg.propose.first_pass_border_pack
            and cfg.propose.first_pass_empty_border_only
        )
        if border_batch:
            if pinned.shape[0] > 0:
                batch_parts.append(pinned)
                jitter_n = sc.structured_jitter_per_proposal_empty
                if jitter_n > 0:
                    jittered = expand_structured_transforms(
                        pinned,
                        cfg.propose.structured_jitter_border_scale,
                        jitter_n,
                    )
                    if jittered.shape[0] > 0:
                        batch_parts.append(jittered)
            if proposed.shape[0] > 0:
                batch_parts.append(proposed)
        elif proposed.shape[0] > 0:
            # Proposed rows go into the stratified proposals niche; only jitter here.
            if pinned.shape[0] == 0:
                jitter_n = (
                    sc.structured_jitter_per_proposal_empty
                    if empty_sheet
                    else sc.structured_jitter_per_proposal
                )
                jittered = expand_structured_transforms(
                    proposed,
                    sc.structured_jitter_scale,
                    jitter_n,
                )
                if jittered.shape[0] > 0:
                    batch_parts.append(jittered)
        n_random = (
            cfg.propose.random_per_iter_empty_border
            if empty_sheet and cfg.propose.use_border_focus and not border_batch
            else (
                sc.random_per_iter_when_proposed
                if proposed.shape[0] > 0
                else sc.random_per_iter
            )
        )
        allowed = None
        if group_allowed_angles and group_id < len(group_allowed_angles):
            allowed = group_allowed_angles[group_id]
        # Sterile propose on a packed sheet: keep history/shuffle material so the
        # pool does not collapse to the current selection when proposers dip.
        sterile_pack = (
            not empty_sheet
            and proposed.shape[0] == 0
            and not border_batch
        )
        expand_n = sc.selection_expand_n
        shuffle_passes = int(sc.shuffle_passes)
        if not empty_sheet:
            # Collect already emits selection_expand; keep mixer expand thin.
            expand_n = max(1, expand_n // 2)
        elite = (
            void_elite_t[group_id]
            if void_elite_t is not None and group_id < len(void_elite_t)
            else np.zeros((0, 3))
        )
        expand_parts: list[np.ndarray] = []
        hist_niche = np.zeros((0, 3), dtype=np.float64)
        expand_parts.append(rng.uniform(-1, 1, (n_random, 3)) * scale)
        if hist.shape[0] > 0:
            hist_niche = hist
        carry = (
            graph_valid_carry[group_id]
            if (
                graph_valid_carry is not None
                and group_id < len(graph_valid_carry)
                and bool(getattr(cfg.propose, "enable_graph_valid_carry", True))
            )
            else np.zeros((0, 3), dtype=np.float64)
        )
        if carry.shape[0] > 0:
            # Last-iter board-valid survivors: hist niche first, remainder expand.
            if hist_niche.shape[0] == 0:
                hist_niche = carry
            else:
                hist_niche = np.concatenate([hist_niche, carry], axis=0)
            expand_parts.append(carry)
        if sel.shape[0] > 0:
            expand_parts.extend(transform_selection(sel, expand_n, rng))
            if hist.shape[0] > 0:
                expand_parts.extend(transform_history(hist, sc.history_expand_n, rng))
        if window.shape[0] > 0:
            # Window elites fold into history niche; light expand into remainder.
            if hist_niche.shape[0] == 0:
                hist_niche = window
            else:
                hist_niche = np.concatenate([hist_niche, window], axis=0)
            expand_parts.extend(transform_selection(window, expand_n, rng))
        if shuffle_passes > 0 and (
            sel.shape[0] > 0
            or hist.shape[0] > 0
            or window.shape[0] > 0
            or carry.shape[0] > 0
        ):
            for _ in range(shuffle_passes):
                shuffle_base = hist
                if carry.shape[0] > 0:
                    shuffle_base = (
                        np.concatenate([hist, carry], axis=0)
                        if hist.shape[0]
                        else carry
                    )
                expand_parts.append(
                    transform_shuffle_mix(
                        sel, shuffle_base, sc.shuffle_per_pass, rng, sc.shuffle_scale,
                    )
                )
        if expand_parts:
            expand_rest = dedupe_transforms(np.concatenate(expand_parts))
        else:
            expand_rest = np.zeros((0, 3), dtype=np.float64)
        # Structured jitter / border pins from earlier batch_parts → expand remainder.
        if batch_parts:
            extra = dedupe_transforms(np.concatenate(batch_parts))
            expand_rest = (
                dedupe_transforms(np.concatenate([expand_rest, extra]))
                if expand_rest.shape[0]
                else extra
            )
        if (
            bool(getattr(cfg.propose, "prune_colliding_transforms", False))
            and full_packed_geoms
            and part_bases is not None
            and group_id in part_bases
            and expand_rest.shape[0] > 0
        ):
            expand_rest = prune_transforms_vs_packed(
                expand_rest, part_bases[group_id], full_packed_geoms,
            )
        densify_hit = (
            int((densify_stats or {}).get("accepted", 0) or 0) > 0
            or any("void_seek" in str(z) for z in zones_used)
        )
        n_props = (
            int(ProposeConfig.void_seek_budget_floors()[1])
            if densify_hit
            else max(int(cfg.propose.max_proposals), 1)
        )
        # P1: DG / Motif soft steer — raise mix floors under void_seek or Motif gids.
        mcts_zone = str((propose_stats_out or {}).get("mcts_zone") or "")
        motif_gids_pre = (propose_stats_out or {}).get("mcts_motif_gids") or []
        if mcts_zone == "void_seek" or motif_gids_pre:
            floor_props = int(ProposeConfig.void_seek_budget_floors()[1])
            n_props = max(n_props, int(floor_props * 1.25) + 8)
            if propose_stats_out is not None and group_id == 0:
                propose_stats_out["dg_mix_boost"] = 1
        # D1: soft mcts_part_gid / Motif-pair mix boost (no hard filter).
        prefer_gid = int((propose_stats_out or {}).get("mcts_part_gid", -1))
        motif_gids = motif_gids_pre
        motif_set = {int(g) for g in motif_gids}
        if prefer_gid >= 0 and int(group_id) == prefer_gid:
            n_props = max(n_props, int(n_props * 1.35) + 4)
            expand_n = max(expand_n, expand_n + 2)
            if propose_stats_out is not None:
                propose_stats_out["mcts_part_gid_boost"] = int(
                    propose_stats_out.get("mcts_part_gid_boost", 0)
                ) + 1
        elif int(group_id) in motif_set:
            n_props = max(n_props, int(n_props * 1.25) + 2)
            expand_n = max(expand_n, expand_n + 1)
            if propose_stats_out is not None:
                propose_stats_out["mcts_motif_gid_boost"] = int(
                    propose_stats_out.get("mcts_motif_gid_boost", 0)
                ) + 1
        proposal_pins = (
            proposed if proposed.shape[0] > 0 else np.zeros((0, 3), dtype=np.float64)
        )
        if pinned.shape[0] > 0:
            proposal_pins = (
                dedupe_transforms(np.concatenate([pinned, proposal_pins], axis=0))
                if proposal_pins.shape[0]
                else pinned
            )
        if allowed is not None and proposal_pins.shape[0] > 0:
            proposal_pins = dedupe_transforms(
                project_angles_to_allowed(proposal_pins, allowed)
            )
        elite_q = int(getattr(cfg.propose, "stratified_void_elite_quota", 15))
        hist_q = int(getattr(cfg.propose, "stratified_history_quota", 15))
        if sterile_pack:
            cut_boost = bool(
                propose_stats_out is not None
                and propose_stats_out.get("cut_sterile_hist_boost")
            )
            if not cut_boost:
                hist_boost = int(
                    getattr(cfg.propose, "sterile_history_quota_boost", 128) or 0
                )
                if hist_boost > 0:
                    hist_q = max(hist_q, hist_boost)
        if propose_stats_out is not None and group_id == 0:
            propose_stats_out["carry_n"] = int(carry.shape[0])
            propose_stats_out["hist_niche_n"] = int(hist_niche.shape[0])
            propose_stats_out["hist_q"] = int(hist_q)
            propose_stats_out["mix_props"] = int(n_props)
            propose_stats_out["rim_skip"] = int(bool(rim_sat))
        merged = subsample_transforms_stratified(
            selection=sel,
            proposals=proposal_pins,
            void_elite=elite,
            history=hist_niche,
            expand_rest=expand_rest,
            max_n=sc.max_transforms_per_group,
            rng=rng,
            n_props=n_props,
            n_void_elite=elite_q,
            n_hist=hist_q,
        )
        if propose_stats_out is not None:
            sel_keys = {
                transform_row_key(np.asarray(r, dtype=np.float64)) for r in sel
            }
            mix_keys = {
                transform_row_key(np.asarray(r, dtype=np.float64)) for r in merged
            }
            kept = len(sel_keys & mix_keys) if sel_keys else 0
            propose_stats_out["sel_kept"] = int(
                propose_stats_out.get("sel_kept", 0)
            ) + kept
        if allowed is not None and merged.shape[0] > 0:
            merged = project_angles_to_allowed(merged, allowed)
            merged = dedupe_transforms(merged)
        return merged

    out = []
    for i in range(len(selected_t)):
        sel = selected_t[i] if i < len(selected_t) else np.zeros((0, 3))
        hist = history[i] if i < len(history) else np.zeros((0, 3))
        win = window_t[i] if i < len(window_t) else np.zeros((0, 3))
        out.append(_mix_group_transform_batch(i, sel, hist, win))
    mixed = tuple(out)
    if propose_stats_out is not None:
        mixed_keys: dict[int, set[tuple[float, float, float]]] = {}
        for gid, arr in enumerate(mixed):
            mixed_keys[gid] = {
                transform_row_key(np.asarray(r, dtype=np.float64)) for r in arr
            }
        for name in ("sniper_keys", "proposal_keys"):
            raw = propose_stats_out.get(name) or {}
            if not raw:
                continue
            propose_stats_out[name] = {
                int(gid): set(keys) & mixed_keys.get(int(gid), set())
                for gid, keys in raw.items()
            }
    return mixed


