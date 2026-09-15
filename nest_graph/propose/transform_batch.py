"""Propose transform-batch assembly (moved from build_graph)."""

from typing import Any, Sequence

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
from nest_graph.graph import PlacementRuleSet
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
from nest_graph.propose.motif_keys import resolve_motif_keys
from nest_graph.propose.placement_common import (
    clear_of_geoms,
    dual_pose_from_base,
    placement_obstacles,
)
from nest_graph.utils import compose_transforms


def rim_sat_proposer_updates(
    mcts_zone: str,
    *,
    soft_side_pack: bool = False,
    side_pack_scale: float = 0.25,
    side_pack_top_n: int = 256,
) -> dict:
    """Q145/Q182: rim-sat mute. Soft-scale side_pack when inward bridge on.

    Void/Motif map to ``void_seek`` (Q104). ``cluster_copy`` is never muted here.
    """
    updates: dict = {}
    if soft_side_pack:
        scale = max(0.0, min(1.0, float(side_pack_scale)))
        updates["side_pack_top_n"] = max(1, int(float(side_pack_top_n) * scale))
    else:
        updates["use_side_pack"] = False
    zone = str(mcts_zone or "")
    if zone in ("cluster_edge", "interior_pocket"):
        updates["use_history_expand"] = False
    return updates


def prune_transforms_vs_packed(
    transforms: np.ndarray,
    base: Geometry | None,
    packed: Sequence[Geometry] | None,
) -> np.ndarray:
    """Drop expand_rest rows that penetrate locked packed solids.

    Penetrating SoT via ``intersects_any`` (not guidance ``batch_check_validity``).
    """
    if (
        transforms is None
        or transforms.size == 0
        or base is None
        or not packed
    ):
        return transforms
    # One list for the loop — avoid per-row list(packed) copy thrash.
    packed_list = packed if isinstance(packed, list) else list(packed)
    keep: list[np.ndarray] = []
    for row in np.asarray(transforms, dtype=np.float64).reshape(-1, 3):
        placed = base.apply_transform(row)
        if placed is None:
            continue
        if placed.intersects_any(packed_list):
            continue
        keep.append(row)
    if not keep:
        return np.zeros((0, 3), dtype=np.float64)
    return np.asarray(keep, dtype=np.float64).reshape(-1, 3)


def _archive_mix_pin_rows(
    *,
    archived_patterns: Sequence,
    group_id: int,
    motif_floor_n: int,
    motif_key_set: set[tuple[float, float, float]],
    nest_state: Any,
    part_bases: dict,
    parts: Sequence,
    min_dist: float,
    propose_stats_out: dict | None,
    existing_rows: list[np.ndarray],
) -> list[np.ndarray]:
    """Q333/Q340: archive ref_transform pins (leader + follower world poses)."""
    if (
        motif_floor_n <= 0
        or not archived_patterns
        or nest_state is None
        or part_bases is None
    ):
        return existing_rows
    voids_ob = list(getattr(nest_state, "void_geoms", None) or ())
    packed_ob = list(nest_state.native_geoms or [])
    rows = list(existing_rows)
    seen_pin_keys = {
        transform_row_key(np.asarray(r, dtype=np.float64)) for r in rows
    }
    reject_n = 0
    for pat in archived_patterns:
        if len(rows) >= motif_floor_n:
            break
        members = tuple(getattr(pat, "members", ()) or ())
        if len(members) < 2:
            continue
        gid_a, _rel_a = members[0]
        gid_b, rel_b = members[1]
        rt = np.asarray(pat.ref_transform, dtype=np.float64).reshape(3)
        pin_pairs: list[tuple[int, np.ndarray]] = []
        if int(group_id) == int(gid_a):
            pin_pairs.append((int(gid_a), rt))
        if int(group_id) == int(gid_b):
            pin_pairs.append(
                (
                    int(gid_b),
                    np.asarray(
                        compose_transforms(
                            (float(rt[0]), float(rt[1]), float(rt[2])),
                            (float(rel_b[0]), float(rel_b[1]), float(rel_b[2])),
                        ),
                        dtype=np.float64,
                    ).reshape(3),
                )
            )
        for gid_pat, pin_tf in pin_pairs:
            if len(rows) >= motif_floor_n:
                break
            if int(group_id) != int(gid_pat):
                continue
            key = transform_row_key(pin_tf)
            if key in seen_pin_keys or key in motif_key_set:
                continue
            base_g = part_bases.get(int(group_id))
            part_poly = None
            if parts:
                for p, g in parts:
                    if int(g) == int(group_id):
                        part_poly = p
                        break
            if base_g is None or part_poly is None:
                reject_n += 1
                continue
            cand_g, _ = dual_pose_from_base(base_g, part_poly, pin_tf)
            if cand_g is None:
                reject_n += 1
                continue
            obs = placement_obstacles(voids_ob, [])
            if obs and not clear_of_geoms(cand_g, obs, 0.0):
                reject_n += 1
                continue
            rows.append(pin_tf)
            seen_pin_keys.add(key)
    if propose_stats_out is not None and reject_n > 0:
        propose_stats_out["archive_mix_reject_n"] = int(
            propose_stats_out.get("archive_mix_reject_n", 0)
        ) + int(reject_n)
    added = max(0, len(rows) - len(existing_rows))
    if propose_stats_out is not None and added > 0:
        propose_stats_out["archive_mix_floor_hits"] = int(
            propose_stats_out.get("archive_mix_floor_hits", 0)
        ) + int(added)
    return rows


def _motif_mix_floor_rows(
    *,
    group_id: int,
    motif_floor_n: int,
    motif_key_set: set[tuple[float, float, float]],
    archived_patterns: Sequence,
    nest_state: Any,
    part_bases: dict,
    parts: Sequence,
    min_dist: float,
    proposal_pins: np.ndarray,
    propose_stats_out: dict | None,
    archive_ok: bool,
) -> tuple[np.ndarray, set[tuple[float, float, float]]]:
    """U-A2: archive-first mix floor, cluster_copy fills remainder (Q333)."""
    if motif_floor_n <= 0:
        if propose_stats_out is not None and group_id == 0:
            propose_stats_out["motif_floor_active"] = 0
        return np.zeros((0, 3), dtype=np.float64), set()
    if propose_stats_out is not None and group_id == 0:
        propose_stats_out["motif_floor_active"] = 1
        propose_stats_out["archive_mix_attempt_n"] = int(
            propose_stats_out.get("archive_mix_attempt_n", 0)
        ) + 1
    rows: list[np.ndarray] = []
    skip_reason = "none"
    if not archived_patterns:
        skip_reason = "no_archive"
    elif not archive_ok:
        skip_reason = str(
            (propose_stats_out or {}).get("archive_mix_skip_reason") or "no_void_seek"
        )
    elif nest_state is None or part_bases is None:
        skip_reason = "no_archive"
    if archive_ok and archived_patterns and nest_state is not None and part_bases is not None:
        rows = _archive_mix_pin_rows(
            archived_patterns=archived_patterns,
            group_id=int(group_id),
            motif_floor_n=int(motif_floor_n),
            motif_key_set=motif_key_set,
            nest_state=nest_state,
            part_bases=part_bases,
            parts=parts,
            min_dist=min_dist,
            propose_stats_out=propose_stats_out,
            existing_rows=rows,
        )
        if not rows and propose_stats_out is not None and group_id == 0:
            skip_reason = "floor_zero"
    elif propose_stats_out is not None and group_id == 0 and skip_reason == "none":
        fk = str((propose_stats_out or {}).get("free_kind", "") or "")
        on_plat = bool((propose_stats_out or {}).get("on_plateau", False))
        if not on_plat:
            skip_reason = "no_plateau"
        elif fk != "large_void":
            skip_reason = "no_void_seek"
    if motif_key_set and len(rows) < motif_floor_n:
        cc_before = len(rows)
        for r in proposal_pins:
            if transform_row_key(np.asarray(r, dtype=np.float64)) in motif_key_set:
                rows.append(np.asarray(r, dtype=np.float64))
                if len(rows) >= motif_floor_n:
                    break
        cc_added = len(rows) - cc_before
        if cc_added and propose_stats_out is not None:
            propose_stats_out["cluster_copy_mix_floor_hits"] = int(
                propose_stats_out.get("cluster_copy_mix_floor_hits", 0)
            ) + int(cc_added)
    pin_keys = {
        transform_row_key(np.asarray(r, dtype=np.float64)) for r in rows
    }
    if propose_stats_out is not None:
        merged_pins = set(propose_stats_out.get("archive_mix_pin_keys") or ())
        merged_pins.update(pin_keys)
        propose_stats_out["archive_mix_pin_keys"] = merged_pins
        if group_id == 0 and skip_reason != "none":
            propose_stats_out["archive_mix_skip_reason"] = skip_reason
    if not rows:
        return np.zeros((0, 3), dtype=np.float64), pin_keys
    return np.asarray(rows, dtype=np.float64).reshape(-1, 3), pin_keys


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


def project_proposer_keys(
    proposer_keys: dict[str, set[tuple[float, float, float]]] | None,
    group_allowed_angles: Sequence | None,
) -> dict[str, set[tuple[float, float, float]]]:
    """Project flat emit keys so MIS boost / telem match graph transforms."""
    if not proposer_keys:
        return {}
    allowed_list = list(group_allowed_angles or ())
    out: dict[str, set[tuple[float, float, float]]] = {}
    for name, keys in proposer_keys.items():
        proj: set[tuple[float, float, float]] = set()
        for key in keys or ():
            if allowed_list:
                for allowed in allowed_list:
                    proj.add(project_row_key(key, allowed))
            else:
                proj.add(project_row_key(key, None))
        if proj:
            out[str(name)] = proj
    return out


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


def _ensure_archive_pin_pairs(
    mixed: tuple[np.ndarray, ...],
    archived_patterns: Sequence,
    group_allowed_angles: Sequence[tuple[float, ...] | None] | tuple,
) -> tuple[tuple[np.ndarray, ...], int]:
    """Q340: if subsample kept one archive pin but dropped its partner, restore it."""
    if not archived_patterns or not mixed:
        return mixed, 0
    out: list[np.ndarray] = []
    for arr in mixed:
        if arr is None or np.asarray(arr).size == 0:
            out.append(np.zeros((0, 3), dtype=np.float64))
        else:
            out.append(np.asarray(arr, dtype=np.float64).reshape(-1, 3))
    n_groups = len(out)
    restored = 0
    for pat in archived_patterns:
        members = tuple(getattr(pat, "members", ()) or ())
        if len(members) < 2:
            continue
        gid_a, _rel_a = members[0]
        gid_b, rel_b = members[1]
        rt_raw = getattr(pat, "ref_transform", None)
        if rt_raw is None:
            continue
        rt = np.asarray(rt_raw, dtype=np.float64).reshape(3)
        follower_tf = np.asarray(
            compose_transforms(
                (float(rt[0]), float(rt[1]), float(rt[2])),
                (float(rel_b[0]), float(rel_b[1]), float(rel_b[2])),
            ),
            dtype=np.float64,
        ).reshape(3)
        pair_rows = [(int(gid_a), rt), (int(gid_b), follower_tf)]
        present: list[tuple[int, np.ndarray, bool]] = []
        for gid_pat, tf_row in pair_rows:
            if gid_pat < 0 or gid_pat >= n_groups:
                continue
            allowed = allowed_for_gid(group_allowed_angles, gid_pat)
            tf_use = tf_row.reshape(1, 3)
            if allowed is not None:
                tf_use = project_angles_to_allowed(tf_use, allowed)
            row = tf_use[0]
            keys_in = {
                transform_row_key(np.asarray(r, dtype=np.float64)) for r in out[gid_pat]
            }
            present.append((gid_pat, row, transform_row_key(row) in keys_in))
        if len(present) != 2:
            continue
        if present[0][2] and present[1][2]:
            continue
        if not present[0][2] and not present[1][2]:
            continue
        for gid_pat, row, is_present in present:
            if is_present:
                continue
            cur = out[gid_pat]
            out[gid_pat] = dedupe_transforms(
                np.concatenate([cur, row.reshape(1, 3)], axis=0)
                if cur.shape[0] > 0
                else row.reshape(1, 3)
            )
            restored += 1
    return tuple(out), int(restored)


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
    densify_stats: dict[str, Any] = {}
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
            soft_side = bool(getattr(propose_cfg, "enable_inward_bridge", True))
            propose_cfg = propose_cfg.model_copy(
                update=rim_sat_proposer_updates(
                    mcts_zone,
                    soft_side_pack=soft_side,
                    side_pack_scale=float(
                        getattr(propose_cfg, "rim_sat_side_pack_scale", 0.25) or 0.25
                    ),
                    side_pack_top_n=int(
                        getattr(propose_cfg, "side_pack_top_n", 256) or 256
                    ),
                ),
            )
            if propose_stats_out is not None:
                propose_stats_out["rim_saturated_skip"] = not soft_side
                propose_stats_out["rim_sat_soft_side_pack"] = int(soft_side)
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
        zones_used = []
        pocket_keys_raw: dict[int, set[tuple[float, float, float]]] = {}
        densify_stats = {}
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
            pk_raw = (densify or {}).get("proposer_keys") or {}
            if pk_raw:
                pk_proj = project_proposer_keys(pk_raw, group_allowed_angles)
                propose_stats_out["proposer_keys"] = pk_proj
                densify_stats["proposer_keys"] = {
                    name: set(keys) for name, keys in pk_proj.items()
                }
            propose_stats_out["zones_used"] = zones_used
            propose_stats_out["densify_stats"] = densify_stats
            propose_stats_out["ray_ms"] = float(densify_stats.get("ray_ms", 0.0) or 0.0)
            propose_stats_out["erosion_ms"] = float(
                densify_stats.get("erosion_ms", 0.0) or 0.0
            )
            propose_stats_out["pocket_ms"] = float(
                densify_stats.get("pocket_ms", 0.0) or 0.0
            )
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
        hist_expand_n = int(sc.history_expand_n)
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
                expand_parts.extend(transform_history(hist, hist_expand_n, rng))
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
        # Q191: plateau + free remaining → expand void/inward propose budget.
        # Last leaf also boosts when short fixtures never reach plateau.
        on_plateau = bool((propose_stats_out or {}).get("on_plateau", False))
        last_leaf = bool((propose_stats_out or {}).get("is_last_leaf", False))
        free_kind = str((propose_stats_out or {}).get("free_kind", "") or "")
        if free_kind == "large_void" and (on_plateau or last_leaf):
            n_props = max(n_props, int(n_props * 1.35) + 32)
            if propose_stats_out is not None and group_id == 0:
                propose_stats_out["plateau_props_boost"] = 1
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
        # Q186: under plateau (or last leaf) + inward bridge, hard floor
        # motif/cluster_copy keys in mix. Short fixtures (iters < flat_iters) never
        # reach PlateauTracker; last leaf still needs structure reuse.
        motif_floor_n = 0
        last_leaf = bool((propose_stats_out or {}).get("is_last_leaf", False))
        if bool(getattr(cfg.propose, "enable_inward_bridge", True)) and (
            on_plateau or last_leaf
        ):
            motif_floor_n = int(
                getattr(cfg.propose, "cluster_copy_mix_floor", 12) or 12
            )
        motif_pin = np.zeros((0, 3), dtype=np.float64)
        if motif_floor_n > 0:
            resolved = resolve_motif_keys(
                propose_stats_out, densify=(propose_stats_out or {}).get("densify_stats"),
                gid=int(group_id),
            )
            motif_key_set = set(resolved.get(int(group_id)) or ())
            # Q336: plateau + large_void archive mix regardless of mcts_zone.
            archive_ok = (
                archived_patterns
                and nest_state is not None
                and part_bases is not None
                and (
                    on_plateau and free_kind == "large_void"
                    or mcts_zone == "void_seek"
                    or any("void_seek" in str(z) for z in zones_used)
                )
            )
            motif_pin, _pin_keys = _motif_mix_floor_rows(
                group_id=int(group_id),
                motif_floor_n=int(motif_floor_n),
                motif_key_set=motif_key_set,
                archived_patterns=archived_patterns or (),
                nest_state=nest_state,
                part_bases=part_bases or {},
                parts=parts or [],
                min_dist=min_dist,
                proposal_pins=proposal_pins,
                propose_stats_out=propose_stats_out,
                archive_ok=bool(archive_ok),
            )
        if motif_pin.shape[0] > 0:
            proposal_pins = dedupe_transforms(
                np.concatenate([motif_pin, proposal_pins], axis=0)
            )
            if on_plateau or last_leaf:
                sel = (
                    dedupe_transforms(np.concatenate([motif_pin, sel], axis=0))
                    if sel.shape[0]
                    else motif_pin
                )
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
        if propose_stats_out is not None:
            sel_arr = sel
            hist_arr = hist_niche
            if allowed is not None:
                if sel_arr.shape[0] > 0:
                    sel_arr = project_angles_to_allowed(sel_arr, allowed)
                if hist_arr.shape[0] > 0:
                    hist_arr = project_angles_to_allowed(hist_arr, allowed)
            propose_stats_out.setdefault("sel_keys", {})[int(group_id)] = {
                transform_row_key(np.asarray(r, dtype=np.float64)) for r in sel_arr
            }
            propose_stats_out.setdefault("hist_keys", {})[int(group_id)] = {
                transform_row_key(np.asarray(r, dtype=np.float64)) for r in hist_arr
            }
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
            raw_stats = propose_stats_out.get(name)
            if not isinstance(raw_stats, dict) or not raw_stats:
                continue
            propose_stats_out[name] = {
                int(gid): set(keys) & mixed_keys.get(int(gid), set())
                for gid, keys in raw_stats.items()
            }
        sel_keys_all = propose_stats_out.get("sel_keys") or {}
        hist_keys_all = propose_stats_out.get("hist_keys") or {}
        propose_stats_out["epoch_keys"] = {
            int(gid): set(keys)
            - set(sel_keys_all.get(int(gid)) or ())
            - set(hist_keys_all.get(int(gid)) or ())
            for gid, keys in mixed_keys.items()
        }
    if archived_patterns:
        mixed, pair_restore = _ensure_archive_pin_pairs(
            mixed, archived_patterns, group_allowed_angles,
        )
        if propose_stats_out is not None and pair_restore > 0:
            propose_stats_out["archive_pin_pair_restore_n"] = int(
                propose_stats_out.get("archive_pin_pair_restore_n", 0)
            ) + int(pair_restore)
    return mixed


