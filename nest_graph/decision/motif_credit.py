"""Motif / niche credit helpers for Macro-MCTS expand bookkeep."""

from typing import Any, Sequence

import numpy as np

from nest_graph.elem_graph import MacroRegion, Se2
from nest_graph.propose.void_selection import centroid_in_free, transform_row_key
from nest_graph.utils import relative_transform


def niche_amaf_key(action: Any | None) -> tuple[int, int, int]:
    """Q140/Q141: ``(region, 0, motif_id)``; Void/-1 when there is no action."""
    if action is None:
        return (int(getattr(MacroRegion.Void, "value", 1)), 0, -1)
    region = action.region
    region_i = int(getattr(region, "value", region))
    motif_id = int(getattr(action, "motif_id", -1) or -1)
    if motif_id < 0:
        motif_id = -1
    return (region_i, 0, motif_id)


def _key_is_void_seek(key: tuple[int, int, int]) -> bool:
    region_i = int(key[0])
    void_i = int(getattr(MacroRegion.Void, "value", 1))
    motif_i = int(getattr(MacroRegion.Motif, "value", 3))
    return region_i == void_i or region_i == motif_i


def _pose_in_proposer_keys(row3, proposer_keys: dict | None) -> bool:
    if not proposer_keys:
        return False
    key = transform_row_key(np.asarray(row3, dtype=np.float64))
    for owned in proposer_keys.values():
        if key in (owned or ()):
            return True
    return False


def _niche_rows_from_indices(
    indices: Sequence[int],
    *,
    polys: Sequence,
    transform: Sequence,
    group_id: Sequence[int],
    free_poly: Any,
    cif,
    proposer_keys: dict | None,
    cap: int | None = None,
) -> tuple[list[tuple[int, float, float, float]], list[tuple[int, float, float, float]]]:
    """Return (proposer-tagged nest-in-free rows, all nest-in-free rows)."""
    tagged: list[tuple[int, float, float, float]] = []
    in_free: list[tuple[int, float, float, float]] = []
    for vi in indices:
        vii = int(vi)
        if vii < 0 or vii >= len(polys) or vii >= len(transform):
            continue
        if not cif(polys[vii], free_poly):
            continue
        gid = int(group_id[vii]) if vii < len(group_id) else 0
        row = np.asarray(transform[vii], dtype=np.float64).reshape(3)
        rec = (gid, float(row[0]), float(row[1]), float(row[2]))
        in_free.append(rec)
        if _pose_in_proposer_keys(row, proposer_keys):
            tagged.append(rec)
        if cap is not None and len(in_free) >= int(cap):
            break
    return tagged, in_free


def credit_motif_on_nest_survival(
    motif_base: Any,
    *,
    selected_polys: Sequence[int],
    group_id: Sequence[int],
    transform: Sequence,
    motif_keys: dict | None,
    ttl: int,
    telem: dict | None = None,
) -> int:
    """Q116: accept_count++ when nest selection keeps Motif-keyed vertices."""
    if motif_base is None or not selected_polys or not motif_keys:
        return 0
    credited = 0
    keys_by_gid = motif_keys or {}
    motif_idxs: list[int] = []
    for i in selected_polys:
        gi = int(i)
        if gi < 0 or gi >= len(group_id) or gi >= len(transform):
            continue
        gid = int(group_id[gi])
        owned = keys_by_gid.get(gid) or set()
        if not owned:
            continue
        key = transform_row_key(np.asarray(transform[gi], dtype=np.float64))
        if key in owned:
            motif_idxs.append(gi)
    for a, b in zip(motif_idxs, motif_idxs[1:]):
        ga, gb = int(group_id[a]), int(group_id[b])
        ta, tb = transform[a], transform[b]
        rel = relative_transform(
            (float(ta[0]), float(ta[1]), float(ta[2])),
            (float(tb[0]), float(tb[1]), float(tb[2])),
        )
        mid = int(
            motif_base.find_exact_id(
                ga,
                gb,
                Se2(float(rel[0]), float(rel[1]), float(rel[2])),
            )
        )
        if mid < 0:
            mid = int(
                motif_base.find_nearest_id(
                    ga,
                    gb,
                    Se2(float(rel[0]), float(rel[1]), float(rel[2])),
                )
            )
        if mid >= 0 and motif_base.credit_accept(mid, int(ttl)):
            credited += 1
    if telem is not None:
        telem["motif_nest_credit"] = int(telem.get("motif_nest_credit", 0)) + credited
    return credited


def merge_void_elite_with_archive(
    current: dict[int, list],
    archive_by_group: dict[int, list],
    *,
    elite_quota: int,
    void_seek: bool,
) -> dict[int, list]:
    """Hybrid abs elite bag: current void_elite ∪ archive (Q137).

    When current is empty under void_seek, archive supplies the **full** quota
    (not 40%). Otherwise 60% current / 40% archive.
    """
    q = max(int(elite_quota), 0)
    if q <= 0:
        return {}
    if not void_seek or not archive_by_group:
        out: dict[int, list] = {}
        for gid, rows in (current or {}).items():
            out[int(gid)] = list(rows)[:q]
        return out
    cur_total = sum(len(v) for v in (current or {}).values())
    if cur_total <= 0:
        n_cur = 0
        n_arch = q
    else:
        n_cur = max(1, int(round(0.6 * q)))
        n_arch = max(0, q - n_cur)
    out: dict[int, list] = {}
    gids = set(int(g) for g in (current or {})) | set(int(g) for g in archive_by_group)
    for gid in gids:
        cur_rows = list((current or {}).get(gid) or [])[:n_cur]
        arch_rows = list(archive_by_group.get(gid) or [])[:n_arch]
        merged = cur_rows + arch_rows
        seen: set[tuple] = set()
        kept: list = []
        for row in merged:
            arr = np.asarray(row, dtype=np.float64).reshape(3)
            key = (
                round(float(arr[0]), 4),
                round(float(arr[1]), 4),
                round(float(arr[2]), 4),
            )
            if key in seen:
                continue
            seen.add(key)
            kept.append(arr)
            if len(kept) >= q:
                break
        if kept:
            out[int(gid)] = kept
    return out


def credit_void_niche_from_iter(
    niche_archive: Any,
    *,
    free_kind: str,
    bottleneck: str,
    n_void_nest: int,
    n_void_graph: int,
    prev_void_nest: int,
    polys: Sequence,
    transform: Sequence,
    group_id: Sequence[int],
    selected: Sequence[int],
    free_poly: Any,
    outline_cov: float,
    ttl: int,
    max_seed: int = 64,
    centroid_in_free_fn: Any = None,
    amaf_key: tuple[int, int, int] | None = None,
    proposer_keys: dict | None = None,
    action: Any | None = None,
) -> dict[str, int]:
    """One niche write per iter (Q140/Q142): current AMAF key, one ``append_positive``.

    Hollow: ``append_negative(key)`` then one positive — proposer-tagged nest
    survivors if any, else nest-in-void, else centroid ghosts. ``(Void, 0, -1)``
    only when there is no action.
    """
    cif = centroid_in_free_fn or centroid_in_free
    telem = {"hollow_miss": 0, "niche_pos": 0, "niche_rescue": 0}
    large_void = str(free_kind or "") == "large_void"
    if niche_archive is None or not large_void:
        return telem
    key = amaf_key if amaf_key is not None else niche_amaf_key(action)
    hollow = bool(bottleneck == "graph_to_nest" or int(n_void_nest) <= 0)
    telem["hollow_miss"] = int(hollow)
    tagged, nest_free = _niche_rows_from_indices(
        selected,
        polys=polys,
        transform=transform,
        group_id=group_id,
        free_poly=free_poly,
        cif=cif,
        proposer_keys=proposer_keys,
    )
    pos_rows = tagged or nest_free
    if hollow:
        niche_archive.append_negative(key, void_nest=int(n_void_nest), score=0.0)
        # Q140: current key only — no Void dump on Rim/Sheet.
        # Q142: tagged nest survivors first; else ghosts (old hollow SoT), not
        # untagged nest_free (that subset starved void_seek vs full graph seeds).
        pos_rows = list(tagged)
        if not pos_rows and _key_is_void_seek(key) and int(n_void_graph) > 0:
            _tagged_g, ghosts = _niche_rows_from_indices(
                range(len(polys)),
                polys=polys,
                transform=transform,
                group_id=group_id,
                free_poly=free_poly,
                cif=cif,
                proposer_keys=None,
                cap=max(int(max_seed), 1),
            )
            del _tagged_g
            pos_rows = ghosts
        if pos_rows:
            niche_archive.append_positive(
                key,
                rows=pos_rows,
                void_nest=0 if not tagged else int(n_void_nest),
                score=float(outline_cov),
                ttl=max(int(ttl), 1),
            )
            if tagged:
                telem["niche_pos"] = len(pos_rows)
            else:
                telem["niche_rescue"] = len(pos_rows)
    elif int(n_void_nest) > int(prev_void_nest) and pos_rows:
        niche_archive.append_positive(
            key,
            rows=pos_rows,
            void_nest=int(n_void_nest),
            score=float(outline_cov),
            ttl=max(int(ttl), 1),
        )
        telem["niche_pos"] = len(pos_rows)
    return telem
