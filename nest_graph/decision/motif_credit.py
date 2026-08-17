"""Motif / niche credit helpers for Macro-MCTS expand bookkeep."""

from typing import Any, Sequence

import numpy as np

from nest_graph.elem_graph import Se2
from nest_graph.propose.void_selection import transform_row_key
from nest_graph.utils import relative_transform


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
) -> dict[str, int]:
    """P0 evaluator/build_graph SoT: hollow rescue + void-nest↑ niche credit.

    Uses Void AMAF key ``(Void, 0, -1)`` when no MacroAction is available
    (evaluator spine). Returns telem counters.
    """
    from nest_graph.elem_graph import MacroRegion
    from nest_graph.propose.void_selection import centroid_in_free as _cif

    cif = centroid_in_free_fn or _cif
    telem = {"hollow_miss": 0, "niche_pos": 0, "niche_rescue": 0}
    large_void = str(free_kind or "") == "large_void"
    if niche_archive is None or not large_void:
        return telem
    void_key = (
        int(getattr(MacroRegion.Void, "value", 1)),
        0,
        -1,
    )
    hollow = bool(
        bottleneck == "graph_to_nest" or int(n_void_nest) <= 0
    )
    telem["hollow_miss"] = int(hollow)
    if hollow and int(n_void_graph) > 0:
        niche_archive.append_negative(void_key, void_nest=int(n_void_nest), score=0.0)
        pos_rows: list[tuple[int, float, float, float]] = []
        seed_idxs = [
            i for i in range(len(polys)) if cif(polys[i], free_poly)
        ][: max(int(max_seed), 1)]
        for vii in seed_idxs:
            if vii < 0 or vii >= len(transform):
                continue
            gid = int(group_id[vii]) if vii < len(group_id) else 0
            row = np.asarray(transform[vii], dtype=np.float64).reshape(3)
            pos_rows.append((gid, float(row[0]), float(row[1]), float(row[2])))
        if pos_rows:
            niche_archive.append_positive(
                void_key,
                rows=pos_rows,
                void_nest=0,
                score=float(outline_cov),
                ttl=max(int(ttl), 1),
            )
            telem["niche_rescue"] = len(pos_rows)
    elif int(n_void_nest) > int(prev_void_nest):
        pos_rows = []
        for vi in selected:
            vii = int(vi)
            if vii < 0 or vii >= len(polys) or vii >= len(transform):
                continue
            if not cif(polys[vii], free_poly):
                continue
            gid = int(group_id[vii]) if vii < len(group_id) else 0
            row = np.asarray(transform[vii], dtype=np.float64).reshape(3)
            pos_rows.append((gid, float(row[0]), float(row[1]), float(row[2])))
        if pos_rows:
            niche_archive.append_positive(
                void_key,
                rows=pos_rows,
                void_nest=int(n_void_nest),
                score=float(outline_cov),
                ttl=max(int(ttl), 1),
            )
            telem["niche_pos"] = len(pos_rows)
    return telem
