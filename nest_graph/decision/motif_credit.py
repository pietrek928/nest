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
    """Q137: under VOID_SEEK, 60% current void_elite / 40% archive abs SE2."""
    q = max(int(elite_quota), 0)
    if q <= 0:
        return {}
    if not void_seek or not archive_by_group:
        out: dict[int, list] = {}
        for gid, rows in (current or {}).items():
            out[int(gid)] = list(rows)[:q]
        return out
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
