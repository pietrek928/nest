"""Motif key SoT helpers (Q196–Q197): fold emit keys + one resolve prefer path.

Leaf module: no imports from pattern_archive / void_selection (avoids cycles).
"""

from typing import Any, Mapping, Sequence

import numpy as np

from nest_graph.utils import transform_row_key


def _as_key_tuple(raw: Any) -> tuple[float, float, float] | None:
    """Round-4 join key (same as transform_row_key) so fold ↔ mix floor match."""
    if raw is None:
        return None
    try:
        if isinstance(raw, tuple) and len(raw) >= 3:
            return transform_row_key(raw)
        arr = list(raw)
    except (TypeError, ValueError, IndexError):
        return None
    if len(arr) < 3:
        return None
    return transform_row_key(arr)


def fold_emit_motif_keys(
    by_gid: dict[int, set[tuple[float, float, float]]],
    *,
    group_id: int,
    motif_hole_keys: Sequence = (),
    cluster_copy_keys: Sequence = (),
) -> None:
    """Q197: one fold of motif_hole + cluster_copy emit keys into per-gid sets."""
    gid = int(group_id)
    bucket = by_gid.setdefault(gid, set())
    for raw in motif_hole_keys or ():
        key = _as_key_tuple(raw)
        if key is not None:
            bucket.add(key)
    for raw in cluster_copy_keys or ():
        key = _as_key_tuple(raw)
        if key is not None:
            bucket.add(key)


def resolve_motif_keys(
    propose_stats: Mapping[str, Any] | None,
    *,
    densify: Mapping[str, Any] | None = None,
    gid: int | None = None,
) -> dict[int, set[tuple[float, float, float]]]:
    """Q196/Q217: one motif-key SoT — absorb densify/projected then ∪ fold emit.

    Layers (union, not exclusive early-return): projected propose_stats → densify
    fold → cluster_copy ∪ motif_hole from proposer_keys. Disjoint densify-only
    keys no-hit mix floor naturally; emit fold still populates (Q218).
    """
    stats = propose_stats or {}
    dens = densify if densify is not None else (stats.get("densify_stats") or {})
    out: dict[int, set[tuple[float, float, float]]] = {}

    def _absorb(src: Mapping[Any, Any] | None) -> int:
        if not isinstance(src, Mapping) or not src:
            return 0
        n = 0
        for g, keys in src.items():
            gi = int(g)
            if gid is not None and gi != int(gid):
                continue
            bucket = out.setdefault(gi, set())
            for raw in keys or ():
                key = _as_key_tuple(raw)
                if key is not None and key not in bucket:
                    bucket.add(key)
                    n += 1
        return n

    def _finalize() -> dict[int, set[tuple[float, float, float]]]:
        if gid is None:
            return out
        return {int(gid): out.get(int(gid), set())}

    _absorb(stats.get("motif_keys"))
    _absorb(stats.get("motif_graph_keys"))
    _absorb(dens.get("motif_keys") if isinstance(dens, Mapping) else None)

    pk = stats.get("proposer_keys") or (
        dens.get("proposer_keys") if isinstance(dens, Mapping) else None
    ) or {}
    cc = list(pk.get("cluster_copy") or ())
    holes_raw = dens.get("motif_hole_keys") if isinstance(dens, Mapping) else None
    holes = list(holes_raw or ())
    if gid is not None:
        fold_emit_motif_keys(
            out,
            group_id=int(gid),
            motif_hole_keys=holes,
            cluster_copy_keys=cc,
        )
        return _finalize()
    if cc or holes:
        gids = [int(g) for g in out] or [
            int(g) for g in (dens.get("motif_keys") or {})
        ] or [-1]
        for i, g in enumerate(gids):
            fold_emit_motif_keys(
                out,
                group_id=g,
                motif_hole_keys=holes if i == 0 else (),
                cluster_copy_keys=cc,
            )
    return _finalize()


def pose_in_motif_keys(
    gid: int,
    row: Any,
    motif_keys: Mapping[int, set[tuple[float, float, float]]] | None,
) -> bool:
    """Membership helper for credit / epoch (Q196)."""
    if not motif_keys:
        return False
    key = transform_row_key(row)
    return key in (motif_keys.get(int(gid)) or set())


def merge_motif_cohorts(*sources: Sequence | None) -> list[dict]:
    """Dedupe motif cohort dicts by (motif_id, leader_gid, leader_key)."""
    out: list[dict] = []
    seen: set[tuple[int, int, tuple[float, float, float] | None]] = set()
    for src in sources:
        for cohort in src or ():
            if not isinstance(cohort, dict):
                continue
            mid = int(cohort.get("motif_id", -1) or -1)
            lg = int(cohort.get("leader_gid", -1))
            lk = _as_key_tuple(cohort.get("leader_key"))
            sig = (mid, lg, lk)
            if sig in seen:
                continue
            seen.add(sig)
            out.append(dict(cohort))
    return out


def proposer_indices_on_graph(
    group_id: Sequence[int],
    transform: Sequence,
    proposer_keys: Mapping[str, set[tuple[float, float, float]]] | None,
    name: str,
) -> list[int]:
    """Graph indices whose projected key is in one proposer's emit set."""
    if not proposer_keys:
        return []
    keys = proposer_keys.get(name) or set()
    if not keys:
        return []
    out: list[int] = []
    for i, tr in enumerate(transform):
        if i >= len(group_id):
            break
        key = transform_row_key(np.asarray(tr, dtype=np.float64))
        if key in keys:
            out.append(int(i))
    return out


def cluster_copy_lock_pool(
    group_id: Sequence[int],
    transform: Sequence,
    proposer_keys: Mapping[str, set[tuple[float, float, float]]] | None,
    cohorts: Sequence | None,
    key_lookup: Mapping[tuple[int, tuple[float, float, float]], Any],
) -> list[int]:
    """Pattern-aware cc lock pool: emit survivors ∪ cohort member indices (one gate)."""
    pool = set(proposer_indices_on_graph(
        group_id, transform, proposer_keys, "cluster_copy",
    ))
    for cohort in cohorts or ():
        if not isinstance(cohort, dict):
            continue
        idxs, _miss = cohort_member_indices(cohort, key_lookup)
        pool.update(int(i) for i in idxs)
    return sorted(pool)


def cohort_keys_from_cohorts(
    cohorts: Sequence | None,
) -> dict[int, set[tuple[float, float, float]]]:
    """Union (gid, round-4 key) from motif cohort member_keys (one gate for boosts)."""
    out: dict[int, set[tuple[float, float, float]]] = {}
    for cohort in cohorts or ():
        if not isinstance(cohort, dict):
            continue
        for item in cohort.get("member_keys") or ():
            if not isinstance(item, (list, tuple)) or len(item) < 2:
                continue
            gid_m, key_m = int(item[0]), item[1]
            key_t = _as_key_tuple(key_m)
            if key_t is not None:
                out.setdefault(gid_m, set()).add(key_t)
    return out


def cohort_member_indices(
    cohort: Mapping[str, Any],
    key_lookup: Mapping[tuple[int, tuple[float, float, float]], Any],
    *,
    first_only: bool = True,
) -> tuple[list[int], int]:
    """Resolve cohort member_keys → graph indices; missing = not in lookup."""
    idxs: list[int] = []
    missing = 0
    for item in cohort.get("member_keys") or ():
        if not isinstance(item, (list, tuple)) or len(item) < 2:
            missing += 1
            continue
        gid_m, key_m = int(item[0]), item[1]
        key_t = _as_key_tuple(key_m)
        if key_t is None:
            missing += 1
            continue
        hit = key_lookup.get((gid_m, key_t))
        if hit is None:
            missing += 1
            continue
        if first_only:
            idxs.append(int(hit))
        else:
            hits = hit if isinstance(hit, (list, tuple)) else (int(hit),)
            if hits:
                idxs.append(int(hits[0]))
            else:
                missing += 1
    return idxs, missing


def boost_score_indices(
    scores: list[float],
    indices: Sequence[int],
    weight: float,
) -> int:
    """Add weight to scores at graph indices (Q262 packing-clear soft steer)."""
    if weight <= 0.0 or not scores or not indices:
        return 0
    n = 0
    for raw in indices:
        ix = int(raw)
        if 0 <= ix < len(scores):
            scores[ix] = float(scores[ix]) + float(weight)
            n += 1
    return n


VOID_EMIT_PROPOSERS = frozenset({
    "cluster_copy",
    "history_expand",
    "pocket_fit",
    "free_space_cloud",
    "raycasting",
    "side_pack",
})


def proposer_survivors_on_graph(
    group_id: Sequence[int],
    transform: Sequence,
    proposer_keys: Mapping[str, set[tuple[float, float, float]]] | None,
    names: frozenset[str] | None = None,
    *,
    key_to_verts: Mapping[
        tuple[int, tuple[float, float, float]], Sequence[int]
    ] | None = None,
) -> dict[int, set[tuple[float, float, float]]]:
    """Emit keys that actually exist on the current graph (pool → MIS SoT)."""
    if not proposer_keys or not group_id or not transform:
        return {}
    tagged: set[tuple[float, float, float]] = set()
    use = names if names is not None else VOID_EMIT_PROPOSERS
    for pname in use:
        tagged.update(proposer_keys.get(pname) or ())
    if not tagged:
        return {}
    out: dict[int, set[tuple[float, float, float]]] = {}
    if key_to_verts is not None:
        for (gid, key), _idxs in key_to_verts.items():
            if key in tagged:
                out.setdefault(int(gid), set()).add(key)
        return out
    for i, tr in enumerate(transform):
        if i >= len(group_id):
            break
        key = transform_row_key(np.asarray(tr, dtype=np.float64))
        if key in tagged:
            out.setdefault(int(group_id[i]), set()).add(key)
    return out


def count_proposer_on_selection(
    group_id: Sequence[int],
    transform: Sequence,
    selected: Sequence[int],
    proposer_keys: Mapping[str, set[tuple[float, float, float]]] | None,
    name: str,
    *,
    key_to_verts: Mapping[
        tuple[int, tuple[float, float, float]], Sequence[int]
    ] | None = None,
) -> tuple[int, int]:
    """Return (on_graph, in_selection) for one proposer's projected keys."""
    if not proposer_keys:
        return 0, 0
    keys = proposer_keys.get(name) or set()
    if not keys:
        return 0, 0
    on_graph = 0
    in_sel = 0
    sel_set = {int(i) for i in selected}
    if key_to_verts is not None:
        for (gid, key), idxs in key_to_verts.items():
            if key not in keys:
                continue
            n = len(idxs)
            on_graph += n
            in_sel += sum(1 for i in idxs if int(i) in sel_set)
        return on_graph, in_sel
    for i, tr in enumerate(transform):
        if i >= len(group_id):
            break
        key = transform_row_key(np.asarray(tr, dtype=np.float64))
        if key not in keys:
            continue
        on_graph += 1
        if int(i) in sel_set:
            in_sel += 1
    return on_graph, in_sel
