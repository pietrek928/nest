"""Motif key SoT helpers (Q196–Q197): fold emit keys + one resolve prefer path.

Leaf module: no imports from pattern_archive / void_selection (avoids cycles).
"""

from typing import Any, Mapping, Sequence

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
