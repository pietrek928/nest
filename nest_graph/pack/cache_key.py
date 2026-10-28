"""Cheap MCTS cache key helpers (Q143/Q271) — leaf module, no pack/propose cycles."""

import zlib
from typing import Any, Sequence


def macro_motif_id(action: Any | None) -> int:
    """Motif id from MacroAction; motif_id=0 is valid (never use ``x or -1``)."""
    if action is None:
        return -1
    raw = getattr(action, "motif_id", -1)
    if raw is None:
        return -1
    return int(raw)


def cheap_lock_fingerprint(motif_locked: Sequence | None) -> int:
    """Stable fingerprint of sorted lock indices (Q271 identity beyond compose_sz)."""
    if not motif_locked:
        return 0
    idxs = tuple(sorted(int(i) for i in motif_locked if int(i) >= 0))
    if not idxs:
        return 0
    return int(zlib.crc32(",".join(str(i) for i in idxs).encode("utf-8")) & 0x7FFFFFFF)


def cheap_pack_cache_key(
    zone,
    action,
    *,
    compose_sz: int = 0,
    cohort_sig: int = 0,
    lock_fp: int = 0,
) -> tuple[str, int, int, int, int, int, int, int]:
    """Q143/S2a + Q369 + M2a + P5 + DgPreset + B: zone+motif+rule+compose+cohort+lock+preset+part_gid."""
    motif_id = -1
    rule_id = 0
    preset_id = 0
    part_gid = 0
    if action is not None:
        mid = macro_motif_id(action)
        if mid >= 0:
            motif_id = mid
        rid = int(getattr(action, "rule_id", 0) or 0)
        rule_id = rid if rid >= 0 else 0
        preset_id = int(getattr(action, "preset_id", 0) or 0)
        raw_gid = getattr(action, "part_gid", 0)
        part_gid = int(raw_gid) if raw_gid is not None else 0
        if part_gid < 0:
            part_gid = 0
        if cohort_sig == 0:
            cohort_sig = int(getattr(action, "cohort_sig", 0) or 0)
    return (
        str(zone or ""),
        motif_id,
        rule_id,
        int(compose_sz),
        int(cohort_sig),
        int(lock_fp),
        int(preset_id),
        int(part_gid),
    )


__all__ = [
    "macro_motif_id",
    "cheap_lock_fingerprint",
    "cheap_pack_cache_key",
]
