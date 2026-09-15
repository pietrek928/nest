"""Cheap MCTS cache key helpers (Q143/Q271) — leaf module, no pack/propose cycles."""

import zlib
from typing import Sequence


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
) -> tuple[str, int, int, int, int, int]:
    """Q143/S2a + Q369 + M2a + P5: (zone, motif_id, rule_id, compose_sz, cohort_sig, lock_fp)."""
    motif_id = -1
    rule_id = 0
    if action is not None:
        if int(getattr(action, "motif_id", -1) or -1) >= 0:
            motif_id = int(action.motif_id)
        rid = int(getattr(action, "rule_id", 0) or 0)
        rule_id = rid if rid >= 0 else 0
        if cohort_sig == 0:
            cohort_sig = int(getattr(action, "cohort_sig", 0) or 0)
    return (
        str(zone or ""),
        motif_id,
        rule_id,
        int(compose_sz),
        int(cohort_sig),
        int(lock_fp),
    )


__all__ = [
    "cheap_lock_fingerprint",
    "cheap_pack_cache_key",
]
