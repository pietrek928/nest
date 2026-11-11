"""Cheap MCTS cache key helpers (Q143/Q271) — leaf module, no pack/propose cycles."""

import zlib
from typing import Any, Sequence

from nest_graph.place_zone_util import place_zone_token


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
    pack_preset_id: int | None = None,
) -> tuple[str, int, int, int, int, int, int, int]:
    """Q143/S2a + Q369 + M2a + P5 + DgPreset + B: zone+motif+rule+compose+cohort+lock+preset+part_gid.

    P2: ``pack_preset_id`` overrides tip ``action.preset_id`` when Motif carve
    packs a densify preset while AMAF tip stays 0.
    """
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
    if pack_preset_id is not None:
        preset_id = int(pack_preset_id)
    return (
        place_zone_token(zone) or "",
        motif_id,
        rule_id,
        int(compose_sz),
        int(cohort_sig),
        int(lock_fp),
        int(preset_id),
        int(part_gid),
    )


def motif_cohort_sig(cohorts: Sequence) -> int:
    """Deterministic cohort fingerprint for cheap cache (M2a; no PYTHONHASHSEED)."""
    parts: list[str] = []
    for c in cohorts:
        if not isinstance(c, dict):
            continue
        # motif_id=0 / leader_gid=0 are valid — never ``x or -1``.
        _mid_raw = c.get("motif_id", -1)
        mid = -1 if _mid_raw is None else int(_mid_raw)
        _lead_raw = c.get("leader_gid", -1)
        leader = -1 if _lead_raw is None else int(_lead_raw)
        keys = sorted(str(mk) for mk in (c.get("member_keys") or ()))
        # L1: include gid-keyed spoke mids when present.
        spokes = c.get("spoke_mids") or ()
        spoke_s = ",".join(
            f"{int(a)}-{int(b)}:{int(m)}"
            for a, b, m in sorted(
                (
                    (int(t[0]), int(t[1]), int(t[2]))
                    for t in spokes
                    if isinstance(t, (tuple, list)) and len(t) >= 3
                )
            )
        )
        parts.append(f"{mid}:{leader}:{','.join(keys)}:{spoke_s}")
    if not parts:
        return 0
    return int(zlib.crc32("|".join(sorted(parts)).encode("utf-8")) & 0x7FFFFFFF)


__all__ = [
    "macro_motif_id",
    "cheap_lock_fingerprint",
    "cheap_pack_cache_key",
    "motif_cohort_sig",
]
