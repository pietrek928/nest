"""MotifBase ↔ ClusterPattern adapter (cross-iter SoT is C++ MotifBase; Q90/Q91).

Same-iter live extract still uses ``extract_cluster_patterns`` →
``merge_cluster_patterns`` in the propose pipeline. Cross-iter read/write
goes only through MotifBase (pairs). N-way clusters are never archived.
"""

from typing import Any, Mapping, Sequence

from nest_graph.geometry import nfp_lite_pair_relative
from nest_graph.propose.placements_pattern import ClusterPattern, extract_cluster_patterns
from nest_graph.propose.void_selection import pose_key_to_verts, transform_row_key
from nest_graph.utils import compose_transforms


def record_to_cluster_pattern(rec: Any) -> ClusterPattern:
    """One MotifRecord → pair ClusterPattern."""
    members = (
        (int(rec.gid_a), (0.0, 0.0, 0.0)),
        (
            int(rec.gid_b),
            (float(rec.relative.x), float(rec.relative.y), float(rec.relative.a)),
        ),
    )
    return ClusterPattern(
        members=members,
        part_count=2,
        ref_transform=(0.0, 0.0, 0.0),
    )


def motif_to_cluster_patterns(motif_base: Any, action: Any) -> list[ClusterPattern]:
    """PLACE_MOTIF action → ClusterPattern list (single MotifBase record)."""
    if int(action.motif_id) < 0 or int(action.motif_id) >= int(motif_base.size()):
        return []
    return [record_to_cluster_pattern(motif_base.at(int(action.motif_id)))]


def motif_graph_hits(
    patterns: Sequence[ClusterPattern],
    group_id: Sequence[int],
    transform: Sequence,
) -> tuple[dict[int, set[tuple[float, float, float]]], list[dict], int]:
    """Match Motif-local ClusterPatterns onto graph world poses (cheap inject).

    Returns (motif_keys, motif_cohorts, n_hits). Sequential accept and score
    boost already consume those propose_stats fields.
    """
    key_map = pose_key_to_verts(group_id, transform)
    motif_keys: dict[int, set[tuple[float, float, float]]] = {}
    cohorts: list[dict] = []
    n_hits = 0
    for pat in patterns:
        if len(getattr(pat, "members", ()) or ()) < 2:
            continue
        gid_a, _t_a = pat.members[0]
        gid_b, t_b = pat.members[1]
        rel_b = (float(t_b[0]), float(t_b[1]), float(t_b[2]))
        for i, (gid, t) in enumerate(zip(group_id, transform)):
            if int(gid) != int(gid_a):
                continue
            world_a = (float(t[0]), float(t[1]), float(t[2]))
            world_b = compose_transforms(world_a, rel_b)
            key_b = transform_row_key(world_b)
            if (int(gid_b), key_b) not in key_map:
                continue
            key_a = transform_row_key(world_a)
            motif_keys.setdefault(int(gid_a), set()).add(key_a)
            motif_keys.setdefault(int(gid_b), set()).add(key_b)
            cohorts.append({
                "leader_gid": int(gid_a),
                "leader_key": key_a,
                "member_keys": [
                    (int(gid_a), key_a),
                    (int(gid_b), key_b),
                ],
            })
            n_hits += 1
    return motif_keys, cohorts, n_hits


def merge_motif_hits(
    propose_stats: dict,
    motif_keys: dict[int, set[tuple[float, float, float]]] | None,
    cohorts: Sequence[dict] | None,
) -> None:
    """Cheap Motif inject: union motif_keys / append cohorts (Q143)."""
    if motif_keys:
        merged = dict(propose_stats.get("motif_keys") or {})
        for gid, keys in motif_keys.items():
            merged.setdefault(int(gid), set()).update(keys or ())
        propose_stats["motif_keys"] = merged
    if cohorts:
        propose_stats["motif_cohorts"] = list(
            propose_stats.get("motif_cohorts") or []
        ) + list(cohorts)


def patterns_from_motif_base(
    motif_base: Any,
    *,
    max_keep: int = 4,
    prefer_motif_id: int = -1,
) -> list[ClusterPattern]:
    """Cross-iter inject list via MotifBase.list_for_inject (Q91)."""
    if motif_base is None or int(motif_base.size()) <= 0:
        return []
    out: list[ClusterPattern] = []
    seen: set[int] = set()
    if prefer_motif_id >= 0 and prefer_motif_id < int(motif_base.size()):
        out.append(record_to_cluster_pattern(motif_base.at(int(prefer_motif_id))))
        seen.add(int(prefer_motif_id))
    for mid in motif_base.list_for_inject(int(max_keep)):
        mid_i = int(mid)
        if mid_i in seen:
            continue
        seen.add(mid_i)
        out.append(record_to_cluster_pattern(motif_base.at(mid_i)))
        if max_keep > 0 and len(out) >= int(max_keep):
            break
    return out


def motif_patterns_for_inject(
    motif_base: Any,
    *,
    max_keep: int = 4,
    prefer_motif_id: int = -1,
    part_bases: Mapping[int, Any] | None = None,
    min_dist: float = 0.0,
    telem: dict | None = None,
    polish: bool = True,
) -> list[ClusterPattern]:
    """One Motif inject SoT: list_for_inject (+ optional NFP-lite polish).

    Outer propose uses polish=True. Cheap expand may call with polish=False for
    telem-only pattern counts — patterns are not re-stamped on cache compose.
    """
    pats = patterns_from_motif_base(
        motif_base,
        max_keep=int(max_keep),
        prefer_motif_id=int(prefer_motif_id),
    )
    if not pats or not polish or part_bases is None:
        return pats
    return polish_patterns_at_inject(
        pats,
        part_bases,
        min_dist=float(min_dist),
        telem=telem,
    )


def polish_patterns_at_inject(
    patterns: Sequence[ClusterPattern],
    part_bases: Mapping[int, Any],
    *,
    min_dist: float,
    telem: dict | None = None,
) -> list[ClusterPattern]:
    """Np: NFP-lite polish Motif relatives at inject (never MotifBase upsert).

    Polishes follower SE2 in the Motif frame (leader at its Motif-local pose).
    On failure or missing bases, keeps the unpolished relative.
    """
    if not patterns:
        return []
    out: list[ClusterPattern] = []
    n_ok = 0
    n_keep = 0
    for pat in patterns:
        if len(pat.members) < 2:
            out.append(pat)
            continue
        gid_a, t_a = pat.members[0]
        gid_b, t_b = pat.members[1]
        base_a = part_bases.get(int(gid_a))
        base_b = part_bases.get(int(gid_b))
        if base_a is None or base_b is None:
            out.append(pat)
            n_keep += 1
            continue
        try:
            t_a0 = (float(t_a[0]), float(t_a[1]), float(t_a[2]))
            t_b0 = (float(t_b[0]), float(t_b[1]), float(t_b[2]))
            t_b_world = (
                compose_transforms(t_a0, t_b0) if t_a0 != (0.0, 0.0, 0.0) else t_b0
            )
            anchor = base_a.apply_transform(
                float(t_a0[0]), float(t_a0[1]), float(t_a0[2]),
            )
            new_rel = nfp_lite_pair_relative(
                base_b,
                t_a0,
                t_b_world,
                anchor,
                min_dist=float(min_dist),
            )
            old_len = (t_b0[0] ** 2 + t_b0[1] ** 2) ** 0.5
            new_len = (float(new_rel[0]) ** 2 + float(new_rel[1]) ** 2) ** 0.5
            if new_len > old_len * 3.0 + max(float(min_dist), 1.0) * 2.0:
                out.append(pat)
                n_keep += 1
                continue
            polished = ClusterPattern(
                members=(
                    (int(gid_a), t_a0),
                    (
                        int(gid_b),
                        (float(new_rel[0]), float(new_rel[1]), float(new_rel[2])),
                    ),
                )
                + tuple(pat.members[2:]),
                part_count=int(pat.part_count),
                ref_transform=pat.ref_transform,
            )
            out.append(polished)
            n_ok += 1
        except Exception:
            out.append(pat)
            n_keep += 1
    if telem is not None:
        telem["nfp_lite_ok"] = int(telem.get("nfp_lite_ok", 0)) + int(n_ok)
        telem["nfp_lite_keep"] = int(telem.get("nfp_lite_keep", 0)) + int(n_keep)
    return out


def age_motif_library(motif_base: Any, step: int = 1) -> int:
    """Decrement MotifBase TTL; returns drop count."""
    if motif_base is None:
        return 0
    return int(motif_base.age(int(step)))


def note_motif_hollow_miss(motif_base: Any, motif_id: int) -> bool:
    """Q113 hollow Motif: ban unproven / floor proven via MotifBase.note_hollow_miss."""
    if motif_base is None or int(motif_id) < 0:
        return False
    if int(motif_id) >= int(motif_base.size()):
        return False
    return bool(motif_base.note_hollow_miss(int(motif_id)))


__all__ = [
    "age_motif_library",
    "extract_cluster_patterns",
    "motif_graph_hits",
    "merge_motif_hits",
    "motif_patterns_for_inject",
    "motif_to_cluster_patterns",
    "note_motif_hollow_miss",
    "patterns_from_motif_base",
    "polish_patterns_at_inject",
    "record_to_cluster_pattern",
]
