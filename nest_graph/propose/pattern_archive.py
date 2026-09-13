"""MotifBase ↔ ClusterPattern adapter (cross-iter SoT is C++ MotifBase; Q90/Q91).

Same-iter live extract still uses ``extract_cluster_patterns`` →
``merge_cluster_patterns`` in the propose pipeline. Cross-iter read/write
goes only through MotifBase (pairs). N-way clusters are never archived.
"""

from typing import Any, Mapping, Sequence

import numpy as np

from nest_graph.geometry import nfp_lite_pair_relative
from nest_graph.propose.placements_pattern import ClusterPattern, extract_cluster_patterns
from nest_graph.propose.void_selection import pose_key_to_verts, transform_row_key
from nest_graph.propose.motif_keys import (
    cohort_member_indices,
    fold_emit_motif_keys,
    pose_in_motif_keys,
    resolve_motif_keys,
)
from nest_graph.utils import compose_transforms, invert_transform

# Q278: leader world anchor at upsert time (MotifRecord has relative only).
_motif_ref_anchors: dict[int, tuple[float, float, float]] = {}


def note_motif_ref_anchors(
    motif_base: Any,
    gids: Sequence[int],
    transforms: Sequence,
) -> None:
    """Update world ref_transform cache for MotifBase leader gids."""
    if motif_base is None or int(motif_base.size()) <= 0:
        return
    gid_tf = {
        int(g): (
            float(t[0]),
            float(t[1]),
            float(t[2]),
        )
        for g, t in zip(gids, transforms, strict=False)
    }
    for mid in range(int(motif_base.size())):
        rec = motif_base.at(mid)
        ga = int(rec.gid_a)
        ref = gid_tf.get(ga)
        if ref is not None:
            _motif_ref_anchors[int(mid)] = ref


def note_motif_ref_anchors_from_nest(motif_base: Any, nest_state: Any) -> None:
    """Refresh leader world anchors from a packed NestState (Q324)."""
    if nest_state is None:
        return
    indices = getattr(nest_state, "selected_indices", None) or ()
    if not indices:
        return
    gids: list[int] = []
    transforms: list = []
    group_id = getattr(nest_state, "group_id", None) or []
    transform = getattr(nest_state, "transform", None) or []
    for i in indices:
        ii = int(i)
        if ii < 0 or ii >= len(group_id) or ii >= len(transform):
            continue
        gids.append(int(group_id[ii]))
        transforms.append(transform[ii])
    if gids:
        note_motif_ref_anchors(motif_base, gids, transforms)


def archive_ref_origin_count(patterns: Sequence[ClusterPattern]) -> int:
    """Count archived patterns still at default origin ref_transform (A1 telem)."""
    n = 0
    for pat in patterns:
        rt = getattr(pat, "ref_transform", None)
        if rt is None:
            n += 1
            continue
        if (
            float(rt[0]) == 0.0
            and float(rt[1]) == 0.0
            and float(rt[2]) == 0.0
        ):
            n += 1
    return n


def record_archive_ref_telem(
    patterns: Sequence[ClusterPattern],
    telem: dict | None,
) -> None:
    """A1 gate telem: archive_ref_origin_n on inject list."""
    if telem is None or not patterns:
        return
    origin_n = archive_ref_origin_count(patterns)
    telem["archive_ref_origin_n"] = int(telem.get("archive_ref_origin_n", 0)) + int(
        origin_n
    )
    telem["archive_ref_nonzero_n"] = int(telem.get("archive_ref_nonzero_n", 0)) + int(
        len(patterns) - origin_n
    )


def record_to_cluster_pattern(
    rec: Any,
    *,
    motif_id: int = -1,
    ref_transform: tuple[float, float, float] | None = None,
) -> ClusterPattern:
    """One MotifRecord → pair ClusterPattern."""
    mid = int(motif_id)
    ref = ref_transform
    if ref is None and mid >= 0:
        ref = _motif_ref_anchors.get(mid)
    if ref is None:
        ref = (0.0, 0.0, 0.0)
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
        ref_transform=(
            float(ref[0]),
            float(ref[1]),
            float(ref[2]),
        ),
        motif_id=mid,
    )


def motif_to_cluster_patterns(motif_base: Any, action: Any) -> list[ClusterPattern]:
    """PLACE_MOTIF action → ClusterPattern list (single MotifBase record)."""
    if int(action.motif_id) < 0 or int(action.motif_id) >= int(motif_base.size()):
        return []
    mid = int(action.motif_id)
    return [record_to_cluster_pattern(motif_base.at(mid), motif_id=mid)]


def _pattern_full_clique_hit(
    pat: ClusterPattern,
    group_id: Sequence[int],
    transform: Sequence,
    key_map: dict,
) -> bool:
    """True when all pattern members match world poses on the graph."""
    members = tuple(getattr(pat, "members", ()) or ())
    if len(members) < 2:
        return False
    gid_a, _t_a = members[0]
    for gid, t in zip(group_id, transform, strict=False):
        if int(gid) != int(gid_a):
            continue
        world_a = (float(t[0]), float(t[1]), float(t[2]))
        key_a = transform_row_key(world_a)
        if (int(gid_a), key_a) not in key_map:
            continue
        ok = True
        for gid_m, t_m in members[1:]:
            rel = (float(t_m[0]), float(t_m[1]), float(t_m[2]))
            world_m = compose_transforms(world_a, rel)
            key_m = transform_row_key(world_m)
            if (int(gid_m), key_m) not in key_map:
                ok = False
                break
        if ok:
            return True
    return False


def _cohort_member_keys_for_leader(
    members: Sequence,
    world_a: tuple[float, float, float],
    key_map: Mapping[tuple[int, tuple[float, float, float]], Any],
) -> list[tuple[int, tuple[float, float, float]]]:
    """Q387: stamp every in-graph member under a leader pose (full cohort)."""
    key_a = transform_row_key(world_a)
    gid_a = int(members[0][0])
    out: list[tuple[int, tuple[float, float, float]]] = [(gid_a, key_a)]
    for gid_m, t_m in members[1:]:
        rel = (float(t_m[0]), float(t_m[1]), float(t_m[2]))
        world_m = compose_transforms(world_a, rel)
        key_m = transform_row_key(world_m)
        if (int(gid_m), key_m) not in key_map:
            continue
        out.append((int(gid_m), key_m))
    return out


def _ref_transform_pair_cohort(
    pat: Any,
    idx: int,
    key_map: Mapping[tuple[int, tuple[float, float, float]], Any],
) -> dict | None:
    """Q340/Q387: ref_transform cohort with full in-graph member_keys."""
    members = tuple(getattr(pat, "members", ()) or ())
    if len(members) < 2:
        return None
    rt_raw = getattr(pat, "ref_transform", None)
    if rt_raw is None:
        return None
    rt = np.asarray(rt_raw, dtype=np.float64).reshape(3)
    world_a = (float(rt[0]), float(rt[1]), float(rt[2]))
    member_keys = _cohort_member_keys_for_leader(members, world_a, key_map)
    if len(member_keys) < 2:
        return None
    mid = int(getattr(pat, "motif_id", -1))
    if mid < 0:
        mid = int(idx)
    return {
        "leader_gid": int(member_keys[0][0]),
        "leader_key": member_keys[0][1],
        "member_keys": member_keys,
        "motif_id": mid,
    }


def motif_graph_hits(
    patterns: Sequence[ClusterPattern],
    group_id: Sequence[int],
    transform: Sequence,
    telem: dict | None = None,
) -> tuple[dict[int, set[tuple[float, float, float]]], list[dict], int]:
    """Match Motif-local ClusterPatterns onto graph world poses (cheap inject).

    Returns (motif_keys, motif_cohorts, n_hits). Sequential accept and score
    boost already consume those propose_stats fields.
    """
    key_map = pose_key_to_verts(group_id, transform)
    motif_keys: dict[int, set[tuple[float, float, float]]] = {}
    cohorts: list[dict] = []
    cohort_sigs: set[tuple[int, tuple[float, float, float]]] = set()
    n_hits = 0
    clique_pairs_n = 0
    clique_full_hits = 0
    leader_hit_n = 0
    follower_miss_n = 0
    for idx, pat in enumerate(patterns):
        members = tuple(getattr(pat, "members", ()) or ())
        if len(members) < 2:
            continue
        clique_pairs_n += len(members) - 1
        if _pattern_full_clique_hit(pat, group_id, transform, key_map):
            clique_full_hits += 1
        mid = int(getattr(pat, "motif_id", -1))
        if mid < 0:
            mid = int(idx)
        gid_a, _t_a = members[0]
        for i, (gid, t) in enumerate(zip(group_id, transform, strict=False)):
            if int(gid) != int(gid_a):
                continue
            world_a = (float(t[0]), float(t[1]), float(t[2]))
            key_a = transform_row_key(world_a)
            leader_in = (int(gid_a), key_a) in key_map
            if leader_in:
                leader_hit_n += 1
            member_keys = _cohort_member_keys_for_leader(members, world_a, key_map)
            if len(member_keys) < 2:
                if leader_in:
                    follower_miss_n += 1
                continue
            for gid_m, key_m in member_keys:
                motif_keys.setdefault(int(gid_m), set()).add(key_m)
            cohorts.append({
                "leader_gid": int(gid_a),
                "leader_key": key_a,
                "member_keys": member_keys,
                "motif_id": mid,
            })
            cohort_sigs.add((mid, key_a))
            n_hits += 1
        ref_cohort = _ref_transform_pair_cohort(pat, idx, key_map)
        if ref_cohort is not None:
            sig = (
                int(ref_cohort["motif_id"]),
                tuple(ref_cohort["leader_key"]),
            )
            if sig not in cohort_sigs:
                lk = ref_cohort["leader_key"]
                gid_a = int(ref_cohort["leader_gid"])
                motif_keys.setdefault(gid_a, set()).add(lk)
                for gid_m, key_m in ref_cohort["member_keys"]:
                    mk = transform_row_key(key_m) if not isinstance(key_m, tuple) else key_m
                    motif_keys.setdefault(int(gid_m), set()).add(mk)
                cohorts.append(ref_cohort)
                cohort_sigs.add(sig)
                n_hits += 1
    if telem is not None:
        telem["motif_clique_pairs_n"] = int(
            telem.get("motif_clique_pairs_n", 0)
        ) + int(clique_pairs_n)
        telem["motif_clique_full_hits"] = int(
            telem.get("motif_clique_full_hits", 0)
        ) + int(clique_full_hits)
        telem["motif_graph_leader_hit_n"] = int(
            telem.get("motif_graph_leader_hit_n", 0)
        ) + int(leader_hit_n)
        telem["motif_graph_follower_miss_n"] = int(
            telem.get("motif_graph_follower_miss_n", 0)
        ) + int(follower_miss_n)
        if leader_hit_n > 0 or follower_miss_n > 0:
            telem["motif_graph_hit_pass_n"] = int(
                telem.get("motif_graph_hit_pass_n", 0)
            ) + 1
    return motif_keys, cohorts, n_hits


def merge_motif_hits(
    propose_stats: dict,
    motif_keys: dict[int, set[tuple[float, float, float]]] | None,
    cohorts: Sequence[dict] | None,
) -> None:
    """Cheap Motif inject: union motif_graph_keys / append cohorts (Q143/Q200).

    Writes ``motif_graph_keys`` only — never ``motif_keys`` (emit SoT / Q196).
    """
    if motif_keys:
        merged = dict(propose_stats.get("motif_graph_keys") or {})
        for gid, keys in motif_keys.items():
            merged.setdefault(int(gid), set()).update(keys or ())
        propose_stats["motif_graph_keys"] = merged
    if cohorts:
        existing = list(propose_stats.get("motif_cohorts") or [])
        seen: set[tuple[int, tuple[float, float, float] | None]] = set()
        for c in existing:
            if isinstance(c, dict):
                lk = c.get("leader_key")
                key_t = tuple(lk) if isinstance(lk, tuple) else None
                seen.add((int(c.get("motif_id", -1) or -1), key_t))
        added = 0
        for c in cohorts:
            lk = c.get("leader_key")
            key_t = tuple(lk) if isinstance(lk, tuple) else None
            sig = (int(c.get("motif_id", -1) or -1), key_t)
            if sig in seen:
                continue
            seen.add(sig)
            existing.append(c)
            added += 1
        if added:
            propose_stats["motif_cohorts"] = existing
            propose_stats["motif_cohort_dedup_n"] = int(
                propose_stats.get("motif_cohort_dedup_n", 0)
            ) + int(added)


def inject_cohorts_from_patterns(
    patterns: Sequence,
    group_id: Sequence[int],
    transform: Sequence,
    propose_stats: dict,
    *,
    telem: dict | None = None,
) -> int:
    """Archive → graph-hit cohort inject (Q255). Pre-bind outer + cheap compose only."""
    if not patterns:
        return 0
    sink = telem if telem is not None else propose_stats
    keys, cohorts, n = motif_graph_hits(
        patterns, group_id, transform, telem=sink,
    )
    merge_motif_hits(propose_stats, keys, cohorts)
    n_i = int(n)
    propose_stats["motif_graph_hit_n"] = int(
        propose_stats.get("motif_graph_hit_n", 0)
    ) + n_i
    return n_i


def _pair_edges_from_ids(
    motif_base: Any,
    mids: Sequence[int],
) -> list[tuple[int, int, int, tuple[float, float, float]]]:
    """(mid, gid_a, gid_b, rel_xyz) for valid MotifBase pair records."""
    edges: list[tuple[int, int, int, tuple[float, float, float]]] = []
    n = int(motif_base.size())
    for mid in mids:
        mid_i = int(mid)
        if mid_i < 0 or mid_i >= n:
            continue
        rec = motif_base.at(mid_i)
        ga, gb = int(rec.gid_a), int(rec.gid_b)
        if ga < 0 or gb < 0 or ga == gb:
            continue
        rel = (
            float(rec.relative.x),
            float(rec.relative.y),
            float(rec.relative.a),
        )
        edges.append((mid_i, ga, gb, rel))
    return edges


def stitch_leader_star_patterns(
    motif_base: Any,
    mids: Sequence[int],
    *,
    telem: dict | None = None,
) -> tuple[list[ClusterPattern], set[int]]:
    """Q237: regroup MotifBase pairs that share a hub into k-member patterns.

    Returns (stitched patterns, motif ids consumed by a stitch). Unconsumed
    pair ids should still emit via ``record_to_cluster_pattern``.
    """
    edges = _pair_edges_from_ids(motif_base, mids)
    if len(edges) < 2:
        return [], set()
    degree: dict[int, int] = {}
    for _mid, ga, gb, _rel in edges:
        degree[ga] = int(degree.get(ga, 0)) + 1
        degree[gb] = int(degree.get(gb, 0)) + 1
    hubs = {g for g, d in degree.items() if int(d) >= 2}
    if not hubs:
        return [], set()
    # Prefer higher-degree hubs; stable by gid.
    hub_order = sorted(hubs, key=lambda g: (-int(degree[g]), int(g)))
    used_mids: set[int] = set()
    stitched: list[ClusterPattern] = []
    for hub in hub_order:
        members: list[tuple[int, tuple[float, float, float]]] = [
            (int(hub), (0.0, 0.0, 0.0)),
        ]
        seen_follower: set[int] = {int(hub)}
        star_mids: list[int] = []
        best_mid = -1
        best_gci = -1.0
        for mid_i, ga, gb, rel in edges:
            if mid_i in used_mids:
                continue
            if int(ga) == int(hub):
                follower, fol_rel = int(gb), rel
            elif int(gb) == int(hub):
                follower, fol_rel = int(ga), invert_transform(rel)
            else:
                continue
            if follower in seen_follower:
                continue
            seen_follower.add(follower)
            members.append((follower, fol_rel))
            star_mids.append(int(mid_i))
            rec = motif_base.at(int(mid_i))
            gci = float(getattr(rec, "gci", 0.0) or 0.0)
            if gci > best_gci:
                best_gci = gci
                best_mid = int(mid_i)
        if len(members) < 3:
            continue
        for mid_i in star_mids:
            used_mids.add(int(mid_i))
        mid_tag = int(best_mid) if best_mid >= 0 else int(star_mids[0])
        ref = _motif_ref_anchors.get(mid_tag)
        if ref is None:
            ref = (0.0, 0.0, 0.0)
        stitched.append(
            ClusterPattern(
                members=tuple(members),
                part_count=len(members),
                ref_transform=(float(ref[0]), float(ref[1]), float(ref[2])),
                motif_id=mid_tag,
            )
        )
    if telem is not None and stitched:
        telem["star_stitch_n"] = int(telem.get("star_stitch_n", 0) or 0) + int(
            len(stitched)
        )
        telem["star_stitch_members"] = int(
            telem.get("star_stitch_members", 0) or 0
        ) + int(sum(int(p.part_count) for p in stitched))
    return stitched, used_mids


def patterns_from_motif_base(
    motif_base: Any,
    *,
    max_keep: int = 4,
    prefer_motif_id: int = -1,
    telem: dict | None = None,
) -> list[ClusterPattern]:
    """Cross-iter inject list via MotifBase.list_for_inject (Q91) + Q237 stitch."""
    if motif_base is None or int(motif_base.size()) <= 0:
        return []
    cand_ids: list[int] = []
    seen: set[int] = set()
    if prefer_motif_id >= 0 and prefer_motif_id < int(motif_base.size()):
        cand_ids.append(int(prefer_motif_id))
        seen.add(int(prefer_motif_id))
    # Pull a wider window so star hubs can reconstruct before max_keep trim.
    inject_cap = max(int(max_keep) * 3, int(max_keep), 1)
    for mid in motif_base.list_for_inject(int(inject_cap)):
        mid_i = int(mid)
        if mid_i in seen:
            continue
        seen.add(mid_i)
        cand_ids.append(mid_i)
    if not cand_ids and int(motif_base.size()) > 0:
        ranked = sorted(
            range(int(motif_base.size())),
            key=lambda i: (
                int(getattr(motif_base.at(i), "accept_count", 0) or 0),
                float(getattr(motif_base.at(i), "gci", 0.0) or 0.0),
            ),
            reverse=True,
        )
        for mid_i in ranked:
            if int(mid_i) in seen:
                continue
            rec = motif_base.at(int(mid_i))
            if int(getattr(rec, "accept_count", 0) or 0) <= 0:
                if int(getattr(rec, "ttl_remaining", 0) or 0) < 0:
                    continue
            seen.add(int(mid_i))
            cand_ids.append(int(mid_i))
            if len(cand_ids) >= inject_cap:
                break
    stitched, used = stitch_leader_star_patterns(
        motif_base, cand_ids, telem=telem,
    )
    out: list[ClusterPattern] = list(stitched)
    for mid_i in cand_ids:
        if int(mid_i) in used:
            continue
        out.append(
            record_to_cluster_pattern(
                motif_base.at(int(mid_i)),
                motif_id=int(mid_i),
            )
        )
        if max_keep > 0 and len(out) >= int(max_keep):
            break
    if max_keep > 0 and len(out) > int(max_keep):
        out = out[: int(max_keep)]
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
        telem=telem,
    )
    if pats and telem is not None:
        record_archive_ref_telem(pats, telem)
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
    "fold_emit_motif_keys",
    "inject_cohorts_from_patterns",
    "motif_graph_hits",
    "merge_motif_hits",
    "motif_patterns_for_inject",
    "motif_to_cluster_patterns",
    "note_motif_hollow_miss",
    "note_motif_ref_anchors",
    "note_motif_ref_anchors_from_nest",
    "archive_ref_origin_count",
    "record_archive_ref_telem",
    "patterns_from_motif_base",
    "polish_patterns_at_inject",
    "pose_in_motif_keys",
    "record_to_cluster_pattern",
    "resolve_motif_keys",
    "stitch_leader_star_patterns",
]
