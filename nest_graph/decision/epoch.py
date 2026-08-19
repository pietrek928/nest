"""Bind a PoseGraph epoch onto DecisionGraph (copy-in + Kind + Attach)."""

from typing import Sequence

from nest_graph.elem_graph import DecisionGraph, MacroRegion
from nest_graph.propose.void_selection import pose_key_to_verts, transform_row_key

_UNTAGGED = 255


def _zone_kind(zone: str | None) -> int | None:
    z = str(zone or "")
    if z in ("cluster_edge", "empty_border"):
        return int(getattr(MacroRegion.Rim, "value", 0))
    if z in ("void_seek", "void"):
        return int(getattr(MacroRegion.Void, "value", 1))
    if z in ("interior_pocket",):
        return int(getattr(MacroRegion.Sheet, "value", 2))
    return None


def _gid_key_union(src: dict | None) -> dict[int, set[tuple[float, float, float]]]:
    out: dict[int, set[tuple[float, float, float]]] = {}
    if not src:
        return out
    for gid, keys in src.items():
        out.setdefault(int(gid), set()).update(keys or ())
    return out


def _proposer_key_union(proposer_keys: dict | None) -> set[tuple[float, float, float]]:
    out: set[tuple[float, float, float]] = set()
    if not proposer_keys:
        return out
    for keys in proposer_keys.values():
        out.update(keys or ())
    return out


def bind_epoch(
    dg: DecisionGraph | None,
    graph,
    propose_stats: dict | None,
    group_id: Sequence[int] | None,
    transform: Sequence | None,
) -> None:
    """Copy-in poses, stamp MemberOf, Attach from attract pairs. Skip if no dg."""
    if dg is None or graph is None:
        return
    dg.replace_poses(graph)
    stats = propose_stats if propose_stats is not None else {}
    n = int(len(group_id)) if group_id is not None else 0
    if transform is None or n == 0 or n != len(transform):
        stats["attach_n"] = int(dg.attach_n())
        stats["kind_n"] = int(dg.kind_tagged_n())
        stats["mutex_n"] = int(dg.mutex_n())
        stats.setdefault("kind_keys", {})
        return

    motif_keys = _gid_key_union(stats.get("motif_keys"))
    pocket_keys = _gid_key_union(stats.get("pocket_keys"))
    proposal_keys = _gid_key_union(stats.get("proposal_keys"))
    sniper_keys = _gid_key_union(stats.get("sniper_keys"))
    densify = stats.get("densify_stats") or {}
    if not sniper_keys:
        sniper_keys = _gid_key_union(densify.get("sniper_keys"))
    proposer_any = _proposer_key_union(stats.get("proposer_keys"))
    if not proposer_any:
        proposer_any = _proposer_key_union(densify.get("proposer_keys"))
    epoch_keys = _gid_key_union(stats.get("epoch_keys"))
    zone_kind = _zone_kind(stats.get("mcts_zone"))
    densify_zone = _zone_kind(densify.get("cascade_zone") or densify.get("mcts_zone"))

    kinds: list[int] = [_UNTAGGED] * n
    kind_keys: dict[int, set[tuple[float, float, float]]] = {}
    for i, (gid, tr) in enumerate(zip(group_id, transform, strict=False)):
        gi = int(gid)
        key = transform_row_key(tr)
        tagged = (
            key in motif_keys.get(gi, ())
            or key in pocket_keys.get(gi, ())
            or key in proposal_keys.get(gi, ())
            or key in sniper_keys.get(gi, ())
            or key in proposer_any
            or key in epoch_keys.get(gi, ())
        )
        if key in motif_keys.get(gi, ()):
            kinds[i] = int(getattr(MacroRegion.Motif, "value", 3))
        elif not tagged:
            kinds[i] = _UNTAGGED
        elif zone_kind is not None:
            kinds[i] = int(zone_kind)
        elif densify_zone is not None:
            kinds[i] = int(densify_zone)
        elif key in pocket_keys.get(gi, ()):
            kinds[i] = int(getattr(MacroRegion.Sheet, "value", 2))
        else:
            kinds[i] = int(getattr(MacroRegion.Rim, "value", 0))
        if kinds[i] != _UNTAGGED:
            kind_keys.setdefault(gi, set()).add(key)
    dg.set_pose_kinds(kinds)

    attract = getattr(graph, "attract", None)
    if attract is not None:
        seen: set[tuple[int, int]] = set()
        for i, edges in enumerate(attract):
            for e in edges:
                j = int(getattr(e, "target", e))
                a, b = (i, j) if i < j else (j, i)
                if a == b or (a, b) in seen:
                    continue
                seen.add((a, b))
                dg.add_attach(a, b)

    verts = pose_key_to_verts(group_id, transform)
    cohorts = stats.get("motif_cohorts") or densify.get("motif_cohorts") or ()
    for cohort in cohorts:
        if not isinstance(cohort, dict):
            continue
        mid = int(cohort.get("motif_id", -1) or -1)
        members = list(cohort.get("member_keys") or ())
        idxs: list[int] = []
        for item in members:
            if item is None or len(item) < 2:
                continue
            gid_m, key_m = int(item[0]), item[1]
            key_t = tuple(key_m) if not isinstance(key_m, tuple) else key_m
            hits = verts.get((gid_m, key_t)) or ()
            if hits:
                idxs.append(int(hits[0]))
        if len(idxs) >= 2:
            dg.add_motif_join(mid, idxs[0], idxs[1])

    stats["kind_keys"] = kind_keys
    stats["attach_n"] = int(dg.attach_n())
    stats["kind_n"] = int(dg.kind_tagged_n())
    stats["mutex_n"] = int(dg.mutex_n())
    if propose_stats is not None:
        propose_stats.update(stats)


def materialize_selection(dg, selected: Sequence[int], propose_stats: dict | None = None) -> dict:
    """Flag Attach/MotifJoin whose members survived MWIS (Q154)."""
    out = {
        "materialized_attach": 0,
        "materialized_motif": 0,
        "member_hits": 0,
        "kind_survive": 0,
        "kind_survive_hist": [0, 0, 0, 0],
        "attach_n": 0,
        "mutex_n": 0,
    }
    if dg is None:
        return out
    raw = dg.materialize_selection([int(i) for i in selected])
    out["materialized_attach"] = int(raw.get("materialized_attach", 0) or 0)
    out["materialized_motif"] = int(raw.get("materialized_motif", 0) or 0)
    out["member_hits"] = int(raw.get("member_hits", 0) or 0)
    hist = [int(x) for x in (raw.get("kind_survive") or (0, 0, 0, 0))]
    hist = (hist + [0, 0, 0, 0])[:4]
    out["kind_survive_hist"] = hist
    out["kind_survive"] = int(sum(hist))
    out["attach_n"] = int(dg.attach_n())
    out["mutex_n"] = int(dg.mutex_n())
    if propose_stats is not None:
        propose_stats["materialized_attach"] = out["materialized_attach"]
        propose_stats["member_hits"] = out["member_hits"]
        propose_stats["kind_survive"] = out["kind_survive"]
        propose_stats["kind_survive_hist"] = list(hist)
        propose_stats["attach_n"] = out["attach_n"]
        propose_stats["mutex_n"] = out["mutex_n"]
    return out
