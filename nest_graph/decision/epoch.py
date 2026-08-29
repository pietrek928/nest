"""Bind a PoseGraph epoch onto DecisionGraph (copy-in + Kind + Attach)."""

from typing import Sequence

import numpy as np

from nest_graph.elem_graph import DecisionGraph, MacroRegion
from nest_graph.propose.motif_keys import cohort_member_indices
from nest_graph.propose.transform_batch import graph_valid_carry_by_group
from nest_graph.propose.void_selection import pose_key_to_verts, transform_row_key

_UNTAGGED = 255


def _elem_area_proxy(graph, idx: int) -> float:
    coords = getattr(graph, "coords", None)
    if coords is not None and 0 <= int(idx) < len(coords):
        c = coords[int(idx)]
        r = float(getattr(c, "radius", getattr(c, "r", 0.0)) or 0.0)
        return r * r
    return 0.0


def _cohort_leader_idx(idxs: Sequence[int], graph) -> int:
    """Q241: dynamic leader = max bbox-area proxy (tie → highest contact degree)."""
    if not idxs:
        return -1
    best_i = int(idxs[0])
    best_area = _elem_area_proxy(graph, best_i)
    best_deg = 0
    collisions = getattr(graph, "collisions", None)
    if collisions is not None and best_i < len(collisions):
        best_deg = sum(1 for u in collisions[best_i] if int(u) in idxs)
    for ix in idxs[1:]:
        i = int(ix)
        area = _elem_area_proxy(graph, i)
        deg = 0
        if collisions is not None and i < len(collisions):
            deg = sum(1 for u in collisions[i] if int(u) in idxs)
        if area > best_area + 1e-12 or (
            abs(area - best_area) <= 1e-12 and deg > best_deg
        ):
            best_i = i
            best_area = area
            best_deg = deg
    return best_i


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
    motif_join_n = 0
    cohort_kind_touch = False
    for cohort in cohorts:
        if not isinstance(cohort, dict):
            continue
        mid = int(cohort.get("motif_id", -1) or -1)
        idxs, _missing = cohort_member_indices(cohort, verts, first_only=False)
        if len(idxs) < 2:
            continue
        leader = _cohort_leader_idx(idxs, graph)
        if leader < 0:
            continue
        motif_val = int(getattr(MacroRegion.Motif, "value", 3))
        for ix in idxs:
            gi = int(group_id[int(ix)])
            key = transform_row_key(transform[int(ix)])
            kinds[int(ix)] = motif_val
            kind_keys.setdefault(gi, set()).add(key)
            cohort_kind_touch = True
            if int(ix) == int(leader):
                continue
            dg.add_motif_join(mid, int(leader), int(ix))
            motif_join_n += 1
    if cohort_kind_touch:
        dg.set_pose_kinds(kinds)
    stats["motif_join_n"] = int(motif_join_n)

    stats["kind_keys"] = kind_keys
    stats["attach_n"] = int(dg.attach_n())
    stats["kind_n"] = int(dg.kind_tagged_n())
    stats["mutex_n"] = int(dg.mutex_n())
    if propose_stats is not None:
        propose_stats.update(stats)


def realize_selection(dg, selected: Sequence[int], propose_stats: dict | None = None) -> dict:
    """Flag Attach/MotifJoin whose members survived MWIS (Q154/Q381)."""
    out = {
        "materialized_attach": 0,
        "materialized_motif": 0,
        "member_hits": 0,
        "kind_survive": 0,
        "kind_survive_hist": [0, 0, 0, 0],
        "attach_n": 0,
        "mutex_n": 0,
        "survive_by_motif": {},
    }
    if dg is None:
        return out
    raw = dg.realize([int(i) for i in selected])
    out["materialized_attach"] = int(raw.get("attach", raw.get("materialized_attach", 0)) or 0)
    out["materialized_motif"] = int(raw.get("motif", raw.get("materialized_motif", 0)) or 0)
    out["member_hits"] = int(raw.get("member_hits", 0) or 0)
    hist = [int(x) for x in (raw.get("kind_survive") or (0, 0, 0, 0))]
    hist = (hist + [0, 0, 0, 0])[:4]
    out["kind_survive_hist"] = hist
    out["kind_survive"] = int(sum(hist))
    out["attach_n"] = int(dg.attach_n())
    out["mutex_n"] = int(dg.mutex_n())
    sc = dg.survive_counts() if hasattr(dg, "survive_counts") else {}
    out["survive_by_motif"] = {int(k): int(v) for k, v in dict(sc).items()}
    if propose_stats is not None:
        propose_stats["materialized_attach"] = out["materialized_attach"]
        propose_stats["materialized_motif"] = out["materialized_motif"]
        propose_stats["member_hits"] = out["member_hits"]
        propose_stats["kind_survive"] = out["kind_survive"]
        propose_stats["kind_survive_hist"] = list(hist)
        propose_stats["attach_n"] = out["attach_n"]
        propose_stats["mutex_n"] = out["mutex_n"]
        propose_stats["survive_by_motif"] = dict(out["survive_by_motif"])
        propose_stats["survive_motif_n"] = int(
            sum(int(v) for v in out["survive_by_motif"].values())
        )
        motif_locked = propose_stats.get("motif_locked") or ()
        sel_set = {int(i) for i in selected}
        propose_stats["lock_n_materialize"] = len(
            [int(i) for i in motif_locked if int(i) in sel_set]
        )
        propose_stats["lock_n_realize"] = int(propose_stats["lock_n_materialize"])
    return out


def bind_graph_epoch(
    dg,
    graph,
    group_id: Sequence[int],
    transform: Sequence,
    propose_stats: dict,
    cfg,
) -> tuple:
    """bind_epoch + graph_valid_carry_by_group (single bind site)."""
    bind_epoch(dg, graph, propose_stats, group_id, transform)
    ngroups = int(getattr(cfg.rules, "ngroups", 0) or len(group_id))
    carry_max = int(getattr(cfg.propose, "graph_valid_carry_max", 512) or 512)
    if bool(getattr(cfg.propose, "enable_graph_valid_carry", True)):
        carry = graph_valid_carry_by_group(
            group_id, transform, ngroups=ngroups, max_keep=carry_max,
        )
    else:
        carry = tuple(
            np.zeros((0, 3), dtype=np.float64) for _ in range(ngroups)
        )
    propose_stats["graph_valid_n"] = int(len(transform))
    propose_stats["carry_n_next"] = int(
        sum(int(a.shape[0]) for a in carry)
    )
    return carry


def inject_cohorts_and_bind_graph(
    dg,
    graph,
    group_id: Sequence[int],
    transform: Sequence,
    propose_stats: dict,
    cfg,
    *,
    patterns: Sequence | None = None,
    agent=None,
) -> tuple:
    """Q255: inject archive cohorts then bind epoch (one gate for both call sites)."""
    if patterns:
        from nest_graph.propose.pattern_archive import inject_cohorts_from_patterns

        inject_cohorts_from_patterns(
            patterns, group_id, transform, propose_stats,
        )
        if agent is not None:
            agent.motif_cohorts = tuple(propose_stats.get("motif_cohorts") or ())
    return bind_graph_epoch(dg, graph, group_id, transform, propose_stats, cfg)
