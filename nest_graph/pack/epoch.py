"""Bind a PoseGraph epoch onto DecisionGraph (copy-in + Kind + Attach)."""

from typing import Any, Sequence

import numpy as np

from nest_graph.graph import DecisionGraph, MacroRegion
from nest_graph.pack.cache_key import motif_cohort_sig
from nest_graph.propose.motif_keys import cohort_member_indices
from nest_graph.propose.pattern_archive import inject_cohorts_from_patterns
from nest_graph.propose.transform_batch import graph_valid_carry_by_group
from nest_graph.propose.void_selection import pose_key_to_verts, transform_row_key
from nest_graph.proposer_names import ZONE_PROPOSERS, PlaceZone

_UNTAGGED = 255
_VOID_PROPOSER_NAMES = frozenset(p.value for p in ZONE_PROPOSERS[PlaceZone.VoidSeek])


def _motif_record_gci(stats: dict, mid: int) -> float:
    """D1: MotifBase gci for MotifJoin fracture weight (missing → 0)."""
    if mid < 0 or not isinstance(stats, dict):
        return 0.0
    mb = stats.get("motif_base")
    if mb is None:
        return 0.0
    try:
        if int(mid) >= int(mb.size()):
            return 0.0
        return float(mb.at(int(mid)).gci)
    except Exception:
        return 0.0


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


def _gid_key_union(src: dict | None) -> dict[int, set[tuple[float, float, float]]]:
    out: dict[int, set[tuple[float, float, float]]] = {}
    if not src:
        return out
    for gid, keys in src.items():
        out.setdefault(int(gid), set()).update(keys or ())
    return out


def _void_seek_proposer_keys(proposer_keys: dict | None) -> set[tuple[float, float, float]]:
    """Keys from proposers allowed under VOID_SEEK — not flat proposal/epoch unions."""
    out: set[tuple[float, float, float]] = set()
    if not proposer_keys:
        return out
    for name, keys in proposer_keys.items():
        if str(name) not in _VOID_PROPOSER_NAMES:
            continue
        out.update(keys or ())
    return out


def _proposer_key_union(proposer_keys: dict | None) -> set[tuple[float, float, float]]:
    out: set[tuple[float, float, float]] = set()
    if not proposer_keys:
        return out
    for keys in proposer_keys.values():
        out.update(keys or ())
    return out


def _hollow_graph_kind_gate(stats: dict, n_graph: int) -> bool:
    """Same thin-nest numbers as bind_graph_epoch carry_thin — restore kind mass late."""
    if n_graph <= 200:
        return False
    prev_nest = int(stats.get("last_nest_n", 0) or 0)
    return prev_nest * 4 < n_graph


def _action_region_i(stats: dict) -> int | None:
    raw = stats.get("mcts_region")
    if raw is None:
        return None
    if isinstance(raw, MacroRegion):
        return int(getattr(raw, "value", raw))
    try:
        return int(raw)
    except (TypeError, ValueError):
        return None


def _allow_void_proposer_kind(stats: dict) -> bool:
    """Void-proposer keys → Void kind only for Void-region macros (not Motif/Q104)."""
    ri = _action_region_i(stats)
    void_i = int(getattr(MacroRegion.Void, "value", 1))
    return ri == void_i


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
    if group_id is None or transform is None or n == 0 or n != len(transform):
        stats["attach_n"] = int(dg.attach_n())
        stats["kind_n"] = int(dg.kind_tagged_n())
        stats["mutex_n"] = int(dg.mutex_n())
        stats.setdefault("kind_keys", {})
        return

    densify = stats.get("densify_stats") or {}
    motif_keys = _gid_key_union(stats.get("motif_keys"))
    if densify.get("motif_keys"):
        for gi, keys in _gid_key_union(densify.get("motif_keys")).items():
            motif_keys.setdefault(gi, set()).update(keys)
    pocket_keys = _gid_key_union(stats.get("pocket_keys"))
    if not pocket_keys:
        pocket_keys = _gid_key_union(densify.get("pocket_keys"))
    sniper_keys = _gid_key_union(stats.get("sniper_keys"))
    if not sniper_keys:
        sniper_keys = _gid_key_union(densify.get("sniper_keys"))
    # L1: source buckets; unclassified emits → Sheet (not Void/Q104 zone paint).
    proposal_keys = _gid_key_union(stats.get("proposal_keys"))
    epoch_keys = _gid_key_union(stats.get("epoch_keys"))
    proposer_src = stats.get("proposer_keys") or densify.get("proposer_keys")
    void_prop_keys = _void_seek_proposer_keys(proposer_src)
    proposer_any = _proposer_key_union(proposer_src)
    allow_void_kind = _allow_void_proposer_kind(stats)
    ri = _action_region_i(stats)
    void_i = int(getattr(MacroRegion.Void, "value", 1))
    motif_val = int(getattr(MacroRegion.Motif, "value", 3))
    sheet_val = int(getattr(MacroRegion.Sheet, "value", 2))
    rim_val = int(getattr(MacroRegion.Rim, "value", 0))
    void_val = void_i
    # Hollow restore only on Motif tips (Q104 void_seek zone) — always-on hollow
    # flooded early Void epochs with Sheet and dropped early med below H3.
    hollow_kind = (
        _hollow_graph_kind_gate(stats, n) and ri == motif_val
    )

    kinds: list[int] = [_UNTAGGED] * n
    kind_keys: dict[int, set[tuple[float, float, float]]] = {}
    for i, (gid, tr) in enumerate(zip(group_id, transform, strict=False)):
        gi = int(gid)
        key = transform_row_key(tr)
        if key in motif_keys.get(gi, ()):
            kinds[i] = motif_val
        elif key in pocket_keys.get(gi, ()):
            kinds[i] = sheet_val
        elif key in sniper_keys.get(gi, ()):
            kinds[i] = rim_val
        elif allow_void_kind and key in void_prop_keys:
            kinds[i] = void_val
        elif key in proposal_keys.get(gi, ()):
            # Emitted but not pocket/sniper/void — Sheet, never Void via zone.
            kinds[i] = sheet_val
        elif hollow_kind and (
            key in epoch_keys.get(gi, ()) or key in proposer_any
        ):
            # Late hollow Motif tips: Sheet (not Void — region is Motif).
            kinds[i] = sheet_val
        else:
            kinds[i] = _UNTAGGED
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
    motif_join_from_base = 0
    cohort_kind_touch = False
    for cohort in cohorts:
        if not isinstance(cohort, dict):
            continue
        # motif_id=0 is valid — never ``x or -1``.
        _mid_raw = cohort.get("motif_id", -1)
        mid = -1 if _mid_raw is None else int(_mid_raw)
        idxs, _missing = cohort_member_indices(cohort, verts, first_only=False)
        if len(idxs) < 2:
            continue
        leader = _cohort_leader_idx(idxs, graph)
        if leader < 0:
            continue
        motif_val = int(getattr(MacroRegion.Motif, "value", 3))
        # L1: gid-keyed spoke mids from cohort (hub/follower MotifBase pair ids).
        spoke_lookup: dict[tuple[int, int], int] = {}
        for t in cohort.get("spoke_mids") or ():
            if not isinstance(t, (tuple, list)) or len(t) < 3:
                continue
            ga, gb, sm = int(t[0]), int(t[1]), int(t[2])
            if sm < 0:
                continue
            lo, hi = (ga, gb) if ga <= gb else (gb, ga)
            spoke_lookup[(lo, hi)] = sm
        lead_gid = int(group_id[int(leader)])
        for ix in idxs:
            gi = int(group_id[int(ix)])
            key = transform_row_key(transform[int(ix)])
            kinds[int(ix)] = motif_val
            kind_keys.setdefault(gi, set()).add(key)
            cohort_kind_touch = True
            if int(ix) == int(leader):
                continue
            # L1: prefer spoke mid whose MotifRecord gid pair matches endpoints.
            pair = (lead_gid, gi) if lead_gid <= gi else (gi, lead_gid)
            spoke_mid = spoke_lookup.get(pair, -1)
            join_mid = int(spoke_mid) if int(spoke_mid) >= 0 else mid
            dg.add_motif_join(
                join_mid,
                int(leader),
                int(ix),
                float(_motif_record_gci(stats, join_mid)),
            )
            motif_join_n += 1
            # F0: MotifBase-backed Join attribution (mid >= 0 from pattern/cohort).
            if join_mid >= 0:
                motif_join_from_base += 1
                if int(spoke_mid) >= 0 and int(spoke_mid) != mid:
                    stats["motif_join_spoke_n"] = int(
                        stats.get("motif_join_spoke_n", 0) or 0
                    ) + 1
    if cohort_kind_touch:
        dg.set_pose_kinds(kinds)
    stats["motif_join_n"] = int(motif_join_n)
    stats["motif_join_from_base"] = int(motif_join_from_base)

    stats["kind_keys"] = kind_keys
    stats["attach_n"] = int(dg.attach_n())
    stats["kind_n"] = int(dg.kind_tagged_n())
    stats["mutex_n"] = int(dg.mutex_n())
    if propose_stats is not None:
        prior_base_j = int(propose_stats.get("motif_join_from_base", 0) or 0)
        prior_spoke = int(propose_stats.get("motif_join_spoke_n", 0) or 0)
        propose_stats.update(stats)
        # Accumulate MotifBase→Join edges across binds this iter.
        propose_stats["motif_join_from_base"] = prior_base_j + int(motif_join_from_base)
        propose_stats["motif_join_spoke_n"] = prior_spoke + int(
            stats.get("motif_join_spoke_n", 0) or 0
        )


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
    sc_raw = dg.survive_counts() if hasattr(dg, "survive_counts") else {}
    sc = sc_raw if isinstance(sc_raw, dict) else {}
    out["survive_by_motif"] = {int(k): int(v) for k, v in sc.items()}
    if propose_stats is not None:
        propose_stats["materialized_attach"] = out["materialized_attach"]
        propose_stats["materialized_motif"] = out["materialized_motif"]
        propose_stats["member_hits"] = out["member_hits"]
        propose_stats["kind_survive"] = out["kind_survive"]
        propose_stats["kind_survive_hist"] = list(hist)
        propose_stats["attach_n"] = out["attach_n"]
        propose_stats["mutex_n"] = out["mutex_n"]
        propose_stats["survive_by_motif"] = sc
        propose_stats["survive_motif_n"] = int(sum(int(v) for v in sc.values()))
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
    # D1b: fat graph vs thin nest → soft-cap carry (PoseGraph bloat).
    prev_graph = int(
        propose_stats.get("last_graph_n", propose_stats.get("graph_valid_n", 0)) or 0
    )
    prev_nest = int(propose_stats.get("last_nest_n", 0) or 0)
    if prev_graph > 200 and prev_nest * 4 < prev_graph:
        carry_max = min(carry_max, 256)
        propose_stats["carry_thin"] = 1
    drop_by_group = propose_stats.get("carry_drop_keys_by_group")
    sample_seed = propose_stats.get("carry_sample_seed")
    if bool(getattr(cfg.propose, "enable_graph_valid_carry", True)):
        carry = graph_valid_carry_by_group(
            group_id,
            transform,
            ngroups=ngroups,
            max_keep=carry_max,
            drop_keys_by_group=drop_by_group,
            sample_seed=int(sample_seed) if sample_seed is not None else None,
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
    pats = list(patterns or ())
    if pats:
        inject_cohorts_from_patterns(
            pats, group_id, transform, propose_stats,
        )
        if agent is not None:
            agent.motif_cohorts = tuple(propose_stats.get("motif_cohorts") or ())
    carry = bind_graph_epoch(dg, graph, group_id, transform, propose_stats, cfg)
    # F0: mirror base_j onto agent telem for path census / dg_funnel.
    # D4 inject-sync / force-sticky dual-missed early↔late — park mid-iter sync.
    if agent is not None:
        at = getattr(agent, "telem", None)
        if isinstance(at, dict):
            at["motif_join_from_base"] = max(
                int(at.get("motif_join_from_base", 0) or 0),
                int(propose_stats.get("motif_join_from_base", 0) or 0),
            )
    return carry


def sync_agent_motif_cohorts(runner: Any, propose_stats: dict | None) -> int:
    """Letter D/M2b: feed propose motif_cohorts + sticky readiness telem for PLACE_COHORT."""
    agent = getattr(runner, "agent", None)
    # D4: allow passing the agent directly.
    if agent is None and hasattr(runner, "motif_cohorts"):
        agent = runner
    if agent is None:
        return 0
    cohorts = list((propose_stats or {}).get("motif_cohorts") or ())
    # Prefer agent-hydrated cohorts when stats omit them (ckpt iter0 / cold tip).
    if not cohorts:
        cohorts = list(getattr(agent, "motif_cohorts", ()) or ())
    # Always keep cohorts on the agent (ablation winner for density); gate PLACE_COHORT
    # expand via place_cohort_ready so soft/path still sees members (hybrid M2b).
    agent.motif_cohorts = tuple(cohorts)
    n_ready = 0
    for c in cohorts:
        if not isinstance(c, dict):
            continue
        if int(len(c.get("member_keys") or ())) >= 2:
            n_ready += 1
    if propose_stats is not None:
        compose_sz = int(propose_stats.get("motif_compose_accepted_size", 0) or 0)
        if compose_sz <= 0:
            compose_sz = len(propose_stats.get("motif_locked") or ())
        member_hits = int(propose_stats.get("member_hits", 0) or 0)
        if member_hits <= 0 and isinstance(propose_stats.get("void_leak"), dict):
            member_hits = int(
                (propose_stats.get("void_leak") or {}).get("member_hits", 0) or 0
            )
        hard = bool(n_ready > 0 and compose_sz >= 2 and member_hits > 0)
        # D4 soft: drop member_hits chicken-egg for PLACE_COHORT expand when
        # compose_sz≥2 already sticky (AMAF/cohort still learn; hard telem kept).
        soft = bool(n_ready > 0 and compose_sz >= 2)
        ready = int(soft)
        propose_stats["place_cohort_specs_n"] = int(n_ready)
        propose_stats["place_cohort_ready"] = ready
        propose_stats["place_cohort_ready_hard"] = int(hard)
        propose_stats["motif_cohort_sig"] = (
            motif_cohort_sig(cohorts) if ready else 0
        )
        if hasattr(agent, "place_cohort_ready"):
            agent.place_cohort_ready = bool(ready)
        return int(n_ready)
    # D4: stats None (cold tip / ckpt iter0) — keep ready if cohorts already hydrated.
    if n_ready > 0 and bool(getattr(agent, "place_cohort_ready", False)):
        return int(n_ready)
    if hasattr(agent, "place_cohort_ready") and n_ready == 0:
        agent.place_cohort_ready = False
    return int(n_ready)

