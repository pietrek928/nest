"""Cheap expand slave: wrap propose knobs → nest (no DFS/3b/se2)."""


import time
from dataclasses import dataclass
from typing import Any, Callable, Sequence

from nest_graph.decision.action_gen import region_to_zone
from nest_graph.decision.mcts import leaf_reward, timed_expand_ms
from nest_graph.decision.types import BoardSnapshot
from nest_graph.elem_graph import (
    ContactEdge,
    MacroAction,
    MacroRegion,
    MotifBase,
    Se2,
    clamp01,
    gci_surrogate,
)
from nest_graph.utils import relative_transform


@dataclass(slots=True)
class ExpandResult:
    snapshot: BoardSnapshot
    reward: float
    ok: bool


def _motif_inject_patterns(
    motif_base: MotifBase,
    action: MacroAction,
) -> list[Any]:
    """Build ClusterPattern-compatible inject list from MotifBase (adapter)."""
    from nest_graph.propose.pattern_archive import motif_to_cluster_patterns

    return list(motif_to_cluster_patterns(motif_base, action))


def _pair_compactness(ga, gb) -> float:
    from nest_graph.geometry import convex_hull_area_of

    area_sum = abs(float(ga.area())) + abs(float(gb.area()))
    if area_sum <= 0.0:
        return 0.0
    hull = float(convex_hull_area_of([ga, gb]))
    if hull <= 1e-12:
        return 0.0
    return float(clamp01(area_sum / hull))


def motif_floor_compactness(
    motif_base: MotifBase,
    cfg_min: float,
) -> float:
    """Q94: hard floor raised by moving median when library non-empty."""
    floor = float(cfg_min)
    if int(motif_base.size()) > 0:
        floor = max(floor, float(motif_base.moving_median_compactness()))
    return floor


def upsert_from_contacts(
    motif_base: MotifBase,
    geoms: Sequence,
    gids: Sequence[int],
    transforms: Sequence,
    *,
    gap: float,
    min_compactness: float = 0.35,
    ttl: int = 0,
    max_keep: int = 0,
    telem: dict | None = None,
    selection_mask: Sequence[bool] | None = None,
) -> int:
    """ContactGRG → MotifBase via bound ContactEdge + gci_surrogate (C1).

    Distance query stays Python (Geometry module); GCI/score use C++ ContactGRG
    helpers; Motif upsert via ``upsert_contact``.
    """
    from nest_graph.geometry import find_polygon_distances

    n = len(geoms)
    if n < 2 or len(gids) != n or len(transforms) != n:
        return 0
    contact = 2.0 * float(gap)
    contact_eps = contact + 1e-9
    aura = max(contact, 0.5) * 2.0
    try:
        results = find_polygon_distances(list(geoms), aura=aura)
    except Exception:
        if telem is not None:
            telem["contact_grg_fail"] = int(telem.get("contact_grg_fail", 0)) + 1
        return 0

    floor = motif_floor_compactness(motif_base, min_compactness)
    n_up = 0
    for r in results:
        i = int(r.polyA_idx)
        j = int(r.polyB_idx)
        if i < 0 or j < 0 or i >= n or j >= n or i >= j:
            continue
        if selection_mask is not None:
            if i >= len(selection_mask) or j >= len(selection_mask):
                continue
            if not (bool(selection_mask[i]) and bool(selection_mask[j])):
                continue
        dist = 0.0 if bool(r.intersect) else float(r.distance)
        if (not bool(r.intersect)) and dist > contact_eps:
            continue
        contact_score = 1.0
        if contact > 0.0 and not bool(r.intersect):
            contact_score = float(clamp01(1.0 - dist / contact))
        ga, gb = geoms[i], geoms[j]
        compactness = _pair_compactness(ga, gb)
        gci = float(gci_surrogate(float(compactness), float(contact_score)))
        area_i = abs(float(ga.area()))
        area_j = abs(float(gb.area()))
        gid_i, gid_j = int(gids[i]), int(gids[j])
        if area_j > area_i + 1e-12 or (
            abs(area_j - area_i) <= 1e-12 and gid_j < gid_i
        ):
            ia, ib = j, i
            area_a, area_b = area_j, area_i
        else:
            ia, ib = i, j
            area_a, area_b = area_i, area_j
        del area_a, area_b
        t_a = transforms[ia]
        t_b = transforms[ib]
        rel = relative_transform(
            (float(t_a[0]), float(t_a[1]), float(t_a[2])),
            (float(t_b[0]), float(t_b[1]), float(t_b[2])),
        )
        edge = ContactEdge()
        edge.gid_a = int(gids[ia])
        edge.gid_b = int(gids[ib])
        edge.packed_i = int(ia)
        edge.packed_j = int(ib)
        edge.relative_pose = Se2(float(rel[0]), float(rel[1]), float(rel[2]))
        edge.contact_score = float(contact_score)
        edge.compactness = float(compactness)
        edge.gci = float(gci)
        mid = int(motif_base.upsert_contact(edge, float(floor), int(ttl)))
        if mid >= 0:
            n_up += 1
    if max_keep > 0:
        motif_base.truncate(int(max_keep))
    if telem is not None:
        telem["contact_grg_upserts"] = int(telem.get("contact_grg_upserts", 0)) + n_up
        telem["motif_floor"] = float(floor)
    return n_up


def cheap_expand_slave(
    parent: BoardSnapshot,
    action: MacroAction,
    *,
    motif_base: MotifBase,
    execute_fn: Callable[..., BoardSnapshot] | None = None,
    telem: dict | None = None,
) -> ExpandResult:
    """
    Run one cheap expand.

    ``execute_fn`` if provided: (parent, zone, action, patterns) -> BoardSnapshot.
    Default stub advances coverage slightly for unit tests without full nest.
    """
    t0 = time.perf_counter()
    zone = region_to_zone(action.region)
    patterns = []
    if action.region == MacroRegion.Motif:
        patterns = _motif_inject_patterns(motif_base, action)

    if execute_fn is not None:
        snap = execute_fn(parent, zone=zone, action=action, patterns=patterns)
    else:
        # Stub path for tests / dry runs
        placed = list(parent.packed_gids)
        rem = list(parent.remaining_gids)
        if rem:
            gid = int(action.part_gid) if int(action.part_gid) in rem else rem[0]
            rem = [g for g in rem if g != gid]
            placed.append(gid)
        transforms = list(parent.packed_transforms)
        transforms.append((0.0, 0.0, 0.0))
        snap = BoardSnapshot(
            packed_gids=tuple(placed),
            packed_transforms=tuple(transforms[: len(placed)]),
            remaining_gids=tuple(rem),
            coverage=min(1.0, float(parent.coverage) + 0.05),
            arena_node_id=parent.arena_node_id,
            kiss_pairs=parent.kiss_pairs,
            mean_compactness=parent.mean_compactness,
            rim_fill=parent.rim_fill,
            void_fill=parent.void_fill,
            free_kind=parent.free_kind,
            motif_ids_used=parent.motif_ids_used,
            telem=dict(parent.telem or {}),
        )
    timed_expand_ms(telem, t0)
    return ExpandResult(snapshot=snap, reward=leaf_reward(snap), ok=True)
