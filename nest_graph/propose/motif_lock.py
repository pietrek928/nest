"""Pre-MIS full-motif lock helpers.

C++ nest/refine honor ``locked_indices``; Scene growing-clear stays here.
"""

from dataclasses import dataclass
from typing import Sequence

from shapely.geometry import Point

from nest_graph.geometry import Geometry
from nest_graph.propose.placement_common import as_geometry, is_pose_clear
from nest_graph.propose.void_selection import transform_row_key


@dataclass
class LargeVoidMotifPlateau:
    """Q27: large_void-only plateau for Track D (≠ PlateauTracker).

    Fires when |Δcov| < cov_eps (coverage percentage points) for ``flat_iters``
    consecutive large_void iters that also have cluster_copy refine > 0.
    """

    flat_iters: int = 5
    cov_eps: float = 1.0
    last_cov: float | None = None
    streak: int = 0
    ready: bool = False

    def update(
        self,
        *,
        free_kind: str | None,
        cov: float,
        cluster_copy_refine: int,
    ) -> bool:
        if free_kind != "large_void":
            self.streak = 0
            self.last_cov = None
            self.ready = False
            return False
        cov_f = float(cov)
        copy_ok = int(cluster_copy_refine) > 0
        if self.last_cov is not None and copy_ok:
            flat = abs(cov_f - float(self.last_cov)) < float(self.cov_eps)
            self.streak = self.streak + 1 if flat else 0
        else:
            self.streak = 0
        self.last_cov = cov_f
        self.ready = self.streak >= max(int(self.flat_iters), 1) and copy_ok
        return self.ready


def _key_index_map(
    group_id: Sequence[int],
    transform: Sequence,
) -> dict[tuple[int, tuple[float, float, float]], int]:
    out: dict[tuple[int, tuple[float, float, float]], int] = {}
    for i, (gid, tr) in enumerate(zip(group_id, transform, strict=False)):
        out[(int(gid), transform_row_key(tr))] = int(i)
    return out


def sequential_accept_motif_cohorts(
    *,
    graph,
    scores: Sequence[float],
    group_id: Sequence[int],
    transform: Sequence,
    cohorts: Sequence[dict] | None,
    candidate_geoms: Sequence | None,
    void_geoms: Sequence | None,
    packed_geoms: Sequence | None,
    min_dist: float,
    pole: Point | None = None,
    max_accept: int = 3,
    rcl_top_k: int = 10,
) -> tuple[list[int], dict]:
    """Pre-MIS: accept full motif cohorts under growing Scene clear.

    Skip cohorts with missing in-graph followers. Deterministic pole top-k (v1).
    """
    telem = {
        "motif_sequential_full": 0,
        "motif_sequential_skipped_missing": 0,
        "motif_sequential_partial": 0,
        "motif_sequential_rcl": 0,
        "motif_sequential_clear_fail": 0,
        "motif_lock_sets": [],
        "motif_beam_sets": 0,
    }
    if not cohorts:
        return [], telem
    key_map = _key_index_map(group_id, transform)
    pole_xy: tuple[float, float] | None = None
    if pole is not None and not getattr(pole, "is_empty", True):
        pole_xy = (float(pole.x), float(pole.y))

    ranked: list[tuple[float, float, dict]] = []
    for cohort in cohorts:
        leader_key = cohort.get("leader_key")
        leader_gid = int(cohort.get("leader_gid", -1))
        if leader_key is None:
            continue
        lk = tuple(leader_key) if not isinstance(leader_key, tuple) else leader_key
        dist = 0.0
        if pole_xy is not None:
            dist = (float(lk[0]) - pole_xy[0]) ** 2 + (float(lk[1]) - pole_xy[1]) ** 2
        sc = 0.0
        idx = key_map.get((leader_gid, lk))
        if idx is not None and idx < len(scores):
            sc = float(scores[idx])
        ranked.append((dist, -sc, cohort))
    ranked.sort(key=lambda x: (x[0], x[1]))
    rcl = [c for _d, _s, c in ranked[: max(int(rcl_top_k), 1)]]
    telem["motif_sequential_rcl"] = len(rcl)

    voids = [g for g in (void_geoms or []) if g is not None]
    base_packed: list = [g for g in (packed_geoms or []) if g is not None]
    growing_packed: list = list(base_packed)
    beam_cap = 4

    locked: list[int] = []
    locked_set: set[int] = set()
    lock_sets: list[list[int]] = []
    accepted = 0
    for cohort in rcl:
        members = cohort.get("member_keys") or []
        idxs: list[int] = []
        missing = 0
        for item in members:
            if isinstance(item, (list, tuple)) and len(item) == 2:
                gid_m, key_m = int(item[0]), item[1]
            else:
                missing += 1
                continue
            key_t = tuple(key_m) if not isinstance(key_m, tuple) else key_m
            ix = key_map.get((gid_m, key_t))
            if ix is None:
                missing += 1
                continue
            idxs.append(int(ix))
        # Q17: do not invent missing nodes. Accept present subset when ≥2 survive
        # graph prune (emit recorded only packing-clear same-group keys).
        if len(idxs) < 2:
            telem["motif_sequential_skipped_missing"] += 1
            continue
        if missing:
            telem["motif_sequential_partial"] = int(
                telem.get("motif_sequential_partial", 0)
            ) + 1
        collisions = getattr(graph, "collisions", None)
        if collisions is not None:
            intra = False
            for i in idxs:
                if any(int(u) in idxs and int(u) != i for u in collisions[i]):
                    intra = True
                    break
            if intra:
                continue
        # Beam lock-sets: Scene-clear vs board packed only (independent alternatives).
        if len(lock_sets) < beam_cap:
            trial_ind = list(base_packed)
            clear_ind = True
            for i in idxs:
                if candidate_geoms is None or i >= len(candidate_geoms):
                    clear_ind = False
                    break
                geom = candidate_geoms[i]
                cg = as_geometry(geom) if not isinstance(geom, Geometry) else geom
                if cg is None or not is_pose_clear(
                    cg, voids, trial_ind, float(min_dist),
                ):
                    clear_ind = False
                    break
                trial_ind.append(cg)
            if clear_ind:
                lock_sets.append(list(idxs))

        if accepted >= int(max_accept):
            continue
        if any(i in locked_set for i in idxs):
            continue
        if collisions is not None:
            blocked = False
            for i in idxs:
                if any(int(u) in locked_set for u in collisions[i]):
                    blocked = True
                    break
            if blocked:
                continue
        # Growing Scene clear among present members (combined lock telem / tests).
        trial_packed = list(growing_packed)
        clear_ok = True
        member_geoms: list[Geometry] = []
        for i in idxs:
            if candidate_geoms is None or i >= len(candidate_geoms):
                clear_ok = False
                break
            geom = candidate_geoms[i]
            cg = as_geometry(geom) if not isinstance(geom, Geometry) else geom
            if cg is None or not is_pose_clear(
                cg, voids, trial_packed, float(min_dist),
            ):
                clear_ok = False
                break
            trial_packed.append(cg)
            member_geoms.append(cg)
        if not clear_ok:
            telem["motif_sequential_clear_fail"] += 1
            continue
        for i in idxs:
            locked.append(i)
            locked_set.add(i)
        growing_packed.extend(member_geoms)
        accepted += 1
        telem["motif_sequential_full"] += 1
    telem["motif_lock_sets"] = lock_sets
    telem["motif_beam_sets"] = len(lock_sets)
    return locked, telem
