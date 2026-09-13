"""Pre-MIS full-motif lock helpers.

C++ nest/refine honor ``locked_indices``; Scene growing-clear stays here.
One gate for sequential accept, hollow cc pattern locks, and score steer fallback.
"""

from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from typing import Any

from shapely.geometry import Point

from nest_graph.geometry import Geometry
from nest_graph.propose.block_replace import _packing_independent
from nest_graph.propose.motif_keys import (
    boost_score_indices,
    cluster_copy_lock_pool,
    cohort_member_indices,
)
from nest_graph.propose.context import cluster_contact_components
from nest_graph.propose.placement_common import (
    as_geometry,
    clear_of_geoms,
    is_board_adj,
    is_pose_clear,
    outline_ring_geom,
    placement_obstacles,
)
from nest_graph.propose.void_selection import centroid_in_free, pose_key_to_index
from nest_graph.utils import transform_row_key


def prefer_void_core_locks(
    locks: Sequence[Sequence[int]],
    polys: Sequence | None,
    free_poly,
) -> tuple[list[list[int]], int]:
    """Prefer locks whose members are void-core; fallback to full list if none."""
    raw = [list(lock) for lock in locks if len(lock) >= 2]
    if (
        not raw
        or polys is None
        or free_poly is None
        or getattr(free_poly, "is_empty", True)
    ):
        return raw, 0
    preferred: list[list[int]] = []
    for lock in raw:
        if all(
            0 <= int(i) < len(polys)
            and centroid_in_free(polys[int(i)], free_poly)
            for i in lock
        ):
            preferred.append([int(i) for i in lock])
    if preferred:
        return preferred, len(preferred)
    return raw, 0


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


def _growing_subset_indices(
    idxs: Sequence[int],
    candidate_geoms: Sequence | None,
    trial_packed: list,
    accept: Callable[[Geometry, list], bool],
) -> tuple[list[int], list[Geometry], list]:
    """Greedy growing subset: keep members that pass accept(cg, trial)."""
    kept: list[int] = []
    geoms: list[Geometry] = []
    trial = list(trial_packed)
    for i in idxs:
        if candidate_geoms is None or i >= len(candidate_geoms):
            break
        geom = candidate_geoms[i]
        cg = as_geometry(geom) if not isinstance(geom, Geometry) else geom
        if cg is None or not accept(cg, trial):
            continue
        trial.append(cg)
        geoms.append(cg)
        kept.append(int(i))
    return kept, geoms, trial


def _lock_sig(idxs: Sequence[int]) -> tuple[int, ...]:
    return tuple(sorted(int(i) for i in idxs))


def _append_lock_set(
    lock_sets: list[list[int]],
    seen: set[tuple[int, ...]],
    idxs: Sequence[int],
    *,
    scene_max: list[int],
) -> None:
    if len(idxs) < 2:
        return
    sig = _lock_sig(idxs)
    if sig in seen:
        return
    seen.add(sig)
    lock_sets.append([int(i) for i in idxs])
    scene_max[0] = max(scene_max[0], len(idxs))


def _rank_cohorts(
    cohorts: Sequence[dict],
    key_map: Mapping[tuple[int, tuple[float, float, float]], Any],
    scores: Sequence[float],
    pole: Point | None,
    *,
    polys: Sequence | None = None,
    free_poly=None,
) -> list[dict]:
    """Rank cohorts: more free members → closer to void pole → −leader score."""
    pole_xy: tuple[float, float] | None = None
    if pole is not None and not getattr(pole, "is_empty", True):
        pole_xy = (float(pole.x), float(pole.y))
    use_free = (
        polys is not None
        and free_poly is not None
        and not getattr(free_poly, "is_empty", True)
    )
    ranked: list[tuple[int, float, float, dict]] = []
    for cohort in cohorts:
        leader_key = cohort.get("leader_key")
        leader_gid = int(cohort.get("leader_gid", -1))
        if leader_key is None:
            continue
        lk = transform_row_key(leader_key)
        dist = 0.0
        if pole_xy is not None:
            dist = (float(lk[0]) - pole_xy[0]) ** 2 + (float(lk[1]) - pole_xy[1]) ** 2
        sc = 0.0
        idx = key_map.get((leader_gid, lk))
        if idx is not None and idx < len(scores):
            sc = float(scores[idx])
        n_free = 0
        if use_free:
            idxs, _miss = cohort_member_indices(cohort, key_map)
            for i in idxs:
                ii = int(i)
                if 0 <= ii < len(polys) and centroid_in_free(polys[ii], free_poly):
                    n_free += 1
        ranked.append((-int(n_free), dist, -sc, cohort))
    ranked.sort(key=lambda x: (x[0], x[1], x[2]))
    return [c for _nf, _d, _s, c in ranked]


def _scene_accept_factory(
    void_geoms: Sequence | None,
    min_dist: float,
) -> Callable[[Geometry, list], bool]:
    voids = [g for g in (void_geoms or []) if g is not None]
    min_dist_f = float(min_dist)

    def _scene_accept(cg: Geometry, trial: list) -> bool:
        return is_pose_clear(cg, voids, trial, min_dist_f)

    return _scene_accept


def scene_pair_locks_from_indices(
    *,
    graph,
    indices: Sequence[int],
    candidate_geoms: Sequence | None,
    void_geoms: Sequence | None,
    packed_geoms: Sequence | None,
    min_dist: float,
    scores: Sequence[float] | None = None,
    max_pairs: int = 3,
) -> list[list[int]]:
    """Independent index pairs that pass Scene growing clear."""
    idxs = sorted({int(i) for i in indices if int(i) >= 0})
    if len(idxs) < 2:
        return []
    collisions = getattr(graph, "collisions", None)
    base_packed: list = [g for g in (packed_geoms or []) if g is not None]
    scene_accept = _scene_accept_factory(void_geoms, min_dist)

    ranked: list[tuple[float, list[int]]] = []
    for ai, a in enumerate(idxs):
        for b in idxs[ai + 1 :]:
            if collisions is not None:
                if any(int(u) == b for u in collisions[a]):
                    continue
            pair_idxs, _, _ = _growing_subset_indices(
                [a, b], candidate_geoms, base_packed, scene_accept,
            )
            if len(pair_idxs) < 2:
                continue
            pair_score = 0.0
            if scores is not None:
                for ix in pair_idxs:
                    if 0 <= int(ix) < len(scores):
                        pair_score += float(scores[int(ix)])
            ranked.append((-pair_score, [int(i) for i in pair_idxs]))
    ranked.sort(key=lambda x: x[0])
    cap = max(int(max_pairs), 0)
    return [pair for _sc, pair in ranked[:cap]]


def hollow_pattern_lock_sets(
    *,
    graph,
    cohorts: Sequence[dict] | None,
    cc_pool: Sequence[int],
    key_map: Mapping[tuple[int, tuple[float, float, float]], Any],
    candidate_geoms: Sequence | None,
    void_geoms: Sequence | None,
    packed_geoms: Sequence | None,
    min_dist: float,
    scores: Sequence[float] | None = None,
    pole: Point | None = None,
    max_locks: int = 4,
) -> tuple[list[list[int]], dict]:
    """Cohort-first Scene lock sets when cc_graph≥2 but cc_nest=0 (Q359 unify).

    Prefer growing Scene subset per stamped cohort; fallback to flat cc pairs.
    """
    telem = {"hollow_pattern_locks": 0, "hollow_flat_pair_locks": 0}
    cc_set = {int(i) for i in cc_pool}
    if len(cc_set) < 2:
        return [], telem

    base_packed: list = [g for g in (packed_geoms or []) if g is not None]
    scene_accept = _scene_accept_factory(void_geoms, min_dist)
    lock_sets: list[list[int]] = []
    seen: set[tuple[int, ...]] = set()
    scene_max = [0]
    cap = max(int(max_locks), 0)

    for cohort in _rank_cohorts(
        list(cohorts or ()),
        key_map,
        scores or (),
        pole,
        polys=candidate_geoms,
        free_poly=None,
    ):
        idxs, _miss = cohort_member_indices(cohort, key_map)
        pattern_idxs = [int(i) for i in idxs if int(i) in cc_set]
        if len(pattern_idxs) < 2 or not _packing_independent(pattern_idxs, graph):
            continue
        beam_idxs, _, _ = _growing_subset_indices(
            pattern_idxs, candidate_geoms, base_packed, scene_accept,
        )
        if len(beam_idxs) >= 2:
            _append_lock_set(lock_sets, seen, beam_idxs, scene_max=scene_max)
            telem["hollow_pattern_locks"] = int(telem["hollow_pattern_locks"]) + 1
        if len(lock_sets) < cap and len(pattern_idxs) >= 2:
            for pair in scene_pair_locks_from_indices(
                graph=graph,
                indices=pattern_idxs,
                candidate_geoms=candidate_geoms,
                void_geoms=void_geoms,
                packed_geoms=packed_geoms,
                min_dist=min_dist,
                scores=scores,
                max_pairs=1,
            ):
                _append_lock_set(lock_sets, seen, pair, scene_max=scene_max)
        if len(lock_sets) >= cap:
            break

    if not lock_sets:
        flat = scene_pair_locks_from_indices(
            graph=graph,
            indices=sorted(cc_set),
            candidate_geoms=candidate_geoms,
            void_geoms=void_geoms,
            packed_geoms=packed_geoms,
            min_dist=min_dist,
            scores=scores,
            max_pairs=cap,
        )
        for pair in flat:
            _append_lock_set(lock_sets, seen, pair, scene_max=scene_max)
        telem["hollow_flat_pair_locks"] = len(flat)

    if scene_max[0] > 0:
        telem["motif_scene_max_sz"] = scene_max[0]
    return lock_sets[:cap], telem


def hollow_cc_score_steer(
    scores: list[float],
    transform: Sequence,
    cc_keys: set[tuple[float, float, float]],
    *,
    motif_weight: float,
    void_term: float,
) -> int:
    """Last-resort MIS soft steer when Scene pair locks fail (Q262: never lock)."""
    boost_w = float(motif_weight) * 2.0 if motif_weight > 0.0 else float(void_term)
    if boost_w <= 0.0 or not cc_keys:
        return 0
    boost_idxs: list[int] = []
    for i, tr in enumerate(transform):
        if transform_row_key(tr) in cc_keys:
            boost_idxs.append(int(i))
    return boost_score_indices(scores, boost_idxs, boost_w)


@dataclass
class MotifComposeResult:
    lock_sets: list[list[int]]
    boost_idxs: list[int]
    telem: dict


def anchored_nest_indices(
    *,
    selected_nest: Sequence[int],
    polys: Sequence,
    candidate_geoms: Sequence | None,
    sheet,
    min_dist: float,
    packed_geoms: Sequence | None = None,
) -> tuple[list[int], dict]:
    """Q361: board_adj or contact-connected to board_adj / seed packed."""
    telem = {"trial_packed_anchored_n": 0, "trial_packed_float_skip_n": 0}
    sel = sorted({int(i) for i in selected_nest if int(i) >= 0})
    if not sel:
        return [], telem

    board_adj: set[int] = set()
    board_ring = outline_ring_geom(sheet)
    for i in sel:
        if i < len(polys) and is_board_adj(
            polys[i], sheet, min_dist, ring=board_ring,
        ):
            board_adj.add(i)

    anchored: set[int] = set(board_adj)
    geoms: list[Geometry] = []
    idx_map: list[int] = []
    for i in sel:
        if candidate_geoms is None or i >= len(candidate_geoms):
            continue
        raw = candidate_geoms[i]
        if raw is None:
            continue
        cg = raw if isinstance(raw, Geometry) else as_geometry(raw)
        if cg is None:
            continue
        geoms.append(cg)
        idx_map.append(int(i))

    if geoms:
        from nest_graph.propose.context import _cluster_merge_gap

        sample = [polys[i] for i in sel if i < len(polys)]
        gap = _cluster_merge_gap(sample, min_dist, sheet) if sample else float(min_dist)
        board_ring = outline_ring_geom(sheet)
        board_gap = float(min_dist) + 2.0 * gap
        comps = cluster_contact_components(
            geoms, gap, board_ring=board_ring, board_gap=board_gap,
        )
        for members, board_comp in comps:
            member_idxs = [idx_map[li] for li in members]
            if board_comp or any(gi in board_adj for gi in member_idxs):
                for gi in member_idxs:
                    anchored.add(gi)

    packed_list: list[Geometry] = []
    for g in packed_geoms or ():
        if g is None:
            continue
        pg = g if isinstance(g, Geometry) else as_geometry(g)
        if pg is not None:
            packed_list.append(pg)
    if packed_list and geoms:
        touch = 2.0 * float(min_dist)
        for li, gi in enumerate(idx_map):
            if gi in anchored:
                continue
            cg = geoms[li]
            if any(float(cg.standoff_distance(pg)) <= touch + 1e-9 for pg in packed_list):
                anchored.add(gi)
        for members, board_comp in cluster_contact_components(
            geoms, gap if geoms else float(min_dist),
            board_ring=outline_ring_geom(sheet),
            board_gap=float(min_dist) + 2.0 * (gap if geoms else float(min_dist)),
        ):
            member_idxs = [idx_map[li] for li in members]
            if board_comp or any(gi in board_adj for gi in member_idxs):
                for gi in member_idxs:
                    anchored.add(gi)
            elif any(gi in anchored for gi in member_idxs):
                for gi in member_idxs:
                    anchored.add(gi)

    out = sorted(anchored)
    telem["trial_packed_anchored_n"] = len(out)
    telem["trial_packed_float_skip_n"] = max(0, len(sel) - len(out))
    return out, telem


def build_anchored_trial_packed(
    *,
    packed_geoms: Sequence | None,
    candidate_geoms: Sequence | None,
    anchored_indices: Sequence[int],
) -> list:
    """Augment seed packed with anchored nest geoms only (Q361)."""
    trial = [g for g in (packed_geoms or ()) if g is not None]
    if candidate_geoms is None:
        return trial
    for i in anchored_indices:
        ii = int(i)
        if ii < 0 or ii >= len(candidate_geoms):
            continue
        raw = candidate_geoms[ii]
        if raw is None:
            continue
        cg = raw if isinstance(raw, Geometry) else as_geometry(raw)
        if cg is not None:
            trial.append(cg)
    return trial


def motif_join_lock_sets(
    dg,
    graph,
    *,
    max_locks: int = 4,
) -> list[list[int]]:
    """Independent MotifJoin pairs as Scene-fallback lock sets (one MotifJoin SoT)."""
    out: list[list[int]] = []
    seen: set[tuple[int, ...]] = set()
    for m in getattr(dg, "motifs", ()) or ():
        a = int(getattr(m, "a", -1))
        b = int(getattr(m, "b", -1))
        if a < 0 or b < 0 or a == b:
            continue
        sig = _lock_sig([a, b])
        if sig in seen:
            continue
        if not _packing_independent([a, b], graph):
            continue
        seen.add(sig)
        out.append([a, b])
        if len(out) >= max(int(max_locks), 0):
            break
    return out


def pin_if_in_nest(
    lock: Sequence[int],
    selected: Sequence[int],
    graph,
    *,
    propose_stats: dict | None = None,
    source: str = "join_in_nest",
) -> list[int] | None:
    """One ⊆-nest pin gate: lock ⊆ selected + packing-independent → motif_locked.

    ``source`` stamps ``motif_lock_source`` / pin telem (join_in_nest,
    join_after_unlock, void_scene_in_nest).
    """
    lock_idxs = [int(i) for i in lock if int(i) >= 0]
    if len(lock_idxs) < 2:
        return None
    lock_set = set(lock_idxs)
    sel_set = {int(i) for i in selected}
    if not lock_set <= sel_set:
        return None
    if graph is not None and not _packing_independent(lock_idxs, graph):
        return None
    if propose_stats is not None:
        propose_stats["compose_motif_hold"] = 1
        propose_stats["motif_lock_source"] = str(source)
        propose_stats["join_in_nest_pin"] = 1
        if source == "void_scene_in_nest":
            propose_stats["void_scene_in_nest_pin"] = int(
                propose_stats.get("void_scene_in_nest_pin", 0) or 0
            ) + 1
    return list(lock_idxs)


def hybrid_compose_pick(
    *,
    graph,
    lock: Sequence[int],
    area_cand: float,
    area_orig: float,
    void_cand: int,
    void_orig: int,
    cc_n2: int,
    lex_better: bool,
    telem: dict | None = None,
    lock_len: int | None = None,
    join_prefer: bool = False,
    unlocked_void: bool = False,
    count_cand: int | None = None,
    count_orig: int | None = None,
    void_scene: bool = False,
) -> bool:
    """Q362: indep → cc 0.90× → void (0.88× / join_prefer 0.84×) → lex.

    ``unlocked_void`` (Q377) arms the void soft floor without a MotifJoin lock.
    join_prefer soft floor also requires non-decreasing selection count when
    ``count_*`` are provided — except ``void_scene`` (Letter A: drop count gate).
    """
    n_lock = int(lock_len if lock_len is not None else len(lock))
    if telem is not None:
        telem["hybrid_pick_trials"] = int(telem.get("hybrid_pick_trials", 0)) + 1
    if graph is not None and n_lock >= 2 and not _packing_independent(lock, graph):
        if telem is not None:
            telem["hybrid_pick_reject_indep"] = int(
                telem.get("hybrid_pick_reject_indep", 0)
            ) + 1
        return False
    if cc_n2 > 0 and area_cand + 1e-12 >= 0.90 * area_orig:
        if telem is not None:
            telem["hybrid_pick_wins"] = int(telem.get("hybrid_pick_wins", 0)) + 1
        return True
    void_floor = 0.84 if join_prefer else 0.88
    void_gate = (n_lock >= 2) or bool(unlocked_void)
    count_ok = True
    if (
        join_prefer
        and not bool(void_scene)
        and count_cand is not None
        and count_orig is not None
    ):
        count_ok = int(count_cand) >= int(count_orig)
    if (
        void_gate
        and void_cand > void_orig
        and count_ok
        and area_cand + 1e-12 >= void_floor * area_orig
    ):
        if telem is not None:
            telem["hybrid_pick_wins"] = int(telem.get("hybrid_pick_wins", 0)) + 1
            if join_prefer:
                telem["hybrid_pick_join_soft"] = int(
                    telem.get("hybrid_pick_join_soft", 0)
                ) + 1
        return True
    if lex_better:
        if telem is not None:
            telem["hybrid_pick_wins"] = int(telem.get("hybrid_pick_wins", 0)) + 1
        return True
    if telem is not None:
        telem["hybrid_pick_reject_area"] = int(
            telem.get("hybrid_pick_reject_area", 0)
        ) + 1
    return False


def compose_motif_pipeline(
    phase: str,
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
    selected_nest: Sequence[int] | None = None,
    polys: Sequence | None = None,
    sheet=None,
    proposer_keys: Mapping[str, set[tuple[float, float, float]]] | None = None,
    cc_graph_n: int = 0,
    cc_nest_n: int = 0,
    graph_to_nest_hollow: bool = False,
    pole: Point | None = None,
    max_accept: int = 3,
    rcl_top_k: int = 10,
    large_void: bool = False,
    max_locks: int = 4,
    free_poly=None,
) -> MotifComposeResult:
    """Unified pre_nest (soft steer) and post_hollow (anchored Scene locks) compose."""
    telem: dict = {}
    lock_sets: list[list[int]] = []
    boost_idxs: list[int] = []

    if phase == "pre_nest":
        _combined, seq_telem = sequential_accept_motif_cohorts(
            graph=graph,
            scores=scores,
            group_id=group_id,
            transform=transform,
            cohorts=cohorts,
            candidate_geoms=candidate_geoms,
            void_geoms=void_geoms,
            packed_geoms=packed_geoms,
            min_dist=min_dist,
            pole=pole,
            max_accept=max_accept,
            rcl_top_k=rcl_top_k,
            large_void=large_void,
            polys=polys,
            free_poly=free_poly,
        )
        del _combined
        telem.update(seq_telem)
        lock_sets = [list(s) for s in (seq_telem.get("motif_lock_sets") or [])][:4]
        boost_idxs = list(seq_telem.get("motif_packing_score_boost_idxs") or [])
        scene_sz = int(seq_telem.get("motif_scene_max_sz", 0) or 0)
        telem["motif_scene_max_sz_pre"] = scene_sz
        return MotifComposeResult(lock_sets=lock_sets, boost_idxs=boost_idxs, telem=telem)

    if phase != "post_hollow":
        return MotifComposeResult(lock_sets=[], boost_idxs=[], telem=telem)

    trial_packed = list(packed_geoms or ())
    if selected_nest and polys is not None and sheet is not None:
        anchored, anc_telem = anchored_nest_indices(
            selected_nest=selected_nest,
            polys=polys,
            candidate_geoms=candidate_geoms,
            sheet=sheet,
            min_dist=min_dist,
            packed_geoms=packed_geoms,
        )
        telem.update(anc_telem)
        trial_packed = build_anchored_trial_packed(
            packed_geoms=packed_geoms,
            candidate_geoms=candidate_geoms,
            anchored_indices=anchored,
        )

    hollow_locks, hollow_telem = resolve_hollow_cc_lock_sets(
        graph=graph,
        group_id=group_id,
        transform=transform,
        proposer_keys=proposer_keys,
        cohorts=cohorts,
        cc_graph_n=int(cc_graph_n),
        cc_nest_n=int(cc_nest_n),
        graph_to_nest_hollow=bool(graph_to_nest_hollow),
        candidate_geoms=candidate_geoms,
        void_geoms=void_geoms,
        packed_geoms=trial_packed,
        min_dist=min_dist,
        scores=scores,
        pole=pole,
        max_locks=max_locks,
    )
    telem.update(hollow_telem)
    # Hollow Scene vs full anchored nest often rejects void motifs; retry Scene
    # against board packed only (still is_pose_clear — not packing-clear).
    if (
        not hollow_locks
        and bool(graph_to_nest_hollow)
        and trial_packed is not None
        and list(trial_packed) != list(packed_geoms or ())
    ):
        board_locks, board_telem = resolve_hollow_cc_lock_sets(
            graph=graph,
            group_id=group_id,
            transform=transform,
            proposer_keys=proposer_keys,
            cohorts=cohorts,
            cc_graph_n=int(cc_graph_n),
            cc_nest_n=int(cc_nest_n),
            graph_to_nest_hollow=True,
            candidate_geoms=candidate_geoms,
            void_geoms=void_geoms,
            packed_geoms=list(packed_geoms or ()),
            min_dist=min_dist,
            scores=scores,
            pole=pole,
            max_locks=max_locks,
        )
        telem["hollow_lock_board_retry"] = 1
        for k, v in board_telem.items():
            if k in ("hollow_pattern_locks", "hollow_flat_pair_locks", "hollow_cc_gap"):
                telem[k] = max(int(telem.get(k, 0) or 0), int(v or 0))
            elif k == "motif_scene_max_sz":
                telem[k] = max(int(telem.get(k, 0) or 0), int(v or 0))
        if board_locks:
            hollow_locks = board_locks
            telem["hollow_lock_board_hit"] = 1
    if (
        not hollow_locks
        and bool(graph_to_nest_hollow)
        and int(hollow_telem.get("hollow_cc_gap", 0) or 0) > 0
    ):
        # Scene growing against anchored nest blocks all void pairs; retry void
        # solids only (still is_pose_clear — not packing-clear lock).
        void_locks, void_telem = resolve_hollow_cc_lock_sets(
            graph=graph,
            group_id=group_id,
            transform=transform,
            proposer_keys=proposer_keys,
            cohorts=cohorts,
            cc_graph_n=int(cc_graph_n),
            cc_nest_n=int(cc_nest_n),
            graph_to_nest_hollow=True,
            candidate_geoms=candidate_geoms,
            void_geoms=void_geoms,
            packed_geoms=[],
            min_dist=min_dist,
            scores=scores,
            pole=pole,
            max_locks=max_locks,
        )
        telem["hollow_lock_void_retry"] = 1
        for k, v in void_telem.items():
            if k in ("hollow_pattern_locks", "hollow_flat_pair_locks", "motif_scene_max_sz"):
                telem[k] = max(int(telem.get(k, 0) or 0), int(v or 0))
        if void_locks:
            preferred, n_core = prefer_void_core_locks(
                void_locks, polys, free_poly,
            )
            telem["hollow_void_core_locks"] = int(n_core)
            hollow_locks = preferred
            telem["hollow_lock_void_hit"] = 1
    lock_sets = hollow_locks
    scene_sz = int(telem.get("motif_scene_max_sz", 0) or 0)
    telem["motif_scene_max_sz_post"] = scene_sz
    return MotifComposeResult(lock_sets=lock_sets, boost_idxs=boost_idxs, telem=telem)


def resolve_hollow_cc_lock_sets(
    *,
    graph,
    group_id: Sequence[int],
    transform: Sequence,
    proposer_keys: Mapping[str, set[tuple[float, float, float]]] | None,
    cohorts: Sequence[dict] | None,
    cc_graph_n: int,
    cc_nest_n: int,
    graph_to_nest_hollow: bool,
    candidate_geoms: Sequence | None,
    void_geoms: Sequence | None,
    packed_geoms: Sequence | None,
    min_dist: float,
    scores: Sequence[float] | None = None,
    pole: Point | None = None,
    max_locks: int = 4,
) -> tuple[list[list[int]], dict]:
    """One gate: build pattern lock sets when cc gap + hollow basin."""
    telem: dict = {"hollow_cc_gap": 0}
    if not graph_to_nest_hollow or int(cc_nest_n) > 0:
        return [], telem
    key_map = pose_key_to_index(group_id, transform)
    cc_pool = cluster_copy_lock_pool(
        group_id, transform, proposer_keys, cohorts, key_map,
    )
    if len(cc_pool) < 2:
        return [], telem
    if int(cc_graph_n) < 2 and len(cc_pool) < 2:
        return [], telem
    telem["hollow_cc_gap"] = 1
    lock_sets, pat_telem = hollow_pattern_lock_sets(
        graph=graph,
        cohorts=cohorts,
        cc_pool=cc_pool,
        key_map=key_map,
        candidate_geoms=candidate_geoms,
        void_geoms=void_geoms,
        packed_geoms=packed_geoms,
        min_dist=min_dist,
        scores=scores,
        pole=pole,
        max_locks=max_locks,
    )
    telem.update(pat_telem)
    return lock_sets, telem


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
    large_void: bool = False,
    polys: Sequence | None = None,
    free_poly=None,
) -> tuple[list[int], dict]:
    """Pre-MIS: accept full motif cohorts under growing Scene clear.

    Skip cohorts with missing in-graph followers. Deterministic pole top-k (v1).
    """
    telem: dict[str, Any] = {
        "motif_sequential_full": 0,
        "motif_sequential_skipped_missing": 0,
        "motif_sequential_partial": 0,
        "motif_sequential_rcl": 0,
        "motif_sequential_clear_fail": 0,
        "motif_sequential_packing_clear": 0,
        "motif_packing_score_boost_idxs": [],
        "motif_lock_sets": [],
        "motif_beam_sets": 0,
        "motif_scene_max_sz": 0,
        "motif_pack_max_sz": 0,
        "cohort_void_rank_n": 0,
    }
    if not cohorts:
        return [], telem
    key_map = pose_key_to_index(group_id, transform)
    rank_polys = polys if polys is not None else candidate_geoms
    rcl = _rank_cohorts(
        list(cohorts),
        key_map,
        scores,
        pole,
        polys=rank_polys,
        free_poly=free_poly,
    )[: max(int(rcl_top_k), 1)]
    if (
        free_poly is not None
        and not getattr(free_poly, "is_empty", True)
        and rank_polys is not None
    ):
        telem["cohort_void_rank_n"] = int(len(rcl))
    telem["motif_sequential_rcl"] = len(rcl)

    voids = [g for g in (void_geoms or []) if g is not None]
    base_packed: list = [g for g in (packed_geoms or []) if g is not None]
    growing_packed: list = list(base_packed)
    beam_cap = 4
    min_dist_f = float(min_dist)
    scene_accept = _scene_accept_factory(void_geoms, min_dist)

    def _pack_accept(cg: Geometry, trial: list) -> bool:
        return clear_of_geoms(cg, placement_obstacles(voids or [], trial), 0.0)

    locked: list[int] = []
    locked_set: set[int] = set()
    lock_sets: list[list[int]] = []
    accepted = 0
    for cohort in rcl:
        idxs, missing = cohort_member_indices(cohort, key_map)
        if len(idxs) < 2:
            telem["motif_sequential_skipped_missing"] += 1
            continue
        if missing:
            telem["motif_sequential_partial"] = int(
                telem.get("motif_sequential_partial", 0)
            ) + 1
        if not _packing_independent(idxs, graph):
            continue
        if len(lock_sets) < beam_cap:
            beam_idxs, _beam_geoms, _ = _growing_subset_indices(
                idxs, candidate_geoms, base_packed, scene_accept,
            )
            if len(beam_idxs) >= 2:
                lock_sets.append(beam_idxs)
                telem["motif_scene_max_sz"] = max(
                    int(telem.get("motif_scene_max_sz", 0)), len(beam_idxs),
                )

        if accepted >= int(max_accept):
            continue
        if any(i in locked_set for i in idxs):
            continue
        collisions = getattr(graph, "collisions", None)
        if collisions is not None:
            blocked = False
            for i in idxs:
                if any(int(u) in locked_set for u in collisions[i]):
                    blocked = True
                    break
            if blocked:
                continue
        scene_idxs, member_geoms, _ = _growing_subset_indices(
            idxs, candidate_geoms, growing_packed, scene_accept,
        )
        if len(scene_idxs) >= 2:
            if len(scene_idxs) < len(idxs):
                telem["motif_sequential_partial"] = int(
                    telem.get("motif_sequential_partial", 0)
                ) + 1
            if scene_idxs and not telem.get("motif_beam_seeded"):
                if scene_idxs not in lock_sets:
                    lock_sets.insert(0, list(scene_idxs))
                telem["motif_beam_seeded"] = 1
                lock_sets[:] = lock_sets[:beam_cap]
            for i in scene_idxs:
                locked.append(i)
                locked_set.add(i)
            growing_packed.extend(member_geoms)
            accepted += 1
            telem["motif_sequential_full"] += 1
            continue
        if large_void:
            pack_idxs, _pack_geoms, _ = _growing_subset_indices(
                idxs, candidate_geoms, growing_packed, _pack_accept,
            )
            if len(pack_idxs) >= 2:
                telem["motif_sequential_packing_clear"] = int(
                    telem.get("motif_sequential_packing_clear", 0)
                ) + 1
                telem["motif_pack_max_sz"] = max(
                    int(telem.get("motif_pack_max_sz", 0)), len(pack_idxs),
                )
                boost_list = list(telem.get("motif_packing_score_boost_idxs") or [])
                for ix in pack_idxs:
                    if ix not in boost_list:
                        boost_list.append(int(ix))
                telem["motif_packing_score_boost_idxs"] = boost_list
                continue
        telem["motif_sequential_clear_fail"] += 1
    telem["motif_lock_sets"] = lock_sets
    telem["motif_beam_sets"] = len(lock_sets)
    return locked, telem
