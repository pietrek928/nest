"""Block replace: 3a motif-cohort lock-swap and 3b contact-CC hole re-nest."""

from typing import Sequence

import numpy as np
from shapely import Point, Polygon
from shapely.geometry.base import BaseGeometry
from shapely.ops import unary_union

from nest_graph.elem_graph import SelectMode, SelectOptions, nest_by_scores
from nest_graph.geometry import Geometry
from nest_graph.propose.context import cluster_packed_indices
from nest_graph.propose.first_pass_border import build_elem_graph
from nest_graph.propose.placement_common import (
    as_geometry,
    is_board_adj,
    is_pose_clear,
    part_void_adj,
    selection_pairwise_independent,
)
from nest_graph.propose.placement_outline import outline_ring_geom
from nest_graph.propose.void_selection import _sel_area, pose_key_to_index
from nest_graph.proposer_names import ProposerName
from nest_graph.utils import transform_poly

_HOLE_PROPOSERS = frozenset({ProposerName.CLUSTER_COPY, ProposerName.POCKET_FIT})
_HOLE_EMIT_CAP = 128
_COHORT_RELATED_CAP = 8


def _cohort_member_indices(
    cohort: dict,
    key_map: dict[tuple[int, tuple[float, float, float]], int],
) -> list[int]:
    idxs: list[int] = []
    for item in cohort.get("member_keys") or []:
        if not (isinstance(item, (list, tuple)) and len(item) == 2):
            continue
        gid_m, key_m = int(item[0]), item[1]
        key_t = tuple(key_m) if not isinstance(key_m, tuple) else key_m
        ix = key_map.get((gid_m, key_t))
        if ix is not None:
            idxs.append(int(ix))
    return idxs


def lex_count_area_better(
    *,
    old_count: int,
    old_area: float,
    new_count: int,
    new_area: float,
) -> bool:
    if new_count > old_count:
        return True
    if new_count < old_count:
        return False
    return new_area > old_area + 1e-9


def _cohorts_collide(a: Sequence[int], b: Sequence[int], graph) -> bool:
    a_set = set(int(i) for i in a)
    collisions = getattr(graph, "collisions", None)
    if collisions is None:
        return False
    for j in b:
        jj = int(j)
        if jj < 0 or jj >= len(collisions):
            continue
        if any(int(u) in a_set for u in collisions[jj]):
            return True
    return False


def _packing_independent(idxs: Sequence[int], graph) -> bool:
    collisions = getattr(graph, "collisions", None)
    if collisions is None:
        return True
    s = set(int(i) for i in idxs)
    for i in s:
        if i < 0 or i >= len(collisions):
            continue
        if any(int(u) in s and int(u) != i for u in collisions[i]):
            return False
    return True


def try_block_cohort_swap(
    *,
    graph,
    scores: Sequence[float],
    selected: Sequence[int],
    locked_motif: Sequence[int],
    cohorts: Sequence[dict] | None,
    candidate_geoms: Sequence | None,
    void_geoms: Sequence | None,
    group_id: Sequence[int],
    transform: Sequence,
    part_areas: Sequence[float],
    min_dist: float,
    max_related: int = _COHORT_RELATED_CAP,
) -> tuple[list[int], list[int], dict]:
    """3a: trial-eject pinned cohort A for colliding in-graph cohort B."""
    telem = {
        "block_cohort_tried": 0,
        "block_cohort_related": 0,
        "block_cohort_accepted": 0,
        "block_cohort_dcount": 0,
        "block_cohort_darea": 0.0,
    }
    sel = [int(i) for i in selected]
    locked = [int(i) for i in locked_motif]
    if not sel or not locked or not cohorts:
        return sel, locked, telem
    key_map = pose_key_to_index(group_id, transform)
    resolved: list[list[int]] = []
    for cohort in cohorts:
        idxs = _cohort_member_indices(cohort, key_map)
        if len(idxs) >= 2:
            resolved.append(idxs)
    if not resolved:
        return sel, locked, telem

    sel_set = set(sel)
    locked_set = set(locked)
    pinned: list[list[int]] = [
        a for a in resolved if all(i in locked_set for i in a) and len(a) >= 2
    ]
    voids = [g for g in (void_geoms or ()) if g is not None]
    old_area = _sel_area(sel, group_id, part_areas)
    related = 0
    for a in pinned:
        a_set = set(a)
        for b in resolved:
            if b is a or len(b) < 2:
                continue
            if set(b) <= sel_set:
                continue
            if not _cohorts_collide(a, b, graph):
                continue
            related += 1
            if related > int(max_related):
                telem["block_cohort_related"] = related
                return sel, locked, telem
            telem["block_cohort_related"] = related
            kept_sel = [i for i in sel if i not in a_set]
            kept_geoms: list = []
            for i in kept_sel:
                if candidate_geoms is None or i >= len(candidate_geoms):
                    continue
                cg = as_geometry(candidate_geoms[i])
                if cg is not None:
                    kept_geoms.append(cg)
            scene_ok = True
            for j in b:
                if candidate_geoms is None or j >= len(candidate_geoms):
                    scene_ok = False
                    break
                cg = as_geometry(candidate_geoms[j])
                if cg is None or not is_pose_clear(cg, voids, kept_geoms, float(min_dist)):
                    scene_ok = False
                    break
                kept_geoms.append(cg)
            if not scene_ok:
                continue
            lock_set = (set(sel) - a_set) | set(b)
            if not _packing_independent(list(lock_set), graph):
                continue
            telem["block_cohort_tried"] += 1
            opts = SelectOptions()
            opts.local_swap = False
            opts.mode = SelectMode.greedy_score
            opts.locked_indices = [int(i) for i in sorted(lock_set)]
            new_sel = list(nest_by_scores(graph, list(scores), opts))
            new_area = _sel_area(new_sel, group_id, part_areas)
            if not lex_count_area_better(
                old_count=len(sel),
                old_area=old_area,
                new_count=len(new_sel),
                new_area=new_area,
            ):
                continue
            telem["block_cohort_accepted"] = 1
            telem["block_cohort_dcount"] = len(new_sel) - len(sel)
            telem["block_cohort_darea"] = float(new_area - old_area)
            new_locked = sorted((set(locked) - a_set) | set(b))
            return new_sel, new_locked, telem
    telem["block_cohort_related"] = related
    return sel, locked, telem


def pick_block_hole_victim(
    selected: Sequence[int],
    polys: Sequence[BaseGeometry],
    *,
    min_dist: float,
    sheet: Polygon,
    pole: Point | None,
    void_poly=None,
    min_size: int = 3,
    max_size: int = 6,
    allow_board_adj_fallback: bool = True,
) -> list[int] | None:
    """Contact CC of size [3,6]; prefer non-board_adj; Q215 void-facing board_adj only."""
    sel = [int(i) for i in selected if 0 <= int(i) < len(polys)]
    placed = [polys[i] for i in sel]
    if len(sel) < min_size:
        return None
    groups = cluster_packed_indices(placed, min_dist, sheet=sheet)
    ring = outline_ring_geom(sheet)
    best_interior: list[int] | None = None
    best_interior_d = float("inf")
    best_rim: list[int] | None = None
    best_rim_d = float("inf")
    for local in groups:
        if not (min_size <= len(local) <= max_size):
            continue
        global_idxs = [sel[j] for j in local]
        board_hit = False
        for i in global_idxs:
            poly = polys[i]
            if poly is not None and not getattr(poly, "is_empty", True) and is_board_adj(
                poly, sheet, min_dist, ring=ring,
            ):
                board_hit = True
                break
        members = [polys[i] for i in global_idxs if polys[i] is not None]
        if not members:
            continue
        blob = unary_union(members) if len(members) > 1 else members[0]
        if blob is None or blob.is_empty:
            continue
        if pole is None or getattr(pole, "is_empty", True):
            dist = 0.0
        else:
            dist = float(blob.centroid.distance(pole))
        if not board_hit:
            if dist < best_interior_d:
                best_interior_d = dist
                best_interior = list(global_idxs)
        elif allow_board_adj_fallback and dist < best_rim_d:
            # Q215: board_adj only if void-facing (pure rim peel is waste on void_fill).
            void_facing = any(
                part_void_adj(polys[i], void_poly, min_dist) for i in global_idxs
            )
            if not void_facing:
                continue
            best_rim_d = dist
            best_rim = list(global_idxs)
    if best_interior is not None:
        return best_interior
    if allow_board_adj_fallback:
        return best_rim
    return None


def _emit_hole_candidates(
    *,
    sheet: Polygon,
    kept_geoms: Sequence[BaseGeometry],
    part_by_group: dict[int, Polygon],
    min_dist: float,
    propose_cfg,
    pole: Point | None,
    void_poly,
    void_geoms: Sequence,
    free_space,
    cluster_patterns,
) -> tuple[list[tuple[int, np.ndarray, BaseGeometry, Geometry]], int]:
    from nest_graph.config import ProposeConfig
    from nest_graph.propose.pipeline import propose_coords_with_strategy

    kept_list = [p for p in kept_geoms if p is not None and not getattr(p, "is_empty", True)]
    kept_union = unary_union(kept_list) if kept_list else Polygon()
    push = pole if pole is not None else sheet.centroid
    zone_cfg = ProposeConfig.for_place("void_seek", base=propose_cfg)
    enabled = _HOLE_PROPOSERS
    packed_g = [_as_geom(p) for p in kept_list]
    packed_g = [g for g in packed_g if g is not None]
    voids = [g for g in (void_geoms or ()) if g is not None]
    out: list[tuple[int, np.ndarray, BaseGeometry, Geometry]] = []
    in_hull = 0
    hull = void_poly if void_poly is not None and not getattr(void_poly, "is_empty", True) else None
    for gid, part in part_by_group.items():
        if len(out) >= _HOLE_EMIT_CAP:
            break
        if part is None or getattr(part, "is_empty", True):
            continue
        try:
            coords = propose_coords_with_strategy(
                kept_union,
                part,
                sheet,
                zone_cfg,
                min_dist=min_dist,
                pt_push=push,
                group_id=int(gid),
                enabled_proposers=enabled,
                cluster_patterns=cluster_patterns,
                packed_polys=kept_list,
                packed_group_ids=[0] * len(kept_list),
                packed_transforms=[(0.0, 0.0, 0.0)] * len(kept_list),
                full_packed_geoms=packed_g,
                cascade_zone="void_seek",
                void_pole=pole,
                free_space=free_space,
            )
        except Exception:
            coords = []
        part_g = Geometry.from_shapely(part)
        for c in coords:
            if len(out) >= _HOLE_EMIT_CAP:
                break
            tr = np.asarray(c, dtype=np.float64).reshape(3)
            cand_g = part_g.apply_transform(float(tr[0]), float(tr[1]), float(tr[2]))
            if not is_pose_clear(cand_g, voids, packed_g, float(min_dist)):
                continue
            poly = transform_poly(part, tr)
            if hull is not None:
                try:
                    if hull.contains(poly.centroid) or float(poly.distance(hull)) <= float(min_dist) * 2.0:
                        in_hull += 1
                except Exception:
                    pass
            else:
                in_hull += 1
            out.append((int(gid), tr, poly, cand_g))
    return out, in_hull


def _as_geom(p) -> Geometry | None:
    return as_geometry(p)


def try_block_hole_renest(
    *,
    selected: Sequence[int],
    polys: list,
    transforms: list,
    group_id: list,
    candidate_geoms: list | None,
    scores: Sequence[float],
    part_areas: Sequence[float],
    part_by_group: dict[int, Polygon],
    sheet: Polygon,
    min_dist: float,
    propose_cfg,
    pole: Point | None,
    void_poly,
    void_geoms: Sequence | None,
    free_space=None,
    cluster_patterns=None,
    victim_override: Sequence[int] | None = None,
) -> tuple[list[int], list, list, list, list | None, dict]:
    """3b: peel contact CC, re-emit into the hole, nest_by_scores locked=kept.

    Returns (selection, polys, transforms, group_id, candidate_geoms, telem).
    On reject, inputs are unchanged. ``block_hole_victim`` is set whenever a
    victim was chosen so stamp can reuse it.
    """
    telem = {
        "block_hole_tried": 0,
        "block_hole_accepted": 0,
        "block_hole_emit_in_hull": 0,
        "block_hole_victim": None,
        "block_hole_dcount": 0,
        "block_hole_darea": 0.0,
    }
    sel = [int(i) for i in selected]
    if victim_override:
        victim = [int(i) for i in victim_override]
    else:
        victim = pick_block_hole_victim(
            sel, polys, min_dist=min_dist, sheet=sheet, pole=pole, void_poly=void_poly,
        )
    if not victim:
        return sel, polys, transforms, group_id, candidate_geoms, telem
    telem["block_hole_victim"] = list(victim)
    telem["block_hole_tried"] = 1
    victim_set = set(victim)
    kept = [i for i in sel if i not in victim_set]
    kept_geoms = [
        polys[i] for i in kept
        if 0 <= i < len(polys) and polys[i] is not None and not polys[i].is_empty
    ]
    hole_hull = unary_union([
        polys[i] for i in victim
        if 0 <= i < len(polys) and polys[i] is not None and not polys[i].is_empty
    ])
    emit_hull = hole_hull if (hole_hull is not None and not hole_hull.is_empty) else void_poly
    cands, in_hull = _emit_hole_candidates(
        sheet=sheet,
        kept_geoms=kept_geoms,
        part_by_group=part_by_group,
        min_dist=min_dist,
        propose_cfg=propose_cfg,
        pole=pole,
        void_poly=emit_hull,
        void_geoms=void_geoms or (),
        free_space=free_space,
        cluster_patterns=cluster_patterns,
    )
    telem["block_hole_emit_in_hull"] = int(in_hull)
    if in_hull <= 0 or not cands:
        return sel, polys, transforms, group_id, candidate_geoms, telem

    kept_native: list[Geometry] = []
    kept_angles: list[float] = []
    kept_gids: list[int] = []
    kept_used: list[int] = []
    for i in kept:
        g = None
        if candidate_geoms is not None and i < len(candidate_geoms):
            g = as_geometry(candidate_geoms[i])
        if g is None and 0 <= i < len(polys):
            g = as_geometry(polys[i])
        if g is None:
            continue
        kept_native.append(g)
        kept_used.append(i)
        kept_gids.append(int(group_id[i]) if i < len(group_id) else 0)
        if i < len(transforms):
            kept_angles.append(float(np.asarray(transforms[i]).reshape(3)[2]))
        else:
            kept_angles.append(0.0)
    hole_gids = [gid for gid, _tr, _p, _g in cands]
    hole_geoms = [g for _gid, _tr, _p, g in cands]
    hole_angles = [float(tr[2]) for _gid, tr, _p, _g in cands]
    mini_gids = kept_gids + hole_gids
    mini_geoms = kept_native + hole_geoms
    mini_angles = kept_angles + hole_angles
    if len(mini_geoms) < 2:
        return sel, polys, transforms, group_id, candidate_geoms, telem
    graph = build_elem_graph(mini_gids, mini_geoms, mini_angles, attract_pairs=[])
    n_kept = len(kept_native)
    mini_scores = []
    for i, gid in enumerate(mini_gids):
        if i < n_kept:
            src = kept_used[i] if i < len(kept_used) else -1
            if 0 <= src < len(scores):
                mini_scores.append(float(scores[src]))
            elif 0 <= gid < len(part_areas):
                mini_scores.append(float(part_areas[gid]))
            else:
                mini_scores.append(0.0)
        elif 0 <= gid < len(part_areas):
            mini_scores.append(float(part_areas[gid]))
        else:
            mini_scores.append(0.0)
    opts = SelectOptions()
    opts.local_swap = False
    opts.mode = SelectMode.greedy_score
    opts.locked_indices = list(range(n_kept))
    new_mini = list(nest_by_scores(graph, mini_scores, opts))
    new_sel: list[int] = []
    append_gids: list[int] = []
    append_tr: list[np.ndarray] = []
    append_poly: list[BaseGeometry] = []
    append_geom: list[Geometry] = []
    for mi in new_mini:
        if mi < n_kept:
            new_sel.append(kept_used[mi])
        else:
            h = mi - n_kept
            if 0 <= h < len(cands):
                gid, tr, poly, geom = cands[h]
                append_gids.append(gid)
                append_tr.append(tr)
                append_poly.append(poly)
                append_geom.append(geom)
    base = len(polys)
    new_polys = list(polys)
    new_tr = list(transforms)
    new_gid = list(group_id)
    new_cgeoms = list(candidate_geoms) if candidate_geoms is not None else None
    for k, (gid, tr, poly, geom) in enumerate(zip(
        append_gids, append_tr, append_poly, append_geom, strict=True,
    )):
        new_sel.append(base + k)
        new_polys.append(poly)
        new_tr.append(tr)
        new_gid.append(gid)
        if new_cgeoms is not None:
            new_cgeoms.append(geom)
    old_area = _sel_area(sel, group_id, part_areas)
    new_area = _sel_area(new_sel, new_gid, part_areas)
    if not lex_count_area_better(
        old_count=len(sel),
        old_area=old_area,
        new_count=len(new_sel),
        new_area=new_area,
    ):
        return sel, polys, transforms, group_id, candidate_geoms, telem
    if not selection_pairwise_independent(new_polys, new_sel):
        return sel, polys, transforms, group_id, candidate_geoms, telem
    # Q215/Q216: board_adj soft-gate lives in pick_block_hole_victim (void-facing
    # fallback only; interior CC preferred). Do not reject after lex+indep — that
    # killed dense interior 3b and still lost void vs F3 OFF (which keeps 3b_ok).
    telem["block_hole_accepted"] = 1
    telem["block_hole_dcount"] = len(new_sel) - len(sel)
    telem["block_hole_darea"] = float(new_area - old_area)
    return new_sel, new_polys, new_tr, new_gid, new_cgeoms, telem


def maybe_block_hole_renest(
    *,
    selected: Sequence[int],
    polys: list,
    transforms: list,
    group_id: list,
    candidate_geoms: list | None,
    refine_scores: Sequence[float],
    part_areas: Sequence[float],
    part_by_group: dict[int, Polygon],
    sheet: Polygon,
    min_dist: float,
    propose_cfg,
    free_info,
    free_poly,
    void_geoms: Sequence | None,
    cluster_patterns=None,
    polish_budget,
    propose_stats: dict,
) -> tuple[list[int], list, list, list, list | None, dict]:
    """Gate + call 3b hole re-nest (mid-pack, after refine)."""
    propose_stats.setdefault("block_hole_accepted", 0)
    propose_stats.setdefault("block_hole_emit_in_hull", 0)
    propose_stats.setdefault("block_hole_victim", None)
    propose_stats.setdefault("block_hole_tried", 0)
    skip_3b = int(propose_stats.get("block_cohort_accepted", 0) or 0) > 0
    if (
        not bool(getattr(polish_budget, "run_3b", False))
        or skip_3b
        or not bool(getattr(propose_cfg, "enable_lns_rebuild", True))
        or not selected
        or free_info is None
        or getattr(free_info, "kind", None) != "large_void"
    ):
        return list(selected), polys, transforms, group_id, candidate_geoms, {}
    from nest_graph.propose.repair_cohort import (
        REPAIR_EMIT,
        REPAIR_EMIT_OK,
        REPAIR_STAMP,
        begin_repair_cohort,
    )

    cohort = begin_repair_cohort(
        selected,
        list(polys),
        list(group_id),
        list(transforms),
        min_dist=float(min_dist),
        sheet=sheet,
        pole=getattr(free_info, "target_pt", None),
        void_poly=free_poly,
        archived=list(cluster_patterns or ()),
        max_patterns=int(getattr(propose_cfg, "cluster_copy_max_patterns", 8) or 8),
        min_size=3,
        max_size=6,
    )
    patterns_for_emit = cohort.patterns if cohort.patterns else cluster_patterns
    (
        sel_out,
        polys_out,
        tr_out,
        gid_out,
        cgeom_out,
        hole_telem,
    ) = try_block_hole_renest(
        selected=selected,
        polys=list(polys),
        transforms=list(transforms),
        group_id=list(group_id),
        candidate_geoms=list(candidate_geoms) if candidate_geoms is not None else None,
        scores=refine_scores,
        part_areas=part_areas,
        part_by_group=part_by_group,
        sheet=sheet,
        min_dist=min_dist,
        propose_cfg=propose_cfg,
        pole=getattr(free_info, "target_pt", None),
        void_poly=free_poly,
        void_geoms=void_geoms,
        cluster_patterns=patterns_for_emit,
        victim_override=cohort.victim,
    )
    propose_stats.update(hole_telem)
    cohort.emit_in_hull = int(hole_telem.get("block_hole_emit_in_hull", 0) or 0)
    cohort.accepted = int(hole_telem.get("block_hole_accepted", 0) or 0) > 0
    if cohort.accepted:
        cohort.mode = REPAIR_EMIT_OK
    elif cohort.victim is not None and cohort.emit_in_hull <= 0:
        cohort.mode = REPAIR_STAMP
    elif cohort.victim is not None:
        cohort.mode = REPAIR_EMIT
    cohort.telem_into(propose_stats)
    propose_stats["_repair_patterns"] = list(cohort.patterns)
    return list(sel_out), polys_out, tr_out, gid_out, cgeom_out, hole_telem
