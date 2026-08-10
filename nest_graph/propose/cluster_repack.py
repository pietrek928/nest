"""Post-DFS board-peer cluster peel, motif stamp, and floating relocate."""

import heapq
import math
from collections import Counter
from typing import Sequence

import numpy as np
from shapely import Point, Polygon
from shapely.geometry.base import BaseGeometry
from shapely.ops import unary_union

from nest_graph.board import board_context_from_geometry
from nest_graph.config import ProposeConfig
from nest_graph.geometry import Geometry
from nest_graph.propose.placement_common import (
    as_geometry as _as_geometry,
    is_pose_clear,
    selection_pairwise_independent,
)
from nest_graph.propose.selection_edit import SelectionEditCtx
from nest_graph.propose.context import (
    _cluster_merge_gap,
    cluster_contact_components,
    cluster_contact_neighbors,
    cluster_packed_indices,
    void_pole_seed_coords,
)
from nest_graph.propose.geometry import ProposeGeometry
from nest_graph.propose.placement_outline import outline_ring_geom
from nest_graph.propose.placements_pattern import (
    ClusterPattern,
    dedupe_anchors,
    free_pocket_anchors,
    void_seek_motif_anchors,
)
from nest_graph.propose.placements_pocket import aligned_poses_for_pocket
from nest_graph.propose.pipeline import propose_coords_with_strategy
from nest_graph.propose.ranking import score_placement_tightness
from nest_graph.utils import compose_transforms, relative_transform, transform_poly


def _obstacle_geoms(shape) -> list[Geometry]:
    """Explode a Shapely union / part into Geometry obstacles (no clearance union)."""
    if shape is None:
        return []
    if isinstance(shape, Geometry):
        return [shape]
    if hasattr(shape, "is_empty") and shape.is_empty:
        return []
    if hasattr(shape, "geoms"):
        out: list[Geometry] = []
        for g in shape.geoms:
            og = _as_geometry(g)
            if og is not None:
                out.append(og)
        return out
    og = _as_geometry(shape)
    return [og] if og is not None else []


def cluster_indices_with_board(
    polys: Sequence[BaseGeometry],
    min_dist: float,
    sheet: Polygon,
) -> list[tuple[list[int], bool]]:
    """Contact clusters among parts + board ring; return (idxs, board_adj)."""
    placed_idx = [i for i, p in enumerate(polys) if p is not None and not p.is_empty]
    if not placed_idx:
        return []
    parts = [polys[i] for i in placed_idx]
    gap = _cluster_merge_gap(parts, min_dist, sheet)
    geoms = [
        p if isinstance(p, Geometry) else Geometry.from_shapely(p)
        for p in parts
    ]
    board_ring = outline_ring_geom(sheet)
    comps = cluster_contact_components(
        geoms,
        gap,
        board_ring=board_ring,
        # exterior.buffer(min_dist) + mutual buffer(gap) ≡ standoff ≤ min_dist+2*gap
        board_gap=float(min_dist) + 2.0 * gap,
    )
    return [([placed_idx[j] for j in members], board_adj) for members, board_adj in comps]


def _part_void_adj(
    poly: BaseGeometry,
    void_poly: Polygon | None,
    min_dist: float,
) -> bool:
    if void_poly is None or void_poly.is_empty or poly is None or poly.is_empty:
        return False
    try:
        return (
            poly.intersects(void_poly)
            or float(poly.distance(void_poly)) <= float(min_dist) * 2.0
        )
    except Exception:
        return False


def bfs_peel_victim(
    selected_indices: Sequence[int],
    polys: Sequence[BaseGeometry],
    *,
    min_dist: float,
    sheet: Polygon,
    pole: Point | None,
    void_poly: Polygon | None,
    min_size: int = 3,
    max_size: int = 6,
) -> tuple[list[int], bool] | None:
    """Contact-BFS peel of size [min_size, max_size] from a board/void mega-cluster.

    Returns ``(global_indices, board_adj)`` or None.
    """
    idxs = [int(i) for i in selected_indices if polys[i] is not None and not polys[i].is_empty]
    if len(idxs) < min_size:
        return None
    selected_polys = [polys[i] for i in idxs]
    # Map local → global after board-aware cluster on selected subset.
    local_groups = cluster_indices_with_board(selected_polys, min_dist, sheet)
    gap = _cluster_merge_gap(selected_polys, min_dist, sheet)

    candidates: list[tuple[float, list[int], bool]] = []
    for local_members, board_adj in local_groups:
        global_comp = [idxs[j] for j in local_members]
        if len(global_comp) < min_size:
            continue
        # Prefer void∧board, then void, then board, then largest.
        void_hits = sum(
            1 for i in global_comp if _part_void_adj(polys[i], void_poly, min_dist)
        )
        prefer = 0.0
        if void_hits and board_adj:
            prefer = 0.0
        elif void_hits:
            prefer = 1.0
        elif board_adj:
            prefer = 2.0
        else:
            prefer = 3.0
        # Seed: void_adj preferably board-touching; else board closest to pole.
        seeds = [
            i for i in global_comp
            if _part_void_adj(polys[i], void_poly, min_dist)
        ]
        if not seeds and board_adj:
            seeds = list(global_comp)
        if not seeds:
            continue
        if pole is not None and not pole.is_empty:
            seeds.sort(key=lambda i: float(polys[i].centroid.distance(pole)))
        seed = seeds[0]
        adj = cluster_contact_neighbors(global_comp, polys, gap)
        # Priority BFS: expand by contact, prefer closer to pole.
        peel = _priority_bfs_peel(seed, adj, polys, pole, max_size)
        if len(peel) < min_size:
            continue
        peel = peel[:max_size]
        blob_dist = (
            float(unary_union([polys[i] for i in peel]).centroid.distance(pole))
            if pole is not None and not pole.is_empty
            else 0.0
        )
        candidates.append((prefer * 1e6 + blob_dist, peel, board_adj))

    if not candidates:
        return None
    candidates.sort(key=lambda x: x[0])
    _, peel, board_adj = candidates[0]
    return peel, board_adj


def _priority_bfs_peel(
    seed: int,
    adj: dict[int, list[int]],
    polys: Sequence[BaseGeometry],
    pole: Point | None,
    max_size: int,
) -> list[int]:
    def _prio(i: int) -> float:
        if pole is None or pole.is_empty:
            return 0.0
        return float(polys[i].centroid.distance(pole))

    visited = {seed}
    peel = [seed]
    # heap of (dist, counter, node)
    counter = 0
    heap: list[tuple[float, int, int]] = []
    for n in adj.get(seed, []):
        heapq.heappush(heap, (_prio(n), counter, n))
        counter += 1
    while heap and len(peel) < max_size:
        _, _, node = heapq.heappop(heap)
        if node in visited:
            continue
        visited.add(node)
        peel.append(node)
        for n in adj.get(node, []):
            if n not in visited:
                heapq.heappush(heap, (_prio(n), counter, n))
                counter += 1
    return peel


# Back-compat alias used by older tests.
def select_void_adjacent_victim(
    selected_indices: Sequence[int],
    polys: Sequence[BaseGeometry],
    *,
    min_dist: float,
    sheet: Polygon | None,
    pole: Point | None,
    void_poly: Polygon | None,
    min_size: int = 3,
    max_size: int = 6,
) -> list[int] | None:
    if sheet is None or sheet.is_empty:
        return None
    got = bfs_peel_victim(
        selected_indices,
        polys,
        min_dist=min_dist,
        sheet=sheet,
        pole=pole,
        void_poly=void_poly,
        min_size=min_size,
        max_size=max_size,
    )
    return None if got is None else got[0]


def _selection_area(
    selected_indices: Sequence[int],
    group_ids: Sequence[int],
    part_by_group: dict[int, Polygon],
) -> float:
    total = 0.0
    for i in selected_indices:
        part = part_by_group.get(int(group_ids[i]))
        if part is not None and not part.is_empty:
            total += float(part.area)
    return total


def pattern_from_indices(
    indices: Sequence[int],
    polys: Sequence[BaseGeometry],
    group_ids: Sequence[int],
    transforms: Sequence,
) -> ClusterPattern | None:
    """Build a ClusterPattern from a contact-connected index set."""
    idxs = [int(i) for i in indices]
    if len(idxs) < 2:
        return None
    ref_i = max(idxs, key=lambda i: float(polys[i].area))
    ref_t = (
        float(transforms[ref_i][0]),
        float(transforms[ref_i][1]),
        float(transforms[ref_i][2]),
    )
    members: list[tuple[int, tuple[float, float, float]]] = []
    for i in idxs:
        t = (
            float(transforms[i][0]),
            float(transforms[i][1]),
            float(transforms[i][2]),
        )
        members.append((int(group_ids[i]), relative_transform(ref_t, t)))
    return ClusterPattern(
        members=tuple(members),
        part_count=len(members),
        ref_transform=ref_t,
    )


def pattern_fits_peeled(pat: ClusterPattern, peeled_gids: Sequence[int]) -> bool:
    need = Counter(gid for gid, _ in pat.members)
    have = Counter(int(g) for g in peeled_gids)
    return all(have[g] >= n for g, n in need.items())


def extract_capped_subpatterns(
    placed: Sequence[BaseGeometry],
    group_ids: Sequence[int],
    transforms: Sequence,
    *,
    min_dist: float,
    max_members: int,
    sheet: Polygon | None = None,
    max_patterns: int = 4,
) -> list[ClusterPattern]:
    """Contact BFS submotifs with part_count ≤ max_members from placed pack."""
    if max_members < 2 or len(placed) < 2:
        return []
    n = min(len(placed), len(group_ids), len(transforms))
    groups = cluster_packed_indices(list(placed[:n]), min_dist, sheet=sheet)
    gap = _cluster_merge_gap(list(placed[:n]), min_dist, sheet)
    out: list[ClusterPattern] = []
    for idxs in groups:
        if len(idxs) < 2:
            continue
        adj = cluster_contact_neighbors(idxs, placed, gap)
        # Seed = largest member; peel up to max_members.
        seed = max(idxs, key=lambda i: float(placed[i].area))
        peel = _priority_bfs_peel(seed, adj, placed, None, max_members)
        if len(peel) < 2:
            continue
        pat = pattern_from_indices(peel, placed, group_ids, transforms)
        if pat is not None:
            out.append(pat)
        if len(out) >= max_patterns:
            break
    out.sort(key=lambda p: p.part_count, reverse=True)
    return out


def assign_peeled_to_pattern(
    pat: ClusterPattern,
    peeled_indices: Sequence[int],
    group_ids: Sequence[int],
) -> list[tuple[int, tuple[float, float, float]]] | None:
    """Map each pattern member to a distinct peeled index with matching gid."""
    pool = {g: [] for g in set(int(group_ids[i]) for i in peeled_indices)}
    for i in peeled_indices:
        pool.setdefault(int(group_ids[i]), []).append(int(i))
    assigned: list[tuple[int, tuple[float, float, float]]] = []
    for gid, t_rel in pat.members:
        bucket = pool.get(int(gid)) or []
        if not bucket:
            return None
        idx = bucket.pop()
        assigned.append((idx, t_rel))
    return assigned


def stamp_motif_at_anchor(
    pat: ClusterPattern,
    t_anchor: tuple[float, float, float],
    *,
    peeled_indices: Sequence[int],
    group_ids: Sequence[int],
    part_by_group: dict[int, Polygon],
    sheet: Polygon,
    fixed: BaseGeometry,
    min_dist: float,
    void_geoms: Sequence | None = None,
) -> list[tuple[int, np.ndarray, BaseGeometry]] | None:
    """Atomic peeled-motif stamp under Scene ``is_pose_clear``.

    Assigns peeled indices to pattern members and accepts only when every
    member clears. Returns ``[(idx, tr, poly), ...]`` or None. Propose emit
    uses ``stamp_motif_leader_follower`` (packing clear, single-group
    candidates); partial recovery here is ``pattern_fallback`` in the caller.
    """
    mapping = assign_peeled_to_pattern(pat, peeled_indices, group_ids)
    if mapping is None:
        return None
    if void_geoms is None:
        _, void_geoms = board_context_from_geometry(sheet)
    voids = list(void_geoms) if void_geoms else []
    obs = _obstacle_geoms(fixed)
    part_geoms: dict[int, Geometry] = {}
    placed: list[tuple[int, np.ndarray, BaseGeometry]] = []
    for idx, t_rel in mapping:
        gid = int(group_ids[idx])
        part = part_by_group.get(gid)
        if part is None or part.is_empty:
            return None
        t_abs = compose_transforms(t_anchor, t_rel)
        cand_tr = np.asarray(t_abs, dtype=np.float64).reshape(3)
        if gid not in part_geoms:
            part_geoms[gid] = Geometry.from_shapely(part)
        cand_g = part_geoms[gid].apply_transform(
            float(cand_tr[0]), float(cand_tr[1]), float(cand_tr[2]),
        )
        if not is_pose_clear(cand_g, voids, obs, min_dist):
            return None
        cand = transform_poly(part, cand_tr)
        placed.append((idx, cand_tr, cand))
        obs.append(cand_g)
    return placed


def _motif_stamp_attempt(
    patterns: Sequence[ClusterPattern],
    peeled: Sequence[int],
    group_ids: Sequence[int],
    part_by_group: dict[int, Polygon],
    sheet: Polygon,
    kept_union: BaseGeometry,
    min_dist: float,
    propose_cfg: ProposeConfig,
    pole: Point | None,
    *,
    free_space=None,
    void_poly: Polygon | None = None,
    void_geoms: Sequence | None = None,
) -> list[tuple[int, np.ndarray, BaseGeometry]] | None:
    peeled_gids = [int(group_ids[i]) for i in peeled]
    fitting = [p for p in patterns if pattern_fits_peeled(p, peeled_gids)]
    fitting.sort(key=lambda p: p.part_count, reverse=True)
    if not fitting:
        return None

    anchors: list[tuple[float, float, float]] = []
    # Unified void_seek anchor priority (§4) — topology/pole → pocket → (mirror via patterns).
    anchors.extend(
        void_seek_motif_anchors(
            sheet,
            kept_union,
            min_dist=min_dist,
            propose_cfg=propose_cfg,
            free_space=free_space,
            void_pole=pole,
            patterns=fitting,
        )
    )
    if (
        propose_cfg.motif_use_topo_anchors
        and void_poly is not None
        and not void_poly.is_empty
        and fitting
    ):
        largest_gid = max(
            (int(group_ids[i]) for i in peeled),
            key=lambda g: float(part_by_group[g].area) if g in part_by_group else 0.0,
        )
        part = part_by_group.get(largest_gid)
        if part is not None and not part.is_empty:
            for coords, _tag in aligned_poses_for_pocket(
                part, void_poly, min_dist=min_dist, allowed_angles=None,
            ):
                anchors.append(coords)

    unique_anchors = dedupe_anchors(anchors)

    for pat in fitting:
        for anchor in unique_anchors:
            stamped = stamp_motif_at_anchor(
                pat,
                anchor,
                peeled_indices=peeled,
                group_ids=group_ids,
                part_by_group=part_by_group,
                sheet=sheet,
                fixed=kept_union,
                min_dist=min_dist,
                void_geoms=void_geoms,
            )
            if stamped is not None:
                return stamped
    return None


def cluster_repack_selection(
    sheet: Polygon | SelectionEditCtx,
    polys: list[BaseGeometry] | None = None,
    transforms: list | None = None,
    group_ids: Sequence[int] | None = None,
    selected_indices: Sequence[int] | None = None,
    part_by_group: dict[int, Polygon] | None = None,
    min_dist: float | None = None,
    propose_cfg: ProposeConfig | None = None,
    *,
    pole: Point | None = None,
    void_poly: Polygon | None = None,
    fixed_obstacles: Sequence[BaseGeometry] | None = None,
    pt_push: Point | None = None,
    free_space=None,
) -> tuple[list[BaseGeometry], list, list[int], dict]:
    """BFS-peel a rim/void chunk; motif-stamp into free; else ranked per-part fallback."""
    void_geoms = None
    if isinstance(sheet, SelectionEditCtx):
        ctx = sheet
        sheet = ctx.sheet
        polys = ctx.polys
        transforms = ctx.transforms
        group_ids = ctx.group_ids
        selected_indices = ctx.selected_indices
        part_by_group = ctx.part_by_group
        min_dist = ctx.min_dist
        propose_cfg = ctx.propose_cfg if propose_cfg is None else propose_cfg
        pole = ctx.pole if pole is None else pole
        fixed_obstacles = (
            ctx.fixed_obstacles if fixed_obstacles is None else fixed_obstacles
        )
        void_geoms = ctx.void_geoms
    assert polys is not None and transforms is not None
    assert group_ids is not None and selected_indices is not None
    assert part_by_group is not None and min_dist is not None and propose_cfg is not None
    stats: dict = {
        "attempted": 0,
        "accepted": 0,
        "victim_size": 0,
        "placed": 0,
        "peeled": 0,
        "victim_board_adj": 0,
        "motif_accepted": 0,
        "pattern_fallback": 0,
    }
    out_polys = list(polys)
    out_tr = [np.asarray(t, dtype=np.float64).reshape(3) for t in transforms]
    sel = [int(i) for i in selected_indices]
    if (
        not propose_cfg.enable_cluster_repack
        or sheet is None
        or sheet.is_empty
        or len(sel) < int(propose_cfg.cluster_repack_min_size)
    ):
        return out_polys, out_tr, sel, stats

    if void_geoms is None:
        _, void_geoms = board_context_from_geometry(sheet)
    voids = list(void_geoms) if void_geoms else []

    peeled_info = bfs_peel_victim(
        sel,
        out_polys,
        min_dist=min_dist,
        sheet=sheet,
        pole=pole,
        void_poly=void_poly,
        min_size=int(propose_cfg.cluster_repack_min_size),
        max_size=int(propose_cfg.cluster_repack_max_size),
    )
    if not peeled_info:
        return out_polys, out_tr, sel, stats

    victim, board_adj = peeled_info
    stats["attempted"] = 1
    stats["victim_size"] = len(victim)
    stats["peeled"] = len(victim)
    stats["victim_board_adj"] = int(board_adj)
    victim_set = set(victim)
    kept = [i for i in sel if i not in victim_set]
    old_area = _selection_area(sel, group_ids, part_by_group)

    locked = [
        g for g in (fixed_obstacles or ())
        if g is not None and not g.is_empty
    ]
    kept_geoms = [
        out_polys[i] for i in kept
        if out_polys[i] is not None and not out_polys[i].is_empty
    ]
    kept_union = unary_union(locked + kept_geoms) if (locked or kept_geoms) else Polygon()
    push = pt_push if pt_push is not None else (
        pole if pole is not None else sheet.centroid
    )

    # Patterns: peel motif first, then capped subpatterns from kept.
    patterns: list[ClusterPattern] = []
    peel_pat = pattern_from_indices(victim, out_polys, group_ids, out_tr)
    if peel_pat is not None:
        patterns.append(peel_pat)
    if kept:
        patterns.extend(
            extract_capped_subpatterns(
                [out_polys[i] for i in kept],
                [int(group_ids[i]) for i in kept],
                [out_tr[i] for i in kept],
                min_dist=min_dist,
                max_members=len(victim),
                sheet=sheet,
            )
        )

    stamped = _motif_stamp_attempt(
        patterns,
        victim,
        group_ids,
        part_by_group,
        sheet,
        kept_union,
        min_dist,
        propose_cfg,
        pole,
        free_space=free_space,
        void_poly=void_poly,
        void_geoms=voids,
    )
    void_facing = any(_part_void_adj(out_polys[i], void_poly, min_dist) for i in victim)
    if stamped is not None:
        trial_polys = list(out_polys)
        trial_tr = list(out_tr)
        placed_idxs = [idx for idx, _, _ in stamped]
        for idx, tr, poly in stamped:
            trial_polys[idx] = poly
            trial_tr[idx] = tr
        new_sel = list(kept) + placed_idxs
        new_area = _selection_area(new_sel, group_ids, part_by_group)
        accept_ratio = float(propose_cfg.cluster_repack_area_accept_ratio)
        if void_facing:
            accept_ratio = min(accept_ratio, 0.95)
        if (
            new_area + 1e-12 >= old_area * accept_ratio
            and selection_pairwise_independent(trial_polys, new_sel)
        ):
            stats["accepted"] = 1
            stats["placed"] = len(placed_idxs)
            stats["motif_accepted"] = 1
            return trial_polys, trial_tr, new_sel, stats

    # Per-part ranked fallback.
    stats["pattern_fallback"] = 1
    # Void-facing peel always routes void_seek (even when component is board_adj).
    zone = "void_seek" if void_facing or not board_adj else "border_gap"
    zone_cfg = ProposeConfig.for_place(zone, base=propose_cfg)
    enabled = ProposeConfig.proposers_for_place(zone)
    seeds = void_pole_seed_coords(pole) if pole is not None and zone == "void_seek" else None
    victim_order = sorted(
        victim,
        key=lambda i: float(part_by_group.get(int(group_ids[i]), Polygon()).area),
        reverse=True,
    )
    working_kept_geoms = list(kept_geoms)
    working_base = kept_union
    working_tr: dict[int, np.ndarray] = {}
    working_poly: dict[int, BaseGeometry] = {}
    placed_idxs: list[int] = []
    obs: list[Geometry] = []
    for p in locked + working_kept_geoms:
        og = _as_geometry(p)
        if og is not None:
            obs.append(og)
    part_geoms: dict[int, Geometry] = {}
    for vi in victim_order:
        gid = int(group_ids[vi])
        part = part_by_group.get(gid)
        if part is None or part.is_empty:
            continue
        packed_polys = list(working_kept_geoms)
        packed_gids = [int(group_ids[i]) for i in kept] + [
            int(group_ids[j]) for j in placed_idxs
        ]
        packed_trs = [out_tr[i] for i in kept] + [working_tr[j] for j in placed_idxs]
        try:
            coords = propose_coords_with_strategy(
                working_base,
                part,
                sheet,
                zone_cfg,
                min_dist=min_dist,
                pt_push=push,
                group_id=gid,
                enabled_proposers=enabled,
                guidance_seed_coords=seeds,
                cluster_patterns=patterns,
                packed_polys=packed_polys,
                packed_group_ids=packed_gids[: len(packed_polys)],
                packed_transforms=packed_trs[: len(packed_polys)],
                full_packed_geoms=packed_polys,
                cascade_zone=zone,
            )
        except Exception:
            coords = []
        if gid not in part_geoms:
            part_geoms[gid] = Geometry.from_shapely(part)
        part_g = part_geoms[gid]
        clear: list[tuple[tuple[float, float, float], BaseGeometry, np.ndarray, Geometry]] = []
        for c in coords:
            cand_tr = np.asarray(c, dtype=np.float64).reshape(3)
            cand_g = part_g.apply_transform(
                float(cand_tr[0]), float(cand_tr[1]), float(cand_tr[2]),
            )
            if not is_pose_clear(cand_g, voids, obs, min_dist):
                continue
            cand = transform_poly(part, cand_tr)
            clear.append((c, cand, cand_tr, cand_g))
        if not clear:
            continue
        geom = ProposeGeometry(
            sheet, working_base, part, min_dist, propose_cfg=zone_cfg,
        )
        best_c, best_poly, best_tr, best_g = max(
            clear,
            key=lambda item: score_placement_tightness(
                item[0], geom, push, min_dist,
            ),
        )
        working_tr[vi] = best_tr
        working_poly[vi] = best_poly
        working_kept_geoms.append(best_poly)
        obs.append(best_g)
        working_base = (
            unary_union([working_base, best_poly])
            if not working_base.is_empty
            else best_poly
        )
        placed_idxs.append(vi)

    new_sel = list(kept) + list(placed_idxs)
    new_area = _selection_area(new_sel, group_ids, part_by_group)
    accept_ratio = float(propose_cfg.cluster_repack_area_accept_ratio)
    if void_facing:
        accept_ratio = min(accept_ratio, 0.95)
    if new_area + 1e-12 < old_area * accept_ratio:
        return out_polys, out_tr, sel, stats
    trial_polys = list(out_polys)
    trial_tr = list(out_tr)
    for vi in placed_idxs:
        trial_polys[vi] = working_poly[vi]
        trial_tr[vi] = working_tr[vi]
    if not selection_pairwise_independent(trial_polys, new_sel):
        return out_polys, out_tr, sel, stats
    stats["accepted"] = 1
    stats["placed"] = len(placed_idxs)
    return trial_polys, trial_tr, new_sel, stats


def _component_board_adj(
    global_idxs: Sequence[int],
    polys: Sequence[BaseGeometry],
    sheet: Polygon,
    min_dist: float,
) -> bool:
    from nest_graph.propose.placement_common import is_board_adj
    from nest_graph.propose.placement_outline import outline_ring_geom

    ring = outline_ring_geom(sheet)
    for i in global_idxs:
        poly = polys[i]
        if poly is not None and not poly.is_empty and is_board_adj(
            poly, sheet, min_dist, ring=ring,
        ):
            return True
    return False


def cluster_relocate_selection(
    sheet: Polygon | SelectionEditCtx,
    polys: list[BaseGeometry] | None = None,
    transforms: list | None = None,
    group_ids: Sequence[int] | None = None,
    selected_indices: Sequence[int] | None = None,
    part_by_group: dict[int, Polygon] | None = None,
    min_dist: float | None = None,
    propose_cfg: ProposeConfig | None = None,
    *,
    pole: Point | None = None,
    fixed_obstacles: Sequence[BaseGeometry] | None = None,
    void_geoms: Sequence | None = None,
    max_steps: int = 24,
) -> tuple[list[BaseGeometry], list, dict]:
    """Rigid-translate floating (non-board_adj) contact islands toward ``pole``.

    Prefer ``SelectionEditCtx`` as the first argument; legacy kwargs remain.
    """
    if isinstance(sheet, SelectionEditCtx):
        ctx = sheet
        sheet = ctx.sheet
        polys = ctx.polys
        transforms = ctx.transforms
        group_ids = ctx.group_ids
        selected_indices = ctx.selected_indices
        part_by_group = ctx.part_by_group
        min_dist = ctx.min_dist
        propose_cfg = ctx.propose_cfg
        pole = ctx.pole if pole is None else pole
        fixed_obstacles = (
            ctx.fixed_obstacles if fixed_obstacles is None else fixed_obstacles
        )
        void_geoms = ctx.void_geoms if void_geoms is None else void_geoms
    assert polys is not None and transforms is not None
    assert group_ids is not None and selected_indices is not None
    assert part_by_group is not None and min_dist is not None and propose_cfg is not None
    stats = {"attempted": 0, "accepted": 0, "moved": 0}
    out_polys = list(polys)
    out_tr = [np.asarray(t, dtype=np.float64).reshape(3) for t in transforms]
    sel = [int(i) for i in selected_indices]
    if (
        not propose_cfg.enable_cluster_relocate
        or pole is None
        or pole.is_empty
        or sheet is None
        or sheet.is_empty
        or len(sel) < 2
    ):
        return out_polys, out_tr, stats

    if void_geoms is None:
        _, void_geoms = board_context_from_geometry(sheet)
    voids = list(void_geoms) if void_geoms else []

    idxs = [i for i in sel if out_polys[i] is not None and not out_polys[i].is_empty]
    selected_local = [out_polys[i] for i in idxs]
    local_groups = cluster_packed_indices(selected_local, min_dist, sheet=sheet)
    locked = [
        g for g in (fixed_obstacles or ())
        if g is not None and not g.is_empty
    ]
    step = max(0.5 * float(min_dist), 1e-4)
    part_geoms: dict[int, Geometry] = {}

    for local in local_groups:
        if len(local) < 2:
            continue
        global_idxs = [idxs[j] for j in local]
        if _component_board_adj(global_idxs, out_polys, sheet, min_dist):
            continue  # rim — tangent SE(2) only
        members = [out_polys[i] for i in global_idxs]
        blob = unary_union(members) if len(members) > 1 else members[0]
        if blob is None or blob.is_empty:
            continue
        stats["attempted"] += 1
        cx, cy = float(blob.centroid.x), float(blob.centroid.y)
        dx, dy = float(pole.x) - cx, float(pole.y) - cy
        dist = math.hypot(dx, dy)
        if dist < 1e-9:
            continue
        ux, uy = dx / dist, dy / dist
        others = [
            out_polys[j]
            for j in sel
            if j not in set(global_idxs)
            and out_polys[j] is not None
            and not out_polys[j].is_empty
        ]
        obs: list[Geometry] = []
        for p in locked + others:
            og = _as_geometry(p)
            if og is not None:
                obs.append(og)

        best_delta = (0.0, 0.0)
        for s in range(1, max_steps + 1):
            ox, oy = ux * step * s, uy * step * s
            ok = True
            for gi in global_idxs:
                gid = int(group_ids[gi])
                part = part_by_group.get(gid)
                if part is None:
                    ok = False
                    break
                tr = out_tr[gi]
                cand_tr = np.array([tr[0] + ox, tr[1] + oy, tr[2]], dtype=np.float64)
                if gid not in part_geoms:
                    part_geoms[gid] = Geometry.from_shapely(part)
                cand_g = part_geoms[gid].apply_transform(
                    float(cand_tr[0]), float(cand_tr[1]), float(cand_tr[2]),
                )
                if not is_pose_clear(cand_g, voids, obs, min_dist):
                    ok = False
                    break
            if not ok:
                break
            best_delta = (ox, oy)

        ox, oy = best_delta
        if abs(ox) < 1e-12 and abs(oy) < 1e-12:
            continue
        trial_polys = list(out_polys)
        trial_tr = list(out_tr)
        for gi in global_idxs:
            part = part_by_group[int(group_ids[gi])]
            tr = out_tr[gi]
            cand_tr = np.array([tr[0] + ox, tr[1] + oy, tr[2]], dtype=np.float64)
            trial_tr[gi] = cand_tr
            trial_polys[gi] = transform_poly(part, cand_tr)
        if not selection_pairwise_independent(trial_polys, sel):
            continue
        out_polys = trial_polys
        out_tr = trial_tr
        stats["accepted"] += 1
        stats["moved"] += len(global_idxs)

    return out_polys, out_tr, stats
