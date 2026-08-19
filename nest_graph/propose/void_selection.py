"""Void-aware selection scoring, pin repair, and propose-funnel telemetry.

Extracted from ``build_graph`` so the demo loop and ``nesting_evaluator`` share
one implementation. Graph objects are duck-typed (``collisions`` / ``group_id``)
so this module stays free of ``elem_graph`` imports.
"""

import time
from typing import Sequence

import numpy as np
from shapely import Point
from shapely.geometry.base import BaseGeometry

from nest_graph.propose.placement_outline import (
    outline_kiss_tolerance,
    outline_ring_geom,
    outline_standoff_distance,
)
from nest_graph.geometry import (
    Geometry,
    PlacementRankConfig,
    PlacementRankMode,
    batch_score_placed_contact_hybrid,
)
from nest_graph.utils import transform_row_key


def pose_key_to_index(
    group_id: Sequence[int],
    transform: Sequence,
) -> dict[tuple[int, tuple[float, float, float]], int]:
    """Last-wins (gid, round-4 key) → vertex index."""
    out: dict[tuple[int, tuple[float, float, float]], int] = {}
    for i, (gid, tr) in enumerate(zip(group_id, transform, strict=False)):
        out[(int(gid), transform_row_key(tr))] = int(i)
    return out


def pose_key_to_verts(
    group_id: Sequence[int],
    transform: Sequence,
) -> dict[tuple[int, tuple[float, float, float]], list[int]]:
    """(gid, round-4 key) → all matching vertex indices."""
    out: dict[tuple[int, tuple[float, float, float]], list[int]] = {}
    for i, (gid, tr) in enumerate(zip(group_id, transform, strict=False)):
        out.setdefault((int(gid), transform_row_key(tr)), []).append(int(i))
    return out


def _sel_area(
    idxs: Sequence[int],
    group_id: Sequence[int] | None,
    part_areas: Sequence[float] | None,
    *,
    empty_as_count: bool = False,
) -> float:
    if not group_id or not part_areas:
        return float(len(idxs)) if empty_as_count else 0.0
    total = 0.0
    n_areas = len(part_areas)
    n_gid = len(group_id)
    for i in idxs:
        gi = int(group_id[int(i)]) if 0 <= int(i) < n_gid else -1
        if 0 <= gi < n_areas:
            total += float(part_areas[gi])
    return total


def void_attractor_radius(
    min_dist: float,
    sheet_diag: float,
    place_rule_radius: float,
) -> float:
    """Sheet-aware PointPlaceRule radius (legacy local r is too sharp for large voids)."""
    return max(float(place_rule_radius), float(min_dist) * 4.0, 0.25 * float(sheet_diag))


def void_pole_near_radius(sheet_diag: float, ratio: float = 0.25) -> float:
    """Shared radius for densify pole_near accept and void_leak props_pole telem.

    Densify measures placed *centroid* distance; props_pole measures transform
    *(x, y)* — same radius, different measure.
    """
    if float(sheet_diag) <= 1e-12 or float(ratio) <= 0.0:
        return 0.0
    return float(ratio) * float(sheet_diag)


def apply_void_centroid_score_term(
    polys: Sequence,
    scores: list[float],
    *,
    free_info,
    free_poly: BaseGeometry | None,
    void_term: float,
) -> int:
    """Add free-centroid void term to nest/refine scores (L1; one SoT).

    Returns number of nodes that received the term.
    """
    if (
        void_term <= 0.0
        or free_info is None
        or getattr(free_info, "kind", None) != "large_void"
        or free_poly is None
        or getattr(free_poly, "is_empty", True)
    ):
        return 0
    hits = 0
    for i, poly in enumerate(polys):
        if i >= len(scores):
            break
        try:
            if poly is not None and not poly.is_empty and (
                free_poly.contains(poly.centroid) or free_poly.intersects(poly.centroid)
            ):
                scores[i] = float(scores[i]) + float(void_term)
                hits += 1
        except Exception:
            continue
    return hits


def boost_border_scores(
    polys: list,
    scores: list[float],
    outline: BaseGeometry,
    min_dist: float,
    *,
    weight: float = 8.0,
) -> None:
    """Favor graph nodes flush to the nest outline when scoring nest/DFS."""
    scale = outline_kiss_tolerance(min_dist)
    for i, sc in enumerate(scores):
        err = abs(outline_standoff_distance(polys[i], outline) - min_dist)
        scores[i] = sc + weight * max(0.0, 1.0 - err / scale)


def boost_void_island_scores(
    polys: list,
    scores: list[float],
    free_poly: BaseGeometry | None,
    *,
    weight: float,
    pole: Point | None = None,
    pole_radius: float = 0.0,
    sheet_diag: float = 0.0,
) -> int:
    """EMS-style distance-to-pole boost for Python DFS/finalize scores.

    ``weight`` scales ``max(0, 1 - dist/pole_norm)`` for nodes in the free
    poly (or within ``pole_radius`` of the pole). Flat legacy boost when
    ``sheet_diag <= 0`` and pole is unset beyond the in_free check.
    """
    if weight <= 0.0 or not scores:
        return 0
    diag = float(sheet_diag)
    if diag <= 1e-12 and free_poly is not None and not free_poly.is_empty:
        minx, miny, maxx, maxy = free_poly.bounds
        diag = float(np.hypot(maxx - minx, maxy - miny))
    if diag <= 1e-12 and pole is not None and pole_radius > 0.0:
        diag = float(pole_radius) * 4.0
    if diag <= 1e-12:
        diag = 1.0

    n = 0
    for i, sc in enumerate(scores):
        if i >= len(polys):
            break
        poly = polys[i]
        if poly is None or poly.is_empty:
            continue
        c = poly.centroid
        in_free = (
            free_poly is not None
            and not free_poly.is_empty
            and (free_poly.contains(c) or free_poly.intersects(c))
        )
        near_pole = (
            pole is not None
            and pole_radius > 0.0
            and float(c.distance(pole)) <= float(pole_radius)
        )
        if not (in_free or near_pole):
            continue
        if pole is not None:
            dist = float(c.distance(pole))
            # Q118: inverse-square decay (steeper than linear) toward void pole.
            norm = max(dist / diag, 0.0)
            factor = 1.0 / (1.0 + norm * norm)
        else:
            factor = 1.0
        scores[i] = float(sc) + float(weight) * factor
        n += 1
    return n


def boost_keyed_proposal_scores(
    group_id: Sequence[int],
    transforms: Sequence,
    scores: list[float],
    keys_by_group: dict[int, set[tuple[float, float, float]]] | None,
    *,
    weight: float,
) -> int:
    """Add ``weight`` to scores whose (group, transform key) is in ``keys_by_group``."""
    if weight <= 0.0 or not scores or not keys_by_group:
        return 0
    n = 0
    for i, sc in enumerate(scores):
        if i >= len(group_id) or i >= len(transforms):
            break
        gid = int(group_id[i])
        keys = keys_by_group.get(gid)
        if not keys:
            continue
        key = transform_row_key(np.asarray(transforms[i], dtype=np.float64))
        if key not in keys:
            continue
        scores[i] = float(sc) + float(weight)
        n += 1
    return n


def boost_small_part_scores(
    group_id: Sequence[int],
    scores: list[float],
    part_areas: Sequence[float],
    *,
    weight: float,
) -> int:
    """Boost smaller catalog groups: weight * (1 - area/max_area) on large_void."""
    if weight <= 0.0 or not scores or not part_areas:
        return 0
    areas = [float(a) for a in part_areas]
    max_a = max(areas) if areas else 0.0
    if max_a <= 1e-12:
        return 0
    n = 0
    for i, sc in enumerate(scores):
        if i >= len(group_id):
            break
        gid = int(group_id[i])
        if gid < 0 or gid >= len(areas):
            continue
        factor = max(0.0, 1.0 - areas[gid] / max_a)
        if factor <= 0.0:
            continue
        scores[i] = float(sc) + float(weight) * factor
        n += 1
    return n


def boost_large_part_scores(
    group_id: Sequence[int],
    scores: list[float],
    part_areas: Sequence[float],
    *,
    weight: float,
) -> int:
    """Prefer larger catalog groups into open basins (scrap density)."""
    if weight <= 0.0 or not scores or not part_areas:
        return 0
    areas = [float(a) for a in part_areas]
    max_a = max(areas) if areas else 0.0
    if max_a <= 1e-12:
        return 0
    n = 0
    for i, sc in enumerate(scores):
        if i >= len(group_id):
            break
        gid = int(group_id[i])
        if gid < 0 or gid >= len(areas):
            continue
        factor = max(0.0, areas[gid] / max_a)
        if factor <= 0.0:
            continue
        scores[i] = float(sc) + float(weight) * factor
        n += 1
    return n


def boost_selection_geom_quality(
    candidate_geoms: list[Geometry],
    scores: list[float],
    *,
    board_ring: Geometry,
    packed: list[Geometry],
    min_dist: float,
    part_areas: Sequence[float],
    group_id: Sequence[int],
    sheet_area: float,
    propose_cfg,
    stats_out: dict | None = None,
) -> int:
    """Add non-negative C++ contact_hybrid quality into nest/DFS scores."""
    w = float(getattr(propose_cfg, "selection_geom_weight", 0.0) or 0.0)
    if w <= 0.0 or not scores or not candidate_geoms:
        if stats_out is not None:
            stats_out["geom_ms"] = 0.0
            stats_out["geom_share"] = 0.0
            stats_out["geom_hits"] = 0
        return 0
    t0 = time.perf_counter()
    band = float(getattr(propose_cfg, "edge_free_band_min_dist_mult", 3.5)) * float(min_dist)
    cfg = PlacementRankConfig()
    cfg.min_dist = float(min_dist)
    cfg.clearance_weight = float(propose_cfg.contact_clearance_hybrid_weight)
    cfg.tightness_weight = float(propose_cfg.contact_tightness_hybrid_weight)
    cfg.edge_free_weight = float(propose_cfg.edge_free_weight)
    cfg.edge_free_band_mult = float(getattr(propose_cfg, "edge_free_band_min_dist_mult", 3.5))
    cfg.kiss_tol = float(outline_kiss_tolerance(min_dist))
    cfg.tight_scale = max(band, 1e-6)
    cfg.sheet_area = max(float(sheet_area), 1e-12)
    cfg.mode = PlacementRankMode.contact_hybrid
    # Batch API uses one part_area for edge_free area_frac; mean is fine for MIS ordering.
    areas = [float(a) for a in part_areas] if part_areas else [1.0]
    cfg.part_area = float(np.mean(areas)) if areas else 1.0
    _ = group_id  # reserved for per-group area_frac if a follow-up batches by group

    results = batch_score_placed_contact_hybrid(
        candidate_geoms,
        board_ring,
        list(packed),
        None,
        cfg,
    )
    geom_added = 0.0
    n = 0
    for i, res in enumerate(results):
        if i >= len(scores):
            break
        q = max(0.0, float(res.quality))
        delta = w * q
        scores[i] = float(scores[i]) + delta
        geom_added += delta
        n += 1
    elapsed_ms = (time.perf_counter() - t0) * 1000.0
    total = sum(abs(float(s)) for s in scores) + 1e-12
    if stats_out is not None:
        stats_out["geom_ms"] = float(elapsed_ms)
        stats_out["geom_share"] = float(geom_added / total)
        stats_out["geom_hits"] = int(n)
    return n


def apply_void_selection_boosts(
    *,
    polys: list,
    group_id: Sequence[int],
    transform: Sequence,
    scores: list[float],
    free_info,
    free_poly,
    part_areas: Sequence[float],
    propose_stats: dict | None,
    cfg,
    sheet_diag: float,
    void_r: float,
    candidate_geoms: list[Geometry] | None = None,
    packed_geoms: list[Geometry] | None = None,
    outline: BaseGeometry | None = None,
    min_dist: float = 0.0,
    sheet_area: float = 0.0,
    geom_stats_out: dict | None = None,
) -> dict[str, int]:
    """Apply void-island, pocket-key, small-part, and selection-geom score boosts."""
    hits = {
        "void_island": 0,
        "pocket_keys": 0,
        "motif_keys": 0,
        "kind_keys": 0,
        "small_part": 0,
        "large_part": 0,
        "selection_geom": 0,
    }
    pole_w = float(cfg.propose.void_island_score_boost)
    pocket_w = float(getattr(cfg.propose, "pocket_score_boost", 0.0) or 0.0)
    motif_w = float(getattr(cfg.propose, "motif_score_boost", 0.0) or 0.0)
    small_w = float(getattr(cfg.propose, "small_part_void_score_boost", 0.0) or 0.0)
    geom_w = float(getattr(cfg.propose, "selection_geom_weight", 0.0) or 0.0)
    # Soft nest scale on large_void (decision + evaluator share this SoT).
    mcts_zone = str((propose_stats or {}).get("mcts_zone") or "")
    if free_info is not None and getattr(free_info, "kind", None) == "large_void" and pole_w > 0.0:
        scale = 1.5 if mcts_zone == "void_seek" else 1.25
        pole_w = float(pole_w) * scale
        if propose_stats is not None:
            propose_stats["void_island_soft_scale"] = float(scale)
    if free_info.kind == "large_void" and pole_w > 0.0:
        hits["void_island"] = boost_void_island_scores(
            polys,
            scores,
            free_poly,
            weight=pole_w,
            pole=free_info.target_pt,
            pole_radius=void_r,
            sheet_diag=sheet_diag,
        )
    if pocket_w > 0.0 and propose_stats is not None:
        keys = propose_stats.get("pocket_keys") or {}
        hits["pocket_keys"] = boost_keyed_proposal_scores(
            group_id,
            transform,
            scores,
            keys,
            weight=pocket_w,
        )
        kind_keys = propose_stats.get("kind_keys") or {}
        motif_for_kind = propose_stats.get("motif_keys") or {}
        void_kind: dict[int, set[tuple[float, float, float]]] = {}
        for gid, kset in kind_keys.items():
            gi = int(gid)
            void_kind[gi] = set(kset or ()) - set(motif_for_kind.get(gi) or ())
        hits["kind_keys"] = boost_keyed_proposal_scores(
            group_id,
            transform,
            scores,
            void_kind,
            weight=pocket_w,
        )
    if motif_w > 0.0 and propose_stats is not None:
        motif_keys = propose_stats.get("motif_keys") or {}
        if not motif_keys:
            # Fallback: cluster_copy keys from densify/proposer telemetry.
            densify = propose_stats.get("densify_stats") or {}
            pk = propose_stats.get("proposer_keys") or densify.get("proposer_keys") or {}
            cc = pk.get("cluster_copy") or densify.get("motif_keys") or set()
            if cc:
                # Keys lack group — apply to all groups via matching transform keys.
                motif_keys = {
                    int(gid): set(cc) for gid in set(int(g) for g in group_id)
                }
        hits["motif_keys"] = boost_keyed_proposal_scores(
            group_id,
            transform,
            scores,
            motif_keys,
            weight=motif_w,
        )
    # Open basin (high void ratio): prefer large parts for scrap density;
    # swiss-cheese / modest voids keep small-part pocket fill.
    void_ratio = float(getattr(free_info, "max_void_ratio", 0.0) or 0.0)
    if (
        free_info is not None
        and getattr(free_info, "kind", None) == "large_void"
        and void_ratio >= 8.0
        and small_w > 0.0
    ):
        hits["large_part"] = boost_large_part_scores(
            group_id, scores, part_areas, weight=small_w,
        )
    elif small_w > 0.0:
        hits["small_part"] = boost_small_part_scores(
            group_id, scores, part_areas, weight=small_w,
        )
    if geom_w > 0.0 and candidate_geoms is not None and outline is not None:
        board_ring = outline_ring_geom(outline)
        if board_ring is None and isinstance(outline, Geometry):
            board_ring = outline
        if board_ring is not None:
            hits["selection_geom"] = boost_selection_geom_quality(
                candidate_geoms,
                scores,
                board_ring=board_ring,
                packed=list(packed_geoms or []),
                min_dist=float(min_dist),
                part_areas=part_areas,
                group_id=group_id,
                sheet_area=float(sheet_area) if sheet_area > 0 else float(getattr(outline, "area", 1.0) or 1.0),
                propose_cfg=cfg.propose,
                stats_out=geom_stats_out,
            )
    return hits


def centroid_in_free(
    poly,
    free_poly: BaseGeometry | None,
    *,
    interior_margin: float = 0.0,
) -> bool:
    """Centroid in free; optional eroded core for true-interior colonize (V2)."""
    if free_poly is None or free_poly.is_empty or poly is None or poly.is_empty:
        return False
    c = poly.centroid
    region = free_poly
    margin = float(interior_margin)
    if margin > 1e-12:
        try:
            core = free_poly.buffer(-margin)
            if core is not None and not getattr(core, "is_empty", True):
                region = core
        except Exception:
            region = free_poly
    return bool(region.contains(c) or region.intersects(c))


def xy_in_free(x: float, y: float, free_poly: BaseGeometry | None) -> bool:
    if free_poly is None or free_poly.is_empty:
        return False
    p = Point(float(x), float(y))
    return bool(free_poly.contains(p) or free_poly.intersects(p))


def count_selected_in_free(
    polys: list,
    selected: Sequence[int],
    free_poly: BaseGeometry | None,
    *,
    interior_margin: float = 0.0,
) -> int:
    return sum(
        1
        for i in selected
        if centroid_in_free(polys[i], free_poly, interior_margin=interior_margin)
    )


def count_graph_in_free(
    polys: list,
    free_poly: BaseGeometry | None,
    *,
    interior_margin: float = 0.0,
) -> int:
    return sum(
        1 for p in polys
        if centroid_in_free(p, free_poly, interior_margin=interior_margin)
    )


def count_props_in_free(
    proposed_by_group: Sequence[np.ndarray] | None,
    free_poly: BaseGeometry | None,
) -> int:
    if not proposed_by_group or free_poly is None or free_poly.is_empty:
        return 0
    n = 0
    for arr in proposed_by_group:
        if arr is None or len(arr) == 0:
            continue
        for row in np.asarray(arr, dtype=np.float64).reshape(-1, 3):
            if xy_in_free(float(row[0]), float(row[1]), free_poly):
                n += 1
    return n


def count_props_near_pole(
    proposed_by_group: Sequence[np.ndarray] | None,
    pole: Point | None,
    radius: float,
) -> int:
    """Count proposals whose (x,y) lies within ``radius`` of the void pole."""
    if not proposed_by_group or pole is None or float(radius) <= 0.0:
        return 0
    r = float(radius)
    n = 0
    for arr in proposed_by_group:
        if arr is None or len(arr) == 0:
            continue
        for row in np.asarray(arr, dtype=np.float64).reshape(-1, 3):
            d = float(
                np.hypot(float(row[0]) - float(pole.x), float(row[1]) - float(pole.y))
            )
            if d <= r:
                n += 1
    return n


def proposer_key_owner(
    proposer_keys: dict[str, set[tuple[float, float, float]]] | None,
) -> dict[tuple[float, float, float], str]:
    """Map transform key → first-writer proposer name."""
    owner: dict[tuple[float, float, float], str] = {}
    if not proposer_keys:
        return owner
    for name, keys in proposer_keys.items():
        for key in keys:
            if key not in owner:
                owner[key] = name
    return owner


def count_selected_by_proposer(
    transforms: Sequence,
    selected: Sequence[int],
    proposer_keys: dict[str, set[tuple[float, float, float]]] | None,
) -> dict[str, int]:
    owner = proposer_key_owner(proposer_keys)
    counts: dict[str, int] = {}
    for i in selected:
        if i < 0 or i >= len(transforms):
            continue
        key = transform_row_key(np.asarray(transforms[i], dtype=np.float64))
        name = owner.get(key)
        if name is None:
            continue
        counts[name] = counts.get(name, 0) + 1
    return counts


def format_prop_accept(
    emitted: dict[str, int],
    pool: dict[str, int],
    nest: dict[str, int],
    refine: dict[str, int],
    *,
    limit: int = 8,
) -> str:
    """AGENTS funnel line: ``name:e/p/n/r`` per proposer, emit-count ordered."""
    names = sorted(
        set(emitted) | set(pool) | set(nest) | set(refine),
        key=lambda n: (-int(emitted.get(n, 0)), n),
    )
    parts = []
    for name in names[:limit]:
        parts.append(
            f"{name}:e{int(emitted.get(name, 0))}/p{int(pool.get(name, 0))}/"
            f"n{int(nest.get(name, 0))}/r{int(refine.get(name, 0))}"
        )
    return " ".join(parts)


def colonize_pin_clear(
    candidates: Sequence[int],
    collisions,
    cur: list[int],
    cur_set: set[int],
) -> tuple[list[int], set[int], int, int]:
    """Pin collision-clear void candidates onto ``cur`` (one SoT walk)."""
    add = 0
    blk = 0
    for v in candidates:
        vi = int(v)
        if vi in cur_set:
            continue
        if any(int(u) in cur_set for u in collisions[vi]):
            blk += 1
            continue
        cur.append(vi)
        cur_set.add(vi)
        add += 1
    return cur, cur_set, add, blk


def colonize_blocker_order(
    candidates: Sequence[int],
    collisions,
    out_set: set[int],
    polys: list,
    free_poly: BaseGeometry | None,
    margin: float,
) -> list[int]:
    """Prefer rim plugs; protect free-core residents; fall back to any plug."""
    unlocks: dict[int, int] = {}
    core_m = float(margin) if float(margin) > 1e-12 else 1e-6
    for v in list(candidates)[:64]:
        hit = [int(u) for u in collisions[int(v)] if int(u) in out_set]
        if not hit:
            continue
        for ui in hit:
            if centroid_in_free(polys[ui], free_poly, interior_margin=core_m):
                continue
            unlocks[ui] = int(unlocks.get(ui, 0)) + 1
    if not unlocks:
        # All plugs sit on the free fringe — still drop highest-degree plugs.
        for v in list(candidates)[:64]:
            for u in collisions[int(v)]:
                ui = int(u)
                if ui in out_set:
                    unlocks[ui] = int(unlocks.get(ui, 0)) + 1
    if not unlocks:
        return []
    return sorted(unlocks.keys(), key=lambda ui: -int(unlocks[ui]))


def _colonize_trial_better(
    trial: Sequence[int],
    cur: Sequence[int],
    *,
    base_len: int,
    base_area: float,
    group_id: Sequence[int] | None,
    part_areas: Sequence[float] | None,
    n_drop: int,
    add: int,
) -> bool:
    """Accept unlock when area holds and void pins replace rim plugs."""
    if add <= 0:
        return False
    t_area = _sel_area(trial, group_id, part_areas, empty_as_count=True)
    c_area = _sel_area(cur, group_id, part_areas, empty_as_count=True)
    # Hybrid: allow a small dip when the unlock nets more void pins than plugs.
    area_floor = base_area
    if add > n_drop:
        area_floor = 0.95 * base_area
    if t_area + 1e-12 < area_floor:
        return False
    if t_area > c_area + 1e-12:
        return True
    if t_area + 1e-12 < c_area:
        return False
    if len(trial) > len(cur):
        return True
    if len(trial) < len(cur):
        return False
    return add >= n_drop and len(trial) >= max(base_len, len(cur))


def colonize_void_onto_base(
    graph,
    base: Sequence[int],
    polys: list,
    free_poly: BaseGeometry | None,
    scores: list[float] | None = None,
    *,
    stats_out: dict | None = None,
    interior_margin: float = 0.0,
    max_rim_drop: int = 16,
    group_id: Sequence[int] | None = None,
    part_areas: Sequence[float] | None = None,
) -> list[int]:
    """Pin free-centroid graph nodes onto ``base`` if collision-clear.

    Same collision walk as ``pin_nest_void_independent`` (one SoT). Used after
    incumbent hold so rim density and void pins hybridize.

    V2: ``interior_margin`` prefers true free core. V3: iterative rim-blocker
    unlock — drop ≤``max_rim_drop`` plugs when area-aware retry is non-worse.
    """
    t0 = time.perf_counter()
    out = list(base)
    out_set = set(int(i) for i in out)
    collisions = getattr(graph, "collisions", None)
    margin = float(interior_margin)
    if collisions is None or free_poly is None or getattr(free_poly, "is_empty", True):
        if stats_out is not None:
            stats_out["colonize_candidates"] = 0
            stats_out["colonize_pinned"] = 0
            stats_out["colonize_blocked"] = 0
            stats_out["colonize_rim_drop"] = 0
            stats_out["colonize_ms"] = (time.perf_counter() - t0) * 1000.0
        return out
    candidates = [
        i
        for i in range(len(collisions))
        if i not in out_set
        and i < len(polys)
        and centroid_in_free(polys[i], free_poly, interior_margin=margin)
    ]
    # Fallback to full free if eroded core empties candidates.
    if not candidates and margin > 1e-12:
        candidates = [
            i
            for i in range(len(collisions))
            if i not in out_set
            and i < len(polys)
            and centroid_in_free(polys[i], free_poly, interior_margin=0.0)
        ]
    pinned = 0
    blocked = 0
    rim_drop = 0
    base_len = len(out)
    base_area = _sel_area(out, group_id, part_areas, empty_as_count=True)
    if scores is not None and len(scores) >= len(collisions):
        candidates.sort(key=lambda v: float(scores[v]), reverse=True)

    out, out_set, pinned, blocked = colonize_pin_clear(
        candidates, collisions, out, out_set,
    )
    drop_budget = max(0, int(max_rim_drop))
    core_m = float(margin) if float(margin) > 1e-12 else 1e-6
    tried_victims: set[frozenset[int]] = set()
    while drop_budget > 0 and blocked > 0 and candidates:
        # Map still-blocked cands → rim plugs (protect free-core residents).
        blocked_plugs: list[tuple[int, list[int]]] = []
        for v in candidates:
            vi = int(v)
            if vi in out_set:
                continue
            plugs = [int(u) for u in collisions[vi] if int(u) in out_set]
            if not plugs:
                continue
            rim_plugs = [
                ui for ui in plugs
                if not centroid_in_free(
                    polys[ui], free_poly, interior_margin=core_m,
                )
            ]
            if not rim_plugs:
                rim_plugs = list(plugs)
            blocked_plugs.append((vi, rim_plugs))
        if not blocked_plugs:
            ordered = colonize_blocker_order(
                candidates, collisions, out_set, polys, free_poly, margin,
            )
            if not ordered:
                break
            blocked_plugs = [(-1, [int(ordered[0])])]
        # Prefer unlocks that clear ≥ as many void cands as plugs dropped
        # (area-safe fanout); then fewest plugs; then highest void score.
        def _unlock_key(item: tuple[int, list[int]]) -> tuple:
            vi, rim_plugs = item
            drop_set = frozenset(rim_plugs)
            cover = sum(
                1 for _w, plugs in blocked_plugs
                if drop_set.issuperset(plugs)
            )
            sc = (
                float(scores[vi])
                if scores is not None and 0 <= vi < len(scores)
                else 0.0
            )
            return (-cover, len(rim_plugs), -sc)

        blocked_plugs.sort(key=_unlock_key)
        progressed = False
        for vi, rim_plugs in blocked_plugs:
            if not rim_plugs:
                continue
            # V3 minimal drop: at most 2 highest-cover plugs, not the full collision star
            # (stars of 10–20 rim hits were skipped by drop_budget and never unlocked).
            if len(rim_plugs) > 2:
                deg: dict[int, int] = {}
                for u in rim_plugs:
                    deg[int(u)] = sum(
                        1 for _w, plugs in blocked_plugs if int(u) in plugs
                    )
                rim_plugs = sorted(rim_plugs, key=lambda u: -int(deg.get(int(u), 0)))[:2]
            if len(rim_plugs) > drop_budget:
                rim_plugs = list(rim_plugs)[:drop_budget]
            if not rim_plugs:
                continue
            drop_set = frozenset(int(u) for u in rim_plugs)
            if drop_set in tried_victims:
                continue
            tried_victims.add(drop_set)
            trial = [i for i in out if int(i) not in drop_set]
            trial_set = set(int(i) for i in trial)
            trial, trial_set, add, blk = colonize_pin_clear(
                candidates, collisions, trial, trial_set,
            )
            n_drop = len(drop_set)
            if not _colonize_trial_better(
                trial,
                out,
                base_len=base_len,
                base_area=base_area,
                group_id=group_id,
                part_areas=part_areas,
                n_drop=n_drop,
                add=add,
            ):
                continue
            out = trial
            out_set = trial_set
            pinned += add
            blocked = blk
            rim_drop += n_drop
            drop_budget -= n_drop
            progressed = True
            break
        if not progressed:
            break
    # Cohort unlock: drop the top-k highest-degree rim plugs together when
    # single-set V3 stalls (many void cands share a small plug clique).
    if drop_budget >= 2 and blocked > 0:
        ordered = colonize_blocker_order(
            candidates, collisions, out_set, polys, free_poly, margin,
        )
        k = min(4, int(drop_budget), len(ordered))
        if k >= 2:
            drop_set = frozenset(int(u) for u in ordered[:k])
            trial = [i for i in out if int(i) not in drop_set]
            trial_set = set(int(i) for i in trial)
            trial, trial_set, add, blk = colonize_pin_clear(
                candidates, collisions, trial, trial_set,
            )
            if _colonize_trial_better(
                trial,
                out,
                base_len=base_len,
                base_area=base_area,
                group_id=group_id,
                part_areas=part_areas,
                n_drop=k,
                add=add,
            ):
                out = trial
                out_set = trial_set
                pinned += add
                blocked = blk
                rim_drop += k
    if stats_out is not None:
        stats_out["colonize_candidates"] = len(candidates)
        stats_out["colonize_pinned"] = pinned
        stats_out["colonize_blocked"] = blocked
        stats_out["colonize_rim_drop"] = rim_drop
        stats_out["colonize_ms"] = (time.perf_counter() - t0) * 1000.0
    return out


def void_core_then_rim(
    graph,
    polys: list,
    free_poly: BaseGeometry | None,
    scores: list[float] | None = None,
    *,
    interior_margin: float = 0.0,
    stats_out: dict | None = None,
) -> list[int]:
    """Void-first MIS then rim fill (hybrid complement to rim-first colonize)."""
    t0 = time.perf_counter()
    collisions = getattr(graph, "collisions", None)
    if collisions is None or free_poly is None or getattr(free_poly, "is_empty", True):
        if stats_out is not None:
            stats_out["void_core_n"] = 0
            stats_out["void_core_rim_add"] = 0
            stats_out["void_core_ms"] = (time.perf_counter() - t0) * 1000.0
        return []
    margin = float(interior_margin)
    void_cands = [
        i
        for i in range(len(collisions))
        if i < len(polys)
        and centroid_in_free(polys[i], free_poly, interior_margin=margin)
    ]
    if not void_cands and margin > 1e-12:
        void_cands = [
            i
            for i in range(len(collisions))
            if i < len(polys)
            and centroid_in_free(polys[i], free_poly, interior_margin=0.0)
        ]
    if scores is not None and len(scores) >= len(collisions):
        void_cands.sort(key=lambda v: float(scores[v]), reverse=True)
    core: list[int] = []
    core_set: set[int] = set()
    for v in void_cands:
        vi = int(v)
        if any(int(u) in core_set for u in collisions[vi]):
            continue
        core.append(vi)
        core_set.add(vi)
    rest = [
        i for i in range(len(collisions))
        if i not in core_set and i < len(polys)
    ]
    if scores is not None and len(scores) >= len(collisions):
        rest.sort(key=lambda v: float(scores[v]), reverse=True)
    out, out_set, add, _blk = colonize_pin_clear(rest, collisions, core, core_set)
    if stats_out is not None:
        stats_out["void_core_n"] = len(core)
        stats_out["void_core_rim_add"] = int(add)
        stats_out["void_core_total"] = len(out)
        stats_out["void_core_ms"] = (time.perf_counter() - t0) * 1000.0
    return out


def pin_nest_void_independent(
    graph,
    selected_nest: Sequence[int],
    selected_refine: Sequence[int],
    polys: list,
    free_poly: BaseGeometry | None,
    scores: list[float] | None = None,
    *,
    stats_out: dict | None = None,
) -> list[int]:
    """P3: re-add nest-void nodes missing from refine if collision-clear."""
    t0 = time.perf_counter()
    refine = list(selected_refine)
    refine_set = set(refine)
    candidates = [
        i for i in selected_nest
        if i not in refine_set and centroid_in_free(polys[i], free_poly)
    ]
    if stats_out is not None:
        stats_out["pin_candidates"] = len(candidates)
    if not candidates:
        if stats_out is not None:
            stats_out["pin_added"] = 0
            stats_out["pin_blocked_collision"] = 0
            stats_out["pin_ms"] = (time.perf_counter() - t0) * 1000.0
        return refine
    if scores is not None and len(scores) == len(graph.group_id):
        candidates.sort(key=lambda v: scores[v], reverse=True)
    refine, refine_set, pin_added, pin_blocked = colonize_pin_clear(
        candidates,
        graph.collisions,
        refine,
        refine_set,
    )
    if stats_out is not None:
        stats_out["pin_added"] = int(pin_added)
        stats_out["pin_blocked_collision"] = int(pin_blocked)
        stats_out["pin_ms"] = (time.perf_counter() - t0) * 1000.0
    return refine


def zones_have_void_hijack(zones: Sequence[str] | None) -> bool:
    if not zones:
        return False
    return any("void_seek(large_void)" in str(z) for z in zones)
