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


def transform_row_key(t) -> tuple[float, float, float]:
    """Round-4 (x, y, θ) join key shared by propose / nest / refine telemetry."""
    return (round(float(t[0]), 4), round(float(t[1]), 4), round(float(t[2]), 4))


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
            factor = max(0.0, 1.0 - dist / diag)
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
        "small_part": 0,
        "selection_geom": 0,
    }
    pole_w = float(cfg.propose.void_island_score_boost)
    pocket_w = float(getattr(cfg.propose, "pocket_score_boost", 0.0) or 0.0)
    motif_w = float(getattr(cfg.propose, "motif_score_boost", 0.0) or 0.0)
    small_w = float(getattr(cfg.propose, "small_part_void_score_boost", 0.0) or 0.0)
    geom_w = float(getattr(cfg.propose, "selection_geom_weight", 0.0) or 0.0)
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
    if free_info.kind == "large_void" and small_w > 0.0:
        hits["small_part"] = boost_small_part_scores(
            group_id,
            scores,
            part_areas,
            weight=small_w,
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


def centroid_in_free(poly, free_poly: BaseGeometry | None) -> bool:
    if free_poly is None or free_poly.is_empty or poly is None or poly.is_empty:
        return False
    c = poly.centroid
    return bool(free_poly.contains(c) or free_poly.intersects(c))


def xy_in_free(x: float, y: float, free_poly: BaseGeometry | None) -> bool:
    if free_poly is None or free_poly.is_empty:
        return False
    p = Point(float(x), float(y))
    return bool(free_poly.contains(p) or free_poly.intersects(p))


def count_selected_in_free(
    polys: list,
    selected: Sequence[int],
    free_poly: BaseGeometry | None,
) -> int:
    return sum(1 for i in selected if centroid_in_free(polys[i], free_poly))


def count_graph_in_free(polys: list, free_poly: BaseGeometry | None) -> int:
    return sum(1 for p in polys if centroid_in_free(p, free_poly))


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
    """P3: re-add nest-void nodes missing from refine if collision-clear.

    Uses ``graph.collisions`` (not Shapely pose-clear). Optional ``stats_out``
    records pin_candidates / pin_added / pin_blocked_collision / pin_ms.
    """
    t0 = time.perf_counter()
    refine = list(selected_refine)
    refine_set = set(refine)
    candidates = [
        i for i in selected_nest
        if i not in refine_set and centroid_in_free(polys[i], free_poly)
    ]
    pin_added = 0
    pin_blocked = 0
    if not candidates:
        if stats_out is not None:
            stats_out["pin_candidates"] = 0
            stats_out["pin_added"] = 0
            stats_out["pin_blocked_collision"] = 0
            stats_out["pin_ms"] = (time.perf_counter() - t0) * 1000.0
        return refine
    if scores is not None and len(scores) == len(graph.group_id):
        candidates.sort(key=lambda v: scores[v], reverse=True)
    for v in candidates:
        if any(u in refine_set for u in graph.collisions[v]):
            pin_blocked += 1
            continue
        refine.append(v)
        refine_set.add(v)
        pin_added += 1
    if stats_out is not None:
        stats_out["pin_candidates"] = len(candidates)
        stats_out["pin_added"] = pin_added
        stats_out["pin_blocked_collision"] = pin_blocked
        stats_out["pin_ms"] = (time.perf_counter() - t0) * 1000.0
    return refine


def zones_have_void_hijack(zones: Sequence[str] | None) -> bool:
    if not zones:
        return False
    return any("void_seek(large_void)" in str(z) for z in zones)
