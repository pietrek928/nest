"""Propose orchestration: staged candidate emit, ranking handoff, batch loops.

phase2-pipeline-split — kept as one module on purpose. After the collect split
the file is ~2.5k LOC, and the collect stack itself is only ~590 of those
(``_CollectState`` 62, ``_collect_pocket_candidates`` 93,
``_collect_builder_candidates`` 107, ``_collect_explorer_candidates`` 183,
``_collect_candidates`` 69, ``collect_propose_candidates`` 77). Splitting those
into ``collect_*.py`` would move ~590 LOC while adding six cross-module hops on
the hottest path and would not shrink the actual outlier,
``proposed_transforms_for_groups`` (716 LOC), which is a per-group orchestration
loop, not a proposer. Stage-extracting that loop (plan phase 3) is the split
worth doing; do it before carving up the collect stages.
"""

from typing import List, Optional, Sequence, Tuple, Union
import logging
import math
from dataclasses import replace
from math import hypot

import numpy as np
from shapely import Point, Polygon
from shapely.geometry.base import BaseGeometry
from shapely.ops import unary_union

from nest_graph.board import board_context_from_geometry
from nest_graph.config import ProposeConfig, RankingMode, dedupe_transforms, floor_void_seek_budgets
from nest_graph.geometry import Geometry
from nest_graph.proposer_names import (
    BATCH_FOLLOW_PROPOSERS,
    CORRIDOR_PROPOSERS,
    FIRST_PASS_EMPTY_BORDER_PROPOSERS,
    ProposerName,
    first_pass_packed_border_proposers,
)
from nest_graph.utils import transform_poly

from nest_graph.propose.context import (
    placement_contact_error,
    analyze_free_space,
    apply_proposer_pool_scales,
    border_focal_for_propose,
    build_free_space_snapshot,
    classify_propose_zone_info,
    corridor_channel_samples,
    corridor_channel_target,
    corridor_seed_coords_from_samples,
    effective_ranking_mode,
    focal_shape_for_propose,
    FreeSpaceSnapshot,
    local_packed_near_target,
    nearest_channel_attract,
    obstacle_shape_for_propose,
    part_extents,
    part_is_concave,
    propose_push_point,
    sheet_has_narrow_corridor,
    should_use_border_focus,
    simplify_obstacle_union,
    void_pole_seed_coords,
    PlaceZoneInfo,
)
from nest_graph.propose.geometry import ProposeGeometry, batch_valid_flags, filter_candidates_batch
from nest_graph.propose.void_selection import transform_row_key
from nest_graph.propose.placements_edge import (
    propose_placements_group_fit,
    propose_placements_ribbon_free,
    propose_placements_sheet_corners,
    propose_placements_sheet_edge,
    propose_placements_side_pack,
)
from nest_graph.propose.placements_geo import propose_placements_raycasting, propose_placements_voronoi
from nest_graph.propose.placements_guidance import (
    propose_placements_board_edge,
    propose_placements_guidance_cast,
    propose_placements_guidance_walk,
)
from nest_graph.propose.placements_primary import (
    propose_placements_erosion,
    propose_placements_neighbor_slide,
    propose_placements_perimeter_walk,
)
from nest_graph.propose.placement_common import cluster_seed_coords, is_pose_clear
from nest_graph.propose.types import (
    PackedProposeExtras,
    PocketStats,
    ProposeContext,
    make_propose_context,
)
from nest_graph.propose.placements_pattern import (
    extract_cluster_patterns,
    merge_cluster_patterns,
    propose_placements_cluster_copy,
    synthesize_mate_patterns,
)
from nest_graph.propose.placements_free_space_cloud import (
    propose_placements_free_space_cloud,
)
from nest_graph.propose.placements_pocket import (
    TAG_MOTIF_HOLE,
    propose_placements_pocket_fit,
)
from nest_graph.propose.placements_pso import propose_placements_point_cloud
from nest_graph.propose.ranking import (
    _rank_proposal_coords,
    _score_placement_border,
    _score_placement_clearance,
    _trim_candidates_by_clearance,
    _trim_candidates_by_contact,
    _trim_candidates_stratified,
    cast_squeeze_ranked_coords,
    select_guidance_cast_seeds,
)
from nest_graph.propose.placements_selection_expand import (
    propose_placements_history_expand,
    propose_placements_selection_expand,
)
from nest_graph.propose.pose_diversity import (
    apply_conflict_degree_penalty,
    apply_pose_nms,
)
from nest_graph.propose.void_selection import transform_row_key, void_pole_near_radius

_LOG = logging.getLogger(__name__)
_proposal_key = transform_row_key


_CASCADE_HARD_ZONES = frozenset({"void_seek", "interior_pocket"})
_CASCADE_EXPLORERS = frozenset({
    "perimeter_walk",
    "neighbor_slide",
    "erosion",
    "raycasting",
    "voronoi",
    "point_cloud",
    "guidance_walk",
    "ribbon_free",
})
_CASCADE_BUILDERS = frozenset({
    "group_fit",
    "side_pack",
    "board_edge",
    "sheet_corners",
    "sheet_edge",
})


def _fast_packed_crowd_ref(packed_transforms: Sequence | None) -> Point | None:
    """Bbox center of packed (x, y) — O(N), no Shapely union."""
    if not packed_transforms:
        return None
    xs: list[float] = []
    ys: list[float] = []
    for t in packed_transforms:
        if t is None:
            continue
        try:
            xs.append(float(t[0]))
            ys.append(float(t[1]))
        except (TypeError, IndexError, ValueError):
            continue
    if not xs:
        return None
    return Point(0.5 * (min(xs) + max(xs)), 0.5 * (min(ys) + max(ys)))


def _packed_count(extras: PackedProposeExtras) -> int:
    if extras.packed_transforms:
        return len(extras.packed_transforms)
    if extras.packed_polys:
        return len(extras.packed_polys)
    return 0


def _emit_side_pack(
    ctx: ProposeContext,
    extras: PackedProposeExtras,
    state: _CollectState,
    *,
    top_n: int | None = None,
    cascade_zone: str | None = None,
) -> None:
    cfg = ctx.propose_cfg
    if not (
        _proposer_enabled("side_pack", ctx.enabled_proposers)
        and bool(getattr(cfg, "use_side_pack", True))
    ):
        return
    crowd = _fast_packed_crowd_ref(extras.packed_transforms)
    pole = extras.void_pole if extras.void_pole is not None else ctx.void_pole
    # Pack bbox is the anti-crowd ref; void_pole substitutes only when pack is empty.
    if crowd is None and pole is not None and not getattr(pole, "is_empty", True):
        crowd = pole
    cap = int(top_n) if top_n is not None else int(getattr(cfg, "side_pack_top_n", 16) or 16)
    # Void / densify: over-emit so packing filter still leaves survivors.
    if cascade_zone == "void_seek":
        cap = max(cap, max(int(cfg.max_proposals) // 2, 16))
    cap = max(1, min(cap, max(int(ctx.pool), 1)))
    state.ext(
        "side_pack",
        propose_placements_side_pack(
            ctx.shape_to_place,
            ctx.sheet,
            min_dist=ctx.min_dist,
            propose_cfg=cfg,
            propose_geom=ctx.propose_geom,
            top_n=cap,
            crowd_ref=crowd,
            samples_per_edge=int(getattr(cfg, "side_pack_samples_per_edge", 12) or 12),
            num_angles=min(ctx.n_angles, 8),
        ),
        max_items=cap * 2,
    )


def _emit_board_edge_reserve(
    ctx: ProposeContext,
    extras: PackedProposeExtras,
    state: _CollectState,
    *,
    reserve_only: bool = False,
) -> bool:
    """Emit board_edge snaps. Returns True when the proposer was armed.

    ``reserve_only`` caps to ``board_edge_batch_reserve`` (mid-pack / void /
    cascade sniper bypass). Full pool×2 when False and not mid-pack side path.
    """
    cfg = ctx.propose_cfg
    if not (
        bool(cfg.use_board_edge_seeds)
        and _proposer_enabled("board_edge", ctx.enabled_proposers)
        and (ctx.border_focus or cfg.board_edge_when_packed)
    ):
        return False
    packed_near_border = True
    if ctx.base_shape is not None and not ctx.base_shape.is_empty:
        packed_near_border = float(
            ctx.base_shape.distance(ctx.sheet.exterior)
        ) <= max(ctx.min_dist * 8.0, 1e-3)
    if not (packed_near_border or ctx.border_focus):
        return False
    n_angles = ctx.n_angles
    pool = ctx.pool
    be_angles = (
        max(n_angles, 12)
        if ctx.base_shape is None or ctx.base_shape.is_empty
        else min(n_angles, 8)
    )
    packed_n = _packed_count(extras)
    mid_pack = packed_n >= 2 or reserve_only
    be_top = (
        max(int(cfg.board_edge_batch_reserve), 1)
        if mid_pack
        else pool * 2
    )
    state.ext(
        "board_edge",
        propose_placements_board_edge(
            ctx.shape_to_place,
            ctx.sheet,
            ctx.base_shape,
            min_dist=ctx.min_dist,
            propose_cfg=cfg,
            propose_geom=ctx.propose_geom,
            pt_push=ctx.pt_push,
            num_angles=be_angles,
            top_n=be_top,
        ),
        max_items=be_top,
    )
    return True


def _cascade_active(propose_cfg: ProposeConfig, zone: str | None) -> bool:
    return bool(getattr(propose_cfg, "propose_cascade_short_circuit", False)) and bool(zone)


def _count_fast_valid_seeds(
    seeds: Sequence[Tuple[float, float, float]],
    propose_geom: ProposeGeometry,
    *,
    limit: int,
    pt_push: Point | None = None,
    crash_counter: dict | None = None,
) -> int:
    if not seeds or limit <= 0:
        return 0
    sample = list(seeds)[: max(limit * 3, limit)]
    push = pt_push if pt_push is not None else Point(0.0, 0.0)
    try:
        flags = batch_valid_flags(propose_geom, sample, push)
    except (ValueError, RuntimeError) as e:
        _LOG.warning("Fast valid check crashed: %s", e)
        if crash_counter is not None:
            crash_counter["cascade_crashes"] = int(
                crash_counter.get("cascade_crashes", 0)
            ) + 1
        return 0
    return sum(1 for f in flags if f)


def propositions_to_ndarray(coords_list: Sequence[Tuple[float, float, float]]) -> np.ndarray:
    if not coords_list:
        return np.zeros((0, 3), dtype=np.float64)
    return np.asarray(coords_list, dtype=np.float64)


def _count_transforms_in_void(
    arr: np.ndarray,
    target_poly: Polygon | None,
    part_poly: Polygon,
) -> int:
    """Count SE(2) rows whose placed centroid lies in ``target_poly`` (iv)."""
    if (
        arr is None
        or getattr(arr, "shape", (0,))[0] == 0
        or target_poly is None
        or target_poly.is_empty
        or part_poly is None
        or part_poly.is_empty
    ):
        return 0
    n = 0
    for row in np.asarray(arr, dtype=np.float64).reshape(-1, 3):
        placed = transform_poly(
            part_poly, (float(row[0]), float(row[1]), float(row[2])),
        )
        if target_poly.covers(placed.centroid):
            n += 1
    return n


def _count_transforms_pole_near(
    arr: np.ndarray,
    void_pole: Point | None,
    part_poly: Polygon,
    *,
    sheet_diag: float,
    near_ratio: float = 0.25,
) -> int:
    """Count packing-candidate rows with centroid within near_ratio * sheet_diag of pole."""
    if (
        arr is None
        or getattr(arr, "shape", (0,))[0] == 0
        or void_pole is None
        or getattr(void_pole, "is_empty", True)
        or part_poly is None
        or part_poly.is_empty
        or sheet_diag <= 1e-12
    ):
        return 0
    thr = void_pole_near_radius(sheet_diag, near_ratio)
    if thr <= 0.0:
        return 0
    n = 0
    for row in np.asarray(arr, dtype=np.float64).reshape(-1, 3):
        placed = transform_poly(
            part_poly, (float(row[0]), float(row[1]), float(row[2])),
        )
        if placed is None or placed.is_empty:
            continue
        c = placed.centroid
        if float(Point(float(c.x), float(c.y)).distance(void_pole)) <= thr:
            n += 1
    return n


def allow_void_repack(
    *,
    free_kind: str | None,
    n_void_nest: int,
    n_void_refine: int,
) -> bool:
    """Unified allow_repack predicate (§9)."""
    if free_kind == "large_void":
        return True
    return int(n_void_nest) > int(n_void_refine)


def _void_seek_densify(
    *,
    arr: np.ndarray,
    zone: str | None,
    zone_label: str,
    board: BaseGeometry,
    sheet: Polygon,
    part_poly: Polygon,
    placed: Sequence[BaseGeometry],
    obstacle_shape: BaseGeometry,
    propose_cfg: ProposeConfig,
    cfg: ProposeConfig,
    min_dist: float,
    primary_target: Point | None,
    void_hijack_from: str | None,
    part_free_space,
    full_packed_geoms: Sequence[Geometry],
    cluster_patterns,
    angle_override,
    corridor_guidance_seeds,
    packed_gids_aligned,
    packed_trs_aligned,
    poles_reserve_only: bool,
    rules,
    group_id: int,
    densify_count_floor: int,
    floor_ratio: float,
    group_pocket_emitted: int,
    free_ratio: float,
    sterile_zones: tuple[str, ...],
    border_only_propose: bool,
    push: Point,
) -> tuple[np.ndarray, str, bool, bool, str | None, dict]:
    """Sterile densify + optional free_space_cloud (§3, §6). Behavior-neutral extract."""
    telem: dict = {
        "total_counts": {},
        "proposer_keys": {},
        "emitted_by_proposer": {},
        "pool_by_proposer": {},
        "pocket_by_tag": {},
        "pocket_emitted": 0,
        "pocket_attempted": 0,
        "pocket_accepted": 0,
        "pocket_keys": [],
        "motif_hole_keys": [],
        "motif_skip": {},
        "pocket_skips": [],
    }
    hijack_densify = (
        bool(propose_cfg.densify_on_void_hijack)
        and placed
        and not border_only_propose
        and (
            void_hijack_from is not None
            or zone == "void_seek"
        )
        and (
            group_pocket_emitted == 0
            or arr.shape[0] < densify_count_floor
        )
    )
    need_densify = hijack_densify or (
        arr.shape[0] < densify_count_floor
        and placed
        and not border_only_propose
        and free_ratio > 0.2
        and zone in sterile_zones
    )
    if (
        (not need_densify)
        and floor_ratio > 0.0
        and arr.shape[0] > 0
        and placed
        and not border_only_propose
        and (
            (zone in sterile_zones and free_ratio > 0.2)
            or (
                void_hijack_from is not None
                and bool(propose_cfg.densify_on_void_hijack)
            )
        )
    ):
        densify_geom = ProposeGeometry(
            board,
            obstacle_shape,
            part_poly,
            min_dist,
            epsilon_ratio=cfg.placement_clearance_epsilon_ratio,
            propose_cfg=cfg,
            full_packed_geoms=full_packed_geoms,
        )
        best_cl = max(
            (
                _score_placement_clearance(tuple(r), densify_geom, push, cfg, min_dist)
                for r in arr
            ),
            default=float("-inf"),
        )
        if best_cl < float(min_dist) * floor_ratio:
            need_densify = True
    if not need_densify:
        return arr, zone_label, False, False, None, telem

    densify_cfg = ProposeConfig.for_place("void_seek", base=propose_cfg)
    densify_rank = (
        RankingMode.CONTACT_HYBRID if propose_cfg.use_pocket_fit else RankingMode.CLEARANCE
    )
    densify_cfg = densify_cfg.model_copy(update={
        "use_guidance_propositions": True,
        "guidance_use_tight_packing": True,
        "guidance_use_corner_alignment": True,
        "guidance_enable_grid": False,
        "cast_squeeze_top_k": max(4, int(densify_cfg.cast_squeeze_top_k)),
        "use_neighbor_slide": False,
        "use_ribbon_seeds": True,
        "use_full_packed_obstacle": True,
        "use_border_focus": False,
        "border_focus_ranking": False,
        "ranking_mode": densify_rank,
        "use_contact_ranking": densify_rank == RankingMode.CONTACT_HYBRID,
        "use_contact_clearance_hybrid": densify_rank == RankingMode.CONTACT_HYBRID,
        "propose_cascade_short_circuit": bool(
            propose_cfg.propose_cascade_short_circuit
        ),
        "cascade_sniper_stop_n": int(propose_cfg.cascade_sniper_stop_n),
        "cascade_explorer_budget_scale": float(
            propose_cfg.cascade_explorer_budget_scale
        ),
    })
    densify_cfg = apply_proposer_pool_scales(
        densify_cfg, propose_cfg.place_proposer_pool_scales,
    )
    densify_cfg = floor_void_seek_budgets(densify_cfg)
    densify_obs = simplify_obstacle_union(placed, min_dist)
    if densify_obs is None or densify_obs.is_empty:
        densify_obs = obstacle_shape_for_propose(
            placed,
            part_poly,
            min_dist,
            propose_cfg=densify_cfg,
            sheet=sheet,
            ref_point=primary_target,
        )
    densify_push = (
        primary_target
        if primary_target is not None
        else propose_push_point(
            board,
            densify_obs,
            smart_push=True,
            min_dist=min_dist,
            use_border_focus=False,
        )
    )
    densify_enabled = ProposeConfig.proposers_for_place("void_seek")
    densify_counts: dict[str, int] = {}
    densify_pocket: dict = {}
    densify_keys: dict[str, set[tuple[float, float, float]]] = {}
    group_cascade: dict = {}
    group_diversity: dict = {}
    densify_coords = propose_coords_with_strategy(
        densify_obs,
        part_poly,
        board,
        densify_cfg,
        min_dist=min_dist,
        pt_push=densify_push,
        focal_shape=densify_obs,
        enabled_proposers=densify_enabled,
        rules=rules,
        group_id=group_id,
        proposer_counts=densify_counts,
        full_packed_geoms=full_packed_geoms,
        cluster_patterns=cluster_patterns,
        placement_angles_override=angle_override,
        guidance_seed_coords=corridor_guidance_seeds,
        packed_polys=placed,
        packed_group_ids=packed_gids_aligned,
        packed_transforms=packed_trs_aligned,
        pocket_stats_out=densify_pocket,
        free_space=part_free_space,
        guidance_reserve_only=poles_reserve_only,
        void_pole=primary_target,
        proposer_keys_out=densify_keys,
        cascade_zone="void_seek",
        cascade_stats_out=group_cascade,
        diversity_stats_out=group_diversity,
    )
    for name, n in densify_counts.items():
        n_u = len(densify_keys.get(name) or ())
        telem["total_counts"][name] = n_u if n_u else int(n)
    for name, keys in densify_keys.items():
        telem["proposer_keys"][name] = set(keys)
        telem["emitted_by_proposer"][name] = len(keys)
    for name, n in (densify_pocket.get("pool_by_proposer") or {}).items():
        telem["pool_by_proposer"][name] = int(n)
    for tag, n in (densify_pocket.get("by_tag") or {}).items():
        telem["pocket_by_tag"][tag] = int(n)
    telem["pocket_emitted"] = int(densify_pocket.get("emitted", 0))
    telem["pocket_attempted"] = int(densify_pocket.get("attempted", 0))
    telem["pocket_accepted"] = int(densify_pocket.get("selected", 0))
    telem["pocket_keys"] = list(densify_pocket.get("pocket_keys") or [])
    telem["motif_hole_keys"] = list(densify_pocket.get("motif_hole_keys") or [])
    telem["motif_skip"] = dict(densify_pocket.get("motif_skip") or {})
    telem["motif_cohorts"] = list(densify_pocket.get("motif_cohorts") or [])
    telem["pocket_skips"] = [str(sk) for sk in (densify_pocket.get("pocket_skip") or [])]

    densify_arr = propositions_to_ndarray(densify_coords)
    yield_poly = None
    if part_free_space is not None and part_free_space.analysis is not None:
        yield_poly = part_free_space.analysis.target_poly
    use_void_yield = (
        bool(getattr(propose_cfg, "enable_void_yield_densify_accept", True))
        and (void_hijack_from is not None or zone == "void_seek")
        and yield_poly is not None
        and not yield_poly.is_empty
    )
    accepted = False
    reason: str | None = None
    old_iv = 0
    new_iv = 0
    if use_void_yield:
        old_iv = _count_transforms_in_void(arr, yield_poly, part_poly)
        new_iv = _count_transforms_in_void(densify_arr, yield_poly, part_poly)
        pole_pt = primary_target
        sheet_diag = 0.0
        if sheet is not None and not sheet.is_empty:
            minx, miny, maxx, maxy = sheet.bounds
            sheet_diag = float(np.hypot(maxx - minx, maxy - miny))
        near_r = float(
            getattr(propose_cfg, "void_pole_near_diag_ratio", 0.25) or 0.25
        )
        use_pole = bool(
            getattr(propose_cfg, "enable_void_pole_clear_densify", True)
        )
        old_pn = (
            _count_transforms_pole_near(
                arr, pole_pt, part_poly,
                sheet_diag=sheet_diag, near_ratio=near_r,
            )
            if use_pole else 0
        )
        new_pn = (
            _count_transforms_pole_near(
                densify_arr, pole_pt, part_poly,
                sheet_diag=sheet_diag, near_ratio=near_r,
            )
            if use_pole else 0
        )
        if new_iv > old_iv:
            accepted = True
            reason = "void_yield_gain"
            arr = densify_arr[: densify_cfg.max_proposals]
            zone_label = f"{zone}→void_seek"
        elif (
            old_iv == 0
            and new_iv == 0
            and use_pole
            and new_pn > old_pn
        ):
            accepted = True
            reason = "void_pole_clear"
            arr = densify_arr[: densify_cfg.max_proposals]
            zone_label = f"{zone}→void_seek"
        else:
            reason = "void_yield_drop"
    elif densify_arr.shape[0] > arr.shape[0] or (
        arr.shape[0] == 0 and densify_arr.shape[0] > 0
    ):
        accepted = True
        reason = "count_gain"
        arr = densify_arr[: densify_cfg.max_proposals]
        zone_label = f"{zone}→void_seek"
    else:
        reason = "count_drop"

    void_path = void_hijack_from is not None or zone == "void_seek"
    # Fire cloud when the pool is empty OR densify left zero in-void centroids
    # (one rim survivor must not block void-seek cloud fallback).
    need_cloud = arr.shape[0] == 0 or (
        not accepted and old_iv == 0 and new_iv == 0
    )
    if (
        need_cloud
        and void_path
        and bool(getattr(propose_cfg, "use_free_space_cloud", True))
        and _proposer_enabled("free_space_cloud", densify_enabled)
        and yield_poly is not None
        and not yield_poly.is_empty
    ):
        cloud_geom = ProposeGeometry(
            board,
            densify_obs,
            part_poly,
            min_dist,
            epsilon_ratio=cfg.placement_clearance_epsilon_ratio,
            propose_cfg=densify_cfg,
            full_packed_geoms=full_packed_geoms,
        )
        cloud = propose_placements_free_space_cloud(
            yield_poly,
            propose_geom=cloud_geom,
            propose_cfg=propose_cfg,
            top_n=max(int(densify_cfg.max_proposals), 1),
            allowed_angles=angle_override,
        )
        if cloud:
            accepted = True
            reason = "free_space_cloud"
            arr = propositions_to_ndarray(cloud)[: densify_cfg.max_proposals]
            zone_label = f"{zone}→void_seek"
            cloud_keys = {_proposal_key(c) for c in cloud}
            telem["proposer_keys"]["free_space_cloud"] = cloud_keys
            telem["emitted_by_proposer"]["free_space_cloud"] = len(cloud)
            telem["pool_by_proposer"]["free_space_cloud"] = len(cloud)
            telem["total_counts"]["free_space_cloud"] = len(cloud)

    return arr, zone_label, True, accepted, reason, telem


def base_shape_from_selection(
    polys: Sequence[BaseGeometry],
    selected_indices: Sequence[int],
) -> Union[Polygon, BaseGeometry]:
    placed = [polys[i] for i in selected_indices]
    if not placed:
        return Polygon()
    return unary_union(placed)


def _proposer_enabled(
    name: ProposerName | str,
    enabled_proposers: frozenset[ProposerName] | frozenset[str] | None,
) -> bool:
    if enabled_proposers is None:
        return True
    key = name.value if isinstance(name, ProposerName) else str(name)
    return key in _normalize_proposers(enabled_proposers)


def _normalize_proposers(iterable) -> set[str]:
    """Canonical proposer name set (enum.value or str) for safe set difference."""
    out: set[str] = set()
    if iterable is None:
        return out
    for item in iterable:
        if isinstance(item, ProposerName):
            out.add(item.value)
        else:
            out.add(str(item))
    return out


def _extend_counted(
    candidates: list[tuple[float, float, float]],
    proposer_counts: dict[str, int] | None,
    name: str,
    new_items: Sequence[tuple[float, float, float]],
    *,
    max_items: int | None = None,
    proposer_keys: dict[str, set[tuple[float, float, float]]] | None = None,
    claimed_keys: set[tuple[float, float, float]] | None = None,
) -> None:
    items = list(new_items)
    if max_items is not None and len(items) > max_items:
        items = items[:max_items]
    to_add: list[tuple[float, float, float]] = []
    for item in items:
        key = _proposal_key(item)
        if claimed_keys is not None:
            if key in claimed_keys:
                continue
            claimed_keys.add(key)
        if proposer_keys is not None:
            proposer_keys.setdefault(name, set()).add(key)
        to_add.append(item)
    if proposer_counts is not None:
        proposer_counts[name] = proposer_counts.get(name, 0) + len(to_add)
    candidates.extend(to_add)


def _filter_valid_candidates(
    candidates: Sequence[Tuple[float, float, float]],
    propose_geom: ProposeGeometry,
    pt_push: Point,
    min_dist: float,
    epsilon_ratio: float,
    *,
    guidance_cfg=None,
) -> List[Tuple[float, float, float]]:
    if not candidates:
        return []
    return filter_candidates_batch(
        propose_geom,
        candidates,
        pt_push,
        guidance_cfg=guidance_cfg,
    )


def pre_filter_candidates(
    candidates: Sequence[Tuple[float, float, float]],
    propose_geom: ProposeGeometry,
    pt_push: Point,
    min_dist: float,
    epsilon_ratio: float,
    *,
    trim_by_validity: bool = True,
    guidance_cfg=None,
) -> List[Tuple[float, float, float]]:
    if not candidates:
        return []
    pool = list(candidates)
    if propose_geom.full_packed_geoms:
        pool = [
            c for c in pool
            if propose_geom.passes_full_packed_collision(propose_geom.placed_at(c))
        ]
    if not pool or not trim_by_validity:
        return pool
    return _filter_valid_candidates(
        pool, propose_geom, pt_push, min_dist, epsilon_ratio,
        guidance_cfg=guidance_cfg,
    )


def _filter_distant_collisions(
    candidates: Sequence[Tuple[float, float, float]],
    propose_geom: ProposeGeometry,
) -> List[Tuple[float, float, float]]:
    return pre_filter_candidates(
        candidates,
        propose_geom,
        Point(0, 0),
        0.0,
        0.0,
        trim_by_validity=False,
    )


class _CollectState:
    """Per-call emit bookkeeping for the collect stages.

    Owns the candidate list, per-proposer counts/keys, and cascade skip
    telemetry so the pocket / builder / explorer stages stay pure functions of
    ``(ctx, extras, state)``.
    """

    def __init__(
        self,
        *,
        proposer_counts: dict[str, int] | None,
        proposer_keys: dict[str, set[tuple[float, float, float]]] | None,
        cascade_stats_out: dict | None,
    ) -> None:
        self.candidates: List[Tuple[float, float, float]] = []
        self.proposer_counts = proposer_counts
        self.proposer_keys = proposer_keys
        # Always dedupe emit→pool keys so counts match unique survivors.
        self.claimed_keys: set[tuple[float, float, float]] = set()
        self.cascade_stats_out = cascade_stats_out
        self.skip_builders = False
        self.skip_explorers = False
        self.explorer_scale = 1.0
        self.board_edge_reserved = False
        if cascade_stats_out is not None:
            cascade_stats_out.setdefault("cascade_stopped_after", "none")
            cascade_stats_out.setdefault("cascade_skipped_proposers", [])

    def ext(
        self,
        name: str,
        new_items: Sequence[tuple[float, float, float]],
        *,
        max_items: int | None = None,
    ) -> None:
        _extend_counted(
            self.candidates,
            self.proposer_counts,
            name,
            new_items,
            max_items=max_items,
            proposer_keys=self.proposer_keys,
            claimed_keys=self.claimed_keys,
        )

    def mark_skip(self, stage: str, names: Sequence[str]) -> None:
        if self.cascade_stats_out is None:
            return
        self.cascade_stats_out["cascade_stopped_after"] = stage
        skipped = list(self.cascade_stats_out.get("cascade_skipped_proposers") or [])
        for n in names:
            if n not in skipped:
                skipped.append(n)
        self.cascade_stats_out["cascade_skipped_proposers"] = skipped

    def explorer_cap(self, base_cap: int) -> int:
        if self.skip_explorers:
            return 0
        if self.explorer_scale >= 0.999:
            return base_cap
        return max(1, int(math.ceil(base_cap * self.explorer_scale)))


def _collect_pocket_candidates(
    ctx: ProposeContext,
    extras: PackedProposeExtras,
    state: _CollectState,
    *,
    group_id: int,
    guidance_reserve_only: bool,
) -> tuple[list[tuple[float, float, float]], list[tuple[float, float, float]]]:
    """Sniper stage: corridor / void poles, ``pocket_fit``, then ``cluster_copy``.

    AGENTS emit order is static here: poles → ``pocket_fit`` → ``cluster_copy``
    (``cluster_copy`` always before any sweeper).
    Returns ``(pocket_reserve, motif_reserve)``.
    """
    cfg = ctx.propose_cfg
    packed_list = list(extras.packed_polys) if extras.packed_polys is not None else []
    pocket_coords: list[tuple[float, float, float]] = []
    motif_coords: list[tuple[float, float, float]] = []

    if ctx.guidance_seed_coords and not guidance_reserve_only:
        state.ext(
            "corridor_channel",
            list(ctx.guidance_seed_coords),
            max_items=len(ctx.guidance_seed_coords),
        )

    # Pocket teleports early so they sit in the candidate pool before slides.
    if (
        _proposer_enabled("pocket_fit", ctx.enabled_proposers)
        and cfg.use_pocket_fit
        and packed_list
    ):
        allowed = None
        if (
            ctx.placement_angles_override is not None
            and len(ctx.placement_angles_override) > 0
        ):
            allowed = [
                float(a)
                for a in np.asarray(ctx.placement_angles_override).reshape(-1)
            ]
        stats = extras.pocket_stats
        tags: list[str] = []
        attempts: list[int] = []
        skips: list[str] = []
        pocket_coords = propose_placements_pocket_fit(
            ctx.shape_to_place,
            ctx.sheet,
            packed_list,
            min_dist=ctx.min_dist,
            propose_geom=ctx.propose_geom,
            pt_push=ctx.pt_push,
            propose_cfg=cfg,
            group_id=group_id,
            allowed_angles=allowed,
            cluster_patterns=extras.cluster_patterns,
            packed_group_ids=extras.packed_group_ids,
            packed_transforms=extras.packed_transforms,
            top_n=max(ctx.pool, 8),
            tags_out=tags,
            attempts_out=attempts,
            free_space=extras.free_space,
            skip_reasons_out=skips,
        )
        state.ext("pocket_fit", pocket_coords, max_items=len(pocket_coords))
        if stats is not None:
            stats.tags.extend(tags)
            if attempts:
                stats.attempts = int(attempts[0])
            for sk in skips:
                stats.skip_reasons[str(sk)] = stats.skip_reasons.get(str(sk), 0) + 1

    if (
        _proposer_enabled("cluster_copy", ctx.enabled_proposers)
        and cfg.use_cluster_copy
        and extras.cluster_patterns
    ):
        motif_skips: dict[str, int] = {}
        motif_cohorts: list = []
        motif_coords = propose_placements_cluster_copy(
            extras.cluster_patterns,
            group_id,
            ctx.shape_to_place,
            ctx.sheet,
            ctx.base_shape,
            min_dist=ctx.min_dist,
            propose_geom=ctx.propose_geom,
            pt_push=ctx.pt_push,
            propose_cfg=cfg,
            top_n=ctx.pool,
            free_space=extras.free_space,
            void_pole=extras.void_pole if extras.void_pole is not None else ctx.void_pole,
            skip_reasons=motif_skips,
            cohorts_out=motif_cohorts,
        )
        # Track D / Q25: Scene dry-run motif reserve only (after packing emit, before claim).
        if motif_coords and bool(getattr(cfg, "enable_motif_scene_dry_run", False)):
            from nest_graph.propose.placement_common import as_geometry, is_pose_clear

            voids = list(getattr(ctx.propose_geom.scene, "void_geoms", None) or [])
            packed = list(getattr(ctx.propose_geom, "full_packed_geoms", None) or [])
            kept_motif: list[tuple[float, float, float]] = []
            dropped = 0
            for coords in motif_coords:
                placed = ctx.propose_geom.placed_at(coords)
                cg = as_geometry(placed) if placed is not None else None
                if cg is not None and is_pose_clear(
                    cg, voids, packed, float(ctx.min_dist),
                ):
                    kept_motif.append(coords)
                else:
                    dropped += 1
            if dropped:
                motif_skips["scene_dry_run"] = (
                    motif_skips.get("scene_dry_run", 0) + dropped
                )
            motif_coords = kept_motif
        state.ext("cluster_copy", motif_coords)
        if extras.pocket_stats is not None:
            for k, v in motif_skips.items():
                extras.pocket_stats.skip_reasons[f"motif_{k}"] = (
                    extras.pocket_stats.skip_reasons.get(f"motif_{k}", 0) + int(v)
                )
            extras.pocket_stats.motif_cohorts.extend(motif_cohorts)

    return list(pocket_coords), list(motif_coords)


def _collect_builder_candidates(
    ctx: ProposeContext,
    extras: PackedProposeExtras,
    state: _CollectState,
    *,
    cascade_zone: str | None = None,
) -> None:
    """Builder stage: sheet/cluster edge snaps + group_fit.

    Zone permission is ``ZONE_PROPOSERS`` / ``enabled_proposers``. Staging:
    mid-pack / void enables ``side_pack``; ``board_edge`` stays available with a
    reserve quota (no hard XOR-off when packed_n >= 2).
    """
    if state.skip_builders:
        return
    cfg = ctx.propose_cfg
    pool = ctx.pool
    n_angles = ctx.n_angles
    border_focus = ctx.border_focus
    packed_n = _packed_count(extras)
    void_path = cascade_zone == "void_seek"
    # Staging: mid-pack or void → side_pack; board_edge may co-exist via reserve.
    use_side = bool(getattr(cfg, "use_side_pack", True)) and (
        packed_n >= 2 or void_path
    )
    use_board = bool(cfg.use_board_edge_seeds)

    # Void path: side_pack XOR group_fit. Non-void: group_fit when enabled.
    run_group_fit = (
        (not void_path)
        and _proposer_enabled("group_fit", ctx.enabled_proposers)
        and cfg.use_group_edge_seeds
        and ctx.focal_shape is not None
        and not ctx.focal_shape.is_empty
        and not ctx.base_shape.is_empty
        and ctx.use_free_region
    )
    if run_group_fit:
        gf_angles = min(n_angles, 8)
        state.ext(
            "group_fit",
            propose_placements_group_fit(
                ctx.focal_shape,
                ctx.shape_to_place,
                ctx.sheet,
                ctx.base_shape,
                min_dist=ctx.min_dist,
                num_angles=gf_angles,
                top_n=pool * 2,
                samples_per_edge=cfg.group_edge_samples_per_edge,
                propose_geom=ctx.propose_geom,
                pt_push=ctx.pt_push,
            ),
        )

    # Rim reserve before side_pack so claimed_keys keep board_edge snaps.
    # Skip if void_seek already reserved early (before explorers).
    board_edge_on = (
        use_board
        and _proposer_enabled("board_edge", ctx.enabled_proposers)
        and (border_focus or cfg.board_edge_when_packed)
        and not state.board_edge_reserved
    )
    if board_edge_on:
        _emit_board_edge_reserve(ctx, extras, state, reserve_only=use_side)

    # Zone permission via enabled_proposers inside _emit_side_pack.
    if use_side:
        _emit_side_pack(ctx, extras, state, cascade_zone=cascade_zone)

    if (
        _proposer_enabled("sheet_corners", ctx.enabled_proposers)
        and cfg.use_border_edge_seeds
        and border_focus
        and not board_edge_on
        and not use_side
    ):
        # Fold sheet_edge samples into sheet_corners counting (no separate key).
        state.ext(
            "sheet_corners",
            propose_placements_sheet_corners(
                ctx.shape_to_place,
                ctx.sheet,
                ctx.min_dist,
                propose_geom=ctx.propose_geom,
                pt_push=ctx.pt_push,
                num_angles=max(n_angles * 4, 24),
                top_n=pool * 2,
            ),
        )
        state.ext(
            "sheet_corners",
            propose_placements_sheet_edge(
                ctx.shape_to_place,
                ctx.sheet,
                ctx.min_dist,
                propose_geom=ctx.propose_geom,
                pt_push=ctx.pt_push,
                num_angles=max(n_angles, 12),
                top_n=pool * 2,
                samples_per_edge=cfg.sheet_edge_samples_per_edge,
                base_shape=ctx.base_shape,
            ),
        )


def _collect_explorer_candidates(
    ctx: ProposeContext,
    extras: PackedProposeExtras,
    state: _CollectState,
) -> None:
    """Explorer stage: perimeter walk, neighbor slide, erosion, raycast, voronoi,
    point cloud, guidance walk, ribbon."""
    if state.skip_explorers:
        return
    cfg = ctx.propose_cfg
    pool = ctx.pool
    n_angles = ctx.n_angles
    placement_angles = ctx.placement_angles
    border_focus = ctx.border_focus
    use_free_region = ctx.use_free_region

    if _proposer_enabled("perimeter_walk", ctx.enabled_proposers):
        pw_cap = state.explorer_cap(pool * 2)
        if pw_cap > 0:
            state.ext(
                "perimeter_walk",
                propose_placements_perimeter_walk(
                    ctx.base_shape,
                    ctx.shape_to_place,
                    ctx.sheet,
                    ctx.min_dist,
                    propose_geom=ctx.propose_geom,
                    pt_push=ctx.pt_push,
                    use_free_region=use_free_region,
                    border_focus=border_focus,
                    num_angles=n_angles,
                    top_n=pw_cap,
                    placement_angles=placement_angles,
                ),
                max_items=pw_cap,
            )
    if (
        _proposer_enabled("neighbor_slide", ctx.enabled_proposers)
        and cfg.use_neighbor_slide
        and not ctx.base_shape.is_empty
    ):
        neighbor_top = max(
            int(pool * cfg.neighbor_slide_pool_fraction),
            n_angles,
        )
        neighbor_top = state.explorer_cap(neighbor_top)
        if neighbor_top > 0:
            state.ext(
                "neighbor_slide",
                propose_placements_neighbor_slide(
                    ctx.base_shape,
                    ctx.shape_to_place,
                    ctx.sheet,
                    ctx.min_dist,
                    propose_geom=ctx.propose_geom,
                    pt_push=ctx.pt_push,
                    num_angles=n_angles,
                    top_n=neighbor_top,
                    placement_angles=placement_angles,
                ),
                max_items=neighbor_top,
            )
    if _proposer_enabled("erosion", ctx.enabled_proposers):
        er_cap = state.explorer_cap(pool)
        if er_cap > 0:
            state.ext(
                "erosion",
                propose_placements_erosion(
                    ctx.base_shape,
                    ctx.shape_to_place,
                    ctx.sheet,
                    ctx.min_dist,
                    propose_geom=ctx.propose_geom,
                    pt_push=ctx.pt_push,
                    use_free_region=use_free_region,
                    border_focus=border_focus,
                    focal_shape=ctx.focal_shape,
                    num_angles=n_angles,
                    top_n=er_cap,
                    placement_angles=placement_angles,
                ),
                max_items=er_cap,
            )
    if _proposer_enabled("raycasting", ctx.enabled_proposers):
        rc_cap = state.explorer_cap(pool)
        if rc_cap > 0:
            state.ext(
                "raycasting",
                propose_placements_raycasting(
                    ctx.base_shape,
                    ctx.shape_to_place,
                    ctx.sheet,
                    ctx.min_dist,
                    use_free_region=use_free_region,
                    top_n=rc_cap,
                    num_rays=cfg.raycast_num_rays,
                    num_angles=cfg.raycast_num_angles,
                    anchor_stride=cfg.raycast_anchor_stride,
                    focal_shape=ctx.focal_shape,
                    border_focus=border_focus,
                    propose_geom=ctx.propose_geom,
                    pt_push=ctx.pt_push,
                ),
            )
    if _proposer_enabled("voronoi", ctx.enabled_proposers) and cfg.use_voronoi:
        vo_cap = state.explorer_cap(pool)
        if vo_cap > 0:
            state.ext(
                "voronoi",
                propose_placements_voronoi(
                    ctx.base_shape,
                    ctx.shape_to_place,
                    ctx.sheet,
                    ctx.min_dist,
                    use_free_region=use_free_region,
                    top_n=vo_cap,
                    num_angles=cfg.voronoi_num_angles,
                    densify_divisor=cfg.voronoi_densify_divisor,
                    max_sites=cfg.voronoi_max_sites,
                    focal_shape=ctx.focal_shape,
                    border_focus=border_focus,
                    propose_geom=ctx.propose_geom,
                    pt_push=ctx.pt_push,
                ),
            )
    if _proposer_enabled("point_cloud", ctx.enabled_proposers) and cfg.use_point_cloud:
        pc_cap = state.explorer_cap(pool)
        if pc_cap > 0:
            state.ext(
                "point_cloud",
                propose_placements_point_cloud(
                    ctx.base_shape,
                    ctx.shape_to_place,
                    ctx.sheet,
                    pt_push=ctx.pt_push,
                    min_dist=ctx.min_dist,
                    top_n=pc_cap,
                    num_particles=cfg.point_cloud_particles,
                    max_iterations=cfg.point_cloud_iterations,
                    nudge_iters=cfg.point_cloud_nudge_iters,
                    ray_dirs=cfg.point_cloud_ray_dirs,
                    cull_ratio=cfg.point_cloud_cull_ratio,
                    propose_geom=ctx.propose_geom,
                ),
            )
    if (
        _proposer_enabled("guidance_walk", ctx.enabled_proposers)
        and cfg.use_guidance_walk
    ):
        gw_cap = state.explorer_cap(pool)
        if gw_cap > 0:
            state.ext(
                "guidance_walk",
                propose_placements_guidance_walk(
                    ctx.base_shape,
                    ctx.shape_to_place,
                    ctx.sheet,
                    ctx.pt_push,
                    ctx.propose_geom,
                    min_dist=ctx.min_dist,
                    top_n=gw_cap,
                ),
            )
    if (
        _proposer_enabled("ribbon_free", ctx.enabled_proposers)
        and cfg.use_ribbon_seeds
        and use_free_region
    ):
        rb_cap = state.explorer_cap(pool)
        if rb_cap > 0:
            state.ext(
                "ribbon_free",
                propose_placements_ribbon_free(
                    ctx.base_shape,
                    ctx.shape_to_place,
                    ctx.sheet,
                    ctx.min_dist,
                    num_angles=n_angles,
                    top_n=rb_cap,
                    propose_geom=ctx.propose_geom,
                    pt_push=ctx.pt_push,
                ),
            )


def _collect_cast_refine_candidates(
    ctx: ProposeContext,
    state: _CollectState,
) -> None:
    """Final stage: guidance cast refine on clustered seeds from the pool."""
    cfg = ctx.propose_cfg
    if not (
        _proposer_enabled("guidance_cast_refine", ctx.enabled_proposers)
        and cfg.use_guidance_propositions
        and cfg.guidance_cast_refine_top_k > 0
    ):
        return
    seed_limit = cfg.guidance_cast_refine_top_k
    clustered = cluster_seed_coords(list(state.candidates))
    structured = select_guidance_cast_seeds(
        clustered,
        seed_limit,
        ctx.shape_to_place,
        ctx.propose_geom,
        ctx.pt_push,
        ctx.min_dist,
        ctx.focal_shape,
    )
    if not structured and ctx.guidance_seed_coords:
        structured = list(ctx.guidance_seed_coords)[:seed_limit]
    if not structured:
        return
    state.ext(
        "guidance_cast_refine",
        propose_placements_guidance_cast(
            structured,
            ctx.pt_push,
            ctx.propose_geom,
            cfg,
            min_dist=ctx.min_dist,
            top_n=cfg.candidate_pool,
        ),
    )


def _collect_expand_candidates(
    ctx: ProposeContext,
    extras: PackedProposeExtras,
    state: _CollectState,
    *,
    group_id: int = 0,
) -> None:
    """Emit selection_expand (and optional history_expand) after cast refine."""
    cfg = ctx.propose_cfg
    rng = np.random.default_rng(abs(hash((group_id, round(ctx.min_dist, 6)))) % (2**31))
    seeds: list[tuple[float, float, float]] = []
    if extras.packed_transforms is not None and extras.packed_group_ids is not None:
        for gid, tr in zip(extras.packed_group_ids, extras.packed_transforms, strict=False):
            if int(gid) != int(group_id):
                continue
            t = tuple(float(x) for x in tr[:3])
            seeds.append(t)
    if (
        seeds
        and _proposer_enabled("selection_expand", ctx.enabled_proposers)
    ):
        # Light emit — mixer still adds a thin expand remainder.
        n = 1
        state.ext(
            "selection_expand",
            propose_placements_selection_expand(seeds, n=n, rng=rng),
            max_items=max(8, cfg.candidate_pool // 4),
        )
    if (
        seeds
        and _proposer_enabled("history_expand", ctx.enabled_proposers)
        and bool(getattr(cfg, "use_history_expand", True))
    ):
        state.ext(
            "history_expand",
            propose_placements_history_expand(seeds, n=1, rng=rng),
            max_items=max(4, cfg.candidate_pool // 8),
        )


def _apply_cascade_budget(
    ctx: ProposeContext,
    extras: PackedProposeExtras,
    state: _CollectState,
    *,
    cascade_zone: str | None,
    pocket_reserve: Sequence[tuple[float, float, float]],
    motif_reserve: Sequence[tuple[float, float, float]],
) -> None:
    """Lean Void Cascade: sniper short-circuit / explorer budget scale."""
    cfg = ctx.propose_cfg
    if not _cascade_active(cfg, cascade_zone):
        return
    hard_zone = bool(cascade_zone) and cascade_zone in _CASCADE_HARD_ZONES
    if hard_zone:
        sniper_seeds: list[tuple[float, float, float]] = []
        sniper_seeds.extend(pocket_reserve)
        sniper_seeds.extend(motif_reserve)
        if ctx.guidance_seed_coords:
            sniper_seeds.extend(list(ctx.guidance_seed_coords))
        stop_n = max(int(getattr(cfg, "cascade_sniper_stop_n", 4) or 4), 1)
        n_valid = _count_fast_valid_seeds(
            sniper_seeds,
            ctx.propose_geom,
            limit=stop_n,
            pt_push=ctx.pt_push,
            crash_counter=state.cascade_stats_out,
        )
        min_motif_pocket = int(getattr(cfg, "cascade_min_motif_pocket_emit", 0) or 0)
        n_motif_pocket = len(pocket_reserve) + len(motif_reserve)
        if n_valid >= stop_n and n_motif_pocket >= min_motif_pocket:
            state.skip_builders = True
            state.skip_explorers = True
            state.mark_skip("snipers", sorted(_CASCADE_BUILDERS | _CASCADE_EXPLORERS))
        return
    if cascade_zone not in ("empty_border", None, ""):
        scale = float(getattr(cfg, "cascade_explorer_budget_scale", 0.35) or 0.35)
        state.explorer_scale = min(max(scale, 0.05), 1.0)


def _collect_candidates(
    ctx: ProposeContext,
    extras: PackedProposeExtras,
    *,
    mode: str | None = None,
    group_id: int = 0,
    guidance_reserve_only: bool = False,
    pocket_reserve_out: list | None = None,
    motif_reserve_out: list | None = None,
    proposer_keys: dict[str, set[tuple[float, float, float]]] | None = None,
    cascade_zone: str | None = None,
    cascade_stats_out: dict | None = None,
) -> List[Tuple[float, float, float]]:
    """Run the propose stages in AGENTS emit order and return valid candidates.

    ``mode`` selects the stage order (statically, no registry):
    ``"cascade"`` = snipers → builders → explorers (void / short-circuit zones),
    ``"free"`` = snipers → explorers → builders (default packed / border walk).
    Defaults to whichever the cascade config implies for ``cascade_zone``.
    """
    cascade_on = _cascade_active(ctx.propose_cfg, cascade_zone)
    if mode is None:
        mode = "cascade" if cascade_on else "free"
    state = _CollectState(
        proposer_counts=ctx.proposer_counts,
        proposer_keys=proposer_keys,
        cascade_stats_out=cascade_stats_out,
    )

    pocket_reserve, motif_reserve = _collect_pocket_candidates(
        ctx,
        extras,
        state,
        group_id=group_id,
        guidance_reserve_only=guidance_reserve_only,
    )
    if pocket_reserve_out is not None:
        pocket_reserve_out.extend(pocket_reserve)
    if motif_reserve_out is not None:
        motif_reserve_out.extend(motif_reserve)

    _apply_cascade_budget(
        ctx,
        extras,
        state,
        cascade_zone=cascade_zone,
        pocket_reserve=pocket_reserve,
        motif_reserve=motif_reserve,
    )

    # Void / mid-pack: claim board_edge rim slots before explorers / side_pack
    # flood claimed_keys (mode=free runs explorers first).
    if cascade_zone == "void_seek":
        if _emit_board_edge_reserve(ctx, extras, state, reserve_only=True):
            state.board_edge_reserved = True

    # Wall-fill: void_seek sniper short-circuit still emits side_pack.
    if state.skip_builders and cascade_zone == "void_seek":
        n0 = len(state.candidates)
        if bool(getattr(ctx.propose_cfg, "use_side_pack", True)):
            _emit_side_pack(ctx, extras, state, cascade_zone=cascade_zone)
        if (
            len(state.candidates) > n0
            and state.cascade_stats_out is not None
        ):
            skipped = list(
                state.cascade_stats_out.get("cascade_skipped_proposers") or []
            )
            keep = {"side_pack", "board_edge"}
            if keep.intersection(skipped):
                state.cascade_stats_out["cascade_skipped_proposers"] = [
                    n for n in skipped if n not in keep
                ]

    if mode == "cascade":
        _collect_builder_candidates(ctx, extras, state, cascade_zone=cascade_zone)
        # Builder-stage short-circuit: pool already full of snipers+builders.
        hard_zone = bool(cascade_zone) and cascade_zone in _CASCADE_HARD_ZONES
        if (
            cascade_on
            and hard_zone
            and not state.skip_explorers
            and len(state.candidates) >= max(int(ctx.propose_cfg.candidate_pool), 1)
        ):
            state.skip_explorers = True
            state.mark_skip("builders", sorted(_CASCADE_EXPLORERS))
        _collect_explorer_candidates(ctx, extras, state)
    else:
        _collect_explorer_candidates(ctx, extras, state)
        _collect_builder_candidates(ctx, extras, state, cascade_zone=cascade_zone)

    _collect_cast_refine_candidates(ctx, state)
    _collect_expand_candidates(ctx, extras, state, group_id=group_id)
    return _filter_distant_collisions(state.candidates, ctx.propose_geom)


def collect_propose_candidates(
    base_shape: BaseGeometry,
    shape_to_place: Polygon,
    sheet: Polygon,
    propose_cfg: ProposeConfig,
    *,
    min_dist: float,
    pt_push: Point,
    propose_geom: ProposeGeometry,
    focal_shape: Optional[BaseGeometry] = None,
    enabled_proposers: frozenset[str] | None = None,
    proposer_counts: dict[str, int] | None = None,
    guidance_seed_coords: Sequence[tuple[float, float, float]] | None = None,
    cluster_patterns: Sequence | None = None,
    group_id: int = 0,
    placement_angles_override: np.ndarray | None = None,
    border_focus: bool | None = None,
    packed_polys: Sequence[BaseGeometry] | None = None,
    packed_group_ids: Sequence[int] | None = None,
    packed_transforms: Sequence | None = None,
    pocket_reserve_out: list | None = None,
    pocket_tags_out: list[str] | None = None,
    pocket_attempts_out: list[int] | None = None,
    free_space=None,
    guidance_reserve_only: bool = False,
    motif_reserve_out: list | None = None,
    pocket_skip_out: list[str] | None = None,
    proposer_keys: dict[str, set[tuple[float, float, float]]] | None = None,
    cascade_zone: str | None = None,
    cascade_stats_out: dict | None = None,
    void_pole: Point | None = None,
) -> List[Tuple[float, float, float]]:
    """Flat entry point for ``_collect_candidates`` (builds ctx / extras)."""
    ctx = make_propose_context(
        base_shape=base_shape,
        shape_to_place=shape_to_place,
        sheet=sheet,
        propose_cfg=propose_cfg,
        min_dist=min_dist,
        pt_push=pt_push,
        propose_geom=propose_geom,
        focal_shape=focal_shape,
        enabled_proposers=enabled_proposers,
        proposer_counts=proposer_counts,
        guidance_seed_coords=guidance_seed_coords,
        placement_angles_override=placement_angles_override,
        border_focus=border_focus,
        void_pole=void_pole,
    )
    pocket_stats = PocketStats()
    extras = PackedProposeExtras(
        packed_polys=packed_polys,
        packed_group_ids=packed_group_ids,
        packed_transforms=packed_transforms,
        free_space=free_space,
        cluster_patterns=cluster_patterns,
        pocket_stats=pocket_stats,
        void_pole=void_pole,
    )
    out = _collect_candidates(
        ctx,
        extras,
        group_id=group_id,
        guidance_reserve_only=guidance_reserve_only,
        pocket_reserve_out=pocket_reserve_out,
        motif_reserve_out=motif_reserve_out,
        proposer_keys=proposer_keys,
        cascade_zone=cascade_zone,
        cascade_stats_out=cascade_stats_out,
    )
    if pocket_tags_out is not None:
        pocket_tags_out.extend(pocket_stats.tags)
    if pocket_attempts_out is not None and pocket_stats.attempts:
        pocket_attempts_out.append(int(pocket_stats.attempts))
    if pocket_skip_out is not None and pocket_stats.skip_reasons:
        pocket_skip_out.extend(pocket_stats.skip_reasons)
    return out


def _trim_candidates_by_border(
    candidates: Sequence[Tuple[float, float, float]],
    shape_to_place: Polygon,
    propose_geom: ProposeGeometry,
    pt_push: Point,
    min_dist: float,
    limit: int,
) -> List[Tuple[float, float, float]]:
    if limit <= 0 or not candidates:
        return []
    scored: list[tuple[float, Tuple[float, float, float]]] = []
    seen: set[tuple[float, float, float]] = set()
    for coords in candidates:
        key = (round(coords[0], 4), round(coords[1], 4), round(coords[2], 4))
        if key in seen:
            continue
        seen.add(key)
        score = _score_placement_border(coords, shape_to_place, propose_geom, pt_push, min_dist)
        if score > float("-inf"):
            scored.append((score, coords))
    scored.sort(key=lambda x: x[0], reverse=True)
    return [coords for _, coords in scored[:limit]]


def _merge_spaced_channel_seeds(
    ranked: Sequence[Tuple[float, float, float]],
    seeds: Sequence[Tuple[float, float, float]],
    *,
    max_n: int,
    min_spacing: float,
    allow_multi_theta: bool = False,
) -> List[Tuple[float, float, float]]:
    """Prefer spaced seeds in the final proposal set.

    When ``allow_multi_theta``, same-XY different-θ seeds are kept (pocket
    multi-angle inserts); XY spacing only applies across distinct cells.
    """
    if max_n <= 0:
        return []
    if not seeds:
        return list(ranked)[:max_n]
    kept: list[Tuple[float, float, float]] = []
    seen: set[tuple[float, float, float]] = set()

    def _xy_key(c: Tuple[float, float, float]) -> tuple[float, float]:
        return (round(c[0], 4), round(c[1], 4))

    def _try_add(coords: Tuple[float, float, float], *, seed: bool) -> bool:
        key = (round(coords[0], 4), round(coords[1], 4), round(coords[2], 4))
        if key in seen:
            return False
        for k in kept:
            if hypot(coords[0] - k[0], coords[1] - k[1]) >= min_spacing:
                continue
            if allow_multi_theta and seed and _xy_key(coords) == _xy_key(k):
                continue
            return False
        seen.add(key)
        kept.append(coords)
        return True

    for coords in seeds:
        if len(kept) >= max_n:
            break
        _try_add(coords, seed=True)
    for coords in ranked:
        if len(kept) >= max_n:
            break
        _try_add(coords, seed=False)
    if len(kept) < max_n:
        for coords in list(seeds) + list(ranked):
            if len(kept) >= max_n:
                break
            key = (round(coords[0], 4), round(coords[1], 4), round(coords[2], 4))
            if key in seen:
                continue
            seen.add(key)
            kept.append(coords)
    return kept[:max_n]


def propose_coords_from_candidates(
    base_shape: BaseGeometry,
    shape_to_place: Polygon,
    boundary: BaseGeometry,
    propose_cfg: ProposeConfig,
    *,
    min_dist: float,
    pt_push: Point,
    candidates: Sequence[Tuple[float, float, float]],
    rank_mode: str,
    focal_shape: Optional[BaseGeometry] = None,
    rules=None,
    group_id: int = 0,
    reserve_coords: Sequence[Tuple[float, float, float]] | None = None,
    allow_multi_theta_reserve: bool = False,
    void_pole: Point | None = None,
    diversity_stats_out: dict | None = None,
    propose_geom: ProposeGeometry | None = None,
    full_packed_geoms: list[Geometry] | None = None,
) -> List[Tuple[float, float, float]]:
    if propose_geom is not None:
        if full_packed_geoms is not None:
            raise AssertionError("Cannot pass packed geoms when reusing propose_geom")
        if propose_geom.part_poly is not shape_to_place:
            raise AssertionError("propose_geom part mismatch!")
        geom = propose_geom
    else:
        geom = ProposeGeometry(
            boundary,
            base_shape,
            shape_to_place,
            min_dist,
            epsilon_ratio=propose_cfg.placement_clearance_epsilon_ratio,
            propose_cfg=propose_cfg,
            full_packed_geoms=full_packed_geoms,
        )
    pool = list(candidates)
    if propose_cfg.trim_candidates_by_clearance:
        pool = pre_filter_candidates(
            pool,
            geom,
            pt_push,
            min_dist,
            propose_cfg.placement_clearance_epsilon_ratio,
        )
    if len(pool) > propose_cfg.candidate_pool:
        if rank_mode == "border" and propose_cfg.trim_candidates_by_clearance:
            pool = _trim_candidates_by_border(
                pool, shape_to_place, geom, pt_push, min_dist, propose_cfg.candidate_pool,
            )
        elif rank_mode in ("contact", "contact_hybrid", "rule_hybrid") and propose_cfg.trim_candidates_by_clearance:
            if propose_cfg.use_stratified_contact_trim and rank_mode in (
                "contact_hybrid", "rule_hybrid",
            ):
                pool = _trim_candidates_stratified(
                    pool,
                    shape_to_place,
                    geom,
                    pt_push,
                    min_dist,
                    propose_cfg.candidate_pool,
                    focal_shape,
                    contact_fraction=propose_cfg.contact_trim_fraction,
                    rank_mode=rank_mode,
                    clearance_weight=propose_cfg.contact_clearance_hybrid_weight,
                    propose_cfg=propose_cfg,
                    tightness_weight=propose_cfg.contact_tightness_hybrid_weight,
                    rules=rules,
                    group_id=group_id,
                    base_shape=base_shape,
                    void_pole=void_pole,
                )
            else:
                pool = _trim_candidates_by_contact(
                    pool,
                    shape_to_place,
                    geom,
                    pt_push,
                    min_dist,
                    propose_cfg.candidate_pool,
                    focal_shape,
                )
        elif (
            propose_cfg.trim_candidates_by_clearance
            and rank_mode in ("clearance", "hybrid")
        ):
            pool = _trim_candidates_by_clearance(
                pool, geom, pt_push, propose_cfg.candidate_pool,
            )
    ranked = _rank_proposal_coords(
        pool,
        base_shape,
        shape_to_place,
        boundary,
        pt_push,
        min_dist,
        propose_cfg.max_proposals,
        geom,
        propose_cfg,
        rank_mode=rank_mode,
        focal_shape=focal_shape,
        rules=rules,
        group_id=group_id,
        void_pole=void_pole,
    )
    squeezed = _apply_cast_squeeze_if_enabled(
        ranked,
        shape_to_place=shape_to_place,
        propose_geom=geom,
        propose_cfg=propose_cfg,
        pt_push=pt_push,
        min_dist=min_dist,
        rank_mode=rank_mode,
        focal_shape=focal_shape,
        rules=rules,
        group_id=group_id,
        base_shape=base_shape,
    )
    diversity_stats: dict = {}
    if getattr(propose_cfg, "use_conflict_degree_rank", False) and squeezed:
        n = len(squeezed)
        scored = [(float(n - i), c) for i, c in enumerate(squeezed)]
        rescored = apply_conflict_degree_penalty(
            scored,
            shape_to_place,
            lambda_pen=float(getattr(propose_cfg, "conflict_degree_lambda", 0.05) or 0.05),
            max_overlap=int(getattr(propose_cfg, "conflict_degree_max_overlap", 5) or 5),
            pad=float(min_dist),
            stats_out=diversity_stats,
        )
        squeezed = [c for _s, c in rescored]
    if getattr(propose_cfg, "use_pose_nms", False) and squeezed:
        eps = float(getattr(propose_cfg, "pose_nms_eps", 1.0) or 1.0)
        if eps <= 0.0:
            eps = max(float(min_dist), 1e-3)
        squeezed = apply_pose_nms(
            squeezed,
            eps=eps,
            theta_tol=float(getattr(propose_cfg, "pose_nms_theta_tol", 0.15) or 0.15),
            stats_out=diversity_stats,
        )
    if diversity_stats_out is not None and diversity_stats:
        for k, v in diversity_stats.items():
            if isinstance(v, (int, float)):
                diversity_stats_out[k] = diversity_stats_out.get(k, 0) + v
            else:
                diversity_stats_out[k] = v
    filtered_reserve: list[Tuple[float, float, float]] = []
    if reserve_coords:
        for coords in reserve_coords:
            c = (float(coords[0]), float(coords[1]), float(coords[2]))
            if not geom.valid_at(c, pt_push):
                continue
            if geom.full_packed_geoms and not geom.passes_full_packed_collision(
                geom.placed_at(c)
            ):
                continue
            filtered_reserve.append(c)
    if filtered_reserve:
        minx, miny, maxx, maxy = shape_to_place.bounds
        spacing = max(min_dist * 2.0, 0.9 * max(maxx - minx, maxy - miny, 1e-3))
        return _merge_spaced_channel_seeds(
            squeezed,
            filtered_reserve,
            max_n=propose_cfg.max_proposals,
            min_spacing=spacing,
            allow_multi_theta=allow_multi_theta_reserve,
        )
    return squeezed


def _apply_cast_squeeze_if_enabled(
    ranked: List[Tuple[float, float, float]],
    *,
    shape_to_place: Polygon,
    propose_geom: ProposeGeometry,
    propose_cfg: ProposeConfig,
    pt_push: Point,
    min_dist: float,
    rank_mode: str,
    focal_shape: Optional[BaseGeometry],
    rules=None,
    group_id: int = 0,
    base_shape: BaseGeometry | None = None,
) -> List[Tuple[float, float, float]]:
    if propose_cfg.cast_squeeze_top_k <= 0 or rank_mode not in (
        "contact", "contact_hybrid", "border", "rule_hybrid",
    ):
        return ranked
    out = ranked
    passes = max(propose_cfg.cast_squeeze_passes, 1)
    for _ in range(passes):
        out = cast_squeeze_ranked_coords(
            out,
            shape_to_place,
            propose_geom,
            propose_cfg,
            pt_push,
            min_dist,
            focal_shape=focal_shape,
            rank_mode=rank_mode,
            rules=rules,
            group_id=group_id,
            base_shape=base_shape,
        )
    return out


def propose_coords_with_strategy(
    base_shape: BaseGeometry,
    shape_to_place: Polygon,
    boundary: BaseGeometry,
    propose_cfg: ProposeConfig,
    *,
    min_dist: float,
    pt_push: Point,
    focal_shape: Optional[BaseGeometry] = None,
    enabled_proposers: frozenset[str] | None = None,
    rules=None,
    group_id: int = 0,
    proposer_counts: dict[str, int] | None = None,
    full_packed_geoms: list[Geometry] | None = None,
    cluster_patterns: Sequence | None = None,
    placement_angles_override: np.ndarray | None = None,
    guidance_seed_coords: Sequence[tuple[float, float, float]] | None = None,
    packed_polys: Sequence[BaseGeometry] | None = None,
    packed_group_ids: Sequence[int] | None = None,
    packed_transforms: Sequence | None = None,
    pocket_stats_out: dict | None = None,
    free_space=None,
    guidance_reserve_only: bool = False,
    void_pole: Point | None = None,
    proposer_keys_out: dict[str, set[tuple[float, float, float]]] | None = None,
    cascade_zone: str | None = None,
    cascade_stats_out: dict | None = None,
    diversity_stats_out: dict | None = None,
) -> List[Tuple[float, float, float]]:
    cfg = propose_cfg
    if cfg.use_guidance_walk and should_use_border_focus(base_shape, cfg):
        cfg = cfg.model_copy(update={"use_guidance_walk": False})
    rank_mode = effective_ranking_mode(cfg, base_shape, rules=rules)
    border_focus = should_use_border_focus(base_shape, cfg)
    if focal_shape is None:
        if border_focus:
            focal_shape = border_focal_for_propose(boundary, min_dist)
        elif base_shape is not None and not base_shape.is_empty:
            focal_shape = base_shape
    if border_focus:
        pt_push = propose_push_point(
            boundary,
            base_shape,
            smart_push=cfg.smart_push_target,
            min_dist=min_dist,
            use_border_focus=True,
        )
    sheet, _voids = board_context_from_geometry(boundary)
    geom = ProposeGeometry(
        boundary,
        base_shape,
        shape_to_place,
        min_dist,
        epsilon_ratio=cfg.placement_clearance_epsilon_ratio,
        propose_cfg=cfg,
        full_packed_geoms=full_packed_geoms,
        border_focus=border_focus,
    )
    pocket_reserve: list[tuple[float, float, float]] = []
    motif_reserve: list[tuple[float, float, float]] = []
    local_proposer_keys: dict[str, set[tuple[float, float, float]]] = {}
    ctx = make_propose_context(
        base_shape=base_shape,
        shape_to_place=shape_to_place,
        sheet=sheet,
        propose_cfg=cfg,
        min_dist=min_dist,
        pt_push=pt_push,
        propose_geom=geom,
        focal_shape=focal_shape,
        enabled_proposers=enabled_proposers,
        proposer_counts=proposer_counts,
        guidance_seed_coords=guidance_seed_coords,
        placement_angles_override=placement_angles_override,
        border_focus=border_focus,
        void_pole=void_pole,
    )
    pocket_stats = PocketStats()
    extras = PackedProposeExtras(
        packed_polys=packed_polys,
        packed_group_ids=packed_group_ids,
        packed_transforms=packed_transforms,
        free_space=free_space,
        cluster_patterns=cluster_patterns,
        pocket_stats=pocket_stats,
        void_pole=void_pole,
    )
    candidates = _collect_candidates(
        ctx,
        extras,
        group_id=group_id,
        guidance_reserve_only=guidance_reserve_only,
        pocket_reserve_out=pocket_reserve,
        motif_reserve_out=motif_reserve if cfg.unified_void_reserve else None,
        proposer_keys=local_proposer_keys,
        cascade_zone=cascade_zone,
        cascade_stats_out=cascade_stats_out,
    )
    pocket_tags = list(pocket_stats.tags)
    pocket_attempts = [pocket_stats.attempts] if pocket_stats.attempts else []
    pocket_skips = list(pocket_stats.skip_reasons)
    # Keep corridor / pocket seeds by skipping clearance pool trim; force hybrid
    # when pocket teleports are present so clearance ranking cannot discard them.
    if guidance_seed_coords or pocket_reserve:
        hybrid_update = {
            "trim_candidates_by_clearance": False,
        }
        if rank_mode == RankingMode.CLEARANCE:
            rank_mode = RankingMode.CONTACT_HYBRID
            hybrid_update.update({
                "use_contact_ranking": True,
                "use_contact_clearance_hybrid": True,
                "ranking_mode": RankingMode.CONTACT_HYBRID,
            })
        cfg = cfg.model_copy(update=hybrid_update)

    # Merge Mode A / corridor poles with pocket_fit (+ motif) into a shared reserved budget.
    max_n = int(cfg.max_proposals)
    reserve_frac = float(cfg.pocket_fit_reserve_fraction)
    r_budget = max(1, int(math.ceil(reserve_frac * max_n))) if reserve_frac > 0 else 0
    poles = list(guidance_seed_coords) if guidance_seed_coords else []
    merged_reserve: list[tuple[float, float, float]] = []
    if cfg.unified_void_reserve and r_budget > 0:
        streams = [s for s in (poles, pocket_reserve, motif_reserve) if s]
        if streams:
            per = max(1, r_budget // len(streams))
            remain = r_budget
            for i, stream in enumerate(streams):
                cap = remain if i == len(streams) - 1 else min(per, remain)
                merged_reserve.extend(stream[:cap])
                remain -= min(cap, len(stream[:cap]))
                if remain <= 0:
                    break
    elif poles and pocket_reserve and r_budget > 0:
        pole_cap = r_budget // 2
        pocket_cap = r_budget - pole_cap
        merged_reserve.extend(poles[:pole_cap])
        merged_reserve.extend(pocket_reserve[:pocket_cap])
    elif pocket_reserve and r_budget > 0:
        merged_reserve.extend(pocket_reserve[:r_budget])
    elif poles:
        # Poles-only: keep existing guidance behavior (up to max_n).
        merged_reserve.extend(poles[:max_n])

    result = propose_coords_from_candidates(
        base_shape,
        shape_to_place,
        boundary,
        cfg,
        min_dist=min_dist,
        pt_push=pt_push,
        candidates=candidates,
        rank_mode=rank_mode,
        focal_shape=focal_shape,
        rules=rules,
        group_id=group_id,
        reserve_coords=merged_reserve if merged_reserve else None,
        allow_multi_theta_reserve=bool(pocket_reserve),
        void_pole=void_pole,
        diversity_stats_out=diversity_stats_out,
        propose_geom=geom,
    )
    final_keys = {_proposal_key(c) for c in result}
    pool_by_proposer: dict[str, int] = {}
    for name, keys in local_proposer_keys.items():
        pool_by_proposer[name] = sum(1 for k in keys if k in final_keys)
    if proposer_keys_out is not None:
        proposer_keys_out.clear()
        for name, keys in local_proposer_keys.items():
            proposer_keys_out[name] = set(keys)
    if pocket_stats_out is not None:
        selected = 0
        by_tag: dict[str, int] = {}
        pocket_keys: list[tuple[float, float, float]] = []
        motif_hole_keys: list[tuple[float, float, float]] = []
        for coords, tag in zip(pocket_reserve, pocket_tags):
            key = _proposal_key(coords)
            if str(tag) == TAG_MOTIF_HOLE:
                motif_hole_keys.append(key)
            else:
                pocket_keys.append(key)
            if key in final_keys:
                selected += 1
                by_tag[tag] = by_tag.get(tag, 0) + 1
        emitted = len(pocket_reserve)
        attempted = int(pocket_attempts[0]) if pocket_attempts else emitted
        pocket_stats_out["emitted"] = emitted
        pocket_stats_out["attempted"] = attempted
        pocket_stats_out["selected"] = selected
        pocket_stats_out["valid_rate"] = (
            float(emitted) / float(attempted) if attempted else 0.0
        )
        # Legacy alias used by some callers / void_leak.
        pocket_stats_out["survival"] = pocket_stats_out["valid_rate"]
        pocket_stats_out["by_tag"] = by_tag
        pocket_stats_out["tags"] = list(pocket_tags)
        pocket_stats_out["pocket_keys"] = pocket_keys
        pocket_stats_out["motif_hole_keys"] = motif_hole_keys
        pocket_stats_out["pocket_skip"] = list(dict.fromkeys(pocket_skips))
        motif_skip = {
            str(k): int(v)
            for k, v in pocket_stats.skip_reasons.items()
            if str(k).startswith("motif_")
        }
        pocket_stats_out["motif_skip"] = motif_skip
        pocket_stats_out["motif_cohorts"] = list(pocket_stats.motif_cohorts)
        pocket_stats_out["pool_by_proposer"] = pool_by_proposer
        pocket_stats_out["emitted_by_proposer"] = {
            n: len(ks) for n, ks in local_proposer_keys.items()
        }
    return result


_FIRST_PASS_PACKED_BORDER_PROPOSERS = first_pass_packed_border_proposers()


def _batch_follow_proposer_coords(
    board: BaseGeometry,
    obstacle_shape: BaseGeometry,
    follow_poly: Polygon,
    propose_cfg: ProposeConfig,
    *,
    min_dist: float,
    pt_push: Point,
    focal: BaseGeometry | None,
    sheet: Polygon,
) -> list[tuple[float, float, float]]:
    rank_mode = effective_ranking_mode(propose_cfg, obstacle_shape)
    geom = ProposeGeometry(
        board,
        obstacle_shape,
        follow_poly,
        min_dist,
        epsilon_ratio=propose_cfg.placement_clearance_epsilon_ratio,
        propose_cfg=propose_cfg,
    )
    candidates = collect_propose_candidates(
        obstacle_shape,
        follow_poly,
        sheet,
        propose_cfg,
        min_dist=min_dist,
        pt_push=pt_push,
        propose_geom=geom,
        focal_shape=focal,
        enabled_proposers=BATCH_FOLLOW_PROPOSERS,
    )
    return propose_coords_from_candidates(
        obstacle_shape,
        follow_poly,
        board,
        propose_cfg,
        min_dist=min_dist,
        pt_push=pt_push,
        candidates=candidates,
        rank_mode=rank_mode,
        focal_shape=focal,
        propose_geom=geom,
    )


def _batch_parts_by_group(
    parts: Sequence[tuple[Polygon, int]],
) -> dict[int, Polygon]:
    out: dict[int, Polygon] = {}
    for poly, gid in parts:
        if gid not in out:
            out[gid] = poly
    return out


def _batch_top_seed_coords(
    arr: np.ndarray,
    n: int,
) -> list[tuple[float, float, float]]:
    if arr.shape[0] == 0 or n <= 0:
        return []
    take = min(n, arr.shape[0])
    return [tuple(float(x) for x in row) for row in arr[:take]]


def _as_batch_geometry(g) -> Geometry | None:
    if g is None:
        return None
    if isinstance(g, Geometry):
        return g
    if hasattr(g, "is_empty") and g.is_empty:
        return None
    return Geometry.from_shapely(g)


def _batch_obstacle_geoms(placed: Sequence[BaseGeometry]) -> list[Geometry]:
    out: list[Geometry] = []
    for p in placed:
        g = _as_batch_geometry(p)
        if g is not None:
            out.append(g)
    return out


def _batch_pair_valid(
    voids: list[Geometry],
    part_a: Geometry,
    coords_a: tuple[float, float, float],
    part_b: Geometry,
    coords_b: tuple[float, float, float],
    obstacle_geoms: list[Geometry],
    min_dist: float,
) -> bool:
    placed_a = part_a.apply_transform(coords_a)
    placed_b = part_b.apply_transform(coords_b)
    if not is_pose_clear(placed_a, voids, obstacle_geoms, min_dist):
        return False
    return is_pose_clear(
        placed_b, voids, [*obstacle_geoms, placed_a], min_dist,
    )


def _batch_pair_contact_score(
    placed_a: Geometry,
    placed_b: Geometry,
    sheet: Polygon,
    min_dist: float,
    focal: BaseGeometry | Geometry | None,
) -> float:
    err_a = placement_contact_error(placed_a, sheet, min_dist, focal)
    err_b = placement_contact_error(placed_b, sheet, min_dist, placed_a)
    inter = abs(float(placed_a.distance(placed_b)) - min_dist)
    return err_a + err_b + inter


def _batch_pack_pair_order(
    board: BaseGeometry,
    anchor_gid: int,
    follow_gid: int,
    parts_by_group: dict[int, Polygon],
    base_obstacle: BaseGeometry,
    per_group: dict[int, np.ndarray],
    propose_cfg: ProposeConfig,
    *,
    min_dist: float,
    placed: Sequence[BaseGeometry],
    obstacle_geoms: list[Geometry] | None = None,
) -> list[tuple[tuple[float, float, float], tuple[float, float, float], float]]:
    anchor_poly = parts_by_group[anchor_gid]
    follow_poly = parts_by_group[follow_gid]
    anchor_seeds = _batch_top_seed_coords(
        per_group.get(anchor_gid, np.zeros((0, 3))),
        propose_cfg.batch_pack_anchor_seeds,
    )
    if not anchor_seeds:
        return []

    follow_cfg = propose_cfg.model_copy(
        update={
            "max_proposals": propose_cfg.batch_pack_follow_proposals,
            "candidate_pool": min(
                propose_cfg.candidate_pool,
                propose_cfg.batch_pack_follow_pool,
            ),
        },
    )
    sheet, voids = board_context_from_geometry(board)
    obs = (
        list(obstacle_geoms)
        if obstacle_geoms is not None
        else _batch_obstacle_geoms(placed)
    )
    anchor_part = Geometry.from_shapely(anchor_poly)
    follow_part = Geometry.from_shapely(follow_poly)
    pairs: list[tuple[tuple[float, float, float], tuple[float, float, float], float]] = []

    for coords_a in anchor_seeds:
        placed_a_g = anchor_part.apply_transform(coords_a)
        if not is_pose_clear(placed_a_g, voids, obs, min_dist):
            continue

        # Shapely mirror only for follow proposers / free-space topology.
        placed_a = transform_poly(anchor_poly, coords_a)
        extended = (
            unary_union([base_obstacle, placed_a])
            if not base_obstacle.is_empty
            else placed_a
        )
        placed_with_anchor = list(placed) + [placed_a]
        focal = focal_shape_for_propose(
            board, placed_with_anchor, follow_poly, min_dist, propose_cfg,
        )
        border_focus = should_use_border_focus(extended, propose_cfg)
        push = propose_push_point(
            board,
            extended,
            smart_push=propose_cfg.smart_push_target,
            min_dist=min_dist,
            use_border_focus=border_focus,
        )
        follow_coords = _batch_follow_proposer_coords(
            board,
            extended,
            follow_poly,
            follow_cfg,
            min_dist=min_dist,
            pt_push=push,
            focal=focal,
            sheet=sheet,
        )
        if not follow_coords:
            continue

        focal_g = _as_batch_geometry(focal) if focal is not None else None
        for coords_b in follow_coords[: propose_cfg.batch_pack_follow_proposals]:
            if not _batch_pair_valid(
                voids, anchor_part, coords_a, follow_part, coords_b,
                obs, min_dist,
            ):
                continue
            placed_b_g = follow_part.apply_transform(coords_b)
            score = _batch_pair_contact_score(
                placed_a_g, placed_b_g, sheet, min_dist, focal_g,
            )
            pairs.append((coords_a, coords_b, score))

    pairs.sort(key=lambda p: p[2])
    return pairs[: propose_cfg.batch_pack_max_pairs]


def augment_batch_pack_proposals(
    board: BaseGeometry,
    parts: Sequence[tuple[Polygon, int]],
    placed: Sequence[BaseGeometry],
    per_group: dict[int, np.ndarray],
    propose_cfg: ProposeConfig,
    *,
    min_dist: float,
    pairs_out: list | None = None,
) -> dict[int, list[tuple[float, float, float]]]:
    """Pack groups sequentially; return extra (x, y, angle) seeds per group."""
    if not propose_cfg.use_batch_pack:
        return {}

    parts_by_group = _batch_parts_by_group(parts)
    group_ids = sorted(parts_by_group)
    if len(group_ids) < 2:
        return {}

    # Shapely union only for follow ProposeGeometry / free-space; clearance uses Geometry list.
    base_obstacle = unary_union(placed) if placed else Polygon()
    obstacle_geoms = _batch_obstacle_geoms(placed)
    all_pairs: list[
        tuple[tuple[float, float, float], tuple[float, float, float], float, int, int]
    ] = []

    orders: list[tuple[int, int]] = []
    if len(group_ids) == 2:
        orders = [(group_ids[0], group_ids[1]), (group_ids[1], group_ids[0])]
    else:
        # Limit the number of pairs to avoid combinatorial explosion
        # Just pair each group with the next one or two
        for i, anchor in enumerate(group_ids):
            for j in range(1, min(3, len(group_ids))):
                follow = group_ids[(i + j) % len(group_ids)]
                orders.append((anchor, follow))

    seen_orders: set[tuple[int, int]] = set()
    for anchor_gid, follow_gid in orders:
        if (anchor_gid, follow_gid) in seen_orders:
            continue
        seen_orders.add((anchor_gid, follow_gid))
        for coords_a, coords_b, score in _batch_pack_pair_order(
            board,
            anchor_gid,
            follow_gid,
            parts_by_group,
            base_obstacle,
            per_group,
            propose_cfg,
            min_dist=min_dist,
            placed=placed,
            obstacle_geoms=obstacle_geoms,
        ):
            all_pairs.append((coords_a, coords_b, score, anchor_gid, follow_gid))

    all_pairs.sort(key=lambda p: p[2])
    all_pairs = all_pairs[: propose_cfg.batch_pack_max_pairs]

    extra: dict[int, list[tuple[float, float, float]]] = {gid: [] for gid in group_ids}
    per_gid_cap = max(1, propose_cfg.batch_pack_max_pairs // len(group_ids))
    gid_counts = {gid: 0 for gid in group_ids}

    if pairs_out is not None:
        for coords_a, coords_b, _score, anchor_gid, follow_gid in all_pairs:
            pairs_out.append((
                tuple(float(x) for x in coords_a),
                tuple(float(x) for x in coords_b),
                int(anchor_gid),
                int(follow_gid),
            ))

    for coords_a, coords_b, _score, anchor_gid, follow_gid in all_pairs:
        if gid_counts[anchor_gid] < per_gid_cap:
            extra[anchor_gid].append(coords_a)
            gid_counts[anchor_gid] += 1
        if gid_counts[follow_gid] < per_gid_cap:
            extra[follow_gid].append(coords_b)
            gid_counts[follow_gid] += 1

    for gid in group_ids:
        if extra[gid]:
            extra[gid] = list(
                map(
                    tuple,
                    dedupe_transforms(np.asarray(extra[gid], dtype=np.float64)),
                ),
            )
    return extra


def border_edge_transforms_for_group(
    board: BaseGeometry,
    part_poly: Polygon,
    base_shape: BaseGeometry,
    propose_cfg: ProposeConfig,
    *,
    min_dist: float,
    pt_push: Point | None = None,
) -> np.ndarray:
    """Edge-tight (x, y, θ) for graph batch pinning when the sheet is mostly empty."""
    if not propose_cfg.use_board_edge_seeds:
        return np.zeros((0, 3), dtype=np.float64)

    sheet, _ = board_context_from_geometry(board)
    push = pt_push if pt_push is not None else propose_push_point(
        board,
        base_shape,
        smart_push=propose_cfg.smart_push_target,
        min_dist=min_dist,
        use_border_focus=True,
    )
    geom = ProposeGeometry(
        board,
        base_shape,
        part_poly,
        min_dist,
        epsilon_ratio=propose_cfg.placement_clearance_epsilon_ratio,
        propose_cfg=propose_cfg,
    )
    reserve = max(propose_cfg.board_edge_batch_reserve, propose_cfg.max_proposals // 2)
    coords = propose_placements_board_edge(
        part_poly,
        sheet,
        base_shape,
        min_dist=min_dist,
        propose_cfg=propose_cfg,
        propose_geom=geom,
        pt_push=push,
        top_n=reserve,
    )
    return propositions_to_ndarray(coords)


def collect_propose_batch_for_nest(
    board: BaseGeometry,
    parts: Sequence[Tuple[Polygon, int]],
    polys: Sequence[BaseGeometry],
    selected: Sequence[int],
    propose_cfg: ProposeConfig,
    *,
    min_dist: float,
    border_only: bool = False,
    use_full_packed_obstacle: bool = False,
    rules=None,
    proposer_counts_out: dict[str, int] | None = None,
    propose_feedback=None,
    packed_group_ids: Sequence[int] | None = None,
    packed_transforms: Sequence | None = None,
    group_allowed_angles: Sequence[tuple[float, ...] | None] | None = None,
    user_holes: tuple = (),
    seeded: bool = False,
    full_packed_geoms: Sequence[Geometry] | None = None,
    archived_patterns: Sequence | None = None,
) -> tuple[dict[int, np.ndarray], dict]:
    """Propose slice of nest transform-batch (proposals + densify/pocket stats).

    Angle projection / sampling mix stay in ``build_graph._build_transform_batch``.
    """
    zones_used: list[str] = []
    pocket_keys_raw: dict[int, set[tuple[float, float, float]]] = {}
    densify_stats: dict = {}
    propose_by_group = proposed_transforms_for_groups(
        board,
        parts,
        polys,
        selected,
        propose_cfg,
        min_dist=min_dist,
        border_only_propose=border_only,
        use_full_packed_obstacle=use_full_packed_obstacle,
        rules=rules,
        proposer_counts_out=proposer_counts_out,
        zones_used_out=zones_used,
        propose_feedback=propose_feedback,
        packed_group_ids=packed_group_ids,
        packed_transforms=packed_transforms,
        group_allowed_angles=group_allowed_angles,
        user_holes=user_holes,
        seeded=seeded,
        pocket_keys_out=pocket_keys_raw,
        densify_stats_out=densify_stats,
        full_packed_geoms=full_packed_geoms,
        archived_patterns=archived_patterns,
    )
    stats = {
        "proposal_count": sum(arr.shape[0] for arr in propose_by_group.values()),
        "zones_used": zones_used,
        "pocket_keys_raw": pocket_keys_raw,
        "densify_stats": dict(densify_stats),
        "proposed_by_group": {
            gid: np.asarray(arr, dtype=np.float64)
            for gid, arr in propose_by_group.items()
        },
        "border_only": bool(border_only),
        "proposer_keys": densify_stats.get("proposer_keys") or {},
        "emitted_by_proposer": densify_stats.get("emitted_by_proposer") or {},
        "pool_by_proposer": densify_stats.get("pool_by_proposer") or {},
        "pocket_by_tag": densify_stats.get("pocket_by_tag") or {},
        "cascade_stopped_after": densify_stats.get("cascade_stopped_after", "none"),
        "cascade_skipped_proposers": list(
            densify_stats.get("cascade_skipped_proposers") or []
        ),
        "nms_kept": int(densify_stats.get("nms_kept", 0)),
        "nms_dropped": int(densify_stats.get("nms_dropped", 0)),
        "conflict_penalty_applied": int(
            densify_stats.get("conflict_penalty_applied", 0)
        ),
        "batch_pack_pairs": list(densify_stats.get("batch_pack_pairs") or []),
        "sniper_keys": dict(densify_stats.get("sniper_keys") or {}),
    }
    return propose_by_group, stats


def _prepare_group_propose(
    board: BaseGeometry,
    parts: Sequence[Tuple[Polygon, int]],
    selected_polys: Sequence[BaseGeometry],
    selected_indices: Sequence[int],
    propose_cfg: ProposeConfig,
    *,
    min_dist: float,
    user_holes: tuple[tuple[tuple[float, float], ...], ...] = (),
    seeded: bool = False,
    packed_group_ids: Sequence[int] | None = None,
    packed_transforms: Sequence | None = None,
    full_packed_geoms: Sequence[Geometry] | None = None,
    archived_patterns: Sequence | None = None,
) -> tuple:
    """Shared pre-loop setup for ``proposed_transforms_for_groups`` (behavior-neutral)."""
    sheet, _ = board_context_from_geometry(board, user_holes=user_holes)
    placed = [selected_polys[i] for i in selected_indices]
    n_holes = len(getattr(sheet, "interiors", ()) or ())
    sheet_vertices = len(list(sheet.exterior.coords)) if hasattr(sheet, "exterior") else 0
    max_verts = 0
    max_interiors = 0
    concave_parts = False
    for part_poly, _gid in parts:
        max_verts = max(max_verts, len(list(part_poly.exterior.coords)))
        max_interiors = max(max_interiors, len(getattr(part_poly, "interiors", ()) or ()))
        if part_is_concave(part_poly):
            concave_parts = True
    propose_cfg = propose_cfg.with_complexity_lean(
        n_holes=n_holes,
        max_part_vertices=max_verts,
        max_part_interiors=max_interiors,
        sheet_vertices=sheet_vertices,
        concave_parts=concave_parts,
        seeded=seeded,
    )
    if full_packed_geoms is not None and len(full_packed_geoms) == len(placed):
        full_packed_geoms = list(full_packed_geoms)
    else:
        full_packed_geoms = [
            p if isinstance(p, Geometry) else Geometry.from_shapely(p)
            for p in placed
        ]
    cluster_patterns: list = []
    packed_gids_aligned: list[int] | None = None
    packed_trs_aligned: list | None = None
    if packed_group_ids is not None and packed_transforms is not None and placed:
        packed_gids_aligned = [
            int(packed_group_ids[i]) for i in selected_indices if i < len(packed_group_ids)
        ]
        packed_trs_aligned = [
            packed_transforms[i]
            for i in selected_indices
            if i < len(packed_transforms)
        ]
        if len(packed_gids_aligned) != len(placed) or len(packed_trs_aligned) != len(placed):
            packed_gids_aligned = None
            packed_trs_aligned = None
    if (
        propose_cfg.use_cluster_copy
        and packed_gids_aligned is not None
        and packed_trs_aligned is not None
    ):
        cluster_patterns = extract_cluster_patterns(
            placed,
            packed_gids_aligned,
            packed_trs_aligned,
            min_dist=min_dist,
            max_patterns=propose_cfg.cluster_copy_max_patterns,
            min_members=propose_cfg.cluster_copy_min_members,
            sheet=sheet,
            min_compactness=float(
                getattr(propose_cfg, "motif_min_compactness", 0.0) or 0.0
            ),
        )
    archived: list = []
    if (
        bool(getattr(propose_cfg, "enable_accepted_pattern_archive", True))
        and bool(getattr(propose_cfg, "use_cluster_copy", True))
        and archived_patterns
    ):
        archived = list(archived_patterns)
    synth: list = []
    if (
        bool(getattr(propose_cfg, "enable_mate_synth", True))
        and bool(getattr(propose_cfg, "use_cluster_copy", True))
    ):
        synth = synthesize_mate_patterns(
            parts,
            min_dist=min_dist,
            max_patterns=int(propose_cfg.cluster_copy_max_patterns),
        )
    if archived or synth:
        cluster_patterns = merge_cluster_patterns(
            cluster_patterns,
            synth,
            max_patterns=int(propose_cfg.cluster_copy_max_patterns),
            archived=archived,
        )
    void_thr = float(propose_cfg.late_border_void_override_ratio)
    if void_thr <= 0.0:
        void_thr = 2.5
    free_snap = None
    if placed:
        mean_area = float(np.mean([float(p.area) for p, _ in parts])) if parts else 1.0
        free_snap = build_free_space_snapshot(
            sheet,
            placed,
            mean_area,
            min_dist,
            void_ratio_threshold=void_thr,
            pack_geoms=full_packed_geoms,
        )
    return (
        sheet,
        placed,
        propose_cfg,
        full_packed_geoms,
        cluster_patterns,
        packed_gids_aligned,
        packed_trs_aligned,
        void_thr,
        free_snap,
    )


def proposed_transforms_for_groups(
    board: BaseGeometry,
    parts: Sequence[Tuple[Polygon, int]],
    selected_polys: Sequence[BaseGeometry],
    selected_indices: Sequence[int],
    propose_cfg: ProposeConfig,
    *,
    min_dist: float,
    pt_push: Optional[Point] = None,
    border_only_propose: bool = False,
    use_full_packed_obstacle: bool = False,
    rules=None,
    proposer_counts_out: dict[str, int] | None = None,
    zones_used_out: list[str] | None = None,
    propose_feedback=None,
    packed_group_ids: Sequence[int] | None = None,
    packed_transforms: Sequence | None = None,
    group_allowed_angles: Sequence[tuple[float, ...] | None] | None = None,
    user_holes: tuple[tuple[tuple[float, float], ...], ...] = (),
    seeded: bool = False,
    pocket_keys_out: dict[int, set[tuple[float, float, float]]] | None = None,
    densify_stats_out: dict | None = None,
    full_packed_geoms: Sequence[Geometry] | None = None,
    archived_patterns: Sequence | None = None,
) -> dict[int, np.ndarray]:
    """Propose (x, y, angle) seeds per part group.

    Propose uses the nearest packed cluster as obstacles only.
    ``make_polygon_graph`` still filters against the full selection.

    Pass ``full_packed_geoms`` aligned with ``selected_indices`` (e.g. from
    ``NestState.native_geoms``) to skip per-call ``from_shapely`` of the pack.
    """
    (
        sheet,
        placed,
        propose_cfg,
        full_packed_geoms,
        cluster_patterns,
        packed_gids_aligned,
        packed_trs_aligned,
        void_thr,
        free_snap,
    ) = _prepare_group_propose(
        board,
        parts,
        selected_polys,
        selected_indices,
        propose_cfg,
        min_dist=min_dist,
        user_holes=user_holes,
        seeded=seeded,
        packed_group_ids=packed_group_ids,
        packed_transforms=packed_transforms,
        full_packed_geoms=full_packed_geoms,
        archived_patterns=archived_patterns,
    )
    densify_stats_patterns = len(cluster_patterns)
    out: dict[int, np.ndarray] = {}
    total_counts: dict[str, int] = {}
    pocket_emitted = 0
    pocket_attempted = 0
    pocket_accepted = 0
    pocket_keys_by_group: dict[int, set[tuple[float, float, float]]] = {}
    motif_keys_by_group: dict[int, set[tuple[float, float, float]]] = {}
    motif_skip_agg: dict[str, int] = {}
    motif_cohorts_agg: list = []
    densify_fired = 0
    densify_accepted = 0
    densify_reasons: list[str] = []
    pocket_skips_all: list[str] = []
    cascade_agg: dict = {
        "cascade_stopped_after": "none",
        "cascade_skipped_proposers": [],
    }
    diversity_agg: dict = {}
    proposer_keys_agg: dict[str, set[tuple[float, float, float]]] = {}
    sniper_keys_by_group: dict[int, set[tuple[float, float, float]]] = {}
    emitted_by_proposer: dict[str, int] = {}
    pool_by_proposer: dict[str, int] = {}
    pocket_by_tag: dict[str, int] = {}
    for part_idx, (part_poly, group_id) in enumerate(parts):
        zone: str | None = None
        zone_info = None
        cfg = propose_cfg
        primary_target = None
        void_hijack_from: str | None = None
        free_analysis = None
        if propose_cfg.place_profiles_enabled and not border_only_propose:
            if not placed:
                if sheet_has_narrow_corridor(sheet, part_poly, min_dist):
                    zone = "cluster_edge"
                    channel_pt = corridor_channel_target(sheet, min_dist)
                    zone_info = PlaceZoneInfo(
                        zone="cluster_edge",
                        free_ratio=1.0,
                        is_corridor=True,
                        primary_target=channel_pt,
                    )
                    primary_target = channel_pt
                else:
                    zone = "empty_border"
                    zone_info = PlaceZoneInfo(zone="empty_border")
            else:
                # Classify against the full pack so free_ratio / targets stay honest;
                # nearest-k local obstacles are applied only when proposing.
                obs_preview = simplify_obstacle_union(placed, min_dist)
                zone_info = classify_propose_zone_info(
                    board,
                    obs_preview,
                    part_poly,
                    min_dist=min_dist,
                    propose_cfg=propose_cfg,
                    selected_polys=placed,
                    user_holes=user_holes,
                    sheet=sheet,
                )
                zone = zone_info.zone
                primary_target = zone_info.primary_target
                # Mode A: large contiguous void → force void_seek + pole seeds.
                free_analysis = analyze_free_space(
                    sheet,
                    placed,
                    float(part_poly.area),
                    min_dist,
                    void_ratio_threshold=void_thr,
                )
                if (
                    getattr(propose_cfg, "enable_void_large_hijack", True)
                    and free_analysis.kind == "large_void"
                    and free_analysis.target_pt is not None
                    and not (zone_info.is_corridor or zone_info.is_annulus)
                ):
                    void_hijack_from = zone
                    zone = "void_seek"
                    primary_target = free_analysis.target_pt
                    zone_info = PlaceZoneInfo(
                        zone="void_seek",
                        free_ratio=zone_info.free_ratio,
                        n_clusters=zone_info.n_clusters,
                        outline_coverage=zone_info.outline_coverage,
                        primary_target=primary_target,
                        is_annulus=False,
                        is_corridor=False,
                    )
            cfg = ProposeConfig.for_place(zone, base=propose_cfg)
            cfg = apply_proposer_pool_scales(cfg, propose_cfg.place_proposer_pool_scales)
            if zone == "void_seek":
                cfg = floor_void_seek_budgets(cfg)
            if (
                zone == "border_gap"
                and zone_info is not None
                and zone_info.is_annulus
            ):
                # Annulus rim: keep border docking but shoot inward toward the hole.
                cfg = cfg.model_copy(update={
                    "ranking_mode": RankingMode.CONTACT_HYBRID,
                    "use_contact_ranking": True,
                    "use_contact_clearance_hybrid": True,
                    "use_full_packed_obstacle": True,
                    "border_focus_ranking": False,
                })
            if zone_info is not None and zone_info.is_corridor:
                # for_place won't re-enable flags the base left False; force tube docking.
                # Drop perimeter/border bias — those pin proposals to the channel mouth.
                cfg = cfg.model_copy(update={
                    "use_neighbor_slide": True,
                    "use_ribbon_seeds": True,
                    "use_full_packed_obstacle": True,
                    "use_contact_ranking": True,
                    "use_contact_clearance_hybrid": True,
                    "use_border_focus": False,
                    "border_focus_ranking": False,
                    "ranking_mode": RankingMode.CONTACT_HYBRID,
                    "smart_push_target": True,
                    "use_cluster_copy": True,
                })
                channel_mid = (
                    zone_info.primary_target
                    or corridor_channel_target(sheet, min_dist)
                )
                channel_samples = corridor_channel_samples(sheet, min_dist, n=12)
                primary_target = nearest_channel_attract(
                    channel_samples,
                    channel_mid=channel_mid,
                    packed=placed,
                )
                if primary_target is not None:
                    zone_info = PlaceZoneInfo(
                        zone=zone_info.zone,
                        free_ratio=zone_info.free_ratio,
                        n_clusters=zone_info.n_clusters,
                        outline_coverage=zone_info.outline_coverage,
                        primary_target=primary_target,
                        is_annulus=zone_info.is_annulus,
                        is_corridor=True,
                    )

        corridor_guidance_seeds: list[tuple[float, float, float]] | None = None
        if zone_info is not None and zone_info.is_corridor:
            samples = corridor_channel_samples(sheet, min_dist, n=12)
            corridor_guidance_seeds = corridor_seed_coords_from_samples(
                samples,
                num_angles=min(4, int(cfg.placement_num_angles) if cfg.placement_num_angles else 4),
            )
        elif (
            zone == "void_seek"
            and primary_target is not None
            and not border_only_propose
        ):
            corridor_guidance_seeds = void_pole_seed_coords(
                primary_target,
                num_angles=min(4, int(cfg.placement_num_angles) if cfg.placement_num_angles else 4),
            )

        use_full = use_full_packed_obstacle
        if zone is not None:
            n_clusters = zone_info.n_clusters if zone_info is not None else 1
            free_ratio = zone_info.free_ratio if zone_info is not None else 1.0
            coverage = zone_info.outline_coverage if zone_info is not None else 0.0
            use_full, nearest_k = ProposeConfig.obstacle_scope_for_place(
                zone,
                n_clusters=n_clusters,
                outline_coverage=coverage,
                border_coverage_threshold=propose_cfg.place_border_coverage_threshold,
            )
            if nearest_k > 0:
                cfg = cfg.model_copy(update={"obstacle_nearest_k": nearest_k})
            if zone_info is not None and (zone_info.is_annulus or zone_info.is_corridor):
                use_full = True

        if propose_feedback is not None and propose_feedback.last_proposal_yield < 0.4:
            if placed:
                bumped_k = min(cfg.obstacle_nearest_k + 1, len(placed))
                cfg = cfg.model_copy(update={"obstacle_nearest_k": bumped_k})
            if zone == "inter_cluster":
                use_full = True

        if use_full and placed:
            obstacle_shape = simplify_obstacle_union(placed, min_dist)
        else:
            obstacle_shape = obstacle_shape_for_propose(
                placed,
                part_poly,
                min_dist,
                propose_cfg=cfg,
                sheet=sheet,
                ref_point=primary_target,
            )
        # Reuse propose obstacle as focal when full-pack (avoids nearest-k mismatch).
        if use_full and placed:
            focal = obstacle_shape
            if zone == "interior_pocket" and primary_target is not None:
                pocket_proposers = ProposeConfig.proposers_for_place(zone) or frozenset()
                if "group_fit" in pocket_proposers:
                    _, part_max = part_extents(part_poly)
                    local_focal = local_packed_near_target(
                        placed, primary_target, part_max,
                    )
                    if local_focal is not None and not local_focal.is_empty:
                        focal = local_focal
        else:
            focal = focal_shape_for_propose(
                board,
                placed,
                part_poly,
                min_dist,
                cfg,
                sheet=sheet,
                ref_point=primary_target,
            )
        border_focus = should_use_border_focus(obstacle_shape, cfg)
        push = pt_push if pt_push is not None else propose_push_point(
            board,
            obstacle_shape,
            smart_push=cfg.smart_push_target,
            min_dist=min_dist,
            use_border_focus=border_focus,
        )
        enabled = None
        if border_only_propose:
            if should_use_border_focus(obstacle_shape, cfg):
                enabled = FIRST_PASS_EMPTY_BORDER_PROPOSERS
            elif placed:
                enabled = _FIRST_PASS_PACKED_BORDER_PROPOSERS
        elif zone is not None:
            if zone_info is not None and zone_info.is_corridor:
                # Channel shooters only — perimeter_walk parks at the exterior mouth.
                enabled = CORRIDOR_PROPOSERS
            else:
                annulus = bool(zone_info is not None and zone_info.is_annulus)
                enabled = ProposeConfig.proposers_for_place(zone, annulus=annulus)

        if zone_info is not None and zone_info.is_corridor and primary_target is not None:
            push = primary_target
        elif zone == "void_seek" and primary_target is not None:
            # OOS-4: void_pole is not the packed-centroid push; attract to polylabel.
            push = primary_target
        void_pole = (
            primary_target
            if zone == "void_seek" and primary_target is not None
            else None
        )

        angle_override = None
        if group_allowed_angles is not None:
            # Index by group_id (not part list position) so grain stays correct.
            if group_id < len(group_allowed_angles):
                allowed = group_allowed_angles[group_id]
                if allowed is not None:
                    angle_override = np.asarray(allowed, dtype=np.float64)
        part_free_space = free_snap
        # On Mode A hijack, prefer per-part free analysis (target_poly) over mean-area snap.
        if (
            void_hijack_from is not None
            and free_analysis is not None
            and free_snap is not None
        ):
            part_free_space = replace(free_snap, analysis=free_analysis)
        elif void_hijack_from is not None and free_analysis is not None:
            part_free_space = FreeSpaceSnapshot(
                analysis=free_analysis,
                trapped_voids=(),
                hull_bays=(),
                topology_poles=(),
            )
        # Skip pocket when part dwarfs the largest void (still allow other proposers).
        if (
            free_snap is not None
            and free_snap.max_void_area > 0.0
            and float(part_poly.area)
            > float(propose_cfg.pocket_fit_area_ratio) * free_snap.max_void_area * 2.0
            and enabled is not None
            and ProposerName.POCKET_FIT in enabled
        ):
            enabled = frozenset(n for n in enabled if n != ProposerName.POCKET_FIT)
        group_counts: dict[str, int] = {}
        pocket_stats: dict = {}
        group_proposer_keys: dict[str, set[tuple[float, float, float]]] = {}
        group_cascade: dict = {}
        group_diversity: dict = {}
        poles_reserve_only = (
            void_hijack_from is not None
            and bool(propose_cfg.poles_reserve_only_on_hijack)
        )
        coords = propose_coords_with_strategy(
            obstacle_shape,
            part_poly,
            board,
            cfg,
            min_dist=min_dist,
            pt_push=push,
            focal_shape=focal,
            enabled_proposers=enabled,
            rules=rules,
            group_id=group_id,
            proposer_counts=group_counts,
            full_packed_geoms=full_packed_geoms,
            cluster_patterns=cluster_patterns,
            placement_angles_override=angle_override,
            guidance_seed_coords=corridor_guidance_seeds,
            packed_polys=placed,
            packed_group_ids=packed_gids_aligned,
            packed_transforms=packed_trs_aligned,
            pocket_stats_out=pocket_stats,
            free_space=part_free_space,
            guidance_reserve_only=poles_reserve_only,
            void_pole=void_pole,
            proposer_keys_out=group_proposer_keys,
            cascade_zone=zone,
            cascade_stats_out=group_cascade,
            diversity_stats_out=group_diversity,
        )
        pocket_emitted += int(pocket_stats.get("emitted", 0))
        pocket_attempted += int(pocket_stats.get("attempted", 0))
        pocket_accepted += int(pocket_stats.get("selected", 0))
        for sk in pocket_stats.get("pocket_skip") or []:
            pocket_skips_all.append(str(sk))
        for mk, mv in (pocket_stats.get("motif_skip") or {}).items():
            motif_skip_agg[str(mk)] = motif_skip_agg.get(str(mk), 0) + int(mv)
        for cohort in pocket_stats.get("motif_cohorts") or []:
            motif_cohorts_agg.append(cohort)
        for pk in pocket_stats.get("pocket_keys") or []:
            pocket_keys_by_group.setdefault(int(group_id), set()).add(tuple(pk))
        for hk in pocket_stats.get("motif_hole_keys") or []:
            motif_keys_by_group.setdefault(int(group_id), set()).add(tuple(hk))
        for key in group_proposer_keys.get("cluster_copy") or ():
            motif_keys_by_group.setdefault(int(group_id), set()).add(tuple(key))
        for name, n in group_counts.items():
            total_counts[name] = total_counts.get(name, 0) + n
        for name, keys in group_proposer_keys.items():
            proposer_keys_agg.setdefault(name, set()).update(keys)
        sniper_keys_by_group.setdefault(int(group_id), set()).update(
            group_proposer_keys.get("cluster_copy") or ()
        )
        sniper_keys_by_group.setdefault(int(group_id), set()).update(
            group_proposer_keys.get("pocket_fit") or ()
        )
        if group_cascade.get("cascade_stopped_after") not in (None, "none"):
            cascade_agg["cascade_stopped_after"] = group_cascade.get(
                "cascade_stopped_after"
            )
            skipped = list(cascade_agg.get("cascade_skipped_proposers") or [])
            for n in group_cascade.get("cascade_skipped_proposers") or []:
                if n not in skipped:
                    skipped.append(n)
            cascade_agg["cascade_skipped_proposers"] = skipped
        for k, v in group_diversity.items():
            if isinstance(v, (int, float)):
                diversity_agg[k] = diversity_agg.get(k, 0) + v
            else:
                diversity_agg[k] = v
        for name, n in (pocket_stats.get("emitted_by_proposer") or {}).items():
            emitted_by_proposer[name] = emitted_by_proposer.get(name, 0) + int(n)
        for name, n in (pocket_stats.get("pool_by_proposer") or {}).items():
            pool_by_proposer[name] = pool_by_proposer.get(name, 0) + int(n)
        for tag, n in (pocket_stats.get("by_tag") or {}).items():
            pocket_by_tag[tag] = pocket_by_tag.get(tag, 0) + int(n)
        arr = propositions_to_ndarray(coords)
        enabled_names = (
            _normalize_proposers(enabled) if enabled is not None else None
        )
        board_edge_in_collect = (
            (enabled_names is None or "board_edge" in enabled_names)
            and cfg.use_board_edge_seeds
            and (border_focus or cfg.board_edge_when_packed)
        )
        if border_focus and cfg.use_board_edge_seeds and not board_edge_in_collect:
            edge_arr = border_edge_transforms_for_group(
                board,
                part_poly,
                obstacle_shape,
                cfg,
                min_dist=min_dist,
                pt_push=push,
            )
            if edge_arr.shape[0] > 0:
                arr = dedupe_transforms(
                    np.concatenate([edge_arr, arr], axis=0),
                )[: cfg.max_proposals]

        zone_label = zone if zone is not None else "default"
        if void_hijack_from is not None:
            zone_label = f"{void_hijack_from}→void_seek(large_void)"
        free_ratio = zone_info.free_ratio if zone_info is not None else 1.0
        sterile_zones = ("cluster_edge", "border_gap", "interior_pocket")
        densify_count_floor = max(4, int(cfg.max_proposals) // 3)
        floor_ratio = float(getattr(propose_cfg, "densify_clearance_floor_ratio", 1.0))
        group_pocket_emitted = int(pocket_stats.get("emitted", 0))
        arr, zone_label, fired, accepted, densify_reason, densify_telem = (
            _void_seek_densify(
                arr=arr,
                zone=zone,
                zone_label=zone_label,
                board=board,
                sheet=sheet,
                part_poly=part_poly,
                placed=placed,
                obstacle_shape=obstacle_shape,
                propose_cfg=propose_cfg,
                cfg=cfg,
                min_dist=min_dist,
                primary_target=primary_target,
                void_hijack_from=void_hijack_from,
                part_free_space=part_free_space,
                full_packed_geoms=full_packed_geoms,
                cluster_patterns=cluster_patterns,
                angle_override=angle_override,
                corridor_guidance_seeds=corridor_guidance_seeds,
                packed_gids_aligned=packed_gids_aligned,
                packed_trs_aligned=packed_trs_aligned,
                poles_reserve_only=poles_reserve_only,
                rules=rules,
                group_id=group_id,
                densify_count_floor=densify_count_floor,
                floor_ratio=floor_ratio,
                group_pocket_emitted=group_pocket_emitted,
                free_ratio=free_ratio,
                sterile_zones=sterile_zones,
                border_only_propose=border_only_propose,
                push=push,
            )
        )
        if fired:
            densify_fired += 1
            if accepted:
                densify_accepted += 1
            if densify_reason is not None:
                densify_reasons.append(densify_reason)
            for name, n in densify_telem["total_counts"].items():
                total_counts[name] = total_counts.get(name, 0) + int(n)
            for name, keys in densify_telem["proposer_keys"].items():
                proposer_keys_agg.setdefault(name, set()).update(keys)
            sniper_keys_by_group.setdefault(int(group_id), set()).update(
                densify_telem["proposer_keys"].get("cluster_copy") or ()
            )
            sniper_keys_by_group.setdefault(int(group_id), set()).update(
                densify_telem["proposer_keys"].get("pocket_fit") or ()
            )
            for name, n in densify_telem["emitted_by_proposer"].items():
                emitted_by_proposer[name] = max(
                    int(emitted_by_proposer.get(name, 0)), int(n),
                )
            for name, n in densify_telem["pool_by_proposer"].items():
                pool_by_proposer[name] = max(
                    int(pool_by_proposer.get(name, 0)), int(n),
                )
            for tag, n in densify_telem["pocket_by_tag"].items():
                pocket_by_tag[tag] = pocket_by_tag.get(tag, 0) + int(n)
            pocket_emitted += int(densify_telem["pocket_emitted"])
            pocket_attempted += int(densify_telem["pocket_attempted"])
            pocket_accepted += int(densify_telem["pocket_accepted"])
            for pk in densify_telem["pocket_keys"]:
                pocket_keys_by_group.setdefault(int(group_id), set()).add(tuple(pk))
            for hk in densify_telem.get("motif_hole_keys") or []:
                motif_keys_by_group.setdefault(int(group_id), set()).add(tuple(hk))
            for key in densify_telem.get("proposer_keys", {}).get("cluster_copy") or ():
                motif_keys_by_group.setdefault(int(group_id), set()).add(tuple(key))
            pocket_skips_all.extend(densify_telem["pocket_skips"])
            for mk, mv in (densify_telem.get("motif_skip") or {}).items():
                motif_skip_agg[str(mk)] = motif_skip_agg.get(str(mk), 0) + int(mv)
            for cohort in densify_telem.get("motif_cohorts") or []:
                motif_cohorts_agg.append(cohort)
        if (
            zones_used_out is not None
            and propose_cfg.place_profiles_enabled
            and not border_only_propose
        ):
            zones_used_out.append(zone_label)

        out[group_id] = arr

    if proposer_counts_out is not None:
        proposer_counts_out.clear()
        proposer_counts_out.update(total_counts)
        # Meta keys for telemetry only (not used as pool-scale proposer names).
        proposer_counts_out["_pocket_fit_emitted"] = pocket_emitted
        proposer_counts_out["_pocket_fit_attempted"] = pocket_attempted
        proposer_counts_out["_pocket_fit_selected"] = pocket_accepted
        if pocket_attempted > 0:
            proposer_counts_out["_pocket_fit_survival_pct"] = int(
                round(100.0 * pocket_emitted / pocket_attempted)
            )
        proposer_counts_out["_densify_fired"] = densify_fired
        proposer_counts_out["_densify_accepted"] = densify_accepted
    if pocket_keys_out is not None:
        pocket_keys_out.clear()
        for gid, keys in pocket_keys_by_group.items():
            pocket_keys_out[gid] = set(keys)
    if densify_stats_out is not None:
        densify_stats_out.clear()
        densify_stats_out["fired"] = densify_fired
        densify_stats_out["accepted"] = densify_accepted
        densify_stats_out["densify_reason"] = (
            densify_reasons[-1] if densify_reasons else None
        )
        densify_stats_out["densify_reasons"] = list(densify_reasons)
        densify_stats_out["pocket_skip"] = list(dict.fromkeys(pocket_skips_all))
        densify_stats_out["proposer_keys"] = {
            name: set(keys) for name, keys in proposer_keys_agg.items()
        }
        densify_stats_out["motif_keys"] = {
            gid: set(keys) for gid, keys in motif_keys_by_group.items()
        }
        densify_stats_out["motif_skip"] = dict(motif_skip_agg)
        densify_stats_out["motif_cohorts"] = list(motif_cohorts_agg)
        densify_stats_out["emitted_by_proposer"] = dict(emitted_by_proposer)
        densify_stats_out["pool_by_proposer"] = dict(pool_by_proposer)
        densify_stats_out["pocket_by_tag"] = dict(pocket_by_tag)
        densify_stats_out["cascade_stopped_after"] = cascade_agg.get(
            "cascade_stopped_after", "none"
        )
        densify_stats_out["cascade_skipped_proposers"] = list(
            cascade_agg.get("cascade_skipped_proposers") or []
        )
        densify_stats_out["nms_kept"] = int(diversity_agg.get("nms_kept", 0))
        densify_stats_out["nms_dropped"] = int(diversity_agg.get("nms_dropped", 0))
        densify_stats_out["conflict_penalty_applied"] = int(
            diversity_agg.get("conflict_penalty_applied", 0)
        )
        densify_stats_out["cluster_patterns"] = int(densify_stats_patterns)
        densify_stats_out["sniper_keys"] = {
            gid: set(keys) for gid, keys in sniper_keys_by_group.items()
        }
        densify_stats_out["batch_pack_pairs"] = []

    # Batch-pack re-runs full proposers per anchor; only useful on empty / near-empty sheets.
    if propose_cfg.use_batch_pack and len(out) >= 2 and not placed:
        batch_pairs: list = []
        batch_extra = augment_batch_pack_proposals(
            board,
            parts,
            placed,
            out,
            propose_cfg,
            min_dist=min_dist,
            pairs_out=batch_pairs,
        )
        for gid, extra in batch_extra.items():
            if not extra:
                continue
            merged = np.concatenate(
                [out.get(gid, np.zeros((0, 3))), propositions_to_ndarray(extra)],
            )
            out[gid] = dedupe_transforms(merged)
        if densify_stats_out is not None:
            densify_stats_out["batch_pack_pairs"] = list(batch_pairs)
            pk = densify_stats_out.setdefault("proposer_keys", {})
            bp_keys = set(pk.get("batch_pack") or ())
            for rec in batch_pairs:
                bp_keys.add(transform_row_key(rec[0]))
                bp_keys.add(transform_row_key(rec[1]))
            pk["batch_pack"] = bp_keys
            proposer_keys_agg.setdefault("batch_pack", set()).update(bp_keys)

    return out
