from typing import List, Optional, Sequence, Tuple, Union
import math
from dataclasses import replace
from math import hypot

import numpy as np
from shapely import Point, Polygon
from shapely.geometry.base import BaseGeometry
from shapely.ops import unary_union

from nest_graph.board import board_context_from_geometry
from nest_graph.config import ProposeConfig, dedupe_transforms
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
from nest_graph.propose.placements_edge import (
    propose_placements_board_edge,
    propose_placements_group_fit,
    propose_placements_ribbon_free,
    propose_placements_sheet_corners,
    propose_placements_sheet_edge,
)
from nest_graph.propose.placements_geo import propose_placements_raycasting, propose_placements_voronoi
from nest_graph.propose.placements_guidance import (
    propose_placements_guidance_cast,
    propose_placements_guidance_walk,
)
from nest_graph.propose.placements_primary import (
    propose_placements_erosion,
    propose_placements_neighbor_slide,
    propose_placements_perimeter_walk,
)
from nest_graph.propose.placement_common import cluster_seed_coords
from nest_graph.propose.query_context import ProposeContext
from nest_graph.propose.placements_pattern import (
    extract_cluster_patterns,
    propose_placements_cluster_copy,
)
from nest_graph.propose.placements_pocket import (
    propose_placements_pocket_fit,
)
from nest_graph.propose.placements_pso import propose_placements_point_cloud
from nest_graph.propose.ranking import (
    _rank_proposal_coords,
    _score_placement_border,
    _trim_candidates_by_clearance,
    _trim_candidates_by_contact,
    _trim_candidates_stratified,
    cast_squeeze_ranked_coords,
    select_guidance_cast_seeds,
)

def propositions_to_ndarray(coords_list: Sequence[Tuple[float, float, float]]) -> np.ndarray:
    if not coords_list:
        return np.zeros((0, 3), dtype=np.float64)
    return np.asarray(coords_list, dtype=np.float64)


def _count_transforms_in_void(
    arr: np.ndarray,
    target_poly: Polygon | None,
    part_poly: Polygon,
) -> int:
    """Count SE(2) rows whose placed centroid lies in ``target_poly``."""
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
    return name in enabled_proposers


def _proposal_key(coords: tuple[float, float, float] | Sequence[float]) -> tuple[float, float, float]:
    """Match build_graph._transform_row_key (round 4) for nest/refine joins."""
    return (round(float(coords[0]), 4), round(float(coords[1]), 4), round(float(coords[2]), 4))


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
    n = len(items)
    if proposer_counts is not None:
        proposer_counts[name] = proposer_counts.get(name, 0) + n
    if proposer_keys is not None:
        bucket = proposer_keys.setdefault(name, set())
        for item in items:
            key = _proposal_key(item)
            if claimed_keys is not None:
                if key in claimed_keys:
                    continue
                claimed_keys.add(key)
            bucket.add(key)
    candidates.extend(items)


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
) -> List[Tuple[float, float, float]]:
    ctx = ProposeContext(
        base_shape,
        shape_to_place,
        sheet,
        propose_cfg,
        min_dist=min_dist,
        pt_push=pt_push,
        propose_geom=propose_geom,
        focal_shape=focal_shape,
        enabled_proposers=enabled_proposers,
        proposer_counts=proposer_counts,
        guidance_seed_coords=guidance_seed_coords,
        placement_angles_override=placement_angles_override,
    )
    pool = ctx.pool
    border_focus = ctx.border_focus
    use_free_region = ctx.use_free_region
    n_angles = ctx.n_angles
    placement_angles = ctx.placement_angles
    candidates: List[Tuple[float, float, float]] = []
    packed_list = list(packed_polys) if packed_polys is not None else []
    claimed_keys: set[tuple[float, float, float]] | None = (
        set() if proposer_keys is not None else None
    )

    def _ext(
        name: str,
        new_items: Sequence[tuple[float, float, float]],
        *,
        max_items: int | None = None,
    ) -> None:
        _extend_counted(
            candidates,
            proposer_counts,
            name,
            new_items,
            max_items=max_items,
            proposer_keys=proposer_keys,
            claimed_keys=claimed_keys,
        )

    if guidance_seed_coords and not guidance_reserve_only:
        _ext(
            "corridor_channel",
            list(guidance_seed_coords),
            max_items=len(guidance_seed_coords),
        )

    # Pocket teleports early so they sit in the candidate pool before slides.
    if (
        _proposer_enabled("pocket_fit", enabled_proposers)
        and propose_cfg.use_pocket_fit
        and packed_list
    ):
        allowed = None
        if placement_angles_override is not None and len(placement_angles_override) > 0:
            allowed = [float(a) for a in np.asarray(placement_angles_override).reshape(-1)]
        tags: list[str] = []
        attempts: list[int] = []
        skips: list[str] = []
        pocket_coords = propose_placements_pocket_fit(
            shape_to_place,
            sheet,
            packed_list,
            min_dist=min_dist,
            propose_geom=propose_geom,
            pt_push=pt_push,
            propose_cfg=propose_cfg,
            group_id=group_id,
            allowed_angles=allowed,
            cluster_patterns=cluster_patterns,
            packed_group_ids=packed_group_ids,
            packed_transforms=packed_transforms,
            top_n=max(pool, 8),
            tags_out=tags,
            attempts_out=attempts,
            free_space=free_space,
            skip_reasons_out=skips,
        )
        _ext(
            "pocket_fit",
            pocket_coords,
            max_items=len(pocket_coords),
        )
        if pocket_reserve_out is not None:
            pocket_reserve_out.extend(pocket_coords)
        if pocket_tags_out is not None:
            pocket_tags_out.extend(tags)
        if pocket_attempts_out is not None and attempts:
            pocket_attempts_out.append(int(attempts[0]))
        if pocket_skip_out is not None and skips:
            pocket_skip_out.extend(skips)
        # XY+θ keys already in candidates — cluster_copy will skip duplicates via seen.

    if (
        _proposer_enabled("cluster_copy", enabled_proposers)
        and propose_cfg.use_cluster_copy
        and cluster_patterns
    ):
        motif_coords = propose_placements_cluster_copy(
            cluster_patterns,
            group_id,
            shape_to_place,
            sheet,
            base_shape,
            min_dist=min_dist,
            propose_geom=propose_geom,
            pt_push=pt_push,
            propose_cfg=propose_cfg,
            top_n=pool,
            free_space=free_space,
        )
        _ext(
            "cluster_copy",
            motif_coords,
        )
        if motif_reserve_out is not None:
            motif_reserve_out.extend(motif_coords)

    if _proposer_enabled("perimeter_walk", enabled_proposers):
        _ext(
            "perimeter_walk",
            propose_placements_perimeter_walk(
                base_shape,
                shape_to_place,
                sheet,
                min_dist,
                propose_geom=propose_geom,
                pt_push=pt_push,
                use_free_region=use_free_region,
                border_focus=border_focus,
                num_angles=n_angles,
                top_n=pool * 2,
                placement_angles=placement_angles,
            ),
            max_items=pool * 2,
        )
    if (
        _proposer_enabled("neighbor_slide", enabled_proposers)
        and propose_cfg.use_neighbor_slide
        and not base_shape.is_empty
    ):
        neighbor_top = max(
            int(pool * propose_cfg.neighbor_slide_pool_fraction),
            n_angles,
        )
        _ext(
            "neighbor_slide",
            propose_placements_neighbor_slide(
                base_shape,
                shape_to_place,
                sheet,
                min_dist,
                propose_geom=propose_geom,
                pt_push=pt_push,
                num_angles=n_angles,
                top_n=neighbor_top,
                placement_angles=placement_angles,
            ),
            max_items=neighbor_top,
        )
    if _proposer_enabled("erosion", enabled_proposers):
        _ext(
            "erosion",
            propose_placements_erosion(
                base_shape,
                shape_to_place,
                sheet,
                min_dist,
                propose_geom=propose_geom,
                pt_push=pt_push,
                use_free_region=use_free_region,
                border_focus=border_focus,
                focal_shape=focal_shape,
                num_angles=n_angles,
                top_n=pool,
                placement_angles=placement_angles,
            ),
            max_items=pool,
        )
    if _proposer_enabled("raycasting", enabled_proposers):
        _ext(
            "raycasting",
            propose_placements_raycasting(
                base_shape,
                shape_to_place,
                sheet,
                min_dist,
                use_free_region=use_free_region,
                top_n=pool,
                num_rays=propose_cfg.raycast_num_rays,
                num_angles=propose_cfg.raycast_num_angles,
                anchor_stride=propose_cfg.raycast_anchor_stride,
                focal_shape=focal_shape,
                border_focus=border_focus,
                propose_geom=propose_geom,
                pt_push=pt_push,
            ),
        )
    if _proposer_enabled("voronoi", enabled_proposers) and propose_cfg.use_voronoi:
        _ext(
            "voronoi",
            propose_placements_voronoi(
                base_shape,
                shape_to_place,
                sheet,
                min_dist,
                use_free_region=use_free_region,
                top_n=pool,
                num_angles=propose_cfg.voronoi_num_angles,
                densify_divisor=propose_cfg.voronoi_densify_divisor,
                max_sites=propose_cfg.voronoi_max_sites,
                focal_shape=focal_shape,
                border_focus=border_focus,
                propose_geom=propose_geom,
                pt_push=pt_push,
            ),
        )
    if _proposer_enabled("point_cloud", enabled_proposers) and propose_cfg.use_point_cloud:
        _ext(
            "point_cloud",
            propose_placements_point_cloud(
                base_shape,
                shape_to_place,
                sheet,
                pt_push=pt_push,
                min_dist=min_dist,
                top_n=pool,
                num_particles=propose_cfg.point_cloud_particles,
                max_iterations=propose_cfg.point_cloud_iterations,
                nudge_iters=propose_cfg.point_cloud_nudge_iters,
                ray_dirs=propose_cfg.point_cloud_ray_dirs,
                cull_ratio=propose_cfg.point_cloud_cull_ratio,
                propose_geom=propose_geom,
            ),
        )
    if _proposer_enabled("guidance_walk", enabled_proposers) and propose_cfg.use_guidance_walk:
        _ext(
            "guidance_walk",
            propose_placements_guidance_walk(
                base_shape,
                shape_to_place,
                sheet,
                pt_push,
                propose_geom,
                min_dist=min_dist,
                top_n=pool,
            ),
        )
    if (
        _proposer_enabled("ribbon_free", enabled_proposers)
        and propose_cfg.use_ribbon_seeds
        and use_free_region
    ):
        _ext(
            "ribbon_free",
            propose_placements_ribbon_free(
                base_shape,
                shape_to_place,
                sheet,
                min_dist,
                num_angles=n_angles,
                top_n=pool,
                propose_geom=propose_geom,
                pt_push=pt_push,
            ),
        )
    if (
        _proposer_enabled("group_fit", enabled_proposers)
        and propose_cfg.use_group_edge_seeds
        and focal_shape is not None
        and not focal_shape.is_empty
        and not base_shape.is_empty
        and use_free_region
    ):
        # Packed packs: keep angles/samples modest (profiles already lower samples).
        gf_angles = min(n_angles, 8)
        _ext(
            "group_fit",
            propose_placements_group_fit(
                focal_shape,
                shape_to_place,
                sheet,
                base_shape,
                min_dist=min_dist,
                num_angles=gf_angles,
                top_n=pool * 2,
                samples_per_edge=propose_cfg.group_edge_samples_per_edge,
                propose_geom=propose_geom,
                pt_push=pt_push,
            ),
        )
    board_edge_on = (
        _proposer_enabled("board_edge", enabled_proposers)
        and propose_cfg.use_board_edge_seeds
        and (
            should_use_border_focus(base_shape, propose_cfg)
            or propose_cfg.board_edge_when_packed
        )
    )
    if (
        _proposer_enabled("sheet_corners", enabled_proposers)
        and propose_cfg.use_border_edge_seeds
        and should_use_border_focus(base_shape, propose_cfg)
        and not board_edge_on
    ):
        # Fold sheet_edge samples into sheet_corners counting (no separate key).
        _ext(
            "sheet_corners",
            propose_placements_sheet_corners(
                shape_to_place,
                sheet,
                min_dist,
                propose_geom=propose_geom,
                pt_push=pt_push,
                num_angles=max(n_angles * 4, 24),
                top_n=pool * 2,
            ),
        )
        _ext(
            "sheet_corners",
            propose_placements_sheet_edge(
                shape_to_place,
                sheet,
                min_dist,
                propose_geom=propose_geom,
                pt_push=pt_push,
                num_angles=max(n_angles, 12),
                top_n=pool * 2,
                samples_per_edge=propose_cfg.sheet_edge_samples_per_edge,
                base_shape=base_shape,
            ),
        )
    if board_edge_on:
        # Skip expensive outline snaps when packed mass is far from the border.
        packed_near_border = True
        if base_shape is not None and not base_shape.is_empty:
            packed_near_border = float(base_shape.distance(sheet.exterior)) <= max(
                min_dist * 8.0, 1e-3,
            )
        if packed_near_border or should_use_border_focus(base_shape, propose_cfg):
            be_angles = (
                max(n_angles, 12)
                if base_shape is None or base_shape.is_empty
                else min(n_angles, 8)
            )
            _ext(
                "board_edge",
                propose_placements_board_edge(
                    shape_to_place,
                    sheet,
                    base_shape,
                    min_dist=min_dist,
                    propose_cfg=propose_cfg,
                    propose_geom=propose_geom,
                    pt_push=pt_push,
                    num_angles=be_angles,
                    top_n=pool * 2,
                ),
            )
    if (
        _proposer_enabled("guidance_cast_refine", enabled_proposers)
        and propose_cfg.use_guidance_propositions
        and propose_cfg.guidance_cast_refine_top_k > 0
    ):
        seed_limit = propose_cfg.guidance_cast_refine_top_k
        clustered = cluster_seed_coords(list(candidates))
        structured = select_guidance_cast_seeds(
            clustered,
            seed_limit,
            shape_to_place,
            propose_geom,
            pt_push,
            min_dist,
            focal_shape,
        )
        if not structured and guidance_seed_coords:
            structured = list(guidance_seed_coords)[:seed_limit]
        if structured:
            _ext(
                "guidance_cast_refine",
                propose_placements_guidance_cast(
                    structured,
                    pt_push,
                    propose_geom,
                    propose_cfg,
                    min_dist=min_dist,
                    top_n=propose_cfg.candidate_pool,
                ),
            )
    return _filter_distant_collisions(candidates, propose_geom)


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
) -> List[Tuple[float, float, float]]:
    geom = ProposeGeometry(
        boundary,
        base_shape,
        shape_to_place,
        min_dist,
        epsilon_ratio=propose_cfg.placement_clearance_epsilon_ratio,
        propose_cfg=propose_cfg,
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
    if reserve_coords:
        minx, miny, maxx, maxy = shape_to_place.bounds
        spacing = max(min_dist * 2.0, 0.9 * max(maxx - minx, maxy - miny, 1e-3))
        return _merge_spaced_channel_seeds(
            squeezed,
            reserve_coords,
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
    )
    pocket_reserve: list[tuple[float, float, float]] = []
    pocket_tags: list[str] = []
    pocket_attempts: list[int] = []
    pocket_skips: list[str] = []
    motif_reserve: list[tuple[float, float, float]] = []
    local_proposer_keys: dict[str, set[tuple[float, float, float]]] = {}
    candidates = collect_propose_candidates(
        base_shape,
        shape_to_place,
        sheet,
        cfg,
        min_dist=min_dist,
        pt_push=pt_push,
        propose_geom=geom,
        focal_shape=focal_shape,
        enabled_proposers=enabled_proposers,
        proposer_counts=proposer_counts,
        guidance_seed_coords=guidance_seed_coords,
        cluster_patterns=cluster_patterns,
        group_id=group_id,
        placement_angles_override=placement_angles_override,
        packed_polys=packed_polys,
        packed_group_ids=packed_group_ids,
        packed_transforms=packed_transforms,
        pocket_reserve_out=pocket_reserve,
        pocket_tags_out=pocket_tags,
        pocket_attempts_out=pocket_attempts,
        free_space=free_space,
        guidance_reserve_only=guidance_reserve_only,
        motif_reserve_out=motif_reserve if cfg.unified_void_reserve else None,
        pocket_skip_out=pocket_skips,
        proposer_keys=local_proposer_keys,
    )
    # Keep corridor / pocket seeds by skipping clearance pool trim; force hybrid
    # when pocket teleports are present so clearance ranking cannot discard them.
    if guidance_seed_coords or pocket_reserve:
        hybrid_update = {
            "trim_candidates_by_clearance": False,
        }
        if rank_mode == "clearance":
            rank_mode = "contact_hybrid"
            hybrid_update.update({
                "use_contact_ranking": True,
                "use_contact_clearance_hybrid": True,
                "ranking_mode": "contact_hybrid",
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
        for coords, tag in zip(pocket_reserve, pocket_tags):
            key = _proposal_key(coords)
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
        pocket_stats_out["pocket_skip"] = list(dict.fromkeys(pocket_skips))
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


def _batch_pair_valid(
    sheet: Polygon,
    poly_a: Polygon,
    coords_a: tuple[float, float, float],
    poly_b: Polygon,
    coords_b: tuple[float, float, float],
    base_obstacle: BaseGeometry,
    min_dist: float,
) -> bool:
    placed_a = transform_poly(poly_a, coords_a)
    placed_b = transform_poly(poly_b, coords_b)
    if not sheet.contains(placed_a) or not sheet.contains(placed_b):
        return False
    eps = max(1e-6, min_dist * 1e-4)
    if not base_obstacle.is_empty:
        if base_obstacle.intersects(placed_a) or base_obstacle.intersects(placed_b):
            return False
        if base_obstacle.distance(placed_a) < min_dist - eps:
            return False
        if base_obstacle.distance(placed_b) < min_dist - eps:
            return False
    if placed_a.intersects(placed_b):
        return False
    if placed_a.distance(placed_b) < min_dist - eps:
        return False
    return True


def _batch_pair_contact_score(
    placed_a: BaseGeometry,
    placed_b: BaseGeometry,
    sheet: Polygon,
    min_dist: float,
    focal: BaseGeometry | None,
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
    sheet, _voids = board_context_from_geometry(board)
    pairs: list[tuple[tuple[float, float, float], tuple[float, float, float], float]] = []

    for coords_a in anchor_seeds:
        placed_a = transform_poly(anchor_poly, coords_a)
        if not sheet.contains(placed_a):
            continue
        if not base_obstacle.is_empty:
            if base_obstacle.intersects(placed_a):
                continue
            if base_obstacle.distance(placed_a) < min_dist - 1e-6:
                continue

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

        focal_anchor = (
            unary_union([focal, placed_a])
            if focal is not None and not focal.is_empty
            else placed_a
        )
        for coords_b in follow_coords[: propose_cfg.batch_pack_follow_proposals]:
            if not _batch_pair_valid(
                sheet, anchor_poly, coords_a, follow_poly, coords_b,
                base_obstacle, min_dist,
            ):
                continue
            placed_b = transform_poly(follow_poly, coords_b)
            score = _batch_pair_contact_score(
                placed_a, placed_b, sheet, min_dist, focal_anchor,
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
) -> dict[int, list[tuple[float, float, float]]]:
    """Pack groups sequentially; return extra (x, y, angle) seeds per group."""
    if not propose_cfg.use_batch_pack:
        return {}

    parts_by_group = _batch_parts_by_group(parts)
    group_ids = sorted(parts_by_group)
    if len(group_ids) < 2:
        return {}

    base_obstacle = unary_union(placed) if placed else Polygon()
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
        ):
            all_pairs.append((coords_a, coords_b, score, anchor_gid, follow_gid))

    all_pairs.sort(key=lambda p: p[2])
    all_pairs = all_pairs[: propose_cfg.batch_pack_max_pairs]

    extra: dict[int, list[tuple[float, float, float]]] = {gid: [] for gid in group_ids}
    per_gid_cap = max(1, propose_cfg.batch_pack_max_pairs // len(group_ids))
    gid_counts = {gid: 0 for gid in group_ids}

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


def _best_proposer_coords(
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
) -> List[Tuple[float, float, float]]:
    """All proposers; rank with configured search region and ranking mode."""
    return propose_coords_with_strategy(
        base_shape,
        shape_to_place,
        boundary,
        propose_cfg,
        min_dist=min_dist,
        pt_push=pt_push,
        focal_shape=focal_shape,
        enabled_proposers=enabled_proposers,
        rules=rules,
        group_id=group_id,
        proposer_counts=proposer_counts,
        full_packed_geoms=full_packed_geoms,
        cluster_patterns=cluster_patterns,
        placement_angles_override=placement_angles_override,
        guidance_seed_coords=guidance_seed_coords,
        packed_polys=packed_polys,
        packed_group_ids=packed_group_ids,
        packed_transforms=packed_transforms,
        pocket_stats_out=pocket_stats_out,
        free_space=free_space,
        guidance_reserve_only=guidance_reserve_only,
        void_pole=void_pole,
        proposer_keys_out=proposer_keys_out,
    )


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
) -> dict[int, np.ndarray]:
    """Propose (x, y, angle) seeds per part group.

    Propose uses the nearest packed cluster as obstacles only.
    ``make_polygon_graph`` still filters against the full selection.
    """
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
    full_packed_geoms = [Geometry.from_shapely(p) for p in placed]
    cluster_patterns = []
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
        )
    out: dict[int, np.ndarray] = {}
    total_counts: dict[str, int] = {}
    pocket_emitted = 0
    pocket_attempted = 0
    pocket_accepted = 0
    pocket_keys_by_group: dict[int, set[tuple[float, float, float]]] = {}
    densify_fired = 0
    densify_accepted = 0
    densify_reasons: list[str] = []
    pocket_skips_all: list[str] = []
    proposer_keys_agg: dict[str, set[tuple[float, float, float]]] = {}
    emitted_by_proposer: dict[str, int] = {}
    pool_by_proposer: dict[str, int] = {}
    pocket_by_tag: dict[str, int] = {}
    void_thr = float(propose_cfg.late_border_void_override_ratio)
    if void_thr <= 0.0:
        void_thr = 2.5
    free_snap = None
    if placed:
        # Mean catalog area for a stable board-level snapshot; Mode A still
        # re-checks per part area against the same threshold.
        mean_area = float(np.mean([float(p.area) for p, _ in parts])) if parts else 1.0
        free_snap = build_free_space_snapshot(
            sheet,
            placed,
            mean_area,
            min_dist,
            void_ratio_threshold=void_thr,
        )
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
            if (
                zone == "border_gap"
                and zone_info is not None
                and zone_info.is_annulus
            ):
                # Annulus rim: keep border docking but shoot inward toward the hole.
                cfg = cfg.model_copy(update={
                    "ranking_mode": "contact_hybrid",
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
                    "ranking_mode": "contact_hybrid",
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
        poles_reserve_only = (
            void_hijack_from is not None
            and bool(propose_cfg.poles_reserve_only_on_hijack)
        )
        coords = _best_proposer_coords(
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
        )
        pocket_emitted += int(pocket_stats.get("emitted", 0))
        pocket_attempted += int(pocket_stats.get("attempted", 0))
        pocket_accepted += int(pocket_stats.get("selected", 0))
        for sk in pocket_stats.get("pocket_skip") or []:
            pocket_skips_all.append(str(sk))
        for pk in pocket_stats.get("pocket_keys") or []:
            pocket_keys_by_group.setdefault(int(group_id), set()).add(tuple(pk))
        for name, n in group_counts.items():
            total_counts[name] = total_counts.get(name, 0) + n
        for name, keys in group_proposer_keys.items():
            proposer_keys_agg.setdefault(name, set()).update(keys)
        for name, n in (pocket_stats.get("emitted_by_proposer") or {}).items():
            emitted_by_proposer[name] = emitted_by_proposer.get(name, 0) + int(n)
        for name, n in (pocket_stats.get("pool_by_proposer") or {}).items():
            pool_by_proposer[name] = pool_by_proposer.get(name, 0) + int(n)
        for tag, n in (pocket_stats.get("by_tag") or {}).items():
            pocket_by_tag[tag] = pocket_by_tag.get(tag, 0) + int(n)
        arr = propositions_to_ndarray(coords)
        if border_focus and cfg.use_board_edge_seeds:
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
        # Mode A hijack already implies a large void — do not require free_ratio.
        hijack_densify = (
            bool(propose_cfg.densify_on_void_hijack)
            and void_hijack_from is not None
            and placed
            and not border_only_propose
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
            from nest_graph.propose.ranking import _score_placement_clearance
            densify_geom = ProposeGeometry(
                board,
                obstacle_shape,
                part_poly,
                min_dist,
                epsilon_ratio=cfg.placement_clearance_epsilon_ratio,
                propose_cfg=cfg,
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
        if need_densify:
            densify_fired += 1
            densify_cfg = ProposeConfig.for_place("void_seek", base=propose_cfg)
            densify_rank = "contact_hybrid" if propose_cfg.use_pocket_fit else "clearance"
            densify_cfg = densify_cfg.model_copy(update={
                "use_guidance_propositions": True,
                "guidance_use_tight_packing": True,
                "guidance_use_corner_alignment": True,
                "guidance_enable_grid": False,
                "cast_squeeze_top_k": max(4, int(densify_cfg.cast_squeeze_top_k)),
                "use_neighbor_slide": True,
                "use_ribbon_seeds": True,
                "use_full_packed_obstacle": True,
                "use_border_focus": False,
                "border_focus_ranking": False,
                "ranking_mode": densify_rank,
                "use_contact_ranking": densify_rank == "contact_hybrid",
                "use_contact_clearance_hybrid": densify_rank == "contact_hybrid",
            })
            densify_cfg = apply_proposer_pool_scales(
                densify_cfg, propose_cfg.place_proposer_pool_scales,
            )
            densify_obs = obstacle_shape_for_propose(
                placed,
                part_poly,
                min_dist,
                propose_cfg=densify_cfg.model_copy(update={"obstacle_nearest_k": 6}),
                sheet=sheet,
                ref_point=primary_target,
            )
            if densify_obs is None or densify_obs.is_empty:
                densify_obs = simplify_obstacle_union(placed, min_dist)
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
            densify_coords = _best_proposer_coords(
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
            )
            for name, n in densify_counts.items():
                total_counts[name] = total_counts.get(name, 0) + n
            for name, keys in densify_keys.items():
                proposer_keys_agg.setdefault(name, set()).update(keys)
            for name, n in (densify_pocket.get("emitted_by_proposer") or {}).items():
                emitted_by_proposer[name] = emitted_by_proposer.get(name, 0) + int(n)
            for name, n in (densify_pocket.get("pool_by_proposer") or {}).items():
                pool_by_proposer[name] = pool_by_proposer.get(name, 0) + int(n)
            for tag, n in (densify_pocket.get("by_tag") or {}).items():
                pocket_by_tag[tag] = pocket_by_tag.get(tag, 0) + int(n)
            pocket_emitted += int(densify_pocket.get("emitted", 0))
            pocket_attempted += int(densify_pocket.get("attempted", 0))
            pocket_accepted += int(densify_pocket.get("selected", 0))
            for pk in densify_pocket.get("pocket_keys") or []:
                pocket_keys_by_group.setdefault(int(group_id), set()).add(tuple(pk))
            for sk in densify_pocket.get("pocket_skip") or []:
                pocket_skips_all.append(str(sk))
            densify_arr = propositions_to_ndarray(densify_coords)
            yield_poly = None
            if part_free_space is not None and part_free_space.analysis is not None:
                yield_poly = part_free_space.analysis.target_poly
            use_void_yield = (
                bool(getattr(propose_cfg, "enable_void_yield_densify_accept", True))
                and void_hijack_from is not None
                and yield_poly is not None
                and not yield_poly.is_empty
            )
            if use_void_yield:
                old_iv = _count_transforms_in_void(arr, yield_poly, part_poly)
                new_iv = _count_transforms_in_void(densify_arr, yield_poly, part_poly)
                if new_iv > old_iv:
                    densify_accepted += 1
                    densify_reasons.append("void_yield_gain")
                    arr = densify_arr[: densify_cfg.max_proposals]
                    zone_label = f"{zone}→void_seek"
                else:
                    densify_reasons.append("void_yield_drop")
            elif densify_arr.shape[0] > arr.shape[0]:
                densify_accepted += 1
                densify_reasons.append("count_gain")
                arr = densify_arr[: densify_cfg.max_proposals]
                zone_label = f"{zone}→void_seek"
            else:
                densify_reasons.append("count_drop")

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
        densify_stats_out["emitted_by_proposer"] = dict(emitted_by_proposer)
        densify_stats_out["pool_by_proposer"] = dict(pool_by_proposer)
        densify_stats_out["pocket_by_tag"] = dict(pocket_by_tag)

    # Batch-pack re-runs full proposers per anchor; only useful on empty / near-empty sheets.
    if propose_cfg.use_batch_pack and len(out) >= 2 and not placed:
        batch_extra = augment_batch_pack_proposals(
            board,
            parts,
            placed,
            out,
            propose_cfg,
            min_dist=min_dist,
        )
        for gid, extra in batch_extra.items():
            if not extra:
                continue
            merged = np.concatenate(
                [out.get(gid, np.zeros((0, 3))), propositions_to_ndarray(extra)],
            )
            out[gid] = dedupe_transforms(merged)

    return out
