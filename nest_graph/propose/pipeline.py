from typing import List, Optional, Sequence, Tuple, Union

import numpy as np
from shapely import Point, Polygon
from shapely.geometry.base import BaseGeometry
from shapely.ops import unary_union

from nest_graph.board import board_context_from_geometry
from nest_graph.config import ProposeConfig, dedupe_transforms
from nest_graph.geometry import Geometry

from nest_graph.propose.context import (
    apply_proposer_pool_scales,
    border_focal_for_propose,
    classify_propose_zone,
    effective_ranking_mode,
    focal_shape_for_propose,
    obstacle_shape_for_propose,
    propose_push_point,
    should_use_border_focus,
)
from nest_graph.propose.geometry import ProposeGeometry, batch_valid_flags
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
from nest_graph.propose.placements_pso import propose_placements_point_cloud
from nest_graph.propose.placements_batch import augment_batch_pack_proposals
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


def base_shape_from_selection(
    polys: Sequence[BaseGeometry],
    selected_indices: Sequence[int],
) -> Union[Polygon, BaseGeometry]:
    placed = [polys[i] for i in selected_indices]
    if not placed:
        return Polygon()
    return unary_union(placed)


ALL_PROPOSER_NAMES: tuple[str, ...] = (
    "perimeter_walk",
    "neighbor_slide",
    "erosion",
    "raycasting",
    "voronoi",
    "point_cloud",
    "guidance_walk",
    "ribbon_free",
    "group_fit",
    "sheet_corners",
    "sheet_edge",
    "board_edge",
    "guidance_cast_refine",
    "batch_pack",
)


def _proposer_enabled(
    name: str,
    enabled_proposers: frozenset[str] | None,
) -> bool:
    if enabled_proposers is None:
        return True
    return name in enabled_proposers


def _extend_counted(
    candidates: list[tuple[float, float, float]],
    proposer_counts: dict[str, int] | None,
    name: str,
    new_items: Sequence[tuple[float, float, float]],
    *,
    max_items: int | None = None,
) -> None:
    items = list(new_items)
    if max_items is not None and len(items) > max_items:
        items = items[:max_items]
    n = len(items)
    if proposer_counts is not None:
        proposer_counts[name] = proposer_counts.get(name, 0) + n
    candidates.extend(items)


def pre_filter_candidates(
    candidates: Sequence[Tuple[float, float, float]],
    propose_geom: ProposeGeometry,
    pt_push: Point,
    min_dist: float,
    epsilon_ratio: float,
    *,
    trim_by_validity: bool = True,
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
    )


def _filter_valid_candidates(
    candidates: Sequence[Tuple[float, float, float]],
    propose_geom: ProposeGeometry,
    pt_push: Point,
    min_dist: float,
    epsilon_ratio: float,
) -> List[Tuple[float, float, float]]:
    if not candidates:
        return []
    flags = batch_valid_flags(propose_geom, candidates, pt_push, return_guidance=False)
    return [c for c, ok in zip(candidates, flags, strict=True) if ok]


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
    )
    pool = ctx.pool
    border_focus = ctx.border_focus
    use_free_region = ctx.use_free_region
    n_angles = ctx.n_angles
    placement_angles = ctx.placement_angles
    candidates: List[Tuple[float, float, float]] = []

    if _proposer_enabled("perimeter_walk", enabled_proposers):
        _extend_counted(
            candidates,
            proposer_counts,
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
        _extend_counted(
            candidates,
            proposer_counts,
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
        _extend_counted(
            candidates,
            proposer_counts,
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
        _extend_counted(
            candidates,
            proposer_counts,
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
        _extend_counted(
            candidates,
            proposer_counts,
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
        _extend_counted(
            candidates,
            proposer_counts,
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
        _extend_counted(
            candidates,
            proposer_counts,
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
        _extend_counted(
            candidates,
            proposer_counts,
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
        _extend_counted(
            candidates,
            proposer_counts,
            "group_fit",
            propose_placements_group_fit(
                focal_shape,
                shape_to_place,
                sheet,
                base_shape,
                min_dist=min_dist,
                num_angles=max(n_angles, 12),
                top_n=pool * 2,
                samples_per_edge=propose_cfg.group_edge_samples_per_edge,
                propose_geom=propose_geom,
                pt_push=pt_push,
            ),
        )
    if (
        _proposer_enabled("sheet_corners", enabled_proposers)
        and propose_cfg.use_border_edge_seeds
        and should_use_border_focus(base_shape, propose_cfg)
    ):
        _extend_counted(
            candidates,
            proposer_counts,
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
        _extend_counted(
            candidates,
            proposer_counts,
            "sheet_edge",
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
    if (
        _proposer_enabled("board_edge", enabled_proposers)
        and propose_cfg.use_board_edge_seeds
        and (
            should_use_border_focus(base_shape, propose_cfg)
            or propose_cfg.board_edge_when_packed
        )
    ):
        _extend_counted(
            candidates,
            proposer_counts,
            "board_edge",
            propose_placements_board_edge(
                shape_to_place,
                sheet,
                base_shape,
                min_dist=min_dist,
                propose_cfg=propose_cfg,
                propose_geom=propose_geom,
                pt_push=pt_push,
                num_angles=max(n_angles, 12),
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
            _extend_counted(
                candidates,
                proposer_counts,
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
        key = (round(coords[0], 3), round(coords[1], 3), round(coords[2], 3))
        if key in seen:
            continue
        seen.add(key)
        score = _score_placement_border(coords, shape_to_place, propose_geom, pt_push, min_dist)
        if score > float("-inf"):
            scored.append((score, coords))
    scored.sort(key=lambda x: x[0], reverse=True)
    return [coords for _, coords in scored[:limit]]


def _propose_coords_from_candidates(
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
    )
    return _apply_cast_squeeze_if_enabled(
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
    )
    return _propose_coords_from_candidates(
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
    )


_FIRST_PASS_EMPTY_BORDER_PROPOSERS = frozenset({
    "board_edge", "sheet_corners", "perimeter_walk",
})


def _first_pass_packed_border_proposers(propose_cfg: ProposeConfig) -> frozenset[str]:
    return frozenset({
        "board_edge", "sheet_corners", "group_fit", "neighbor_slide", "ribbon_free",
    })


_FIRST_PASS_PACKED_BORDER_PROPOSERS = _first_pass_packed_border_proposers(
    ProposeConfig(),
)


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
    from nest_graph.propose.placements_edge import propose_placements_board_edge
    from nest_graph.propose.geometry import ProposeGeometry

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
) -> dict[int, np.ndarray]:
    """Propose (x, y, angle) seeds per part group.

    Propose uses the nearest packed cluster as obstacles only.
    ``make_polygon_graph`` still filters against the full selection.
    """
    placed = [selected_polys[i] for i in selected_indices]
    full_packed_geoms = [Geometry.from_shapely(p) for p in placed]
    out: dict[int, np.ndarray] = {}
    total_counts: dict[str, int] = {}
    for part_poly, group_id in parts:
        zone: str | None = None
        cfg = propose_cfg
        if propose_cfg.place_profiles_enabled and not border_only_propose:
            if not placed:
                zone = "empty_border"
            else:
                obs_preview = obstacle_shape_for_propose(
                    placed, part_poly, min_dist, propose_cfg=propose_cfg,
                )
                zone = classify_propose_zone(
                    board,
                    obs_preview,
                    part_poly,
                    min_dist=min_dist,
                    propose_cfg=propose_cfg,
                    selected_polys=placed,
                )
            cfg = ProposeConfig.for_place(zone, base=propose_cfg)
            cfg = apply_proposer_pool_scales(cfg, propose_cfg.place_proposer_pool_scales)
            if zones_used_out is not None:
                zones_used_out.append(zone)

        use_full = use_full_packed_obstacle
        if zone is not None:
            use_full, nearest_k = ProposeConfig.obstacle_scope_for_place(zone)
            if nearest_k > 0:
                cfg = cfg.model_copy(update={"obstacle_nearest_k": nearest_k})

        if propose_feedback is not None and propose_feedback.last_proposal_yield < 0.4:
            if placed:
                bumped_k = min(cfg.obstacle_nearest_k + 1, len(placed))
                cfg = cfg.model_copy(update={"obstacle_nearest_k": bumped_k})
            if zone == "inter_cluster":
                use_full = True

        if use_full and placed:
            obstacle_shape = unary_union(placed)
        else:
            obstacle_shape = obstacle_shape_for_propose(
                placed, part_poly, min_dist, propose_cfg=cfg,
            )
        focal = focal_shape_for_propose(
            board, placed, part_poly, min_dist, cfg,
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
                enabled = _FIRST_PASS_EMPTY_BORDER_PROPOSERS
            elif placed:
                enabled = _first_pass_packed_border_proposers(cfg)
        elif zone is not None:
            enabled = ProposeConfig.proposers_for_place(zone)
        group_counts: dict[str, int] = {}
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
        )
        for name, n in group_counts.items():
            total_counts[name] = total_counts.get(name, 0) + n
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
        out[group_id] = arr

    if proposer_counts_out is not None:
        proposer_counts_out.clear()
        proposer_counts_out.update(total_counts)

    if propose_cfg.use_batch_pack and len(out) >= 2:
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
