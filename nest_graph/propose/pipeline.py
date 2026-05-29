import math
from typing import Dict, List, Optional, Sequence, Tuple, Union

import numpy as np
from shapely import LineString, LinearRing, MultiLineString, MultiPoint, MultiPolygon, Point, Polygon
from shapely.affinity import rotate, translate
from shapely.geometry.base import BaseGeometry
from shapely.ops import nearest_points, polylabel, unary_union, voronoi_diagram

from nest_graph.board import board_context_from_geometry
from nest_graph.config import ProposeConfig, dedupe_transforms
from nest_graph.geometry import Geometry
from nest_graph.placement_scene import (
    PLACEMENT_EPSILON_RATIO,
    best_proposition,
    build_placement_scene,
    guidance_config_for_propose,
    guidance_config_for_scene,
    guidance_ray_direction_candidates,
    is_valid_placement,
    placement_footprint_inside_board,
    footprints_inside_board,
    proposition_translation,
    tiered_propositions,
)
from nest_graph.utils import get_shape_exteriors, get_shape_polygons_coords, transform_poly

from nest_graph.propose.context import (
    border_focal_for_propose,
    effective_ranking_mode,
    focal_shape_for_propose,
    obstacle_shape_for_propose,
    propose_push_point,
    should_use_border_focus,
)
from nest_graph.propose.geometry import ProposeGeometry
from nest_graph.propose.placements_edge import (
    propose_placements_group_fit,
    propose_placements_ribbon_free,
    propose_placements_sheet_corners,
    propose_placements_sheet_edge,
)
from nest_graph.propose.placements_geo import propose_placements_raycasting, propose_placements_voronoi
from nest_graph.propose.placements_guidance import (
    propose_placements_guidance_propositions,
    propose_placements_guidance_walk,
)
from nest_graph.propose.placements_primary import (
    propose_placements_axis_push,
    propose_placements_bottom_left,
    propose_placements_erosion,
    propose_placements_neighbor_slide,
    propose_placements_nfp_vertices,
    propose_placements_perimeter_walk,
)
from nest_graph.propose.placements_pso import propose_placements_point_cloud
from nest_graph.propose.ranking import (
    _rank_proposal_coords,
    _score_placement_border,
    _trim_candidates_by_clearance,
    _trim_candidates_by_contact,
    _trim_candidates_stratified,
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
    "axis_push",
    "bottom_left",
    "nfp_vertices",
    "erosion",
    "raycasting",
    "voronoi",
    "point_cloud",
    "guidance_walk",
    "ribbon_free",
    "group_fit",
    "sheet_corners",
    "sheet_edge",
    "guidance_propositions",
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
) -> None:
    n = len(new_items)
    if proposer_counts is not None:
        proposer_counts[name] = proposer_counts.get(name, 0) + n
    candidates.extend(new_items)


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
    pool = propose_cfg.candidate_pool
    border_focus = should_use_border_focus(base_shape, propose_cfg)
    use_free_region = propose_cfg.use_free_region_search
    n_angles = propose_cfg.placement_num_angles
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
            ),
        )
    if (
        _proposer_enabled("neighbor_slide", enabled_proposers)
        and propose_cfg.use_neighbor_slide
        and not base_shape.is_empty
    ):
        neighbor_top = pool if propose_cfg.guidance_use_tight_packing else pool * 2
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
            ),
        )
    if (
        _proposer_enabled("axis_push", enabled_proposers)
        and propose_cfg.use_axis_push
        and not base_shape.is_empty
    ):
        _extend_counted(
            candidates,
            proposer_counts,
            "axis_push",
            propose_placements_axis_push(
                base_shape,
                shape_to_place,
                sheet,
                min_dist,
                propose_geom=propose_geom,
                pt_push=pt_push,
                num_angles=n_angles,
                top_n=pool * 2,
            ),
        )
    if _proposer_enabled("bottom_left", enabled_proposers) and propose_cfg.use_bottom_left:
        _extend_counted(
            candidates,
            proposer_counts,
            "bottom_left",
            propose_placements_bottom_left(
                base_shape,
                shape_to_place,
                sheet,
                min_dist,
                propose_geom=propose_geom,
                pt_push=pt_push,
                use_free_region=use_free_region,
                border_focus=border_focus,
                num_angles=n_angles,
                top_n=pool,
                vertices_per_angle=propose_cfg.bottom_left_vertices_per_angle,
            ),
        )
    if _proposer_enabled("nfp_vertices", enabled_proposers) and propose_cfg.use_nfp_vertices:
        _extend_counted(
            candidates,
            proposer_counts,
            "nfp_vertices",
            propose_placements_nfp_vertices(
                base_shape,
                shape_to_place,
                sheet,
                min_dist,
                propose_geom=propose_geom,
                pt_push=pt_push,
                num_angles=n_angles,
                top_n=pool * 2,
            ),
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
            ),
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
        _proposer_enabled("guidance_propositions", enabled_proposers)
        and propose_cfg.use_guidance_propositions
    ):
        structured = list(candidates)
        if not structured and guidance_seed_coords:
            structured = list(guidance_seed_coords)
        if not structured:
            return candidates
        if len(structured) > propose_cfg.guidance_proposition_seed_count:
            structured = structured[: propose_cfg.guidance_proposition_seed_count]
        _extend_counted(
            candidates,
            proposer_counts,
            "guidance_propositions",
            propose_placements_guidance_propositions(
                structured,
                pt_push,
                propose_geom,
                propose_cfg,
                min_dist=min_dist,
            ),
        )
    return candidates


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
    if len(pool) > propose_cfg.candidate_pool:
        if rank_mode == "border" and propose_cfg.trim_candidates_by_clearance:
            pool = _trim_candidates_by_border(
                pool, shape_to_place, geom, pt_push, min_dist, propose_cfg.candidate_pool,
            )
        elif rank_mode in ("contact", "contact_hybrid") and propose_cfg.trim_candidates_by_clearance:
            if propose_cfg.use_stratified_contact_trim and rank_mode == "contact_hybrid":
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
    return _rank_proposal_coords(
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
    )


def propose_coords_with_strategy(
    base_shape: BaseGeometry,
    shape_to_place: Polygon,
    boundary: BaseGeometry,
    propose_cfg: ProposeConfig,
    *,
    min_dist: float,
    pt_push: Point,
    focal_shape: Optional[BaseGeometry] = None,
) -> List[Tuple[float, float, float]]:
    cfg = propose_cfg
    rank_mode = effective_ranking_mode(cfg, base_shape)
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
) -> dict[int, np.ndarray]:
    """Propose (x, y, angle) seeds per part group.

    Propose uses the nearest packed cluster as obstacles only.
    ``make_polygon_graph`` still filters against the full selection.
    """
    placed = [selected_polys[i] for i in selected_indices]
    out: dict[int, np.ndarray] = {}
    for part_poly, group_id in parts:
        obstacle_shape = obstacle_shape_for_propose(placed, part_poly, min_dist)
        focal = focal_shape_for_propose(
            board, placed, part_poly, min_dist, propose_cfg,
        )
        border_focus = should_use_border_focus(obstacle_shape, propose_cfg)
        push = pt_push if pt_push is not None else propose_push_point(
            board,
            obstacle_shape,
            smart_push=propose_cfg.smart_push_target,
            min_dist=min_dist,
            use_border_focus=border_focus,
        )
        coords = _best_proposer_coords(
            obstacle_shape,
            part_poly,
            board,
            propose_cfg,
            min_dist=min_dist,
            pt_push=push,
            focal_shape=focal,
        )
        out[group_id] = propositions_to_ndarray(coords)
    return out
