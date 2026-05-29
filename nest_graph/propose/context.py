import math
from typing import List, Optional, Sequence, Tuple, Union

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

def placement_free_region(
    sheet: Polygon,
    base_shape: BaseGeometry,
    min_dist: float,
) -> BaseGeometry:
    """Nestable sheet minus clearance buffer around the packed layout."""
    if base_shape is None or base_shape.is_empty:
        return sheet
    free = sheet.difference(base_shape.buffer(min_dist))
    if free.is_empty:
        return sheet
    return free


def cluster_packed_solid_groups(
    polys: Sequence[BaseGeometry],
    min_dist: float,
) -> list[BaseGeometry]:
    """Connected clusters of packed parts (touching within clearance gap)."""
    placed = [p for p in polys if p is not None and not p.is_empty]
    if not placed:
        return []
    if len(placed) == 1:
        return [unary_union(placed)]

    gap = max(min_dist * 0.5, 1e-6)
    buffered = [p.buffer(gap) for p in placed]
    merged = unary_union(buffered)
    if merged.is_empty:
        return []

    if isinstance(merged, MultiPolygon):
        blobs: list[BaseGeometry] = list(merged.geoms)
    else:
        blobs = [merged]
    groups: list[BaseGeometry] = []
    for blob in blobs:
        members = [p for p in placed if p.buffer(gap).intersects(blob)]
        if members:
            groups.append(unary_union(members))
    return groups


def border_solid_focal(sheet: Polygon, min_dist: float) -> BaseGeometry:
    """Thin ring along the nestable sheet outer boundary for edge-fitting proposals."""
    inset = max(min_dist * 2.0, 1e-4)
    inner = sheet.buffer(-inset)
    if inner.is_empty or inner.area < sheet.area * 1e-6:
        return sheet
    ring = sheet.difference(inner)
    return ring if not ring.is_empty else sheet


def obstacle_polys_for_propose(
    placed: Sequence[BaseGeometry],
    part_poly: Polygon,
    min_dist: float,
) -> list[BaseGeometry]:
    """Nearest packed cluster only; graph still checks the full layout."""
    if not placed:
        return []
    groups = cluster_packed_solid_groups(placed, min_dist)
    if not groups:
        return []
    if len(groups) == 1:
        return list(placed)
    gap = max(min_dist * 0.5, 1e-6)
    ref = part_poly.centroid
    nearest = min(groups, key=lambda g: g.distance(ref))
    return [p for p in placed if p.buffer(gap).intersects(nearest)]


def obstacle_shape_for_propose(
    placed: Sequence[BaseGeometry],
    part_poly: Polygon,
    min_dist: float,
) -> BaseGeometry:
    polys = obstacle_polys_for_propose(placed, part_poly, min_dist)
    if not polys:
        return Polygon()
    return unary_union(polys)


def focal_shape_for_propose(
    board: BaseGeometry,
    placed: Sequence[BaseGeometry],
    part_poly: Polygon,
    min_dist: float,
    propose_cfg: ProposeConfig,
) -> Optional[BaseGeometry]:
    """Focal geometry for ray anchors and group/board edge seeds."""
    obstacle = obstacle_shape_for_propose(placed, part_poly, min_dist)
    if obstacle is not None and not obstacle.is_empty:
        return obstacle
    if propose_cfg.use_border_focus:
        return border_focal_for_propose(board, min_dist)
    return None


def search_region_for_placement(
    base_shape: BaseGeometry,
    boundary: Optional[BaseGeometry],
    sheet: Optional[Polygon],
    min_dist: float,
    *,
    use_free_region: bool,
    border_focus: bool = False,
) -> BaseGeometry:
    if use_free_region and sheet is not None:
        return placement_free_region(sheet, base_shape, min_dist)
    if base_shape is not None and not base_shape.is_empty:
        return base_shape
    if boundary is not None and not boundary.is_empty:
        return boundary
    return sheet if sheet is not None else Polygon()


def should_use_border_focus(
    base_shape: BaseGeometry,
    propose_cfg: ProposeConfig,
) -> bool:
    """Empty sheet: fit parts against the nestable border, not the board centroid."""
    if not propose_cfg.use_border_focus:
        return False
    return base_shape is None or base_shape.is_empty


def border_focal_for_propose(
    board: BaseGeometry,
    min_dist: float,
) -> BaseGeometry:
    sheet, _voids = board_context_from_geometry(board)
    return border_solid_focal(sheet, min_dist)


def propose_push_point(
    board: BaseGeometry,
    base_shape: BaseGeometry,
    *,
    smart_push: bool,
    min_dist: float = 0.0,
    use_border_focus: bool = False,
) -> Point:
    if smart_push and base_shape is not None and not base_shape.is_empty:
        return base_shape.centroid
    if use_border_focus and (base_shape is None or base_shape.is_empty):
        border = border_focal_for_propose(board, max(min_dist, 1e-6))
        if not border.is_empty:
            return border.centroid
    return board.centroid


def effective_ranking_mode(
    propose_cfg: ProposeConfig,
    base_shape: BaseGeometry,
) -> str:
    if (
        propose_cfg.border_focus_ranking
        and should_use_border_focus(base_shape, propose_cfg)
    ):
        return "border"
    if (
        propose_cfg.use_contact_ranking
        and base_shape is not None
        and not base_shape.is_empty
    ):
        if propose_cfg.use_contact_clearance_hybrid:
            return "contact_hybrid"
        return "contact"
    return propose_cfg.ranking_mode


def _placement_contact_error(
    placed: BaseGeometry,
    sheet: Polygon,
    min_dist: float,
    focal_shape: Optional[BaseGeometry] = None,
) -> float:
    """Distance from ideal standoff (0 = flush against border or group)."""
    border_err = abs(float(placed.distance(sheet.exterior)) - min_dist)
    if focal_shape is not None and not focal_shape.is_empty:
        group_err = abs(float(focal_shape.distance(placed)) - min_dist)
        return min(border_err, group_err)
    return border_err

