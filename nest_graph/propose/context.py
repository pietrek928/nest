from typing import Optional, Sequence

from shapely import MultiPolygon, Point, Polygon
from shapely.geometry.base import BaseGeometry
from shapely.ops import unary_union

from nest_graph.board import board_context_from_geometry
from nest_graph.config import ProposeConfig
from nest_graph.geometry import Geometry
from nest_graph.propose.placement_outline import outline_standoff_distance

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
    index_groups = cluster_packed_indices(polys, min_dist)
    groups: list[BaseGeometry] = []
    for idxs in index_groups:
        members = [polys[i] for i in idxs]
        groups.append(unary_union(members) if len(members) > 1 else members[0])
    return groups


def cluster_packed_indices(
    polys: Sequence[BaseGeometry],
    min_dist: float,
) -> list[list[int]]:
    """Indices of packed parts grouped into contact-connected clusters."""
    placed_idx = [i for i, p in enumerate(polys) if p is not None and not p.is_empty]
    if not placed_idx:
        return []
    if len(placed_idx) == 1:
        return [placed_idx]

    gap = max(min_dist * 0.5, 1e-6)
    buffered = {i: polys[i].buffer(gap) for i in placed_idx}
    merged = unary_union(list(buffered.values()))
    if merged.is_empty:
        return []

    if isinstance(merged, MultiPolygon):
        blobs: list[BaseGeometry] = list(merged.geoms)
    else:
        blobs = [merged]
    groups: list[list[int]] = []
    for blob in blobs:
        members = [i for i, b in buffered.items() if b.intersects(blob)]
        if members:
            groups.append(members)
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
    *,
    nearest_k: int | None = None,
    propose_cfg: ProposeConfig | None = None,
) -> list[BaseGeometry]:
    """Parts in the k nearest packed clusters; graph still checks the full layout."""
    if not placed:
        return []
    k_val = nearest_k
    if k_val is None and propose_cfg is not None:
        k_val = propose_cfg.obstacle_nearest_k
    if k_val is None:
        k_val = 2
    groups = cluster_packed_solid_groups(placed, min_dist)
    if not groups:
        return []
    gap = max(min_dist * 0.5, 1e-6)
    if len(groups) == 1:
        return list(placed)
    ref = part_poly.centroid
    k = max(1, min(k_val, len(groups)))
    ranked = sorted(groups, key=lambda g: g.distance(ref))[:k]
    obstacle = unary_union(ranked)
    return [p for p in placed if p.intersects(obstacle)]


def obstacle_shape_for_propose(
    placed: Sequence[BaseGeometry],
    part_poly: Polygon,
    min_dist: float,
    *,
    propose_cfg: ProposeConfig | None = None,
) -> BaseGeometry:
    polys = obstacle_polys_for_propose(
        placed, part_poly, min_dist, propose_cfg=propose_cfg,
    )
    if not polys:
        return Polygon()
    if len(polys) == 1:
        return polys[0]
    return simplify_obstacle_union(polys, min_dist)


def simplify_obstacle_union(
    polys: Sequence[BaseGeometry],
    min_dist: float,
) -> BaseGeometry:
    """Merge packed parts; hull only for very dense clusters to keep proposers cheap."""
    if not polys:
        return Polygon()
    if len(polys) == 1:
        return polys[0]
    gap = max(min_dist * 0.5, 1e-6)
    merged = unary_union([p.buffer(gap) for p in polys]).buffer(-gap)
    if merged.is_empty:
        return unary_union(list(polys))
    # Preserve concavities for moderate packs (group_fit / neighbor_slide).
    if len(polys) >= 12:
        hull = merged.convex_hull
        if not hull.is_empty:
            return hull
    return merged


def focal_shape_for_propose(
    board: BaseGeometry,
    placed: Sequence[BaseGeometry],
    part_poly: Polygon,
    min_dist: float,
    propose_cfg: ProposeConfig,
) -> Optional[BaseGeometry]:
    """Focal geometry for ray anchors and group/board edge seeds."""
    obstacle = obstacle_shape_for_propose(
        placed, part_poly, min_dist, propose_cfg=propose_cfg,
    )
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
    *,
    rules=None,
) -> str:
    if (
        propose_cfg.border_focus_ranking
        and should_use_border_focus(base_shape, propose_cfg)
    ):
        base_mode = "border"
    elif (
        propose_cfg.use_contact_ranking
        and base_shape is not None
        and not base_shape.is_empty
    ):
        if propose_cfg.use_contact_clearance_hybrid:
            base_mode = "contact_hybrid"
        else:
            base_mode = "contact"
    else:
        base_mode = propose_cfg.ranking_mode
    if (
        propose_cfg.use_rule_ranking
        and rules is not None
        and rules.size() > 0
        and base_mode in ("border", "contact", "contact_hybrid", "rule_hybrid")
    ):
        return "rule_hybrid"
    return base_mode


def outline_coverage_ratio(
    placed: Sequence[BaseGeometry],
    sheet: Polygon,
    min_dist: float,
) -> float:
    if sheet.is_empty or not placed:
        return 0.0
    perimeter = float(sheet.exterior.length)
    if perimeter <= 0:
        return 0.0
    tol = max(min_dist * 2.0, 1e-4)
    merged = unary_union(placed)
    if merged.is_empty:
        return 0.0
    covered_len = merged.buffer(tol).intersection(sheet.exterior).length
    return float(covered_len / perimeter)


def classify_propose_zone(
    board: BaseGeometry,
    obstacle_shape: BaseGeometry,
    part_poly: Polygon,
    *,
    min_dist: float,
    propose_cfg: ProposeConfig,
    selected_polys: Sequence[BaseGeometry] | None = None,
    selected_indices: Sequence[int] | None = None,
) -> str:
    """Classify where on the sheet this part should be proposed."""
    sheet, voids = board_context_from_geometry(board)
    placed = list(selected_polys or [])
    if not placed or obstacle_shape is None or obstacle_shape.is_empty:
        return "empty_border"

    free = placement_free_region(sheet, obstacle_shape, min_dist)
    free_ratio = float(free.area / sheet.area) if not sheet.is_empty else 0.0
    clusters = cluster_packed_solid_groups(placed, min_dist)
    part_c = part_poly.centroid
    border_dist = float(part_c.distance(sheet.exterior))

    coverage = outline_coverage_ratio(placed, sheet, min_dist)
    border_touch_tol = max(min_dist * 8.0, 1e-3)
    packed_near_border = any(
        float(p.distance(sheet.exterior)) <= border_touch_tol for p in placed
    )
    # Interior-only packs (e.g. center interlock seeds) must not use border_gap.
    if (
        coverage < propose_cfg.place_border_coverage_threshold
        and packed_near_border
    ):
        return "border_gap"

    if voids:
        part_geom = Geometry.from_shapely(part_poly)
        void_tol = min_dist * 8.0
        for void_g in voids:
            if void_g.distance(part_geom) < void_tol:
                return "void_seek"

    if len(clusters) >= 2 and free_ratio > 0.08:
        return "inter_cluster"

    if free_ratio >= propose_cfg.place_free_area_interior_threshold:
        return "interior_pocket"

    inset = max(min_dist * 4.0, 1e-4)
    inner = sheet.buffer(-inset)
    if (
        not inner.is_empty
        and inner.contains(part_c)
        and border_dist > min_dist * 6.0
    ):
        return "interior_pocket"

    return "cluster_edge"


def apply_proposer_pool_scales(
    propose_cfg: ProposeConfig,
    pool_scales: dict[str, float],
) -> ProposeConfig:
    if not pool_scales:
        return propose_cfg
    data = propose_cfg.model_dump()
    ns = float(pool_scales.get("neighbor_slide", 1.0))
    data["neighbor_slide_pool_fraction"] = min(
        1.0,
        max(0.05, propose_cfg.neighbor_slide_pool_fraction * ns),
    )
    return ProposeConfig(**data)


def placement_contact_error(
    placed: BaseGeometry,
    sheet: Polygon,
    min_dist: float,
    focal_shape: Optional[BaseGeometry] = None,
) -> float:
    """Distance from ideal standoff (0 = flush against border or group)."""
    if isinstance(placed, Geometry):
        border_err = abs(outline_standoff_distance(placed, sheet) - min_dist)
    else:
        border_err = abs(float(placed.distance(sheet.exterior)) - min_dist)
    
    if focal_shape is not None:
        is_empty = False
        if not isinstance(focal_shape, Geometry):
            is_empty = focal_shape.is_empty
        if not is_empty:
            if isinstance(placed, Geometry):
                focal_geom = (
                    focal_shape
                    if isinstance(focal_shape, Geometry)
                    else Geometry.from_shapely(focal_shape)
                )
                group_err = abs(placed.distance(focal_geom) - min_dist)
            else:
                group_err = abs(float(focal_shape.distance(placed)) - min_dist)
            return border_err + group_err
    return border_err

