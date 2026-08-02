from dataclasses import dataclass
from math import hypot, pi
from statistics import median
from typing import Optional, Sequence

from shapely import LineString, MultiLineString, MultiPolygon, Point, Polygon
from shapely.geometry.base import BaseGeometry
from shapely.ops import polylabel, unary_union

from nest_graph.board import board_context_from_geometry, sheet_hole_polygons
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


def _polygon_components(geom: BaseGeometry) -> list[Polygon]:
    if geom is None or geom.is_empty:
        return []
    if isinstance(geom, Polygon):
        return [geom]
    if isinstance(geom, MultiPolygon):
        return [g for g in geom.geoms if isinstance(g, Polygon) and not g.is_empty]
    if hasattr(geom, "geoms"):
        out: list[Polygon] = []
        for g in geom.geoms:
            out.extend(_polygon_components(g))
        return out
    return []


def free_space_targets(
    free: BaseGeometry,
    min_dist: float,
) -> list[Point]:
    """One polylabel (or representative) point per free polygon component."""
    tol = max(float(min_dist), 1e-3)
    targets: list[Point] = []
    for poly in sorted(_polygon_components(free), key=lambda p: -p.area):
        try:
            targets.append(Point(polylabel(poly, tolerance=tol)))
        except Exception:
            targets.append(poly.representative_point())
    return targets


@dataclass(frozen=True)
class FreeSpaceAnalysis:
    """Topological free-space summary for mid-pack routing."""

    kind: str  # "large_void" | "swiss_cheese" | "full"
    max_void_ratio: float
    largest_area: float = 0.0
    target_poly: Polygon | None = None
    target_pt: Point | None = None


def analyze_free_space(
    sheet: Polygon,
    packed: Sequence[BaseGeometry],
    part_area: float,
    min_dist: float,
    *,
    void_ratio_threshold: float = 2.5,
) -> FreeSpaceAnalysis:
    """Classify free space as one large void vs Swiss-cheese slivers."""
    placed = [p for p in packed if p is not None and not p.is_empty]
    if sheet is None or sheet.is_empty:
        return FreeSpaceAnalysis(kind="full", max_void_ratio=0.0)
    if not placed:
        area = float(sheet.area)
        ratio = area / max(float(part_area), 1e-12)
        try:
            pt = Point(polylabel(sheet, tolerance=max(float(min_dist), 1e-3)))
        except Exception:
            pt = sheet.representative_point()
        return FreeSpaceAnalysis(
            kind="large_void" if ratio > void_ratio_threshold else "swiss_cheese",
            max_void_ratio=ratio,
            largest_area=area,
            target_poly=sheet if isinstance(sheet, Polygon) else None,
            target_pt=pt,
        )
    free = sheet.difference(unary_union(placed))
    components = _polygon_components(free)
    if not components:
        return FreeSpaceAnalysis(kind="full", max_void_ratio=0.0)
    largest = max(components, key=lambda p: float(p.area))
    ratio = float(largest.area) / max(float(part_area), 1e-12)
    if ratio > void_ratio_threshold:
        try:
            pt = Point(polylabel(largest, tolerance=max(float(min_dist), 1e-3)))
        except Exception:
            pt = largest.representative_point()
        return FreeSpaceAnalysis(
            kind="large_void",
            max_void_ratio=ratio,
            largest_area=float(largest.area),
            target_poly=largest,
            target_pt=pt,
        )
    return FreeSpaceAnalysis(
        kind="swiss_cheese",
        max_void_ratio=ratio,
        largest_area=float(largest.area),
        target_poly=largest,
        target_pt=largest.representative_point(),
    )


def void_pole_seed_coords(
    target_pt: Point,
    *,
    num_angles: int = 4,
) -> list[tuple[float, float, float]]:
    """Multi-angle (x, y, θ) seeds at a free-space pole of inaccessibility."""
    return corridor_seed_coords_from_samples([target_pt], num_angles=num_angles)


def part_extents(part_poly: Polygon) -> tuple[float, float]:
    """Return (min_extent, max_extent) of a catalog part bbox."""
    minx, miny, maxx, maxy = part_poly.bounds
    w = max(maxx - minx, 1e-9)
    h = max(maxy - miny, 1e-9)
    return min(w, h), max(w, h)


def part_hull_fill_ratio(part_poly: Polygon) -> float:
    """Part area / convex-hull area; concave parts fall below ~0.9."""
    if part_poly is None or part_poly.is_empty:
        return 1.0
    hull = part_poly.convex_hull
    if hull is None or hull.is_empty or float(hull.area) <= 1e-12:
        return 1.0
    return float(part_poly.area / hull.area)


def part_is_concave(part_poly: Polygon, *, fill_threshold: float = 0.9) -> bool:
    return part_hull_fill_ratio(part_poly) < fill_threshold


def _closest_hole_pair(
    holes: Sequence[Polygon],
) -> tuple[Polygon, Polygon, float] | None:
    if len(holes) < 2:
        return None
    best_i = best_j = -1
    best_d: float | None = None
    for i in range(len(holes)):
        for j in range(i + 1, len(holes)):
            gap = float(holes[i].distance(holes[j]))
            if gap <= 0.0:
                continue
            if best_d is None or gap < best_d:
                best_d = gap
                best_i, best_j = i, j
    if best_i < 0 or best_d is None:
        return None
    return holes[best_i], holes[best_j], best_d


def _narrow_multi_hole_corridor(
    holes: Sequence[Polygon],
    part_min: float,
    min_dist: float,
) -> bool:
    """True when ≥2 sheet holes leave a narrow channel between them."""
    pair = _closest_hole_pair(holes)
    if pair is None:
        return False
    _h1, _h2, best = pair
    limit = max(3.0 * part_min, 12.0 * min_dist)
    return best <= limit


def sheet_has_narrow_corridor(
    sheet: Polygon,
    part_poly: Polygon,
    min_dist: float,
) -> bool:
    """Public corridor check for empty-sheet place routing."""
    part_min, _ = part_extents(part_poly)
    return _narrow_multi_hole_corridor(
        sheet_hole_polygons(sheet), part_min, min_dist,
    )


def corridor_channel_target(
    sheet: Polygon,
    min_dist: float,
) -> Point | None:
    """Midpoint between the closest pair of sheet holes (channel focus)."""
    pair = _closest_hole_pair(sheet_hole_polygons(sheet))
    if pair is None:
        return None
    h1, h2, _gap = pair
    ci = h1.centroid
    cj = h2.centroid
    mid = Point(0.5 * (ci.x + cj.x), 0.5 * (ci.y + cj.y))
    if sheet.contains(mid) or float(mid.distance(sheet)) <= min_dist:
        return mid
    return sheet.representative_point()


def corridor_channel_samples(
    sheet: Polygon,
    min_dist: float,
    n: int = 12,
) -> list[Point]:
    """Evenly spaced points along the corridor axis (perpendicular to hole-pair)."""
    del min_dist  # reserved for future free-region clamping
    if n <= 0:
        return []
    pair = _closest_hole_pair(sheet_hole_polygons(sheet))
    if pair is None:
        return []
    h1, h2, _gap = pair
    c1, c2 = h1.centroid, h2.centroid
    mid = Point(0.5 * (c1.x + c2.x), 0.5 * (c1.y + c2.y))
    dx, dy = c2.x - c1.x, c2.y - c1.y
    length = hypot(dx, dy)
    if length < 1e-6:
        return [mid]
    # Perpendicular to the hole-pair segment = channel longitudinal axis.
    px, py = -dy / length, dx / length
    minx, miny, maxx, maxy = sheet.bounds
    diag = hypot(maxx - minx, maxy - miny)
    if diag <= 0.0:
        return [mid]
    axis_line = LineString([
        (mid.x + px * diag, mid.y + py * diag),
        (mid.x - px * diag, mid.y - py * diag),
    ])
    valid_axis = axis_line.intersection(sheet)
    if valid_axis is None or valid_axis.is_empty:
        return [mid]
    if isinstance(valid_axis, MultiLineString):
        geoms = [g for g in valid_axis.geoms if not g.is_empty and g.length > 0]
        if not geoms:
            return [mid]
        valid_axis = max(geoms, key=lambda g: g.length)
    if not isinstance(valid_axis, LineString) or valid_axis.length <= 0:
        return [mid]
    samples: list[Point] = []
    for i in range(n):
        frac = (i + 0.5) / n
        samples.append(valid_axis.interpolate(frac, normalized=True))
    return samples


def corridor_seed_coords_from_samples(
    samples: Sequence[Point],
    *,
    num_angles: int = 4,
) -> list[tuple[float, float, float]]:
    """(x, y, θ) seeds along the channel for candidate injection."""
    if not samples:
        return []
    n_ang = max(1, min(int(num_angles), 4))
    angles = [2.0 * pi * i / n_ang for i in range(n_ang)]
    out: list[tuple[float, float, float]] = []
    for pt in samples:
        for ang in angles:
            out.append((float(pt.x), float(pt.y), float(ang)))
    return out


def nearest_channel_attract(
    samples: Sequence[Point],
    *,
    channel_mid: Point | None,
    packed: Sequence[BaseGeometry] | None,
) -> Point | None:
    """Prefer the channel sample nearest the last packed centroid; else mid."""
    if packed:
        last = packed[-1]
        if last is not None and not last.is_empty:
            ref = last.centroid
            if samples:
                return min(samples, key=lambda p: float(p.distance(ref)))
            return channel_mid
    if channel_mid is not None:
        return channel_mid
    return samples[0] if samples else None


def _free_is_annulus(
    free_poly: Polygon,
    sheet: Polygon,
    holes: Sequence[Polygon],
    *,
    min_dist: float,
    part_min: float,
    part_max: float,
) -> bool:
    """Free space touches both the exterior ring and a sheet-hole band."""
    if free_poly is None or free_poly.is_empty or not holes:
        return False
    ring_w = max(2.0 * min_dist, 0.5 * part_min)
    exterior_ring = sheet.exterior.buffer(ring_w)
    if free_poly.intersection(exterior_ring).is_empty:
        return False
    hole_band = unary_union(list(holes)).buffer(max(2.0 * part_max, min_dist * 8.0))
    return not free_poly.intersection(hole_band).is_empty


def _cluster_merge_gap(
    polys: Sequence[BaseGeometry],
    min_dist: float,
    sheet: Polygon | None,
) -> float:
    gap = max(min_dist * 0.5, 1e-6)
    extents: list[float] = []
    for p in polys:
        if p is None or p.is_empty:
            continue
        minx, miny, maxx, maxy = p.bounds
        extents.append(max(maxx - minx, maxy - miny))
    if extents:
        gap = max(gap, 0.35 * float(median(extents)))
    if sheet is not None and not sheet.is_empty:
        minx, miny, maxx, maxy = sheet.bounds
        characteristic = min(maxx - minx, maxy - miny)
        if characteristic > 0.0:
            gap = min(gap, 0.15 * characteristic)
    return gap


def cluster_packed_solid_groups(
    polys: Sequence[BaseGeometry],
    min_dist: float,
    *,
    sheet: Polygon | None = None,
) -> list[BaseGeometry]:
    """Connected clusters of packed parts (touching within clearance gap)."""
    index_groups = cluster_packed_indices(polys, min_dist, sheet=sheet)
    groups: list[BaseGeometry] = []
    for idxs in index_groups:
        members = [polys[i] for i in idxs]
        groups.append(unary_union(members) if len(members) > 1 else members[0])
    return groups


def cluster_packed_indices(
    polys: Sequence[BaseGeometry],
    min_dist: float,
    *,
    sheet: Polygon | None = None,
) -> list[list[int]]:
    """Indices of packed parts grouped into contact-connected clusters."""
    placed_idx = [i for i, p in enumerate(polys) if p is not None and not p.is_empty]
    if not placed_idx:
        return []
    if len(placed_idx) == 1:
        return [placed_idx]

    gap = _cluster_merge_gap([polys[i] for i in placed_idx], min_dist, sheet)
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


def significant_cluster_groups(
    groups: Sequence[BaseGeometry],
    part_area: float,
) -> list[BaseGeometry]:
    """Drop tiny noise islands before inter_cluster decisions."""
    floor = max(part_area * 0.25, 1e-9)
    return [g for g in groups if not g.is_empty and float(g.area) >= floor]


def border_solid_focal(sheet: Polygon, min_dist: float) -> BaseGeometry:
    """Thin ring along the nestable sheet outer boundary for edge-fitting proposals."""
    inset = max(min_dist * 2.0, 1e-4)
    inner = sheet.buffer(-inset)
    if inner.is_empty or inner.area < sheet.area * 1e-6:
        return sheet
    ring = sheet.difference(inner)
    return ring if not ring.is_empty else sheet


def _propose_obstacle_ref_point(
    placed: Sequence[BaseGeometry],
    min_dist: float,
    *,
    sheet: Polygon | None,
    ref_point: Point | None,
) -> Point:
    if ref_point is not None:
        return ref_point
    if sheet is not None and not sheet.is_empty:
        free = placement_free_region(sheet, unary_union(list(placed)), min_dist)
        targets = free_space_targets(free, min_dist)
        if targets:
            return targets[0]
        return sheet.centroid
    return Point(0.0, 0.0)


def obstacle_polys_for_propose(
    placed: Sequence[BaseGeometry],
    part_poly: Polygon,
    min_dist: float,
    *,
    nearest_k: int | None = None,
    propose_cfg: ProposeConfig | None = None,
    sheet: Polygon | None = None,
    ref_point: Point | None = None,
) -> list[BaseGeometry]:
    """Parts in the k nearest packed clusters; graph still checks the full layout.

    When the pack is a single contact blob (or fewer clusters than k), fall back to
    the k nearest individual parts so local proposers are not forced onto the full layout.
    """
    if not placed:
        return []
    k_val = nearest_k
    if k_val is None and propose_cfg is not None:
        k_val = propose_cfg.obstacle_nearest_k
    if k_val is None:
        k_val = 2
    valid = [p for p in placed if p is not None and not p.is_empty]
    if not valid:
        return []
    groups = cluster_packed_solid_groups(valid, min_dist, sheet=sheet)
    if not groups:
        return []
    ref = _propose_obstacle_ref_point(
        valid, min_dist, sheet=sheet, ref_point=ref_point,
    )
    if len(groups) == 1 or len(groups) < max(1, k_val):
        k = max(1, min(k_val, len(valid)))
        ranked = sorted(valid, key=lambda p: p.distance(ref))[:k]
        return list(ranked)
    k = max(1, min(k_val, len(groups)))
    ranked = sorted(groups, key=lambda g: g.distance(ref))[:k]
    obstacle = unary_union(ranked)
    return [p for p in valid if p.intersects(obstacle)]


def obstacle_shape_for_propose(
    placed: Sequence[BaseGeometry],
    part_poly: Polygon,
    min_dist: float,
    *,
    propose_cfg: ProposeConfig | None = None,
    sheet: Polygon | None = None,
    ref_point: Point | None = None,
) -> BaseGeometry:
    polys = obstacle_polys_for_propose(
        placed,
        part_poly,
        min_dist,
        propose_cfg=propose_cfg,
        sheet=sheet,
        ref_point=ref_point,
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
    # Skip hull when it would fill a rim/donut interior (area blows up).
    if len(polys) >= 12:
        hull = merged.convex_hull
        if (
            not hull.is_empty
            and float(merged.area) > 0.0
            and float(hull.area) <= float(merged.area) * 1.5
        ):
            return hull
    return merged


def focal_shape_for_propose(
    board: BaseGeometry,
    placed: Sequence[BaseGeometry],
    part_poly: Polygon,
    min_dist: float,
    propose_cfg: ProposeConfig,
    *,
    sheet: Polygon | None = None,
    ref_point: Point | None = None,
) -> Optional[BaseGeometry]:
    """Focal geometry for ray anchors and group/board edge seeds."""
    obstacle = obstacle_shape_for_propose(
        placed,
        part_poly,
        min_dist,
        propose_cfg=propose_cfg,
        sheet=sheet,
        ref_point=ref_point,
    )
    if obstacle is not None and not obstacle.is_empty:
        return obstacle
    if propose_cfg.use_border_focus:
        return border_focal_for_propose(board, min_dist)
    return None


def local_packed_near_target(
    placed: Sequence[BaseGeometry],
    target: Point,
    part_max_extent: float,
) -> BaseGeometry:
    """Packed subset within 3·part_max of a free-space target (GROUP_FIT cap)."""
    radius = max(3.0 * part_max_extent, 1e-6)
    near = [p for p in placed if p is not None and not p.is_empty and float(p.distance(target)) <= radius]
    if not near:
        return Polygon()
    if len(near) == 1:
        return near[0]
    return unary_union(near)


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
    if perimeter <= 0.0:
        return 0.0
    tol = max(min_dist * 2.0, 1e-4)
    merged = unary_union(placed)
    if merged.is_empty:
        return 0.0
    covered_len = merged.buffer(tol).intersection(sheet.exterior).length
    return float(covered_len / perimeter)


def _hole_mouth_void_seek(
    free_poly: Polygon,
    target: Point,
    holes: Sequence[Polygon],
    *,
    min_dist: float,
    part_max_extent: float,
) -> bool:
    if not holes or free_poly.is_empty:
        return False
    hole_union = unary_union(list(holes))
    if hole_union.is_empty:
        return False
    near_tol = max(min_dist * 8.0, part_max_extent)
    if float(target.distance(hole_union)) < near_tol:
        return True
    band = hole_union.buffer(max(min_dist * 8.0, 2.0 * part_max_extent))
    inter = free_poly.intersection(band)
    if inter.is_empty or free_poly.area <= 0.0:
        return False
    return float(inter.area / free_poly.area) >= 0.12


def _local_interior_pocket(
    free: BaseGeometry,
    packed: BaseGeometry,
    sheet: Polygon,
    *,
    min_dist: float,
    part_min_extent: float,
) -> bool:
    if free.is_empty or packed is None or packed.is_empty:
        return False
    local_buf = max(4.0 * min_dist, part_min_extent)
    local = free.intersection(packed.buffer(local_buf))
    if local.is_empty:
        return False
    ring_w = max(2.0 * min_dist, 0.5 * part_min_extent)
    exterior_ring = sheet.exterior.buffer(ring_w)
    return local.intersection(exterior_ring).is_empty


def _free_width_at_target(free_poly: Polygon, target: Point) -> float:
    if free_poly.is_empty or free_poly.exterior is None:
        return 0.0
    return 2.0 * float(target.distance(free_poly.exterior))


@dataclass(frozen=True)
class PlaceZoneInfo:
    zone: str
    free_ratio: float = 1.0
    n_clusters: int = 0
    outline_coverage: float = 0.0
    primary_target: Point | None = None
    is_annulus: bool = False
    is_corridor: bool = False


def classify_propose_zone_info(
    board: BaseGeometry,
    obstacle_shape: BaseGeometry,
    part_poly: Polygon,
    *,
    min_dist: float,
    propose_cfg: ProposeConfig,
    selected_polys: Sequence[BaseGeometry] | None = None,
    selected_indices: Sequence[int] | None = None,
    user_holes: tuple[tuple[tuple[float, float], ...], ...] = (),
    sheet: Polygon | None = None,
) -> PlaceZoneInfo:
    """Classify propose zone with metrics for obstacle-scope decisions."""
    del selected_indices  # reserved for callers that mirror pipeline selection
    if sheet is None:
        sheet, _ = board_context_from_geometry(board, user_holes=user_holes)
    placed = list(selected_polys or [])
    if not placed or obstacle_shape is None or obstacle_shape.is_empty:
        return PlaceZoneInfo(zone="empty_border")

    free = placement_free_region(sheet, obstacle_shape, min_dist)
    free_ratio = float(free.area / sheet.area) if not sheet.is_empty and sheet.area > 0 else 0.0
    coverage = outline_coverage_ratio(placed, sheet, min_dist)
    clusters = cluster_packed_solid_groups(placed, min_dist, sheet=sheet)
    part_min, part_max = part_extents(part_poly)
    part_area = max(float(part_poly.area), 1e-9)
    sig_clusters = significant_cluster_groups(clusters, part_area)
    components = sorted(_polygon_components(free), key=lambda p: -p.area)
    targets = free_space_targets(free, min_dist)
    primary_target = targets[0] if targets else None
    primary_free = components[0] if components else None

    holes = sheet_hole_polygons(sheet)
    border_touch_tol = max(min_dist * 8.0, 0.75 * part_max, 1e-3)
    packed_near_border = any(
        float(p.distance(sheet.exterior)) <= border_touch_tol for p in placed
    )
    if _narrow_multi_hole_corridor(holes, part_min, min_dist):
        corridor_pt = corridor_channel_target(sheet, min_dist)
        return PlaceZoneInfo(
            zone="cluster_edge",
            free_ratio=free_ratio,
            n_clusters=len(sig_clusters),
            outline_coverage=coverage,
            primary_target=corridor_pt,
            is_corridor=True,
        )
    if primary_free is not None and primary_target is not None:
        hole_mouth = _hole_mouth_void_seek(
            primary_free,
            primary_target,
            holes,
            min_dist=min_dist,
            part_max_extent=part_max,
        )
        if hole_mouth:
            # Narrow free near holes → corridor docking, not clearance void_seek.
            width = _free_width_at_target(primary_free, primary_target)
            if width < 1.5 * part_min:
                return PlaceZoneInfo(
                    zone="cluster_edge",
                    free_ratio=free_ratio,
                    n_clusters=len(sig_clusters),
                    outline_coverage=coverage,
                    primary_target=primary_target,
                )
            # Defer void_seek while parts are already on the exterior (rim / corridor
            # ends): prefer border_gap / cluster_edge docking first.
            if not packed_near_border:
                return PlaceZoneInfo(
                    zone="void_seek",
                    free_ratio=free_ratio,
                    n_clusters=len(sig_clusters),
                    outline_coverage=coverage,
                    primary_target=primary_target,
                )

    if (
        len(sig_clusters) >= 2
        and free_ratio > 0.08
        and not (
            packed_near_border
            and coverage >= propose_cfg.place_border_coverage_threshold
        )
    ):
        minx, miny, maxx, maxy = sheet.bounds
        char = min(maxx - minx, maxy - miny)
        prox = min(4.0 * part_max, 0.35 * char if char > 0 else 4.0 * part_max)
        # Ignore nearly-adjacent seeds (should merge); require a true gap between islands.
        min_half = 2.0 * part_max
        # Closest cluster-pair midpoint (not global free polylabel) so a giant empty
        # free region cannot sample far from both islands.
        best_half: float | None = None
        for i in range(len(sig_clusters)):
            for j in range(i + 1, len(sig_clusters)):
                ci = sig_clusters[i]
                cj = sig_clusters[j]
                if float(free.distance(ci)) > min_dist * 2.0:
                    continue
                if float(free.distance(cj)) > min_dist * 2.0:
                    continue
                pi = ci.centroid
                pj = cj.centroid
                mid = Point(0.5 * (pi.x + pj.x), 0.5 * (pi.y + pj.y))
                if float(mid.distance(free)) > part_max:
                    continue
                half = 0.5 * float(pi.distance(pj))
                if half < min_half:
                    continue
                if best_half is None or half < best_half:
                    best_half = half
        if best_half is not None and best_half <= prox:
            return PlaceZoneInfo(
                zone="inter_cluster",
                free_ratio=free_ratio,
                n_clusters=len(sig_clusters),
                outline_coverage=coverage,
                primary_target=primary_target,
            )

    packed_union = unary_union(placed)
    if _local_interior_pocket(
        free,
        packed_union,
        sheet,
        min_dist=min_dist,
        part_min_extent=part_min,
    ):
        return PlaceZoneInfo(
            zone="interior_pocket",
            free_ratio=free_ratio,
            n_clusters=len(sig_clusters),
            outline_coverage=coverage,
            primary_target=primary_target,
        )

    ring_w = max(2.0 * min_dist, 0.5 * part_min)
    exterior_ring = sheet.exterior.buffer(ring_w)
    touches_exterior = (
        primary_free is not None
        and not primary_free.intersection(exterior_ring).is_empty
    )
    if touches_exterior and primary_free is not None and primary_target is not None:
        width = _free_width_at_target(primary_free, primary_target)
        if width < 1.5 * part_min:
            zone = "cluster_edge"
            annulus = False
        else:
            zone = "border_gap"
            annulus = _free_is_annulus(
                primary_free,
                sheet,
                holes,
                min_dist=min_dist,
                part_min=part_min,
                part_max=part_max,
            )
        return PlaceZoneInfo(
            zone=zone,
            free_ratio=free_ratio,
            n_clusters=len(sig_clusters),
            outline_coverage=coverage,
            primary_target=primary_target,
            is_annulus=annulus,
        )

    return PlaceZoneInfo(
        zone="cluster_edge",
        free_ratio=free_ratio,
        n_clusters=len(sig_clusters),
        outline_coverage=coverage,
        primary_target=primary_target,
    )


def classify_propose_zone(
    board: BaseGeometry,
    obstacle_shape: BaseGeometry,
    part_poly: Polygon,
    *,
    min_dist: float,
    propose_cfg: ProposeConfig,
    selected_polys: Sequence[BaseGeometry] | None = None,
    selected_indices: Sequence[int] | None = None,
    user_holes: tuple[tuple[tuple[float, float], ...], ...] = (),
    sheet: Polygon | None = None,
) -> str:
    """Classify where on the sheet this part should be proposed."""
    return classify_propose_zone_info(
        board,
        obstacle_shape,
        part_poly,
        min_dist=min_dist,
        propose_cfg=propose_cfg,
        selected_polys=selected_polys,
        selected_indices=selected_indices,
        user_holes=user_holes,
        sheet=sheet,
    ).zone


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
