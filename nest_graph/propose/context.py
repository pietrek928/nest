from dataclasses import dataclass
from math import hypot, pi
from statistics import median
from typing import NamedTuple, Optional, Sequence

from shapely import LineString, MultiLineString, MultiPolygon, Point, Polygon
from shapely.geometry.base import BaseGeometry
from shapely.ops import unary_union

from nest_graph.board import board_context_from_geometry, sheet_hole_polygons
from nest_graph.config import ProposeConfig, RankingMode
from nest_graph.geometry import Geometry, find_polygon_distances_bipartite
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


def iter_polygons(geom: BaseGeometry) -> list[Polygon]:
    """Flatten Polygon / MultiPolygon / GeometryCollection into polygon parts."""
    if geom is None or geom.is_empty:
        return []
    if isinstance(geom, Polygon):
        return [geom]
    if isinstance(geom, MultiPolygon):
        return [g for g in geom.geoms if isinstance(g, Polygon) and not g.is_empty]
    if hasattr(geom, "geoms"):
        out: list[Polygon] = []
        for g in geom.geoms:
            out.extend(iter_polygons(g))
        return out
    return []


def _polygon_components(geom: BaseGeometry) -> list[Polygon]:
    return iter_polygons(geom)


def free_space_targets(
    free: BaseGeometry,
    min_dist: float,
) -> list[Point]:
    """One polylabel (or representative) point per free polygon component."""
    from nest_graph.propose.void_topology import polylabel as void_polylabel

    tol = max(float(min_dist), 1e-3)
    targets: list[Point] = []
    for poly in sorted(_polygon_components(free), key=lambda p: -p.area):
        try:
            targets.append(Point(void_polylabel(poly, tolerance=tol)))
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


@dataclass(frozen=True)
class FreeSpaceSnapshot:
    """Cached free-space geometry for one packed layout (analyze + pockets)."""

    analysis: FreeSpaceAnalysis
    trapped_voids: tuple[Polygon, ...] = ()
    hull_bays: tuple[Polygon, ...] = ()
    topology_poles: tuple[tuple[float, float, float], ...] = ()

    @property
    def max_void_area(self) -> float:
        return float(self.analysis.largest_area)


def analyze_free_space(
    sheet: Polygon,
    packed: Sequence[BaseGeometry],
    part_area: float,
    min_dist: float,
    *,
    void_ratio_threshold: float = 2.5,
    free: BaseGeometry | None = None,
) -> FreeSpaceAnalysis:
    """Classify free space as one large void vs Swiss-cheese slivers.

    ``void_ratio_threshold`` should match ``ProposeConfig.late_border_void_override_ratio``
    so Mode A / late-sat / MIS agree on what counts as large_void.
    Pass ``free`` when the caller already computed ``sheet.difference(union(packed))``.
    """
    from nest_graph.propose.void_topology import polylabel as void_polylabel

    placed = [p for p in packed if p is not None and not p.is_empty]
    if sheet is None or sheet.is_empty:
        return FreeSpaceAnalysis(kind="full", max_void_ratio=0.0)
    if not placed:
        area = float(sheet.area)
        ratio = area / max(float(part_area), 1e-12)
        try:
            pt = Point(void_polylabel(sheet, tolerance=max(float(min_dist), 1e-3)))
        except Exception:
            pt = sheet.representative_point()
        return FreeSpaceAnalysis(
            kind="large_void" if ratio > void_ratio_threshold else "swiss_cheese",
            max_void_ratio=ratio,
            largest_area=area,
            target_poly=sheet if isinstance(sheet, Polygon) else None,
            target_pt=pt,
        )
    if free is None:
        free = sheet.difference(unary_union(placed))
    components = _polygon_components(free)
    if not components:
        return FreeSpaceAnalysis(kind="full", max_void_ratio=0.0)
    largest = max(components, key=lambda p: float(p.area))
    ratio = float(largest.area) / max(float(part_area), 1e-12)
    if ratio > void_ratio_threshold:
        try:
            pt = Point(void_polylabel(largest, tolerance=max(float(min_dist), 1e-3)))
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


def build_free_space_snapshot(
    sheet: Polygon,
    packed: Sequence[BaseGeometry],
    part_area: float,
    min_dist: float,
    *,
    void_ratio_threshold: float = 2.5,
    max_topology_anchors: int = 6,
    pack_geoms: Sequence[Geometry] | None = None,
) -> FreeSpaceSnapshot:
    """Build analyze + trapped voids + hull bays + topology poles once per pack.

    Pass ``pack_geoms`` (e.g. from ``NestState.native_geoms``) to union via
    ``geoms_to_shapely_union`` instead of ``unary_union`` on Shapely packed.
    """
    from nest_graph.geometry_util import geoms_to_shapely_union
    from nest_graph.propose.void_topology import (
        hull_bay_polygons,
        topology_pocket_poles,
        trapped_void_polygons,
    )

    placed = [p for p in packed if p is not None and not p.is_empty]
    free = None
    if sheet is not None and not sheet.is_empty:
        try:
            if pack_geoms is not None:
                geoms = [
                    g for g in pack_geoms
                    if g is not None and not getattr(g, "is_empty", False)
                ]
                if geoms:
                    free = sheet.difference(geoms_to_shapely_union(list(geoms)))
            elif placed:
                free = sheet.difference(unary_union(placed))
        except Exception:
            free = None
    analysis = analyze_free_space(
        sheet,
        packed,
        part_area,
        min_dist,
        void_ratio_threshold=void_ratio_threshold,
        free=free,
    )
    voids = tuple(trapped_void_polygons(sheet, packed, free=free))
    bays = tuple(hull_bay_polygons(packed, min_dist=min_dist, sheet=sheet))
    poles = tuple(
        topology_pocket_poles(
            sheet,
            packed,
            min_dist=min_dist,
            max_anchors=max_topology_anchors,
            voids=voids,
            bays=bays,
        )
    )
    return FreeSpaceSnapshot(
        analysis=analysis,
        trapped_voids=voids,
        hull_bays=bays,
        topology_poles=poles,
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


def cluster_contact_components(
    geoms: Sequence[Geometry],
    gap: float,
    *,
    board_ring: Geometry | None = None,
    board_gap: float | None = None,
) -> list[tuple[list[int], bool]]:
    """Connected components where ``dist(i,j) <= 2*gap``; optional board adjacency.

    ``2*gap`` matches the old ``buffer(gap).intersects(buffer(gap))`` merge.
    Returns ``(member_indices, board_adj)``. Uses Geometry distances (no buffer).
    ``board_gap`` is a raw distance threshold to ``board_ring`` (defaults to
    ``2*gap``). Pass ``min_dist + 2*gap`` to match exterior.buffer(min_dist)
    sentinel plus mutual gap buffers.
    """
    n = len(geoms)
    if n == 0:
        return []
    parent = list(range(n))

    def find(i: int) -> int:
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    def union(a: int, b: int) -> None:
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[rb] = ra

    gap = float(gap)
    contact = 2.0 * gap
    if n >= 2:
        results = find_polygon_distances_bipartite(
            list(geoms), list(geoms), aura=max(contact, 0.5) * 2.0,
        )
        for r in results:
            i, j = int(r.polyA_idx), int(r.polyB_idx)
            if i >= j:
                continue
            if r.intersect or (r.distance_sq ** 0.5) <= contact + 1e-9:
                union(i, j)
    board_touch = [False] * n
    if board_ring is not None:
        bgap = float(board_gap) if board_gap is not None else contact
        br = find_polygon_distances_bipartite(
            list(geoms), [board_ring], aura=max(bgap, 0.5) * 2.0,
        )
        for r in br:
            i = int(r.polyA_idx)
            if 0 <= i < n and (
                r.intersect or (r.distance_sq ** 0.5) <= bgap + 1e-9
            ):
                board_touch[i] = True
    groups: dict[int, list[int]] = {}
    for i in range(n):
        groups.setdefault(find(i), []).append(i)
    out: list[tuple[list[int], bool]] = []
    for members in groups.values():
        out.append((sorted(members), any(board_touch[i] for i in members)))
    return out


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

    parts = [polys[i] for i in placed_idx]
    gap = _cluster_merge_gap(parts, min_dist, sheet)
    geoms = [
        p if isinstance(p, Geometry) else Geometry.from_shapely(p)
        for p in parts
    ]
    comps = cluster_contact_components(geoms, gap)
    return [[placed_idx[j] for j in members] for members, _ in comps]


def cluster_contact_neighbors(
    component: Sequence[int],
    polys: Sequence[BaseGeometry],
    gap: float,
) -> dict[int, list[int]]:
    """Adjacency at dist ≤ 2*gap (same policy as cluster_contact_components)."""
    from nest_graph.propose.placement_common import as_geometry

    ids = list(component)
    adj: dict[int, list[int]] = {i: [] for i in ids}
    if len(ids) < 2:
        return adj
    geoms_ids: list[int] = []
    geoms: list[Geometry] = []
    for i in ids:
        g = as_geometry(polys[i])
        if g is None:
            continue
        geoms_ids.append(i)
        geoms.append(g)
    if len(geoms) < 2:
        return adj
    contact = 2.0 * float(gap)
    results = find_polygon_distances_bipartite(
        geoms, geoms, aura=max(contact, 0.5) * 2.0,
    )
    for r in results:
        a, b = int(r.polyA_idx), int(r.polyB_idx)
        if a >= b:
            continue
        if r.intersect or (r.distance_sq ** 0.5) <= contact + 1e-9:
            ia, ib = geoms_ids[a], geoms_ids[b]
            adj[ia].append(ib)
            adj[ib].append(ia)
    return adj


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
) -> RankingMode:
    if (
        propose_cfg.border_focus_ranking
        and should_use_border_focus(base_shape, propose_cfg)
    ):
        base_mode = RankingMode.BORDER
    elif (
        propose_cfg.use_contact_ranking
        and base_shape is not None
        and not base_shape.is_empty
    ):
        if propose_cfg.use_contact_clearance_hybrid:
            base_mode = RankingMode.CONTACT_HYBRID
        else:
            base_mode = RankingMode.CONTACT
    else:
        base_mode = RankingMode(propose_cfg.ranking_mode)
    if (
        propose_cfg.use_rule_ranking
        and rules is not None
        and rules.size() > 0
        and base_mode in (
            RankingMode.BORDER,
            RankingMode.CONTACT,
            RankingMode.CONTACT_HYBRID,
            RankingMode.RULE_HYBRID,
        )
    ):
        return RankingMode.RULE_HYBRID
    return base_mode


def _exterior_sample_points(
    sheet: Polygon,
    n_samples: int,
) -> list[tuple[float, float]]:
    """Arc-length samples along sheet exterior (open ring, no duplicate close)."""
    coords = list(sheet.exterior.coords)
    if len(coords) < 2:
        return []
    seg_lens: list[float] = []
    total = 0.0
    for i in range(len(coords) - 1):
        dx = coords[i + 1][0] - coords[i][0]
        dy = coords[i + 1][1] - coords[i][1]
        L = (dx * dx + dy * dy) ** 0.5
        seg_lens.append(L)
        total += L
    if total <= 1e-12:
        return []
    n = max(int(n_samples), 8)
    out: list[tuple[float, float]] = []
    for k in range(n):
        target = (k + 0.5) / n * total
        acc = 0.0
        x = float(coords[0][0])
        y = float(coords[0][1])
        for i, L in enumerate(seg_lens):
            if acc + L >= target or i == len(seg_lens) - 1:
                t = 0.0 if L < 1e-12 else (target - acc) / L
                t = min(max(t, 0.0), 1.0)
                x0, y0 = coords[i]
                x1, y1 = coords[i + 1]
                x = float(x0 + t * (x1 - x0))
                y = float(y0 + t * (y1 - y0))
                break
            acc += L
        out.append((x, y))
    return out


def outline_coverage_ratio(
    placed: Sequence[BaseGeometry],
    sheet: Polygon,
    min_dist: float,
    *,
    n_samples: int = 96,
    pack_geoms: Sequence[Geometry] | None = None,
) -> float:
    """Fraction of exterior samples within ``tol`` of the pack (Geometry distance).

    Pass ``pack_geoms`` (e.g. from ``NestState.native_geoms``) to skip
    ``from_shapely`` when native Geometry for the pack is already available.
    """
    if sheet.is_empty or (not placed and not pack_geoms):
        return 0.0
    perimeter = float(sheet.exterior.length)
    if perimeter <= 0.0:
        return 0.0
    tol = max(min_dist * 2.0, 1e-4)
    if pack_geoms is None:
        pack_geoms = [
            p if isinstance(p, Geometry) else Geometry.from_shapely(p)
            for p in placed
            if p is not None and not getattr(p, "is_empty", False)
        ]
    else:
        pack_geoms = [
            g for g in pack_geoms
            if g is not None and not getattr(g, "is_empty", False)
        ]
    if not pack_geoms:
        return 0.0
    samples = _exterior_sample_points(sheet, n_samples)
    if not samples:
        return 0.0
    # One probe ring at origin; translate per sample (no per-sample from_ring).
    e = 1e-6
    probe0 = Geometry.from_ring([(0.0, 0.0), (e, 0.0), (0.0, e)])
    covered = 0
    for x, y in samples:
        dmin = float("inf")
        probe = probe0.translate(float(x), float(y))
        for g in pack_geoms:
            if g.contains_point(x, y):
                dmin = 0.0
                break
            dmin = min(dmin, float(g.distance(probe)))
        if dmin <= tol + 1e-12:
            covered += 1
    return float(covered / len(samples))


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
            annulus = _free_is_annulus(
                primary_free,
                sheet,
                holes,
                min_dist=min_dist,
                part_min=part_min,
                part_max=part_max,
            )
            # Annulus rim docking only when the pack already owns the exterior.
            if annulus and packed_near_border:
                return PlaceZoneInfo(
                    zone="border_gap",
                    free_ratio=free_ratio,
                    n_clusters=len(sig_clusters),
                    outline_coverage=coverage,
                    primary_target=primary_target,
                    is_annulus=True,
                )
            # Defer void_seek while parts are already on the exterior (rim / corridor
            # ends): prefer border_gap / cluster_edge docking first — unless the
            # free component is large enough that OOS-1 overrides rim deferral.
            void_ratio = float(primary_free.area) / part_area
            override = float(propose_cfg.late_border_void_override_ratio)
            if (not packed_near_border) or (override > 0.0 and void_ratio > override):
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
            annulus = _free_is_annulus(
                primary_free,
                sheet,
                holes,
                min_dist=min_dist,
                part_min=part_min,
                part_max=part_max,
            )
            # OOS-1: exterior-touching large free → native void_seek (even mid-rim).
            # Keep annulus rim on border_gap so inward shooters stay enabled.
            void_ratio = float(primary_free.area) / part_area
            override = float(propose_cfg.late_border_void_override_ratio)
            if (
                not annulus
                and override > 0.0
                and void_ratio > override
            ):
                return PlaceZoneInfo(
                    zone="void_seek",
                    free_ratio=free_ratio,
                    n_clusters=len(sig_clusters),
                    outline_coverage=coverage,
                    primary_target=primary_target,
                )
            zone = "border_gap"
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
    # Generalize: scale candidate_pool / max_proposals by mean of active scales,
    # and per-emitter sample budgets where named.
    scales = [float(v) for k, v in pool_scales.items() if not str(k).startswith("_")]
    if scales:
        mean_s = sum(scales) / len(scales)
        mean_s = min(2.0, max(0.05, mean_s))
        data["candidate_pool"] = max(8, int(round(propose_cfg.candidate_pool * mean_s)))
        data["max_proposals"] = max(4, int(round(propose_cfg.max_proposals * mean_s)))
    gf = float(pool_scales.get("group_fit", 1.0))
    if gf != 1.0:
        data["group_edge_samples_per_edge"] = max(
            2, int(round(propose_cfg.group_edge_samples_per_edge * min(2.0, max(0.05, gf)))),
        )
    rc = float(pool_scales.get("raycasting", 1.0))
    if rc != 1.0:
        data["raycast_num_rays"] = max(
            4, int(round(propose_cfg.raycast_num_rays * min(2.0, max(0.05, rc)))),
        )
    pf = float(pool_scales.get("pocket_fit", 1.0))
    if pf != 1.0:
        data["pocket_fit_max_targets"] = max(
            1, int(round(propose_cfg.pocket_fit_max_targets * min(2.0, max(0.05, pf)))),
        )
    return ProposeConfig(**data)


def placement_contact_error(
    placed: BaseGeometry,
    sheet: Polygon,
    min_dist: float,
    focal_shape: Optional[BaseGeometry] = None,
) -> float:
    """Distance from ideal standoff (0 = flush against border or group).

    When ``placed`` (and optionally ``focal_shape``) are Geometry, uses native
    standoff / ``Geometry.distance`` rather than Shapely outline distance.
    """
    border_err = abs(outline_standoff_distance(placed, sheet) - min_dist)

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


class LateBorderSatInfo(NamedTuple):
    active: bool
    outline_cov: float
    sat_override: bool
    rim_progress: float
    free_kind: str


def late_border_saturation_info(
    cfg,
    nest_state,
    board: BaseGeometry,
    *,
    had_void_override: bool = False,
) -> LateBorderSatInfo:
    """Whether late border-only propose should run for this pack.

    ``cfg`` is a ``BuildGraphConfig`` and ``nest_state`` a ``NestState``; both
    are duck-typed so this stays on the propose side of the import graph.

    Exp1: skip sat when free space is a large contiguous void.
    Exp3: when not large_void, also require pack-hull rim progress below threshold.
    """
    empty = LateBorderSatInfo(
        active=False, outline_cov=0.0, sat_override=False,
        rim_progress=0.0, free_kind="",
    )
    if nest_state is None or not cfg.propose.late_border_saturation:
        return empty
    placed = [
        nest_state.polys[i]
        for i in nest_state.selected_indices
        if nest_state.polys[i] is not None and not nest_state.polys[i].is_empty
    ]
    if not placed:
        return empty
    sheet, _ = board_context_from_geometry(board)
    min_dist = cfg.board_min_dist()
    native = nest_state.native_geoms
    pack_geoms = [
        native[i]
        for i in nest_state.selected_indices
        if 0 <= i < len(native)
        and nest_state.polys[i] is not None
        and not nest_state.polys[i].is_empty
    ]
    outline_cov = float(
        outline_coverage_ratio(placed, sheet, min_dist, pack_geoms=pack_geoms)
    )
    threshold = float(cfg.propose.place_border_coverage_threshold)

    mean_part = sum(float(p.area) for p in placed) / float(len(placed))
    void_thr = float(cfg.propose.late_border_void_override_ratio)
    if void_thr <= 0.0:
        void_thr = 2.5
    free_info = analyze_free_space(
        sheet, placed, mean_part, min_dist, void_ratio_threshold=void_thr,
    )
    free_kind = str(free_info.kind)
    void_ratio = float(free_info.max_void_ratio)
    override_ratio = float(cfg.propose.late_border_void_override_ratio)
    release_ratio = float(cfg.propose.late_border_void_release_ratio)
    entry_override = (
        override_ratio > 0.0
        and free_info.kind == "large_void"
        and void_ratio > override_ratio
    )
    # Hysteresis: once unlocked this pack, hold while void still meaningful.
    hold_override = (
        had_void_override
        and release_ratio > 0.0
        and outline_cov < threshold
        and void_ratio > release_ratio
    )
    sat_override = entry_override or hold_override

    sheet_perim = float(sheet.exterior.length) if not sheet.is_empty else 0.0
    rim_progress = 0.0
    if sheet_perim > 1e-12:
        hull = unary_union(placed).convex_hull
        if hull is not None and not hull.is_empty and hasattr(hull, "exterior"):
            rim_progress = min(
                max(float(hull.exterior.length) / sheet_perim, 0.0), 1.0,
            )

    if outline_cov >= threshold:
        # Rim metric already satisfied — no late border-only.
        return LateBorderSatInfo(
            active=False,
            outline_cov=outline_cov,
            sat_override=False,
            rim_progress=rim_progress,
            free_kind=free_kind,
        )
    if sat_override:
        # Exp1 / hysteresis: unlock full propose despite low outline_kiss_cov.
        return LateBorderSatInfo(
            active=False,
            outline_cov=outline_cov,
            sat_override=True,
            rim_progress=rim_progress,
            free_kind=free_kind,
        )
    # Without large_void, keep sat only while pack hull_rim_fill is still open.
    hull_thr = float(cfg.propose.late_border_hull_threshold)
    if hull_thr > 0.0 and rim_progress >= hull_thr:
        return LateBorderSatInfo(
            active=False,
            outline_cov=outline_cov,
            sat_override=False,
            rim_progress=rim_progress,
            free_kind=free_kind,
        )
    return LateBorderSatInfo(
        active=True,
        outline_cov=outline_cov,
        sat_override=False,
        rim_progress=rim_progress,
        free_kind=free_kind,
    )


def late_border_saturation_active(
    cfg,
    nest_state,
    board: BaseGeometry,
    *,
    had_void_override: bool = False,
) -> bool:
    return late_border_saturation_info(
        cfg, nest_state, board, had_void_override=had_void_override,
    ).active
