import math
import time
from typing import List, Optional, Sequence, Tuple

import numpy as np
from shapely import LineString, LinearRing, MultiPoint, Point, Polygon
from shapely.geometry.base import BaseGeometry
from shapely.ops import voronoi_diagram

from nest_graph.utils import get_shape_exteriors

from nest_graph.geometry import Geometry
from nest_graph.propose.context import search_region_for_placement
from nest_graph.propose.geometry import (
    ProposeGeometry,
    batch_valid_flags,
    filter_candidates_batch,
)
from nest_graph.propose.placement_perimeter import vertex_anchors_from_geom

def densify_points(geometry, distance):
    """Adds points along the perimeter of the shape for a better Voronoi map."""
    if distance <= 0 or geometry.is_empty:
        return MultiPoint()

    points = []
    for ring in get_shape_exteriors(geometry):
        if ring.length <= 0:
            continue
        line = LineString(ring.coords) if isinstance(ring, LinearRing) else ring
        if line.geom_type != "LineString" or line.length <= 0:
            continue
        for d in np.arange(0, line.length, distance):
            points.append(line.interpolate(d))
        points.append(line.interpolate(line.length))
    return MultiPoint(points) if points else MultiPoint()


def propose_placements_voronoi(
    base_shape: BaseGeometry,
    shape_to_place: Polygon,
    sheet: Polygon,
    min_dist: float,
    *,
    use_free_region: bool = False,
    num_angles: int = 8,
    top_n: int = 3,
    densify_divisor: float = 20.0,
    max_sites: int = 64,
    focal_shape: Optional[BaseGeometry] = None,
    border_focus: bool = False,
    propose_geom: ProposeGeometry,
    pt_push: Point,
) -> List[Tuple[float, float, float]]:
    """
    Proposes placements using Voronoi vertices as candidate centers.
    """
    propositions: list[dict] = []
    region = search_region_for_placement(
        base_shape, sheet, sheet, min_dist,
        use_free_region=use_free_region, border_focus=border_focus,
    )
    if region.is_empty:
        return []

    attract = (
        focal_shape.centroid
        if focal_shape is not None and not focal_shape.is_empty
        else region.centroid
    )
    region_g = propose_geom.region_geometry(region)

    # 1. Densify the layout region and generate Voronoi Diagram
    extent = max(region.bounds[2] - region.bounds[0], region.bounds[3] - region.bounds[1])
    step = max(extent / densify_divisor, 1e-4)
    points = densify_points(region, step)
    if points.is_empty:
        return []

    vor_regions = voronoi_diagram(points)

    candidate_points = []
    for vor_region in vor_regions.geoms:
        rings = get_shape_exteriors(vor_region)
        for ring in rings:
            for vert in ring.coords:
                x, y = float(vert[0]), float(vert[1])
                if region_g is not None:
                    if not region_g.contains_point(x, y):
                        continue
                elif not region.contains(Point(x, y)):
                    continue
                candidate_points.append((x, y))
    if len(candidate_points) > max_sites:
        idx = np.linspace(0, len(candidate_points) - 1, max_sites, dtype=int)
        candidate_points = [candidate_points[i] for i in idx]

    angles = np.linspace(0, 2*np.pi, num_angles, endpoint=False)
    attract_x, attract_y = float(attract.x), float(attract.y)

    raw: list[tuple[float, float, float]] = []
    costs: list[float] = []
    for px, py in candidate_points:
        for angle in angles:
            coords = (float(px), float(py), float(angle))
            raw.append(coords)
            costs.append(math.hypot(px - attract_x, py - attract_y))
    if not raw:
        return []
    valid = set(filter_candidates_batch(propose_geom, raw, pt_push))
    propositions = [
        {"coords": c, "cost": cost}
        for c, cost in zip(raw, costs, strict=True)
        if c in valid
    ]
    propositions.sort(key=lambda x: x["cost"])
    return [p["coords"] for p in propositions[:top_n]]


def propose_placements_raycasting(
    base_shape: BaseGeometry,
    shape_to_place: Polygon,
    sheet: Polygon,
    min_dist: float,
    *,
    use_free_region: bool = False,
    num_rays: int = 12,
    num_angles: int = 8,
    top_n: int = 3,
    anchor_stride: int = 2,
    focal_shape: Optional[BaseGeometry] = None,
    border_focus: bool = False,
    propose_geom: ProposeGeometry,
    pt_push: Point,
    rim_anchor_geoms: Sequence[BaseGeometry | Geometry] | None = None,
) -> List[Tuple[float, float, float]]:
    """
    Proposes placements by casting rays from boundary vertices into the interior.
    """
    region = search_region_for_placement(
        base_shape, sheet, sheet, min_dist,
        use_free_region=use_free_region, border_focus=border_focus,
    )
    if region.is_empty:
        return []

    if focal_shape is not None and not focal_shape.is_empty:
        anchor_source = focal_shape
    elif use_free_region:
        anchor_source = region
    else:
        anchor_source = base_shape if not base_shape.is_empty else region

    attract = (
        focal_shape.centroid
        if focal_shape is not None and not focal_shape.is_empty
        else region.centroid
    )

    attract_x, attract_y = float(attract.x), float(attract.y)

    t_ray0 = time.perf_counter()
    region_g = propose_geom.region_geometry(region)
    if region_g is None:
        return []

    # Prefer native region rings when the anchor source is the search region.
    src_for_harvest: BaseGeometry | Geometry = anchor_source
    if anchor_source is region:
        src_for_harvest = region_g
    anchors_xy, native_n, from_n = vertex_anchors_from_geom(src_for_harvest)
    if rim_anchor_geoms:
        for geom in rim_anchor_geoms:
            if geom is None or getattr(geom, "is_empty", True):
                continue
            extra, n_nat, n_fs = vertex_anchors_from_geom(geom)
            anchors_xy.extend(extra)
            native_n += n_nat
            from_n += n_fs
    propose_geom._last_ray_anchor_native_n = int(native_n)
    propose_geom._last_ray_anchor_from_shapely_n = int(from_n)

    max_anchors = 200
    if len(anchors_xy) > max_anchors:
        step = len(anchors_xy) / max_anchors
        anchors_xy = [anchors_xy[int(i * step)] for i in range(max_anchors)]

    min_x, min_y, max_x, max_y = region.bounds
    ray_len = np.sqrt((max_x - min_x)**2 + (max_y - min_y)**2)

    ray_angles = np.linspace(0, 2*np.pi, num_rays, endpoint=False)
    placement_angles = np.linspace(0, 2*np.pi, num_angles, endpoint=False)

    stride = max(1, anchor_stride)
    sample_fracs = (0.1, 0.5)
    origins: list[tuple[float, float]] = []
    directions: list[tuple[float, float]] = []
    for ax, ay in anchors_xy[::stride]:
        for r_angle in ray_angles:
            dx = ray_len * float(np.cos(r_angle))
            dy = ray_len * float(np.sin(r_angle))
            origins.append((float(ax), float(ay)))
            directions.append((dx, dy))
    # P2: fuse ray hits with per-interval span (pair ordered sample_fracs).
    # sample_fracs=(0.1, 0.5) → span ≈ dist(mouth, mid) / 0.4; mouth = 0.1 hit.
    n_frac = len(sample_fracs)
    mouth_frac = float(sample_fracs[0]) if n_frac else 0.1
    mid_frac = float(sample_fracs[1]) if n_frac >= 2 else mouth_frac
    frac_den = max(abs(mid_frac - mouth_frac), 1e-9)

    def _append_ray_hits(
        pts: Sequence[tuple[float, float]],
        out: list[tuple[float, float, float, int, float]],
    ) -> None:
        """Append (x, y, span, is_mouth, attract) using ordered frac pairs."""
        i = 0
        n = len(pts)
        while i < n:
            if i + 1 < n:
                x0, y0 = float(pts[i][0]), float(pts[i][1])
                x1, y1 = float(pts[i + 1][0]), float(pts[i + 1][1])
                span = math.hypot(x1 - x0, y1 - y0) / frac_den
                a0 = math.hypot(x0 - attract_x, y0 - attract_y)
                a1 = math.hypot(x1 - attract_x, y1 - attract_y)
                out.append((x0, y0, span, 1, a0))
                out.append((x1, y1, span, 0, a1))
                i += 2
            else:
                x0, y0 = float(pts[i][0]), float(pts[i][1])
                a0 = math.hypot(x0 - attract_x, y0 - attract_y)
                out.append((x0, y0, float(ray_len), 1, a0))
                i += 1

    hits_meta: list[tuple[float, float, float, int, float]] = []
    if origins:
        try:
            coords_flat, offsets = region_g.clip_ray_interior_batch(
                origins, directions, float(ray_len), sample_fracs,
            )
            n_rays = len(offsets) - 1
            for i in range(n_rays):
                a = int(offsets[i])
                b = int(offsets[i + 1])
                pts = [
                    (float(coords_flat[2 * k]), float(coords_flat[2 * k + 1]))
                    for k in range(a, b)
                ]
                _append_ray_hits(pts, hits_meta)
        except Exception:
            for (ox, oy), (dx, dy) in zip(origins, directions, strict=True):
                try:
                    hits = region_g.clip_ray_interior(
                        (ox, oy), (dx, dy), float(ray_len), sample_fracs,
                    )
                except Exception:
                    hits = []
                _append_ray_hits(
                    [(float(x), float(y)) for x, y in hits],
                    hits_meta,
                )

    pose_list: list[tuple[float, float, float]] = []
    cost_list: list[float] = []
    span_list: list[float] = []
    rank_span_list: list[float] = []
    mouth_list: list[int] = []
    seen: set[tuple[float, float, float]] = set()
    pool_cap = max(top_n * 3, top_n)
    # Modest expand past pool_cap so short invalid corridors cannot starve clear
    # mid-chords before filter; lex re-rank valid survivors for top_n.
    # P0: thrift void densify ray storm. Was max(pool_cap×2, top_n×6) ≡ top_n×6
    # (peak_ray 3072 / prop_ms~10s). Cap at top_n×4 — attract-sorted head; ×3
    # under-shot early min vs H2e; not Motif tip /4.
    expand_cap = max(int(top_n) * 4, int(top_n))
    # March is 64 steps along ray_len; spans below ~2 steps are noise (late miss).
    span_floor = max(float(ray_len) / 32.0, 1e-6)
    min_span = -1.0
    short_n = 0
    med_span = 0.0
    if hits_meta:
        spans = [h[2] for h in hits_meta]
        min_span = float(min(spans))
        usable = [s for s in spans if s >= span_floor - 1e-12]
        spans_sorted = sorted(usable) if usable else sorted(spans)
        med_span = float(spans_sorted[len(spans_sorted) // 2])
        short_n = sum(
            1 for s in spans if span_floor - 1e-12 <= s <= med_span + 1e-12
        )

    def _corridor_rank(span: float) -> int:
        # 0 = usable corridor band [floor, med]; 1 = noise/fat (demoted).
        if span_floor - 1e-12 <= span <= med_span + 1e-12:
            return 0
        return 1

    # Expand by attract (late-safe mid-chord supply); corridor lex only on clears.
    hits_meta.sort(key=lambda h: (h[4], _corridor_rank(h[2]), 0 if h[3] else 1))
    for pt_x, pt_y, span, is_mouth, hit_cost in hits_meta:
        for p_angle in placement_angles:
            coords = (pt_x, pt_y, float(p_angle))
            key = (round(coords[0], 4), round(coords[1], 4), round(coords[2], 4))
            if key in seen:
                continue
            seen.add(key)
            pose_list.append(coords)
            cost_list.append(hit_cost)
            span_list.append(span)
            rank_span_list.append(float(_corridor_rank(span)))
            mouth_list.append(int(is_mouth))
            if len(pose_list) >= expand_cap:
                break
        if len(pose_list) >= expand_cap:
            break

    if not pose_list:
        propose_geom._last_ray_short_span_kept = 0
        propose_geom._last_ray_short_span_n = int(short_n)
        propose_geom._last_ray_min_span = float(min_span)
        return []
    _ray_ms = (time.perf_counter() - t_ray0) * 1000.0
    propose_geom._last_raycast_ms = float(getattr(propose_geom, "_last_raycast_ms", 0.0) or 0.0) + _ray_ms
    span_map = {c: sp for c, sp in zip(pose_list, span_list, strict=True)}
    # P4: sort by final survivor key, then validity-chunk until top_n (same order
    # as filter-all-then-sort; skip validating the attract-tail once filled).
    order = sorted(
        range(len(pose_list)),
        key=lambda i: (
            rank_span_list[i],
            0 if mouth_list[i] else 1,
            cost_list[i],
        ),
    )
    ordered = [pose_list[i] for i in order]
    chunk = max(int(top_n) * 2, 32)
    valid: list[tuple[float, float, float]] = []
    for start in range(0, len(ordered), chunk):
        block = ordered[start:start + chunk]
        flags = batch_valid_flags(propose_geom, block, pt_push)
        for coords, ok in zip(block, flags, strict=True):
            if not ok:
                continue
            valid.append(coords)
            if len(valid) >= int(top_n):
                break
        if len(valid) >= int(top_n):
            break
    kept = valid[: int(top_n)]
    short_kept = sum(
        1
        for c in kept
        if span_floor - 1e-12 <= span_map.get(c, 1e9) <= med_span + 1e-12
    )
    # Last-call telem (pipeline accumulates into cascade / densify).
    propose_geom._last_ray_short_span_kept = int(short_kept)
    propose_geom._last_ray_short_span_n = int(short_n)
    propose_geom._last_ray_min_span = float(min_span)
    return kept
