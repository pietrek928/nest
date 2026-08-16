import math
from typing import Sequence

import numpy as np
from shapely import MultiPolygon, Polygon, box
from shapely.geometry.base import BaseGeometry
from shapely.ops import unary_union

from nest_graph.geometry import Geometry, StaticCollisionScene, find_polygon_distances_bipartite
from nest_graph.propose.placement_outline import outline_ring_geom

_MAX_OBSTACLE_PARTS = 32
# Prefer bbox safe-zone when it retains enough area for sampling; else Minkowski.
_SAFE_ZONE_BBOX_AREA_FRAC = 1e-3
_SAFE_ZONE_BBOX_AREA_FLOOR = 1e-6


def as_geometry(g) -> Geometry | None:
    """Coerce Shapely / Geometry / None → Geometry | None."""
    if g is None:
        return None
    if isinstance(g, Geometry):
        return g
    if hasattr(g, "is_empty") and g.is_empty:
        return None
    return Geometry.from_shapely(g)


def dual_pose_from_base(
    part_base: Geometry,
    part_poly: Polygon,
    tr: Sequence[float] | np.ndarray,
) -> tuple[Geometry, BaseGeometry]:
    """Native clearance pose + matching Shapely poly (one sync site)."""
    from nest_graph.utils import transform_poly

    t = np.asarray(tr, dtype=np.float64).reshape(3)
    native = part_base.apply_transform(float(t[0]), float(t[1]), float(t[2]))
    return native, transform_poly(part_poly, t)


def part_base_geoms(
    part_by_group: dict[int, Polygon],
    *,
    part_bases: dict[int, Geometry] | None = None,
) -> dict[int, Geometry]:
    """Reuse caller natives when present; otherwise build once per gid."""
    out: dict[int, Geometry] = {}
    if part_bases:
        for gid, g in part_bases.items():
            if g is not None and not getattr(g, "is_empty", False):
                out[int(gid)] = g
    for gid, poly in part_by_group.items():
        ig = int(gid)
        if ig in out or poly is None or getattr(poly, "is_empty", False):
            continue
        out[ig] = Geometry.from_shapely(poly)
    return out


def is_board_adj(
    poly,
    sheet: Polygon,
    min_dist: float,
    *,
    gap: float | None = None,
    ring: Geometry | None = None,
) -> bool:
    """Board-adjacent if standoff ≤ min_dist + 2*gap (edge-to-edge)."""
    if poly is None or (hasattr(poly, "is_empty") and poly.is_empty):
        return False
    if sheet is None or sheet.is_empty:
        return False
    board_ring = ring if ring is not None else outline_ring_geom(sheet)
    if board_ring is None:
        return False
    from nest_graph.propose.context import _cluster_merge_gap

    g_gap = float(gap) if gap is not None else float(
        _cluster_merge_gap([poly], min_dist, sheet)
    )
    geom = as_geometry(poly)
    if geom is None:
        return False
    try:
        return float(geom.standoff_distance(board_ring)) <= float(min_dist) + 2.0 * g_gap + 1e-9
    except Exception:
        return False


def placement_obstacles(voids, packed) -> list[Geometry]:
    """Assemble voids + packed into one obstacle list (board membership + packing)."""
    obs: list[Geometry] = []
    if voids:
        if isinstance(voids, Geometry):
            obs.append(voids)
        else:
            for v in voids:
                vg = as_geometry(v)
                if vg is not None:
                    obs.append(vg)
    if packed is None:
        return obs
    if isinstance(packed, Geometry):
        obs.append(packed)
        return obs
    for o in packed:
        og = as_geometry(o)
        if og is not None:
            obs.append(og)
    return obs


def selection_pairwise_independent(
    polys: Sequence[BaseGeometry],
    selected_indices: Sequence[int],
    min_dist: float = 0.0,
    *,
    require_clearance: bool = False,
) -> bool:
    """Cheap independence check on the selected subset only.

    Matches ``make_polygon_graph`` / C++ packing intersects: hard overlap only
    (EPA penetration depth > 1e-9). Zero-depth edge kisses are independent.
    Pass ``require_clearance=True`` to also enforce pairwise ``min_dist``.
    Uses Geometry intersects/distance (no Shapely intersects hotspot).
    """
    idxs = list(selected_indices)
    geoms: list[Geometry | None] = []
    for ia in idxs:
        pa = polys[ia]
        if pa is None or (hasattr(pa, "is_empty") and pa.is_empty):
            geoms.append(None)
        else:
            geoms.append(as_geometry(pa))
    for a in range(len(geoms)):
        ga = geoms[a]
        if ga is None:
            continue
        for b in range(a + 1, len(geoms)):
            gb = geoms[b]
            if gb is None:
                continue
            if ga.intersects(gb):
                return False
            if require_clearance and float(ga.distance(gb)) < float(min_dist):
                return False
    return True


def is_pose_clear(
    candidate,
    voids,
    packed,
    min_dist: float,
) -> bool:
    """Clearance SoT: StaticCollisionScene / Penetrating vs voids+packed (no fully_inside)."""
    cand_g = as_geometry(candidate)
    if cand_g is None:
        return False
    obs = placement_obstacles(voids, packed)
    if not obs:
        return True
    md = float(min_dist)
    if md <= 0.0:
        return clear_of_geoms(cand_g, obs, 0.0)
    scene = StaticCollisionScene.build(obs, aura=0.5)
    return bool(scene.is_valid_placement(cand_g, min_dist=md))


def clear_of_geoms(candidate: Geometry, others: list[Geometry], min_dist: float) -> bool:
    if not others:
        return True
    results = find_polygon_distances_bipartite([candidate], others, aura=0.5)
    for r in results:
        if r.intersect:
            return False
        if math.sqrt(r.distance_sq) < min_dist - 1e-9:
            return False
    return True


def obstacle_parts(
    base_shape: BaseGeometry,
    *,
    max_parts: int | None = _MAX_OBSTACLE_PARTS,
) -> list[Polygon]:
    """Split packed layout into individual polygons for per-neighbor slides."""
    if base_shape is None or base_shape.is_empty:
        return []
    if isinstance(base_shape, MultiPolygon):
        parts = [
            g for g in base_shape.geoms
            if isinstance(g, Polygon) and not g.is_empty
        ]
    elif isinstance(base_shape, Polygon):
        parts = [base_shape]
    else:
        parts = []
    if max_parts is None or max_parts <= 0:
        return parts
    return parts[:max_parts]


def _boundary_alignment_angles(poly: Polygon) -> list[float]:
    out: list[float] = []
    coords = list(poly.exterior.coords)
    for i in range(len(coords) - 1):
        dx = coords[i + 1][0] - coords[i][0]
        dy = coords[i + 1][1] - coords[i][1]
        if abs(dx) + abs(dy) < 1e-9:
            continue
        edge = math.atan2(dy, dx)
        out.append(edge)
        out.append(edge + math.pi / 2)
        out.append(edge - math.pi / 2)
    return out


def placement_angle_grid(
    sheet: Polygon,
    base_shape: BaseGeometry | None,
    num_angles: int,
) -> np.ndarray:
    """Uniform grid plus board/obstacle edge-alignment angles."""
    grid = list(np.linspace(0, 2 * np.pi, num_angles, endpoint=False))
    extra: list[float] = []
    if isinstance(sheet, Polygon):
        extra.extend(_boundary_alignment_angles(sheet))
    if base_shape is not None and not base_shape.is_empty:
        for part in obstacle_parts(base_shape):
            extra.extend(_boundary_alignment_angles(part))
    merged: list[float] = []
    seen: set[float] = set()
    for angle in grid + extra:
        key = round(angle, 3)
        if key in seen:
            continue
        seen.add(key)
        merged.append(float(angle))
    cap = min(len(merged), max(num_angles, num_angles * 3))
    return np.asarray(merged[:cap], dtype=np.float64)


def resolve_placement_angles(
    angles: np.ndarray | None,
    num_angles: int,
) -> np.ndarray:
    if angles is not None and len(angles) > 0:
        return np.asarray(angles, dtype=np.float64)
    return np.linspace(0, 2 * np.pi, num_angles, endpoint=False)


def cluster_seed_coords(
    candidates: list[tuple[float, float, float]],
    *,
    dist_tol: float = 1.0,
    angle_tol: float = 0.15,
) -> list[tuple[float, float, float]]:
    """Greedy spatial dedupe before cast-refine."""
    if len(candidates) <= 1:
        return list(candidates)
    kept: list[tuple[float, float, float]] = []
    for coords in candidates:
        if any(
            math.hypot(coords[0] - k[0], coords[1] - k[1]) < dist_tol
            and abs(coords[2] - k[2]) < angle_tol
            for k in kept
        ):
            continue
        kept.append(coords)
    return kept


def _rotated_max_dim(rotated: Polygon) -> float:
    bounds = rotated.bounds
    return max(bounds[2] - bounds[0], bounds[3] - bounds[1]) / 2


def _expanded_aabb(geom: BaseGeometry, radius: float) -> Polygon:
    minx, miny, maxx, maxy = geom.bounds
    return box(minx - radius, miny - radius, maxx + radius, maxy + radius)


def placement_safe_zone_bbox(
    region: BaseGeometry,
    base_shape: BaseGeometry,
    rotated: Polygon,
    min_dist: float,
) -> BaseGeometry:
    """Cheap AABB erosion: shrink region bounds, subtract expanded obstacle AABBs."""
    total_buf = _rotated_max_dim(rotated) + min_dist
    minx, miny, maxx, maxy = region.bounds
    if (maxx - minx) <= 2.0 * total_buf or (maxy - miny) <= 2.0 * total_buf:
        return Polygon()
    zone: BaseGeometry = box(
        minx + total_buf, miny + total_buf, maxx - total_buf, maxy - total_buf,
    )
    if base_shape is None or base_shape.is_empty:
        return zone
    parts = obstacle_parts(base_shape, max_parts=None)
    if not parts:
        return zone.difference(_expanded_aabb(base_shape, total_buf))
    for part in parts:
        zone = zone.difference(_expanded_aabb(part, total_buf))
        if zone.is_empty:
            return zone
    return zone


def _placement_safe_zone_minkowski(
    region: BaseGeometry,
    base_shape: BaseGeometry,
    rotated: Polygon,
    min_dist: float,
) -> BaseGeometry:
    """Eroded search region minus per-part Minkowski buffers (same radius as NFP)."""
    total_buf = _rotated_max_dim(rotated) + min_dist
    safe_zone = region.buffer(-total_buf)
    if safe_zone.is_empty:
        return safe_zone
    if base_shape is not None and not base_shape.is_empty:
        parts = obstacle_parts(base_shape, max_parts=None)
        if parts:
            obstacle_union = unary_union([p.buffer(total_buf) for p in parts])
            safe_zone = safe_zone.difference(obstacle_union)
        else:
            safe_zone = safe_zone.difference(base_shape.buffer(total_buf))
    return safe_zone


def placement_safe_zone(
    region: BaseGeometry,
    base_shape: BaseGeometry,
    rotated: Polygon,
    min_dist: float,
) -> BaseGeometry:
    """Search domain for placement seeds: bbox A/B first, Minkowski fallback."""
    bbox_zone = placement_safe_zone_bbox(region, base_shape, rotated, min_dist)
    min_area = max(
        _SAFE_ZONE_BBOX_AREA_FLOOR,
        float(rotated.area) * _SAFE_ZONE_BBOX_AREA_FRAC,
    )
    if not bbox_zone.is_empty and float(bbox_zone.area) >= min_area:
        return bbox_zone
    return _placement_safe_zone_minkowski(region, base_shape, rotated, min_dist)
