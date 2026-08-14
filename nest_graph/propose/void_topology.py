"""Shared void / bay topology for pocket-fit, cluster_copy, and free-space snapshots."""

import math
from typing import Sequence

from shapely import Point, Polygon
from shapely.geometry.base import BaseGeometry
from shapely.ops import unary_union

from nest_graph.propose.context import cluster_packed_indices, iter_polygons

try:
    from nest_graph.geometry import polylabel_rings as _cpp_polylabel_rings
except Exception:  # pragma: no cover - extension optional at import time
    _cpp_polylabel_rings = None


def polylabel(poly: Polygon, tolerance: float = 1.0) -> Point:
    """Pole of inaccessibility; prefer C++ Mapbox polylabel, else Shapely."""
    tol = float(tolerance)
    if _cpp_polylabel_rings is not None and poly is not None and not poly.is_empty:
        try:
            rings = [list(poly.exterior.coords)]
            for interior in poly.interiors:
                rings.append(list(interior.coords))
            x, y, _r = _cpp_polylabel_rings(rings, tol)
            return Point(float(x), float(y))
        except Exception:
            pass
    from shapely.ops import polylabel as shapely_polylabel

    return shapely_polylabel(poly, tolerance=tol)


def _largest_polygon_component(geom: BaseGeometry | None) -> Polygon | None:
    if geom is None or geom.is_empty:
        return None
    best: Polygon | None = None
    best_area = 0.0
    for poly in iter_polygons(geom):
        a = float(poly.area)
        if a > best_area:
            best = poly
            best_area = a
    return best


def iterative_multi_poles(
    void_poly: Polygon,
    *,
    min_dist: float = 1.0,
    max_poles: int = 4,
    radius_scale: float = 0.8,
    min_area_ratio: float = 0.05,
) -> list[tuple[float, float]]:
    """Spine of poles via iterative polylabel − inscribed-circle masks.

    Walks a cheap medial-axis surrogate without Voronoi/CDT. Always keeps the
    largest remaining component after each subtraction.
    """
    if void_poly is None or void_poly.is_empty or max_poles <= 0:
        return []
    base_area = float(void_poly.area)
    if base_area <= 0.0:
        return []
    min_area = max(base_area * float(min_area_ratio), float(min_dist) ** 2)
    tol = max(float(min_dist), 0.5)
    poly = void_poly
    poles: list[tuple[float, float]] = []
    for _ in range(int(max_poles)):
        if poly is None or poly.is_empty or float(poly.area) < min_area:
            break
        try:
            pt = polylabel(poly, tolerance=tol)
        except Exception:
            pt = poly.representative_point()
        if pt is None or pt.is_empty:
            break
        x, y = float(pt.x), float(pt.y)
        poles.append((x, y))
        try:
            r = float(Point(x, y).distance(poly.exterior))
        except Exception:
            r = float(min_dist)
        if r <= 1e-9:
            break
        mask = Point(x, y).buffer(max(r * float(radius_scale), tol * 0.5))
        try:
            remaining = poly.difference(mask)
        except Exception:
            break
        poly = _largest_polygon_component(remaining)
    return poles


def nearest_spine_pole(
    x: float,
    y: float,
    poles: Sequence[Point] | None,
    fallback: Point | None = None,
) -> Point | None:
    """Nearest non-empty spine pole to ``(x, y)``. Empty ``poles`` skips pull."""
    candidates = [
        p for p in (poles or ())
        if p is not None and not getattr(p, "is_empty", True)
    ]
    if not candidates:
        if poles is not None:
            return None
        if fallback is not None and not getattr(fallback, "is_empty", True):
            return fallback
        return None
    best = candidates[0]
    best_d = (float(best.x) - x) ** 2 + (float(best.y) - y) ** 2
    for p in candidates[1:]:
        d = (float(p.x) - x) ** 2 + (float(p.y) - y) ** 2
        if d < best_d:
            best = p
            best_d = d
    return best


def preferred_spine_pole(
    x: float,
    y: float,
    poles: Sequence[Point] | None,
    fallback: Point | None = None,
    *,
    clear_ratio: float = 1.5,
) -> Point | None:
    """Nearest spine pole only when clearly closer than the primary; else primary.

    Secondary-lobe pull must beat the first polylabel by ``clear_ratio`` so
    floaters do not scatter along the surrogate spine.
    """
    nearest = nearest_spine_pole(x, y, poles, fallback=fallback)
    if poles is not None and nearest is None:
        return None
    primary = None
    for p in (poles or ()):
        if p is not None and not getattr(p, "is_empty", True):
            primary = p
            break
    if primary is None:
        primary = (
            fallback
            if fallback is not None and not getattr(fallback, "is_empty", True)
            else None
        )
    if nearest is None:
        return primary
    if primary is None or nearest is primary:
        return nearest
    dn = math.hypot(float(nearest.x) - x, float(nearest.y) - y)
    dp = math.hypot(float(primary.x) - x, float(primary.y) - y)
    if dp <= 1e-12:
        return primary
    if dn * float(clear_ratio) < dp:
        return nearest
    return primary


def multi_pole_seed_coords(
    void_poly: Polygon,
    *,
    min_dist: float = 1.0,
    max_poles: int = 4,
    angles: Sequence[float] | None = None,
) -> list[tuple[float, float, float]]:
    """SE(2) seeds at iterative multi-poles (default θ=0 and π)."""
    poles = iterative_multi_poles(void_poly, min_dist=min_dist, max_poles=max_poles)
    thetas = list(angles) if angles is not None else [0.0, float(math.pi)]
    out: list[tuple[float, float, float]] = []
    for x, y in poles:
        for th in thetas:
            out.append((x, y, float(th)))
    return out


def touches_sheet_exterior(poly: Polygon, sheet: Polygon, eps: float = 1e-6) -> bool:
    if sheet is None or sheet.is_empty or not hasattr(sheet, "exterior"):
        return True
    ext = sheet.exterior
    if poly.touches(ext) or poly.intersects(ext):
        inter = poly.intersection(ext)
        if inter.is_empty:
            return False
        if hasattr(inter, "length") and float(inter.length) > eps:
            return True
        if hasattr(inter, "area") and float(inter.area) > eps:
            return True
        return True
    return False


def trapped_void_polygons(
    sheet: Polygon,
    packed: Sequence[BaseGeometry],
    *,
    free: BaseGeometry | None = None,
) -> list[Polygon]:
    """Free components of unbuffered sheet\\packed that do not touch sheet exterior."""
    placed = [p for p in packed if p is not None and not p.is_empty]
    if sheet is None or sheet.is_empty:
        return []
    if not placed:
        return []
    try:
        if free is None:
            free = sheet.difference(unary_union(placed))
    except Exception:
        return []
    if free is None or free.is_empty:
        return []
    out: list[Polygon] = []
    for poly in iter_polygons(free):
        if touches_sheet_exterior(poly, sheet):
            continue
        out.append(poly)
    out.sort(key=lambda p: float(p.area), reverse=True)
    return out


def hull_bay_polygons(
    packed: Sequence[BaseGeometry],
    *,
    min_dist: float,
    sheet: Polygon | None = None,
) -> list[Polygon]:
    """Convex-hull − cluster pockets for each contact-connected solid group."""
    placed = [p for p in packed if p is not None and not p.is_empty]
    if len(placed) < 2:
        return []
    idxs_groups = cluster_packed_indices(placed, min_dist, sheet=sheet)
    out: list[Polygon] = []
    for idxs in idxs_groups:
        if len(idxs) < 2:
            continue
        cluster = unary_union([placed[i] for i in idxs])
        if cluster is None or cluster.is_empty:
            continue
        hull = cluster.convex_hull
        if hull is None or hull.is_empty:
            continue
        try:
            pockets = hull.difference(cluster)
        except Exception:
            continue
        out.extend(iter_polygons(pockets))
    out.sort(key=lambda p: float(p.area), reverse=True)
    return out


def topology_pocket_poles(
    sheet: Polygon,
    packed: Sequence[BaseGeometry],
    *,
    min_dist: float,
    max_anchors: int = 6,
    voids: Sequence[Polygon] | None = None,
    bays: Sequence[Polygon] | None = None,
) -> list[tuple[float, float, float]]:
    """Polylabel anchors (0 and π) for trapped voids + hull bays."""
    void_list = list(voids) if voids is not None else trapped_void_polygons(sheet, packed)
    bay_list = (
        list(bays)
        if bays is not None
        else hull_bay_polygons(packed, min_dist=min_dist, sheet=sheet)
    )
    polys = void_list + bay_list
    polys.sort(key=lambda p: float(p.area), reverse=True)
    out: list[tuple[float, float, float]] = []
    for poly in polys[:max_anchors]:
        # Prefer multi-pole spine on large remnants; single polylabel otherwise.
        if float(poly.area) >= max(float(min_dist) ** 2 * 16.0, 1.0):
            seeds = multi_pole_seed_coords(
                poly, min_dist=min_dist, max_poles=min(3, max_anchors),
            )
            out.extend(seeds)
        else:
            try:
                pt = polylabel(poly, tolerance=max(float(min_dist), 0.5))
            except Exception:
                pt = poly.representative_point()
            if pt is None or pt.is_empty:
                continue
            out.append((float(pt.x), float(pt.y), 0.0))
            out.append((float(pt.x), float(pt.y), float(math.pi)))
        if len(out) >= max_anchors * 2:
            break
    return out[: max_anchors * 2]
