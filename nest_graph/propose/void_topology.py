"""Shared void / bay topology for pocket-fit, cluster_copy, and free-space snapshots."""

import math
from typing import Sequence

from shapely import Polygon
from shapely.geometry.base import BaseGeometry
from shapely.ops import polylabel, unary_union

from nest_graph.propose.context import cluster_packed_indices, iter_polygons


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
) -> list[Polygon]:
    """Free components of unbuffered sheet\\packed that do not touch sheet exterior."""
    placed = [p for p in packed if p is not None and not p.is_empty]
    if sheet is None or sheet.is_empty:
        return []
    if not placed:
        return []
    try:
        free = sheet.difference(unary_union(placed))
    except Exception:
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
        try:
            pt = polylabel(poly, tolerance=max(float(min_dist), 0.5))
        except Exception:
            pt = poly.representative_point()
        if pt is None or pt.is_empty:
            continue
        out.append((float(pt.x), float(pt.y), 0.0))
        out.append((float(pt.x), float(pt.y), float(math.pi)))
    return out[: max_anchors * 2]
