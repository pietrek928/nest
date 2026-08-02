"""Post-refine gravity compaction for already-selected placements."""

from typing import Sequence

import numpy as np
from shapely import Point, Polygon
from shapely.geometry.base import BaseGeometry
from shapely.ops import unary_union

from nest_graph.utils import transform_poly


def sheet_gravity_point(sheet: Polygon) -> Point:
    """Prefer the exterior vertex with minimum x+y (right-angle corner on demo triangle)."""
    coords = list(sheet.exterior.coords)
    if len(coords) >= 2 and coords[0] == coords[-1]:
        coords = coords[:-1]
    if not coords:
        return sheet.centroid
    return Point(min(coords, key=lambda xy: float(xy[0]) + float(xy[1])))


def _pose_clear(
    candidate: BaseGeometry,
    sheet: Polygon,
    fixed: BaseGeometry,
    min_dist: float,
) -> bool:
    if candidate is None or candidate.is_empty:
        return False
    if not sheet.contains(candidate) and not sheet.buffer(1e-9).contains(candidate):
        return False
    if fixed is None or fixed.is_empty:
        return True
    if candidate.intersects(fixed) and candidate.intersection(fixed).area > 1e-12:
        return False
    return float(candidate.distance(fixed)) + 1e-12 >= float(min_dist)


def compact_selection(
    sheet: Polygon,
    polys: list[BaseGeometry],
    transforms: list[np.ndarray] | list,
    group_ids: Sequence[int],
    selected_indices: Sequence[int],
    part_by_group: dict[int, Polygon],
    min_dist: float,
    *,
    gravity: Point | None = None,
    max_steps: int = 48,
    fixed_obstacles: Sequence[BaseGeometry] | None = None,
) -> tuple[list[BaseGeometry], list]:
    """Slide selected parts toward gravity; keep angles; reject unsafe slides.

    Returns updated ``(polys, transforms)`` copies (same length as inputs).
    ``fixed_obstacles`` (e.g. locked seeds / sheet voids) never move.
    """
    if len(selected_indices) < 2 or sheet is None or sheet.is_empty:
        return list(polys), list(transforms)

    grav = gravity if gravity is not None else sheet_gravity_point(sheet)
    gx, gy = float(grav.x), float(grav.y)
    step = max(0.25 * float(min_dist), 1e-4)

    out_polys = list(polys)
    out_tr = [np.asarray(t, dtype=np.float64).reshape(3) for t in transforms]

    order = sorted(
        selected_indices,
        key=lambda i: float(out_polys[i].centroid.distance(grav)),
    )
    locked = [
        g for g in (fixed_obstacles or ())
        if g is not None and not g.is_empty
    ]
    fixed_parts: list[BaseGeometry] = list(locked)

    for idx in order:
        poly = out_polys[idx]
        tr = out_tr[idx]
        gid = int(group_ids[idx])
        part = part_by_group.get(gid)
        if part is None or poly is None or poly.is_empty:
            if poly is not None and not poly.is_empty:
                fixed_parts.append(poly)
            continue

        cx, cy = float(poly.centroid.x), float(poly.centroid.y)
        dx, dy = gx - cx, gy - cy
        dist = float(np.hypot(dx, dy))
        if dist < 1e-9:
            fixed_parts.append(poly)
            continue
        ux, uy = dx / dist, dy / dist

        others = [
            out_polys[j]
            for j in selected_indices
            if int(j) != int(idx) and out_polys[j] is not None and not out_polys[j].is_empty
        ]
        fixed = unary_union(fixed_parts + others) if (fixed_parts or others) else Polygon()
        best_poly = poly
        best_tr = tr.copy()
        # Grow along gravity until blocked; keep last clear pose.
        for s in range(1, max_steps + 1):
            ox = ux * step * s
            oy = uy * step * s
            cand_tr = np.array([tr[0] + ox, tr[1] + oy, tr[2]], dtype=np.float64)
            cand = transform_poly(part, cand_tr)
            if not _pose_clear(cand, sheet, fixed, min_dist):
                break
            best_poly = cand
            best_tr = cand_tr

        prev_poly, prev_tr = out_polys[idx], out_tr[idx]
        out_polys[idx] = best_poly
        out_tr[idx] = best_tr
        # Reject only hard overlaps (graph semantics). Clearance is enforced
        # during the slide via ``_pose_clear``; pre-refine packs may already
        # sit under min_dist without collision edges.
        if not selection_pairwise_independent(out_polys, selected_indices):
            out_polys[idx] = prev_poly
            out_tr[idx] = prev_tr
            best_poly = prev_poly
        fixed_parts.append(best_poly)

    return out_polys, out_tr


def selection_pairwise_independent(
    polys: Sequence[BaseGeometry],
    selected_indices: Sequence[int],
    min_dist: float = 0.0,
    *,
    require_clearance: bool = False,
) -> bool:
    """Cheap independence check on the selected subset only.

    Matches ``make_polygon_graph`` by default (hard intersections only). Pass
    ``require_clearance=True`` to also enforce pairwise ``min_dist``.
    """
    idxs = list(selected_indices)
    for a in range(len(idxs)):
        ia = idxs[a]
        pa = polys[ia]
        if pa is None or pa.is_empty:
            continue
        for b in range(a + 1, len(idxs)):
            ib = idxs[b]
            pb = polys[ib]
            if pb is None or pb.is_empty:
                continue
            if pa.intersects(pb) and pa.intersection(pb).area > 1e-12:
                return False
            if require_clearance and float(pa.distance(pb)) + 1e-12 < float(min_dist):
                return False
    return True
