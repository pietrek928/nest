"""Post-refine gravity compaction for already-selected placements."""

from typing import Sequence

import numpy as np
from shapely import Point, Polygon
from shapely.geometry.base import BaseGeometry

from nest_graph.geometry import Geometry
from nest_graph.propose.placement_common import as_geometry, is_pose_clear
from nest_graph.propose.selection_edit import SelectionEditCtx
from nest_graph.utils import transform_poly


def sheet_gravity_point(sheet: Polygon) -> Point:
    """Prefer the exterior vertex with minimum x+y (right-angle corner on demo triangle).

    Callers with a large free void should pass ``gravity=void_pole`` to
    ``compact_selection`` instead of relying on this corner default.
    """
    coords = list(sheet.exterior.coords)
    if len(coords) >= 2 and coords[0] == coords[-1]:
        coords = coords[:-1]
    if not coords:
        return sheet.centroid
    return Point(min(coords, key=lambda xy: float(xy[0]) + float(xy[1])))


def compact_selection(
    sheet: Polygon | SelectionEditCtx,
    polys: list[BaseGeometry] | None = None,
    transforms: list[np.ndarray] | list | None = None,
    group_ids: Sequence[int] | None = None,
    selected_indices: Sequence[int] | None = None,
    part_by_group: dict[int, Polygon] | None = None,
    min_dist: float | None = None,
    *,
    gravity: Point | None = None,
    max_steps: int = 48,
    fixed_obstacles: Sequence[BaseGeometry] | None = None,
) -> tuple[list[BaseGeometry], list]:
    if isinstance(sheet, SelectionEditCtx):
        ctx = sheet
        sheet = ctx.sheet
        polys = ctx.polys
        transforms = ctx.transforms
        group_ids = ctx.group_ids
        selected_indices = ctx.selected_indices
        part_by_group = ctx.part_by_group
        min_dist = ctx.min_dist
        gravity = ctx.gravity if gravity is None else gravity
        fixed_obstacles = (
            ctx.fixed_obstacles if fixed_obstacles is None else fixed_obstacles
        )
    assert polys is not None and transforms is not None
    assert group_ids is not None and selected_indices is not None
    assert part_by_group is not None and min_dist is not None
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
    board_g = Geometry.from_shapely(sheet)

    order = sorted(
        selected_indices,
        key=lambda i: float(out_polys[i].centroid.distance(grav)),
    )
    locked = [
        g for g in (fixed_obstacles or ())
        if g is not None and not g.is_empty
    ]
    fixed_geoms: list[Geometry] = [
        g for g in (as_geometry(p) for p in locked) if g is not None
    ]
    part_geoms: dict[int, Geometry] = {}

    for idx in order:
        poly = out_polys[idx]
        tr = out_tr[idx]
        gid = int(group_ids[idx])
        part = part_by_group.get(gid)
        if part is None or poly is None or poly.is_empty:
            if poly is not None and not poly.is_empty:
                pg = as_geometry(poly)
                if pg is not None:
                    fixed_geoms.append(pg)
            continue

        if gid not in part_geoms:
            part_geoms[gid] = Geometry.from_shapely(part)
        part_g = part_geoms[gid]

        cx, cy = float(poly.centroid.x), float(poly.centroid.y)
        dx, dy = gx - cx, gy - cy
        dist = float(np.hypot(dx, dy))
        if dist < 1e-9:
            pg = as_geometry(poly)
            if pg is not None:
                fixed_geoms.append(pg)
            continue
        ux, uy = dx / dist, dy / dist

        others = [
            out_polys[j]
            for j in selected_indices
            if int(j) != int(idx) and out_polys[j] is not None and not out_polys[j].is_empty
        ]
        other_geoms = [
            g for g in (as_geometry(p) for p in others) if g is not None
        ]
        obs_geoms = fixed_geoms + other_geoms
        best_poly = poly
        best_tr = tr.copy()
        best_g = as_geometry(poly)
        # Grow along gravity until blocked; keep last clear pose.
        for s in range(1, max_steps + 1):
            ox = ux * step * s
            oy = uy * step * s
            cand_tr = np.array([tr[0] + ox, tr[1] + oy, tr[2]], dtype=np.float64)
            cand_g = part_g.apply_transform(float(cand_tr[0]), float(cand_tr[1]), float(cand_tr[2]))
            if not is_pose_clear(cand_g, board_g, obs_geoms, min_dist):
                break
            best_poly = transform_poly(part, cand_tr)
            best_tr = cand_tr
            best_g = cand_g

        prev_poly, prev_tr = out_polys[idx], out_tr[idx]
        out_polys[idx] = best_poly
        out_tr[idx] = best_tr
        # Reject only hard overlaps (graph semantics). Clearance is enforced
        # during the slide via ``is_pose_clear``; pre-refine packs may already
        # sit under min_dist without collision edges.
        if not selection_pairwise_independent(out_polys, selected_indices):
            out_polys[idx] = prev_poly
            out_tr[idx] = prev_tr
            best_poly = prev_poly
            best_g = as_geometry(prev_poly)
        if best_g is not None:
            fixed_geoms.append(best_g)

    return out_polys, out_tr


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
