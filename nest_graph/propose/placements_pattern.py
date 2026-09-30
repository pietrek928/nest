"""Rigid cluster-copy proposer: reuse packed motifs elsewhere on the sheet."""

from dataclasses import dataclass
import math
from typing import List, Optional, Sequence, Tuple

from shapely import MultiPolygon, Polygon
from shapely.geometry import Point
from shapely.geometry.base import BaseGeometry
from shapely.ops import unary_union

from nest_graph.config import ProposeConfig
from nest_graph.geometry import Geometry, convex_hull_area_of
from nest_graph.graph import MotifBase, MotifRecord, Se2
from nest_graph.propose.context import (
    cluster_packed_indices,
    part_extents,
    placement_free_region,
    void_pole_seed_coords,
)
from nest_graph.propose.geometry import ProposeGeometry
from nest_graph.propose.placement_common import _boundary_alignment_angles
from nest_graph.propose.void_topology import (
    multi_pole_seed_coords,
    polylabel,
    topology_pocket_poles,
)
from nest_graph.utils import (
    compose_transforms,
    relative_transform,
    transform_poly,
    transform_row_key,
)


@dataclass(frozen=True)
class ClusterPattern:
    """Relative SE(2) motif extracted from a packed contact cluster."""

    members: tuple[tuple[int, tuple[float, float, float]], ...]
    part_count: int
    ref_transform: tuple[float, float, float]
    motif_id: int = -1


# H0: kiss MotifBase inject (ckpt Δxy≈0.1); classify dual same-gid collide.
_SAME_GID_KISS_XY = 0.5
_POLE_ANCHOR_RADIUS = 2.0


def _bump_skip(skip_reasons: dict[str, int] | None, key: str, n: int = 1) -> None:
    if skip_reasons is None:
        return
    skip_reasons[key] = int(skip_reasons.get(key, 0) or 0) + int(n)


def _same_gid_rels_kiss(rels: Sequence[tuple[float, float, float]]) -> bool:
    """True when ≥2 same-gid relatives are near-coincident (kiss dual)."""
    if len(rels) < 2:
        return False
    lim2 = _SAME_GID_KISS_XY * _SAME_GID_KISS_XY
    for i, a in enumerate(rels):
        ax, ay = float(a[0]), float(a[1])
        for b in rels[i + 1 :]:
            dx = ax - float(b[0])
            dy = ay - float(b[1])
            if dx * dx + dy * dy <= lim2:
                return True
    return False


def _anchor_near_pole(
    anchor: tuple[float, float, float],
    pole_xy: tuple[float, float] | None,
    *,
    radius: float = _POLE_ANCHOR_RADIUS,
) -> bool:
    if pole_xy is None:
        return False
    dx = float(anchor[0]) - float(pole_xy[0])
    dy = float(anchor[1]) - float(pole_xy[1])
    return dx * dx + dy * dy <= float(radius) * float(radius)


def _record_stamp_collide(
    skip_reasons: dict[str, int] | None,
    *,
    at_pole: bool,
    same_gid_kiss: bool,
) -> None:
    """H0: split collide into pole/topo + same-gid dual-rel (keep aggregate collide)."""
    _bump_skip(skip_reasons, "collide")
    if same_gid_kiss:
        _bump_skip(skip_reasons, "same_gid_dual_rel_fail")
    if at_pole:
        _bump_skip(skip_reasons, "collide_at_pole")
    else:
        _bump_skip(skip_reasons, "collide_topo")


def _same_gid_leader_rel(
    rels: Sequence[tuple[float, float, float]],
) -> tuple[float, float, float]:
    """Identity-primary leader: nearest-to-origin relative (kiss dual collapses to one)."""
    if not rels:
        return (0.0, 0.0, 0.0)
    return min(
        (tuple(float(x) for x in r[:3]) for r in rels),
        key=lambda r: float(r[0]) * float(r[0]) + float(r[1]) * float(r[1]),
    )


def _void_clear_rate_sample(
    void_poly: Polygon,
    *,
    propose_geom: ProposeGeometry,
    obstacles,
    angles: Sequence[float],
    n_xy: int = 32,
) -> tuple[int, int]:
    """T0: sample n_xy Halton×grain → emit_packing_clear. Returns (clear, trials)."""
    if void_poly is None or getattr(void_poly, "is_empty", True) or not angles:
        return 0, 0
    try:
        minx, miny, maxx, maxy = void_poly.bounds
    except Exception:
        return 0, 0
    w = float(maxx) - float(minx)
    h = float(maxy) - float(miny)
    if w <= 1e-9 or h <= 1e-9:
        return 0, 0
    clear = 0
    trials = 0
    ang = [float(a) for a in angles]
    for i in range(1, max(int(n_xy), 1) + 1):
        u, v = _halton_uv(i)
        x = float(minx) + u * w
        y = float(miny) + v * h
        if not void_poly.covers(Point(x, y)):
            continue
        th = ang[(i - 1) % len(ang)]
        trials += 1
        if emit_packing_clear(
            propose_geom, (x, y, th), obstacles=obstacles,
        ):
            clear += 1
    return int(clear), int(trials)


def _order_anchors_clear_first(
    anchors: Sequence[tuple[float, float, float]],
    *,
    propose_geom: ProposeGeometry,
    obstacles,
    skip_reasons: dict[str, int] | None,
) -> list[tuple[float, float, float]]:
    """H1: prefer packing-clear identity anchors; keep uncleared as fallback (no sterilize)."""
    clear: list[tuple[float, float, float]] = []
    unclear: list[tuple[float, float, float]] = []
    for a in anchors:
        coords = (float(a[0]), float(a[1]), float(a[2]))
        if emit_packing_clear(propose_geom, coords, obstacles=obstacles):
            clear.append(coords)
        else:
            unclear.append(coords)
            _bump_skip(skip_reasons, "anchor_clear_fail")
    if clear:
        _bump_skip(skip_reasons, "anchor_clear_kept", len(clear))
    return clear + unclear


def _halton_uv(index: int) -> tuple[float, float]:
    """Halton (base-2, base-3) sample in (0,1)^2."""
    u = 0.0
    f = 0.5
    ii = int(index)
    while ii > 0:
        u += f * (ii % 2)
        ii //= 2
        f *= 0.5
    v = 0.0
    f = 1.0 / 3.0
    ii = int(index)
    while ii > 0:
        v += f * (ii % 3)
        ii //= 3
        f /= 3.0
    return float(u), float(v)


def _try_clear_xy_at(
    x: float,
    y: float,
    *,
    void_poly: Polygon,
    propose_geom: ProposeGeometry,
    obstacles,
    angles: Sequence[float],
    seen: set,
    found: list[tuple[float, float, float]],
    skip_reasons: dict[str, int] | None,
) -> bool:
    """One XY: try angle set until packing-clear (N4a)."""
    pt = Point(float(x), float(y))
    if not void_poly.covers(pt):
        return False
    # Deep-in-void collide → near-miss fuel for A3 (not rim-edge noise).
    try:
        deep_void = float(void_poly.exterior.distance(pt)) > 0.25
    except Exception:
        deep_void = False
    for th in angles:
        coords = (float(x), float(y), float(th))
        key = transform_row_key(coords)
        if key in seen:
            continue
        seen.add(key)
        _bump_skip(skip_reasons, "clear_xy_try")
        if not emit_packing_clear(propose_geom, coords, obstacles=obstacles):
            _bump_skip(skip_reasons, "clear_xy_fail")
            if deep_void:
                _bump_skip(skip_reasons, "clear_xy_fail_near")
            continue
        found.append(coords)
        _bump_skip(skip_reasons, "clear_xy_ok")
        return True
    return False


def _unlock_clear_xy_anchors(
    anchors: Sequence[tuple[float, float, float]],
    *,
    propose_geom: ProposeGeometry,
    obstacles,
    void_poly: Polygon | None,
    pole: Point | None,
    shape_to_place: Polygon | None,
    skip_reasons: dict[str, int] | None,
    max_new: int = 12,
    allowed_angles: Sequence[float] | None = None,
    sheet: Polygon | None = None,
    min_dist: float = 0.0,
    base_shape: BaseGeometry | None = None,
) -> list[tuple[float, float, float]]:
    """N4a/A1: when densify_clear kept zero clears, find packing-clear XY in free void.

    One gate after ``_order_anchors_clear_first``. Prefers polylabel + grain angles,
    then Halton; kiss-slide unclear survivors via Dc1 helper.
    """
    kept = int((skip_reasons or {}).get("anchor_clear_kept", 0) or 0)
    # Still unlock into free void when only a few topo clears exist (rim/pole).
    if kept >= max(int(max_new), 1):
        return list(anchors)
    if void_poly is None or getattr(void_poly, "is_empty", True):
        _bump_skip(skip_reasons, "clear_xy_skip_no_void")
        return list(anchors)
    if shape_to_place is None or getattr(shape_to_place, "is_empty", True):
        return list(anchors)
    if propose_geom is None:
        return list(anchors)

    minx, miny, maxx, maxy = void_poly.bounds
    w = float(maxx - minx)
    h = float(maxy - miny)
    if w <= 1e-12 or h <= 1e-12:
        _bump_skip(skip_reasons, "clear_xy_skip_no_void")
        return list(anchors)

    compass = (
        0.0,
        math.pi / 4,
        math.pi / 2,
        3.0 * math.pi / 4,
        math.pi,
        -math.pi / 4,
        -math.pi / 2,
        -3.0 * math.pi / 4,
    )
    grain: list[float] = []
    # V: longest free-void edge first (wedge), then caller / part grain.
    wedge = wedge_edge_grain_angles(void_poly)
    if wedge:
        grain.extend(wedge)
        _bump_skip(skip_reasons, "wedge_grain_n", len(wedge))
    if allowed_angles:
        grain.extend(float(a) for a in allowed_angles)
    else:
        try:
            grain.extend(
                float(a) for a in _boundary_alignment_angles(shape_to_place)
            )
        except Exception:
            pass
    # Dedupe while preserving wedge-first order.
    if grain:
        seen_g: set[float] = set()
        uniq: list[float] = []
        for a in grain:
            key = round(float(a), 6)
            if key in seen_g:
                continue
            seen_g.add(key)
            uniq.append(float(a))
        grain = uniq
    # Grain first so clear_xy_ok_grain can be distinguished.
    angles_grain = tuple(grain) if grain else ()
    angles_all = tuple(dict.fromkeys([*grain, *compass]))

    found: list[tuple[float, float, float]] = []
    seen: set = set()
    cap = max(int(max_new), 1)

    # A1: polylabel seed before Halton AABB.
    try:
        poi = polylabel(void_poly, tolerance=max(float(min_dist), 0.5))
        px0, py0 = float(poi.x), float(poi.y)
        before = len(found)
        if angles_grain:
            _try_clear_xy_at(
                px0, py0,
                void_poly=void_poly,
                propose_geom=propose_geom,
                obstacles=obstacles,
                angles=angles_grain,
                seen=seen,
                found=found,
                skip_reasons=skip_reasons,
            )
            if len(found) > before:
                _bump_skip(skip_reasons, "clear_xy_ok_grain", len(found) - before)
        if len(found) < cap:
            _try_clear_xy_at(
                px0, py0,
                void_poly=void_poly,
                propose_geom=propose_geom,
                obstacles=obstacles,
                angles=angles_all,
                seen=seen,
                found=found,
                skip_reasons=skip_reasons,
            )
    except Exception:
        pass

    for i in range(1, 49):
        if len(found) >= cap:
            break
        u, v = _halton_uv(i)
        before = len(found)
        if angles_grain:
            _try_clear_xy_at(
                minx + u * w,
                miny + v * h,
                void_poly=void_poly,
                propose_geom=propose_geom,
                obstacles=obstacles,
                angles=angles_grain,
                seen=seen,
                found=found,
                skip_reasons=skip_reasons,
            )
            if len(found) > before:
                _bump_skip(skip_reasons, "clear_xy_ok_grain", len(found) - before)
                continue
        _try_clear_xy_at(
            minx + u * w,
            miny + v * h,
            void_poly=void_poly,
            propose_geom=propose_geom,
            obstacles=obstacles,
            angles=angles_all,
            seen=seen,
            found=found,
            skip_reasons=skip_reasons,
        )

    if len(found) < cap:
        try:
            cent = void_poly.centroid
            cx, cy = float(cent.x), float(cent.y)
        except Exception:
            cx, cy = 0.5 * (minx + maxx), 0.5 * (miny + maxy)
        px, py = cx, cy
        if pole is not None and not getattr(pole, "is_empty", True):
            px, py = float(pole.x), float(pole.y)
        dx, dy = cx - px, cy - py
        dist = math.hypot(dx, dy)
        ux, uy = (1.0, 0.0) if dist < 1e-9 else (dx / dist, dy / dist)
        max_e = max(float(part_extents(shape_to_place)[1]), 1e-3)
        for sc in (0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 4.0):
            if len(found) >= cap:
                break
            _try_clear_xy_at(
                px + ux * max_e * sc,
                py + uy * max_e * sc,
                void_poly=void_poly,
                propose_geom=propose_geom,
                obstacles=obstacles,
                angles=angles_all,
                seen=seen,
                found=found,
                skip_reasons=skip_reasons,
            )

    # A1: kiss-slide unclear Halton/pole samples already tried (Dc1).
    if len(found) < cap and sheet is not None and base_shape is not None:
        # Cycle: placements_free_space_cloud → emit_packing_clear (this module).
        from nest_graph.propose.placements_free_space_cloud import (
            polish_cloud_slide_toward_obstacles,
        )
        cloud: list[tuple[float, float, float]] = []
        for i in range(1, 25):
            u, v = _halton_uv(i + 50)
            th = float(angles_all[i % max(len(angles_all), 1)]) if angles_all else 0.0
            cloud.append((minx + u * w, miny + v * h, th))
        if pole is not None and not getattr(pole, "is_empty", True):
            cloud.append((float(pole.x), float(pole.y), 0.0))
        polished, slid_n = polish_cloud_slide_toward_obstacles(
            cloud,
            propose_geom=propose_geom,
            base_shape=base_shape,
            sheet=sheet,
            min_dist=float(min_dist),
            shape_to_place=shape_to_place,
        )
        _bump_skip(skip_reasons, "clear_xy_slide_try", int(slid_n) or len(cloud))
        for coords in polished:
            if len(found) >= cap:
                break
            key = transform_row_key(coords)
            if key in seen:
                continue
            if not void_poly.covers(Point(float(coords[0]), float(coords[1]))):
                continue
            seen.add(key)
            if emit_packing_clear(propose_geom, coords, obstacles=obstacles):
                found.append(
                    (float(coords[0]), float(coords[1]), float(coords[2]))
                )
                _bump_skip(skip_reasons, "clear_xy_slide_ok")
                _bump_skip(skip_reasons, "clear_xy_ok")

    # A3: near-miss jostle — micro XY ring around deep-void fails (hybrid w/ A1 slide).
    near_n = int((skip_reasons or {}).get("clear_xy_fail_near", 0) or 0)
    if len(found) < cap and near_n > 0:
        max_e = max(float(part_extents(shape_to_place)[1]), 1e-3)
        step = max(0.15 * max_e, 0.5)
        jostle_xy = (
            (step, 0.0),
            (-step, 0.0),
            (0.0, step),
            (0.0, -step),
            (step, step),
            (-step, step),
            (step, -step),
            (-step, -step),
        )
        before_j = len(found)
        for i in range(1, 17):
            if len(found) >= cap:
                break
            u, v = _halton_uv(i + 80)
            bx, by = minx + u * w, miny + v * h
            for dx, dy in jostle_xy:
                if len(found) >= cap:
                    break
                _try_clear_xy_at(
                    bx + dx,
                    by + dy,
                    void_poly=void_poly,
                    propose_geom=propose_geom,
                    obstacles=obstacles,
                    angles=angles_all,
                    seen=seen,
                    found=found,
                    skip_reasons=skip_reasons,
                )
        if len(found) > before_j:
            _bump_skip(skip_reasons, "clear_xy_jostle_ok", len(found) - before_j)

    if not found:
        _bump_skip(skip_reasons, "clear_xy_none")
        return list(anchors)
    # Densify: clear-XY replaces sterile fuel (do not re-append collide anchors).
    return list(found)


def _void_pole_ring_anchors(
    pole_xy: tuple[float, float],
    *,
    radius: float,
    num: int = 8,
) -> list[tuple[float, float, float]]:
    """H1 research: ring around void pole at ~part extent when pole XY is occupied."""
    if radius <= 1e-9 or num <= 0:
        return []
    out: list[tuple[float, float, float]] = []
    for i in range(int(num)):
        ang = (2.0 * math.pi * float(i)) / float(num)
        out.append(
            (
                float(pole_xy[0]) + float(radius) * math.cos(ang),
                float(pole_xy[1]) + float(radius) * math.sin(ang),
                ang,
            )
        )
    return out


def _free_space_void_poly(free_space) -> Polygon | None:
    """Void polygon from free_space snapshot/analysis (pocket R4 input)."""
    if free_space is None:
        return None
    analysis = getattr(free_space, "analysis", None)
    poly = None
    if analysis is not None:
        poly = getattr(analysis, "target_poly", None)
    if poly is None:
        poly = getattr(free_space, "target_poly", None)
    if poly is None or getattr(poly, "is_empty", True):
        return None
    return poly


def _as_native_geom(p) -> Geometry | None:
    """C1: one Geometry conversion site for placements_pattern."""
    if p is None or getattr(p, "is_empty", False):
        return None
    if isinstance(p, Geometry):
        return p
    try:
        return Geometry.from_shapely(p)
    except Exception:
        return None


def _cluster_compactness(
    placed: Sequence[BaseGeometry],
    idxs: Sequence[int],
) -> float:
    """sum(member areas) / convex_hull_area; prefer native Geometry hull."""
    areas = 0.0
    geoms = []
    for i in idxs:
        p = placed[i]
        if p is None or getattr(p, "is_empty", False):
            continue
        areas += float(p.area)
        g = _as_native_geom(p)
        if g is not None:
            geoms.append(g)
    if areas <= 1e-18:
        return 0.0
    hull = 0.0
    if len(geoms) >= 1:
        try:
            hull = float(convex_hull_area_of(geoms))
        except Exception:
            hull = 0.0
    if hull <= 1e-18:
        try:
            union = unary_union([placed[i] for i in idxs if placed[i] is not None])
            hull = float(union.convex_hull.area) if union is not None else 0.0
        except Exception:
            return 0.0
    if hull <= 1e-18:
        return 0.0
    return float(areas) / float(hull)


def extract_cluster_patterns(
    placed: Sequence[BaseGeometry],
    group_ids: Sequence[int],
    transforms: Sequence[tuple[float, float, float] | Sequence[float]],
    *,
    min_dist: float,
    max_patterns: int = 2,
    min_members: int = 2,
    sheet: Polygon | None = None,
    min_compactness: float = 0.0,
) -> list[ClusterPattern]:
    """Build rigid motifs from contact-connected packed clusters.

    Ranked by hull compactness (sum areas / hull area), then total area.
    """
    if len(placed) < min_members:
        return []
    n = min(len(placed), len(group_ids), len(transforms))
    if n < min_members:
        return []

    index_groups = cluster_packed_indices(list(placed[:n]), min_dist, sheet=sheet)
    scored: list[tuple[float, float, list[int]]] = []
    min_c = float(min_compactness)
    for idxs in index_groups:
        if len(idxs) < min_members:
            continue
        area = sum(float(placed[i].area) for i in idxs)
        compact = _cluster_compactness(placed, idxs)
        if min_c > 0.0 and compact < min_c:
            continue
        scored.append((compact, area, idxs))
    scored.sort(key=lambda x: (x[0], x[1]), reverse=True)

    patterns: list[ClusterPattern] = []
    for _c, _area, idxs in scored[:max_patterns]:
        pat = cluster_pattern_from_indices(idxs, placed, group_ids, transforms)
        if pat is not None:
            patterns.append(pat)
    return patterns


def _poly_ring_coords(poly: Polygon) -> list[tuple[float, float]]:
    if poly is None or poly.is_empty:
        return []
    coords = list(poly.exterior.coords)
    if len(coords) >= 2 and coords[0] == coords[-1]:
        coords = coords[:-1]
    return [(float(x), float(y)) for x, y in coords]


def _longest_edge(coords: Sequence[tuple[float, float]]) -> tuple[int, int, float]:
    """Return (i, j, length) for the longest exterior edge."""
    n = len(coords)
    best = (0, 1 % max(n, 1), 0.0)
    for i in range(n):
        j = (i + 1) % n
        dx = coords[j][0] - coords[i][0]
        dy = coords[j][1] - coords[i][1]
        length = math.hypot(dx, dy)
        if length > best[2]:
            best = (i, j, length)
    return best


def wedge_edge_grain_angles(void_poly: Polygon | None) -> list[float]:
    """V: grain from longest exterior edge of free void (target_poly SoT).

    One gate with ``_boundary_alignment_angles`` / clear-XY grain — prepend only;
    does not retune Halton / cloud / pin.
    """
    if void_poly is None or getattr(void_poly, "is_empty", True):
        return []
    try:
        coords = list(void_poly.exterior.coords)
    except Exception:
        return []
    if len(coords) < 2:
        return []
    pts = [(float(x), float(y)) for x, y in coords[:-1]] if (
        len(coords) > 1 and coords[0] == coords[-1]
    ) else [(float(x), float(y)) for x, y in coords]
    if len(pts) < 2:
        return []
    i, j, length = _longest_edge(pts)
    if length <= 1e-12:
        return []
    dx = pts[j][0] - pts[i][0]
    dy = pts[j][1] - pts[i][1]
    edge = math.atan2(dy, dx)
    return [edge, edge + math.pi / 2.0, edge - math.pi / 2.0]


def _fit_probe_ok(
    void_poly: Polygon,
    shape: Polygon,
    *,
    min_dist: float,
) -> bool:
    """Same A0 probe: polylabel inradius vs part max extent."""
    try:
        poi = polylabel(void_poly, tolerance=max(float(min_dist), 0.5))
        in_r = float(void_poly.exterior.distance(Point(float(poi.x), float(poi.y))))
        _min_e, max_e = part_extents(shape)
        return float(max_e) <= in_r + 1e-9
    except Exception:
        return False


def motif_pair_union_unlock_shape(
    patterns: Sequence[ClusterPattern],
    group_id: int,
    shape_to_place: Polygon,
    part_by_group: dict[int, Polygon] | None,
    void_poly: Polygon | None,
    *,
    min_dist: float,
    skip_reasons: dict[str, int] | None = None,
) -> Polygon:
    """M: pair-union unlock shape only when union fit_probe_ok; else single part."""
    if (
        void_poly is None
        or getattr(void_poly, "is_empty", True)
        or shape_to_place is None
        or getattr(shape_to_place, "is_empty", True)
    ):
        return shape_to_place
    for pat in patterns or ():
        members = list(pat.members or ())
        if len(members) < 2:
            continue
        solids: list = [shape_to_place]
        for gid, t_rel in members:
            tr = (
                float(t_rel[0]),
                float(t_rel[1]),
                float(t_rel[2]),
            )
            if abs(tr[0]) + abs(tr[1]) + abs(tr[2]) <= 1e-9:
                continue
            if int(gid) == int(group_id):
                solids.append(transform_poly(shape_to_place, tr))
            elif part_by_group is not None and int(gid) in part_by_group:
                solids.append(transform_poly(part_by_group[int(gid)], tr))
        solids = [s for s in solids if s is not None and not getattr(s, "is_empty", True)]
        if len(solids) < 2:
            continue
        try:
            u = unary_union(solids)
        except Exception:
            continue
        if u is None or getattr(u, "is_empty", True):
            continue
        if not isinstance(u, Polygon):
            try:
                u = u.convex_hull
            except Exception:
                continue
        if not isinstance(u, Polygon) or u.is_empty:
            continue
        if _fit_probe_ok(void_poly, u, min_dist=min_dist):
            _bump_skip(skip_reasons, "pair_union_fit_ok")
            return u
        _bump_skip(skip_reasons, "pair_union_fit_fail")
    return shape_to_place


def _outward_normal_ccw(
    a: tuple[float, float],
    b: tuple[float, float],
) -> tuple[float, float]:
    """Unit outward normal for CCW edge a→b (right-hand side)."""
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    length = math.hypot(dx, dy)
    if length <= 1e-12:
        return (0.0, 0.0)
    # Right normal of (dx, dy) is (dy, -dx).
    return (dy / length, -dx / length)


def triangle_mate_relative(
    poly: Polygon,
    *,
    min_dist: float,
) -> tuple[float, float, float] | None:
    """180° mate about longest-edge midpoint, shifted by ``min_dist`` along outward normal.

    Returns the mate pose relative to identity placement of ``poly``, or None if
    the part is not a simple triangle.
    """
    coords = _poly_ring_coords(poly)
    if len(coords) != 3:
        return None
    i, j, length = _longest_edge(coords)
    if length <= 1e-12:
        return None
    a, b = coords[i], coords[j]
    mid = (0.5 * (a[0] + b[0]), 0.5 * (a[1] + b[1]))
    nx, ny = _outward_normal_ccw(a, b)
    # rotate-about-origin by π then translate by 2*mid ≡ rotate 180° about mid.
    gap = max(float(min_dist), 0.0)
    tx = 2.0 * mid[0] + gap * nx
    ty = 2.0 * mid[1] + gap * ny
    return (tx, ty, math.pi)


def _pair_hull_area(
    poly: Polygon,
    mate_t: tuple[float, float, float],
) -> float:
    a = poly
    b = transform_poly(poly, mate_t)
    if a is None or b is None or a.is_empty or b.is_empty:
        return float("inf")
    try:
        ga = _as_native_geom(a)
        gb = _as_native_geom(b)
        if ga is None or gb is None:
            raise ValueError("native geom missing")
        return float(convex_hull_area_of([ga, gb]))
    except Exception:
        try:
            return float(unary_union([a, b]).convex_hull.area)
        except Exception:
            return float("inf")


def cluster_pattern_from_indices(
    indices: Sequence[int],
    polys: Sequence[BaseGeometry],
    group_ids: Sequence[int],
    transforms: Sequence,
) -> ClusterPattern | None:
    """Build a ClusterPattern from a contact-connected index set (shared builder)."""
    idxs = [int(i) for i in indices]
    if len(idxs) < 2:
        return None
    ref_i = max(idxs, key=lambda i: float(polys[i].area))
    ref_t = (
        float(transforms[ref_i][0]),
        float(transforms[ref_i][1]),
        float(transforms[ref_i][2]),
    )
    members: list[tuple[int, tuple[float, float, float]]] = []
    for i in idxs:
        t = (
            float(transforms[i][0]),
            float(transforms[i][1]),
            float(transforms[i][2]),
        )
        members.append((int(group_ids[i]), relative_transform(ref_t, t)))
    return ClusterPattern(
        members=tuple(members),
        part_count=len(members),
        ref_transform=ref_t,
    )


def _generic_mate_relative(
    poly: Polygon,
    *,
    min_dist: float,
    n_angles: int = 18,
) -> tuple[float, float, float] | None:
    """Search edge-aligned mates minimizing pair convex-hull area."""
    coords = _poly_ring_coords(poly)
    n = len(coords)
    if n < 3:
        return None
    gap = max(float(min_dist), 0.0)
    best_t: tuple[float, float, float] | None = None
    best_area = float("inf")
    angles = [math.pi]  # 180° mate is the primary candidate
    if n_angles > 1:
        step = (2.0 * math.pi) / float(n_angles)
        angles.extend(i * step for i in range(n_angles) if abs(i * step - math.pi) > 1e-9)
    for i in range(n):
        j = (i + 1) % n
        a, b = coords[i], coords[j]
        edge_len = math.hypot(b[0] - a[0], b[1] - a[1])
        if edge_len <= 1e-12:
            continue
        mid = (0.5 * (a[0] + b[0]), 0.5 * (a[1] + b[1]))
        nx, ny = _outward_normal_ccw(a, b)
        for ang in angles:
            # Rotate about mid: R_mid = T(mid) R(ang) T(-mid).
            # As rotate-about-0 then translate: t = mid - R(ang)@mid, then + gap*n.
            c, s = math.cos(ang), math.sin(ang)
            rx = c * mid[0] - s * mid[1]
            ry = s * mid[0] + c * mid[1]
            tx = mid[0] - rx + gap * nx
            ty = mid[1] - ry + gap * ny
            mate = (tx, ty, ang)
            placed = transform_poly(poly, mate)
            if placed is None or placed.is_empty:
                continue
            if poly.intersects(placed) and poly.distance(placed) < gap * 0.5:
                # Still penetrating after gap push — skip.
                if poly.intersection(placed).area > 1e-9:
                    continue
            area = _pair_hull_area(poly, mate)
            if area < best_area:
                best_area = area
                best_t = mate
    return best_t


def synthesize_mate_patterns(
    parts: Sequence[tuple[Polygon, int]],
    *,
    min_dist: float,
    max_patterns: int = 2,
) -> list[ClusterPattern]:
    """Geometry-derived mated-pair patterns (shape-agnostic; triangles closed-form)."""
    patterns: list[ClusterPattern] = []
    seen_gids: set[int] = set()
    for poly, gid in parts:
        gid_i = int(gid)
        if gid_i in seen_gids:
            continue
        seen_gids.add(gid_i)
        if poly is None or poly.is_empty:
            continue
        mate = triangle_mate_relative(poly, min_dist=min_dist)
        if mate is None:
            mate = _generic_mate_relative(poly, min_dist=min_dist)
        if mate is None:
            continue
        # Validate clearance: pair must not penetrate at identity+mate.
        placed = transform_poly(poly, mate)
        if placed is None or placed.is_empty:
            continue
        if poly.intersects(placed) and float(poly.intersection(placed).area) > 1e-8:
            continue
        patterns.append(
            ClusterPattern(
                members=(
                    (gid_i, (0.0, 0.0, 0.0)),
                    (gid_i, (float(mate[0]), float(mate[1]), float(mate[2]))),
                ),
                part_count=2,
                ref_transform=(0.0, 0.0, 0.0),
            )
        )
        if len(patterns) >= max(int(max_patterns), 1):
            break
    return patterns


def seed_motif_base_from_mates(
    motif_base: MotifBase,
    parts: Sequence[tuple[Polygon, int]],
    *,
    min_dist: float,
    max_patterns: int = 2,
    ttl: int = 8,
) -> int:
    """N1: upsert congruent mate pairs into MotifBase (same relative as synth).

    Reuses ``synthesize_mate_patterns`` — does not invent a second mate algorithm.
    """
    patterns = synthesize_mate_patterns(
        parts, min_dist=float(min_dist), max_patterns=int(max_patterns),
    )
    poly_by_gid: dict[int, Polygon] = {}
    for poly, gid in parts:
        gid_i = int(gid)
        if gid_i not in poly_by_gid and poly is not None and not poly.is_empty:
            poly_by_gid[gid_i] = poly
    seeded = 0
    for pat in patterns:
        if len(pat.members) < 2:
            continue
        gid_a, t_a = pat.members[0]
        gid_b, t_b = pat.members[1]
        poly_a = poly_by_gid.get(int(gid_a))
        if poly_a is None:
            continue
        # Synth stores identity + mate relative; MotifRecord relative is B vs A.
        mate_t = (
            float(t_b[0]) - float(t_a[0]),
            float(t_b[1]) - float(t_a[1]),
            float(t_b[2]) - float(t_a[2]),
        )
        aa = float(poly_a.area)
        ab = float(poly_by_gid[int(gid_b)].area) if int(gid_b) in poly_by_gid else aa
        hull = _pair_hull_area(poly_a, mate_t)
        pair_area = aa + ab
        compact = (
            float(pair_area / hull)
            if hull > 0.0 and math.isfinite(hull)
            else 0.5
        )
        rec = MotifRecord()
        rec.gid_a = int(gid_a)
        rec.gid_b = int(gid_b)
        rec.relative = Se2(float(mate_t[0]), float(mate_t[1]), float(mate_t[2]))
        rec.area_a = aa
        rec.area_b = ab
        rec.compactness = float(compact)
        rec.gci = float(compact)
        mid = int(motif_base.upsert(rec, 0.0, max(int(ttl), 1)))
        if mid >= 0:
            seeded += 1
    return int(seeded)


def _pattern_signature(pat: ClusterPattern) -> tuple:
    """Dedupe key: sorted (gid, rounded relative SE2)."""
    members = tuple(
        sorted(
            (
                int(gid),
                round(float(t[0]), 3),
                round(float(t[1]), 3),
                round(float(t[2]), 3),
            )
            for gid, t in pat.members
        )
    )
    return (int(pat.part_count), members)


def merge_cluster_patterns(
    contact: Sequence[ClusterPattern],
    synthesized: Sequence[ClusterPattern],
    *,
    max_patterns: int,
    archived: Sequence[ClusterPattern] = (),
    reserve_archived: int = 0,
) -> list[ClusterPattern]:
    """Prefer contact → accepted archive → synthesized mates (one prefer path).

    When ``reserve_archived>0`` and archive non-empty, leave room so MotifBase
    archive is not starved by live contact filling the cap (P2 / large_void).
    """
    cap = max(int(max_patterns), 1)
    arch_list = list(archived or ())
    reserve = min(max(int(reserve_archived), 0), cap, len(arch_list))
    contact_cap = max(0, cap - reserve)
    out: list[ClusterPattern] = []
    seen: set[tuple] = set()

    def _add(seq: Sequence[ClusterPattern], limit: int | None = None) -> None:
        for pat in seq:
            if len(out) >= cap:
                return
            if limit is not None and len(out) >= limit:
                return
            sig = _pattern_signature(pat)
            if sig in seen:
                continue
            seen.add(sig)
            out.append(pat)

    _add(contact, contact_cap if reserve > 0 else None)
    _add(arch_list)
    _add(synthesized)
    return out


def motif_lattice_offsets(
    pattern: ClusterPattern,
    *,
    min_step: float = 0.0,
) -> list[tuple[float, float]]:
    """Δxy lattice steps from member relatives (ignore identity).

    ``min_step`` > 0 scales short kiss |Δxy| up to that length (M3 large_void).
    Opposite-orientation mates (|Δθ|≈π) use 2·Δxy before the min_step floor.
    """
    offsets: list[tuple[float, float]] = []
    seen: set[tuple[float, float]] = set()
    min_l = max(float(min_step), 0.0)
    for _gid, t_rel in pattern.members:
        dx, dy = float(t_rel[0]), float(t_rel[1])
        dang = float(t_rel[2]) if len(t_rel) > 2 else 0.0
        if abs(dx) < 1e-6 and abs(dy) < 1e-6:
            continue
        if min_l > 0.0:
            twopi = 2.0 * math.pi
            w = (dang + math.pi) % twopi - math.pi
            if abs(abs(w) - math.pi) < 0.35:
                dx *= 2.0
                dy *= 2.0
            length = math.hypot(dx, dy)
            if length < 1e-9:
                continue
            if length + 1e-12 < min_l:
                scale = min_l / length
                dx *= scale
                dy *= scale
        key = (round(dx, 3), round(dy, 3))
        if key in seen:
            continue
        seen.add(key)
        offsets.append((dx, dy))
        nkey = (round(-dx, 3), round(-dy, 3))
        if nkey not in seen:
            seen.add(nkey)
            offsets.append((-dx, -dy))
    return offsets


def free_pocket_anchors(
    sheet: Polygon,
    obstacle: BaseGeometry,
    min_dist: float,
    max_anchors: int,
) -> list[tuple[float, float, float]]:
    """Polylabel / multi-pole anchors in free pockets of sheet\\obstacle."""
    free = placement_free_region(sheet, obstacle, min_dist)
    if free.is_empty:
        return []
    polys: list[Polygon] = []
    if isinstance(free, MultiPolygon):
        polys = [g for g in free.geoms if isinstance(g, Polygon) and not g.is_empty]
    elif isinstance(free, Polygon):
        polys = [free]
    if not polys:
        return []
    polys.sort(key=lambda p: p.area, reverse=True)
    out: list[tuple[float, float, float]] = []
    for poly in polys[:max_anchors]:
        if float(poly.area) >= max(float(min_dist) ** 2 * 16.0, 1.0):
            out.extend(
                multi_pole_seed_coords(
                    poly, min_dist=min_dist, max_poles=min(3, max_anchors),
                )
            )
        else:
            try:
                pt = polylabel(poly, tolerance=max(min_dist, 0.5))
            except Exception:
                pt = poly.representative_point()
            if pt is None or pt.is_empty:
                continue
            out.append((float(pt.x), float(pt.y), 0.0))
            out.append((float(pt.x), float(pt.y), float(math.pi)))
        if len(out) >= max_anchors * 2:
            break
    return out[: max_anchors * 2]


_free_pocket_anchors = free_pocket_anchors


def _mirror_anchors(
    ref: tuple[float, float, float],
    sheet: Polygon,
) -> list[tuple[float, float, float]]:
    minx, miny, maxx, maxy = sheet.bounds
    cx = 0.5 * (minx + maxx)
    cy = 0.5 * (miny + maxy)
    x, y, a = float(ref[0]), float(ref[1]), float(ref[2])
    return [
        (2.0 * cx - x, y, -a),
        (x, 2.0 * cy - y, math.pi - a),
        (2.0 * cx - x, 2.0 * cy - y, a + float(math.pi)),
    ]


def _dedupe_anchors(
    anchors: Sequence[tuple[float, float, float]],
) -> list[tuple[float, float, float]]:
    seen: set[tuple[float, float, float]] = set()
    out: list[tuple[float, float, float]] = []
    for a in anchors:
        key = (round(float(a[0]), 2), round(float(a[1]), 2), round(float(a[2]), 2))
        if key in seen:
            continue
        seen.add(key)
        out.append((float(a[0]), float(a[1]), float(a[2])))
    return out


def dedupe_anchors(
    anchors: Sequence[tuple[float, float, float]],
) -> list[tuple[float, float, float]]:
    """Public alias for shared void/repack anchor dedupe (round-2 local)."""
    return _dedupe_anchors(anchors)


def void_seek_motif_anchors(
    sheet: Polygon,
    base_shape: BaseGeometry,
    *,
    min_dist: float,
    propose_cfg: ProposeConfig,
    free_space=None,
    void_pole: Point | None = None,
    patterns: Sequence[ClusterPattern] = (),
    lattice_stats_out: dict | None = None,
    void_poly: Polygon | None = None,
    part_for_pocket: Polygon | None = None,
    anchor_cache: dict | None = None,
    allowed_angles: Sequence[float] | None = None,
) -> list[tuple[float, float, float]]:
    """Unified anchor priority for void_seek motif stamps (§4).

    topology / void_pole → topology_pocket → free_pocket → ΔT lattice (pole-sort)
    → optional pocket-aligned poses (R4) → optional AABB mirrors last.
    """
    if anchor_cache is not None and "anchors" in anchor_cache:
        return list(anchor_cache["anchors"])

    anchors: list[tuple[float, float, float]] = []
    n_seed = int(propose_cfg.cluster_copy_anchor_seeds)
    lattice_added = 0
    lattice_kept = 0

    if free_space is not None and getattr(free_space, "topology_poles", None):
        anchors.extend(list(free_space.topology_poles))
    if void_pole is not None and not getattr(void_pole, "is_empty", True):
        anchors.extend(void_pole_seed_coords(void_pole, num_angles=4))

    packed_for_topo: list = []
    if hasattr(base_shape, "geoms"):
        packed_for_topo = [
            g for g in base_shape.geoms if g is not None and not g.is_empty
        ]
    elif base_shape is not None and not base_shape.is_empty:
        packed_for_topo = [base_shape]
    if packed_for_topo and (
        free_space is None or not getattr(free_space, "topology_poles", None)
    ):
        anchors.extend(
            topology_pocket_poles(
                sheet,
                packed_for_topo,
                min_dist=min_dist,
                max_anchors=n_seed,
            )
        )

    anchors.extend(free_pocket_anchors(sheet, base_shape, min_dist, n_seed))

    if bool(getattr(propose_cfg, "enable_motif_lattice", True)) and patterns:
        depth = max(int(getattr(propose_cfg, "motif_lattice_depth", 3) or 0), 0)
        top_k = max(int(getattr(propose_cfg, "motif_lattice_top_k", 10) or 0), 0)
        # M3: min_step>0 + void pole + motif_lattice_2d → Stoyan v0⊕v1; else 1D ±k·ΔT.
        min_step = float(
            getattr(propose_cfg, "motif_lattice_min_step", 0.0) or 0.0
        )
        pole_xy: tuple[float, float] | None = None
        if void_pole is not None and not getattr(void_pole, "is_empty", True):
            pole_xy = (float(void_pole.x), float(void_pole.y))
        use_2d = (
            pole_xy is not None
            and min_step > 0.0
            and bool(getattr(propose_cfg, "motif_lattice_2d", True))
        )
        lattice: list[tuple[float, float, float]] = []
        base_for_lattice = list(anchors)
        # T1 miss-loop: period floor alone still stamps from packed-neighborhood
        # topo bases → collide. Grow lattice from void pole when min_step>0.
        if min_step > 0.0 and pole_xy is not None:
            base_for_lattice = list(
                void_pole_seed_coords(void_pole, num_angles=4)
            )
        if not base_for_lattice:
            for pat in patterns:
                base_for_lattice.append(pat.ref_transform)
        for pat in patterns:
            offsets = motif_lattice_offsets(pat, min_step=min_step)
            if not offsets or depth <= 0:
                continue
            if use_2d:
                pos = [
                    (dx, dy)
                    for dx, dy in offsets
                    if dx > 1e-9 or (abs(dx) <= 1e-9 and dy > 0)
                ]
                if not pos:
                    pos = list(offsets)
                pos.sort(key=lambda o: -(o[0] * o[0] + o[1] * o[1]))
                v0 = pos[0]
                v1: tuple[float, float] | None = None
                if len(pos) >= 2:
                    ax0, ay0 = v0
                    for cand in pos[1:]:
                        bx, by = cand
                        cross = abs(ax0 * by - ay0 * bx)
                        if cross > 1e-6 * (
                            math.hypot(ax0, ay0) * math.hypot(bx, by) + 1e-9
                        ):
                            v1 = cand
                            break
                if v1 is None:
                    lx, ly = v0
                    ln = math.hypot(lx, ly)
                    if ln > 1e-9:
                        v1 = (-ly / ln * min_step, lx / ln * min_step)
                if v1 is not None:
                    for ax, ay, aa in base_for_lattice:
                        for i in range(-depth, depth + 1):
                            for j in range(-depth, depth + 1):
                                if i == 0 and j == 0:
                                    continue
                                lattice.append(
                                    (
                                        float(ax) + i * v0[0] + j * v1[0],
                                        float(ay) + i * v0[1] + j * v1[1],
                                        float(aa),
                                    )
                                )
                    continue
            for ax, ay, aa in base_for_lattice:
                for dx, dy in offsets:
                    for k in range(1, depth + 1):
                        lattice.append(
                            (float(ax) + k * dx, float(ay) + k * dy, float(aa))
                        )
        lattice_added = len(lattice)
        if use_2d and depth > 0:
            # Co-scale top_k so deeper 2D lattices are not pruned to 10 (M3).
            top_k = max(top_k, min(64, depth * 8))
        if lattice and top_k > 0:
            if pole_xy is not None:
                px, py = pole_xy

                def _dist(a: tuple[float, float, float]) -> float:
                    return (float(a[0]) - px) ** 2 + (float(a[1]) - py) ** 2

                lattice.sort(key=_dist)
            lattice = lattice[:top_k]
            lattice_kept = len(lattice)
            anchors.extend(lattice)
        elif lattice:
            lattice_kept = len(lattice)
            anchors.extend(lattice)

    if bool(getattr(propose_cfg, "enable_motif_mirror_anchors", False)):
        for pat in patterns:
            anchors.extend(_mirror_anchors(pat.ref_transform, sheet))

    if (
        propose_cfg.motif_use_topo_anchors
        and void_poly is not None
        and not void_poly.is_empty
        and part_for_pocket is not None
        and not part_for_pocket.is_empty
    ):
        from nest_graph.propose.placements_pocket import aligned_poses_for_pocket

        # V: pocket R4 gets same wedge-edge grain as clear-XY unlock.
        pocket_angles = list(wedge_edge_grain_angles(void_poly))
        if allowed_angles:
            pocket_angles.extend(float(a) for a in allowed_angles)
        for coords, _tag in aligned_poses_for_pocket(
            part_for_pocket,
            void_poly,
            min_dist=min_dist,
            allowed_angles=pocket_angles or allowed_angles,
        ):
            anchors.append(coords)

    if lattice_stats_out is not None:
        lattice_stats_out["lattice_anchors_added"] = int(lattice_added)
        lattice_stats_out["lattice_anchors_kept"] = int(lattice_kept)

    out = _dedupe_anchors(anchors)
    if anchor_cache is not None:
        anchor_cache["anchors"] = list(out)
    return out


def emit_packing_clear(
    propose_geom: ProposeGeometry,
    coords: tuple[float, float, float],
    *,
    obstacles: Sequence | None = None,
) -> bool:
    """Propose-emit packing SoT: Penetrating vs voids+packed (margin 0, not Scene)."""
    from nest_graph.propose.placement_common import placement_obstacles

    placed = propose_geom.placed_at(coords)
    if placed is None:
        return False
    obs = obstacles
    if obs is None:
        obs = placement_obstacles(
            propose_geom.scene.void_geoms,
            propose_geom.full_packed_geoms,
        )
    if not obs:
        return True
    return not placed.intersects_any(list(obs))


def _foreign_member_packing_status(
    part: Polygon,
    coords: tuple[float, float, float],
    propose_geom: ProposeGeometry,
    *,
    obstacles: Sequence | None = None,
) -> str:
    """Penetrating foreign clear status: ``ok`` | ``empty`` | ``sheet`` | ``obs``."""
    placed_m = transform_poly(part, coords)
    if placed_m is None or placed_m.is_empty:
        return "empty"
    if not propose_geom.sheet.buffer(1e-5).covers(placed_m.centroid):
        return "sheet"
    native = _as_native_geom(placed_m)
    if native is None:
        return "empty"
    obs = list(obstacles) if obstacles is not None else []
    if not obs:
        return "ok"
    if native.intersects_any(obs):
        return "obs"
    return "ok"


def _foreign_member_packing_clear(
    part: Polygon,
    coords: tuple[float, float, float],
    propose_geom: ProposeGeometry,
    *,
    obstacles: Sequence | None = None,
) -> bool:
    """Penetrating packing-clear for a foreign solid vs obstacles (+ sheet cover)."""
    return _foreign_member_packing_status(
        part, coords, propose_geom, obstacles=obstacles
    ) == "ok"


def _full_motif_packing_clear(
    pat: ClusterPattern,
    t_anchor: tuple[float, float, float],
    group_id: int,
    shape_to_place: Polygon,
    propose_geom: ProposeGeometry,
    part_by_group: dict[int, Polygon] | None,
    *,
    obstacles: Sequence | None = None,
) -> bool:
    """True if every motif member is packing-clear at ``t_anchor``.

    Same-group uses ``emit_packing_clear``. Foreign members with ``part_by_group``
    use Penetrating solid-vs-obstacles (``transform_poly`` → ``_as_native_geom`` →
    ``intersects_any``) plus sheet cover. Unknown foreign solids are skipped.
    ``shape_to_place`` is unused (kept for call-site compatibility).
    """
    _ = shape_to_place
    for gid, t_rel_m in pat.members:
        t_m = compose_transforms(t_anchor, t_rel_m)
        if int(gid) == int(group_id):
            if not emit_packing_clear(propose_geom, t_m, obstacles=obstacles):
                return False
            continue
        if part_by_group is not None and int(gid) in part_by_group:
            if not _foreign_member_packing_clear(
                part_by_group[int(gid)],
                t_m,
                propose_geom,
                obstacles=obstacles,
            ):
                return False
        else:
            # Unknown foreign solid: require leader-cell clear only (leader path).
            continue
    return True


def foreign_hard_packed_for_stamp(
    full_packed: Sequence | None,
    packed_group_id: Sequence[int] | None,
    packed_transform: Sequence | None,
    *,
    seed_geoms: Sequence | None = None,
) -> tuple[list | None, int]:
    """R1: P2 soft schedule for foreign Penetrating clear at stamp.

    When nest packed gid/tr align with geoms, self-map treats all nest-packed as
    soft (mapped); hard = seed_geoms only (board seeds usually live in void_geoms).
    Returns ``(None, 0)`` when alignment missing so stamp keeps full-packed foreign
    obs (legacy / unit tests).
    """
    packed = [g for g in (full_packed or []) if g is not None]
    if (
        not packed
        or packed_group_id is None
        or packed_transform is None
        or len(packed_group_id) != len(packed_transform)
        or len(packed_group_id) != len(packed)
    ):
        return None, 0
    seeds = [g for g in (seed_geoms or []) if g is not None]
    # Self-map → all nest-packed soft; hard = explicit seeds only.
    return list(seeds), len(packed)


def stamp_motif_leader_follower(
    patterns: Sequence[ClusterPattern],
    group_id: int,
    shape_to_place: Polygon,
    *,
    propose_geom: ProposeGeometry,
    anchors: Sequence[tuple[float, float, float]],
    top_n: int = 16,
    part_by_group: dict[int, Polygon] | None = None,
    skip_reasons: dict[str, int] | None = None,
    cohorts_out: list | None = None,
    foreign_out: dict[int, list] | None = None,
    hard_packed_geoms: Sequence | None = None,
    void_pole: Point | None = None,
) -> list[tuple[float, float, float]]:
    """Propose-side motif stamp: full-motif packing clear, else same-group leader only.

    Emits same-group candidate transforms under packing clearance (not Scene
    margin). On full clear, packing-clear foreign members are co-emitted into
    ``foreign_out`` (Q17: cohort ``member_keys`` = emitted abs keys only).
    Selection/repack uses ``stamp_motif_at_anchor`` (atomic peeled placement under
    ``is_pose_clear``) with ``pattern_fallback`` as the partial path.

    ``hard_packed_geoms`` (R1): foreign Penetrating clear uses this packed list
    (P2 soft schedule — seed∪unmapped, or empty for voids-only). Same-group emit
    still uses full ``propose_geom.full_packed_geoms``. ``None`` = full packed for
    foreign too (legacy / unit tests).

    ``void_pole`` (H0): classifies collide telem as at-pole vs topo.
    """
    if not patterns or not anchors:
        _bump_skip(
            skip_reasons,
            "no_anchors" if not anchors else "no_patterns",
        )
        return []

    from nest_graph.propose.placement_common import placement_obstacles

    voids = propose_geom.scene.void_geoms
    full_packed = list(propose_geom.full_packed_geoms or [])
    obs = placement_obstacles(voids, full_packed)
    if hard_packed_geoms is None:
        obs_foreign = obs
        soft_n = 0
    else:
        hard = [g for g in hard_packed_geoms if g is not None]
        obs_foreign = placement_obstacles(voids, hard)
        soft_n = max(0, len(full_packed) - len(hard))
    if soft_n > 0 and skip_reasons is not None:
        skip_reasons["soft_foreign_incumbent_n"] = int(soft_n)
    pole_xy: tuple[float, float] | None = None
    if void_pole is not None and not getattr(void_pole, "is_empty", True):
        pole_xy = (float(void_pole.x), float(void_pole.y))
    if not anchors:
        _bump_skip(skip_reasons, "no_anchors")
        return []
    seen: set[tuple[float, float, float]] = set()
    foreign_seen: set[tuple[int, tuple[float, float, float]]] = set()
    out: list[tuple[float, float, float]] = []

    def _maybe_add(
        coords: tuple[float, float, float],
        *,
        at_pole: bool,
        same_gid_kiss: bool,
        record_collide: bool,
    ) -> bool:
        key = (round(coords[0], 2), round(coords[1], 2), round(coords[2], 2))
        if key in seen:
            return False
        seen.add(key)
        if not emit_packing_clear(propose_geom, coords, obstacles=obs):
            if record_collide:
                _record_stamp_collide(
                    skip_reasons,
                    at_pole=at_pole,
                    same_gid_kiss=same_gid_kiss,
                )
            return False
        out.append(coords)
        return True

    for pat in patterns:
        rels = [t_rel for gid, t_rel in pat.members if int(gid) == int(group_id)]
        if not rels:
            _bump_skip(skip_reasons, "no_rels")
            continue
        kiss_dual = _same_gid_rels_kiss(rels)
        # H1: same-gid kiss → identity-primary leader only (kiss = lattice period).
        stamp_rels = (
            [_same_gid_leader_rel(rels)] if kiss_dual else list(rels)
        )
        if kiss_dual:
            _bump_skip(skip_reasons, "same_gid_identity_primary")
        for t_anchor in anchors:
            at_pole = _anchor_near_pole(t_anchor, pole_xy)
            # Same-group packing-clear drives emit (restore prior full_clear rate).
            # Foreign co-emit is independent: only when that solid clears (P1 hybrid).
            same_clear = True
            for t_rel in stamp_rels:
                coords = compose_transforms(t_anchor, t_rel)
                if not emit_packing_clear(propose_geom, coords, obstacles=obs):
                    same_clear = False
                    break
            soft_mode = hard_packed_geoms is not None
            if same_clear:
                emitted_members: list[tuple[int, tuple[float, float, float]]] = []
                leader_key = None
                # H1 kiss: same-gid identity leader only; else all same-gid + foreign.
                if kiss_dual:
                    emit_members = [(int(group_id), stamp_rels[0])] + [
                        (int(gid_m), t_rel)
                        for gid_m, t_rel in pat.members
                        if int(gid_m) != int(group_id)
                    ]
                else:
                    emit_members = [
                        (int(gid_m), t_rel) for gid_m, t_rel in pat.members
                    ]
                for gid_i, t_rel in emit_members:
                    coords = compose_transforms(t_anchor, t_rel)
                    if gid_i == int(group_id):
                        if _maybe_add(
                            coords,
                            at_pole=at_pole,
                            same_gid_kiss=kiss_dual,
                            record_collide=True,
                        ):
                            key = transform_row_key(coords)
                            emitted_members.append((gid_i, key))
                            if leader_key is None:
                                leader_key = key
                        continue
                    if (
                        foreign_out is None
                        or part_by_group is None
                        or gid_i not in part_by_group
                    ):
                        continue
                    fstat = _foreign_member_packing_status(
                        part_by_group[gid_i],
                        coords,
                        propose_geom,
                        obstacles=obs,
                    )
                    if fstat != "ok" and soft_mode:
                        fstat_soft = _foreign_member_packing_status(
                            part_by_group[gid_i],
                            coords,
                            propose_geom,
                            obstacles=obs_foreign,
                        )
                        if fstat_soft == "ok":
                            fstat = "ok"
                            _bump_skip(skip_reasons, "foreign_clear_soft_ok")
                    if fstat != "ok":
                        _bump_skip(skip_reasons, f"foreign_clear_fail_{fstat}")
                        continue
                    key = transform_row_key(coords)
                    fseen = (gid_i, key)
                    if fseen in foreign_seen:
                        continue
                    foreign_seen.add(fseen)
                    foreign_out.setdefault(gid_i, []).append(coords)
                    emitted_members.append((gid_i, key))
                    _bump_skip(skip_reasons, "coemit_followers")
                if (
                    cohorts_out is not None
                    and leader_key is not None
                    and len(emitted_members) >= 2
                ):
                    cohorts_out.append({
                        "leader_key": leader_key,
                        "leader_gid": int(group_id),
                        "member_keys": emitted_members,
                        "pattern_sig": _pattern_signature(pat),
                        "motif_id": int(getattr(pat, "motif_id", -1)),
                        "anchor": (
                            float(t_anchor[0]),
                            float(t_anchor[1]),
                            float(t_anchor[2]),
                        ),
                    })
                if emitted_members:
                    _bump_skip(skip_reasons, "full_motif_clear")
                if len(out) >= top_n:
                    return out
                continue
            # H0: one collide subtype record per failed same_clear anchor.
            _record_stamp_collide(
                skip_reasons,
                at_pole=at_pole,
                same_gid_kiss=kiss_dual,
            )
            # Leader-follower fallback: stamp_rels, then identity when densify
            # clear-XY unlock poses are absolute leader cells (A1).
            fallback_rels = list(stamp_rels)
            unlocked = (
                int((skip_reasons or {}).get("clear_xy_ok", 0) or 0) > 0
                or int((skip_reasons or {}).get("clear_xy_slide_ok", 0) or 0) > 0
            )
            if unlocked and not any(
                abs(float(r[0])) <= 1e-9
                and abs(float(r[1])) <= 1e-9
                and abs(float(r[2])) <= 1e-9
                for r in fallback_rels
            ):
                fallback_rels.append((0.0, 0.0, 0.0))
            for t_rel in fallback_rels:
                coords = compose_transforms(t_anchor, t_rel)
                if _maybe_add(
                    coords,
                    at_pole=at_pole,
                    same_gid_kiss=kiss_dual,
                    record_collide=False,
                ):
                    if (
                        abs(float(t_rel[0])) <= 1e-9
                        and abs(float(t_rel[1])) <= 1e-9
                        and abs(float(t_rel[2])) <= 1e-9
                    ):
                        _bump_skip(skip_reasons, "fallback_clear_xy_identity")
                    _bump_skip(skip_reasons, "fallback_leader")
                    if len(out) >= top_n:
                        return out
                else:
                    _bump_skip(skip_reasons, "leader_fail")
    return out


def propose_placements_cluster_copy(
    patterns: Sequence[ClusterPattern],
    group_id: int,
    shape_to_place: Polygon,
    sheet: Polygon,
    base_shape: BaseGeometry,
    *,
    min_dist: float,
    propose_geom: ProposeGeometry,
    pt_push: Point,
    propose_cfg: ProposeConfig,
    top_n: int = 16,
    free_space=None,
    void_pole: Point | None = None,
    skip_reasons: dict[str, int] | None = None,
    cohorts_out: list | None = None,
    part_by_group: dict[int, Polygon] | None = None,
    foreign_out: dict[int, list] | None = None,
    hard_packed_geoms: Sequence | None = None,
    allowed_angles: Sequence[float] | None = None,
) -> List[Tuple[float, float, float]]:
    """Emit absolute transforms for group_id via shared leader-follower stamp.

    F0: pocket + clear pole-ring + one clear-first only on dense packs
    (``n_packed >= 60``, late densify / post-ckpt). Early cold-start unchanged.
    """
    if not patterns or sheet.is_empty:
        return []
    if not propose_cfg.use_cluster_copy:
        return []

    from nest_graph.propose.placement_common import placement_obstacles

    pole = void_pole
    if pole is None and free_space is not None:
        analysis = getattr(free_space, "analysis", None)
        if analysis is not None and getattr(analysis, "target_pt", None) is not None:
            pole = analysis.target_pt
    if pole is None and pt_push is not None and not pt_push.is_empty:
        pole = pt_push

    voids = propose_geom.scene.void_geoms
    full_packed = list(propose_geom.full_packed_geoms or [])
    obs = placement_obstacles(voids, full_packed)
    n_packed = len(full_packed)

    void_poly = _free_space_void_poly(free_space)
    part_for_pocket = None
    densify_clear = (
        bool(getattr(propose_cfg, "motif_use_topo_anchors", True))
        and void_poly is not None
        and shape_to_place is not None
        and not getattr(shape_to_place, "is_empty", True)
        and n_packed >= 60
        and float(void_poly.area) > 4.0 * max(float(shape_to_place.area), 1e-9)
    )
    unlock_shape = shape_to_place
    if densify_clear:
        part_for_pocket = shape_to_place
        # M: pair-union unlock grain/extents only when union fit_probe_ok.
        unlock_shape = motif_pair_union_unlock_shape(
            patterns,
            group_id,
            shape_to_place,
            part_by_group,
            void_poly,
            min_dist=float(min_dist),
            skip_reasons=skip_reasons,
        )

    lattice_stats: dict = {}
    # Baseline topo anchors (no densify pocket) — N4a fallback when clear-XY unlock fails.
    topo_anchors = void_seek_motif_anchors(
        sheet,
        base_shape,
        min_dist=min_dist,
        propose_cfg=propose_cfg,
        free_space=free_space,
        void_pole=pole,
        patterns=patterns,
        lattice_stats_out=None,
        void_poly=None,
        part_for_pocket=None,
        allowed_angles=allowed_angles,
    )
    anchors = void_seek_motif_anchors(
        sheet,
        base_shape,
        min_dist=min_dist,
        propose_cfg=propose_cfg,
        free_space=free_space,
        void_pole=pole,
        patterns=patterns,
        lattice_stats_out=lattice_stats,
        void_poly=void_poly if densify_clear else None,
        part_for_pocket=part_for_pocket,
        allowed_angles=allowed_angles,
    )
    if skip_reasons is not None:
        skip_reasons["lattice_anchors_added"] = int(
            lattice_stats.get("lattice_anchors_added", 0)
        )
        skip_reasons["lattice_anchors_kept"] = int(
            lattice_stats.get("lattice_anchors_kept", 0)
        )
        if part_for_pocket is not None:
            skip_reasons["pocket_align_args"] = 1

    if densify_clear and pole is not None and not getattr(pole, "is_empty", True):
        pole_xy = (float(pole.x), float(pole.y), 0.0)
        if not emit_packing_clear(propose_geom, pole_xy, obstacles=obs):
            max_e = part_extents(shape_to_place)[1]
            ring = _void_pole_ring_anchors(
                (float(pole.x), float(pole.y)),
                radius=float(max_e),
                num=8,
            )
            ring_clear = [
                a for a in ring
                if emit_packing_clear(propose_geom, a, obstacles=obs)
            ]
            if ring_clear:
                _bump_skip(skip_reasons, "pole_ring_clear", len(ring_clear))
                anchors = list(ring_clear) + list(anchors)
            elif ring:
                _bump_skip(skip_reasons, "pole_ring_fail", len(ring))

    if densify_clear and anchors:
        anchors = _order_anchors_clear_first(
            anchors,
            propose_geom=propose_geom,
            obstacles=obs,
            skip_reasons=skip_reasons,
        )
        anchors = _unlock_clear_xy_anchors(
            anchors,
            propose_geom=propose_geom,
            obstacles=obs,
            void_poly=void_poly,
            pole=pole if (pole is not None and not getattr(pole, "is_empty", True)) else None,
            shape_to_place=unlock_shape,
            skip_reasons=skip_reasons,
            allowed_angles=allowed_angles,
            sheet=sheet,
            min_dist=float(min_dist),
            base_shape=base_shape,
        )
        # A2: densify sterile → prepend packing-clear topo only (never unclear).
        if int((skip_reasons or {}).get("clear_xy_none", 0) or 0) > 0 and topo_anchors:
            pole_xy = None
            if pole is not None and not getattr(pole, "is_empty", True):
                pole_xy = (float(pole.x), float(pole.y))
            off_pole = [
                a for a in topo_anchors
                if not _anchor_near_pole(a, pole_xy)
            ]
            use = off_pole if off_pole else list(topo_anchors)
            if skip_reasons is not None:
                skip_reasons["topo_off_pole_n"] = int(len(off_pole))
                skip_reasons["topo_near_pole_n"] = int(
                    max(0, len(topo_anchors) - len(off_pole))
                )
            clear_topo = [
                a for a in use
                if emit_packing_clear(propose_geom, a, obstacles=obs)
            ]
            if clear_topo:
                _bump_skip(skip_reasons, "clear_xy_topo_fallback_clear", len(clear_topo))
                _bump_skip(skip_reasons, "clear_xy_topo_fallback", len(clear_topo))
                anchors = list(clear_topo) + list(anchors)
            else:
                # A2 honest: never prepend unclear; telem stays unclear=0 (not used).
                _bump_skip(skip_reasons, "clear_xy_topo_fallback_refuse", len(use))

    if densify_clear and void_poly is not None and shape_to_place is not None:
        # A0/T0: one fit probe — void inradius + obstacle clearance vs unlock shape.
        try:
            poi = polylabel(void_poly, tolerance=max(float(min_dist), 0.5))
            px, py = float(poi.x), float(poi.y)
            in_r = float(void_poly.exterior.distance(Point(px, py)))
            obs_r = in_r
            # Obstacle clearance at polylabel (native Geometry; densify telem path).
            probe = Geometry.from_ring(
                ((px, py), (px + 1e-6, py), (px, py + 1e-6)),
            )
            for og in full_packed:
                if og is None:
                    continue
                try:
                    d = float(og.distance(probe))
                except Exception:
                    continue
                if d < obs_r:
                    obs_r = d
            eff_r = min(float(in_r), float(obs_r))
            _min_e, max_e = part_extents(unlock_shape)
            if skip_reasons is not None:
                skip_reasons["fit_void_inradius"] = int(round(in_r * 1000.0))
                skip_reasons["fit_obs_inradius"] = int(round(obs_r * 1000.0))
                skip_reasons["fit_part_max_extent"] = int(round(float(max_e) * 1000.0))
                if float(max_e) > eff_r + 1e-9:
                    skip_reasons["fit_probe_fail"] = 1
                else:
                    skip_reasons["fit_probe_ok"] = 1
        except Exception:
            if skip_reasons is not None:
                skip_reasons["fit_probe_fail"] = 1
        # T0/V: 32 xy×grain packing-clear rate (wedge grain first).
        grain_angles: list[float] = list(wedge_edge_grain_angles(void_poly))
        if grain_angles and skip_reasons is not None:
            skip_reasons["wedge_grain_n"] = int(
                skip_reasons.get("wedge_grain_n", 0) or 0
            ) + int(len(grain_angles))
        if allowed_angles:
            grain_angles.extend(float(a) for a in allowed_angles)
        else:
            try:
                grain_angles.extend(
                    float(a) for a in _boundary_alignment_angles(unlock_shape)
                )
            except Exception:
                pass
        if not grain_angles:
            grain_angles = [0.0, math.pi / 2.0]
        # Dedupe preserve order.
        seen_g: set[float] = set()
        uniq_g: list[float] = []
        for a in grain_angles:
            key = round(float(a), 6)
            if key in seen_g:
                continue
            seen_g.add(key)
            uniq_g.append(float(a))
        grain_angles = uniq_g
        n_clear, n_try = _void_clear_rate_sample(
            void_poly,
            propose_geom=propose_geom,
            obstacles=obs,
            angles=grain_angles,
            n_xy=32,
        )
        if skip_reasons is not None and n_try > 0:
            skip_reasons["void_clear_rate"] = int(round(1000.0 * n_clear / n_try))
            skip_reasons["void_clear_try"] = int(n_try)
            skip_reasons["void_clear_ok"] = int(n_clear)

    return stamp_motif_leader_follower(
        patterns,
        group_id,
        shape_to_place,
        propose_geom=propose_geom,
        anchors=anchors,
        top_n=top_n,
        part_by_group=part_by_group,
        skip_reasons=skip_reasons,
        cohorts_out=cohorts_out,
        foreign_out=foreign_out,
        hard_packed_geoms=hard_packed_geoms,
        void_pole=pole,
    )
