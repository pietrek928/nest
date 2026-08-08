"""Pocket-fit proposer: trapped voids, hull bays, MRR/triangle align, motif holes."""

import math
from typing import List, Sequence, Tuple

from shapely import Polygon
from shapely.geometry import Point
from shapely.geometry.base import BaseGeometry
from shapely.ops import polylabel, triangulate, unary_union

from nest_graph.config import ProposeConfig
from nest_graph.propose.context import (
    FreeSpaceSnapshot,
    cluster_packed_indices,
    iter_polygons,
    part_extents,
)
from nest_graph.propose.geometry import ProposeGeometry
from nest_graph.propose.placements_pattern import ClusterPattern
from nest_graph.propose.void_topology import (
    hull_bay_polygons,
    topology_pocket_poles,
    touches_sheet_exterior,
    trapped_void_polygons,
)
from nest_graph.utils import (
    compose_transforms,
    invert_transform,
    relative_transform,
    transform_poly,
)

TAG_POCKET_MRR = "pocket_mrr"
TAG_POCKET_TRIANGLE = "pocket_triangle"
TAG_MOTIF_HOLE = "motif_hole"
TAG_OPEN_VOID = "open_void"

_ANGLE_SNAP_MAX = 0.15
_REL_EPS = 1e-3
_SQUARE_ASPECT = 1.15
_L_SHAPE_MRR_AREA = 0.6

# Back-compat alias for callers that used the private name.
_touches_sheet_exterior = touches_sheet_exterior


def mrr_major_axis_angle(poly: Polygon) -> float:
    """Orientation (radians) of the longest MRR edge, mod π."""
    mrr = poly.minimum_rotated_rectangle
    coords = list(mrr.exterior.coords)
    if len(coords) < 3:
        return 0.0
    dx1, dy1 = coords[1][0] - coords[0][0], coords[1][1] - coords[0][1]
    dx2, dy2 = coords[2][0] - coords[1][0], coords[2][1] - coords[1][1]
    len1 = math.hypot(dx1, dy1)
    len2 = math.hypot(dx2, dy2)
    if len1 >= len2:
        return math.atan2(dy1, dx1) % math.pi
    return math.atan2(dy2, dx2) % math.pi


def mrr_aspect_ratio(poly: Polygon) -> float:
    mrr = poly.minimum_rotated_rectangle
    coords = list(mrr.exterior.coords)
    if len(coords) < 3:
        return 1.0
    len1 = math.hypot(coords[1][0] - coords[0][0], coords[1][1] - coords[0][1])
    len2 = math.hypot(coords[2][0] - coords[1][0], coords[2][1] - coords[1][1])
    a, b = max(len1, len2), min(len1, len2)
    return a / max(b, 1e-12)


def _triangle_longest_edge_angle(tri: Polygon) -> float:
    coords = list(tri.exterior.coords)[:3]
    if len(coords) < 3:
        return 0.0
    best = 0.0
    best_len = -1.0
    for i in range(3):
        j = (i + 1) % 3
        dx = coords[j][0] - coords[i][0]
        dy = coords[j][1] - coords[i][1]
        L = math.hypot(dx, dy)
        if L > best_len:
            best_len = L
            best = math.atan2(dy, dx) % math.pi
    return best


def pocket_target(
    void_poly: Polygon,
    *,
    min_dist: float,
    relax_erosion: bool = False,
) -> tuple[Point, float, str] | None:
    """Return (poi, align_angle, tag) for a void/bay polygon."""
    if void_poly is None or void_poly.is_empty:
        return None
    eroded = void_poly.buffer(-float(min_dist))
    if eroded is None or eroded.is_empty:
        if not relax_erosion:
            return None
        # Open void / narrow clip: still pick a POI on the uneroded poly.
        try:
            poi = polylabel(void_poly, tolerance=max(float(min_dist), 0.1))
        except Exception:
            poi = void_poly.representative_point()
        if poi is None or poi.is_empty:
            return None
        if mrr_aspect_ratio(void_poly) < _SQUARE_ASPECT:
            return poi, 0.0, TAG_POCKET_MRR
        return poi, float(mrr_major_axis_angle(void_poly)), TAG_POCKET_MRR
    mrr = void_poly.minimum_rotated_rectangle
    mrr_area = float(mrr.area) if mrr is not None and not mrr.is_empty else 0.0
    use_tri = mrr_area > 1e-12 and float(void_poly.area) < _L_SHAPE_MRR_AREA * mrr_area
    if use_tri:
        try:
            tris = [
                g for g in triangulate(void_poly)
                if isinstance(g, Polygon) and not g.is_empty
            ]
        except Exception:
            tris = []
        tris.sort(key=lambda t: float(t.area), reverse=True)
        if not tris:
            return None
        tri = tris[0]
        try:
            poi = polylabel(tri, tolerance=max(float(min_dist) * 0.5, 0.05))
        except Exception:
            poi = tri.representative_point()
        angle = _triangle_longest_edge_angle(tri)
        tag = TAG_POCKET_TRIANGLE
    else:
        try:
            poi = polylabel(void_poly, tolerance=max(float(min_dist), 0.1))
        except Exception:
            poi = void_poly.representative_point()
        if mrr_aspect_ratio(void_poly) < _SQUARE_ASPECT:
            angle = 0.0  # ignore MRR; offsets only
            tag = TAG_POCKET_MRR
        else:
            angle = mrr_major_axis_angle(void_poly)
            tag = TAG_POCKET_MRR
    if poi is None or poi.is_empty:
        return None
    return poi, float(angle), tag


def se2_centroid_at_target(
    part: Polygon,
    tx: float,
    ty: float,
    delta_theta: float,
) -> tuple[float, float, float]:
    """SE(2) so transform_poly places part.centroid at (tx, ty) with rotation delta_theta."""
    c = part.centroid
    cx, cy = float(c.x), float(c.y)
    cos_t, sin_t = math.cos(delta_theta), math.sin(delta_theta)
    x = tx - (cx * cos_t - cy * sin_t)
    y = ty - (cx * sin_t + cy * cos_t)
    theta = delta_theta % (2.0 * math.pi)
    return (x, y, theta)


def _snap_angle(
    theta: float,
    allowed: Sequence[float] | None,
) -> float | None:
    if not allowed:
        return float(theta) % (2.0 * math.pi)
    two_pi = 2.0 * math.pi
    t = float(theta) % two_pi
    best = None
    best_d = float("inf")
    for a in allowed:
        aa = float(a) % two_pi
        d = abs(t - aa)
        d = min(d, two_pi - d)
        if d < best_d:
            best_d = d
            best = aa
    if best is None or best_d > _ANGLE_SNAP_MAX:
        return None
    return best


def _cardinal_allowed_angles(
    allowed: Sequence[float] | None,
    *,
    limit: int = 4,
) -> list[float]:
    """Up to ``limit`` distinct allowed angles (for snap-fail fallback)."""
    if not allowed:
        return [0.0, 0.5 * math.pi, math.pi, 1.5 * math.pi][:limit]
    two_pi = 2.0 * math.pi
    out: list[float] = []
    seen: set[float] = set()
    for a in allowed:
        aa = float(a) % two_pi
        key = round(aa, 6)
        if key in seen:
            continue
        seen.add(key)
        out.append(aa)
        if len(out) >= limit:
            break
    return out


def aligned_poses_for_pocket(
    part: Polygon,
    void_poly: Polygon,
    *,
    min_dist: float,
    allowed_angles: Sequence[float] | None = None,
    relax_medial: bool = False,
    max_poses: int | None = None,
) -> list[tuple[tuple[float, float, float], str]]:
    """MRR/triangle-aligned SE(2) proposals for one pocket."""
    part_min, _ = part_extents(part)
    tgt = pocket_target(
        void_poly, min_dist=min_dist, relax_erosion=relax_medial,
    )
    if tgt is None:
        return []
    poi, void_angle, tag = tgt
    if (
        not relax_medial
        and float(poi.distance(void_poly.exterior)) < (part_min / 2.0) + float(min_dist)
    ):
        return []
    area_ratio = float(void_poly.area) / max(float(part.area), 1e-12)
    # Caller may already filter; keep soft check.
    if area_ratio < 0.5:
        return []

    squareish = (not tag == TAG_POCKET_TRIANGLE) and mrr_aspect_ratio(void_poly) < _SQUARE_ASPECT
    part_angle = 0.0 if squareish else mrr_major_axis_angle(part)
    offsets = (0.0, 0.5 * math.pi, math.pi, 1.5 * math.pi)
    out: list[tuple[tuple[float, float, float], str]] = []
    seen: set[tuple[float, float, float]] = set()
    pose_cap = max_poses if max_poses is not None else 10**9

    def _emit(theta: float) -> None:
        if len(out) >= pose_cap:
            return
        coords = se2_centroid_at_target(part, float(poi.x), float(poi.y), theta)
        key = (round(coords[0], 4), round(coords[1], 4), round(coords[2], 4))
        if key in seen:
            return
        seen.add(key)
        out.append((coords, tag))

    any_snap = False
    for k_off in offsets:
        if squareish:
            delta = k_off
        else:
            delta = (void_angle - part_angle + k_off)
        ideal = delta % (2.0 * math.pi)
        snapped = _snap_angle(ideal, allowed_angles)
        if snapped is None:
            continue
        any_snap = True
        _emit(snapped)
        if len(out) >= pose_cap:
            return out
    if not any_snap and allowed_angles:
        # Snap failed: emit allowed cardinals at the same pocket XY.
        for theta in _cardinal_allowed_angles(allowed_angles):
            _emit(theta)
            if len(out) >= pose_cap:
                break
    return out


def _angle_diff(a: float, b: float) -> float:
    two_pi = 2.0 * math.pi
    d = abs((float(a) - float(b)) % two_pi)
    return min(d, two_pi - d)


def _rel_close(
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    eps: float = _REL_EPS,
) -> bool:
    return (
        abs(a[0] - b[0]) < eps
        and abs(a[1] - b[1]) < eps
        and _angle_diff(a[2], b[2]) < eps
    )


def _rel_residual(
    a: tuple[float, float, float],
    b: tuple[float, float, float],
) -> float:
    return abs(a[0] - b[0]) + abs(a[1] - b[1]) + _angle_diff(a[2], b[2])


def propose_motif_hole_fills(
    patterns: Sequence[ClusterPattern],
    packed: Sequence[BaseGeometry],
    packed_group_ids: Sequence[int],
    packed_transforms: Sequence,
    group_id: int,
    *,
    min_dist: float,
    sheet: Polygon | None = None,
) -> list[tuple[tuple[float, float, float], str]]:
    """Complete near-complete motifs missing a member of ``group_id``."""
    if not patterns or len(packed) < 2:
        return []
    n = min(len(packed), len(packed_group_ids), len(packed_transforms))
    if n < 2:
        return []
    gids = [int(packed_group_ids[i]) for i in range(n)]
    trs = [
        (
            float(packed_transforms[i][0]),
            float(packed_transforms[i][1]),
            float(packed_transforms[i][2]),
        )
        for i in range(n)
    ]
    index_groups = cluster_packed_indices(list(packed[:n]), min_dist, sheet=sheet)
    out: list[tuple[tuple[float, float, float], str]] = []
    seen: set[tuple[float, float, float]] = set()

    for pat in patterns:
        missing_rels = [t_rel for gid, t_rel in pat.members if gid == group_id]
        if len(missing_rels) != 1:
            continue
        t_rel_missing = missing_rels[0]
        other_members = [(gid, t_rel) for gid, t_rel in pat.members if gid != group_id]
        if len(other_members) < 2:
            continue

        for idxs in index_groups:
            if len(idxs) < 2:
                continue
            best_pair_res = float("inf")
            best_anchor = None
            for ia in idxs:
                for ib in idxs:
                    if ia == ib:
                        continue
                    for gid_a, t_rel_a in other_members:
                        if gids[ia] != gid_a:
                            continue
                        for gid_b, t_rel_b in other_members:
                            if gid_a == gid_b or gids[ib] != gid_b:
                                continue
                            rel_obs = relative_transform(trs[ia], trs[ib])
                            rel_exp = compose_transforms(
                                invert_transform(t_rel_a), t_rel_b,
                            )
                            res = _rel_residual(rel_obs, rel_exp)
                            if res >= best_pair_res:
                                continue
                            if not _rel_close(rel_obs, rel_exp, eps=5e-3):
                                continue
                            # pose_a = anchor ∘ t_rel_a ⇒ anchor = pose_a ∘ invert(t_rel_a)
                            best_pair_res = res
                            best_anchor = compose_transforms(
                                trs[ia], invert_transform(t_rel_a),
                            )

            if best_anchor is None:
                continue
            coords = compose_transforms(best_anchor, t_rel_missing)
            key = (round(coords[0], 4), round(coords[1], 4), round(coords[2], 4))
            if key in seen:
                continue
            seen.add(key)
            out.append((coords, TAG_MOTIF_HOLE))
    return out


def propose_placements_pocket_fit(
    shape_to_place: Polygon,
    sheet: Polygon,
    packed: Sequence[BaseGeometry],
    *,
    min_dist: float,
    propose_geom: ProposeGeometry,
    pt_push: Point,
    propose_cfg: ProposeConfig,
    group_id: int = 0,
    allowed_angles: Sequence[float] | None = None,
    cluster_patterns: Sequence[ClusterPattern] | None = None,
    packed_group_ids: Sequence[int] | None = None,
    packed_transforms: Sequence | None = None,
    top_n: int = 16,
    tags_out: list[str] | None = None,
    attempts_out: list[int] | None = None,
    free_space: FreeSpaceSnapshot | None = None,
    skip_reasons_out: list[str] | None = None,
) -> List[Tuple[float, float, float]]:
    """Emit SE(2) teleports into trapped voids / hull bays + motif holes."""
    if not propose_cfg.use_pocket_fit or sheet is None or sheet.is_empty:
        return []
    area_ratio = float(propose_cfg.pocket_fit_area_ratio)
    max_targets = int(propose_cfg.pocket_fit_max_targets)
    part_area = float(shape_to_place.area)
    skips: list[str] = []

    if free_space is not None:
        voids = list(free_space.trapped_voids)
        bays = list(free_space.hull_bays)
        analysis = free_space.analysis
    else:
        voids = trapped_void_polygons(sheet, packed)
        bays = hull_bay_polygons(packed, min_dist=min_dist, sheet=sheet)
        analysis = None
    if not voids:
        skips.append("no_trapped")
    # Drop oversized open bays (hull-diff can dwarf the part); keep trapped voids.
    max_bay = part_area * 5.0
    n_bays_raw = len(bays)
    bays = [p for p in bays if float(p.area) <= max_bay]
    if n_bays_raw > 0 and not bays:
        skips.append("hull_bays_oversized")
    pockets = [p for p in (voids + bays) if float(p.area) >= part_area * area_ratio]
    pockets.sort(key=lambda p: float(p.area), reverse=True)
    pockets = pockets[:max_targets]

    # Separate channel: exterior-touching Mode A target (do not mutate trapped_voids).
    open_voids: list[Polygon] = []
    if (
        bool(getattr(propose_cfg, "use_open_void_pocket", True))
        and analysis is not None
        and analysis.kind == "large_void"
        and analysis.target_poly is not None
        and not analysis.target_poly.is_empty
        and _touches_sheet_exterior(analysis.target_poly, sheet)
        and float(analysis.target_poly.area) >= part_area * area_ratio
    ):
        # Cap like hull bays when the free region dwarfs the part; otherwise
        # teleport POI is still the polylabel of the (possibly clipped) poly.
        ov = analysis.target_poly
        if float(ov.area) > max_bay:
            try:
                poi = polylabel(ov, tolerance=max(float(min_dist), 0.1))
            except Exception:
                poi = ov.representative_point()
            # Local window ~sqrt(5×part) around POI, clipped to the free void.
            half = max(math.sqrt(max_bay) * 0.5, float(min_dist) * 4.0)
            window = Point(float(poi.x), float(poi.y)).buffer(half)
            clipped = ov.intersection(window)
            for g in iter_polygons(clipped):
                if float(g.area) >= part_area * area_ratio:
                    open_voids.append(g)
                    break
            if not open_voids:
                # Prefer a slightly larger clip before falling back to full ov.
                half2 = half * 1.5
                window2 = Point(float(poi.x), float(poi.y)).buffer(half2)
                clipped2 = ov.intersection(window2)
                for g in iter_polygons(clipped2):
                    if float(g.area) >= part_area * area_ratio:
                        open_voids.append(g)
                        break
            if not open_voids:
                open_voids.append(ov)
                skips.append("open_void_too_narrow")
        else:
            open_voids.append(ov)
    elif (
        bool(getattr(propose_cfg, "use_open_void_pocket", True))
        and analysis is not None
        and analysis.kind == "large_void"
        and (analysis.target_poly is None or analysis.target_poly.is_empty)
    ):
        skips.append("open_void_empty")

    # Cap open-void windows to top-3 to avoid mouth flood.
    open_voids = open_voids[:3]

    tagged: list[tuple[tuple[float, float, float], str]] = []
    for void_poly in pockets:
        tagged.extend(
            aligned_poses_for_pocket(
                shape_to_place,
                void_poly,
                min_dist=min_dist,
                allowed_angles=allowed_angles,
            )
        )
    open_void_align_n = 0
    for void_poly in open_voids:
        poses = aligned_poses_for_pocket(
            shape_to_place,
            void_poly,
            min_dist=min_dist,
            allowed_angles=allowed_angles,
            relax_medial=True,
            max_poses=3,
        )
        if not poses:
            continue
        open_void_align_n += len(poses)
        for coords, _tag in poses:
            tagged.append((coords, TAG_OPEN_VOID))
    if open_voids and open_void_align_n == 0:
        skips.append("open_void_align_empty")

    if (
        cluster_patterns
        and packed_group_ids is not None
        and packed_transforms is not None
    ):
        tagged.extend(
            propose_motif_hole_fills(
                cluster_patterns,
                packed,
                packed_group_ids,
                packed_transforms,
                group_id,
                min_dist=min_dist,
                sheet=sheet,
            )
        )

    out: list[tuple[float, float, float]] = []
    seen: set[tuple[float, float, float]] = set()
    attempted = 0
    for coords, tag in tagged:
        key = (round(coords[0], 4), round(coords[1], 4), round(coords[2], 4))
        if key in seen:
            continue
        seen.add(key)
        attempted += 1
        if not propose_geom.valid_at(coords, pt_push):
            continue
        out.append(coords)
        if tags_out is not None:
            tags_out.append(tag)
        if len(out) >= top_n:
            break
    if attempts_out is not None:
        attempts_out.append(attempted)
    if skip_reasons_out is not None:
        skip_reasons_out.extend(skips)
    return out
