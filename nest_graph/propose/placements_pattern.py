"""Rigid cluster-copy proposer: reuse packed motifs elsewhere on the sheet."""

from dataclasses import dataclass
import math
from typing import List, Optional, Sequence, Tuple

from shapely import MultiPolygon, Polygon
from shapely.geometry import Point
from shapely.geometry.base import BaseGeometry
from shapely.ops import unary_union

from nest_graph.config import ProposeConfig
from nest_graph.propose.context import (
    cluster_packed_indices,
    placement_free_region,
    void_pole_seed_coords,
)
from nest_graph.propose.geometry import ProposeGeometry
from nest_graph.propose.void_topology import (
    multi_pole_seed_coords,
    polylabel,
    topology_pocket_poles,
)
from nest_graph.utils import compose_transforms, relative_transform, transform_poly


@dataclass(frozen=True)
class ClusterPattern:
    """Relative SE(2) motif extracted from a packed contact cluster."""

    members: tuple[tuple[int, tuple[float, float, float]], ...]
    part_count: int
    ref_transform: tuple[float, float, float]


def extract_cluster_patterns(
    placed: Sequence[BaseGeometry],
    group_ids: Sequence[int],
    transforms: Sequence[tuple[float, float, float] | Sequence[float]],
    *,
    min_dist: float,
    max_patterns: int = 2,
    min_members: int = 2,
    sheet: Polygon | None = None,
) -> list[ClusterPattern]:
    """Build rigid motifs from contact-connected packed clusters."""
    if len(placed) < min_members:
        return []
    n = min(len(placed), len(group_ids), len(transforms))
    if n < min_members:
        return []

    index_groups = cluster_packed_indices(list(placed[:n]), min_dist, sheet=sheet)
    scored: list[tuple[float, list[int]]] = []
    for idxs in index_groups:
        if len(idxs) < min_members:
            continue
        area = sum(float(placed[i].area) for i in idxs)
        scored.append((area, idxs))
    scored.sort(key=lambda x: x[0], reverse=True)

    patterns: list[ClusterPattern] = []
    for _area, idxs in scored[:max_patterns]:
        ref_i = max(idxs, key=lambda i: float(placed[i].area))
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
        patterns.append(
            ClusterPattern(
                members=tuple(members),
                part_count=len(members),
                ref_transform=ref_t,
            )
        )
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
        return float(unary_union([a, b]).convex_hull.area)
    except Exception:
        return float("inf")


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


def merge_cluster_patterns(
    contact: Sequence[ClusterPattern],
    synthesized: Sequence[ClusterPattern],
    *,
    max_patterns: int,
) -> list[ClusterPattern]:
    """Prefer contact patterns; fill remaining slots with synthesized mates."""
    out = list(contact)
    if len(out) >= max(int(max_patterns), 1):
        return out[: max(int(max_patterns), 1)]
    for pat in synthesized:
        out.append(pat)
        if len(out) >= max(int(max_patterns), 1):
            break
    return out


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
) -> list[tuple[float, float, float]]:
    """Unified anchor priority for void_seek motif stamps (§4).

    topology / void_pole → topology_pocket scan → free_pocket → mirror last.
    """
    anchors: list[tuple[float, float, float]] = []
    n_seed = int(propose_cfg.cluster_copy_anchor_seeds)

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

    for pat in patterns:
        anchors.extend(_mirror_anchors(pat.ref_transform, sheet))

    return _dedupe_anchors(anchors)


def emit_packing_clear(
    propose_geom: ProposeGeometry,
    coords: tuple[float, float, float],
) -> bool:
    """Propose-emit packing SoT: Penetrating vs voids+packed (margin 0, not Scene)."""
    from nest_graph.propose.placement_common import placement_obstacles

    placed = propose_geom.placed_at(coords)
    if placed is None:
        return False
    obs = placement_obstacles(
        propose_geom.scene.void_geoms,
        propose_geom.full_packed_geoms,
    )
    if not obs:
        return True
    return not placed.intersects_any(obs)


def _full_motif_packing_clear(
    pat: ClusterPattern,
    t_anchor: tuple[float, float, float],
    group_id: int,
    shape_to_place: Polygon,
    propose_geom: ProposeGeometry,
    part_by_group: dict[int, Polygon] | None,
) -> bool:
    """True if every motif member is packing-clear at ``t_anchor``."""
    for gid, t_rel_m in pat.members:
        t_m = compose_transforms(t_anchor, t_rel_m)
        if int(gid) == int(group_id):
            if not emit_packing_clear(propose_geom, t_m):
                return False
            continue
        if part_by_group is not None and int(gid) in part_by_group:
            # Other groups: centroid must stay on board (no foreign ProposeGeometry).
            placed_m = transform_poly(part_by_group[int(gid)], t_m)
            if placed_m is None or placed_m.is_empty:
                return False
            if not propose_geom.sheet.buffer(1e-5).covers(placed_m.centroid):
                return False
        else:
            # Unknown foreign solid: require leader-cell clear only (leader path).
            continue
    return True


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
) -> list[tuple[float, float, float]]:
    """Propose-side motif stamp: full-motif packing clear, else same-group leader only.

    Emits single-group candidate transforms under packing clearance (not Scene
    margin). Selection/repack uses ``stamp_motif_at_anchor`` (atomic peeled
    placement under ``is_pose_clear``) with ``pattern_fallback`` as the partial path.
    """
    if not patterns or not anchors:
        if skip_reasons is not None:
            skip_reasons["no_anchors" if not anchors else "no_patterns"] = (
                skip_reasons.get("no_anchors" if not anchors else "no_patterns", 0) + 1
            )
        return []

    seen: set[tuple[float, float, float]] = set()
    out: list[tuple[float, float, float]] = []

    def _maybe_add(coords: tuple[float, float, float]) -> bool:
        key = (round(coords[0], 2), round(coords[1], 2), round(coords[2], 2))
        if key in seen:
            return False
        seen.add(key)
        if not emit_packing_clear(propose_geom, coords):
            if skip_reasons is not None:
                skip_reasons["collide"] = skip_reasons.get("collide", 0) + 1
            return False
        out.append(coords)
        return True

    for pat in patterns:
        rels = [t_rel for gid, t_rel in pat.members if int(gid) == int(group_id)]
        if not rels:
            if skip_reasons is not None:
                skip_reasons["no_rels"] = skip_reasons.get("no_rels", 0) + 1
            continue
        for t_anchor in anchors:
            for t_rel in rels:
                coords = compose_transforms(t_anchor, t_rel)
                if _full_motif_packing_clear(
                    pat, t_anchor, group_id, shape_to_place, propose_geom, part_by_group,
                ):
                    if _maybe_add(coords) and len(out) >= top_n:
                        return out
                    continue
                # Leader-follower fallback: same-group member only.
                if _maybe_add(coords):
                    if skip_reasons is not None:
                        skip_reasons["fallback_leader"] = (
                            skip_reasons.get("fallback_leader", 0) + 1
                        )
                    if len(out) >= top_n:
                        return out
                else:
                    if skip_reasons is not None:
                        skip_reasons["leader_fail"] = skip_reasons.get("leader_fail", 0) + 1
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
) -> List[Tuple[float, float, float]]:
    """Emit absolute transforms for group_id via shared leader-follower stamp."""
    if not patterns or sheet.is_empty:
        return []
    if not propose_cfg.use_cluster_copy:
        return []

    pole = void_pole
    if pole is None and free_space is not None:
        analysis = getattr(free_space, "analysis", None)
        if analysis is not None and getattr(analysis, "target_pt", None) is not None:
            pole = analysis.target_pt
    if pole is None and pt_push is not None and not pt_push.is_empty:
        pole = pt_push

    anchors = void_seek_motif_anchors(
        sheet,
        base_shape,
        min_dist=min_dist,
        propose_cfg=propose_cfg,
        free_space=free_space,
        void_pole=pole,
        patterns=patterns,
    )
    return stamp_motif_leader_follower(
        patterns,
        group_id,
        shape_to_place,
        propose_geom=propose_geom,
        anchors=anchors,
        top_n=top_n,
        skip_reasons=skip_reasons,
    )
