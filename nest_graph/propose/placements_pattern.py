"""Rigid cluster-copy proposer: reuse packed motifs elsewhere on the sheet."""

from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

from shapely import MultiPolygon, Polygon
from shapely.geometry import Point
from shapely.geometry.base import BaseGeometry
from shapely.ops import unary_union

from nest_graph.config import ProposeConfig
from nest_graph.propose.context import cluster_packed_indices, placement_free_region
from nest_graph.propose.geometry import ProposeGeometry
from nest_graph.propose.void_topology import polylabel
from nest_graph.utils import compose_transforms, relative_transform, transform_poly


@dataclass(frozen=True)
class ClusterPattern:
    """Relative SE(2) motif extracted from a packed cluster."""

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
        # Reference = largest-area member
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


def free_pocket_anchors(
    sheet: Polygon,
    obstacle: BaseGeometry,
    min_dist: float,
    max_anchors: int,
) -> list[tuple[float, float, float]]:
    """Polylabel / multi-pole anchors in free pockets of sheet\\obstacle."""
    from nest_graph.propose.void_topology import multi_pole_seed_coords

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
            out.append((float(pt.x), float(pt.y), float(__import__("math").pi)))
        if len(out) >= max_anchors * 2:
            break
    return out[: max_anchors * 2]


# Back-compat private alias.
_free_pocket_anchors = free_pocket_anchors


def _mirror_anchors(
    ref: tuple[float, float, float],
    sheet: Polygon,
) -> list[tuple[float, float, float]]:
    minx, miny, maxx, maxy = sheet.bounds
    cx = 0.5 * (minx + maxx)
    cy = 0.5 * (miny + maxy)
    x, y, a = ref
    return [
        (2.0 * cx - x, y, -a),
        (x, 2.0 * cy - y, -a),
        (2.0 * cx - x, 2.0 * cy - y, a + float(__import__("math").pi)),
    ]


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
) -> List[Tuple[float, float, float]]:
    """Emit absolute transforms for group_id by placing pattern members at new anchors."""
    if not patterns or sheet.is_empty:
        return []
    if not propose_cfg.use_cluster_copy:
        return []

    anchors: list[tuple[float, float, float]] = []
    anchors.extend(
        _free_pocket_anchors(
            sheet, base_shape, min_dist, propose_cfg.cluster_copy_anchor_seeds,
        )
    )
    # Prefer snapshot topology poles; fall back to a one-shot scan.
    if free_space is not None and getattr(free_space, "topology_poles", None):
        anchors.extend(list(free_space.topology_poles))
    else:
        packed_for_topo = []
        if hasattr(base_shape, "geoms"):
            packed_for_topo = [
                g for g in base_shape.geoms if g is not None and not g.is_empty
            ]
        elif base_shape is not None and not base_shape.is_empty:
            packed_for_topo = [base_shape]
        if packed_for_topo:
            from nest_graph.propose.void_topology import topology_pocket_poles
            anchors.extend(
                topology_pocket_poles(
                    sheet,
                    packed_for_topo,
                    min_dist=min_dist,
                    max_anchors=propose_cfg.cluster_copy_anchor_seeds,
                )
            )
    for pat in patterns:
        anchors.extend(_mirror_anchors(pat.ref_transform, sheet))

    seen: set[tuple[float, float, float]] = set()
    out: list[tuple[float, float, float]] = []
    for pat in patterns:
        rels = [t_rel for gid, t_rel in pat.members if gid == group_id]
        if not rels:
            continue
        for t_anchor in anchors:
            for t_rel in rels:
                coords = compose_transforms(t_anchor, t_rel)
                key = (round(coords[0], 2), round(coords[1], 2), round(coords[2], 2))
                if key in seen:
                    continue
                seen.add(key)
                if not propose_geom.valid_at(coords, pt_push):
                    continue
                # Cheap full-motif check: every member must stay in sheet and clear voids.
                motif_ok = True
                for gid, t_rel_m in pat.members:
                    t_m = compose_transforms(t_anchor, t_rel_m)
                    if gid == group_id:
                        placed_m = transform_poly(shape_to_place, t_m)
                    else:
                        # Other members: only require footprint inside sheet AABB.
                        placed_m = Point(t_m[0], t_m[1]).buffer(min_dist)
                    if not sheet.buffer(1e-5).covers(placed_m.centroid):
                        motif_ok = False
                        break
                if not motif_ok:
                    continue
                out.append(coords)
                if len(out) >= top_n:
                    return out
    return out
