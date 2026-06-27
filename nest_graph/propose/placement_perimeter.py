import math

import numpy as np
from shapely import LineString, LinearRing, Point, Polygon
from shapely.geometry.base import BaseGeometry

from nest_graph.utils import get_shape_exteriors

_PERIMETER_VERTEX_CAP = 200


def _point_segment_distance_sq(
    px: float,
    py: float,
    x0: float,
    y0: float,
    x1: float,
    y1: float,
) -> tuple[float, float, float, float]:
    """Return (dist_sq, closest_x, closest_y, t) with t in [0, 1] along the segment."""
    dx = x1 - x0
    dy = y1 - y0
    seg_len_sq = dx * dx + dy * dy
    if seg_len_sq < 1e-18:
        dist_sq = (px - x0) ** 2 + (py - y0) ** 2
        return dist_sq, x0, y0, 0.0
    t = ((px - x0) * dx + (py - y0) * dy) / seg_len_sq
    t = max(0.0, min(1.0, t))
    cx = x0 + t * dx
    cy = y0 + t * dy
    dist_sq = (px - cx) ** 2 + (py - cy) ** 2
    return dist_sq, cx, cy, t


def perimeter_ring_vertices(ring: LineString | LinearRing) -> list[tuple[float, float]]:
    if ring.length < 1e-5:
        return []
    coords = list(ring.coords)
    if len(coords) > 1 and coords[0] == coords[-1]:
        coords = coords[:-1]
    if len(coords) <= _PERIMETER_VERTEX_CAP:
        return [(float(x), float(y)) for x, y in coords]
    stride = max(1, len(coords) // _PERIMETER_VERTEX_CAP)
    sampled = coords[::stride]
    return [(float(x), float(y)) for x, y in sampled]


def _exterior_segment_lengths(sheet: Polygon) -> list[float]:
    coords = list(sheet.exterior.coords)
    seg_lens: list[float] = []
    for i in range(len(coords) - 1):
        p0, p1 = coords[i], coords[i + 1]
        seg_lens.append(math.hypot(p1[0] - p0[0], p1[1] - p0[1]))
    return seg_lens


def _segment_index_at_point(sheet: Polygon, contact: Point) -> int:
    coords = list(sheet.exterior.coords)
    cx, cy = float(contact.x), float(contact.y)
    best_i = 0
    best_dist = float("inf")
    for i in range(len(coords) - 1):
        p0 = coords[i]
        p1 = coords[i + 1]
        dist_sq, _, _, _ = _point_segment_distance_sq(
            cx, cy, float(p0[0]), float(p0[1]), float(p1[0]), float(p1[1]),
        )
        if dist_sq < best_dist:
            best_dist = dist_sq
            best_i = i
    return best_i


def _edge_tangent_angles(sheet: Polygon, segment_index: int) -> list[float]:
    coords = list(sheet.exterior.coords)
    if segment_index < 0 or segment_index >= len(coords) - 1:
        return []
    p0, p1 = coords[segment_index], coords[segment_index + 1]
    tang = math.atan2(p1[1] - p0[1], p1[0] - p0[0])
    return [tang + k * math.pi / 2.0 for k in range(4)]


def edge_inward_at_point(
    sheet: Polygon,
    contact: Point,
) -> tuple[Point, tuple[float, float]] | None:
    """Return anchor on the containing edge segment and unit inward normal."""
    coords = list(sheet.exterior.coords)
    cx, cy = float(contact.x), float(contact.y)
    best_anchor: Point | None = None
    best_inward: tuple[float, float] | None = None
    best_dist = float("inf")
    for i in range(len(coords) - 1):
        p0 = coords[i]
        p1 = coords[i + 1]
        x0, y0 = float(p0[0]), float(p0[1])
        x1, y1 = float(p1[0]), float(p1[1])
        dist_sq, ax, ay, _ = _point_segment_distance_sq(cx, cy, x0, y0, x1, y1)
        if dist_sq >= best_dist:
            continue
        dx = x1 - x0
        dy = y1 - y0
        seg_len = math.hypot(dx, dy)
        if seg_len < 1e-9:
            continue
        nx, ny = -dy / seg_len, dx / seg_len
        probe = Point(ax + nx * 1e-4, ay + ny * 1e-4)
        if not sheet.contains(probe):
            nx, ny = -nx, -ny
        best_dist = dist_sq
        best_anchor = Point(ax, ay)
        best_inward = (nx, ny)
    if best_anchor is None or best_inward is None:
        return None
    return best_anchor, best_inward


def exterior_anchor_points(geom: BaseGeometry, samples_per_edge: int) -> list[Point]:
    anchors: list[Point] = []
    for line in get_shape_exteriors(geom):
        if line.length <= 0:
            continue
        coords = list(line.coords)
        seg_lens: list[float] = []
        seg_endpoints: list[tuple[tuple[float, float], tuple[float, float]]] = []
        for i in range(len(coords) - 1):
            p0, p1 = coords[i], coords[i + 1]
            x0, y0 = float(p0[0]), float(p0[1])
            x1, y1 = float(p1[0]), float(p1[1])
            sl = math.hypot(x1 - x0, y1 - y0)
            if sl > 1e-9:
                seg_lens.append(sl)
                seg_endpoints.append(((x0, y0), (x1, y1)))
        total = sum(seg_lens) or 1.0
        n_seg = len(seg_endpoints) or 1
        for sl, ((x0, y0), (x1, y1)) in zip(seg_lens, seg_endpoints, strict=True):
            n = max(2, int(round(samples_per_edge * sl / total * n_seg)))
            for t in np.linspace(0.02, 0.98, n):
                anchors.append(Point(x0 + t * (x1 - x0), y0 + t * (y1 - y0)))
    return anchors


def angles_for_edge_contact(
    boundary: BaseGeometry,
    contact: Point,
    num_angles: int,
) -> list[float]:
    angles = list(np.linspace(0, 2 * np.pi, num_angles, endpoint=False))
    if isinstance(boundary, Polygon):
        seg_i = _segment_index_at_point(boundary, contact)
        angles.extend(_edge_tangent_angles(boundary, seg_i))
    return angles


def select_stratified_by_segment(
    boundary: Polygon,
    propositions: list[dict],
    top_n: int,
    *,
    anchor_key: str = "anchor",
) -> list[dict]:
    """Reserve top_n slots per exterior segment proportional to edge length."""
    if not propositions:
        return []
    seg_lens = _exterior_segment_lengths(boundary)
    total = sum(seg_lens) or 1.0
    by_seg: dict[int, list[dict]] = {}
    for prop in propositions:
        anchor = prop[anchor_key]
        seg_i = _segment_index_at_point(boundary, anchor)
        by_seg.setdefault(seg_i, []).append(prop)

    picked: list[dict] = []
    for seg_i, seg_len in enumerate(seg_lens):
        bucket = sorted(by_seg.get(seg_i, []), key=lambda row: row["cost"])
        quota = max(2, int(round(top_n * seg_len / total)))
        picked.extend(bucket[:quota])

    seen: set[tuple[float, float, float]] = set()
    out: list[dict] = []
    for prop in sorted(picked, key=lambda row: row["cost"]):
        coords = prop["coords"]
        key = (round(coords[0], 2), round(coords[1], 2), round(coords[2], 1))
        if key in seen:
            continue
        seen.add(key)
        out.append(prop)
        if len(out) >= top_n:
            return out

    for prop in sorted(propositions, key=lambda row: row["cost"]):
        coords = prop["coords"]
        key = (round(coords[0], 2), round(coords[1], 2), round(coords[2], 1))
        if key in seen:
            continue
        seen.add(key)
        out.append(prop)
        if len(out) >= top_n:
            break
    return out


def finalize_edge_propositions(
    propositions: list[dict],
    boundary: BaseGeometry,
    top_n: int,
) -> list[tuple[float, float, float]]:
    if isinstance(boundary, Polygon):
        selected = select_stratified_by_segment(boundary, propositions, top_n)
    else:
        selected = sorted(propositions, key=lambda row: row["cost"])
    seen: set[tuple[float, float, float]] = set()
    out: list[tuple[float, float, float]] = []
    for prop in selected:
        coords = prop["coords"]
        key = (round(coords[0], 2), round(coords[1], 2), round(coords[2], 1))
        if key in seen:
            continue
        seen.add(key)
        out.append(coords)
        if len(out) >= top_n:
            break
    return out
