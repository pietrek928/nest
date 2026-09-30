"""Free-space Halton cloud: sterile VOID_SEEK recovery (§6 step C)."""

import math
from typing import List, Optional, Sequence, Tuple

from shapely.geometry import Point, Polygon
from shapely.geometry.base import BaseGeometry

from nest_graph.config import ProposeConfig
from nest_graph.propose.geometry import ProposeGeometry
from nest_graph.propose.placement_common import obstacle_parts
from nest_graph.propose.placement_outline import slide_toward_obstacle
from nest_graph.propose.placements_pattern import emit_packing_clear


def _halton(index: int, base: int) -> float:
    """van der Corput / Halton component in (0, 1)."""
    result = 0.0
    f = 1.0 / base
    i = index
    while i > 0:
        result += f * (i % base)
        i //= base
        f /= base
    return result


def polish_cloud_slide_toward_obstacles(
    cloud: Sequence[Tuple[float, float, float]],
    *,
    propose_geom: ProposeGeometry,
    base_shape: BaseGeometry,
    sheet: Polygon,
    min_dist: float,
    shape_to_place: Optional[Polygon] = None,
) -> tuple[list[tuple[float, float, float]], int]:
    """Dc1: kiss-polish Halton survivors via ``slide_toward_obstacle`` (one gate).

    Does not enable neighbor_slide proposer; seeds from cloud θ only.
    Returns (polished_or_original coords, n_slid_ok).
    """
    if not cloud:
        return [], 0
    obstacles = obstacle_parts(base_shape) if base_shape is not None else []
    if not obstacles:
        return [(float(c[0]), float(c[1]), float(c[2])) for c in cloud], 0
    part = shape_to_place if shape_to_place is not None else propose_geom.part_poly
    out: list[tuple[float, float, float]] = []
    slid_n = 0
    seen: set[tuple[float, float, float]] = set()
    for coords in cloud:
        ang = float(coords[2])
        best = (float(coords[0]), float(coords[1]), ang)
        improved = False
        for obstacle in obstacles:
            slid = slide_toward_obstacle(
                part,
                obstacle,
                ang,
                float(min_dist),
                sheet,
                propose_geom=propose_geom,
            )
            if slid is None:
                continue
            if not emit_packing_clear(propose_geom, slid):
                continue
            best = (float(slid[0]), float(slid[1]), float(slid[2]))
            improved = True
            break
        key = (round(best[0], 4), round(best[1], 4), round(best[2], 4))
        if key in seen:
            continue
        seen.add(key)
        if improved:
            slid_n += 1
        out.append(best)
    return out, slid_n


def propose_placements_free_space_cloud(
    void_poly: BaseGeometry,
    *,
    propose_geom: ProposeGeometry,
    propose_cfg: ProposeConfig,
    top_n: int = 40,
    allowed_angles: Sequence[float] | None = None,
) -> List[Tuple[float, float, float]]:
    """Sample Halton xy in void bbox, filter contains + packing SoT."""
    if (
        void_poly is None
        or getattr(void_poly, "is_empty", True)
        or not bool(getattr(propose_cfg, "use_free_space_cloud", True))
    ):
        return []
    minx, miny, maxx, maxy = void_poly.bounds
    w = float(maxx - minx)
    h = float(maxy - miny)
    if w <= 1e-12 or h <= 1e-12:
        return []

    n_xy = max(int(getattr(propose_cfg, "free_space_cloud_samples", 64)), 1)
    n_ang = max(int(getattr(propose_cfg, "free_space_cloud_angles", 8)), 1)
    if allowed_angles is not None and len(allowed_angles) > 0:
        angles = [float(a) for a in allowed_angles]
    else:
        angles = [2.0 * math.pi * i / n_ang for i in range(n_ang)]

    void_g = None
    if hasattr(propose_geom, "region_geometry"):
        void_g = propose_geom.region_geometry(void_poly)

    out: list[tuple[float, float, float]] = []
    seen: set[tuple[float, float, float]] = set()
    # Skip first Halton index (0,0) clump; start at 1.
    for i in range(1, n_xy + 1):
        u = _halton(i, 2)
        v = _halton(i, 3)
        x = minx + u * w
        y = miny + v * h
        inside = (
            void_g.contains_point(x, y)
            if void_g is not None
            else void_poly.covers(Point(x, y))
        )
        if not inside:
            continue
        for ang in angles:
            coords = (float(x), float(y), float(ang))
            key = (round(coords[0], 2), round(coords[1], 2), round(coords[2], 2))
            if key in seen:
                continue
            seen.add(key)
            if not emit_packing_clear(propose_geom, coords):
                continue
            out.append(coords)
            if len(out) >= top_n:
                return out
    return out
