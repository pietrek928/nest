"""Free-space Halton cloud: sterile VOID_SEEK recovery (§6 step C)."""

import math
from typing import List, Sequence, Tuple

from shapely.geometry import Point
from shapely.geometry.base import BaseGeometry

from nest_graph.config import ProposeConfig
from nest_graph.propose.geometry import ProposeGeometry
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
    if allowed_angles:
        angles = [float(a) for a in allowed_angles]
    else:
        angles = [2.0 * math.pi * i / n_ang for i in range(n_ang)]

    out: list[tuple[float, float, float]] = []
    seen: set[tuple[float, float, float]] = set()
    # Skip first Halton index (0,0) clump; start at 1.
    for i in range(1, n_xy + 1):
        u = _halton(i, 2)
        v = _halton(i, 3)
        x = minx + u * w
        y = miny + v * h
        if not void_poly.covers(Point(x, y)):
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
