from __future__ import annotations

from ._geometry import (
    ComplexDistanceResult,
    Geometry,
    GuidanceConfig,
    PlacementGuidance,
    evaluate_local_placement,
    find_polygon_distances,
    find_polygon_distances_active,
    find_polygon_distances_bipartite,
    find_polygon_intersections,
    find_polygon_intersections_active,
    find_polygon_intersections_bipartite,
)

__all__ = [
    "ComplexDistanceResult",
    "Geometry",
    "GuidanceConfig",
    "PlacementGuidance",
    "evaluate_local_placement",
    "find_polygon_distances",
    "find_polygon_distances_active",
    "find_polygon_distances_bipartite",
    "find_polygon_intersections",
    "find_polygon_intersections_active",
    "find_polygon_intersections_bipartite",
]
