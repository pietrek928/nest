import math
import time
from typing import List, Optional, Sequence, Tuple

import numpy as np
from shapely import LineString, LinearRing, MultiPoint, Point, Polygon
from shapely.geometry.base import BaseGeometry
from shapely.ops import voronoi_diagram

from nest_graph.utils import get_shape_exteriors

from nest_graph.geometry import Geometry
from nest_graph.propose.context import search_region_for_placement
from nest_graph.propose.geometry import ProposeGeometry, filter_candidates_batch
from nest_graph.propose.placement_perimeter import vertex_anchors_from_geom

def densify_points(geometry, distance):
    """Adds points along the perimeter of the shape for a better Voronoi map."""
    if distance <= 0 or geometry.is_empty:
        return MultiPoint()

    points = []
    for ring in get_shape_exteriors(geometry):
        if ring.length <= 0:
            continue
        line = LineString(ring.coords) if isinstance(ring, LinearRing) else ring
        if line.geom_type != "LineString" or line.length <= 0:
            continue
        for d in np.arange(0, line.length, distance):
            points.append(line.interpolate(d))
        points.append(line.interpolate(line.length))
    return MultiPoint(points) if points else MultiPoint()


def propose_placements_voronoi(
    base_shape: BaseGeometry,
    shape_to_place: Polygon,
    sheet: Polygon,
    min_dist: float,
    *,
    use_free_region: bool = False,
    num_angles: int = 8,
    top_n: int = 3,
    densify_divisor: float = 20.0,
    max_sites: int = 64,
    focal_shape: Optional[BaseGeometry] = None,
    border_focus: bool = False,
    propose_geom: ProposeGeometry,
    pt_push: Point,
) -> List[Tuple[float, float, float]]:
    """
    Proposes placements using Voronoi vertices as candidate centers.
    """
    propositions: list[dict] = []
    region = search_region_for_placement(
        base_shape, sheet, sheet, min_dist,
        use_free_region=use_free_region, border_focus=border_focus,
    )
    if region.is_empty:
        return []

    attract = (
        focal_shape.centroid
        if focal_shape is not None and not focal_shape.is_empty
        else region.centroid
    )
    region_g = propose_geom.region_geometry(region)

    # 1. Densify the layout region and generate Voronoi Diagram
    extent = max(region.bounds[2] - region.bounds[0], region.bounds[3] - region.bounds[1])
    step = max(extent / densify_divisor, 1e-4)
    points = densify_points(region, step)
    if points.is_empty:
        return []

    vor_regions = voronoi_diagram(points)

    candidate_points = []
    for vor_region in vor_regions.geoms:
        rings = get_shape_exteriors(vor_region)
        for ring in rings:
            for vert in ring.coords:
                x, y = float(vert[0]), float(vert[1])
                if region_g is not None:
                    if not region_g.contains_point(x, y):
                        continue
                elif not region.contains(Point(x, y)):
                    continue
                candidate_points.append((x, y))
    if len(candidate_points) > max_sites:
        idx = np.linspace(0, len(candidate_points) - 1, max_sites, dtype=int)
        candidate_points = [candidate_points[i] for i in idx]

    angles = np.linspace(0, 2*np.pi, num_angles, endpoint=False)
    attract_x, attract_y = float(attract.x), float(attract.y)

    raw: list[tuple[float, float, float]] = []
    costs: list[float] = []
    for px, py in candidate_points:
        for angle in angles:
            coords = (float(px), float(py), float(angle))
            raw.append(coords)
            costs.append(math.hypot(px - attract_x, py - attract_y))
    if not raw:
        return []
    valid = set(filter_candidates_batch(propose_geom, raw, pt_push))
    propositions = [
        {"coords": c, "cost": cost}
        for c, cost in zip(raw, costs, strict=True)
        if c in valid
    ]
    propositions.sort(key=lambda x: x["cost"])
    return [p["coords"] for p in propositions[:top_n]]


def propose_placements_raycasting(
    base_shape: BaseGeometry,
    shape_to_place: Polygon,
    sheet: Polygon,
    min_dist: float,
    *,
    use_free_region: bool = False,
    num_rays: int = 12,
    num_angles: int = 8,
    top_n: int = 3,
    anchor_stride: int = 2,
    focal_shape: Optional[BaseGeometry] = None,
    border_focus: bool = False,
    propose_geom: ProposeGeometry,
    pt_push: Point,
    rim_anchor_geoms: Sequence[BaseGeometry | Geometry] | None = None,
) -> List[Tuple[float, float, float]]:
    """
    Proposes placements by casting rays from boundary vertices into the interior.
    """
    propositions: list[dict] = []
    region = search_region_for_placement(
        base_shape, sheet, sheet, min_dist,
        use_free_region=use_free_region, border_focus=border_focus,
    )
    if region.is_empty:
        return []

    if focal_shape is not None and not focal_shape.is_empty:
        anchor_source = focal_shape
    elif use_free_region:
        anchor_source = region
    else:
        anchor_source = base_shape if not base_shape.is_empty else region

    attract = (
        focal_shape.centroid
        if focal_shape is not None and not focal_shape.is_empty
        else region.centroid
    )

    attract_x, attract_y = float(attract.x), float(attract.y)

    t_ray0 = time.perf_counter()
    region_g = propose_geom.region_geometry(region)
    if region_g is None:
        return []

    # Prefer native region rings when the anchor source is the search region.
    src_for_harvest: BaseGeometry | Geometry = anchor_source
    if anchor_source is region:
        src_for_harvest = region_g
    anchors_xy, native_n, from_n = vertex_anchors_from_geom(src_for_harvest)
    if rim_anchor_geoms:
        for geom in rim_anchor_geoms:
            if geom is None or getattr(geom, "is_empty", True):
                continue
            extra, n_nat, n_fs = vertex_anchors_from_geom(geom)
            anchors_xy.extend(extra)
            native_n += n_nat
            from_n += n_fs
    propose_geom._last_ray_anchor_native_n = int(native_n)
    propose_geom._last_ray_anchor_from_shapely_n = int(from_n)

    max_anchors = 200
    if len(anchors_xy) > max_anchors:
        step = len(anchors_xy) / max_anchors
        anchors_xy = [anchors_xy[int(i * step)] for i in range(max_anchors)]

    min_x, min_y, max_x, max_y = region.bounds
    ray_len = np.sqrt((max_x - min_x)**2 + (max_y - min_y)**2)

    ray_angles = np.linspace(0, 2*np.pi, num_rays, endpoint=False)
    placement_angles = np.linspace(0, 2*np.pi, num_angles, endpoint=False)

    stride = max(1, anchor_stride)
    sample_fracs = (0.1, 0.5)
    origins: list[tuple[float, float]] = []
    directions: list[tuple[float, float]] = []
    for ax, ay in anchors_xy[::stride]:
        for r_angle in ray_angles:
            dx = ray_len * float(np.cos(r_angle))
            dy = ray_len * float(np.sin(r_angle))
            origins.append((float(ax), float(ay)))
            directions.append((dx, dy))
    if origins:
        try:
            coords_flat, offsets = region_g.clip_ray_interior_batch(
                origins, directions, float(ray_len), sample_fracs,
            )
            n_rays = len(offsets) - 1
            for i in range(n_rays):
                a = int(offsets[i])
                b = int(offsets[i + 1])
                for k in range(a, b):
                    pt_x = float(coords_flat[2 * k])
                    pt_y = float(coords_flat[2 * k + 1])
                    for p_angle in placement_angles:
                        coords = (pt_x, pt_y, float(p_angle))
                        propositions.append({
                            "coords": coords,
                            "cost": math.hypot(pt_x - attract_x, pt_y - attract_y),
                        })
        except Exception:
            for (ox, oy), (dx, dy) in zip(origins, directions, strict=True):
                try:
                    hits = region_g.clip_ray_interior(
                        (ox, oy), (dx, dy), float(ray_len), sample_fracs,
                    )
                except Exception:
                    hits = []
                for x, y in hits:
                    pt_x, pt_y = float(x), float(y)
                    for p_angle in placement_angles:
                        coords = (pt_x, pt_y, float(p_angle))
                        propositions.append({
                            "coords": coords,
                            "cost": math.hypot(pt_x - attract_x, pt_y - attract_y),
                        })

    propositions.sort(key=lambda x: x["cost"])
    unique_props: list[tuple[float, float, float]] = []
    unique_costs: list[float] = []
    seen: set[tuple[float, float, float]] = set()
    pool_cap = max(top_n * 3, top_n)
    for p in propositions:
        key = (round(p["coords"][0], 4), round(p["coords"][1], 4), round(p["coords"][2], 4))
        if key in seen:
            continue
        seen.add(key)
        unique_props.append(p["coords"])
        unique_costs.append(float(p["cost"]))
        if len(unique_props) >= pool_cap:
            break

    if not unique_props:
        return []
    _ray_ms = (time.perf_counter() - t_ray0) * 1000.0
    propose_geom._last_raycast_ms = float(getattr(propose_geom, "_last_raycast_ms", 0.0) or 0.0) + _ray_ms
    cost_map = {c: cost for c, cost in zip(unique_props, unique_costs, strict=True)}
    valid = filter_candidates_batch(propose_geom, unique_props, pt_push)
    valid.sort(key=lambda c: cost_map.get(c, 0.0))
    return valid[:top_n]
