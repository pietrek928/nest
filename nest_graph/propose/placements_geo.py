import math
from typing import List, Optional, Tuple

import numpy as np
from shapely import LineString, LinearRing, MultiLineString, MultiPoint, Point, Polygon
from shapely.affinity import rotate, translate
from shapely.geometry.base import BaseGeometry
from shapely.ops import voronoi_diagram

from nest_graph.utils import get_shape_exteriors

from nest_graph.propose.context import search_region_for_placement
from nest_graph.propose.geometry import ProposeGeometry

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
    propose_geom: Optional[ProposeGeometry] = None,
    pt_push: Optional[Point] = None,
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
    fit_shape = region

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
                p = Point(vert)
                if fit_shape.contains(p):
                    candidate_points.append(p)
    if len(candidate_points) > max_sites:
        idx = np.linspace(0, len(candidate_points) - 1, max_sites, dtype=int)
        candidate_points = [candidate_points[i] for i in idx]

    angles = np.linspace(0, 2*np.pi, num_angles, endpoint=False)
    attract_x, attract_y = float(attract.x), float(attract.y)

    for pt in candidate_points:
        for angle in angles:
            coords = (float(pt.x), float(pt.y), float(angle))
            if propose_geom is not None and pt_push is not None:
                if not propose_geom.valid_at(coords, pt_push):
                    continue
                propositions.append({
                    "coords": coords,
                    "cost": math.hypot(pt.x - attract_x, pt.y - attract_y),
                })
                continue

            orig_centroid = shape_to_place.centroid
            centered_shape = translate(shape_to_place, -orig_centroid.x, -orig_centroid.y)
            rotated_shape = rotate(centered_shape, angle, origin=(0, 0), use_radians=True)
            placed_shape = translate(rotated_shape, pt.x, pt.y)

            safe = fit_shape.buffer(-min_dist)
            if safe.is_empty or not safe.contains(placed_shape):
                continue

            propositions.append({
                "coords": coords,
                "cost": float(pt.distance(attract)),
            })

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
    propose_geom: Optional[ProposeGeometry] = None,
    pt_push: Optional[Point] = None,
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

    safe_base = region.buffer(-min_dist)
    if safe_base.is_empty:
        return []

    # 1. Identify Anchor Points (vertices of the base and holes)
    anchors = []
    for line in get_shape_exteriors(anchor_source):
        anchors.extend([Point(pt) for pt in line.coords])

    min_x, min_y, max_x, max_y = region.bounds
    ray_len = np.sqrt((max_x - min_x)**2 + (max_y - min_y)**2)

    # 3. Cast Rays and Find Candidates
    ray_angles = np.linspace(0, 2*np.pi, num_rays, endpoint=False)
    placement_angles = np.linspace(0, 2*np.pi, num_angles, endpoint=False)

    stride = max(1, anchor_stride)
    for anchor in anchors[::stride]:
        for r_angle in ray_angles:
            # Create a ray from the anchor point
            end_x = anchor.x + ray_len * np.cos(r_angle)
            end_y = anchor.y + ray_len * np.sin(r_angle)
            ray = LineString([anchor, (end_x, end_y)])

            # Find parts of the ray that are inside the 'safe' base
            valid_segments = ray.intersection(safe_base)
            if valid_segments.is_empty:
                continue

            # Check points along the valid segments
            # We focus on the start of the segment (nearest to the wall)
            coords_to_test = []
            if valid_segments.geom_type == 'LineString':
                coords_to_test = [valid_segments.interpolate(0.1, normalized=True),
                                  valid_segments.interpolate(0.5, normalized=True)]
            elif isinstance(valid_segments, MultiLineString):
                for ls in valid_segments.geoms:
                    coords_to_test.append(ls.interpolate(0.1, normalized=True))

            for pt in coords_to_test:
                for p_angle in placement_angles:
                    coords = (float(pt.x), float(pt.y), float(p_angle))
                    if propose_geom is not None and pt_push is not None:
                        if not propose_geom.valid_at(coords, pt_push):
                            continue
                        propositions.append({
                            "coords": coords,
                            "cost": math.hypot(pt.x - attract_x, pt.y - attract_y),
                        })
                        continue

                    rotated_shape = rotate(shape_to_place, p_angle, origin=(0, 0), use_radians=True)
                    placed_shape = translate(rotated_shape, pt.x, pt.y)

                    if safe_base.contains(placed_shape):
                        propositions.append({
                            "coords": coords,
                            "cost": float(pt.distance(attract)),
                        })

    # Sort and return top N
    propositions.sort(key=lambda x: x['cost'])

    # Use a set to filter out nearly identical propositions
    unique_props = []
    seen = set()
    for p in propositions:
        key = (round(p['coords'][0], 1), round(p['coords'][1], 1), round(p['coords'][2], 0))
        if key not in seen:
            unique_props.append(p['coords'])
            seen.add(key)
        if len(unique_props) >= top_n:
            break

    return unique_props

