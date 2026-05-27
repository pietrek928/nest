import math
from typing import List, Optional, Sequence, Tuple, Union

import numpy as np
from shapely import LineString, LinearRing, MultiPoint, MultiPolygon, Point, Polygon
from shapely.affinity import rotate, translate
from shapely.geometry.base import BaseGeometry
from shapely.ops import nearest_points, polylabel, unary_union, voronoi_diagram

from .board import board_context_from_geometry
from .config import ProposeConfig, dedupe_transforms
from .geometry import Geometry
from .placement_scene import (
    PLACEMENT_EPSILON_RATIO,
    build_placement_scene,
    guidance_config_for_propose,
    guidance_config_for_scene,
    guidance_ray_direction_candidates,
    is_valid_placement,
    placement_outside_outer,
)
from .utils import get_shape_exteriors, get_shape_polygons_coords, transform_poly


def placement_free_region(
    sheet: Polygon,
    base_shape: BaseGeometry,
    min_dist: float,
) -> BaseGeometry:
    """Nestable sheet minus clearance buffer around the packed layout."""
    if base_shape is None or base_shape.is_empty:
        return sheet
    free = sheet.difference(base_shape.buffer(min_dist))
    if free.is_empty:
        return sheet
    return free


def cluster_packed_solid_groups(
    polys: Sequence[BaseGeometry],
    min_dist: float,
) -> list[Polygon]:
    """Connected clusters of packed parts (touching within clearance gap)."""
    placed = [p for p in polys if p is not None and not p.is_empty]
    if not placed:
        return []
    if len(placed) == 1:
        return [unary_union(placed)]

    gap = max(min_dist * 0.5, 1e-6)
    buffered = [p.buffer(gap) for p in placed]
    merged = unary_union(buffered)
    if merged.is_empty:
        return []

    blobs = list(merged.geoms) if merged.geom_type == "MultiPolygon" else [merged]
    groups: list[Polygon] = []
    for blob in blobs:
        members = [p for p in placed if p.buffer(gap).intersects(blob)]
        if members:
            groups.append(unary_union(members))
    return groups


def border_solid_focal(sheet: Polygon, min_dist: float) -> Polygon:
    """Thin ring along the nestable sheet outer boundary for edge-fitting proposals."""
    inset = max(min_dist * 2.0, 1e-4)
    inner = sheet.buffer(-inset)
    if inner.is_empty or inner.area < sheet.area * 1e-6:
        return sheet
    ring = sheet.difference(inner)
    return ring if not ring.is_empty else sheet


def obstacle_polys_for_propose(
    placed: Sequence[BaseGeometry],
    part_poly: Polygon,
    min_dist: float,
) -> list[BaseGeometry]:
    """Nearest packed cluster only; graph still checks the full layout."""
    if not placed:
        return []
    groups = cluster_packed_solid_groups(placed, min_dist)
    if not groups:
        return []
    if len(groups) == 1:
        return list(placed)
    gap = max(min_dist * 0.5, 1e-6)
    ref = part_poly.centroid
    nearest = min(groups, key=lambda g: g.distance(ref))
    return [p for p in placed if p.buffer(gap).intersects(nearest)]


def obstacle_shape_for_propose(
    placed: Sequence[BaseGeometry],
    part_poly: Polygon,
    min_dist: float,
) -> BaseGeometry:
    polys = obstacle_polys_for_propose(placed, part_poly, min_dist)
    if not polys:
        return Polygon()
    return unary_union(polys)


def focal_shape_for_propose(
    board: BaseGeometry,
    placed: Sequence[BaseGeometry],
    part_poly: Polygon,
    min_dist: float,
    propose_cfg: ProposeConfig,
) -> Optional[BaseGeometry]:
    """Focal geometry for ray anchors and group/board edge seeds."""
    obstacle = obstacle_shape_for_propose(placed, part_poly, min_dist)
    if obstacle is not None and not obstacle.is_empty:
        return obstacle
    if propose_cfg.use_border_focus:
        return border_focal_for_propose(board, min_dist)
    return None


def search_region_for_placement(
    base_shape: BaseGeometry,
    boundary: Optional[BaseGeometry],
    sheet: Optional[Polygon],
    min_dist: float,
    *,
    use_free_region: bool,
    border_focus: bool = False,
) -> BaseGeometry:
    if use_free_region and sheet is not None:
        return placement_free_region(sheet, base_shape, min_dist)
    if base_shape is not None and not base_shape.is_empty:
        return base_shape
    if boundary is not None and not boundary.is_empty:
        return boundary
    return sheet if sheet is not None else Polygon()


def should_use_border_focus(
    base_shape: BaseGeometry,
    propose_cfg: ProposeConfig,
) -> bool:
    """Empty sheet: fit parts against the nestable border, not the board centroid."""
    if not propose_cfg.use_border_focus:
        return False
    return base_shape is None or base_shape.is_empty


def border_focal_for_propose(
    board: BaseGeometry,
    min_dist: float,
) -> Polygon:
    sheet, _voids = board_context_from_geometry(board)
    return border_solid_focal(sheet, min_dist)


def propose_push_point(
    board: BaseGeometry,
    base_shape: BaseGeometry,
    *,
    smart_push: bool,
    min_dist: float = 0.0,
    use_border_focus: bool = False,
) -> Point:
    if smart_push and base_shape is not None and not base_shape.is_empty:
        return base_shape.centroid
    if use_border_focus and (base_shape is None or base_shape.is_empty):
        border = border_focal_for_propose(board, max(min_dist, 1e-6))
        if not border.is_empty:
            return border.centroid
    return board.centroid


def effective_ranking_mode(
    propose_cfg: ProposeConfig,
    base_shape: BaseGeometry,
) -> str:
    if (
        propose_cfg.border_focus_ranking
        and should_use_border_focus(base_shape, propose_cfg)
    ):
        return "border"
    if (
        propose_cfg.use_contact_ranking
        and base_shape is not None
        and not base_shape.is_empty
    ):
        if propose_cfg.use_contact_clearance_hybrid:
            return "contact_hybrid"
        return "contact"
    return propose_cfg.ranking_mode


def _placement_contact_error(
    placed: BaseGeometry,
    sheet: Polygon,
    min_dist: float,
    focal_shape: Optional[BaseGeometry] = None,
) -> float:
    """Distance from ideal standoff (0 = flush against border or group)."""
    border_err = abs(float(placed.distance(sheet.exterior)) - min_dist)
    if focal_shape is not None and not focal_shape.is_empty:
        group_err = abs(float(focal_shape.distance(placed)) - min_dist)
        return min(border_err, group_err)
    return border_err


class ProposeGeometry:
    """Cached placement scene for fast propose validation and guidance."""

    def __init__(
        self,
        boundary: BaseGeometry,
        base_shape: BaseGeometry,
        part_poly: Polygon,
        min_dist: float,
        *,
        epsilon_ratio: float = PLACEMENT_EPSILON_RATIO,
    ):
        part = Geometry.from_shapely(part_poly)
        base_geoms: list[Geometry] = []
        if base_shape is not None and not base_shape.is_empty:
            base_geoms = [Geometry.from_shapely(base_shape)]
        self.scene = build_placement_scene(boundary, part, base_geoms)
        self.sheet = self.scene.sheet
        self.boundary = self.sheet
        self.part = part
        self._min_dist = min_dist
        self._epsilon_ratio = epsilon_ratio
        self._board_bounds = self.sheet.bounds
        self._guidance_cfg = guidance_config_for_scene(
            min_dist,
            board_bounds=self._board_bounds,
            epsilon_ratio=epsilon_ratio,
        )

    def placed_at(self, coords: Tuple[float, float, float]) -> Geometry:
        return self.part.apply_transform(coords)

    def placement_guidance(
        self,
        placed: Geometry,
        xy: Tuple[float, float],
        push: Point,
    ):
        cfg = guidance_config_for_propose(
            push,
            min_dist=self._min_dist,
            board_bounds=self._board_bounds,
            epsilon_ratio=self._epsilon_ratio,
        )
        return self.scene.guidance(placed, xy, cfg)

    def inside_board(self, placed: Geometry) -> bool:
        if placement_outside_outer(placed, self.sheet):
            return False
        cx, cy = placed.center()
        g = self.scene.guidance(placed, (cx, cy), self._guidance_cfg)
        return not g.is_penetrating

    def hits_base(
        self,
        placed: Geometry,
        push: Point | None = None,
        xy: Tuple[float, float] | None = None,
    ) -> bool:
        if not self.scene.base_geoms:
            return False
        return placed.intersects_any(self.scene.base_geoms)

    def is_valid_placement(
        self,
        placed: Geometry,
        push: Point,
        xy: Tuple[float, float],
    ) -> bool:
        cfg = guidance_config_for_propose(
            push,
            min_dist=self._min_dist,
            board_bounds=self._board_bounds,
            epsilon_ratio=self._epsilon_ratio,
        )
        return is_valid_placement(
            self.scene, placed, xy, self._min_dist, cfg,
            epsilon_ratio=self._epsilon_ratio,
        )

    def attraction_unit(
        self,
        placed: Geometry,
        push: Point,
        xy: Tuple[float, float],
    ) -> np.ndarray:
        g = self.placement_guidance(placed, xy, push)
        if g.is_penetrating:
            vec = np.array(g.ejection_vector, dtype=np.float64)
        else:
            vec = np.array(g.suggested_translation, dtype=np.float64)
        norm = np.linalg.norm(vec)
        return vec / norm if norm > 1e-9 else np.zeros(2)


def propose_placements_erosion(
    base_shape, shape_to_place,
    boundary: Optional[BaseGeometry] = None,
    sheet: Optional[Polygon] = None,
    *,
    use_free_region: bool = False,
    min_dist=2.0, num_angles=8, top_n=3,
    weight_dist=0.5,
    multi_site: bool = False,
    focal_shape: Optional[BaseGeometry] = None,
    border_focus: bool = False,
):
    """
    Finds x, y, angle with a forced minimum distance from boundaries.
    """
    propositions = []
    region = search_region_for_placement(
        base_shape, boundary, sheet, min_dist,
        use_free_region=use_free_region, border_focus=border_focus,
    )
    if region is None or region.is_empty:
        return []

    attract = (
        focal_shape.centroid
        if focal_shape is not None and not focal_shape.is_empty
        else region.centroid
    )
    base_hull_area = base_shape.convex_hull.area if not base_shape.is_empty else 0.0

    angles = np.linspace(0, 2 * np.pi, num_angles, endpoint=False)

    for angle in angles:
        rotated_shape = rotate(shape_to_place, angle, origin=(0, 0), use_radians=True)

        # 2. Calculate the 'Collision Radius' + min_dist
        # We use the distance from the centroid to the furthest vertex
        # plus the required clearance.
        bounds = rotated_shape.bounds
        max_dim = max(bounds[2] - bounds[0], bounds[3] - bounds[1]) / 2
        total_buffer = max_dim + min_dist

        # 3. Generate Safe Zone (Erosion)
        # This automatically respects holes because buffer handles interior rings.
        safe_zone = region.buffer(-total_buffer)

        if safe_zone.is_empty:
            continue

        # 4. Extract Candidate Points
        # We check the 'deepest' points in the safe zone (furthest from walls)
        candidate_points = []
        if isinstance(safe_zone, MultiPolygon):
            polys = safe_zone.geoms
        else:
            polys = (safe_zone,)
        for poly in polys:
            if poly.is_empty:
                continue
            candidate_points.append(polylabel(poly, tolerance=0.5))
            if multi_site:
                candidate_points.append(poly.representative_point())

        for pt in candidate_points:
            # Place the actual shape at this candidate center
            placed_shape = translate(rotated_shape, pt.x, pt.y)

            # 5. Precise Distance Validation
            # Even if the buffer is a heuristic, we double check the min_dist
            if not base_shape.is_empty:
                actual_dist = base_shape.distance(placed_shape)
                if actual_dist < (min_dist - 0.001):
                    continue
            elif not region.contains(placed_shape):
                continue

            dist_to_center = pt.distance(attract)

            parts = [base_shape, placed_shape] if not base_shape.is_empty else [placed_shape]
            combined_hull = unary_union(parts).convex_hull
            hull_growth = max(0, combined_hull.area - base_hull_area)
            hull_score = np.sqrt(hull_growth)

            # Combined Cost (Lower is better)
            total_cost = (weight_dist * dist_to_center) + ((1 - weight_dist) * hull_score)

            propositions.append({
                'coords': (pt.x, pt.y, angle),
                'cost': total_cost
            })

    # Sort and return
    propositions.sort(key=lambda x: x['cost'])
    return [p['coords'] for p in propositions[:top_n]]


def propose_placements_guidance_walk(
    base_shape: BaseGeometry,
    shape_to_place: Polygon,
    sheet: Polygon,
    pt_push: Point,
    propose_geom: ProposeGeometry,
    *,
    min_dist: float,
    num_seeds: int = 12,
    walk_steps: int = 5,
    step_scale: float = 0.15,
    top_n: int = 8,
) -> List[Tuple[float, float, float]]:
    """Seed placements by stepping along C++ guidance in free space."""
    seeds = list(sample_placement_points_ribbon(base_shape, shape_to_place, sheet, min_dist))
    if not seeds:
        seeds = [sheet.centroid if not base_shape.is_empty else pt_push]
    if len(seeds) > num_seeds:
        step = len(seeds) / num_seeds
        seeds = [seeds[int(i * step)] for i in range(num_seeds)]

    angle_grid = np.linspace(0, 2 * np.pi, max(num_seeds, 4), endpoint=False)
    out: list[tuple[float, float, float]] = []
    seen: set[tuple[float, float, float]] = set()
    for i, seed in enumerate(seeds):
        x, y = float(seed.x), float(seed.y)
        theta = float(angle_grid[i % len(angle_grid)])
        for _ in range(walk_steps):
            placed = propose_geom.placed_at((x, y, theta))
            g = propose_geom.placement_guidance(placed, (x, y), pt_push)
            if g.is_penetrating:
                ex, ey = g.ejection_vector
                step = step_scale * max(min_dist, 1e-4)
                if math.hypot(ex, ey) > 1e-9:
                    x += ex / math.hypot(ex, ey) * step
                    y += ey / math.hypot(ex, ey) * step
                else:
                    break
            else:
                sx, sy = g.suggested_translation
                mag = math.hypot(sx, sy)
                if mag < 1e-9:
                    break
                step = step_scale * mag
                x += sx / mag * step
                y += sy / mag * step
        placed = propose_geom.placed_at((x, y, theta))
        if propose_geom.is_valid_placement(placed, pt_push, (x, y)):
            key = (round(x, 3), round(y, 3), round(theta, 3))
            if key not in seen:
                seen.add(key)
                out.append((x, y, theta))
        if len(out) >= top_n:
            break
    return out[:top_n]


def _exterior_anchor_points(geom: BaseGeometry, samples_per_edge: int) -> list[Point]:
    anchors: list[Point] = []
    for line in get_shape_exteriors(geom):
        if line.length <= 0:
            continue
        for t in np.linspace(0.02, 0.98, max(2, samples_per_edge)):
            anchors.append(line.interpolate(t, normalized=True))
    return anchors


def _snap_coords_along_exterior(
    shape_to_place: Polygon,
    boundary: BaseGeometry,
    contact: Point,
    inward: Tuple[float, float],
    angle: float,
    min_dist: float,
    *,
    container: Optional[Polygon] = None,
) -> Optional[Tuple[float, float, float]]:
    """Snap rotated part so nearest point on boundary sits at min_dist along inward."""
    ix, iy = inward
    rotated = rotate(shape_to_place, angle, origin=(0, 0), use_radians=True)
    rc = rotated.centroid
    standoff = boundary.exterior if boundary.geom_type == "Polygon" else boundary
    bounds = rotated.bounds
    pad = max(bounds[2] - bounds[0], bounds[3] - bounds[1]) * 0.5
    _, p_part = nearest_points(standoff, rotated)
    target = Point(
        contact.x + ix * (min_dist + pad),
        contact.y + iy * (min_dist + pad),
    )
    dx = target.x - p_part.x
    dy = target.y - p_part.y
    placed = translate(rotated, dx, dy)
    for _ in range(24):
        gap = float(placed.distance(standoff))
        inside = container is None or container.contains(placed)
        if gap >= min_dist - 1e-6 and inside:
            break
        placed = translate(placed, ix * min_dist * 0.2, iy * min_dist * 0.2)
    if float(placed.distance(standoff)) < min_dist - 1e-6:
        return None
    if container is not None and not container.contains(placed):
        return None
    seed_x = placed.centroid.x - rc.x
    seed_y = placed.centroid.y - rc.y
    return (seed_x, seed_y, angle)


def propose_placements_group_fit(
    focal_shape: BaseGeometry,
    shape_to_place: Polygon,
    sheet: Polygon,
    base_shape: BaseGeometry,
    *,
    min_dist: float,
    num_angles: int = 12,
    top_n: int = 16,
    samples_per_edge: int = 12,
    propose_geom: Optional[ProposeGeometry] = None,
    pt_push: Optional[Point] = None,
) -> List[Tuple[float, float, float]]:
    """Snap the part along the nearest packed-group exterior at standoff min_dist."""
    if focal_shape is None or focal_shape.is_empty:
        return []
    interior = focal_shape.representative_point()
    angles = np.linspace(0, 2 * np.pi, num_angles, endpoint=False)
    propositions: list[dict] = []
    anchor_pts = _exterior_anchor_points(focal_shape, samples_per_edge)

    for angle in angles:
        for contact in anchor_pts:
            ox = contact.x - interior.x
            oy = contact.y - interior.y
            dist = math.hypot(ox, oy)
            if dist < 1e-9:
                continue
            coords = _snap_coords_along_exterior(
                shape_to_place,
                focal_shape,
                contact,
                (ox / dist, oy / dist),
                angle,
                min_dist,
            )
            if coords is None:
                continue
            placed = transform_poly(shape_to_place, coords)
            if not sheet.contains(placed):
                continue
            if not base_shape.is_empty:
                if base_shape.intersects(placed):
                    continue
                if base_shape.distance(placed) < min_dist - 1e-6:
                    continue
            if propose_geom is not None and pt_push is not None:
                placed_g = propose_geom.placed_at(coords)
                if not propose_geom.is_valid_placement(
                    placed_g, pt_push, (coords[0], coords[1]),
                ):
                    continue
            err = _placement_contact_error(placed, sheet, min_dist, focal_shape)
            propositions.append({"coords": coords, "cost": err})

    propositions.sort(key=lambda x: x["cost"])
    seen: set[tuple[float, float, float]] = set()
    out: list[tuple[float, float, float]] = []
    for p in propositions:
        key = (round(p["coords"][0], 2), round(p["coords"][1], 2), round(p["coords"][2], 1))
        if key in seen:
            continue
        seen.add(key)
        out.append(p["coords"])
        if len(out) >= top_n:
            break
    return out


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
    base_shape, shape_to_place,
    boundary: Optional[BaseGeometry] = None,
    sheet: Optional[Polygon] = None,
    *,
    use_free_region: bool = False,
    min_dist=2.0, num_angles=8, top_n=3,
    densify_divisor: float = 20.0,
    max_sites: int = 64,
    weight_dist=0.5,
    focal_shape: Optional[BaseGeometry] = None,
    border_focus: bool = False,
):
    """
    Proposes placements using Voronoi vertices as candidate centers.
    """
    propositions = []
    region = search_region_for_placement(
        base_shape, boundary, sheet, min_dist,
        use_free_region=use_free_region, border_focus=border_focus,
    )
    if region is None or region.is_empty:
        return []

    attract = (
        focal_shape.centroid
        if focal_shape is not None and not focal_shape.is_empty
        else region.centroid
    )
    base_hull_area = base_shape.convex_hull.area if not base_shape.is_empty else 0.0
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

    # 3. Normalize the shape to place
    orig_centroid = shape_to_place.centroid
    centered_shape = translate(shape_to_place, -orig_centroid.x, -orig_centroid.y)

    angles = np.linspace(0, 2*np.pi, num_angles, endpoint=False)

    # 4. Evaluate candidates
    for pt in candidate_points:
        for angle in angles:
            rotated_shape = rotate(centered_shape, angle, origin=(0, 0), use_radians=True)
            placed_shape = translate(rotated_shape, pt.x, pt.y)

            safe = fit_shape.buffer(-min_dist)
            if safe.is_empty or not safe.contains(placed_shape):
                continue

            # 5. Scoring
            dist_val = pt.distance(attract)
            combined = base_shape if not base_shape.is_empty else placed_shape
            combined_hull = unary_union([combined, placed_shape]).convex_hull
            hull_growth = np.sqrt(max(0, combined_hull.area - base_hull_area))

            total_cost = (weight_dist * dist_val) + ((1 - weight_dist) * hull_growth)

            propositions.append({
                'coords': (pt.x, pt.y, angle),
                'cost': total_cost
            })

    # Sort and filter unique locations (Voronoi often creates duplicates)
    propositions.sort(key=lambda x: x['cost'])

    # Return Top N
    return [p['coords'] for p in propositions[:top_n]]


def propose_placements_raycasting(
    base_shape, shape_to_place,
    boundary: Optional[BaseGeometry] = None,
    sheet: Optional[Polygon] = None,
    *,
    use_free_region: bool = False,
    min_dist=2.0, num_rays=12, num_angles=8, top_n=3,
    anchor_stride: int = 2,
    weight_dist=0.5,
    focal_shape: Optional[BaseGeometry] = None,
    border_focus: bool = False,
):
    """
    Proposes placements by casting rays from boundary vertices into the interior.
    """
    propositions = []
    region = search_region_for_placement(
        base_shape, boundary, sheet, min_dist,
        use_free_region=use_free_region, border_focus=border_focus,
    )
    if region is None or region.is_empty:
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
    base_hull_area = base_shape.convex_hull.area if not base_shape.is_empty else 0.0

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
            elif valid_segments.geom_type == 'MultiLineString':
                for ls in valid_segments.geoms:
                    coords_to_test.append(ls.interpolate(0.1, normalized=True))

            for pt in coords_to_test:
                for p_angle in placement_angles:
                    rotated_shape = rotate(shape_to_place, p_angle, origin=(0, 0), use_radians=True)
                    placed_shape = translate(rotated_shape, pt.x, pt.y)

                    # Final validation
                    if safe_base.contains(placed_shape):
                        dist_val = pt.distance(attract)

                        if not base_shape.is_empty:
                            all_pts = list(base_shape.convex_hull.exterior.coords) + \
                                      list(placed_shape.exterior.coords)
                            hull_growth = np.sqrt(max(0, MultiPoint(all_pts).convex_hull.area - base_hull_area))
                        else:
                            hull_growth = 0.0

                        total_cost = (weight_dist * dist_val) + ((1 - weight_dist) * hull_growth)

                        propositions.append({
                            'coords': (pt.x, pt.y, p_angle),
                            'cost': total_cost
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


def propose_placements_point_cloud(
    base_shape, shape_to_place, boundary, pt_push,
    num_particles=64, max_iterations=128, min_dist=1.0, top_n=3,
    omega=0.6, c1=1.2, c2=1.4, nudge_iters=8, pull_factor=0.08,
    cull_ratio=0.2, stagnation_limit=5,
    ray_dirs: int = 16,
    mutation_sigma=np.array([2.0, 2.0, 0.5, 0.5]),
    propose_geom: Optional[ProposeGeometry] = None,
):
    bound_centroid = boundary.centroid
    base_hull_area = base_shape.convex_hull.area if not base_shape.is_empty else 0
    ribbon_pts = sample_placement_points_ribbon(base_shape, shape_to_place, boundary, min_dist)

    # 1. Initialize Particles: [x, y, theta, phi]
    particles = []
    for i in range(num_particles):
        pt = ribbon_pts[i % len(ribbon_pts)] if ribbon_pts else bound_centroid
        particles.append([pt.x, pt.y, np.random.uniform(0, 2*np.pi), np.random.uniform(0, 2*np.pi)])

    particles = np.array(particles)
    velocities = np.random.uniform(-0.5, 0.5, (num_particles, 4))
    p_best_pos = particles.copy()
    p_best_score = np.full(num_particles, float('inf'))
    p_best_settled = particles[:, :3].copy() # Store the (x,y,theta) after nudge
    stagnation_counters = np.zeros(num_particles)

    g_best_pos = particles[0].copy()
    g_best_score = float('inf')

    # 2. Optimization Loop
    for iteration in range(max_iterations):
        current_scores = []
        settled_coords_list = []

        for i in range(num_particles):
            score, settled = evaluate_ray_placement(
                particles[i], base_shape, shape_to_place, boundary,
                base_hull_area, bound_centroid, pt_push, min_dist, nudge_iters, pull_factor,
                ray_dirs=ray_dirs,
                propose_geom=propose_geom,
            )
            current_scores.append(score)
            settled_coords_list.append(settled)

            # Update Personal and Global Bests
            if score < p_best_score[i]:
                p_best_score[i] = score
                p_best_pos[i] = particles[i].copy()
                p_best_settled[i] = settled # Store the successful placement
                stagnation_counters[i] = 0
                if score < g_best_score:
                    g_best_score = score
                    g_best_pos = particles[i].copy()
            else:
                stagnation_counters[i] += 1

        # 3. Population Management
        if iteration % 4 == 0:
            sorted_idx = np.argsort(current_scores)
            num_cull = int(num_particles * cull_ratio)

            # Method 1 & 2: Cull worst and spawn Crossover clones of best
            for j in range(num_cull):
                w_idx = sorted_idx[-(j+1)]
                p1_idx, p2_idx = np.random.choice(sorted_idx[:num_particles//3], 2)

                # Crossover: Position from Parent 1, Rotation/Ray from Parent 2
                particles[w_idx][:2] = p_best_pos[p1_idx][:2]
                particles[w_idx][2:] = p_best_pos[p2_idx][2:]

                # Mutation (Jitter)
                particles[w_idx] += np.random.normal(0, mutation_sigma)
                velocities[w_idx] *= 0.5 # Reset momentum for new spawns

        # Method 3: Stagnation Reset (Re-birth if stuck)
        for i in range(num_particles):
            if stagnation_counters[i] > stagnation_limit:
                new_pt = ribbon_pts[np.random.randint(len(ribbon_pts))] if ribbon_pts else bound_centroid
                particles[i] = [new_pt.x, new_pt.y, np.random.uniform(0, 2*np.pi), np.random.uniform(0, 2*np.pi)]
                stagnation_counters[i] = 0
                velocities[i] = np.random.uniform(-0.2, 0.2, 4)

        # 4. Momentum Update
        r1, r2 = np.random.rand(num_particles, 4), np.random.rand(num_particles, 4)
        velocities = (omega * velocities +
                      c1 * r1 * (p_best_pos - particles) +
                      c2 * r2 * (g_best_pos - particles))

        particles += velocities
        particles[:, 2:4] = np.mod(particles[:, 2:4], 2 * np.pi)

    propositions = [{'coords': p_best_settled[i], 'cost': p_best_score[i]}
                    for i in range(num_particles) if p_best_score[i] < 1e6]
    return finalize_propositions(propositions, top_n)


def evaluate_ray_placement(
    params, base_shape, shape_to_place, boundary, base_hull_area,
    bound_centroid, pt_push, min_dist, nudge_iters, pull_factor,
    ray_dirs: int = 16,
    propose_geom: Optional[ProposeGeometry] = None,
):
    curr_x, curr_y, curr_theta, _ = params

    m_theta = 0.0
    beta_theta = 0.9

    for i in range(1, nudge_iters + 1):
        decay = 1.0 - (i / nudge_iters)
        placed = propose_geom.placed_at((curr_x, curr_y, curr_theta)) if propose_geom is not None else None
        if propose_geom is not None:
            g = propose_geom.placement_guidance(placed, (curr_x, curr_y), pt_push)
            recovery_xy = (bound_centroid.x, bound_centroid.y)
            cx, cy = placed.center()
            lever_arm = np.array([cx - curr_x, cy - curr_y])
            attract_u = propose_geom.attraction_unit(placed, pt_push, (curr_x, curr_y))
            torque = lever_arm[0] * attract_u[1] - lever_arm[1] * attract_u[0]
            m_theta = beta_theta * m_theta + (1 - beta_theta) * torque
            curr_theta += m_theta * 0.3 * decay
            if abs(float(g.suggested_rotation_rad)) > 1e-6:
                delta = float(g.suggested_rotation_rad) - curr_theta
                while delta > np.pi:
                    delta -= 2 * np.pi
                while delta < -np.pi:
                    delta += 2 * np.pi
                curr_theta += delta * 0.3 * decay
            elif g.is_penetrating and g.alternative_rotations:
                alt = float(g.alternative_rotations[0])
                delta = alt - curr_theta
                while delta > np.pi:
                    delta -= 2 * np.pi
                while delta < -np.pi:
                    delta += 2 * np.pi
                curr_theta += delta * 0.2 * decay
        else:
            placed_shape = transform_poly(shape_to_place, (curr_x, curr_y, curr_theta))
            is_inside = boundary.contains(placed_shape)
            is_colliding = base_shape.intersects(placed_shape) if not base_shape.is_empty else False
            push_v = np.array([curr_x - pt_push.x, curr_y - pt_push.y])
            push_v_u = push_v / (np.linalg.norm(push_v) + 1e-6)
            if not base_shape.is_empty:
                from shapely.ops import nearest_points
                p_base, p_shape = nearest_points(base_shape, placed_shape)
                attract_v = np.array([p_base.x - p_shape.x, p_base.y - p_shape.y])
                attract_v_u = attract_v / (np.linalg.norm(attract_v) + 1e-6)
                lever_arm = np.array([
                    p_shape.x - placed_shape.centroid.x,
                    p_shape.y - placed_shape.centroid.y,
                ])
                torque = lever_arm[0] * attract_v_u[1] - lever_arm[1] * attract_v_u[0]
                m_theta = beta_theta * m_theta + (1 - beta_theta) * torque
                curr_theta += m_theta * 0.3 * decay
            else:
                attract_v_u = np.zeros(2)
            if not is_inside or is_colliding:
                to_center = np.array([bound_centroid.x - curr_x, bound_centroid.y - curr_y])
                active_dir = to_center / (np.linalg.norm(to_center) + 1e-6)
            else:
                active_dir = (attract_v_u * 0.7 + push_v_u * 0.3)

        best_ray_v = np.array([0.0, 0.0])
        max_int = -float('inf')

        if propose_geom is not None:
            dir_candidates = guidance_ray_direction_candidates(
                g, push_xy=(curr_x, curr_y), recovery_xy=recovery_xy,
            )
            base_attract_u = attract_u if propose_geom.scene.base_geoms else None
            for r_angle in np.linspace(0, 2 * np.pi, max(4, ray_dirs), endpoint=False):
                rv = np.array([np.cos(r_angle), np.sin(r_angle)])
                for cand_dir in dir_candidates:
                    intensity = np.dot(rv, cand_dir)
                    if base_attract_u is not None:
                        intensity += np.dot(rv, base_attract_u) * 0.5
                    if intensity > max_int:
                        max_int = intensity
                        best_ray_v = rv
        else:
            for r_angle in np.linspace(0, 2 * np.pi, max(4, ray_dirs), endpoint=False):
                rv = np.array([np.cos(r_angle), np.sin(r_angle)])
                intensity = np.dot(rv, active_dir)
                if not base_shape.is_empty:
                    intensity += np.dot(rv, attract_v_u) * 0.5
                if intensity > max_int:
                    max_int = intensity
                    best_ray_v = rv

        curr_x += best_ray_v[0] * pull_factor * 5.0 * decay
        curr_y += best_ray_v[1] * pull_factor * 5.0 * decay

    if propose_geom is not None:
        final_placed = propose_geom.placed_at((curr_x, curr_y, curr_theta))
        if not propose_geom.is_valid_placement(final_placed, pt_push, (curr_x, curr_y)):
            return 1e6, (curr_x, curr_y, curr_theta)
        final_shape = transform_poly(shape_to_place, (curr_x, curr_y, curr_theta))
    else:
        final_shape = transform_poly(shape_to_place, (curr_x, curr_y, curr_theta))
        dist_to_base = base_shape.distance(final_shape) if not base_shape.is_empty else 10.0
        if not (
            boundary.contains(final_shape)
            and dist_to_base >= min_dist
            and (not base_shape.intersects(final_shape) or base_shape.is_empty)
        ):
            return 1e6, (curr_x, curr_y, curr_theta)

    score = calculate_complex_score(
        base_shape, final_shape, base_hull_area, bound_centroid,
        pt_push, w_dist=0.001, w_dir=0.4, w_hull=0.1
    )
    return score, (curr_x, curr_y, curr_theta)


def sample_placement_points_ribbon(base_shape, shape_to_place, boundary, min_dist):
    # RIBBON SEARCH ZONE (Same logic, slightly wider for better capture)
    minx, miny, maxx, maxy = shape_to_place.bounds
    sample_step = max(maxx - minx, maxy - miny) * 0.4

    # Identify the "tightest" and "loosest" fit radii for sampling
    shape_to_place_center = shape_to_place.centroid
    r_min = shape_to_place.exterior.distance(shape_to_place_center)
    r_max = max([shape_to_place_center.distance(Point(p)) for p in shape_to_place.exterior.coords])

    if not base_shape.is_empty:
        outer_ribbon = base_shape.buffer(r_max + min_dist)
        inner_ribbon = base_shape.buffer(r_min + min_dist)
    else:
        outer_ribbon = boundary.buffer(-(r_min + min_dist))
        inner_ribbon = boundary.buffer(-(r_max + min_dist))
    search_zone = outer_ribbon.difference(inner_ribbon).intersection(boundary)

    samples = []
    for line in get_shape_exteriors(search_zone):
        num_pts = max(8, int(line.length / sample_step))
        for d in np.linspace(0, line.length, num_pts):
            samples.append(line.interpolate(d))
        samples.append(line.centroid)

    return tuple(samples)


def propose_placements_sheet_corners(
    shape_to_place: Polygon,
    sheet: Polygon,
    *,
    min_dist: float,
    num_angles: int = 24,
    top_n: int = 16,
    propose_geom: Optional[ProposeGeometry] = None,
    pt_push: Optional[Point] = None,
) -> List[Tuple[float, float, float]]:
    """Dense seeds near sheet vertices (corner nesting)."""
    propositions: list[dict] = []
    angles = np.linspace(0, 2 * np.pi, num_angles, endpoint=False)
    part_center = shape_to_place.centroid
    verts = list(sheet.exterior.coords)[:-1]
    offsets = [0.02, 0.04, 0.06, 0.08, 0.1, 0.12]

    for vx, vy in verts:
        for ox in offsets:
            for oy in offsets:
                seed = Point(vx + ox, vy + oy)
                if not sheet.contains(seed):
                    continue
                for angle in angles:
                    rotated = rotate(shape_to_place, angle, origin=(0, 0), use_radians=True)
                    rc = rotated.centroid
                    sx = seed.x + (part_center.x - rc.x)
                    sy = seed.y + (part_center.y - rc.y)
                    placed = translate(rotated, sx, sy)
                    if not sheet.contains(placed):
                        continue
                    border_dist = float(placed.distance(sheet.exterior))
                    if border_dist < min_dist - 1e-6:
                        continue
                    coords = (sx, sy, angle)
                    if propose_geom is not None and pt_push is not None:
                        placed_g = propose_geom.placed_at(coords)
                        if not propose_geom.is_valid_placement(
                            placed_g, pt_push, (sx, sy),
                        ):
                            continue
                    propositions.append({
                        "coords": coords,
                        "cost": border_dist,
                    })

    propositions.sort(key=lambda x: x["cost"])
    seen: set[tuple[float, float, float]] = set()
    out: list[tuple[float, float, float]] = []
    for p in propositions:
        key = (round(p["coords"][0], 2), round(p["coords"][1], 2), round(p["coords"][2], 1))
        if key in seen:
            continue
        seen.add(key)
        out.append(p["coords"])
        if len(out) >= top_n:
            break
    return out


def propose_placements_sheet_edge(
    shape_to_place: Polygon,
    sheet: Polygon,
    *,
    min_dist: float,
    num_angles: int = 12,
    top_n: int = 12,
    samples_per_edge: int = 16,
    base_shape: Optional[BaseGeometry] = None,
    propose_geom: Optional[ProposeGeometry] = None,
    pt_push: Optional[Point] = None,
) -> List[Tuple[float, float, float]]:
    """Slide the part along the sheet outer boundary with standoff min_dist."""
    interior = sheet.representative_point()
    angles = np.linspace(0, 2 * np.pi, num_angles, endpoint=False)
    propositions: list[dict] = []
    anchor_pts = _exterior_anchor_points(sheet, samples_per_edge)
    obstacle = base_shape if base_shape is not None else Polygon()

    for angle in angles:
        for contact in anchor_pts:
            ix = interior.x - contact.x
            iy = interior.y - contact.y
            dist = math.hypot(ix, iy)
            if dist < 1e-9:
                continue
            coords = _snap_coords_along_exterior(
                shape_to_place,
                sheet,
                contact,
                (ix / dist, iy / dist),
                angle,
                min_dist,
                container=sheet,
            )
            if coords is None:
                continue
            placed = transform_poly(shape_to_place, coords)
            if not obstacle.is_empty:
                if obstacle.intersects(placed):
                    continue
                if obstacle.distance(placed) < min_dist - 1e-6:
                    continue
            if propose_geom is not None and pt_push is not None:
                placed_g = propose_geom.placed_at(coords)
                if not propose_geom.is_valid_placement(
                    placed_g, pt_push, (coords[0], coords[1]),
                ):
                    continue
            err = _placement_contact_error(placed, sheet, min_dist, None)
            propositions.append({"coords": coords, "cost": err})

    propositions.sort(key=lambda x: x["cost"])
    seen: set[tuple[float, float, float]] = set()
    out: list[tuple[float, float, float]] = []
    for p in propositions:
        key = (round(p["coords"][0], 2), round(p["coords"][1], 2), round(p["coords"][2], 1))
        if key in seen:
            continue
        seen.add(key)
        out.append(p["coords"])
        if len(out) >= top_n:
            break
    return out


def propose_placements_ribbon_free(
    base_shape: BaseGeometry,
    shape_to_place: Polygon,
    sheet: Polygon,
    *,
    min_dist: float,
    num_angles: int = 8,
    top_n: int = 8,
    weight_dist: float = 0.5,
) -> List[Tuple[float, float, float]]:
    """Seed placements along the gap ribbon inside the free region."""
    free = placement_free_region(sheet, base_shape, min_dist)
    if free.is_empty:
        return []
    ribbon_pts = sample_placement_points_ribbon(base_shape, shape_to_place, sheet, min_dist)
    seeds: list[Point] = []
    for pt in ribbon_pts:
        if free.contains(pt):
            seeds.append(pt)
            continue
        boundary = free.boundary
        if boundary is not None and not boundary.is_empty and boundary.distance(pt) < 1e-6:
            seeds.append(pt)
    if not seeds:
        return []

    base_centroid = free.centroid
    base_hull_area = base_shape.convex_hull.area if not base_shape.is_empty else 0.0
    propositions: list[dict] = []
    angles = np.linspace(0, 2 * np.pi, num_angles, endpoint=False)
    for angle in angles:
        rotated_shape = rotate(shape_to_place, angle, origin=(0, 0), use_radians=True)
        for pt in seeds:
            placed_shape = translate(rotated_shape, pt.x, pt.y)
            if not free.contains(placed_shape):
                continue
            if not base_shape.is_empty:
                if base_shape.intersects(placed_shape):
                    continue
                if base_shape.distance(placed_shape) < min_dist - 1e-6:
                    continue
            dist_to_center = pt.distance(base_centroid)
            parts = [base_shape, placed_shape] if not base_shape.is_empty else [placed_shape]
            hull_growth = max(0, unary_union(parts).convex_hull.area - base_hull_area)
            total_cost = (weight_dist * dist_to_center) + ((1 - weight_dist) * np.sqrt(hull_growth))
            propositions.append({"coords": (pt.x, pt.y, angle), "cost": total_cost})
    propositions.sort(key=lambda x: x["cost"])
    return [p["coords"] for p in propositions[:top_n]]


def calculate_complex_score(base, placed, base_hull_area, centroid, pt_push, w_dist, w_dir, w_hull):
    # 1. Distance Component
    dist_to_center = Point(placed.centroid).distance(centroid)

    # 2. Directional Component (Dot Product)
    # We want to out of pt_push
    direction_score = -pt_push.distance(placed.centroid)

    pts = get_shape_polygons_coords(base) + get_shape_polygons_coords(placed)
    hull_growth = np.sqrt(max(0, MultiPoint(pts).convex_hull.area - base_hull_area))

    return (w_dist * dist_to_center) + (w_dir * direction_score) + (w_hull * hull_growth)


def finalize_propositions(propositions, top_n):
    """
    Sorts, deduplicates, and returns the top N propositions.
    """
    propositions.sort(key=lambda x: x['cost'])

    unique_props = []
    seen = set()
    for p in propositions:
        key = (round(p['coords'][0], 2), round(p['coords'][1], 2), round(p['coords'][2], 2))
        if key not in seen:
            unique_props.append(p['coords'])
            seen.add(key)
        if len(unique_props) >= top_n:
            break
    return unique_props


def propositions_to_ndarray(coords_list: Sequence[Tuple[float, float, float]]) -> np.ndarray:
    if not coords_list:
        return np.zeros((0, 3), dtype=np.float64)
    return np.asarray(coords_list, dtype=np.float64)


def base_shape_from_selection(
    polys: Sequence[BaseGeometry],
    selected_indices: Sequence[int],
) -> Union[Polygon, BaseGeometry]:
    placed = [polys[i] for i in selected_indices]
    if not placed:
        return Polygon()
    return unary_union(placed)


def _score_placement_coords(
    coords: Tuple[float, float, float],
    base_shape: BaseGeometry,
    shape_to_place: Polygon,
    boundary: BaseGeometry,
    pt_push: Point,
    min_dist: float,
    propose_geom: Optional[ProposeGeometry] = None,
) -> float:
    if propose_geom is not None:
        placed_geom = propose_geom.placed_at(coords)
        if not propose_geom.is_valid_placement(placed_geom, pt_push, (coords[0], coords[1])):
            return float("inf")
        placed = transform_poly(shape_to_place, coords)
    else:
        placed = transform_poly(shape_to_place, coords)
        if not boundary.contains(placed):
            return float("inf")
        if not base_shape.is_empty:
            if base_shape.intersects(placed):
                return float("inf")
            if base_shape.distance(placed) < min_dist - 1e-6:
                return float("inf")
    base_hull_area = base_shape.convex_hull.area if not base_shape.is_empty else 0.0
    return calculate_complex_score(
        base_shape,
        placed,
        base_hull_area,
        boundary.centroid,
        pt_push,
        w_dist=0.001,
        w_dir=0.4,
        w_hull=0.1,
    )


def _score_placement_legacy(
    coords: Tuple[float, float, float],
    base_shape: BaseGeometry,
    shape_to_place: Polygon,
    boundary: BaseGeometry,
    pt_push: Point,
    min_dist: float,
    propose_geom: ProposeGeometry,
) -> float:
    placed_geom = propose_geom.placed_at(coords)
    if not propose_geom.is_valid_placement(placed_geom, pt_push, (coords[0], coords[1])):
        return float("inf")
    placed = transform_poly(shape_to_place, coords)
    base_hull_area = base_shape.convex_hull.area if not base_shape.is_empty else 0.0
    return calculate_complex_score(
        base_shape,
        placed,
        base_hull_area,
        boundary.centroid,
        pt_push,
        w_dist=0.001,
        w_dir=0.4,
        w_hull=0.1,
    )


def _score_placement_clearance(
    coords: Tuple[float, float, float],
    propose_geom: ProposeGeometry,
    pt_push: Point,
) -> float:
    placed_geom = propose_geom.placed_at(coords)
    if not propose_geom.is_valid_placement(placed_geom, pt_push, (coords[0], coords[1])):
        return float("-inf")
    g = propose_geom.placement_guidance(placed_geom, (coords[0], coords[1]), pt_push)
    if g.is_penetrating:
        return float("-inf")
    return float(g.clearance)


def _score_placement_hybrid(
    coords: Tuple[float, float, float],
    base_shape: BaseGeometry,
    shape_to_place: Polygon,
    boundary: BaseGeometry,
    pt_push: Point,
    min_dist: float,
    propose_geom: ProposeGeometry,
    propose_cfg: ProposeConfig,
) -> float:
    clearance = _score_placement_clearance(coords, propose_geom, pt_push)
    if clearance == float("-inf"):
        return float("-inf")
    legacy = _score_placement_legacy(
        coords, base_shape, shape_to_place, boundary, pt_push, min_dist, propose_geom,
    )
    if legacy == float("inf"):
        return float("-inf")
    w_c = propose_cfg.ranking_clearance_weight
    w_h = propose_cfg.ranking_hull_weight
    return w_c * clearance - w_h * legacy


def _score_placement_border(
    coords: Tuple[float, float, float],
    shape_to_place: Polygon,
    propose_geom: ProposeGeometry,
    pt_push: Point,
    min_dist: float,
) -> float:
    """Higher score = tighter fit to sheet border (lower exterior distance)."""
    placed_geom = propose_geom.placed_at(coords)
    if not propose_geom.is_valid_placement(placed_geom, pt_push, (coords[0], coords[1])):
        return float("-inf")
    placed = transform_poly(shape_to_place, coords)
    border_dist = float(placed.distance(propose_geom.sheet.exterior))
    err = abs(border_dist - min_dist)
    return -err


def _score_placement_contact(
    coords: Tuple[float, float, float],
    shape_to_place: Polygon,
    propose_geom: ProposeGeometry,
    pt_push: Point,
    min_dist: float,
    focal_shape: Optional[BaseGeometry] = None,
) -> float:
    """Higher score = tighter fit to sheet border and/or focal group."""
    placed_geom = propose_geom.placed_at(coords)
    if not propose_geom.is_valid_placement(placed_geom, pt_push, (coords[0], coords[1])):
        return float("-inf")
    placed = transform_poly(shape_to_place, coords)
    err = _placement_contact_error(
        placed, propose_geom.sheet, min_dist, focal_shape,
    )
    return -err


def _score_placement_contact_hybrid(
    coords: Tuple[float, float, float],
    shape_to_place: Polygon,
    propose_geom: ProposeGeometry,
    pt_push: Point,
    min_dist: float,
    focal_shape: Optional[BaseGeometry],
    clearance_weight: float,
) -> float:
    contact = _score_placement_contact(
        coords, shape_to_place, propose_geom, pt_push, min_dist, focal_shape,
    )
    if contact == float("-inf"):
        return float("-inf")
    clearance = _score_placement_clearance(coords, propose_geom, pt_push)
    if clearance == float("-inf"):
        return contact
    return contact + clearance_weight * clearance


def _trim_candidates_stratified(
    candidates: Sequence[Tuple[float, float, float]],
    shape_to_place: Polygon,
    propose_geom: ProposeGeometry,
    pt_push: Point,
    min_dist: float,
    limit: int,
    focal_shape: Optional[BaseGeometry],
    *,
    contact_fraction: float,
    rank_mode: str,
    clearance_weight: float,
) -> List[Tuple[float, float, float]]:
    """Keep edge-fit and pocket candidates when trimming an oversized pool."""
    if limit <= 0 or not candidates:
        return []
    n_contact = max(1, min(limit, int(round(limit * contact_fraction))))
    n_clear = max(0, limit - n_contact)

    def score_fn(coords: Tuple[float, float, float]) -> float:
        if rank_mode == "contact_hybrid":
            return _score_placement_contact_hybrid(
                coords, shape_to_place, propose_geom, pt_push, min_dist,
                focal_shape, clearance_weight,
            )
        return _score_placement_contact(
            coords, shape_to_place, propose_geom, pt_push, min_dist, focal_shape,
        )

    scored_contact: list[tuple[float, Tuple[float, float, float]]] = []
    seen: set[tuple[float, float, float]] = set()
    for coords in candidates:
        key = (round(coords[0], 3), round(coords[1], 3), round(coords[2], 3))
        if key in seen:
            continue
        seen.add(key)
        s = score_fn(coords)
        if s > float("-inf"):
            scored_contact.append((s, coords))
    scored_contact.sort(key=lambda x: x[0], reverse=True)
    picked = [coords for _, coords in scored_contact[:n_contact]]
    picked_keys = {
        (round(c[0], 3), round(c[1], 3), round(c[2], 3)) for c in picked
    }

    if n_clear <= 0:
        return picked

    remaining = [
        c for c in candidates
        if (round(c[0], 3), round(c[1], 3), round(c[2], 3)) not in picked_keys
    ]
    clearance_picked = _trim_candidates_by_clearance(
        remaining, propose_geom, pt_push, n_clear,
    )
    out = list(picked)
    for c in clearance_picked:
        key = (round(c[0], 3), round(c[1], 3), round(c[2], 3))
        if key not in picked_keys:
            out.append(c)
            picked_keys.add(key)
        if len(out) >= limit:
            break
    return out[:limit]


def _trim_candidates_by_contact(
    candidates: Sequence[Tuple[float, float, float]],
    shape_to_place: Polygon,
    propose_geom: ProposeGeometry,
    pt_push: Point,
    min_dist: float,
    limit: int,
    focal_shape: Optional[BaseGeometry] = None,
) -> List[Tuple[float, float, float]]:
    if limit <= 0 or not candidates:
        return []
    scored: list[tuple[float, Tuple[float, float, float]]] = []
    seen: set[tuple[float, float, float]] = set()
    for coords in candidates:
        key = (round(coords[0], 3), round(coords[1], 3), round(coords[2], 3))
        if key in seen:
            continue
        seen.add(key)
        score = _score_placement_contact(
            coords, shape_to_place, propose_geom, pt_push, min_dist, focal_shape,
        )
        if score > float("-inf"):
            scored.append((score, coords))
    scored.sort(key=lambda x: x[0], reverse=True)
    return [coords for _, coords in scored[:limit]]


def _trim_candidates_by_clearance(
    candidates: Sequence[Tuple[float, float, float]],
    propose_geom: ProposeGeometry,
    pt_push: Point,
    limit: int,
) -> List[Tuple[float, float, float]]:
    """Keep up to limit candidates with highest placement clearance."""
    if limit <= 0 or not candidates:
        return []
    scored: list[tuple[float, Tuple[float, float, float]]] = []
    seen: set[tuple[float, float, float]] = set()
    for coords in candidates:
        key = (round(coords[0], 3), round(coords[1], 3), round(coords[2], 3))
        if key in seen:
            continue
        seen.add(key)
        score = _score_placement_clearance(coords, propose_geom, pt_push)
        if score > float("-inf"):
            scored.append((score, coords))
    scored.sort(key=lambda x: x[0], reverse=True)
    return [coords for _, coords in scored[:limit]]


def _rank_proposal_coords(
    candidates: Sequence[Tuple[float, float, float]],
    base_shape: BaseGeometry,
    shape_to_place: Polygon,
    boundary: BaseGeometry,
    pt_push: Point,
    min_dist: float,
    max_n: int,
    propose_geom: ProposeGeometry,
    propose_cfg: ProposeConfig,
    *,
    rank_mode: Optional[str] = None,
    focal_shape: Optional[BaseGeometry] = None,
) -> List[Tuple[float, float, float]]:
    """Rank candidates; higher score is better for clearance/hybrid, lower for legacy."""
    scored: list[tuple[float, Tuple[float, float, float]]] = []
    seen: set[tuple[float, float, float]] = set()
    mode = rank_mode if rank_mode is not None else propose_cfg.ranking_mode
    for coords in candidates:
        key = (round(coords[0], 3), round(coords[1], 3), round(coords[2], 3))
        if key in seen:
            continue
        seen.add(key)
        if mode == "border":
            score = _score_placement_border(
                coords, shape_to_place, propose_geom, pt_push, min_dist,
            )
        elif mode == "contact":
            score = _score_placement_contact(
                coords, shape_to_place, propose_geom, pt_push, min_dist, focal_shape,
            )
        elif mode == "contact_hybrid":
            score = _score_placement_contact_hybrid(
                coords, shape_to_place, propose_geom, pt_push, min_dist, focal_shape,
                propose_cfg.contact_clearance_hybrid_weight,
            )
        elif mode == "clearance":
            score = _score_placement_clearance(coords, propose_geom, pt_push)
        elif mode == "hybrid":
            score = _score_placement_hybrid(
                coords, base_shape, shape_to_place, boundary, pt_push, min_dist,
                propose_geom, propose_cfg,
            )
        else:
            score = -_score_placement_legacy(
                coords, base_shape, shape_to_place, boundary, pt_push, min_dist, propose_geom,
            )
        if score > float("-inf") and score < float("inf"):
            scored.append((score, coords))
        elif mode == "legacy" and score > float("-inf"):
            scored.append((score, coords))
    reverse = mode not in ("legacy",)
    scored.sort(key=lambda x: x[0], reverse=reverse)
    return [coords for _, coords in scored[:max_n]]


def collect_propose_candidates(
    base_shape: BaseGeometry,
    shape_to_place: Polygon,
    sheet: Polygon,
    propose_cfg: ProposeConfig,
    *,
    min_dist: float,
    pt_push: Point,
    propose_geom: ProposeGeometry,
    use_free_region: bool,
    use_point_cloud: bool,
    use_guidance_walk: bool,
    focal_shape: Optional[BaseGeometry] = None,
) -> List[Tuple[float, float, float]]:
    pool = propose_cfg.candidate_pool
    border_focus = should_use_border_focus(base_shape, propose_cfg)
    candidates: List[Tuple[float, float, float]] = []
    candidates.extend(
        propose_placements_erosion(
            base_shape,
            shape_to_place,
            boundary=sheet,
            sheet=sheet,
            use_free_region=use_free_region,
            min_dist=min_dist,
            top_n=pool,
            num_angles=propose_cfg.erosion_num_angles,
            multi_site=propose_cfg.multi_site_erosion,
            focal_shape=focal_shape,
            border_focus=border_focus,
        )
    )
    candidates.extend(
        propose_placements_raycasting(
            base_shape,
            shape_to_place,
            boundary=sheet,
            sheet=sheet,
            use_free_region=use_free_region,
            min_dist=min_dist,
            top_n=pool,
            num_rays=propose_cfg.raycast_num_rays,
            num_angles=propose_cfg.raycast_num_angles,
            anchor_stride=propose_cfg.raycast_anchor_stride,
            focal_shape=focal_shape,
            border_focus=border_focus,
        )
    )
    if propose_cfg.use_voronoi:
        candidates.extend(
            propose_placements_voronoi(
                base_shape,
                shape_to_place,
                boundary=sheet,
                sheet=sheet,
                use_free_region=use_free_region,
                min_dist=min_dist,
                top_n=pool,
                num_angles=propose_cfg.voronoi_num_angles,
                densify_divisor=propose_cfg.voronoi_densify_divisor,
                max_sites=propose_cfg.voronoi_max_sites,
                focal_shape=focal_shape,
                border_focus=border_focus,
            )
        )
    if use_point_cloud or propose_cfg.use_point_cloud:
        candidates.extend(
            propose_placements_point_cloud(
                base_shape,
                shape_to_place,
                sheet,
                pt_push=pt_push,
                min_dist=min_dist,
                top_n=pool,
                num_particles=propose_cfg.point_cloud_particles,
                max_iterations=propose_cfg.point_cloud_iterations,
                nudge_iters=propose_cfg.point_cloud_nudge_iters,
                ray_dirs=propose_cfg.point_cloud_ray_dirs,
                cull_ratio=propose_cfg.point_cloud_cull_ratio,
                propose_geom=propose_geom,
            )
        )
    if use_guidance_walk or propose_cfg.use_guidance_walk:
        candidates.extend(
            propose_placements_guidance_walk(
                base_shape,
                shape_to_place,
                sheet,
                pt_push,
                propose_geom,
                min_dist=min_dist,
                top_n=pool,
            )
        )
    if propose_cfg.use_ribbon_seeds and use_free_region:
        candidates.extend(
            propose_placements_ribbon_free(
                base_shape,
                shape_to_place,
                sheet,
                min_dist=min_dist,
                num_angles=propose_cfg.erosion_num_angles,
                top_n=pool,
            )
        )
    if (
        propose_cfg.use_group_edge_seeds
        and focal_shape is not None
        and not focal_shape.is_empty
        and not base_shape.is_empty
        and use_free_region
    ):
        candidates.extend(
            propose_placements_group_fit(
                focal_shape,
                shape_to_place,
                sheet,
                base_shape,
                min_dist=min_dist,
                num_angles=max(propose_cfg.erosion_num_angles, 12),
                top_n=pool * 2,
                samples_per_edge=propose_cfg.group_edge_samples_per_edge,
                propose_geom=propose_geom,
                pt_push=pt_push,
            )
        )
    if propose_cfg.use_border_edge_seeds and should_use_border_focus(base_shape, propose_cfg):
        candidates.extend(
            propose_placements_sheet_corners(
                shape_to_place,
                sheet,
                min_dist=min_dist,
                num_angles=max(propose_cfg.erosion_num_angles * 4, 24),
                top_n=pool * 2,
                propose_geom=propose_geom,
                pt_push=pt_push,
            )
        )
        candidates.extend(
            propose_placements_sheet_edge(
                shape_to_place,
                sheet,
                min_dist=min_dist,
                num_angles=max(propose_cfg.erosion_num_angles, 12),
                top_n=pool * 2,
                samples_per_edge=propose_cfg.sheet_edge_samples_per_edge,
                base_shape=base_shape,
                propose_geom=propose_geom,
                pt_push=pt_push,
            )
        )
    return candidates


def _trim_candidates_by_border(
    candidates: Sequence[Tuple[float, float, float]],
    shape_to_place: Polygon,
    propose_geom: ProposeGeometry,
    pt_push: Point,
    min_dist: float,
    limit: int,
) -> List[Tuple[float, float, float]]:
    if limit <= 0 or not candidates:
        return []
    scored: list[tuple[float, Tuple[float, float, float]]] = []
    seen: set[tuple[float, float, float]] = set()
    for coords in candidates:
        key = (round(coords[0], 3), round(coords[1], 3), round(coords[2], 3))
        if key in seen:
            continue
        seen.add(key)
        score = _score_placement_border(coords, shape_to_place, propose_geom, pt_push, min_dist)
        if score > float("-inf"):
            scored.append((score, coords))
    scored.sort(key=lambda x: x[0], reverse=True)
    return [coords for _, coords in scored[:limit]]


def _propose_coords_from_candidates(
    base_shape: BaseGeometry,
    shape_to_place: Polygon,
    boundary: BaseGeometry,
    propose_cfg: ProposeConfig,
    *,
    min_dist: float,
    pt_push: Point,
    candidates: Sequence[Tuple[float, float, float]],
    rank_mode: str,
    focal_shape: Optional[BaseGeometry] = None,
) -> List[Tuple[float, float, float]]:
    geom = ProposeGeometry(
        boundary,
        base_shape,
        shape_to_place,
        min_dist,
        epsilon_ratio=propose_cfg.placement_clearance_epsilon_ratio,
    )
    pool = list(candidates)
    if len(pool) > propose_cfg.candidate_pool:
        if rank_mode == "border" and propose_cfg.trim_candidates_by_clearance:
            pool = _trim_candidates_by_border(
                pool, shape_to_place, geom, pt_push, min_dist, propose_cfg.candidate_pool,
            )
        elif rank_mode in ("contact", "contact_hybrid") and propose_cfg.trim_candidates_by_clearance:
            if propose_cfg.use_stratified_contact_trim and rank_mode == "contact_hybrid":
                pool = _trim_candidates_stratified(
                    pool,
                    shape_to_place,
                    geom,
                    pt_push,
                    min_dist,
                    propose_cfg.candidate_pool,
                    focal_shape,
                    contact_fraction=propose_cfg.contact_trim_fraction,
                    rank_mode=rank_mode,
                    clearance_weight=propose_cfg.contact_clearance_hybrid_weight,
                )
            else:
                pool = _trim_candidates_by_contact(
                    pool,
                    shape_to_place,
                    geom,
                    pt_push,
                    min_dist,
                    propose_cfg.candidate_pool,
                    focal_shape,
                )
        elif (
            propose_cfg.trim_candidates_by_clearance
            and rank_mode in ("clearance", "hybrid")
        ):
            pool = _trim_candidates_by_clearance(
                pool, geom, pt_push, propose_cfg.candidate_pool,
            )
    return _rank_proposal_coords(
        pool,
        base_shape,
        shape_to_place,
        boundary,
        pt_push,
        min_dist,
        propose_cfg.max_proposals,
        geom,
        propose_cfg,
        rank_mode=rank_mode,
        focal_shape=focal_shape,
    )


def propose_coords_with_strategy(
    base_shape: BaseGeometry,
    shape_to_place: Polygon,
    boundary: BaseGeometry,
    propose_cfg: ProposeConfig,
    *,
    min_dist: float,
    pt_push: Point,
    focal_shape: Optional[BaseGeometry] = None,
) -> List[Tuple[float, float, float]]:
    cfg = propose_cfg
    use_free = cfg.use_free_region_search
    use_pso = cfg.use_point_cloud
    use_walk = cfg.use_guidance_walk
    rank_mode = effective_ranking_mode(cfg, base_shape)
    border_focus = should_use_border_focus(base_shape, cfg)
    if focal_shape is None:
        if border_focus:
            focal_shape = border_focal_for_propose(boundary, min_dist)
        elif base_shape is not None and not base_shape.is_empty:
            focal_shape = base_shape
    if border_focus:
        pt_push = propose_push_point(
            boundary,
            base_shape,
            smart_push=cfg.smart_push_target,
            min_dist=min_dist,
            use_border_focus=True,
        )
    sheet, _voids = board_context_from_geometry(boundary)
    geom = ProposeGeometry(
        boundary,
        base_shape,
        shape_to_place,
        min_dist,
        epsilon_ratio=cfg.placement_clearance_epsilon_ratio,
    )
    candidates = collect_propose_candidates(
        base_shape,
        shape_to_place,
        sheet,
        cfg,
        min_dist=min_dist,
        pt_push=pt_push,
        propose_geom=geom,
        use_free_region=use_free,
        use_point_cloud=use_pso,
        use_guidance_walk=use_walk,
        focal_shape=focal_shape,
    )
    return _propose_coords_from_candidates(
        base_shape,
        shape_to_place,
        boundary,
        cfg,
        min_dist=min_dist,
        pt_push=pt_push,
        candidates=candidates,
        rank_mode=rank_mode,
        focal_shape=focal_shape,
    )


def _best_proposer_coords(
    base_shape: BaseGeometry,
    shape_to_place: Polygon,
    boundary: BaseGeometry,
    propose_cfg: ProposeConfig,
    *,
    min_dist: float,
    pt_push: Point,
    focal_shape: Optional[BaseGeometry] = None,
) -> List[Tuple[float, float, float]]:
    """All proposers; rank with configured search region and ranking mode."""
    return propose_coords_with_strategy(
        base_shape,
        shape_to_place,
        boundary,
        propose_cfg,
        min_dist=min_dist,
        pt_push=pt_push,
        focal_shape=focal_shape,
    )


def proposed_transforms_for_groups(
    board: BaseGeometry,
    parts: Sequence[Tuple[Polygon, int]],
    selected_polys: Sequence[BaseGeometry],
    selected_indices: Sequence[int],
    propose_cfg: ProposeConfig,
    *,
    min_dist: float,
    pt_push: Optional[Point] = None,
) -> dict[int, np.ndarray]:
    """Propose (x, y, angle) seeds per part group.

    Propose uses the nearest packed cluster as obstacles only.
    ``make_polygon_graph`` still filters against the full selection.
    """
    placed = [selected_polys[i] for i in selected_indices]
    out: dict[int, np.ndarray] = {}
    for part_poly, group_id in parts:
        obstacle_shape = obstacle_shape_for_propose(placed, part_poly, min_dist)
        focal = focal_shape_for_propose(
            board, placed, part_poly, min_dist, propose_cfg,
        )
        border_focus = should_use_border_focus(obstacle_shape, propose_cfg)
        push = pt_push if pt_push is not None else propose_push_point(
            board,
            obstacle_shape,
            smart_push=propose_cfg.smart_push_target,
            min_dist=min_dist,
            use_border_focus=border_focus,
        )
        coords = _best_proposer_coords(
            obstacle_shape,
            part_poly,
            board,
            propose_cfg,
            min_dist=min_dist,
            pt_push=push,
            focal_shape=focal,
        )
        out[group_id] = propositions_to_ndarray(coords)
    return out
