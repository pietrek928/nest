import math
from typing import List, Optional, Sequence, Tuple, Union

import numpy as np
from shapely import LineString, LinearRing, MultiLineString, MultiPoint, MultiPolygon, Point, Polygon
from shapely.affinity import rotate, translate
from shapely.geometry.base import BaseGeometry
from shapely.ops import nearest_points, polylabel, unary_union, voronoi_diagram

from nest_graph.board import board_context_from_geometry
from nest_graph.config import ProposeConfig, dedupe_transforms
from nest_graph.geometry import Geometry
from nest_graph.placement_scene import (
    PLACEMENT_EPSILON_RATIO,
    best_proposition,
    build_placement_scene,
    guidance_config_for_propose,
    guidance_config_for_scene,
    guidance_ray_direction_candidates,
    is_valid_placement,
    placement_footprint_inside_board,
    footprints_inside_board,
    proposition_translation,
    tiered_propositions,
)
from nest_graph.utils import get_shape_exteriors, get_shape_polygons_coords, transform_poly

from nest_graph.propose.context import _placement_contact_error, placement_free_region
from nest_graph.propose.geometry import ProposeGeometry
from nest_graph.propose.placement_common import (
    _exterior_anchor_points,
    _propose_valid_at,
    _snap_coords_along_exterior,
)

def _board_edge_snap_seeds(
    shape_to_place: Polygon,
    sheet: Polygon,
    base_shape: BaseGeometry,
    *,
    min_dist: float,
    num_angles: int,
    samples_per_edge: int,
    propose_geom: ProposeGeometry,
    pt_push: Point,
    top_n: int,
) -> list[tuple[tuple[float, float, float], Point, tuple[float, float]]]:
    """Snap seeds along nest outline; return (coords, anchor, inward) for guidance refine."""
    interior = sheet.representative_point()
    angles = np.linspace(0, 2 * np.pi, num_angles, endpoint=False)
    propositions: list[dict] = []
    anchor_pts = _exterior_anchor_points(sheet, samples_per_edge)

    def _add_seed(
        coords: tuple[float, float, float],
        anchor: Point,
        inward: tuple[float, float],
        cost: float,
    ) -> None:
        propositions.append({
            "coords": coords,
            "anchor": anchor,
            "inward": inward,
            "cost": cost,
        })

    for angle in angles:
        for contact in anchor_pts:
            ox = contact.x - interior.x
            oy = contact.y - interior.y
            dist = math.hypot(ox, oy)
            if dist < 1e-9:
                continue
            inward = (ox / dist, oy / dist)
            coords = _snap_coords_along_exterior(
                shape_to_place,
                sheet,
                contact,
                inward,
                angle,
                min_dist,
            )
            if coords is None:
                continue
            placed = transform_poly(shape_to_place, coords)
            if not base_shape.is_empty:
                if base_shape.intersects(placed):
                    continue
                if base_shape.distance(placed) < min_dist - 1e-6:
                    continue
            if not _propose_valid_at(coords, propose_geom, pt_push):
                continue
            err = _placement_contact_error(placed, sheet, min_dist, None)
            _add_seed(coords, contact, inward, err)

    corner_coords = propose_placements_sheet_corners(
        shape_to_place,
        sheet,
        min_dist,
        propose_geom=propose_geom,
        pt_push=pt_push,
        num_angles=max(num_angles * 2, 16),
        top_n=top_n,
    )
    for coords in corner_coords:
        placed = transform_poly(shape_to_place, coords)
        anchor_pt, _ = nearest_points(sheet.exterior, placed)
        ox = anchor_pt.x - interior.x
        oy = anchor_pt.y - interior.y
        dist = math.hypot(ox, oy)
        if dist < 1e-9:
            continue
        inward = (ox / dist, oy / dist)
        err = _placement_contact_error(placed, sheet, min_dist, None)
        _add_seed(coords, anchor_pt, inward, err)

    propositions.sort(key=lambda x: x["cost"])
    seen: set[tuple[float, float, float]] = set()
    out: list[tuple[tuple[float, float, float], Point, tuple[float, float]]] = []
    for p in propositions:
        key = (round(p["coords"][0], 2), round(p["coords"][1], 2), round(p["coords"][2], 1))
        if key in seen:
            continue
        seen.add(key)
        out.append((p["coords"], p["anchor"], p["inward"]))
        if len(out) >= top_n:
            break
    return out


def propose_placements_board_edge(
    shape_to_place: Polygon,
    sheet: Polygon,
    base_shape: BaseGeometry,
    *,
    min_dist: float,
    propose_cfg: ProposeConfig,
    propose_geom: ProposeGeometry,
    pt_push: Point,
    num_angles: int | None = None,
    samples_per_edge: int | None = None,
    top_n: int = 16,
    guidance_refine: bool | None = None,
) -> List[Tuple[float, float, float]]:
    """Dock along nest outline: geometric snap seeds + optional per-edge guidance cast."""
    if sheet.is_empty:
        return []

    n_angles = num_angles if num_angles is not None else propose_cfg.placement_num_angles
    samples = (
        samples_per_edge
        if samples_per_edge is not None
        else propose_cfg.board_edge_samples_per_edge
    )
    refine = (
        guidance_refine
        if guidance_refine is not None
        else propose_cfg.board_edge_guidance_refine
    )

    seed_anchors = _board_edge_snap_seeds(
        shape_to_place,
        sheet,
        base_shape,
        min_dist=min_dist,
        propose_geom=propose_geom,
        pt_push=pt_push,
        num_angles=n_angles,
        samples_per_edge=samples,
        top_n=top_n * 2 if refine else top_n,
    )
    if not seed_anchors:
        return []

    snap_coords = [coords for coords, _anchor, _inward in seed_anchors]
    if not refine:
        return snap_coords[:top_n]

    from nest_graph.propose.placements_guidance import (
        propose_placements_board_edge_guidance_cast,
    )

    refined = propose_placements_board_edge_guidance_cast(
        seed_anchors,
        pt_push,
        propose_geom,
        propose_cfg,
        min_dist=min_dist,
        top_n=top_n,
    )
    merged: list[tuple[float, float, float]] = []
    seen: set[tuple[float, float, float]] = set()
    for coords in refined + snap_coords:
        key = (round(coords[0], 2), round(coords[1], 2), round(coords[2], 1))
        if key in seen:
            continue
        seen.add(key)
        merged.append(coords)
        if len(merged) >= top_n:
            break
    merged.sort(
        key=lambda c: _placement_contact_error(
            transform_poly(shape_to_place, c), sheet, min_dist, None,
        ),
    )
    return merged[:top_n]


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
    min_dist: float,
    *,
    propose_geom: ProposeGeometry,
    pt_push: Point,
    num_angles: int = 24,
    top_n: int = 16,
) -> List[Tuple[float, float, float]]:
    """Perfect corner docking using bounding box alignment to the safe zone."""
    propositions: list[dict] = []
    angles = np.linspace(0, 2 * np.pi, num_angles, endpoint=False)

    # 1. Generate the absolute mathematically safe corner vertices
    eroded_sheet = sheet.buffer(-min_dist)
    if eroded_sheet.is_empty:
        return []

    safe_corners = []
    for ring in get_shape_exteriors(eroded_sheet):
        safe_corners.extend(list(ring.coords)[:-1])

    for angle in angles:
        rotated = rotate(shape_to_place, angle, origin=(0, 0), use_radians=True)

        # 2. Extract local extremes of the rotated shape
        minx, miny, maxx, maxy = rotated.bounds

        for cx, cy in safe_corners:
            # 3. Calculate exact translation to align shape corners with sheet corners
            alignments = [
                (cx - minx, cy - miny),  # Dock Bottom-Left
                (cx - maxx, cy - miny),  # Dock Bottom-Right
                (cx - minx, cy - maxy),  # Dock Top-Left
                (cx - maxx, cy - maxy)   # Dock Top-Right
            ]

            for dx, dy in alignments:
                placed = translate(rotated, dx, dy)

                if not sheet.contains(placed):
                    continue

                border_dist = float(placed.distance(sheet.exterior))
                if border_dist < min_dist - 1e-6:
                    continue

                coords = (dx, dy, angle)
                if not _propose_valid_at(coords, propose_geom, pt_push):
                    continue

                propositions.append({
                    "coords": coords,
                    "cost": border_dist, # Lower is a tighter fit
                })

    propositions.sort(key=lambda x: x["cost"])

    # Deduplicate
    seen: set[tuple[float, float, float]] = set()
    out: list[tuple[float, float, float]] = []
    for p in propositions:
        key = (round(p["coords"][0], 2), round(p["coords"][1], 2), round(p["coords"][2], 1))
        if key not in seen:
            seen.add(key)
            out.append(p["coords"])
            if len(out) >= top_n:
                break

    return out


def propose_placements_sheet_edge(
    shape_to_place: Polygon,
    sheet: Polygon,
    min_dist: float,
    *,
    propose_geom: ProposeGeometry,
    pt_push: Point,
    num_angles: int = 12,
    top_n: int = 12,
    samples_per_edge: int = 16,
    base_shape: Optional[BaseGeometry] = None,
) -> List[Tuple[float, float, float]]:
    """Slide the part along the exact perimeter of the safe zone."""
    angles = np.linspace(0, 2 * np.pi, num_angles, endpoint=False)
    propositions: list[dict] = []
    obstacle = base_shape if base_shape is not None else Polygon()

    # 1. The Safe Halo guarantees min_dist is respected perpendicularly
    safe_halo = sheet.buffer(-min_dist)
    if safe_halo.is_empty:
        return []

    halo_pts = _exterior_anchor_points(safe_halo, samples_per_edge)

    for angle in angles:
        rotated = rotate(shape_to_place, angle, origin=(0, 0), use_radians=True)
        minx, miny, maxx, maxy = rotated.bounds

        # Calculate distance from origin(0,0) to bounding edges
        dx_center = (maxx + minx) / 2.0
        dy_center = (maxy + miny) / 2.0
        width_half = (maxx - minx) / 2.0
        height_half = (maxy - miny) / 2.0

        for h_pt in halo_pts:
            # 2. Test snapping all 4 extreme edges to the halo point
            alignments = [
                (h_pt.x - minx, h_pt.y - dy_center), # Snap Left Edge
                (h_pt.x - maxx, h_pt.y - dy_center), # Snap Right Edge
                (h_pt.x - dx_center, h_pt.y - miny), # Snap Bottom Edge
                (h_pt.x - dx_center, h_pt.y - maxy)  # Snap Top Edge
            ]

            for dx, dy in alignments:
                placed = translate(rotated, dx, dy)

                if not obstacle.is_empty:
                    if obstacle.intersects(placed) or obstacle.distance(placed) < min_dist - 1e-6:
                        continue

                coords = (dx, dy, angle)
                if not _propose_valid_at(coords, propose_geom, pt_push):
                    continue

                err = _placement_contact_error(placed, sheet, min_dist, None)
                propositions.append({"coords": coords, "cost": err})

    propositions.sort(key=lambda x: x["cost"])

    seen: set[tuple[float, float, float]] = set()
    out: list[tuple[float, float, float]] = []
    for p in propositions:
        key = (round(p["coords"][0], 2), round(p["coords"][1], 2), round(p["coords"][2], 1))
        if key not in seen:
            seen.add(key)
            out.append(p["coords"])
            if len(out) >= top_n:
                break
    return out


def propose_placements_ribbon_free(
    base_shape: BaseGeometry,
    shape_to_place: Polygon,
    sheet: Polygon,
    min_dist: float,
    *,
    num_angles: int = 8,
    top_n: int = 8,
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
            propositions.append({
                "coords": (pt.x, pt.y, angle),
                "cost": float(pt.distance(base_centroid)),
            })
    propositions.sort(key=lambda x: x["cost"])
    return [p["coords"] for p in propositions[:top_n]]
