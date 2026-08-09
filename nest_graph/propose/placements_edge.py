from typing import List, Optional, Tuple

import math
import numpy as np
from shapely import Point, Polygon
from shapely.geometry.base import BaseGeometry

from nest_graph.config import ProposeConfig
from nest_graph.geometry import Geometry
from nest_graph.utils import get_shape_exteriors

from nest_graph.propose.context import placement_contact_error, placement_free_region
from nest_graph.propose.geometry import ProposeGeometry, filter_candidates_batch
from nest_graph.propose.placement_common import is_pose_clear
from nest_graph.propose.placement_outline import (
    inward_at_contact,
    outline_ring_geom,
    snap_coords_along_exterior,
)
from nest_graph.propose.placement_perimeter import (
    angles_for_edge_contact,
    anti_crowd_far_segments,
    anti_crowd_segment_weights,
    exterior_anchor_points,
    finalize_edge_propositions,
    select_stratified_by_segment,
    select_stratified_by_segment_weighted,
    _segment_index_at_point,
)


def _sheet_exterior_snap_propositions(
    shape_to_place: Polygon,
    sheet: Polygon,
    *,
    min_dist: float,
    num_angles: int,
    samples_per_edge: int,
    propose_geom: ProposeGeometry,
    segment_filter: set[int] | None = None,
) -> list[dict]:
    """Shared exterior snap loop: anchors → angles → snap → contact error dicts."""
    propositions: list[dict] = []
    for contact in exterior_anchor_points(sheet, samples_per_edge):
        if segment_filter is not None:
            seg_i = _segment_index_at_point(sheet, contact)
            if seg_i not in segment_filter:
                continue
        snap_contact, inward = inward_at_contact(sheet, contact)
        for angle in angles_for_edge_contact(sheet, contact, num_angles):
            coords = snap_coords_along_exterior(
                shape_to_place,
                sheet,
                snap_contact,
                inward,
                angle,
                min_dist,
                container=sheet,
                propose_geom=propose_geom,
                boundary_ring_geom=propose_geom.boundary_ring_geom,
            )
            if coords is None:
                continue
            placed_geom = propose_geom.placed_at(coords)
            if placed_geom is None:
                continue
            err = placement_contact_error(placed_geom, sheet, min_dist, None)
            propositions.append({
                "coords": coords,
                "anchor": snap_contact,
                "inward": inward,
                "cost": err,
            })
    return propositions


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
    _ = base_shape  # reserved for future focal-aware seeding
    propositions = _sheet_exterior_snap_propositions(
        shape_to_place,
        sheet,
        min_dist=min_dist,
        num_angles=num_angles,
        samples_per_edge=samples_per_edge,
        propose_geom=propose_geom,
    )

    # Snap-then-batch: one full-guidance filter for board-edge snaps.
    if propositions and pt_push is not None:
        raw = [p["coords"] for p in propositions]
        valid = set(filter_candidates_batch(propose_geom, raw, pt_push))
        propositions = [p for p in propositions if p["coords"] in valid]

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

    corner_coords = _sheet_corner_seeds(
        shape_to_place,
        sheet,
        min_dist,
        propose_geom=propose_geom,
        pt_push=pt_push,
        num_angles=max(num_angles * 2, 16),
        top_n=top_n,
    )
    for coords in corner_coords:
        placed_geom = propose_geom.placed_at(coords)
        md = placed_geom.standoff_min_distance(propose_geom.boundary_ring_geom)
        anchor_pt = Point(md.closest_b[0], md.closest_b[1])
        anchor_pt, inward = inward_at_contact(sheet, anchor_pt)
        err = placement_contact_error(placed_geom, sheet, min_dist, None)
        _add_seed(coords, anchor_pt, inward, err)

    selected = select_stratified_by_segment(sheet, propositions, top_n)
    return [(p["coords"], p["anchor"], p["inward"]) for p in selected]


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
    # Snap path ignores refine (hybrid is placements_guidance.propose_placements_board_edge).
    _ = refine

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
    # Snap-only SoT. Hybrid refine is placements_guidance.propose_placements_board_edge
    # so this module never imports guidance (no cycle).
    return snap_coords[:top_n]


def propose_placements_side_pack(
    shape_to_place: Polygon,
    sheet: Polygon,
    *,
    min_dist: float,
    propose_cfg: ProposeConfig,
    propose_geom: ProposeGeometry,
    top_n: int = 16,
    crowd_ref: Point | None = None,
    samples_per_edge: int | None = None,
    num_angles: int | None = None,
) -> List[Tuple[float, float, float]]:
    """Sheet-exterior snap with anti-crowd segment weights; emit raw coords (no filter).

    ``crowd_ref`` None → length-only stratify (rim spread). With ``crowd_ref`` (packed
    bbox center), far sheet sides get more slots. Central collect owns clearance.
    """
    if sheet is None or sheet.is_empty or not bool(getattr(propose_cfg, "use_side_pack", True)):
        return []
    n_angles = num_angles if num_angles is not None else propose_cfg.placement_num_angles
    samples = (
        samples_per_edge
        if samples_per_edge is not None
        else int(getattr(propose_cfg, "side_pack_samples_per_edge", 0) or 0)
        or propose_cfg.board_edge_samples_per_edge
    )
    cap = max(int(top_n), 1)
    # Over-emit so packing SoT at collect end can still leave far-side survivors.
    emit_cap = max(cap * 2, cap)
    weights = None
    far_segs: set[int] | None = None
    if crowd_ref is not None and not getattr(crowd_ref, "is_empty", True):
        weights = anti_crowd_segment_weights(sheet, crowd_ref)
        if weights:
            far_segs = anti_crowd_far_segments(weights)

    propositions = _sheet_exterior_snap_propositions(
        shape_to_place,
        sheet,
        min_dist=min_dist,
        num_angles=n_angles,
        samples_per_edge=samples,
        propose_geom=propose_geom,
        segment_filter=far_segs,
    )
    if not propositions:
        return []
    selected = select_stratified_by_segment_weighted(
        sheet,
        propositions,
        emit_cap,
        segment_weights=weights,
    )
    return [p["coords"] for p in selected][:emit_cap]


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
    propose_geom: ProposeGeometry,
    pt_push: Point,
) -> List[Tuple[float, float, float]]:
    """Snap the part along the nearest packed-group exterior at standoff min_dist."""
    if focal_shape is None or focal_shape.is_empty:
        return []

    focal_geom = (
        focal_shape
        if isinstance(focal_shape, Geometry)
        else Geometry.from_shapely(focal_shape)
    )

    propositions: list[dict] = []
    anchor_pts = exterior_anchor_points(focal_shape, samples_per_edge)
    stratify_boundary = focal_shape if isinstance(focal_shape, Polygon) else sheet
    focal_ring_geom = outline_ring_geom(focal_shape)
    base_obs = propose_geom.obstacle_geoms_for_batch()

    for contact in anchor_pts:
        snap_contact, inward = inward_at_contact(focal_shape, contact)
        # focal_shape is a solid obstacle: place the part against its outside.
        inward = (-inward[0], -inward[1])
        for angle in angles_for_edge_contact(focal_shape, contact, num_angles):
            coords = snap_coords_along_exterior(
                shape_to_place,
                focal_shape,
                snap_contact,
                inward,
                angle,
                min_dist,
                container=sheet,
                propose_geom=propose_geom,
                boundary_ring_geom=focal_ring_geom,
            )
            if coords is None:
                continue
            placed_geom = propose_geom.placed_at(coords)
            if not is_pose_clear(
                placed_geom, propose_geom.board_geom, base_obs, min_dist,
            ):
                continue
            err = placement_contact_error(placed_geom, sheet, min_dist, focal_geom)
            propositions.append({
                "coords": coords,
                "anchor": snap_contact,
                "cost": err,
            })

    if propositions:
        raw = [p["coords"] for p in propositions]
        valid = set(filter_candidates_batch(propose_geom, raw, pt_push))
        propositions = [p for p in propositions if p["coords"] in valid]

    return finalize_edge_propositions(propositions, stratify_boundary, top_n)


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


def _sheet_corner_seeds(
    shape_to_place: Polygon,
    sheet: Polygon,
    min_dist: float,
    *,
    propose_geom: ProposeGeometry,
    pt_push: Point,
    num_angles: int = 24,
    top_n: int = 16,
) -> List[Tuple[float, float, float]]:
    """Safe-corner docking seeds (shared by sheet_corners + board_edge)."""
    propositions: list[dict] = []
    angles = np.linspace(0, 2 * np.pi, num_angles, endpoint=False)

    if sheet.is_empty:
        return []

    safe_corners = []
    for ring in get_shape_exteriors(sheet):
        safe_corners.extend(list(ring.coords)[:-1])

    ring_geom = propose_geom.boundary_ring_geom
    for angle in angles:
        rotated = propose_geom.part.rotate(float(angle))
        minx, miny, maxx, maxy = rotated.bounds()

        for cx, cy in safe_corners:
            alignments = [
                (cx - minx, cy - miny),
                (cx - maxx, cy - miny),
                (cx - minx, cy - maxy),
                (cx - maxx, cy - maxy),
            ]

            for dx, dy in alignments:
                placed = rotated.translate(dx, dy)
                if not placed.fully_inside(propose_geom.board_geom):
                    continue
                border_dist = placed.standoff_distance(ring_geom)
                if border_dist < min_dist - 1e-6:
                    continue

                coords = (dx, dy, float(angle))
                propositions.append({
                    "coords": coords,
                    "cost": border_dist,
                })

    if propositions:
        raw = [p["coords"] for p in propositions]
        valid = set(filter_candidates_batch(propose_geom, raw, pt_push))
        propositions = [p for p in propositions if p["coords"] in valid]

    propositions.sort(key=lambda x: x["cost"])

    seen: set[tuple[float, float, float]] = set()
    out: list[tuple[float, float, float]] = []
    for p in propositions:
        key = (round(p["coords"][0], 4), round(p["coords"][1], 4), round(p["coords"][2], 4))
        if key not in seen:
            seen.add(key)
            out.append(p["coords"])
            if len(out) >= top_n:
                break
    return out


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
    """Perfect corner docking using bounding box alignment to the sheet ring."""
    return _sheet_corner_seeds(
        shape_to_place,
        sheet,
        min_dist,
        propose_geom=propose_geom,
        pt_push=pt_push,
        num_angles=num_angles,
        top_n=top_n,
    )


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
    """Slide the part along the exact perimeter of the sheet ring."""
    propositions: list[dict] = []

    if sheet.is_empty:
        return []

    halo_pts = exterior_anchor_points(sheet, samples_per_edge)

    for h_pt in halo_pts:
        for angle in angles_for_edge_contact(sheet, h_pt, num_angles):
            rotated = propose_geom.part.rotate(float(angle))
            minx, miny, maxx, maxy = rotated.bounds()
            dx_center = (maxx + minx) / 2.0
            dy_center = (maxy + miny) / 2.0
            alignments = [
                (h_pt.x - minx, h_pt.y - dy_center),
                (h_pt.x - maxx, h_pt.y - dy_center),
                (h_pt.x - dx_center, h_pt.y - miny),
                (h_pt.x - dx_center, h_pt.y - maxy),
            ]
            for dx, dy in alignments:
                placed = rotated.translate(dx, dy)
                if not placed.fully_inside(propose_geom.board_geom):
                    continue
                coords = (dx, dy, float(angle))
                err = placement_contact_error(placed, sheet, min_dist, None)
                propositions.append({"coords": coords, "anchor": h_pt, "cost": err})

    if propositions and pt_push is not None:
        raw = [p["coords"] for p in propositions]
        valid = set(filter_candidates_batch(propose_geom, raw, pt_push))
        propositions = [p for p in propositions if p["coords"] in valid]

    return finalize_edge_propositions(propositions, sheet, top_n)


def propose_placements_ribbon_free(
    base_shape: BaseGeometry,
    shape_to_place: Polygon,
    sheet: Polygon,
    min_dist: float,
    *,
    num_angles: int = 8,
    top_n: int = 8,
    propose_geom: ProposeGeometry,
    pt_push: Point,
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
    base_cx, base_cy = float(base_centroid.x), float(base_centroid.y)
    raw: list[tuple[float, float, float]] = []
    costs: list[float] = []
    angles = np.linspace(0, 2 * np.pi, num_angles, endpoint=False)
    for angle in angles:
        for pt in seeds:
            coords = (float(pt.x), float(pt.y), float(angle))
            raw.append(coords)
            costs.append(math.hypot(pt.x - base_cx, pt.y - base_cy))

    if not raw:
        return []
    order = sorted(range(len(raw)), key=lambda i: costs[i])
    take = min(len(order), max(top_n * 3, top_n))
    ordered = [raw[i] for i in order[:take]]
    cost_map = {raw[i]: costs[i] for i in order[:take]}
    valid = filter_candidates_batch(propose_geom, ordered, pt_push)
    valid.sort(key=lambda c: cost_map.get(c, 0.0))
    return valid[:top_n]
