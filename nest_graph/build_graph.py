# CFLAGS="-Wno-error=incompatible-pointer-types" pip install --force-reinstall --no-binary=shapely --upgrade shapely
# pip install --force-reinstall --no-binary=rtree --upgrade rtree

import numpy as np
import cv2 as cv
from pydantic import BaseModel, ConfigDict
from shapely import LineString, MultiPoint, MultiPolygon, Polygon
from shapely.ops import polylabel, unary_union, voronoi_diagram
from shapely.geometry.base import BaseGeometry
from shapely.affinity import translate, rotate
from rtree.index import Index
from tqdm import tqdm
from typing import Tuple

from .track_perf import show_performance
from .elem_graph import (
    ElemGraph, BBox, Point,
    PointPlaceRule, BBoxPlaceRule, PointAngleRule, PlacementRuleSet,
    RuleMutationSettings,
    nest_by_graph, sort_graph, score_elems, augment_rules, score_rules,
    increase_selection_dfs, increase_score_dfs
)

# Track performance
nest_by_graph = show_performance(nest_by_graph)
sort_graph = show_performance(sort_graph)
score_elems = show_performance(score_elems)
# augment_rules = show_performance(augment_rules)
score_rules = show_performance(score_rules)


def propose_placements_erosion(
    base_shape, shape_to_place,
    min_dist=2.0, num_angles=8, top_n=3,
    weight_dist=0.5
):
    """
    Finds x, y, angle with a forced minimum distance from boundaries.
    """
    propositions = []

    # Pre-calculate base properties
    base_centroid = base_shape.centroid
    base_hull_area = base_shape.convex_hull.area

    angles = np.linspace(0, 360, num_angles, endpoint=False)

    for angle in angles:
        rotated_shape = rotate(shape_to_place, angle, origin=(0, 0))

        # 2. Calculate the 'Collision Radius' + min_dist
        # We use the distance from the centroid to the furthest vertex
        # plus the required clearance.
        bounds = rotated_shape.bounds
        max_dim = max(bounds[2] - bounds[0], bounds[3] - bounds[1]) / 2
        total_buffer = max_dim + min_dist

        # 3. Generate Safe Zone (Erosion)
        # This automatically respects holes because buffer handles interior rings.
        safe_zone = base_shape.buffer(-total_buffer)

        if safe_zone.is_empty:
            continue

        # 4. Extract Candidate Points
        # We check the 'deepest' points in the safe zone (furthest from walls)
        candidate_points = []
        if isinstance(safe_zone, MultiPolygon):
            for poly in safe_zone.geoms:
                if not poly.is_empty:
                    candidate_points.append(polylabel(poly, tolerance=0.5))
        else:
            candidate_points.append(polylabel(safe_zone, tolerance=0.5))

        for pt in candidate_points:
            # Place the actual shape at this candidate center
            placed_shape = translate(rotated_shape, pt.x, pt.y)

            # 5. Precise Distance Validation
            # Even if the buffer is a heuristic, we double check the min_dist
            actual_dist = base_shape.exterior.distance(placed_shape)
            if actual_dist < (min_dist - 0.001): # Small float tolerance
                continue

            # 6. Scoring Logic
            # Centroid proximity
            dist_to_center = pt.distance(base_centroid)

            # Convex Hull growth (tightness)
            combined_hull = unary_union([base_shape, placed_shape]).convex_hull
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


def densify_points(geometry, distance):
    """Adds points along the perimeter of the shape for a better Voronoi map."""
    if geometry.geom_type == 'Polygon':
        lines = [geometry.exterior] + list(geometry.interiors)
    else:
        lines = [geometry]

    points = []
    for line in lines:
        for d in np.arange(0, line.length, distance):
            points.append(line.interpolate(d))
        points.append(line.interpolate(line.length))
    return MultiPoint(points)


def propose_placements_voronoi(
    base_shape, shape_to_place,
    min_dist=2.0, num_angles=8, top_n=3,
    weight_dist=0.5
):
    """
    Proposes placements using Voronoi vertices as candidate centers.
    """
    propositions = []
    base_centroid = base_shape.centroid
    base_hull_area = base_shape.convex_hull.area

    # 1. Densify the base shape and generate Voronoi Diagram
    # We use a distance-based densification (1/20th of the bounding box size)
    extent = max(base_shape.bounds[2]-base_shape.bounds[0], base_shape.bounds[3]-base_shape.bounds[1])
    points = densify_points(base_shape, extent / 20.0)

    # Generate the Voronoi regions
    vor_regions = voronoi_diagram(points)

    # 2. Extract vertices from Voronoi regions that are INSIDE the base shape
    # These vertices represent the "Medial Axis" or skeleton.
    candidate_points = []
    for region in vor_regions.geoms:
        for vert in region.exterior.coords:
            p = Point(vert)
            if base_shape.contains(p):
                candidate_points.append(p)

    # 3. Normalize the shape to place
    orig_centroid = shape_to_place.centroid
    centered_shape = translate(shape_to_place, -orig_centroid.x, -orig_centroid.y)

    angles = np.linspace(0, 2*np.pi, num_angles, endpoint=False)

    # 4. Evaluate candidates
    for pt in candidate_points:
        for angle in angles:
            rotated_shape = rotate(centered_shape, angle, origin=(0, 0), use_radians=True)
            placed_shape = translate(rotated_shape, pt.x, pt.y)

            # Check if it fits inside with required clearance
            # A negative buffer on the base is the fastest way to check clearance
            if not base_shape.buffer(-min_dist).contains(placed_shape):
                continue

            # 5. Scoring
            dist_val = pt.distance(base_centroid)
            combined_hull = unary_union([base_shape, placed_shape]).convex_hull
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
    min_dist=2.0, num_rays=12, num_angles=8, top_n=3,
    weight_dist=0.5
):
    """
    Proposes placements by casting rays from boundary vertices into the interior.
    """
    propositions = []
    base_centroid = base_shape.centroid
    base_hull_area = base_shape.convex_hull.area

    # Pre-calculate a 'safe' container (the base minus a tiny buffer for clearance)
    safe_base = base_shape.buffer(-min_dist)
    if safe_base.is_empty:
        return []

    # 1. Identify Anchor Points (vertices of the base and holes)
    # We use these as origins for our rays.
    anchors = []
    for line in [base_shape.exterior] + list(base_shape.interiors):
        anchors.extend([Point(pt) for pt in line.coords])

    # Calculate how long the rays should be (diagonal of the base)
    min_x, min_y, max_x, max_y = base_shape.bounds
    ray_len = np.sqrt((max_x - min_x)**2 + (max_y - min_y)**2)

    # 3. Cast Rays and Find Candidates
    ray_angles = np.linspace(0, 2*np.pi, num_rays, endpoint=False)
    placement_angles = np.linspace(0, 2*np.pi, num_angles, endpoint=False)

    for anchor in anchors[::2]:  # Step by 2 to keep it fast
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
                        # Scoring
                        dist_val = pt.distance(base_centroid)

                        # Fast Hull Growth Calculation
                        # Using points instead of unary_union for speed
                        all_pts = list(base_shape.convex_hull.exterior.coords) + \
                                  list(placed_shape.exterior.coords)
                        hull_growth = np.sqrt(max(0, MultiPoint(all_pts).convex_hull.area - base_hull_area))

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


def make_placement_base(base_shape, polys, exclude_p, exclude_dist=0):
    out_polys = [base_shape.exterior]
    for p in polys:
        if exclude_dist:
            if p.exterior.distance(exclude_p) > exclude_dist:
                out_polys.append(p)
        else:
            if not p.intersects(exclude_p):
                out_polys.append(p)
    return unary_union(out_polys)


class PolygonGroup(BaseModel):
    model_config = ConfigDict(arbitrary_types_allowed=True)

    polygon: Polygon
    weight: float
    transforms: np.ndarray


def transform_poly(p: Polygon, transform_data: Tuple[float, float, float]):
    x, y, angle = transform_data[:3]
    return rotate(translate(p, x, y), angle, origin='center', use_radians=True)


def normalize_poly(p: BaseGeometry):
    c = p.centroid
    return translate(p, -c.x, -c.y)


# TODO: make faster polygon operations, shapely lags
def polygon_board_distance(b: BaseGeometry, p: Polygon):
    if b.contains(p):
        return b.exterior.distance(p) + b.exterior.distance(p.centroid)
    if p.intersects(b):
        return 0
    return -b.distance(p)


def select_non_intersecting_polygons(polygons: np.ndarray):
    idx = Index()
    selected = []
    for p in polygons:
        intersects = False
        for i in idx.intersection(p.bounds):
            if p.intersects(selected[i]):
                intersects = True
                break
        if not intersects:
            idx.insert(len(selected), p.bounds)
            selected.append(p)
    return selected


def select_polygons_from_edges(b: BaseGeometry, polygons: Tuple[Tuple[Polygon, float, np.ndarray], ...]):
    result = [[] for _ in range(len(polygons))]
    polys_transformed = []
    for i, (p, w, transforms) in enumerate(polygons):
        for t in transforms:
            poly_t = transform_poly(p, t)
            d = polygon_board_distance(b, poly_t)
            if d > 0:
                d += np.random.rand() * 1e-4
                polys_transformed.append((poly_t.centroid.coords[0][0] + w, i, t, poly_t))
    polys_transformed = sorted(polys_transformed, key=lambda x: x[0])

    idx = Index()
    selected = []
    for _, pnum, t, poly_t in polys_transformed:
        intersects = False
        for i in idx.intersection(poly_t.bounds):
            if poly_t.intersects(selected[i]):
                intersects = True
                break
        if not intersects:
            idx.insert(len(selected), poly_t.bounds)
            selected.append(poly_t)
            result[pnum].append(t)
    return tuple(
        np.array(tt) for tt in result
    )


def make_polygon_matrix(b: BaseGeometry, polygons: Tuple[Tuple[Polygon, float, np.ndarray], ...]):
    idx = Index()
    selected = []
    group_weights = []

    for i, (p, w, transforms) in enumerate(polygons):
        n = len(transforms)
        for t in transforms:
            poly_t = transform_poly(p, t)
            if b.contains(poly_t):
                selected.append(poly_t)
                # group_weights.append(w / n)
                group_weights.append(w)

    M = np.zeros((len(selected), len(selected)), dtype=np.float32)
    for i, poly in enumerate(tqdm(selected)):
        for j in idx.intersection(poly.bounds):
            if poly.intersects(selected[j]):
                M[i, j] = M[j, i] = 1
        idx.insert(i, poly.bounds)
        M[i, i] = -group_weights[i]

    return M, selected


@show_performance
def make_polygon_graph(b: BaseGeometry, polygons: Tuple[Tuple[Polygon, float, np.ndarray], ...]):
    idx = Index()
    selected_polys = []
    selected_group_id = []
    selected_transform = []
    graph = ElemGraph()

    for i, (p, transforms) in enumerate(polygons):
        # n = len(transforms)
        for t in transforms:
            poly_t = transform_poly(p, t)
            if b.contains(poly_t):
                # group_weights.append(w / n)
                center = poly_t.centroid.coords[0]
                bbox = poly_t.bounds
                graph.append_elem(
                    i,
                    Point(x=center[0], y=center[1]),
                    BBox(xstart=bbox[0], ystart=bbox[1], xend=bbox[2], yend=bbox[3])
                )

                n = len(selected_polys)
                for j in idx.intersection(bbox):
                    if poly_t.intersects(selected_polys[j]):
                        graph.add_collision(n, j)
                selected_polys.append(poly_t)
                selected_group_id.append(i)
                selected_transform.append(t)
                idx.insert(n, poly_t.bounds)

    return graph, selected_polys, selected_group_id, selected_transform


def optimize_polygons(M: np.ndarray, v: np.ndarray):
    score = 1e9
    for it in range(1280):
        v += np.random.rand(M.shape[0]) * 1e-5
        v -= 1.5e-2 * (M @ v)
        v = np.clip(v, 0, 1)
        score = v @ M @ v
        print(f'iter {it}, score {score}, mean {v.mean()}, max {v.max()}')
    return v


def score_transforms(b: BaseGeometry, p: Polygon, transforms: np.ndarray):
    scores = np.zeros((transforms.shape[0], ))
    for i, t in enumerate(transforms):
        p_t = transform_poly(p, t)
        scores[i] = polygon_board_distance(b, p_t)
    return scores


# random transforms in range
def transforms_around(p: np.ndarray, s: Tuple[float, float, float], n: int):
    sx, sy, sa = s
    return np.concatenate([
        p + np.random.uniform(-1, 1, (p.shape[0], 3)) * [sx, sy, sa]
        for _ in range(n)
    ])


def scale_coords(
    coords: np.ndarray,
    xstart: float, ystart: float,
    xscale: float, yscale: float
) -> np.ndarray:
    x, y = coords.T
    x = (x - xstart) * xscale
    y = (y - ystart) * yscale
    return np.stack([x, y], axis=-1).astype(np.int32)


def render_placement(b: BaseGeometry, elems: Tuple[Tuple[Polygon, np.ndarray], ...], im_shape=(1024, 1024)):
    xstart, ystart, xend, yend = b.bounds
    xscale = im_shape[0] / (xend - xstart)
    yscale = im_shape[1] / (yend - ystart)
    im = np.zeros((im_shape[0], im_shape[1], 3), dtype=np.uint8)
    cv.drawContours(im, [scale_coords(
        np.array(b.exterior.coords), xstart, ystart, xscale, yscale
    )], -1, (255, 255, 255), 3)
    for p, transforms in elems:
        for t in transforms:
            p_t = transform_poly(p, t)
            cv.drawContours(im, [scale_coords(
                np.array(p_t.exterior.coords), xstart, ystart, xscale, yscale
            )], -1, (100, 100, 100), cv.FILLED)
            cv.drawContours(im, [scale_coords(
                np.array(p_t.exterior.coords), xstart, ystart, xscale, yscale
            )], -1, (255, 255, 255), 3)
    return im


def render_selection(b: BaseGeometry, polys: Tuple[Polygon, ...], v: np.ndarray, im_shape=(1024, 1024)):
    xstart, ystart, xend, yend = b.bounds
    xscale = im_shape[0] / (xend - xstart)
    yscale = im_shape[1] / (yend - ystart)
    im = np.zeros((im_shape[0], im_shape[1], 3), dtype=np.uint8)
    cv.drawContours(im, [scale_coords(
        np.array(b.exterior.coords), xstart, ystart, xscale, yscale
    )], -1, (255, 255, 255), 3)
    for w, p in sorted(zip(v, polys), key=lambda x: x[0]):
        cv.drawContours(im, [scale_coords(
            np.array(p.exterior.coords), xstart, ystart, xscale, yscale
        )], -1, (100*w, 100*w, 100*w), cv.FILLED)
        cv.drawContours(im, [scale_coords(
            np.array(p.exterior.coords), xstart, ystart, xscale, yscale
        )], -1, (255*w, 255*w, 255*w), 3)
    return im


def render_polys(b: BaseGeometry, polys: Tuple[Tuple[Polygon, ...], ...], im_shape=(1024, 1024)):
    xstart, ystart, xend, yend = b.bounds
    xscale = im_shape[0] / (xend - xstart)
    yscale = im_shape[1] / (yend - ystart)
    im = np.zeros((im_shape[0], im_shape[1], 3), dtype=np.uint8)
    cv.drawContours(im, [scale_coords(
        np.array(b.exterior.coords), xstart, ystart, xscale, yscale
    )], -1, (255, 255, 255), 3)
    for poly_set in polys:
        for p in poly_set:
            cv.drawContours(im, [scale_coords(
                np.array(p.exterior.coords), xstart, ystart, xscale, yscale
            )], -1, (100, 100, 100), cv.FILLED)
            cv.drawContours(im, [scale_coords(
                np.array(p.exterior.coords), xstart, ystart, xscale, yscale
            )], -1, (255, 255, 255), 3)
    return im


# @show_performance
def improve_rules(graphs, rules, n):
    new_rules = list(rules)
    new_rules.extend(augment_rules(rules, RuleMutationSettings(
        box=BBox(xstart=0, ystart=0, xend=1.2, yend=1.1),
        dpos=.25,
        dw=.25,
        da=np.pi/4,
        insert_p=0.09,
        remove_p=0.02,
        mutate_p=0.1,
        ngroups=2
    )))
    new_rules.extend(augment_rules(rules, RuleMutationSettings(
        box=BBox(xstart=0, ystart=0, xend=1.2, yend=1.1),
        dpos=.05,
        dw=.05,
        da=np.pi/32,
        insert_p=0.04,
        remove_p=0.01,
        mutate_p=0.1,
        ngroups=2
    )))
    new_rules.extend(augment_rules(rules, RuleMutationSettings(
        box=BBox(xstart=0, ystart=0, xend=1.2, yend=1.1),
        dpos=.01,
        dw=.01,
        da=np.pi/64,
        insert_p=0.01,
        remove_p=0.001,
        mutate_p=0.1,
        ngroups=2
    )))
    scores = score_rules(graphs, new_rules)
    scored = []
    for s, r in zip(scores, new_rules):
        scored.append((s - r.size() * .01, r))
    scored = sorted(scored, key=lambda x: x[0], reverse=True)
    return [
        v[1] for v in scored[:n]
    ]


def transform_selection(s, n):
    yield transforms_around(s, (0.15, 0.15, 2), n)
    yield transforms_around(s, (0.15, 0.15, 0), n)
    yield transforms_around(s, (0, 0, 2), n)
    yield transforms_around(s, (0.05, 0.05, 1), n)
    yield transforms_around(s, (0.05, 0.05, 0), n)
    yield transforms_around(s, (0, 0, 1), n)
    yield transforms_around(s, (0.01, 0.01, 0.01), n)
    yield transforms_around(s, (0.01, 0.01, 0), n)
    yield transforms_around(s, (0, 0, 0.01), n)
    yield transforms_around(s, (0.001, 0.001, 0.001), n)
    yield transforms_around(s, (0.001, 0.001, 0), n)
    yield transforms_around(s, (0, 0, 0.001), n)


def transform_history(h, n):
    yield transforms_around(h, (0.1, 0.1, 0.2), n)
    yield transforms_around(h, (0.1, 0.1, 0), n)
    yield transforms_around(h, (0, 0, 0.2), n)


graphs = []
first_rule_set = PlacementRuleSet()
first_rule_set.append_rule(PointPlaceRule(
    x=0, y=0, r=.1, w=.1, group=0
))
first_rule_set.append_rule(PointPlaceRule(
    x=0, y=0, r=.1, w=.1, group=1
))
rule_sets = [first_rule_set]

p_board = Polygon([(0, 0), (1.2, 0), (0, 1.1)])
p1 = normalize_poly(Polygon([(0, 0), (.1, 0), (.1, .1), (0, .1)]))
p2 = normalize_poly(Polygon([(0, 0), (.15, 0), (0, .07)]))

selected_t = [
    np.random.rand(128, 3) * [1.5, 1.5, 2 * np.pi],
    np.random.rand(128, 3) * [1.5, 1.5, 2 * np.pi],
]

wrect = 1
wtriang = 1
r = .2
rule_set = PlacementRuleSet()
rule_set.append_rule(PointPlaceRule(
    x=0, y=0, r=r, w=wrect, group=0
))
# rule_set.append_point_rule(PointPlaceRule(
#     x=0, y=0, r=r, w=wtriang, group=1
# ))
# rule_set.append_rule(PointPlaceRule(
#     x=1.2, y=0, r=r, w=wrect, group=0
# ))
# rule_set.append_rule(PointPlaceRule(
#     x=1.2, y=0, r=r, w=wtriang, group=1
# ))
# rule_set.append_rule(PointPlaceRule(
#     x=0, y=1.1, r=r, w=wrect, group=0
# ))
# rule_set.append_rule(PointPlaceRule(
#     x=0, y=1.1, r=r, w=wtriang, group=1
# ))
# rule_set.append_rule(PointPlaceRule(
#     x=0.7, y=0.7, r=r, w=wrect, group=0
# ))
rule_set.append_rule(PointPlaceRule(
    x=0.7, y=0.7, r=r, w=wtriang, group=1
))
rule_set.append_rule(PointPlaceRule(
    x=0, y=1.1, r=r, w=wtriang, group=1
))
rule_set.append_rule(PointPlaceRule(
    x=1.2, y=0, r=r, w=wtriang, group=1
))
# rule_set.append_rule(PointAngleRule(
#     x=0.7, y=0.7, r=r, a=np.pi/4, w=.1*wtriang, group=1
# ))
# rule_set.append_rule(PointAngleRule(
#     x=0.7, y=0.7, r=r, a=np.pi*5/4, w=.1*wtriang, group=1
# ))
rule_set.append_rule(PointAngleRule(
    x=0, y=1.1, r=r, a=np.pi/4, w=.1*wtriang, group=1
))
rule_set.append_rule(PointAngleRule(
    x=0, y=1.1, r=r, a=np.pi*5/4, w=.1*wtriang, group=1
))
rule_set.append_rule(PointAngleRule(
    x=1.2, y=0, r=r, a=np.pi/4, w=.1*wtriang, group=1
))
rule_set.append_rule(PointAngleRule(
    x=1.2, y=0, r=r, a=np.pi*5/4, w=.1*wtriang, group=1
))
video = cv.VideoWriter('test.mp4', cv.VideoWriter_fourcc(*'mp4v'), 5, (1024, 1024))

history = [np.zeros((1, 3)), np.zeros((1, 3))]

for it in tqdm(tuple(range(256))):
    s0 = [
        np.random.rand(max(0, 1024-selected_t[0].shape[0]), 3) * [1.5, 1.5, 2 * np.pi],
        history[0]
    ]
    if selected_t[0].shape[0] > 0:
        s0.append(selected_t[0])
        s0.extend(transform_selection(selected_t[0], 8))
        s0.extend(transform_history(history[0], 4))

    s1 = [
        np.random.rand(max(0, 1024-selected_t[1].shape[0]), 3) * [1.5, 1.5, 2 * np.pi],
        history[1]
    ]
    if selected_t[1].shape[0] > 0:
        s1.append(selected_t[1])
        s1.extend(transform_selection(selected_t[1], 8))
        s1.extend(transform_history(history[1], 4))

    selected_t = [
        np.concatenate(s0),
        np.concatenate(s1),
    ]
    graph, polys, group_id, transform = make_polygon_graph(p_board, [(p1, selected_t[0]), (p2, selected_t[1])])
    graphs.append(graph)
    graphs = graphs[-12:]
    for _ in range(8):
        rule_sets = improve_rules(graphs, rule_sets, 64)
    # M, polys = make_polygon_matrix(p_board, [(p1, 20, selected_t[0]), (p2, 12, selected_t[1])])
    # print('------', M.shape, M.sum()/M.shape[0])
    # v = optimize_polygons(M, np.zeros((M.shape[0], )))
    selected_polys = nest_by_graph(graph, rule_sets[:1])[0]
    old_len = len(selected_polys)
    graph_sorted = sort_graph(graph, rule_set)
    graph_sorted_rev = sort_graph(graph, rule_set, reverse=True)
    scores = score_elems(graph, rule_set)
    for _ in range(2):
        selected_polys = increase_selection_dfs(graph_sorted_rev, selected_polys, 8, 2)
        selected_polys = increase_selection_dfs(graph, selected_polys, 8, 1)
        selected_polys = increase_score_dfs(graph_sorted_rev, selected_polys, scores)
        selected_polys = increase_selection_dfs(graph_sorted, selected_polys, 8, 2)
        selected_polys = increase_score_dfs(graph_sorted, selected_polys, scores)
    print(len(polys), old_len, ' -> ', len(selected_polys))
    im = render_polys(p_board, [[
        polys[i] for i in selected_polys
    ]])
    video.write(im)
    cv.imwrite('test.jpg', im)

    selected_t = [[], []]
    for i in selected_polys:
        selected_t[group_id[i]].append(transform[i])
    selected_t = tuple(np.array(t) for t in selected_t)
    if len(history[0]) and len(selected_t[0]):
        history[0] = np.unique(np.concatenate([selected_t[0], history[0]]), axis=0)[5000:, :]
    if len(history[1]) and len(selected_t[1]):
        history[1] = np.unique(np.concatenate([selected_t[1], history[1]]), axis=0)[5000:, :]

# selected_t = select_polygons_from_edges(p_board, [(p1, selected_t[0]), (p2, selected_t[1])])
# video.write(render_placement(p_board, [(p1, selected_t[0]), (p2, selected_t[1])]))
# video.write(render_selection(p_board, polys, v))
# cv.imwrite(f'/tmp/test_{it}.jpg', render_selection(p_board, polys, v))

video.release()

# render
# im = render_placement(p_board, [(p1, selected_t[0]), (p2, selected_t[1])])
# cv.waitKey(0)
