import random
import numpy as np
from shapely import LineString, MultiPoint, MultiPolygon, Point
from shapely.ops import polylabel, unary_union, voronoi_diagram, nearest_points
from shapely.affinity import translate, rotate

from .utils import get_shape_exteriors, get_shape_polygons_coords, transform_poly


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


def propose_placements_nudge(
    base_shape, shape_to_place, boundary,
    direction_vector=(1.0, 0.0),
    min_dist=1.0, num_angles=8, max_nudges=25, top_n=3,
    weight_dist=0.3, weight_dir=0.4, weight_hull=0.3
):
    propositions = []
    base_centroid = base_shape.centroid if not base_shape.is_empty else boundary.centroid
    base_hull_area = base_shape.convex_hull.area

    # Normalize direction vector for dot product
    dir_v = np.array(direction_vector, dtype=float)
    dir_v = dir_v / np.linalg.norm(dir_v) if np.linalg.norm(dir_v) > 0 else dir_v

    # Adaptive Sampling (as before)
    minx, miny, maxx, maxy = shape_to_place.bounds
    sample_step = max(maxx - minx, maxy - miny) * 0.5

    # Calculate min/max radius of the shape_to_place from its centroid
    shape_to_place_center = shape_to_place.centroid
    r_min = shape_to_place.exterior.distance(shape_to_place_center)
    r_max = max([shape_to_place_center.distance(Point(p)) for p in shape_to_place.exterior.coords])

    # Define Ribbon Search Zone
    if not base_shape.is_empty:
        outer_ribbon = base_shape.buffer(r_max + min_dist)
        inner_ribbon = base_shape.buffer(r_min + min_dist)
    else:
        outer_ribbon = boundary.exterior.buffer(r_max + min_dist)
        inner_ribbon = boundary.exterior.buffer(r_min + min_dist)
    search_zone = (
        outer_ribbon.difference(inner_ribbon) if not outer_ribbon.is_empty else outer_ribbon
    ).intersection(boundary)

    samples = []
    for line in get_shape_exteriors(search_zone):
        num_samples = max(8, int(line.length / sample_step))
        for d in np.linspace(0, line.length, num_samples):
            samples.append(line.interpolate(d))
        for p in line.coords:
            samples.append(Point(p))

    angles = np.linspace(0, 2*np.pi, num_angles, endpoint=False)

    for start_pt in samples:
        for start_angle in angles:
            curr_x, curr_y = start_pt.x, start_pt.y
            curr_angle = start_angle

            best_local_score = float('inf')
            best_local_coords = None

            for i in range(max_nudges + 1): # +1 to evaluate the initial position
                learning_rate = 1.0 - (i / max_nudges)
                placed_shape = transform_poly(shape_to_place, (curr_x, curr_y, curr_angle))

                # --- VALIDATION GATE ---
                is_colliding = base_shape.intersects(placed_shape) if not base_shape.is_empty else False
                is_inside = boundary.contains(placed_shape)
                curr_dist = base_shape.distance(placed_shape) if not base_shape.is_empty else float('inf')

                # A placement is valid if it's inside, not colliding, and obeys min_dist
                is_valid = is_inside and (not is_colliding) and (curr_dist >= min_dist)

                # --- SCORE TRACKING ---
                if is_valid:
                    score = calculate_complex_score(
                        base_shape, placed_shape, base_hull_area,
                        base_centroid, dir_v, weight_dist, weight_dir, weight_hull
                    )
                    if score < best_local_score:
                        best_local_score = score
                        best_local_coords = (curr_x, curr_y, curr_angle)

                # --- FORCE CALCULATION ---
                vx, vy, v_torque = 0.0, 0.0, 0.0

                # A. Gravity (The Optimization Drive)
                vx += dir_v[0] * 0.8
                vy += dir_v[1] * 0.8

                # B. Stochastic Forces (Jitter & Shake)
                if i % 6 == 0: # Shake
                    vx += random.uniform(-1.5, 1.5)
                    vy += random.uniform(-1.5, 1.5)
                vx += random.uniform(-0.1, 0.1) # Constant Jitter
                vy += random.uniform(-0.1, 0.1)

                # C. Constraints (Repulsion)
                if not is_inside:
                    p_placed, p_bound = nearest_points(placed_shape, boundary.exterior)
                    vx += (p_bound.x - p_placed.x) * 3.0 # Strong pull back in
                    vy += (p_bound.y - p_placed.y) * 3.0

                elif (is_colliding or curr_dist < min_dist) and not base_shape.is_empty:
                    if is_colliding:
                        inter = base_shape.intersection(placed_shape)
                        # Filter intersection to ignore 0-area artifacts for repulsion anchor
                        if inter.area < 1e-7 and not inter.is_empty:
                            _, p_on_base = nearest_points(Point(curr_x, curr_y), inter)
                        else:
                            p_on_base = inter.centroid
                        p_on_base, p_on_placed = nearest_points(p_on_base, placed_shape.exterior)
                    else:
                        p_on_base, p_on_placed = nearest_points(base_shape, placed_shape)

                    # Push away vector: p_on_placed - p_on_base
                    push_x, push_y = p_on_placed.x - p_on_base.x, p_on_placed.y - p_on_base.y
                    mag = np.sqrt(push_x**2 + push_y**2)
                    if mag > 0:
                        # Repulsion scales to overcome gravity and jitter
                        vx += (push_x / mag) * 2.0
                        vy += (push_y / mag) * 2.0

                    # Torque for rotation optimization
                    rx, ry = p_on_placed.x - curr_x, p_on_placed.y - curr_y
                    v_torque = np.sign((rx * push_y) - (ry * push_x))

                # --- APPLY PHYSICS ---
                curr_x += vx * learning_rate
                curr_y += vy * learning_rate
                curr_angle += v_torque * (5.0/180.0 * np.pi) * learning_rate

            if best_local_coords:
                propositions.append({'coords': best_local_coords, 'cost': best_local_score})

    return finalize_propositions(propositions, top_n)


def calculate_complex_score(base, placed, base_hull_area, centroid, dir_v, w_dist, w_dir, w_hull):
    # 1. Distance Component
    dist_to_center = Point(placed.centroid).distance(centroid)

    # 2. Directional Component (Dot Product)
    # We negate this because we want to MINIMIZE score as we move ALONG the vector
    pos = np.array([placed.centroid.x, placed.centroid.y])
    direction_score = -np.dot(pos, dir_v)

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
