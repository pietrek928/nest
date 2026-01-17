import random
from typing import Tuple
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


def propose_placements_point_cloud(
    base_shape, shape_to_place, boundary, pt_push,
    num_particles=64, max_iterations=128, min_dist=1.0, top_n=3,
    omega=0.6, c1=1.2, c2=1.4, nudge_iters=8, pull_factor=0.08,
    cull_ratio=0.2, stagnation_limit=5,
    mutation_sigma=np.array([2.0, 2.0, 0.5, 0.5]) # [x, y, theta, phi]
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
                base_hull_area, bound_centroid, pt_push, min_dist, nudge_iters, pull_factor
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
    bound_centroid, pt_push, min_dist, nudge_iters, pull_factor
):
    curr_x, curr_y, curr_theta, _ = params

    # Torque momentum
    m_theta = 0.0
    beta_theta = 0.9

    for i in range(1, nudge_iters + 1):
        decay = 1.0 - (i / nudge_iters)
        placed_shape = transform_poly(shape_to_place, (curr_x, curr_y, curr_theta))

        # 1. ANALYZE ENVIRONMENT
        is_inside = boundary.contains(placed_shape)
        is_colliding = base_shape.intersects(placed_shape) if not base_shape.is_empty else False

        # Pull vector away from the push point
        push_v = np.array([curr_x - pt_push.x, curr_y - pt_push.y])
        push_v_u = push_v / (np.linalg.norm(push_v) + 1e-6)

        # 2. CALCULATE ATTRACTION (The 'Tightness' Hint)
        if not base_shape.is_empty:
            p_base, p_shape = nearest_points(base_shape, placed_shape)
            # Vector pointing exactly from us to the base
            attract_v = np.array([p_base.x - p_shape.x, p_base.y - p_shape.y])
            attract_v_u = attract_v / (np.linalg.norm(attract_v) + 1e-6)

            # TORQUE: Rotate to align the nearest points (helps 'roll' into gaps)
            # Cross product to find rotation direction toward the attraction vector
            lever_arm = np.array([p_shape.x - placed_shape.centroid.x, p_shape.y - placed_shape.centroid.y])
            torque = (lever_arm[0] * attract_v_u[1] - lever_arm[1] * attract_v_u[0])
            m_theta = beta_theta * m_theta + (1 - beta_theta) * torque
            curr_theta += m_theta * 0.3 * decay
        else:
            attract_v_u = np.array([0.0, 0.0])

        # 3. DEFINE THE RAY GOAL
        if not is_inside or is_colliding:
            # EMERGENCY: Pull toward center to recover
            to_center = np.array([bound_centroid.x - curr_x, bound_centroid.y - curr_y])
            active_dir = to_center / (np.linalg.norm(to_center) + 1e-6)
        else:
            # TIGHT PACKING: Squeeze the shape between the push-point and the base-cluster
            # We favor the attraction to the base (0.7) over the push (0.3)
            active_dir = (attract_v_u * 0.7 + push_v_u * 0.3)

        # 4. RAY CASTING (Choosing the best movement)
        best_ray_v = np.array([0.0, 0.0])
        max_int = -float('inf')

        # Higher ray count (16) to find narrow gaps for tight packing
        for r_angle in np.linspace(0, 2*np.pi, 16, endpoint=False):
            rv = np.array([np.cos(r_angle), np.sin(r_angle)])

            # Calculate intensity based on alignment with our 'Tightness' goal
            intensity = np.dot(rv, active_dir)

            # Bonus: rays that point directly toward the base get a boost
            if not base_shape.is_empty:
                intensity += np.dot(rv, attract_v_u) * 0.5

            if intensity > max_int:
                max_int = intensity
                best_ray_v = rv

        # Apply displacement
        curr_x += best_ray_v[0] * pull_factor * 5.0 * decay
        curr_y += best_ray_v[1] * pull_factor * 5.0 * decay

    # 5. FINAL EVALUATION (Using your old score function)
    final_shape = transform_poly(shape_to_place, (curr_x, curr_y, curr_theta))

    # Final validity check
    dist_to_base = base_shape.distance(final_shape) if not base_shape.is_empty else 10.0
    if not (boundary.contains(final_shape) and (dist_to_base >= min_dist) and (not base_shape.intersects(final_shape) or base_shape.is_empty)):
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
