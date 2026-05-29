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

from nest_graph.propose.geometry import ProposeGeometry
from nest_graph.propose.placements_edge import sample_placement_points_ribbon
from nest_graph.propose.ranking import calculate_complex_score, finalize_propositions

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
        recovery_xy = (float(bound_centroid.x), float(bound_centroid.y))
        attract_u = np.zeros(2, dtype=np.float64)
        active_dir = np.zeros(2, dtype=np.float64)
        attract_v_u = np.zeros(2, dtype=np.float64)
        g = None

        if propose_geom is not None:
            placed = propose_geom.placed_at((curr_x, curr_y, curr_theta))
            g = propose_geom.placement_guidance(placed, (curr_x, curr_y), pt_push)
            cx, cy = placed.center()
            lever_arm = np.array([cx - curr_x, cy - curr_y])
            attract_u = propose_geom.attraction_unit(placed, pt_push, (curr_x, curr_y))
            torque = lever_arm[0] * attract_u[1] - lever_arm[1] * attract_u[0]
            m_theta = beta_theta * m_theta + (1 - beta_theta) * torque
            curr_theta += m_theta * 0.3 * decay
            prop = best_proposition(g)
            if prop is not None and abs(float(prop.rotation_rad)) > 1e-6:
                delta = float(prop.rotation_rad) - curr_theta
                while delta > np.pi:
                    delta -= 2 * np.pi
                while delta < -np.pi:
                    delta += 2 * np.pi
                curr_theta += delta * 0.3 * decay
            elif g.is_penetrating:
                alt_props = tiered_propositions(g)
                for alt_prop in alt_props:
                    if abs(float(alt_prop.rotation_rad)) > 1e-6:
                        delta = float(alt_prop.rotation_rad) - curr_theta
                        while delta > np.pi:
                            delta -= 2 * np.pi
                        while delta < -np.pi:
                            delta += 2 * np.pi
                        curr_theta += delta * 0.2 * decay
                        break
        else:
            placed_shape = transform_poly(shape_to_place, (curr_x, curr_y, curr_theta))
            is_inside = boundary.contains(placed_shape)
            is_colliding = base_shape.intersects(placed_shape) if not base_shape.is_empty else False
            push_v = np.array([curr_x - pt_push.x, curr_y - pt_push.y])
            push_v_u = push_v / (np.linalg.norm(push_v) + 1e-6)
            if not base_shape.is_empty:
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
            if not is_inside or is_colliding:
                to_center = np.array([bound_centroid.x - curr_x, bound_centroid.y - curr_y])
                active_dir = to_center / (np.linalg.norm(to_center) + 1e-6)
            else:
                active_dir = attract_v_u * 0.7 + push_v_u * 0.3

        best_ray_v = np.array([0.0, 0.0])
        max_int = -float('inf')

        if propose_geom is not None and g is not None:
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

