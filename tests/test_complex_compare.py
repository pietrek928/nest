import math
import pytest
import numpy as np
from shapely.geometry import Polygon, MultiPolygon
from shapely import affinity
from nest_graph.geometry import Geometry, find_polygon_intersections, find_polygon_distances

def create_star(num_points=5, outer_radius=10.0, inner_radius=4.0):
    pts = []
    for i in range(num_points * 2):
        angle = i * math.pi / num_points
        r = outer_radius if i % 2 == 0 else inner_radius
        pts.append((r * math.cos(angle), r * math.sin(angle)))
    return Polygon(pts)

def create_swiss_cheese(size=20.0, num_holes=3):
    outer = [(0, 0), (size, 0), (size, size), (0, size)]
    holes = []
    hole_size = size / (num_holes * 2)
    for i in range(num_holes):
        cx = size * (i + 1) / (num_holes + 1)
        cy = size / 2
        hole = [
            (cx - hole_size, cy - hole_size),
            (cx + hole_size, cy - hole_size),
            (cx + hole_size, cy + hole_size),
            (cx - hole_size, cy + hole_size)
        ]
        holes.append(hole)
    return Polygon(outer, holes)

def create_spiral(turns=2, points_per_turn=16, max_radius=15.0, thickness=2.0):
    outer_pts = []
    inner_pts = []
    for i in range(turns * points_per_turn + 1):
        t = i / points_per_turn
        angle = t * 2 * math.pi
        r_out = (t / turns) * max_radius + thickness
        r_in = (t / turns) * max_radius
        if r_in < 0.1: r_in = 0.1
        
        outer_pts.append((r_out * math.cos(angle), r_out * math.sin(angle)))
        inner_pts.insert(0, (r_in * math.cos(angle), r_in * math.sin(angle)))
        
    return Polygon(outer_pts + inner_pts)

def create_multipolygon():
    p1 = Polygon([(0, 0), (2, 0), (2, 2), (0, 2)])
    p2 = Polygon([(4, 4), (6, 4), (6, 6), (4, 6)])
    p3 = Polygon([(8, 0), (10, 0), (10, 2), (8, 2)])
    return MultiPolygon([p1, p2, p3])

@pytest.fixture
def complex_polygon_soup():
    base_shapes = [
        create_star(5, 10, 4),
        create_star(8, 12, 8),
        create_swiss_cheese(20, 4),
        create_spiral(3, 20, 15, 3),
        create_multipolygon()
    ]
    
    soup = []
    np.random.seed(42)
    
    # Create a dense soup of overlapping and disjoint shapes
    for shape in base_shapes:
        # Original
        soup.append(shape)
        
        # Translated and rotated
        for _ in range(3):
            tx, ty = np.random.uniform(-30, 30, 2)
            angle = np.random.uniform(0, 360)
            transformed = affinity.translate(shape, tx, ty)
            transformed = affinity.rotate(transformed, angle, origin='centroid')
            soup.append(transformed)
            
    return soup


def test_complex_intersections_match_shapely(complex_polygon_soup):
    # 1. Convert all to our Geometry
    geoms = [Geometry.from_shapely(p) for p in complex_polygon_soup]
    
    # 2. Run our batch intersection solver
    results = find_polygon_intersections(geoms)
    our_intersecting_pairs = set((r[0], r[1]) for r in results)
    
    # 3. Compare with Shapely ground truth
    mismatches = []
    for i in range(len(complex_polygon_soup)):
        for j in range(i + 1, len(complex_polygon_soup)):
            shapely_intersects = complex_polygon_soup[i].intersects(complex_polygon_soup[j])
            our_intersects = (i, j) in our_intersecting_pairs
            
            if our_intersects != shapely_intersects:
                mismatches.append((i, j, shapely_intersects, our_intersects))
                
    assert len(mismatches) == 0, f"Found {len(mismatches)} mismatches: {mismatches}"

def test_complex_distances_match_shapely(complex_polygon_soup):
    geoms = [Geometry.from_shapely(p) for p in complex_polygon_soup]
    
    # Run batch distance solver with a large aura to capture all relevant pairs
    results = find_polygon_distances(geoms, aura=100.0)
    
    # Map results by pair (always store as sorted tuple)
    our_distances = {}
    for r in results:
        pair = tuple(sorted((r.polyA_idx, r.polyB_idx)))
        our_distances[pair] = r
    
    mismatches = []
    for i in range(len(complex_polygon_soup)):
        for j in range(i + 1, len(complex_polygon_soup)):
            shapely_dist = complex_polygon_soup[i].distance(complex_polygon_soup[j])
            pair = (i, j)
            
            if shapely_dist == 0.0:
                # If they intersect, our engine should report intersect == True
                if pair in our_distances:
                    if not our_distances[pair].intersect:
                        # Allow micro-gaps due to float precision
                        our_dist = math.sqrt(our_distances[pair].distance_sq)
                        if our_dist > 1e-4:
                            mismatches.append((i, j, "expected intersect", our_dist))
            elif pair in our_distances:
                our_dist = math.sqrt(our_distances[pair].distance_sq)
                if not math.isclose(our_dist, shapely_dist, rel_tol=1e-4, abs_tol=1e-5):
                    mismatches.append((i, j, shapely_dist, our_dist))
                    
    assert len(mismatches) == 0, f"Found {len(mismatches)} distance mismatches: {mismatches}"

