import math

import numpy as np
import pytest
from shapely import affinity
from shapely.geometry import MultiPolygon, Polygon

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
    # Packing SoT = Penetrating only (kiss ≠ packing hit).
    from tests.geometry_oracle import assert_batch_packing_collide

    assert_batch_packing_collide(complex_polygon_soup)


def test_complex_distances_match_shapely(complex_polygon_soup):
    from tests.geometry_oracle import assert_batch_distances_match_shapely

    assert_batch_distances_match_shapely(complex_polygon_soup, aura=100.0)

