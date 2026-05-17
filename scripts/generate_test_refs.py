#!/usr/bin/env python3
"""Print Shapely ground-truth for simple polygons used in C++ tests."""

from shapely.geometry import Polygon

# test_highly_curved_linestrings.cc — spiral-like ring + inner box
spiral_ring = [
    (10.0, 0.0),
    (0.0, 10.0),
    (-10.0, 0.0),
    (0.0, -10.0),
    (8.0, 0.0),
    (0.0, 8.0),
    (-8.0, 0.0),
    (0.0, -8.0),
    (6.0, 0.0),
]
box_ring = [(5.0, -1.0), (7.0, -1.0), (7.0, 1.0), (5.0, 1.0)]

spiral = Polygon(spiral_ring)
box = Polygon(box_ring)

print("// Shapely reference (scripts/generate_test_refs.py):")
print(f"//   spiral.area = {spiral.area}")
print(f"//   box.area = {box.area}")
print(f"//   spiral.intersects(box) = {spiral.intersects(box)}")
print(f"//   spiral.distance(box) = {spiral.distance(box)}")
