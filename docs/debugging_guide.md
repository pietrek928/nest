# Geometry Engine Debugging Guide

This document contains useful Python snippets for debugging the C++ geometry engine, visualizing shapes, and comparing results against the Shapely library.

## 0. Building the Python Extension

`pytest` imports `nest_graph.geometry._geometry` from the source tree (`nest_graph/geometry/_geometry.cpython-*.so`). After changing C++ headers under `nest_graph/poly/`, rebuild that module or tests will run against stale code.

**Preferred (when Python dev headers are available):**

```bash
uv pip install -e .
```

**Workaround (plain CMake in `build/`):** use this when editable install fails with `Could NOT find Python (missing: Development.Module)`:

```bash
mkdir -p build && cd build
cmake .. -DNEST_GRAPH_BUILD_TESTS=ON
cmake --build . --target _geometry
cd ..
pytest tests/
```

The `_geometry` target writes the `.so` next to `nest_graph/geometry/__init__.py`. C++ unit tests: `cmake --build build --target poly_cpp_tests && ./build/nest_graph/poly/poly_cpp_tests`.

**Polygon holes:** `decompose_complex_polygon` decomposes outer rings (additive) and hole rings (reversed winding, `is_subtractive` on each `line_part`). `is_point_inside_solid_space` uses signed ray crossings so hole interiors are void. Hole boundary segments are **not** swept as colliders (only additive `line_parts` enter broad-phase).

**Solid geometry model:** Use `Geometry.from_shapely` or one outer ring plus holes via `decompose_complex_polygon`. Multiple separate `add_boundary_ring` calls on disjoint rectangles do **not** form a union solid (e.g. a manual C-shape built from three rings does not fill the cavity as one region). For union-like shapes, supply a single outer boundary (possibly concave) and holes only.

## 1. Visualizing Shapes with Matplotlib

When intersections or distances mismatch, it's helpful to plot the shapes to see their spatial relationship.

```python
import matplotlib.pyplot as plt
from shapely.geometry import Polygon
from nest_graph.geometry import Geometry

def plot_shapes(shape_a: Polygon, shape_b: Polygon, title="Shape Comparison"):
    fig, ax = plt.subplots()
    
    # Plot Shape A
    x_a, y_a = shape_a.exterior.xy
    ax.plot(x_a, y_a, color='#6699cc', alpha=0.7, linewidth=3, solid_capstyle='round', zorder=2)
    ax.fill(x_a, y_a, alpha=0.3, color='#6699cc')
    
    # Plot Shape B
    x_b, y_b = shape_b.exterior.xy
    ax.plot(x_b, y_b, color='#ff9999', alpha=0.7, linewidth=3, solid_capstyle='round', zorder=2)
    ax.fill(x_b, y_b, alpha=0.3, color='#ff9999')
    
    ax.set_title(title)
    ax.set_aspect('equal')
    plt.grid(True, linestyle=':', alpha=0.6)
    plt.show()
```

## 2. Comparing C++ Engine vs Shapely Ground Truth

This snippet demonstrates how to run a batch of shapes through the C++ engine and compare the intersection results against Shapely.

```python
from nest_graph.geometry import Geometry, find_polygon_intersections

def find_intersections_mismatches(shapely_polygons):
    # Convert Shapely polygons to C++ Geometry objects
    geoms = [Geometry.from_shapely(p) for p in shapely_polygons]
    
    # Run the C++ batch intersection solver
    cpp_results = find_polygon_intersections(geoms)
    cpp_pairs = set((r[0], r[1]) for r in cpp_results)
    
    mismatches = []
    for i in range(len(shapely_polygons)):
        for j in range(i + 1, len(shapely_polygons)):
            shapely_intersects = shapely_polygons[i].intersects(shapely_polygons[j])
            cpp_intersects = (i, j) in cpp_pairs
            
            if cpp_intersects != shapely_intersects:
                mismatches.append({
                    'pair': (i, j),
                    'shapely': shapely_intersects,
                    'cpp': cpp_intersects
                })
                
    return mismatches
```

## 3. Simulating C++ Decomposition in Python

To understand how the C++ engine breaks down concave shapes into convex linestrings, you can simulate the `process_boundary_to_convex_segments` logic in Python.

```python
def simulate_cpp_decomposition(poly: Polygon):
    """
    Simulates the C++ engine's logic of breaking a boundary into convex linestrings,
    splitting when the turn direction changes or exceeds 90 degrees.
    """
    coords = list(poly.exterior.coords)[:-1] # Remove duplicate last point
    n = len(coords)
    if n < 3: return []
    
    parts = []
    current_segment = []
    current_turn_sign = 0
    
    for k in range(n):
        p1 = coords[k]
        p2 = coords[(k + 1) % n]
        p3 = coords[(k + 2) % n]
        
        if not current_segment:
            current_segment.extend([p1, p2])
        else:
            current_segment.append(p2)
            
        # Compute turn (cross product)
        v1 = (p2[0] - p1[0], p2[1] - p1[1])
        v2 = (p3[0] - p2[0], p3[1] - p2[1])
        turn = v1[0] * v2[1] - v1[1] * v2[0]
        
        turn_sign = 1 if turn > 1e-6 else (-1 if turn < -1e-6 else 0)
        
        if current_turn_sign == 0 and turn_sign != 0:
            current_turn_sign = turn_sign
            
        force_split = False
        
        # Split on inflection points
        if current_turn_sign != 0 and turn_sign != 0 and turn_sign != current_turn_sign:
            force_split = True
            
        # Split if accumulated turn > 90 degrees (dot product <= 0)
        if not force_split and len(current_segment) >= 2:
            first_p1 = current_segment[0]
            first_p2 = current_segment[1]
            first_edge = (first_p2[0] - first_p1[0], first_p2[1] - first_p1[1])
            current_edge = (p3[0] - p2[0], p3[1] - p2[1])
            dot = first_edge[0] * current_edge[0] + first_edge[1] * current_edge[1]
            if dot <= 0:
                force_split = True
                
        if force_split:
            parts.append(current_segment)
            current_segment = [p2, p3]
            current_turn_sign = turn_sign
            
    if current_segment:
        parts.append(current_segment)
        
    return parts
```
