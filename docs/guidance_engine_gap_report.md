# Guidance engine gap report (`guide.h`)

Evidence from board-backed scenarios in [`scripts/guidance_placement_fixtures.py`](../scripts/guidance_placement_fixtures.py), visualized via [`scripts/visualize_guidance_placement.py`](../scripts/visualize_guidance_placement.py). Summary metrics: [`docs/guidance_viz/summary.md`](guidance_viz/summary.md).

## 1. Scope

`evaluate_local_placement` in [`nest_graph/geometry/guide/guide.h`](../nest_graph/geometry/guide/guide.h) is a **local, one-shot proposer**. Given a moving polygon index, obstacle polygons (including sheet voids), a part center, and `GuidanceConfig`, it returns ranked translation deltas (plus optional rotation) — not a global packing solution.

Python exposes it through [`nest_graph/geometry/bindings/api.cc`](../nest_graph/geometry/bindings/api.cc) as `GuidanceConfig`, `PlacementProposition`, `PlacementGuidance`, and `evaluate_local_placement`. Binding contract tests live in [`tests/test_guide_bindings.py`](../tests/test_guide_bindings.py).

## 2. Strengths (validated)

| Capability | Scenario evidence |
|------------|-------------------|
| **Overlap ejection** | `overlap_recovery`, `l_snap_to_bar`, `l_shape_pocket` — `Primary Ejection` when `is_penetrating=True` |
| **Vertex corner match** | `concave_notch`, `nest_border_dock`, `rect_cluster_gap` — `Vertex Corner Match` with rotation variants |
| **Forward cast snap** | `nest_border_dock` — cast-based menu (no backward-hit dead zone); `Floor Walk L/R` |
| **Secondary neighbor snap** | `void_corner`, `board_hole_mouth` — `Exact Neighbor Snap` toward multiple obstacles |
| **Non-penetrating rotation** | Alignment normal from closest obstacle enables edge-parallel angles near walls |
| **Diversity filter** | `l_snap_to_bar` — 6 distinct propositions under penetration |
| **Empty-menu guard** | `evaluate_local_placement` always returns ≥1 proposition via gravity fallback |
| **Sheet + void context** | All scenarios use `PlacementScene` with outline, padded sheet, corner voids, optional `board_holes` |
| **Performance** | Hardened scenarios &lt; 0.31 ms (`hex_ring_cavity` 0.21 ms) on test hardware |
| **Bindings** | Full `GuidanceConfig` round-trip; out-of-bounds `placed_poly_idx` raises `IndexError` |

## 3. Fixes applied (this pass)

| Issue | Root cause | Fix |
|-------|------------|-----|
| **Cast dead zone** | `convex_linestrings_cast_impl` reported behind-ray hits with negative `t_entry`, shadowing forward obstacles | Reject `t_max < -ε` in [`cast.h`](../nest_graph/geometry/convex/cast.h); clamp `t_entry` to `≥ 0` in [`polygon_cast.h`](../nest_graph/geometry/guide/polygon_cast.h) |
| **Empty cast menu** | Rejected backward hits left only unconditional `Target Attractor` | Forward cast fix + empty-menu fallback in `evaluate_local_placement` |
| **No rotation near walls** | `alignment_normal` was `(0,0)` when not penetrating | Set from placed→closest-obstacle vector in `aggregate_physics_feedback` |
| **Single-neighbor snap** | Only `closest_poly_idx` tracked | Added `second_closest_*` fields; dual snap + dual corner alignment |
| **Attractor-only propose** | Interior propose used attractor pass alone | Dual-pass guidance in [`placements_guidance.py`](../nest_graph/propose/placements_guidance.py) (tight cast + attractor merge) |
| **Sweep aura conflation** | `search_radius` passed as `aura_multiplier` → disabled broad-phase pruning, radius-relative horizon | Absolute `distance_margin` in [`sweep_engine.h`](../nest_graph/geometry/sweep/sweep_engine.h); guide passes `aura=0`, `distance_margin=search_radius` |
| **Rotation recursion cost** | Full nested `evaluate_local_placement` per alt angle | Lightweight rotate + cast-only pass (single distance query per angle) |
| **Vertex-corner blow-up** | O(V_a × V_b) brute force on stars/gears | Top-K edge-endpoint matching (`collect_edge_endpoints`) |
| **Hole-seek spam** | Unbounded void loop on dense sheets | `max_hole_seeks` + `max_void_polygons_for_holes` budget |
| **Zero-MTV penetration** | Touching overlap reported `penetration_sq=0` → zero ejection | Center-separation fallback MTV in `aggregate_physics_feedback` |
| **Alignment from bbox** | `alignment_normal` used circle centers | Closest feature normal from `ComplexDistanceResult::closest_normal` |
| **Post-ejection menu** | Ejection-only under overlap | Virtual ejection pose + cast snap / re-seat toward other obstacles |
| **Border dock** | Gravity diagonal only | `use_board_edge_cast` axis-aligned casts when `border_focus` |
| **Debug attribution** | No obstacle index on propositions | `PlacementProposition.closest_poly_idx` exposed in bindings |

## 4. Remaining gaps (ranked by nesting impact)

| Area | Limitation | Impact | Mitigation today |
|------|------------|--------|------------------|
| **Horizon** | `search_radius` is absolute cast/distance margin (not sweep aura) | Tune per board via `guidance_config_for_scene`; tracer bench in [`scripts/benchmark_guidance_tracer.py`](../scripts/benchmark_guidance_tracer.py) |
| **Pose semantics** | Returns translation **deltas**, not absolute `(x,y,θ)` | Easy to mis-apply | `proposition_translation`, `_is_cast_move`, `apply_transform` in propose layer |
| **Border packing** | Axis-aligned `Board Edge Cast X/Y` when `use_board_edge_cast`; diagonal gravity still primary | Residual border kiss on jagged boards | `guidance_config_for_board_edge_anchor`, `placements_edge.py` |
| **Hole seek on sheet voids** | **By design:** board voids are *forbidden* regions, not fillable cavities | `void_corner`, `board_hole_mouth` correctly suppress `Hole Seek` | Do not raise `max_hole_size_ratio` for sheet voids |
| **Validity** | `clearance` is pairwise min in horizon, not board standoff | Some props may still fail `board_valid` | `is_valid_placement` filter in propose layer |
| **Scoring** | Fixed heuristic tiers (95/85/75…) | Misaligned with graph/rule ranking | Guide seeds only; `rule_hybrid` ranks in propose |
| **Exploration** | `enable_grid_exploration` = floor walk L/R only | Limited pocket search | `voronoi`, `erosion`, `ribbon_free` |
| **API / debug** | `closest_poly_idx` on propositions; `evaluate_local_placement_traced` for GJK counts | Full `PhysicsContext` still internal | Viz quiver + tracer bench |

## 5. Python workarounds today

- **`PlacementScene`** — packs `[placed, *obstacles, *voids]` and calls `evaluate_local_placement(0, …)` with part center ([`placement_scene.py`](../nest_graph/placement_scene.py)).
- **`guidance_config_for_propose`** / **`guidance_config_for_board_edge_anchor`** — scale `search_radius`, `minimum_placing_distance`, attractor vs gravity per use case.
- **Dual-pass guidance** — tight cast menu merged with attractor pass in [`placements_guidance.py`](../nest_graph/propose/placements_guidance.py).
- **`guidance_walk`** — multi-step cast application in propose layer.
- **Other proposers** — voronoi, ribbon, point cloud, border edge when guide alone is insufficient.

## 6. Scenario evidence (post-fix)

Re-run `scripts/visualize_guidance_placement.py` for PNGs. Key changes vs pre-fix audit:

| Scenario | Before | After |
|----------|--------|-------|
| `nest_border_dock` | 1 prop, `Target Attractor` only | Multi-prop cast menu: `Vertex Corner Match`, `Floor Walk` |
| `rect_cluster_gap` | `Target Attractor` only | `Vertex Corner Match` with alt angles |
| `void_corner` | `Target Attractor`; hole seek absent | `Exact Neighbor Snap` + corner match (hole seek correctly absent) |
| `board_hole_mouth` | `Target Attractor`, `board_valid=False` | Cast snaps; validity filtered in propose |

## 7. Performance (post-improvement pass)

Tracer baseline: [`docs/guidance_tracer_bench.txt`](guidance_tracer_bench.txt) via `scripts/benchmark_guidance_tracer.py`.

| Scenario | exec_ms (before hardening pass) | exec_ms (now) |
|----------|--------------------------------|---------------|
| `hex_ring_cavity` | 0.63 | 0.21 |
| `jagged_inner_dock` | 0.49 | 0.22 |
| `triple_void_lane` | 0.30 | 0.31 |
| `l_shaped_sheet_corner` | 0.18 | 0.19 |

Move menus on all 19 scenarios unchanged under `tests/test_guidance_scenarios.py` (55 tests).

## 8. Recommendations (remaining)

| Priority | Change | Where |
|----------|--------|-------|
| Low | Expose optional full `PhysicsContext` on `PlacementGuidance` | `api.cc` |
| Pipeline | Keep guide as **seed** proposer; do not rely on it for final collision-free output | `refine_selection` / `finalize_selection` |

C++ unit tests: [`nest_graph/geometry/tests/test_guide.cc`](../nest_graph/geometry/tests/test_guide.cc) — includes star-ring cavity and penetrating escape hardened cases.

## 9. How to reproduce

```bash
cmake -S . -B build -DNEST_GRAPH_BUILD_TESTS=ON
cmake --build build --target geometry geometry_cpp_tests
./build/nest_graph/geometry/geometry_cpp_tests "[guide]"

uv run pytest tests/test_guide_bindings.py tests/test_guidance_scenarios.py -q
uv run python scripts/benchmark_guidance_tracer.py
uv run python scripts/visualize_guidance_placement.py --scenarios all --out docs/guidance_viz/
```

PNGs and the markdown table are written under [`docs/guidance_viz/`](guidance_viz/).
