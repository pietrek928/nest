# Guidance cast tuning (`guide.h`)

Cast-based moves from `evaluate_local_placement` (polygon cast). Rebuild after C++ changes:

```bash
cmake --build build --target geometry
PYTHONPATH=. python scripts/benchmark_guidance_flow.py --seeds 0 1 2 3 4
```

## Shipped `ProposeConfig` (2026-05-29, seeds 0–2 pipeline)

| Field | Value | Benchmark note |
|-------|-------|----------------|
| `use_guidance_propositions` | `True` | Cast expansion on contact-ranked seeds |
| `guidance_max_propositions` | `8` | Matches C++ `max_propositions` |
| `guidance_proposition_seed_count` | `16` | Kiss-ranked seeds, not pool prefix |
| `guidance_use_tight_packing` | `True` | `Exact Neighbor Snap` |
| `guidance_use_corner_alignment` | `True` | Vertex/corner match casts |
| `guidance_enable_grid` | `True` | `Floor Walk L/R` |
| `use_neighbor_slide` | `True` | Capped contribution to pool |
| Phase-2 Shapely proposers | off | See [propose_benchmark.md](propose_benchmark.md) |

## Move types (cast path)

| `move_type` | Tier | Role |
|-------------|------|------|
| `Exact Gravity Dock` / `Gravity Fall` | pack | Border / empty sheet |
| `Exact Neighbor Snap` | pack | Kiss to nearest obstacle |
| `Vertex Corner Match` / `Corner Match (Intercept)` | corner | Vertex alignment |
| `Safe Hole Seek` | pack | Sheet voids |
| `Floor Walk L/R` | corner | When `guidance_enable_grid=True` |

## Board-edge cast (2026-06-03)

[`propose_placements_board_edge`](../nest_graph/propose/placements_edge.py) refines snap seeds with [`guidance_config_for_board_edge_anchor`](../nest_graph/placement_scene.py): per-anchor `gravity_vector` points toward the nest edge (not global `(-1,-1)`), `target_position` on the outline anchor, tight packing + corner alignment enabled.

## Pipeline ranking (seeds 0–2, 3 iters)

| flow | parts_mean | partial_pack clearance_min |
|------|------------|----------------------------|
| `shipped_no_props` | 47.3 | 0.034 |
| **`props_no_grid`** (cast, no floor walk) | **47.0** | 0.055 |
| `shipped_props` | 45.3 | 0.055 |

Guidance cast trades ~2 parts vs no-props on this fixture but adds structured kiss poses for graph building. Re-tune after board/proposer changes.

Raw: [`guidance_flow_benchmark.txt`](guidance_flow_benchmark.txt).
