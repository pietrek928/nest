# Nest graph configuration (`NEST_*` environment variables)

Configuration lives in `nest_graph.config`. Load defaults or overrides:

```python
from nest_graph.config import BuildGraphConfig

cfg = BuildGraphConfig.from_env()
```

Run the demo loop:

```bash
uv run python -m nest_graph.build_graph
```

See [README.md](../README.md) for setup and build instructions.

## Presets

| Preset | Suggested env |
|--------|----------------|
| **Fast** (debug / CI) | `NEST_RANDOM_PER_ITER=64` `NEST_MAX_TRANSFORMS=300` `NEST_IMPROVE_ROUNDS=2` `NEST_BUILD_GRAPH_ITERS=32` |
| **Balanced** (code defaults, first-pass tuned) | `256` / `900` / `4` / `256` — see [first_pass_tuning.md](first_pass_tuning.md) |
| **Quality** (slow) | `NEST_RANDOM_PER_ITER=256` `NEST_MAX_TRANSFORMS=1200` `NEST_IMPROVE_ROUNDS=8` `NEST_BUILD_GRAPH_ITERS=256` |

Disable transform cap: `NEST_MAX_TRANSFORMS=none` (or `0`).

Disable progress bar: `NEST_PROGRESS=0`. When enabled, the bar shows `parts` (selected count), `cov` (% of nest-board area), `pool` (graph nodes), and `refine` (nest→DFS).

## Sampling

| Variable | Default | Meaning |
|----------|---------|---------|
| `NEST_RANDOM_PER_ITER` | 128 | Random transforms per group when propose is off |
| (code) `random_per_iter_when_proposed` | 48 | Random pool when structured proposals exist |
| (code) `structured_jitter_per_proposal` | 8 | Deterministic offsets around each proposal |
| `NEST_INITIAL_RANDOM` | 256 | Bootstrap `selected_t` size |
| `NEST_SELECTION_EXPAND_N` | 4 | `transform_selection` expansions |
| `NEST_HISTORY_EXPAND_N` | 2 | `transform_history` expansions |
| `NEST_HISTORY_MAX` | 512 | Max unique history rows kept (tail) |
| `NEST_MAX_TRANSFORMS` | 900 | Subsample cap before graph build; `none` = no cap |
| `NEST_TRANSFORM_SX` / `SY` / `SA` | 1.5 / 1.5 / 2π | Random transform scale |
| `NEST_SEED` | (unset) | RNG seed for reproducibility |
| `NEST_SHUFFLE_PASSES` | 2 | Shuffled selection/history mix passes per group |
| `NEST_SHUFFLE_PER_PASS` | 32 | Transforms per shuffle pass |

## Graph

| Variable | Default | Meaning |
|----------|---------|---------|
| `NEST_GRAPHS_WINDOW` | 12 | Rule-improvement graph history window |

Board nesting uses a **sheet** polygon: bbox outer ring plus void holes (`bbox \\ outline` and optional `board_holes`). Validity and propose guidance share `evaluate_local_placement` with voids as obstacles (`nest_graph.placement_scene`).

Rules fields (Python config, not env): `board_coords` (nest outline), optional `board_holes`, `board_sheet_padding` (extra margin), `board_sheet_padding_ratio` (default `0.08` × bbox diagonal for the sheet outer rectangle), `max_inserts_per_type` (default 2), `max_rules_per_set` (default 24). Video/snapshots draw the nest outline only, not the sheet bbox.

Rule evolution uses elitist mutation (`improve_rules_elite_count`), deduplication, and the same `score_rules` select path as `nest_by_graph`. DFS refinement scores with the best evolved rule set, not a fixed demo set. See [`docs/rules_benchmark_results.txt`](rules_benchmark_results.txt) and `scripts/benchmark_rules.py`.

## Selection / DFS

| Variable | Default | Meaning |
|----------|---------|---------|
| `NEST_IMPROVE_ROUNDS` | 4 | Rule-mutation rounds per iteration |
| `NEST_RULES_KEPT` | 64 | Top rule sets kept after scoring |
| `NEST_RULES_ELITE` | 16 | Top rule sets mutated each improve round |
| `NEST_RULE_SIZE_PENALTY` | 0.03 | Per-rule penalty in `score_rules` (C++) |
| `NEST_SCORE_RULES_LATEST_ONLY` | 1 | Score rules on latest graph only |
| `NEST_SCORE_RULES_COUNT_WEIGHT` | 0.02 | Count bonus in rule fitness |
| `NEST_SCORE_RULES_LOCAL_SWAP` | 1 | Local swap in rule scoring select |
| `NEST_SELECT_MODE` | `weighted_greedy` | `weighted_greedy` or `greedy_score` |
| `NEST_DFS_MAX_TRIES` | 4 | Retries per DFS growth pass |
| `NEST_DFS_PASSES` | 3 | Repeat refine block per iteration |
| `NEST_DFS_REFINE_STAGNANT_PASSES` | 4 | Stop refine after this many sweeps with no score gain |
| `NEST_DFS_REFINE_MAX_PASSES` | 1024 | Hard safety cap on refine sweeps per call |
| `NEST_DFS_REFINE_BEAM` | 2 | Beam width in refine DFS |
| `NEST_DFS_FINALIZE_REPAIR` | 6 | Repair passes in `finalize_selection` |
| `NEST_DFS_FINALIZE_COMPONENT` | 18 | Max overlap component for exact MIS |
| `NEST_DFS_MODE` | `merged_loose_tight` | See [build_graph_tuning.md](build_graph_tuning.md) |
| `NEST_NEST_RULE_SETS` | 1 | Rule sets passed to `nest_by_graph` |

Rendered placements are always collision-free: DFS refinement only adds non-colliding nodes, and the build loop prunes the final selection to an independent set before drawing.

## Placement proposals (`propose.py`)

Enabled by default. Runs **erosion, raycast, voronoi**, optional **ribbon gap seeds** and **point-cloud** (`ProposeConfig`), searches in `sheet.difference(base.buffer(min_dist))` when `use_free_region_search=True`, Propose uses the **nearest packed cluster** as obstacles only; `make_polygon_graph` still collision-filters against the full layout. Trims the merged pool by C++ clearance when `trim_candidates_by_clearance=True`, validates/ranks with `evaluate_local_placement`, and keeps the best `max_proposals` per group. See [`docs/propose_benchmark.md`](propose_benchmark.md).

**Clearance / physics mapping**

| Python | C++ `GuidanceConfig` | Role |
|--------|----------------------|------|
| `board_min_dist()` = board diagonal × `min_dist_ratio` | `minimum_placing_distance` = `min_dist + epsilon` | Inflates MTV on overlap; gravity stops before contact |
| `placement_clearance_epsilon_ratio` (default `0.05`) | (via margin check in Python) | Extra slack so near-touching poses are rejected |
| `guidance_diversity_dist_ratio` | `diversity_distance_threshold` | Scaled from `min_dist` / board diag (not raw C++ defaults) |
| `use_guidance_propositions`, `guidance_max_propositions` | `max_propositions`, proposition menu | Cast-based moves from `evaluate_local_placement` |
| `guidance_use_tight_packing` | `use_tight_packing` | `Exact Neighbor Snap` via polygon cast |
| `guidance_use_corner_alignment` | `use_corner_alignment` | `Vertex Corner Match` / `Corner Match (Intercept)` |
| `guidance_enable_grid` | `enable_grid_exploration` | `Floor Walk L/R` (perpendicular casts along gravity tangent) |
| (C++ default) | `max_hole_size_ratio` | Hole seeking when void radius ∈ `[placed_radius, placed_radius × ratio]` |
| (C++ default) | `search_radius` | Cast/distance horizon; scaled up from board diag in Python |

**Shipped propose defaults** (cast `guide.h`, re-tune with `scripts/benchmark_guidance_flow.py`): `use_guidance_propositions=True`, `guidance_use_tight_packing=True`, `guidance_use_corner_alignment=True`, `guidance_enable_grid=False`, `guidance_max_propositions=6`.

`make_polygon_graph` filters candidates with validity-only guidance (`guidance_config_for_graph`) using the same `board_min_dist()` and epsilon as propose. Set `min_dist_ratio` or `placement_clearance_epsilon_ratio` on `ProposeConfig` to tune spacing.

For the guidance-flow pipeline preset, use `BuildGraphConfig.benchmark_aligned()` (`merged_loose_tight` DFS).

```python
from nest_graph.config import BuildGraphConfig, ProposeConfig

# Leaner
cfg = BuildGraphConfig(propose=ProposeConfig(max_proposals=8, candidate_pool=8, use_point_cloud=False))

# Stronger PSO (slower)
cfg = BuildGraphConfig(propose=ProposeConfig(point_cloud_particles=24, point_cloud_iterations=32))
```

## Output

| Variable | Default |
|----------|---------|
| `NEST_BUILD_GRAPH_ITERS` | 256 |
| `NEST_VIDEO_PATH` | `test.mp4` |
| `NEST_SNAPSHOT_PATH` | `test.jpg` |
| `NEST_VIDEO_FPS` | 5 |
| `NEST_RENDER_SIZE` | 1024 |
| `NEST_PROGRESS` | 1 (set `0` to disable tqdm) |
