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

Disable progress bar: `NEST_PROGRESS=0`.

## Sampling

| Variable | Default | Meaning |
|----------|---------|---------|
| `NEST_RANDOM_PER_ITER` | 256 | Random transforms per group per iteration |
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
| `NEST_BOARD_CHECK` | `vertices` | `vertices` or `contains` (Shapely parity) |
| `NEST_GRAPHS_WINDOW` | 12 | Rule-improvement graph history window |

## Selection / DFS

| Variable | Default |
|----------|---------|
| `NEST_IMPROVE_ROUNDS` | 4 |
| `NEST_RULES_KEPT` | 64 |
| `NEST_RULE_SIZE_PENALTY` | 0.01 |
| `NEST_DFS_MAX_TRIES` | 8 |
| `NEST_DFS_MIN_COLLISIONS_LOOSE` | 2 |
| `NEST_DFS_MIN_COLLISIONS_TIGHT` | 1 |
| `NEST_DFS_PASSES` | 2 |
| `NEST_NEST_RULE_SETS` | 1 |

## Placement proposals (`propose.py`)

Enabled by default. Runs **erosion, raycast, voronoi, and point-cloud** on light settings (`ProposeConfig`), validates/ranks with `nest_graph.geometry` (native intersect + distance), and keeps the best `max_proposals` (12) per group.

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
