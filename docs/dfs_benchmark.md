# DFS refinement benchmark

Tunes `SelectionConfig` and `apply_dfs_refinement` modes for speed vs parts placed.

## Run

Isolated DFS (rebuilds graph per variant, measures DFS wall time):

```bash
PYTHONPATH=. python scripts/benchmark_dfs.py --quick --seeds 0 1 2
```

Full grid (modes × passes × tries × max_passes × repair):

```bash
PYTHONPATH=. python scripts/benchmark_dfs.py --seeds 0 1 2
```

End-to-end mini nest loop (propose + graph + DFS):

```bash
PYTHONPATH=. python scripts/benchmark_nest_pipeline.py --propose shipped \
  --dfs-modes merged_loose_finalize_end merged_loose_tight_finalize_end \
  --compare-dfs-tuning --seeds 0 1 2 --iters 2
```

Output: `docs/dfs_benchmark_results.txt`, `docs/nest_pipeline_benchmark_results.txt`.

## Quick benchmark (first iteration, seeds 0–2)

| Mode | passes | dfs_final | Δnest |
|------|--------|-----------|-------|
| `merged_loose_finalize_end` | 2 | **21.7** | +0.7 |
| `merged_loose_tight_finalize_end` | 3 | 21.0 | +1.3 |
| `merged_single_pass` | 2 | 21.3 | +0.3 |
| `nest_only` | — | 20.0 | 0 |

DFS wall time is sub-millisecond on this small graph; tuning targets outer-loop cost and multi-iter runs.

## End-to-end (2 iterations, shipped propose)

| DFS config | parts_final | time_s |
|------------|-------------|--------|
| legacy (p4, tries8, mp32, tight+finalize) | 38.7 | **35.1** |
| **`merged_loose_finalize_end` (shipped)** | **38.3** | **25.0** |
| `merged_loose_tight_finalize_end` | 37.3 | 27.2 |

**~29% faster** than legacy budget with **−1 part** mean (within seed noise).

## Shipped DFS defaults

| Field | Value | Notes |
|-------|-------|-------|
| `dfs_mode` | `merged_loose_tight_finalize_end` | Loose→tight search; `finalize_selection` once at end |
| `dfs_passes` | 3 | Was 4 |
| `dfs_max_tries` | 4 | Was 8 |
| `dfs_refine_max_stagnant_passes` | 4 | Stop after this many sweeps with no score gain |
| `dfs_refine_max_passes` | 1024 | Hard safety cap on refinement sweeps per call |
| `dfs_refine_beam_width` | 2 | |
| `dfs_finalize_repair_passes` | 6 | Was 8 |

### Modes

- **`merged_loose_tight_finalize_end`** — shipped default; best mean parts in nest-pipeline benchmark.
- **`merged_loose_finalize_end`** — loose only; faster; used by `BuildGraphConfig.benchmark_aligned()`.
- **`merged_loose_tight`** — finalize after every pass (older; more MIS repair work).
- **`merged_single_pass`** — loose only + finalize each pass.

Override: `NEST_DFS_MODE`, `NEST_DFS_PASSES`, `NEST_DFS_MAX_TRIES`, `NEST_DFS_REFINE_STAGNANT_PASSES`, `NEST_DFS_REFINE_MAX_PASSES`, `NEST_DFS_FINALIZE_REPAIR`.
