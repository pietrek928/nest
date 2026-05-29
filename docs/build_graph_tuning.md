# Build-graph tuning (2026-05-29)

Multi-iteration demo pipeline (`rect` + `tri`, 3 iters, triangle board). Run:

```bash
PYTHONPATH=. python scripts/benchmark_build_graph_tune.py --seeds 0 1 2 3 4
```

Combo validation (5 seeds, 3 iters):

| config | mean parts_final |
|--------|------------------|
| **shipped (tuned)** `merged_loose_tight` + `dfs_passes=3` + `improve_rules_rounds=4` | **47.8** |
| previous `merged_loose_tight_finalize_end` + `dfs_passes=4` + `improve_rules_rounds=2` | 46.8 |
| `merged_loose_finalize_end` only | 47.2 |

## Shipped `SelectionConfig` changes

| Field | Was | Now | Why |
|-------|-----|-----|-----|
| `dfs_mode` | `merged_loose_tight_finalize_end` | **`merged_loose_tight`** | +1.0 parts vs previous combo; finalize once at end (not each pass) |
| `dfs_passes` | 4 (docs) / 3 (code) | **3** | Ties best part count with lower DFS cost on `merged_loose_tight` |
| `improve_rules_rounds` | 2 | **4** | Better rule sets before nest/DFS (+0.8 parts in combo test) |

`merged_loose_tight` still runs `finalize_selection` at the end (collision-free output).

## Env overrides

| Variable | Default |
|----------|---------|
| `NEST_DFS_MODE` | `merged_loose_tight` |
| `NEST_DFS_PASSES` | `3` |
| `NEST_IMPROVE_ROUNDS` | `4` |

Raw tables: [`build_graph_tuning_results.txt`](build_graph_tuning_results.txt).

After **`guide.h`** changes, rebuild `geometry` and re-run [guidance_cast_tuning.md](guidance_cast_tuning.md) plus [propose_benchmark.md](propose_benchmark.md) propose matrix.
