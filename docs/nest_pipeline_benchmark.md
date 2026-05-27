# Nest pipeline benchmark (build loop)

End-to-end mini build loop (propose → graph → nest → DFS refine) comparing **propose presets** and **DFS modes**.

## Run

```bash
PYTHONPATH=. python scripts/benchmark_nest_pipeline.py --seeds 0 1 2 3 4 --iters 3
```

Faster spot-check:

```bash
PYTHONPATH=. python scripts/benchmark_nest_pipeline.py --seeds 0 1 2 --iters 2 \
  --dfs-modes merged_loose_tight merged_single_pass
```

Output: `docs/nest_pipeline_benchmark_results.txt`.

First-iteration DFS-only check (shipped propose):

```bash
PYTHONPATH=. python scripts/benchmark_first_pass.py --seeds 0 1 2 \
  --modes merged_loose_tight merged_single_pass --propose-preset shipped
```

## Results (seeds 0–2, 2 iterations, 2026-05-26 v2)

| propose | dfs_mode | parts_final | border_err_min | time_s |
|---------|----------|-------------|----------------|--------|
| **shipped** | **merged_loose_tight_finalize_end** | **39.7** | **0.0009** | 26.7 |
| shipped_prev | merged_loose_tight | 39.7 | 0.0016 | 26.7 |
| shipped | merged_single_pass | 38.0 | 0.0005 | 21.9 |
| shipped_prev | merged_single_pass | 38.7 | 0.0034 | 19.4 |

(`border_err_min` = mean min \|dist_to_sheet − min_dist\| on final selection.)

### Interpretation

1. **`merged_loose_tight_finalize_end`** — loose→tight search each pass, **`finalize_selection` once** at the end (avoids repeated MIS drops). Ties best part count with better border fit than per-pass finalize.
2. **Propose v2** — larger pool (48), contact+clearance hybrid ranking, stratified trim, 12 ray/erosion angles.
3. **`shipped_prev`** — pool 32, contact-only trim; still strong on parts but looser borders.

## Shipped `build_graph` settings

| Component | Setting |
|-----------|---------|
| Propose | `ProposeConfig()` defaults — see [propose_benchmark.md](propose_benchmark.md) |
| Transform batch | `proposed_transforms_for_groups` + `expand_structured_transforms` + `random_per_iter_when_proposed=48` |
| DFS | `merged_loose_finalize_end`, `dfs_passes=3`, `dfs_max_tries=4` — see [dfs_benchmark.md](dfs_benchmark.md) |
| Propose | pool 48, hybrid contact rank, stratified trim, 12-angle erosion/ray |
| Env override | `NEST_DFS_MODE=merged_loose_tight_finalize_end` (+quality, ~10% slower) |
