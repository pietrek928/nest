# First-pass parameter tuning (2026-05-20)

Benchmark: one full iteration of the demo board (rect + tri), preset **J_max_density**, seeds `0..9`.

Run all DFS modes:

```bash
PYTHONPATH=. python scripts/benchmark_first_pass.py
```

Spot-check with 16 DFS passes (historical `74db5db` used 16 outer passes):

```bash
PYTHONPATH=. python scripts/benchmark_first_pass.py --git-spot-check
```

## DFS pipeline comparison (mean over 10 seeds)

| mode | nest_sel | dfs_raw | dfs_final | Δnest | dropped | score_sum | time_s |
|------|----------|---------|-----------|-------|---------|-----------|--------|
| nest_only | 19.6 | 19.6 | 19.6 | +0.0 | 0.0 | 0.05 | 0.77 |
| **merged_loose_tight** | 19.4 | 20.6 | **20.6** | **+1.2** | **0.0** | 0.05 | 0.87 |
| merged_single_pass | 19.1 | 20.2 | 20.2 | +1.1 | 0.0 | 0.05 | 0.93 |
| legacy_alternating | 20.7 | 21.5 | 21.5 | +0.8 | 0.0 | 0.05 | 0.99 |
| head_pipeline | 19.3 | 20.2 | 20.2 | +0.9 | 0.0 | 0.05 | 0.96 |
| high_pass_loose (16 passes) | 19.9 | 20.9 | 20.9 | +1.0 | 0.0 | 0.05 | 0.90 |
| strict_no_prune | 19.4 | 19.4 | 19.4 | +0.0 | 0.0 | 0.04 | 0.91 |
| strict_prune | 20.7 | 20.7 | 20.7 | +0.0 | 0.0 | 0.05 | 1.01 |

All modes assert `selection_is_independent` on **final** output.

## Findings

1. **`nest_by_graph` alone** adds no refinement (+0 vs nest); graph DFS still matters on dense graphs.
2. **Merged pipeline** (`refine_selection` loose→tight + `finalize_selection`) matches or beats legacy 5-call alternation with **zero post-finalize drops** on these seeds — convergence + finalize repair/MIS works without greedy prune.
3. **Strict search** (`min_collisions=0`, `max_root_collisions=0`) cannot grow selection; confirms transient overlap during loose search helps exploration.
4. **`head_pipeline`** (legacy 5-call, no finalize) can return overlapping sets in metrics (`dfs_raw == dfs_final`); merged pipeline is the shipped default.
5. **`dfs_passes`** default raised **2 → 4** after merge (cheaper per pass). Historical **128 → 16 → 2** pass reduction was the largest budget cut; `high_pass_loose` at 16 passes is only marginally above merged×4 on this benchmark (+0.3 parts mean).

## Shipped defaults

| Item | Value |
|------|-------|
| Pipeline | `selection.dfs_mode` → **`merged_loose_tight`** (loose→tight per pass, finalize once) |
| `dfs_passes` | **3** |
| `improve_rules_rounds` | **4** |
| Internal loose caps | `min_collisions=2`, `max_root_collisions=2` (C++) |
| Internal tight caps | `min_collisions=1`, `max_root_collisions=1` |
| Finalize | `finalize_selection` (repair → exact weighted MIS ≤18 nodes) |

See [build_graph_tuning.md](build_graph_tuning.md) for multi-iter pipeline benchmark (2026-05-29).

## Board-edge proposer (2026-06-03)

New `board_edge` proposer: nest-outline geometric seeds + per-edge [`guidance_config_for_board_edge_anchor`](../nest_graph/placement_scene.py) (`Exact Gravity Dock`, vertex corner match, floor walk via [`guide.h`](../nest_graph/geometry/guide/guide.h)).

Border benchmark (empty triangle, seeds 0–2): **`board_edge_hybrid`** `border_dist_min` **0.0014** vs `border_focus` **0.0033** — see [propose_border_benchmark.md](propose_border_benchmark.md).

```bash
PYTHONPATH=. python scripts/benchmark_propose_border.py \
  --presets board_edge_hybrid border_focus
PYTHONPATH=. python scripts/benchmark_first_pass.py --seeds 0 1 2 \
  --propose-preset shipped_board_edge --modes merged_loose_tight
```

Defaults: `use_board_edge_seeds=True`, `board_edge_guidance_refine=True`, `board_edge_samples_per_edge=32`.

## Preset tuning (J_max_density, seeds 0–2, prior run)

| preset | graph_nodes | nest_sel | dfs_sel | time_s |
|--------|-------------|----------|---------|--------|
| J_max_density | 94 | 19 | 20 | 0.47 |

See `docs/first_pass_tuning_results.txt` for latest auto-generated tables.

## Propose presets (gap-fitting spot-check)

```bash
PYTHONPATH=. python scripts/benchmark_first_pass.py --seeds 0 1 2 --propose-preset shipped
```

See [propose_benchmark.md](propose_benchmark.md) for isolated propose benchmarks and [nest_pipeline_benchmark.md](nest_pipeline_benchmark.md) for multi-iteration end-to-end comparison.

## Reproduce

```python
from nest_graph.config import BuildGraphConfig
from scripts.benchmark_first_pass import run_first_pass

m = run_first_pass(BuildGraphConfig(), seed=0, mode="merged_loose_tight")
print(m.nest_sel, m.dfs_sel_final, m.prune_dropped, m.score_sum_final)
```
