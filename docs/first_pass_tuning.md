# First-pass parameter tuning (2026-05-20)

Benchmark: one full iteration of the demo board (rect + tri), seeds `0..2`, metric **`dfs_sel`** = placements after `nest_by_graph` + DFS refinement.

Run: `PYTHONPATH=. python scripts/benchmark_first_pass.py`

## Results (mean over 3 seeds)

| preset | graph_nodes | collision_edges | nest_sel | **dfs_sel** | batch/g0 | batch/g1 | time_s | notes |
|--------|-------------|-----------------|----------|-------------|----------|----------|--------|-------|
| **J_max_density** | **94** | **480** | **19** | **20** | 1000 | 1000 | **0.47** | **Winner — now default** |
| K_recommended | 96 | 515 | 17 | 18 | 900 | 900 | 0.50 | Slightly leaner cap |
| D_high_random | 64 | 200 | 17 | 18 | 900 | 900 | 0.94 | High random, no propose boost |
| C_propose_heavy | 62 | 216 | 15 | 16 | 600 | 600 | 0.76 | + point_cloud (slow) |
| E_shuffle_heavy | 60 | 227 | 16 | 16 | 600 | 600 | 0.90 | shuffle 3×48 only |
| G_quality_rules | 59 | 225 | 15 | 15 | 600 | 600 | 0.95 | improve_rules=6 |
| H_first_pass_combo | 56 | 174 | 13 | 14 | 750 | 750 | 0.44 | Mid propose, no PSO |
| I_propose_shuffle | 53 | 172 | 14 | 14 | 600 | 600 | 0.43 | shuffle + propose |
| A_current_defaults (old) | 51 | 144 | 13 | 14 | 600 | 600 | 0.90 | pre-tuning baseline |
| F_fast_rules | 53 | 203 | 12 | 13 | 600 | 600 | 0.90 | improve_rules=2 |
| B_lean_propose | 58 | 246 | 12 | 12 | 600 | 600 | 0.06 | erosion+raycast only |

## Adopted defaults (`J_max_density`)

| Parameter | Old | New |
|-----------|-----|-----|
| `initial_random` | 128 | **256** |
| `random_per_iter` | 128 | **256** |
| `max_transforms_per_group` | 600 | **900** |
| `propose.max_proposals` | 12 | **20** |
| `propose.candidate_pool` | 12 | **16** |
| `propose.use_point_cloud` | true | **false** (PSO costly; little gain vs voronoi) |
| `shuffle_passes` / `shuffle_per_pass` | 2 / 32 | unchanged |

## Takeaways

1. **First iteration** is dominated by `initial_random` + `random_per_iter` + transform cap — larger batches (~900–1000) yield denser graphs and ~50% more DFS-selected parts (14 → 20).
2. **Propose** (erosion + raycast + voronoi, no PSO) adds diversity at low cost when paired with large batches.
3. **Shuffle** helps later iterations more than pass 1; kept at 2×32 as cheap insurance.
4. **Point-cloud PSO** did not beat `J` on dfs_sel; left off by default (`use_point_cloud=false`).

## Reproduce

```bash
PYTHONPATH=. python scripts/benchmark_first_pass.py
```

Single seed spot-check:

```python
from nest_graph.config import BuildGraphConfig
from scripts.benchmark_first_pass import run_first_pass
m = run_first_pass(BuildGraphConfig(), seed=0)
print(m.selected_dfs, m.graph_nodes, m.time_s)
```
