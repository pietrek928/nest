# Propose benchmark (gap-fitting)

Isolated benchmark for the **propose** step (no full 256-iter nest loop). Compares config presets on triangle-board fixtures.

## Run

```bash
PYTHONPATH=. python scripts/benchmark_propose.py --seeds 0 1 2 3 4 5 6 7 8 9
```

Key presets:

```bash
PYTHONPATH=. python scripts/benchmark_propose.py --presets ribbon_free shipped free_pso_light
```

Output: `docs/propose_benchmark_results.txt`.

## Scenarios

| Scenario | Setup |
|----------|--------|
| `empty_base` | First part on empty sheet |
| `partial_pack` | One rect placed; propose triangle into pocket |
| `two_clusters` | Two separated rects; propose triangle between them |

## Results (seeds 0–9, 2026-05-25)

| preset | scenario | valid | clearance_mean | clearance_min | time_s |
|--------|----------|-------|----------------|---------------|--------|
| **ribbon_free** | partial_pack | 12.0 | **0.0939** | **0.0631** | 0.37 |
| **ribbon_free** | two_clusters | 12.0 | **0.1842** | **0.1603** | 1.47 |
| shipped | partial_pack | 12.0 | 0.0939 | 0.0631 | 0.35 |
| shipped | two_clusters | 12.0 | 0.1842 | 0.1603 | **3.17** |
| free_pso_light | partial_pack | 12.0 | 0.1011 | 0.0676 | 0.47 |
| free_clearance | partial_pack | 12.0 | 0.0818 | 0.0240 | 0.29 |

### Interpretation

1. **`ribbon_free` wins** on gap quality (partial pack + two clusters) at acceptable time.
2. **`free_pso_light`** — slightly higher clearance (~0.068 min) but ~1.3× slower; left off defaults.
3. **`trim_candidates_by_clearance`** — no change vs untrim on this fixture when pool is small; kept on as safeguard.
4. Propose obstacles: **nearest packed cluster** only; graph still validates against full layout.

## Shipped defaults (`ProposeConfig`)

| Field | Value | Rationale |
|-------|-------|-----------|
| `use_free_region_search` | `True` | Valid partial-pack candidates |
| `ranking_mode` | `"clearance"` | C++ clearance ranking |
| `trim_candidates_by_clearance` | `True` | Pool trim before final rank |
| `use_ribbon_seeds` | `True` | Best min clearance on partial pack |
| `use_group_edge_seeds` | `True` | Snap along nearest packed-cluster exterior |
| `use_contact_ranking` | `True` | Packed layouts: rank/trim by tight border/group fit |
| `use_contact_clearance_hybrid` | `True` | `contact_hybrid` = tight fit + clearance for pockets |
| `use_stratified_contact_trim` | `True` | 65% contact + 35% clearance candidates in pool |
| `candidate_pool` | `48` | Larger pool before trim (was 32) |
| `max_proposals` | `24` | More seeds per group to graph |
| `multi_site_erosion` | `True` | Extra deterministic erosion sites |
| `use_border_focus` | `True` | Empty board: push/rank toward sheet edge ([border benchmark](propose_border_benchmark.md)) |
| `use_border_edge_seeds` | `True` | Corner + edge seed generator |
| `border_focus_ranking` | `True` | Empty board: prefer tight border standoff over deep clearance |
| `candidate_pool` | `32` | Needed for corner seeds before trim |
| `use_point_cloud` | `False` | Cost vs benefit |
| `smart_push_target` | `True` | Push toward packed centroid |

Production: `proposed_transforms_for_groups` → nearest-cluster obstacles + ribbon/border seeds; `make_polygon_graph` validates all packed parts.

## Tests

```bash
PYTHONPATH=. python -m pytest tests/test_propose_gap.py -q
```
