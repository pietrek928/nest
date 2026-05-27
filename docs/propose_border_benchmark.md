# Border propose benchmark

Empty triangle board, place a rectangle tight against the sheet edge.

## Run

```bash
PYTHONPATH=. python scripts/benchmark_propose_border.py --seeds 0 1 2 3 4 5 6 7 8 9
```

Results: `docs/propose_border_benchmark_results.txt`

## Metrics

| Metric | Meaning |
|--------|---------|
| `border_dist_min` | Min distance from placed part to sheet exterior (lower = tighter edge fit) |
| `border_slack_mean` | Mean `(border_dist - min_dist)` over proposals |

## Results (seeds 0–9, 2026-05-25)

| preset | border_dist_min | border_slack_mean | time_s |
|--------|-----------------|-------------------|--------|
| **border_focus / shipped** | **0.0080** | **0.0048** | ~0.72 |
| centroid_clearance | 0.1973 | 0.2099 | 0.21 |
| ribbon_only | 0.1973 | 0.2099 | 0.31 |

## Shipped border behavior (empty board)

- `use_border_focus` — push/attract toward border ring, not board centroid
- `use_border_edge_seeds` — corner + edge sampling with validity filter
- `border_focus_ranking` — rank valid poses by tight border standoff (not max clearance)
- `candidate_pool=32` — room for corner seeds before trim

On **partial pack**, `border_focus_ranking` is off; clearance ranking and ribbon still apply.
