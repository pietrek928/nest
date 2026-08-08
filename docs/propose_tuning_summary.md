# Propose tuning summary (2026-08-08)

## Lean Void Cascade

Shipped `propose_cascade_short_circuit=True`: on `void_seek` / `interior_pocket`, after
`pocket_fit` + `cluster_copy` produce enough fast-valid reserved seeds, skip builders/explorers
(or after builders fill the pool, skip explorers). Legacy emit order preserved when cascade is off.

`void_seek` zone no longer enables `neighbor_slide` / `voronoi` (explorer spam on open voids).

## Diversity / ranking flags (gated)

| Flag | Default | Notes |
|------|---------|-------|
| `use_pose_nms` | False | Hash SE(2) NMS; isolated bench dropped `final_count` 16→13 — retune `pose_nms_eps` before shipping |
| `use_conflict_degree_rank` | False | STRtree AABB penalty; enable only if E2E `parts_final` rises |
| `use_multi_pole_void` | True | Iterative polylabel spine for large remnants |
| `use_ema_proposer_scales` | False | Refine-survival EMA (α=0.15, floor 0.05) via `build_graph` feedback |

## E2E smoke (donut_void seed 0)

| Propose | Parts | Area | Time |
|---------|-------|------|------|
| shipped (pre-cascade default) | 42 | 0.258 | 101s |
| lean_void_combo (cascade+multi_pole) | 42 | 0.253 | 108s |
| **cascade_only** | **47** | **0.283** | 113s |

Cascade alone raised part count ~12% on this void fixture; rim/kiss not worse beyond noise.
NMS kept off in shipped combo until eps retuned.

## Harness

```bash
PYTHONPATH=. python scripts/benchmark_propose.py --ablate --out docs/propose_benchmark_results.txt
PYTHONPATH=. python scripts/benchmark_pipeline.py --force --cases donut_void --seeds 0 \
  --propose shipped lean_void_combo cascade_only nms_only conflict_degree_rank
```

`void_leak` now includes `cascade=…`, `nms=kept/dropped`, and `prop_accept name:e/p/n/r`.

## Flatten

- Shared `ranking.finalize_propositions` used by primary + edge finalize
- Multi-pole API in `void_topology`; `free_pocket_anchors` delegates for large pockets
- Pool NMS/conflict hook in `propose_coords_from_candidates` (reserve-safe merge after)
- `apply_proposer_pool_scales` generalizes beyond neighbor_slide
- `ProposeFeedbackState` can EMA from refine survival when `use_ema_proposer_scales`
