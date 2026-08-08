# Propose combo E2E results (2026-08-08)

Smoke: `donut_void_h4.0x4.0` seed=0, dfs=`merged_loose_tight`.

| Propose | Parts | Area | KissO | Time | Notes |
|---------|-------|------|-------|------|-------|
| shipped (pre-default flip) | 42 | 0.258 | 0.43 | 101s | cascade off during this run |
| lean_void_combo | 42 | 0.253 | 0.36 | 108s | cascade on, NMS off |
| **cascade_only** | **47** | **0.283** | 0.40 | 113s | best density |

Decision: ship `propose_cascade_short_circuit=True` by default. Keep `use_pose_nms=False` (isolated propose final_count 16→13). Keep conflict-degree and EMA off pending wider matrix.

Isolated propose (`border_then_fill_s13`, seeds 0–1): see `docs/propose_benchmark_results.txt`.
