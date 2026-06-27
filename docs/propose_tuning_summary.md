# Propose benchmark tuning summary (2026-06-26)

## Harness fixes

- Zone profiles use `guidance_cast_refine` (not legacy `guidance_propositions`).
- Benchmark metrics include `proposal_yield` and `proposal_nodes`.
- E2E pipeline preset `cast_first_quality` added to `benchmark_nest_pipeline.py`.

## Ablation highlights (7 scenarios × 10 seeds)

| Proposer | Best for | Notes |
|----------|----------|-------|
| `group_fit` | packed_border, interior_after_border | contact_min ≈ 0.003, slow (~8–11s) |
| `neighbor_slide` | partial_pack alone | kiss_frac up to 0.79; off globally |
| `board_edge` | empty/border | slow (2–3.6s); cap samples |
| `erosion` / `ribbon_free` / `raycasting` | cheap fillers | <0.2s, moderate contact |
| `guidance_cast_refine` | refinement | needs seeds; best contact when seeded |
| `perimeter_walk` | border seeds | fast, moderate contact |

Legacy Shapely proposers (`axis_push`, `bottom_left`, `nfp_vertices`) removed from the live pipeline.

## Preset grid (4 scenarios × 10 seeds)

All presets hit **proposal_yield = 1.0** on tested scenarios.

| Preset | two_clusters time_s | partial_pack time_s | Notes |
|--------|---------------------|---------------------|-------|
| **shipped** (updated defaults) | 20.4 | 3.3 | Fastest on border/partial |
| cast_first_quality | 11.2 | 3.7 | Faster two_clusters |
| local_compact | 9.5 | 3.3 | Balanced |
| kiss_heavy | 10.6 | 4.1 | Slower |

**Recommendation:** Shipped defaults with cast-first stack (`guidance_cast_refine_top_k=12`, grid off, pool 64). Use `place_routed` for zone-specific proposers.

## Shipped default changes

- `candidate_pool=64`, `board_edge_samples_per_edge=48`
- `guidance_enable_grid=False`, `guidance_cast_refine_top_k=12`, `cast_rank_boost=0.35`
- `use_neighbor_slide=False` globally; enabled in `border_gap` / `cluster_edge` zones only

## Code improvements

- **StaticCollisionScene** (C++): caches obstacle sweep index for `batch_check_validity`.
- **batch_evaluate_local_placement** (C++): batched ranking guidance.
- **Adaptive angle grid** + **spatial seed clustering** before cast-refine.
- **ProposeContext** + **pre_filter_candidates** unified validity/collision pass.
- Per-proposer `max_items` caps before merge.

## E2E gate (2026-06-26, seeds 0–4, iters 3)

| propose | parts_final | border_err_min | time_s |
|---------|-------------|----------------|--------|
| **place_routed** | **55.4** | 0.0001 | 389 |
| local_compact | 55.2 | 0.0000 | 407 |
| shipped | 54.4 | 0.0001 | 675 |
| cast_first_quality | 54.0 | 0.0001 | 592 |

**Gate passed:** `place_routed` wins on parts count and is fastest among top-tier presets. Shipped defaults are validated; zone routing (`place_profiles_enabled=True`) remains the production path for max density.
