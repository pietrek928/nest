# Local placement movement benchmark

Unified benchmark for cast knobs, post-rank `cast_squeeze`, geometric walkers, and combined stacks.

## Run

```bash
# Quick screen (9 presets, 3 scenarios, seeds 0–2)
uv run python scripts/benchmark_local_placement.py --quick

# Full matrix (14 presets, 5 scenarios, seeds 0–9)
uv run python scripts/benchmark_local_placement.py --full

# End-to-end density gate
uv run python scripts/benchmark_nest_pipeline.py \
  --propose density_heavy local_compact shipped \
  --dfs-modes merged_loose_tight --seeds 0 1 2 3 4 --iters 3
```

Outputs: [`local_placement_benchmark.txt`](local_placement_benchmark.txt), [`local_placement_benchmark.json`](local_placement_benchmark.json).

## Scenarios

| Scenario | Setup |
|----------|--------|
| `empty_base` | First part on empty triangle sheet |
| `partial_pack` | One rect placed; propose triangle into pocket |
| `two_clusters` | Two rects; propose into gap |
| `hole_board` | Rectangular sheet with floor rect |
| `packed_border` | Three parts hugging outline edges |

## Metrics

| Metric | Meaning |
|--------|---------|
| `contact_min` | Min distance to obstacle union (lower = tighter kiss) |
| `kiss_fraction` | Share of proposals within standoff margin |
| `squeeze_delta` | Mean contact improvement on top-K after `cast_squeeze` |
| `squeeze_moved` | Count of top-K coords moved by squeeze |
| `composite` | `-3*contact_min + 2*kiss + 0.5*squeeze_delta - 0.01*time` |

Scenario weights for ranking: `partial_pack` and `two_clusters` ×2.

## Quick pass findings (seeds 0–2)

Presets were **tied on kiss/contact** for packed scenarios; differences were mostly **runtime**:

| Preset | Notes |
|--------|--------|
| `compact_cast` / `compact_full` / `neighbor_off` | ~35% faster than `shipped` on `partial_pack` |
| `squeeze_8_double` | Slower with no measurable squeeze delta on fixtures |
| `tight_corner` | Slowest on `two_clusters` (extra cast work) |
| `guidance_walk_on` | No contact gain vs shipped on quick screen |

**Synthesis:** `local_compact` preset — tight+corner cast, grid off, `cast_squeeze_top_k=8`, `cast_squeeze_passes=1`, `use_neighbor_slide=False`, `use_guidance_walk=False` (walk only when packed via pipeline guard).

See [`ProposeConfig.local_compact_profile()`](../nest_graph/config.py) and nest-pipeline preset `local_compact`.

## End-to-end gate (seeds 0–2, 2 iters, `merged_loose_tight`)

| propose | parts_final | border_err_min | time_s |
|---------|-------------|----------------|--------|
| `density_heavy` | 50.3 | 0.0000 | 138.4 |
| `local_compact` | 50.0 | 0.0000 | 119.0 |

`local_compact` matches border kiss and score sum (+15%), with ~14% faster propose/build per iter. Part count is within seed noise of `density_heavy`; shipped defaults unchanged — use `local_compact` when tuning for compaction speed.

## Synthesized pipeline tiers

| Tier | `local_compact` choice |
|------|------------------------|
| Seeds | Shipped mix (`group_fit`, `ribbon`, `raycast`, `board_edge`) |
| Cast expansion | `guidance_propositions`, tight + corner, grid off |
| Rank | `contact_hybrid` (+ `rule_hybrid` when rules passed) |
| Post-rank compact | `cast_squeeze_top_k=8`, single pass |

## Literature mapping

- **Compression phase** (Sparrow 2025): post-rank `cast_squeeze`
- **Overlap-minimization local search** (Egeblad EJOR 2007): C++ cast + neighbor slide
- **NFP edge sliding**: `group_fit`, `perimeter_walk` seeds (unchanged)
