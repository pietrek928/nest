# Propose benchmark (gap-fitting)

Isolated benchmark for the **propose** step (no full nest loop). Compares config presets and per-proposer ablation on triangle-board fixtures.

## Run

```bash
PYTHONPATH=. python scripts/benchmark_propose.py --seeds 0 1 2 3 4 5 6 7 8 9
PYTHONPATH=. python scripts/benchmark_propose_ablation.py --seeds 0 1 2 3 4 5 6 7 8 9
```

Outputs:

- [`docs/propose_benchmark_results.txt`](propose_benchmark_results.txt) — preset comparison
- [`docs/propose_ablation_results.txt`](propose_ablation_results.txt) — one proposer at a time

## Scenarios

| Scenario | Setup |
|----------|--------|
| `empty_base` | First part on empty sheet (border focus) |
| `partial_pack` | One rect placed; propose triangle into pocket |
| `two_clusters` | Two separated rects; propose triangle between them |
| `hole_board` | Rectangular sheet; rect on floor (hole-seeking smoke) |

## Metrics

| Metric | Meaning |
|--------|---------|
| `contact_min` | Min distance from placed part to obstacle union (lower = tighter kiss) |
| `clearance_min` | C++ guidance clearance (deep void signal) |
| `kiss_fraction` | Share of final proposals within `min_dist` + epsilon of obstacles |
| `raw_pool_size` | Candidates before trim/rank |
| `graph+` | Graph nodes when proposals are added vs random-only |

## Calibration memo (2026-05-28, seeds 0–9)

Preset comparison (see [`propose_benchmark_results.txt`](propose_benchmark_results.txt)):

| preset | partial_pack contact_min | two_clusters contact_min | hole_board contact_min |
|--------|--------------------------|---------------------------|-------------------------|
| **contact_rank** / **ribbon_heavy** | **0.003–0.005** | **0.003–0.004** | 0.003–0.004 |
| shipped / shipped_no_phase2 | 0.061 | 0.003 | 0.003 |
| shipped_no_guidance_cast | 0.033 | 0.003 | 0.003 |
| clearance_rank | 0.159 | 0.268 | 0.166 |

After calibration, **shipped** matches **shipped_no_phase2** (phase-2 Shapely proposers default off). `partial_pack` contact_min ~0.06 is limited by combined trim/rank, not phase-2 flags. `contact_rank` and `ribbon_heavy` still win on kiss metrics for packed fixtures; shipped keeps contact hybrid + full seed mix for graph diversity.

Per-proposer ablation (partial_pack, seeds 0–9) — see [`propose_ablation_results.txt`](propose_ablation_results.txt):

| proposer | contact_min | Notes |
|----------|-------------|--------|
| **group_fit** | **0.003** | Best kiss; keep `use_group_edge_seeds` |
| **neighbor_slide** | 0.003 | Good alone; dilutes combined pool → **default off** |
| ribbon_free / raycasting / voronoi | ~0.007–0.008 | Cheap gap fillers; keep |
| axis_push / perimeter / nfp / bottom_left | 0.15–0.30 | Loose alone; **defaults off** |
| guidance_propositions (alone) | ~0.14 | Needs upstream seeds; keep in combined pipeline |

**Shipped default changes:**

- `use_neighbor_slide`, `use_axis_push`, `use_bottom_left`, `use_nfp_vertices` → **False** (enable per job if needed)
- `use_group_edge_seeds`, `use_ribbon_seeds`, `use_guidance_propositions`, contact hybrid ranking → **unchanged**
- `guidance_enable_grid` → **False** (corner casts; optional for irregular voids)

## Shipped defaults (`ProposeConfig`)

See [`nest_graph/config.py`](../nest_graph/config.py). Key fields:

| Field | Value | Rationale |
|-------|-------|-----------|
| `candidate_pool` | `48` | Room before contact/clearance trim |
| `max_proposals` | `24` | Seeds per group to graph |
| `use_contact_ranking` | `True` | Packed: rank by kiss to focal/group |
| `use_contact_clearance_hybrid` | `True` | Keep pocket poses via clearance blend |
| `use_ribbon_seeds` | `True` | Strong on partial_pack ablation |
| `use_group_edge_seeds` | `True` | Best contact_min in ablation |
| `use_guidance_propositions` | `True` | Cast expansion on structured seeds |
| `guidance_enable_grid` | `False` | Corner exploration optional |
| Phase-2 Shapely proposers | **off** | See calibration memo |

## Guidance cast (`guide.h`)

After C++ changes, rebuild `geometry` and re-run:

```bash
cmake --build build --target geometry
PYTHONPATH=. python scripts/benchmark_guidance_flow.py --seeds 0 1 2 3 4
```

Tune `guidance_use_corner_alignment`, `guidance_enable_grid`, `guidance_use_tight_packing` from pipeline scores in `docs/guidance_flow_benchmark.txt`.

## Tests

```bash
.venv/bin/pytest tests/test_geometry_guide.py tests/test_guidance_propositions.py tests/test_propose_gap.py tests/test_propose_benchmark_smoke.py -q
```
