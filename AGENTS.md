# Agent instructions (nest_graph)

## Python

- **Do not use `from __future__ import …`** — not needed on this project’s Python (3.12+). Never add `__future__` imports (including `annotations`).
- If you remove `__future__` imports that were added by mistake, do not reintroduce them.
- Prefer normal type hints (`X | None`, quoted forward refs like `"BuildGraphConfig"` only when required).
- Import only on file top unless u have a good reason

## Native extensions

- After C++ changes, rebuild: `uv pip install -e .` (native sources are in `tool.uv.cache-keys`) or `cmake --build build --target geometry elem_graph`.
- C++ unit tests: `cmake -S . -B build -DNEST_GRAPH_BUILD_TESTS=ON` then `geometry_cpp_tests` / `elem_graph_cpp_tests`.

### C++ style

- **Do not add custom C++ namespaces** (no anonymous `namespace { … }`, no named helper namespaces like `elem_graph_test`). Keep helpers at file scope in `.cc` files or as `inline` functions in headers so duplicate logic stays visible.
- **Namespace aliases are fine** for third-party APIs, e.g. `namespace nb = nanobind;` in bindings.
- **Do not add `static` on functions** unless there is a concrete reason (e.g. required internal linkage to avoid a duplicate symbol). Prefer shared `inline` helpers in `internal/internal.h` for one-liners used in multiple translation units.

## Nesting / collisions

- **Output** must be collision-free (independent set on the overlap graph). Transient overlaps during DFS search are OK; `refine_selection` and `finalize_selection` must not return overlapping sets to Python.
- Do not add `NEST_DFS_MIN_COLLISIONS_*` or similar env vars; loose caps are internal C++ constants in `refine_dfs.cc`.
- Default pipeline: `nest_by_graph` → `refine_selection` (loose then tight) → `finalize_selection` (repair, then optimal weighted MIS on small overlap components).
- `make_polygon_graph` keeps all board-valid nodes and records overlaps as graph edges.
- Board validity and propose guidance use one obstacle list (packed parts + sheet void holes) via `PlacementScene` / `evaluate_local_placement`. Outline config: `board_coords`; sheet = bbox outer + auto corner voids + optional `board_holes`.

## Tests

- Python: `tests/` (integration, bindings, build loop).
- C++: `nest_graph/geometry/tests/`, `nest_graph/elem_graph/tests/`.

## Git

- Do not commit unless the user asks.

## Glossary (propose / OOS)

- **`border_focus`** — propose zone favoring sheet rim / outline kiss. **Not** post-refine gravity compaction (`enable_gravity_compaction`).
- **`void_seek`** — OOS path when free area/part is large; may override packed-near-border.
- **`packed_near_border`** — local packed layout hugs the rim; interacts with void override hysteresis.
- **`valid_at` / guidance validity** — propose clearance on `ProposeGeometry`. **Never OR** with packing independence.
- **`is_pose_clear`** — selection/polish clearance SoT (board `fully_inside` + Scene margin).
- **`fully_inside`** — board containment SoT (prefer over any footprint alias).
- **Packing collide** — `Geometry.intersects` / Penetrating ContactState only.

## OOS Void-Fill Execution Protocol

### 0. Proposer Telemetry (DO FIRST)

- Keys: `round(x,y,θ, 4)` — must match `build_graph._transform_row_key`.
- Emit order: `pocket_fit` → `cluster_copy` → … → `raycast` → `selection_expand` → `history_expand` (`cluster_copy` before sweepers; expand proposers last).
- Funnel: emit → pool → nest → refine per name; `void_leak` `prop_accept e/p/n/r`.
- Gate: no zone/ranking OOS without `refine_by_proposer` visibility.

### 1. OOS-1 Native Void Seek

- Exterior large free (`area/part > late_border_void_override_ratio`) → `void_seek` even if `packed_near_border`.
- Keep corridor / narrow mouth / `first_pass_border` exceptions.

### 2. OOS-4 Void-Aware Ranking

- Plumb `void_pole` (`pt_push` is NOT the pole under `void_seek` by default).
- `pole_bonus = max(0, 1-dist/sheet_diag) * (part_area/sheet.area) * void_rank_pole_weight`
- Not flat void MIS boost (tiny debris pathology). Validate area↑ + `refine_by_proposer`↑.

### 3. Do Not Touch / Last Resort

- Gravity off (evacuates void). PSO off unless `props_pole≈0` after 1+4.
  **Gravity off** means post-refine gravity compaction (`enable_gravity_compaction=False`),
  not propose `border_focus` guidance gravity.
- P3: re-add nest-void idxs only if `graph.collisions`-clear vs refine.
  Telemetry: `void_leak` `pin_candidates` / `pin_added` / `pin_blocked_collision` / `pin_ms`.
- **B1 gate:** bind `greedy_weighted_mis` / fold pin into finalize with `locked_indices`
  only if `pin_added≥1` **and** `pin_ms>30`. Post-hoc collision-clear append does not
  need finalize locks.
- **B2 blocked:** no void-aware nest/refine MIS / external scores until B0/B1 exhausted;
  rim drop >2% = fail.

## Performance & Geometry Strictness Rules

1. **NO PYTHON FALLBACKS:** `polish_se2_part` and `StaticCollisionScene` are the sole sources of truth. Do not write Python grids or Shapely fallbacks if the C++ fails. Fix the C++ (contact-normal/tangent sliding, step size, or penetration limits).
2. **GEOMETRY IS CANONICAL:** `nest_graph.geometry.Geometry` is the primary type for hot paths. `NestState` holds a lazy `native_geoms` cache. Do NOT loop `Geometry.from_shapely()` in propose emits, `_border_tightness_cost`, or coverage calculations.
3. **KISS / FP:** Hard packing collision (graph / independence) iff C++ EPA penetration depth is meaningful (`nest_packing_penetration_eps_sq` ≈ 1e-12 ⇒ ~1e-6 depth; zero-depth / inconclusive EPA after GJK touch ⇒ miss). Clearance `min_dist` margins are separate. Python uses bare `Geometry.intersects` (aligned with the C++ filter). Do not soft-filter in Python. Boundary kiss is not Shapely-identical (Shapely DE-9IM counts touch as intersect).
4. **MARGIN AS DISTANCE (contact):** Never use `.buffer(gap).intersects()` for cluster/board contact. Use `distance() <= threshold`.
   - Cluster contact: `dist <= 2 * gap`.
   - Board adjacency: `dist <= min_dist + 2 * gap`.
   - Ribbon annuli / free-space morph close may still buffer (region construction, not contact graphs).
5. **SEARCH HALOS:** Prefer raw ring samples + batch `valid_at` over `buffer(-min_dist)` seed erosion. Keep ribbon annuli for frontier focus.
6. **OUT OF SCOPE:**
   - Do not add Clipper2, libnest2d, or C++ Voronoi.
   - Do not enable global gravity compaction (destroys hypotenuse / mid-pack voids).
   - Propose `border_focus` guidance gravity is not compaction.

## Geometry / hotpath research (answered)

| Question | Answer |
|----------|--------|
| `from_shapely` hotspot after `native_geoms`? | No — ingest + NestState rebuild only; expect ≪1% wall. |
| Share of `polish_se2_part` None? | ~70–80% mid-pack OK (jammed). 100% = bug. Free-space unit cases must move. |
| Keep edge/geo `buffer(-min_dist)`? | **No** — raw ring + batch `valid_at`. |
| Keep ribbon annuli? | **Yes** — frontier focus. |
| Kiss-strict vs parts_final? | Needs C++ EPA packing filter (zero-depth kiss ≠ collision); then stay within ~±0.5–2% of post-kiss baseline. |
| `outline_coverage` after probe reuse? | Samples + GJK vs `native_geoms`; not a boolean hotspot. |
| Gravity compaction off? | **Yes** — propose `border_focus` ≠ global compaction. |

### Geometry numerics & corner patches (C++ audit — resolved / action)

| Question | Status / note |
|----------|----------------|
| Unify GJK/EPA/cast default eps with touch/packing? | **Resolved.** Separate Geometric vs Application tolerances. GJK internal ~1e-8; packing uses `NEST_PENETRATION_EPS_SQ` (1e-12); touch uses `nest_touch_eps_*`. |
| One meaning of `intersect` near contact? | **Action.** Replace packing bools with `ContactState` (`Disjoint`, `Touch`, `Penetrating`, `Contained`). |
| Delete `edge_mid_inside`? | **Partial.** EPA `{depth=0}` → `Touch`; solid overlaps often also get EPA depth 0 on line-string parts, so promote Touch→`Penetrating` via solid-centroid edge-mid witness (`contact_edge_mid_interior_witness`). Pure boundary kiss stays `Touch`. |
| `kissed_pairs` if strict-interior contain? | **Delete** after contain witness is strict interior; until then Touch must not promote. |
| EPA `{0,true}` / contain op_limit→true? | **Resolved.** Map to `Touch` / never `Penetrating`. |
| 90° decomp vs kiss? | **Do not change angle.** Handle artificial verts via `ContactState` depth; CI tests must use decomp. |
| Tests `polygon_from_quad` vs prod decomp? | **Action.** Packing/kiss fixtures via `decompose_complex_polygon`. |
| Cast vs StaticCollisionScene in polish? | **Scene is authority.** On Scene reject: back off ~1e-6 along normal or `None`. |
| Gradient threshold 24? | Keep for GJK vertex routing; unrelated to `snap_pose` axis 24. |
| `known_overlap`? | **Delete.** |
| Closed ring exact float eq? | **Action.** Closed if endpoint distance `< 1e-12`. |
| Guide length floors vs touch eps? | Keep separate (guidance ≠ packing). |
| Packing eps 1e-12? | Yes until E2E re-baseline. |
| NestState live cache mutate? | **Rebuild-only.** No in-place `polys` mutate; invalidate only if mutation is introduced. |
| `local_se2` coarse/fine? | **max_t only**; single cast path in C++ polish. |
| Pocket `buffer(-min_dist)` vs SEARCH HALOS? | **Drop** pocket seed erosion; raw ring + `valid_at`. |

### Logic duplications (consolidate — no new dual paths)

| Question | Status / note |
|----------|----------------|
| Packing re-implements narrow-phase? | **Action.** `evaluate_narrow_phase`; packing must call it. |
| Warm-index after `nA>nB`? | **Action.** Router un-swaps before return. |
| `cached_narrow_phase_intersect`? | **Delete.** |
| `polish_se2` double cast? | **Action.** One TOI/normal → tangent → Scene once. |
| Clearance SoT? | **Action.** One `is_pose_clear` via `StaticCollisionScene`; guidance `valid_at` separate. |
| Board adj twice? | **Action.** One `is_board_adj` (standoff); one `cluster_contact` (`2·gap`); delete `_contact_neighbors`. |
| Standoff vs solid distance for board adj? | **Standoff** (edge-to-edge). |
| `_as_geometry` copy-paste? | **Action.** Single helper in `placement_common.py`. |
| `footprint_inside` vs `fully_inside`? | **Deprecate** `footprint_inside`; use `fully_inside`. |
| Merge rim / obstacle / guide tangents? | **No.** Document normal sourcing only. |

### Nanobind bindings (copy tax — action)

| Question | Status / note |
|----------|----------------|
| Are list APIs zero-copy? | **No.** `vector<GeometryHolder>` + `solids_from_holders` deep-copies solids (often 2×); Scene.build / `cast_slide` often 3×. |
| Single-Geometry queries? | **OK** — const ref → `.solid` (Scene, snap, distance pair board). |
| `cast_slide` / polish obstacle pack? | **Action.** Stop re-packing active+obstacles every cast; reuse Scene / pointer span. |
| `batch_evaluate_local_placement`? | **Action.** Reuse one polys buffer; do not recline obstacles per pose. |
| Pairwise `intersects` / `min_distance`? | **Action.** Narrow path without owned 2-element vectors. |
| `apply_transform` double clone + RNG? | **Action.** Single-pass SE2 clone; cheap/lazy holder RNG on transform returns. |
| Pointer scene + `keep_alive`? | Later: only after span APIs; today move-into Scene is enough. |
| Mid-loop `from_shapely` via bindings? | **Forbidden** on hot propose; ingest / NestState rebuild only. |
| `elem_graph` bindings? | Low solid-copy risk; leave unless profiled. |

### Research topics (before new stacks)

Answered / locked:
1. Geometric vs application eps — do not conflate.
2. Decomp 90° stays; ContactState absorbs kiss pathology.
3. Scene authority over cast for polish clearance.
4. Native SE2; no Python grid; packing via `Penetrating` only.
5. Cluster `2·gap`; board standoff `min_dist+2·gap`.
6. Ribbon keep; drop seed erosion (edge/geo/pocket).

Do next (patches) before new geometry stacks:
1. **ContactState** end-to-end (kill bool packing + mid-nudge + kissed_pairs).
2. **Unified narrow-phase router** (swap/warm/24 once).
3. **polish_se2 single-cast + Scene backoff**.
4. **Clearance / board-adj / as_geometry consolidation**.
5. **Decomp-parity + nested/shallow units**.
6. **Nanobind zero-copy** — pointer/span list APIs; Scene.build move; no cast/batch re-pack; pairwise without temp vectors.

## Propose module map (post-flatten)

| Module | Owns |
|--------|------|
| `propose/types.py` | `ProposeContext` / `PackedProposeExtras` / `PocketStats` + `make_propose_context`. `query_context.py` is a re-export shim only. |
| `propose/pipeline.py` | `_collect_candidates` staged emit (`_collect_pocket_candidates` → `_collect_builder_candidates` / `_collect_explorer_candidates` → `_collect_cast_refine_candidates`), ranking handoff, `proposed_transforms_for_groups`. |
| `propose/first_pass_border.py` | First-pass border stack: `first_pass_border_coords`, `border_saturation_transform_batch`, `guidance_border_refine`, `first_pass_interior_fill`, `sequential_border_augment`, `border_pack_graph`. |
| `propose/void_selection.py` | Void-aware score boosts, P3 pin repair, `format_prop_accept` funnel line, free/pole counters. |
| `propose/context.py` | Zone classification, free-space analysis, `late_border_saturation_info`. |
| `propose/selection_edit.py` | `SelectionEditCtx` for `local_se2` / `compaction` / `cluster_repack`. |

`build_graph` keeps only graph/selection concerns (`make_polygon_graph`, rule sets,
DFS refine, `_first_pass_layered_selection`) and re-exports the moved private names
for `scripts/nesting_evaluator.py` and existing tests.

**Emit order is static, not a registry.** `_collect_candidates(ctx, extras, mode=…)`
selects only the stage order: `"cascade"` = snipers → builders → explorers,
`"free"` = snipers → explorers → builders. Poles → `pocket_fit` → `cluster_copy`
always precede any sweeper.

## R8 cross-cutting decisions

| Topic | Decision / evidence |
|-------|---------------------|
| **r8-placement-query** | `ProposeContext` stays **propose-only**. It carries propose-shaped state (`ProposeGeometry`, `pt_push`, `border_focus_override`, `void_pole`, proposer enable/count sinks) that selection and graph paths never read; selection edits use `SelectionEditCtx` instead. Do not widen `ProposeContext` into a general placement-query object, and do not import it from `build_graph` / `elem_graph` paths. |
| **r8-legacy-modules** | **Keep all three.** `placements_geo` is live (`raycasting` / `voronoi` are in `_CASCADE_EXPLORERS` and wired in `_collect_explorer_candidates`). `pose_diversity` is live (`apply_pose_nms` + `apply_conflict_degree_penalty` behind `use_pose_nms` / `use_conflict_degree_rank`, reported via `nms_kept` / `nms_dropped`). `feedback` is live (`ProposeFeedbackState` bumps `obstacle_nearest_k` on low proposal yield; covered by `tests/test_propose_place_classifier.py`). No quarantine needed. |
| **r8-nanobind-propose** | Remaining propose-side list copy is the `list[Geometry]` obstacle pack rebuilt per candidate in `is_pose_clear` / `clear_of_geoms` and per-cast in `polish_se2_part`. It is a Python-list rebuild plus one `vector<GeometryHolder>` copy per call — **not** a `from_shapely` re-decomp, so it stays until the span/pointer list APIs in "Nanobind zero-copy" land. Do not add a second propose-side caching layer in Python to work around it. |
