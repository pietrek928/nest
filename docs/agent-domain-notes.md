# Agent domain notes (archive)

Historical research answers, open questions, and patch backlogs that used to live in
root `AGENTS.md`. **Not standing instructions** — agents should follow root
[AGENTS.md](../AGENTS.md) first and open this file only when digging into geometry,
void-fill, or nanobind copy-tax work.

Last split from root AGENTS.md: 2026-08-10.

---

## OOS void-fill protocol (reference)

### Proposer telemetry

- Keys: `round(x,y,θ, 4)` — must match `build_graph._transform_row_key`.
- Emit order: `pocket_fit` → `cluster_copy` → … → `raycast` → `selection_expand` → `history_expand`.
- Funnel: emit → pool → nest → refine per name; `void_leak` `prop_accept e/p/n/r`.
- Gate: no zone/ranking OOS without `refine_by_proposer` visibility.

### OOS-1 native void seek

- Exterior large free (`area/part > late_border_void_override_ratio`) → `void_seek` even if `packed_near_border`.
- Keep corridor / narrow mouth / `first_pass_border` exceptions.

### OOS-4 void-aware ranking

- Plumb `void_pole` (`pt_push` is NOT the pole under `void_seek` by default).
- `pole_bonus = max(0, 1-dist/sheet_diag) * (part_area/sheet.area) * void_rank_pole_weight`
- Not flat void MIS boost (tiny debris pathology). Validate area↑ + `refine_by_proposer`↑.

### P3 / B1 / B2

- P3: re-add nest-void idxs only if `graph.collisions`-clear vs refine.
  Telemetry: `void_leak` `pin_candidates` / `pin_added` / `pin_blocked_collision` / `pin_ms`.
- **B1:** bind `greedy_weighted_mis` / fold pin into finalize with `locked_indices`
  only if `pin_added≥1` **and** `pin_ms>30`.
- **B2 blocked:** no void-aware nest/refine MIS / external scores until B0/B1 exhausted.

---

## Geometry / hotpath research (answered)

| Question | Answer |
|----------|--------|
| `from_shapely` hotspot after `native_geoms`? | No — ingest + NestState rebuild only; expect ≪1% wall. |
| Share of `polish_se2_part` None? | ~70–80% mid-pack OK (jammed). 100% = bug. Free-space unit cases must move. |
| Keep edge/geo `buffer(-min_dist)`? | **No** — raw ring + batch `valid_at`. |
| Keep ribbon annuli? | **Yes** — frontier focus. |
| Kiss-strict vs parts_final? | Needs C++ EPA packing filter (zero-depth kiss ≠ collision); then stay within ~±0.5–2% of post-kiss baseline. |
| `outline_coverage` after probe reuse? | Samples + GJK vs `native_geoms`; not a boolean hotspot. |
| Corner gravity compaction off? | **Yes** — void-pole `local_se2` floater polish is default-on; propose `border_focus` ≠ corner gravity. |

### Geometry numerics & corner patches (C++ audit)

| Question | Status / note |
|----------|----------------|
| Unify GJK/EPA/cast default eps with touch/packing? | **Resolved.** Separate Geometric vs Application tolerances. GJK internal ~1e-8; packing uses `NEST_PENETRATION_EPS_SQ` (1e-12); touch uses `nest_touch_eps_*`. |
| One meaning of `intersect` near contact? | **Action.** Replace packing bools with `ContactState` (`Disjoint`, `Touch`, `Penetrating`, `Contained`). |
| Delete `edge_mid_inside`? | **Partial.** EPA `{depth=0}` → `Touch`; promote Touch→`Penetrating` via `contact_edge_mid_interior_witness` when needed. Pure boundary kiss stays `Touch`. |
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
| NestState live cache mutate? | **Rebuild-only.** |
| `local_se2` coarse/fine? | **max_t only**; single cast path in C++ polish. |
| Pocket `buffer(-min_dist)` vs SEARCH HALOS? | **Drop** pocket seed erosion; raw ring + `valid_at`. |

### Logic duplications (consolidate — no new dual paths)

| Question | Status / note |
|----------|----------------|
| Packing re-implements narrow-phase? | **Action.** `evaluate_narrow_phase`; packing must call it. |
| Warm-index after `nA>nB`? | **Action.** Router un-swaps before return. |
| `cached_narrow_phase_intersect`? | **Delete.** |
| `polish_se2` double cast? | **Action.** One TOI/normal → tangent → Scene once. |
| Clearance SoT? | **Done.** |
| Board adj twice? | **Done.** One `is_board_adj`; one `cluster_contact` (`2·gap`). |
| Standoff vs solid distance for board adj? | **Standoff** (edge-to-edge). |
| `_as_geometry` copy-paste? | **Done.** `placement_common.py`. |
| `footprint_inside` vs `fully_inside`? | **Done.** Hot-path = void solids. |
| Obstacle list forks / contain aliases? | **Done.** `placement_obstacles`. |
| Merge rim / obstacle / guide tangents? | **No.** Document normal sourcing only. |
| side_pack zone whitelist in pipeline? | **Deleted.** Permission = `ZONE_PROPOSERS`. |
| Motif stamp propose vs repack? | **Keep both.** Share anchors + `dedupe_anchors`. |

### Propose / void-fill review (answered)

| Question | Answer |
|----------|--------|
| Repack stamp vs propose leader-follower? | **Keep both.** Share anchors + dedupe only. |
| Unify densify iv/pole with props telem? | **Radius only** via `void_pole_near_radius`. |
| Round-2 vs round-4 funnel break? | **No.** Funnel uses round-4. |
| Filter side_pack at emit? | **No.** Raw over-emit; packing filter at collect end is SoT. |
| Zone policy location? | **Permission** = `ZONE_PROPOSERS`. **Staging** = `packed_n` / void. |
| Keep cloud in zone set? | **Yes**, densify-only emit. |
| Native void densify accept? | **Same lex as hijack** (`void_yield_gain` → `void_pole_clear`). |
| Does repack emit side_pack? | Pass `cascade_zone=zone`. |
| Wall-fill on all hard zones? | **void_seek only.** |

### Ranking / selection hybrid (answered)

| Question | Answer |
|----------|--------|
| Dual-edge / contact_hybrid for propose? | **Yes** — C++ `batch_rank_local_placements`. |
| Same hybrid for DFS/nest? | **Yes** — `batch_score_placed_contact_hybrid`. |
| Rewrite RBF kernels? | **No** — rewrite selection pipeline + MIS scored entry. |
| Shapely hull in prod ranking? | **No** — C++ monotone chain; tests may oracle. |
| Cap clearance / harmonic edge_free? | **Yes**. |
| Border boost with hybrid? | **Skip** when selection geom on. |
| Can DFS reuse the signed propose score? | **No** — selection uses non-negative `quality`. |
| Geometry inside elem_graph? | **No** headers merge. |
| Void attractor with nest_by_scores? | **Skip** when selection geom on. |

### Propose / void-fill research (open)

| Question | Why ask |
|----------|---------|
| Rim saturated (`e ≫ p ≈ 0`): shift budget from sheet-snap to interior colonization? | Structural mid-pack ceiling |
| Densify replaces `arr` but unions pre-densify `proposer_keys` — intersect with final array? | Funnel accuracy |
| Two sterile ladders (densify zones vs graph `sterile_pack`) — one predicate? | Consolidation candidate |
| Weighted stratify: largest-remainder vs min-2 quotas when Σquotas > top_n? | Anti-crowd fidelity |
| Repack-internal propose with `cascade_zone=zone` — net gain or churn? | Watch bench |
| Densify clearance floor uses ranking clearance — rename or keep? | Naming vs SoT |
| Return natives from `make_polygon_graph`? | Avoid rebuild via transforms |
| Native hull verts for bay difference? | More Shapely removal |
| Document ranking ↔ guidance sweep? | Copy tax |
| `selection_geom_weight` vs rule scale? | Mid-pack balance |
| Export C++ tightness for first_pass/repack? | Kill G27 dual |

Glossary: `side_pack` permitted by zone (incl. `cluster_edge`), staged by pack count / void; raw emit; `free_space_cloud` densify recovery with funnel keys.

### Nanobind bindings (copy tax)

| Question | Status / note |
|----------|----------------|
| Are list APIs zero-copy? | **No.** Holder + solids deep-copy (often 2×); Scene/`cast_slide` often 3×. |
| Single-Geometry queries? | **OK** — const ref → `.solid`. |
| `cast_slide` / polish obstacle pack? | **Action.** Reuse Scene / pointer span. |
| `batch_evaluate_local_placement`? | **Action.** Reuse one polys buffer. |
| Pairwise `intersects` / `min_distance`? | **Action.** No owned 2-element vectors. |
| `apply_transform` double clone + RNG? | **Action.** Single-pass SE2 clone. |
| Pointer scene + `keep_alive`? | Later: after span APIs. |
| Mid-loop `from_shapely` via bindings? | **Forbidden** on hot propose. |
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
4. **Clearance / board-adj / as_geometry consolidation** (largely done — verify).
5. **Decomp-parity + nested/shallow units**.
6. **Nanobind zero-copy** — pointer/span list APIs; Scene.build move; no cast/batch re-pack.

## R8 cross-cutting decisions

| Topic | Decision |
|-------|----------|
| **r8-placement-query** | `ProposeContext` stays propose-only; selection edits use `SelectionEditCtx`. |
| **r8-legacy-modules** | Keep `placements_geo`, `pose_diversity`, `feedback` — all live. |
| **r8-nanobind-propose** | Propose-side list rebuild in clear/polish stays until span APIs; no second Python cache. |
