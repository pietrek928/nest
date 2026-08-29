# Agent domain notes (archive)

Historical research answers, open questions, and patch backlogs that used to live in
root `AGENTS.md`. **Not standing instructions** — agents should follow root
[AGENTS.md](../AGENTS.md) first and open this file only when digging into geometry,
void-fill, or nanobind copy-tax work.

Last split from root AGENTS.md: 2026-08-10.

---

## OOS void-fill protocol (reference)

### Proposer telemetry

- Keys: `round(x,y,θ, 4)` — must match `propose/void_selection.transform_row_key`.
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
| Zone policy location? | **Permission** = `ZONE_PROPOSERS`. **Staging** = `packed_n` / void / `use_*` flags. Void path: `side_pack` XOR `group_fit` at staging (`use_side=False` on void; hijack unions `group_fit` into `enabled`). `GROUP_FIT` stays out of `ZONE_PROPOSERS[VOID_SEEK]`. |
| Keep cloud in zone set? | **Yes**, densify-only emit. |
| Native void densify accept? | **Union** via `subsample_transforms_with_pinned` (pinned densify/cloud prefix, rest = old `arr`, cap `max_proposals`). Reason `void_yield_union` when densify pinned; cloud may also pin when densify empty, `void_yield_drop`, or densify xy∉free (keep union reason if densify already pinned). Telem tags `void_yield_gain` / `void_pole_clear` / `void_yield_drop`. |
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
| Densify unions into propose pool (pinned prefix); intersect densify `proposer_keys` with final array? | Funnel accuracy |
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

---

## Pattern propagation (net-only agent Q&A — locked verdicts)

Surgical motif archive + pole-first ΔT lattice inside `void_seek_motif_anchors`. No BLF/GRASP outer rewrite. Relatives only in archive (not carry/elite/hist).

### Architecture and merge

| Q | Verdict | Implementation constraint |
|---|---------|---------------------------|
| Q1 Archive vs re-extract | **Archive with accept_count + TTL** (Memetic/GA building blocks; Gomez & Oliveira 2006). Re-extract alone forgets patterns destroyed by local swap. | On refine/repack accept: upsert and **reset TTL**. Unused for N iters → age out. Mirror void_elite APIs (`archive_accepted_patterns`). |
| Q2 Compound MWIS nodes? | **No.** Leader emit in MWIS; atomic follower attach in repack/`stamp_motif_at_anchor`. Compound nodes → hypergraph density (Burke et al. 2010). | Keep `stamp_motif_leader_follower`. Never add compound graph nodes. |
| Q3 Lattice in anchors without widening ProposeContext? | **Yes.** Δxy is a property of `ClusterPattern` + base anchors. | Only extend `void_seek_motif_anchors`. Respect ProposeContext fence. |

### Geometry and clearance

| Q | Verdict | Implementation constraint |
|---|---------|---------------------------|
| Q4 Does `emit_packing_clear` predict Scene `is_pose_clear`? | **Mostly**; float/graze edges can disagree. | Keep graceful degrade: full motif → subset/leader on Scene fail. Do not abort entire teleport for one clipped follower. |
| Q5 Contact ≤2·gap over-merge? | **Require hull compactness.** | Sort/filter by `sum(part_areas) / convex_hull_area(cluster)` via native hull. Archive/emit only high-compactness motifs. |

### Search policy and telemetry

| Q | Verdict | Implementation constraint |
|---|---------|---------------------------|
| Q6 Period-flood vs pole-first? | **Pole-first.** Full flood → candidate explosion / MWIS choke. | Generate ±k·ΔT, **sort by distance to void_pole**, truncate `top_k≈10`. |
| Q7 Does `motif_score_boost` move refine? | **Yes if it hits C++ MWIS weights**, not only Python pool ranking. | Route keys into `motif_keys` → `motif_score_boost`; XOR `TAG_MOTIF_HOLE` out of `pocket_keys`. |

### BLF / GRASP (rewrite rejected)

| Q | Verdict |
|---|---------|
| Q8 BLF vs lattice+archive on triangle sheet? | **BLF loses.** AABB gravity drives acute corners and wastes hypotenuse (Hopper & Turton 2001). ΔT lattice + MWIS + pole attractor = void gravity without AABB pathology. |
| Q9 What does GRASP add? | Construction RCL only — LS already LNS/repack/se2. **Phase 2b GRASP-lite** only if Gate 2 fails: sequential full-motif accept before MIS (`enable_motif_sequential_accept`). |

### Duplication / offload (Q10–Q15)

| Q | Verdict |
|---|---------|
| Q10 Second merge path? | **No** — one `merge_cluster_patterns` prefer: contact → archive → synth. |
| Q11 Densify-specific lattice? | **No** — lattice only in `void_seek_motif_anchors`; densify `cluster_copy` keys fold into `motif_keys`. |
| Q12 Relatives in carry/elite? | **No** — abs SE2 stay in hist/void_elite/carry; archive stores relatives + TTL only. |
| Q13 Unify packing vs Scene stamp SoT? | **No** — intentional split (`emit_packing_clear` vs `is_pose_clear`). |
| Q14 New C++ for lattice/TTL? | **No** — Python policy. Packing batch-clear only if stamp wall-time bites (twin of `batch_check_validity`). |
| Q15 Ablation switch? | `NO_PATTERN_PROPAGATE` disables archive, lattice, motif MIS boost; re-enables AABB mirrors. |

### Live telemetry keys

- `motif_telem`: `full_motif_clear`, `fallback_leader`, `lattice_anchors_*`, `motif_key_boost_hits`, `motif_refine_hits`, `accepted_patterns_*`
- void_leak: `motif_boost=` next to `key_boost=`
- Track A: `motif_cohorts`, `motif_sequential_full`, `motif_sequential_skipped_missing`

### Deferred polish (net-only Q16–Q28 — locked)

Pre-MIS full-motif lock via C++ `nest_by_scores(..., locked_indices=)` (Scene-clear cohorts). Identity-XOR on `group_id` is rejected. Attract (`NEAR`) is a score-tier bonus, not a clearance SoT. Cohorts at emit. Growing `is_pose_clear`. No union-first / OS-thread portfolio as primary.

| Q | Verdict | Implementation constraint |
|---|---------|---------------------------|
| Q16 Sequential vs MIS boost | Worth when rim-tuned MWIS clips void motifs | Pre-MIS **lock** during `nest_by_scores`; MWIS expands around the fixed set. Post-nest trial-eject of a pin is **Q56**, not a Q16 repeal. Do not trial-eject inside nest or DFS. |
| Q17 Missing followers | **Skip invent**; accept present ≥2 | No invent/hypergraph; emit records packing-clear same-group keys only; graph-pruned remainder → `motif_sequential_partial` |
| Q18 Batch Scene vs growing | **Growing `is_pose_clear`** | \(P_k\) vs obstacles ∪ prior members; not batch Scene for full motif |
| Q19 Timing | **Before nest/MIS** | Not after-nest/before-refine |
| Q20 Cap | RCL / pole top-k | Cap 2–3 from top 10 by `void_pole`; v1 may be deterministic |
| Q21 Hoist vs packing batch | **Hoist enough** | No C++ packing-margin-0 batch yet |
| Q22 Compactness on peels | Soft rank only | Sort peels by compactness; no hard min drop |
| Q23 Native vs Shapely hull | Hygiene/speed | `_pair_hull_area` → `convex_hull_area_of` |
| Q24 LNS recreate | Re-emit into the opened hole (Q57) | Leftover `motif_keys` from the pre-peel pool were emitted while the victim was an obstacle and cannot occupy the hole. Archive stamp iff re-emit is sterile (`block_hole_emit_in_hull==0`). |
| Q25 Scene dry-run scope | Motif reserve only | Do not thin general pool |
| Q26 Ownership | Pre-MIS sequential owns force-in | Pin = single-node; repack = orphaned leaders |
| Q27 Escalate Track D | large_void plateau | Δcov &lt;1% ×5 large_void iters AND `cluster_copy` r&gt;0 (≠ `PlateauTracker`) |
| Q28 Ablation | `no_motif_sequential_accept` | Keep archive/lattice/boost; disable sequential only |

**Rejected as Track A primary:** prefer-motif union after unconstrained nest; OS-thread portfolio exchanging proposals (serial archives already island-lite).

---

## Graph roles, gravity, nest SoT, block replace (Q29–Q60 — locked)

Net-only GO 2026-08-14. Nodes stay board-valid poses of catalog groups. **YES** = Scene-clear motif pins during `nest_by_scores`. **NO** = Penetrating collisions. **NEAR** = pairwise kiss, lexicographically under count/area. No identity-XOR, no compound MWIS nodes (Q2), no corner / min-x+y gravity.

### Sniper / attract (Q29–Q35)

| Q | Verdict | Implementation constraint |
|---|---------|---------------------------|
| Q29 Restrict NEAR to sniper fill | **Yes.** Union `cluster_copy`, `pocket_fit`, `group_fit`, `neighbor_slide` into `sniper_keys`. | Exclude `board_edge`, `side_pack`, explorers, mixer, hist/jitter. `border_pack_graph` attract=`[]`. No dummy packed vertices. No mid-pack `batch_pack`. Motif `member_keys`: consecutive pairs, not an extra all-pairs clique. |
| Q30 Attract vs count/area | **Attract cannot beat count/area.** | Tie-break / F only. 3a/3b accept has no attract term. No attract in nest greedy or DFS `path_delta`. Production nest `local_swap=False`. |
| Q31 pocket vs packed | **Not attract.** | Packed is not a graph vertex. Candidate↔obstacle only. |
| Q32 Mid-pack `batch_pack` | **No.** | Pairs stay empty-sheet / in-pool records only. |
| Q33 Evaluator join | **Same `make_polygon_graph` formula.** | Pass `propose_stats` + attract knobs. No second join. |
| Q34 `border_pack_graph` attract | **Stay `[]`.** | Packed set is already an IS. |
| Q35 Mixer in sniper? | **Never.** | hist/jitter/expand are near-duplicates. |

### Gravity field (Q36–Q44)

| Q | Verdict | Implementation constraint |
|---|---------|---------------------------|
| Q36 Multi-pole vs SW | **Spine-pole surrogate + rim inward normal. Kill SW.** | Tight pass gravity = unit(`pt_push − xy`) (off dropped parts). `border_focus` gravity = inward normal at seed xy. Floaters = `preferred_spine_pole` (nearest only when clearly closer than the first polylabel; raw nearest scattered the pack). Empty poles → skip pole pull, no corner/SW fallback. |
| Q37 Inter-cluster | **Override push to closest-island midpoint.** | Corridor seal, not Jostle L↔R. `gap_midpoint` is the push; obstacle/focal `primary_target` stays the free polylabel so large-void nearest-k is not stolen. |
| Q38–Q44 folded | Nearest pole per floater; post-motif `local_se2` again; rim tangent XOR pole | Kiss-hold veto / ray-cross skip are later; not this slice. |

### Nest SoT / extensions (Q45–Q55, folded)

Four kiss signals stay split (graph attract, `selection_geom`, `local_se2` kiss, outline boost). Unary void-island boost stays; dead `void_attractor_rule_weight` PointPlaceRules die. Production DFS locks stay **unset** (`_refine_options` must not assign `locked_indices`; finalize `insert_clear_locks` re-inserts). First-pass vs mid-pack SelectOptions stay split. No C++ SoA flags, `locked_groups`, directed attract, attract-degree in greedy, or nest `local_swap`. Partner keys only if sniper fill misses. Same-gid near-dup skip in join (not identity-XOR).

### Block replace (Q56–Q60)

Python set operators wrapping `nest_by_scores`. No hypervertices. One ruin stack: 3a ejection → 3b contact-CC re-emit → stamp fallback on the **same** victim.

| Q | Verdict | Implementation constraint |
|---|---------|---------------------------|
| Q56 Trial-eject Q16 pins | **Yes** (ejection chain, Glover 1996). Depth 1. | After nest, mid-pack only. Only cohort B that collides with A. `nest_by_scores(locked=(sel \ A) ∪ B)`. `motif_locked = (old \ A) ∪ B`. Cap 1 accept/iter. Skip `first_pass`. |
| Q57 Hole fill | **Re-emit + `nest_by_scores` locked=kept.** | True ruin-and-recreate (Shaw 1998). Stamp fallback on the same victim. Do not restore greedy leftover of `selected_nest`. |
| Q58 Victim unit | **3a = motif cohort; 3b = contact CC size 3–6.** | Drop void-kNN destroy. “Nearest CC to void pole” is a choice among CCs, not vertex-kNN. |
| Q59 Rim islands | **No.** | Skip `board_adj`. Outer-to-inner frame. |
| Q60 C++ cliques vs Python | **Python + existing `nest_by_scores`.** | No `locked_groups`, no compound nodes. |

**Flags:** `enable_block_replace` → 3a. `enable_lns_rebuild` → 3b. `enable_cluster_repack` → stamp. `NO_BLOCK_REPLACE` turns 3a off only. Neither ablation revives void-kNN leftover.

**Rejected unless a Q reopens:** identity-XOR; compound MWIS nodes; `ProposeContext` in `build_graph` / `elem_graph`; `proposer_id` on vertices; corner gravity; attract-ranked DFS; nest `local_swap=True` as default; dummy obstacle vertices; Touch as NO.

---

## Hollow nest / graph search (addressed)

Live hollow-rim packs swung coverage when densify **replaced** the propose pool, mix shuffled prefixes, pole-gravity walked rim seeds into the hole, and `nest_by_scores` rebuilt with no incumbent hold. Fix was stable **pool + nest**, staged A–E — no compound MWIS nodes, no attract on mixer, no DFS `locked_indices`, no `local_swap`.

| | |
|--|--|
| **Verdict** | Keep densify/cloud as a pinned prefix union; void-stage XOR `side_pack`/`group_fit`; tight-pass rim gravity only inside a part-scale band; prefix-stable mix with one `n_props` cut; sequential RCL beam (≤4 Scene-clear lock-sets + unlocked) then incumbent lex-hold; refine restore vs `nest_before_refine` on rim drop **or** not lex-better. |
| **Evidence** | Demo 2-iter: densify `void_yield_union`, `side_pack=0/0` on void, `mix_props` at void_seek floor, `rim_skip=1` at `rim≥0.9`, `sel_kept` = last packed, `incumbent_hold=1` prevents iter-2 collapse. `void_fill` seed 0 stays ≥0.9× shipped 44/0.494 with `independent_ok`. |
| **Constraint** | Reuse `subsample_transforms_with_pinned`, `transform_row_key` / `pose_key_to_index` / `pose_key_to_verts`, `lex_count_area_better` / `_sel_area` / `_packing_independent`, `_apply_rim_gravity` + `part_extents`, `sequential_accept_motif_cohorts` (extend, no second pole-RCL). Permission stays `ZONE_PROPOSERS`; void XOR is staging only. No new prepend/key/gravity helpers; no `group_fit` in `ZONE_PROPOSERS[VOID_SEEK]`. |

### Void colonization pull (follow-up)

Post hollow-nest stability left `nest=0` / rim freeze. Colonization recovery:

| | |
|--|--|
| **Verdict** | Cloud when densify is empty, `void_yield_drop`, or densify xy∉free; densify reason stays `void_yield_union` when densify pinned. `void_densify_pole_gravity` flag skips rim-band in `_merged_guidance_propositions` (densify enable rolled back — void_fill miss). Incumbent hold overridden on `large_void` when cand has more free centroids and count ≥0.9× incumbent. |
| **Evidence** | 4-iter smoke: `nest=14–24`, `incumbent_hold=0`, coverage ends 54.6% / 65 parts (no sterile rim lock). `void_fill` seed 0 stays near shipped floor with `independent_ok`. |
| **Constraint** | One hold gate beside existing lex; no second gravity helper; do not enable densify pole-gravity by default until re-gated. D/E (explorer budget / 3b) not needed once nest>0. |

---

## Macro-MCTS × PoseGraph (Q61–Q89, shipped 2026-08-15)

Two-tier only: Python Macro-MCTS over C++ PoseGraph. Clean break from ElemGraph / old iter loop dual. Gate loop after every letter (0.9× quality / 1.5× time; hard stop on `independent_ok=false`).

### Language ownership

| Component | Lang |
|-----------|------|
| UCB1 / PW / AMAF / ActionGenerator | Python |
| Propose emit / mix / zone wrap | Python orchestration |
| DecisionArena / MotifBase / SE2 / ContactGRG+GCI / PoseGraph | C++ |
| BoardSnapshot | Thin Python ledger (gids + float SE2 + coverage) — no Shapely clone per expand |

### Expand vs best leaf (Q69)

| Phase | Runs | Skips |
|-------|------|-------|
| Expand | `for_place` propose; freeze `improve_rules`; MotifBase→`cluster_copy`; greedy nest; 3a; **`dfs_passes=1` finalize_end refine** | 3b, `local_se2`/post_pack |
| Best leaf (final iter) | shipped DFS + finalize growth + 3b + post_pack/`local_se2` | — |

Polish budgets: `polish_budget_mid()` / `polish_budget_last()` in `heavy_polish.py`. `dual_nest_for` stays Q105 (last leaf **or** large_void).

### Locked table

| Q | Verdict | Constraint |
|---|---------|------------|
| Q61 | YES two-tier | Python MCTS × C++ PoseGraph; no 4D |
| Q62 | NO | Vertices geometric only |
| Q63 | NO | No compound MIS; locks/stamps |
| Q64 | YES | Delete ElemGraph; no facade |
| Q65 | YES | `local_swap=False` |
| Q66 | YES finalize NEAR | Sniper keys; not greedy obj |
| Q67 | YES motif pins | Locks pre-nest for motifs; DFS refine unset |
| Q68 | YES count→area | MCTS owns coverage reward |
| Q69 | YES cheap expand | Nest+motif+3a + low-`dfs_passes` refine mid; full DFS+3b/se2 on best leaf |
| Q70 | YES | Force `for_place` via `mcts_zone` |
| Q71 | YES | Unplaced ∩ allowlist |
| Q72 | YES | Skip `improve_rules` on expand |
| Q73 | YES | Inject → `cluster_copy` |
| Q74 | YES | Keep densify on void |
| Q75 | YES | Keep incumbent hold |
| Q76 | YES | MotifBase SoT |
| Q77 | YES either | Coverage or compact median; independent |
| Q78 | YES fixed | GCI α=β=0.5 |
| Q79 | YES warm motifs | AMAF not related-merged; related-sig warms MotifBase only |
| Q80 | YES | PW α=0.5 c=1.5 |
| Q81 | NO | No part-hash in AMAF |
| Q82 | YES | MCTS only outer loop |
| Q83 | YES | Research not rollback |
| Q84 | YES | Assert independence at tree end |
| Q85 | YES | Fix MCTS; no greedy resurrect |
| Q86 | **Reversed Q130** | `BoardSnapshot` lives on `DecisionArena` (POD; no telem). Thin Python ledger deleted. |
| Q87 | NO | No FAISS/SQLite |
| Q88 | YES | ContactGRG motif path (C++ SoT; upsert uses GCI) |
| Q89 | YES NFP-lite | `find_closest_polygon_cast` + `polish_se2_part` only |

### MotifBase SoT + nest/zone (Q90–Q104, locked 2026-08-16)

Unify letters N0→U5: MotifBase cross-iter library; retire Python `ArchivedPattern`; ContactGRG-only mining; cheap expand `local_swap=False`.

| Q | Verdict | Constraint |
|---|---------|------------|
| Q90 | YES pair SoT + live extract | MotifBase stores **pairs only**. N-way `ClusterPattern` on-the-fly same-iter only — **never** archive N-way. |
| Q91 | Hybrid | After M2 MotifBase = sole **cross-iter** source; `extract_cluster_patterns` may still feed `merge_cluster_patterns` **same iter** for immediate re-stamp. |
| Q92 | TTL+age **and** rank+truncate | Reset TTL on accept; `age` drops TTL≤0; at `max_keep` truncate by `accept_count` → `gci`. |
| Q93 | ContactGRG **only** | Delete NFP-lite last-two MotifBase mining. One miner. |
| Q94 | Floor = max(cfg, median) | Hard floor `motif_min_compactness=0.35`; moving median only **raises** floor when library healthy. |
| Q95 | Keep split | Relatives `se2_key3` / round-3; absolute MIS `transform_row_key` round-4. Do not unify. |
| Q96 | Cheap False + void dual | Expand: `local_swap=False` by default. Dual lex on **heavy/final leaf** (Q96 base). |
| Q97 | YES override + telem | Rim force + `large_void` → soft hijack to `void_seek`; log `void_hijack_over_mcts`. |
| Q98 | `interior_pocket` | Sheet MacroRegion → `PlaceZone.interior_pocket`. |
| Q99 | `part_gid` only | Tag/soft boost propose by `action.part_gid`; do **not** hard-filter remaining gids. |
| Q100 | Defer `runner.run` | Linear `pick_expand_action` until Ub/D3 multi-sim. |
| Q101 | Scalars until after M2 | Related warm: rim/void scalars — **no** 8×8 Hamming in this plan. |
| Q102 | Stay `rule_id=0` | No secondary rule preset exploration. |
| Q103 | Keep dual clearance | `emit_packing_clear` vs `is_pose_clear`; MotifBase pair stamps obey emit rules into MWIS. |
| Q104 | Motif → `void_seek` | PLACE_MOTIF forces `void_seek` (rigid pairs need free space). |
| Q105 | Dual = heavy OR large_void | Hybrid of Q96 strict + void basins: `dual_nest` True when heavy leaf **or** `free_info.kind == large_void`. |

Gate scrap (unify): void_fill area ≥ **0.585**, time **&lt; 170s**, `independent_ok`. (Raised from 0.570 under Path D / G1.)

### Gate scrap (void_fill seed0 shipped)

| Ref | Parts | Area | Time | Indep |
|-----|-------|------|------|-------|
| Baseline (pre-R0) | 48 | 0.490 | 62.95s | True |
| U3 cutover | 50 | 0.498 | 73.89s | True |
| G0 (pre Path D) | 43 | 0.508 | 159.05s | True |
| Path D + V1/P1/M1/T1/R1 (live) | 44 | 0.537 | ~60–87s | True |

G1 floor raised to **0.585** in fixtures; live seed0 still miss on `area_coverage` (hollow-rim / `graph_to_nest` with `void_nest≈0`). Continue miss-loop on colonization (colonize telem + mid 3b shipped; void score deepen shipped).

### Path D locks (Q107–Q128 — shipped with raise-gate)

| Q | Verdict |
|---|----------|
| Q107 | `n_sims=K=4` multi-sim cheap_pack after cache warm |
| Q108 | `leaf_reward` raises void λ under `large_void` |
| Q111/114 | Motif Scene dry-run sticky when patterns/library present |
| Q118 | Void-centroid `0.75×` + void_seek 1.25; island inv-sq pole decay |
| Q128 | Soft-cap `attract_max_degree` to 3 when graph ≫ nest |

### Open research (Q106–Q129 — residual)

| Q | Question | Why ask |
|---|----------|---------|
| Q106 | Soft `force_zone`: Rim Q97 full reclaim (shipped) vs densify-floor-only (Q126)? | Validate scrap |
| Q109 | Motif PLACE vs Void: keep Q104 `void_seek` alias? | **Locked Q147:** keep alias |
| Q110 | Unlock `rule_id` presets after U3, or stay Q102=`0`? | **Locked Q141:** stay `rule_id=0` |
| Q112 | Motif→`join_attract_pairs` net density or noise (Q30)? | Soft MIS |
| Q113 | Adaptive inject prune via compactness / accept_count? | **Locked:** TTL miss-streak; accept=0→delete/ttl=-1; accept>0 floor TTL=1 |
| Q115 | `find_nearest` at stamp time vs inject-only warm? | Pattern match |
| Q116 | `find_exact` / accept_count bump on nest Motif hit? | **Locked:** credit on nest Motif survival + ContactGRG; `credit_accept` |
| Q117 | Incumbent outline_cov ε (S0) enough, or tighten further? | Cov oscillation |
| Q119 | Mid DFS every iter vs every-K after time miss? | Budget |
| Q120 | Keep nest_void_term / void_override / refine_ms on void_leak? | Telem SoT |
| Q121 | `compose_nest_kwargs` — any Uh-only paste that must stay? | Drift |
| Q122 | void_refine_hold in restore — void_fill scrap impact? | Polish parity |
| Q123 | Delete unused optimize_polygons / score_transforms / ribbon? | Dead weight |
| Q124 | Raise mid `dfs_passes` before mid post_pack on time miss? | Iteration budget |
| Q125 | Soft Motif gids (shipped) vs prefer_motif_id only? | no_rels |
| Q126 | Rim Q97 zone flip vs densify-floor-only? | Preference vs geometry |
| Q127 | Keep `refine_ms` on void_leak permanently? | Stage SoT |
| Q129 | Keep `mcts_heavy` alias or telem `dfs_passes` only? | Telem SoT |

### Native DG storage + pack execute (Q130–Q149, locked 2026-08-17)

Q86 reversed. Q89/Q93/Q102/Q104 kept. Letter pass = no drop vs snapshot (`independent_ok`, area ≥ snapshot, time ≤1.5× and not slower unless area rose). Fixture floor 0.585 unchanged.

| Q | Verdict | Constraint |
|---|---------|------------|
| Q130 | Native `BoardSnapshot` on `DecisionArena` | `vector<BoardSnapshot>` indexed by node id. `snapshots.size()==arena.size()`. |
| Q131 | Telem Python only | Existing `mcts_telem`. No `snapshot.telem`, no parallel `telem_by_node`. |
| Q132 | AoS snapshots, SoA inside one snapshot | `vector<int32_t> packed_gids` + `vector<Se2> packed_transforms`. |
| Q133 | Bitmask remaining if contiguous ≤64 | Catalog `ngroups` (default 2). `uint64_t remaining_mask`. Packed instances stay vectors. |
| Q134 | Copy parent, then mutate | No COW. |
| Q135 | Native `nfp_lite` | Batch inject loop in C++; one Scene. |
| Q136 | Keep Q93 | NFP-lite is inject polish only. MotifBase miner = ContactGRG. |
| Q137 | `geometry/common/nfp_lite.h` | Bind next to `polish_se2_part`. Relative via `se2_relative`. |
| Q138 | Scene fail-closed | Cast must pass Scene; keep original on fail. |
| Q139 | Native `MacroNicheArchive` | Sibling type; runner holds one. Not on `DecisionArena` / snapshots. |
| Q140 | Niche key = current AMAF tuple | `(region, 0, motif_id)`; `(Void, 0, -1)` only when no action. |
| Q141 | Keep Q102 `rule_id=0` | No per-proposer AMAF. EMA pool scales stay proposer budget. |
| Q142 | Survivors and hollow ghosts | One `append_positive`: nest `proposer_keys` else centroids. |
| Q143 | Cheap must apply sampled Motif | Cache key `(zone, motif_id)`. Miss → re-compose. |
| Q144 | ContactGRG upsert outer leaf only | Do not upsert inside `run_mcts_multi_sim`. |
| Q145 | Do not mute replay when tree chose Void/Motif | Rim-sat `use_history_expand=False` only for Rim/Sheet. |
| Q146 | `free_space_cloud` in main `void_seek` explorer | One emit; densify reuses it. |
| Q147 | Keep Q104 Motif → `void_seek` | Rigid pairs need free space. |
| Q148 | One `execute_pack` + flags | Calls `run_pack_stages`. No `cheap_pack_from_cache` / `run_pack_body`. |
| Q149 | Evaluator = same execute | Delete `analyze_free_space` fork. |

### Hybrid DecisionGraph (Q150–Q165, locked 2026-08-19)

Q61 reopens as one C++ `DecisionGraph` owner (partitioned Pose + Arena). Not 4D product verts (Q62). Q30/Q33/Q63/Q70/Q90/Q141/Q144/Q148 stay in force. Do not lock these in AGENTS.md.

| Q | Verdict | Constraint |
|---|---------|------------|
| Q150 | YES one hybrid C++ owner | `DecisionGraph` owns partitions. `replace_poses` copy-in updates geometric arrays; **keeps** Sequence/AMAF/snapshots. Kind **identity** (MacroRegion 0–3) persists. **Reject** Python sidecar / orphaned PoseGraph aliases (`graphs` window keeps `make_polygon_graph` object). |
| Q151 | Kind (MacroRegion) | MemberOf → stable Kind (`pose_kind[]`, 255=untagged). **Reject** arena node ids. |
| Q152 | YES first-class Attach | Pair of Pose ids; not in `PoseGraph::elems`. |
| Q153 | Derived Collision walk | Attach X Mutex Y iff some member pair Collides. **Reject** parallel intersection buffers. Mutex **query**, not a stored CSR. |
| Q154 | Slice + materialize | `nest_by_scores` Pose-slice only; then `realized` flags on Attach/MotifJoin + Kind histogram. |
| Q155 | Keep product-vertex ban | MemberOf, not `(x,y,θ)×Rim` MWIS verts. |
| Q156 | G1b / overlay | Miss → Kind score overlay or mix quota. No 4D clones. |
| Q157 | YES orthogonal | Motif boosts MotifJoin; Void boosts Kind. Same `void_seek` sample, different MCTS semantics. |
| Q158 | Neutral unless tagged | History/elite default no region boost. |
| Q159 | Sequence-OR only | No Macro–Macro packing Mutex. |
| Q160 | Pairs only | Attach = 2 pose ids. |
| Q161 | YES drop epoch overlays | `replace_poses` clears MemberOf / Attach. Rebuild that epoch. |
| Q162 | Keep Q143 | Cheap key `(zone, motif_id)`. **Do not** hash MemberOf. |
| Q163 | YES Pose only | Refine/finalize `const PoseGraph &`. |
| Q164 | Keep type | `DecisionArena` public; `DecisionGraph` contains one. |
| Q165 | YES fold survival | `_amaf_pick_score` reads **realized** Kind/Attach hits (outer leaf / Q144 cadence), not only Sequence commands. |

**Q150 vs Q161:** Kind overlays in Q150 = the four Kind identities + arena. Epoch MemberOf/Attach still wipe (Q161).

## Refine vs DecisionGraph (Q166–Q178 — locked 2026-08-19)

Refine operates on **`const PoseGraph &` only** (Q163). DecisionGraph influences refine through **one score SoT** (`apply_void_selection_boosts` → `refine_scores = list(scores)`), **soft MotifJoin ε** and **attract sub-lex tie-breaks** in C++ DFS (when `dg_aware_refine`), and **one materialize readback** post-3b/pin (`finalize_iter_mcts`). No compound vertices, mid-epoch re-bind, or DFS growth locks.

### Score SoT (unified)

```
bind_epoch → pose_kind[] / Attach / MotifJoin
apply_void_selection_boosts (+ G1b pose_kind when dg present)
apply_void_centroid_score_term
nest_by_scores(scores)
refine_scores = list(scores)   # after 3a swap; no second boost at refine entry
refine_selection_dfs(refine_scores)
materialize_selection once post-3b/pin
```

Duplication removed: compose mid-refine and pack_loop mid-refine `materialize_selection` calls; kind bias merged into `apply_void_selection_boosts` via `pose_kind[]` (skip duplicate `kind_keys` keyed boost when CSR tagged).

### Local convergence (DFS / Motif / Attract)

| Q | Verdict | Constraint |
|---|---------|------------|
| Q166 | YES soft MotifJoin ε | Subtract tiny ε from DFS `path_delta` when breaking a MotifJoin edge; Pose-only search (Q163/Q47). |
| Q167 | YES attract sub-lex | Count → Area → score sum → attract pairs; attract never beats count/area (Q30). Shared helper: `selected_attract_pairs` in finalize + DFS. |
| Q170 | YES mirror nest scores | Single boosted `scores[]` copied to `refine_scores` at compose boundary; no refine-entry re-boost. |
| Q171 | NO CC-local refine | Keep global conflict-resolution sweep. |
| Q173 | NO DFS motif locks | Soft ε (Q166) + finalize re-insert; no growth locks (Q47). |

### Global convergence (DG readback & MCTS)

| Q | Verdict | Constraint |
|---|---------|------------|
| Q168 | NO mid-epoch re-bind | `bind_epoch` propose-only (Q161). |
| Q169 | YES materialize post-3b/pin | Single `materialize_selection` in `finalize_iter_mcts`; AMAF sees final vertices (Q165). |
| Q172 | NO cheap materialize | Cheap expand stays DG-free (Q143). |
| Q174 | NO refine-delta reward | Leaf reward = absolute final state only. |

### MWIS interaction

| Q | Verdict | Constraint |
|---|---------|------------|
| Q175 | YES G1b Kind overlay | Extend `apply_void_selection_boosts` with `dg.pose_kind[]`; not a second overlay module. |
| Q176 | NO dual refine on large_void | Single DFS pass per iter. |
| Q177 | YES void shed gate | `apply_refine_with_restore`: void count drop ≥1 → restore unless refine wins **count** lex. |
| Q178 | YES ablation flag | `ProposeConfig.dg_aware_refine` toggles **C++ DFS only** (Q166+Q167); Q169/Q175/Q177 ship without flag. |

Cross-link: [decision_graph.md — Refine boundary](decision_graph.md#refine-vs-decisiongraph-boundary-q166q178).

**N0 snapshot** (2026-08-19, seed 0, shipped, `--gate` fixture FAIL vs 0.585 is not a miss):

| Tag / case | Area | Time | Indep |
|------------|------|------|-------|
| void_fill `demo_triangle_corner_cluster_s6` | 0.558 | 315.58s | True |
| dense `dense_cluster_pockets_s8` | 0.581 | 403.63s | True |

**N0 snapshot** (2026-08-17, seed 0, shipped, `--gate` fixture FAIL vs 0.585 is not a miss):

| Tag / case | Area | Time | Indep |
|------------|------|------|-------|
| void_fill `demo_triangle_corner_cluster_s6` | 0.536 | 321.49s | True |
| mid_pack `demo_triangle_corner_cluster_s6` | 0.536 | 222.21s | True |
| mid_pack `border_then_fill_s13` | 0.538 | 189.46s | True |
| mid_pack `loose_cluster_compact_s9` | 0.462 | 241.35s | True |
| mid_pack `dense_cluster_pockets_s8` | 0.558 | 153.61s | True |

## Edge→center bridge + local-minima escape (Q179–Q195 — locked 2026-08-20)

Structural gap: rigid outer rim (`EMPTY_BORDER` / `board_edge`) without geometric connectivity into the interior; accidental Motif structures under-reused; plateau tapers budgets instead of repairing. Fix by extending `ZONE_PROPOSERS`, `corridor_seed_coords_from_samples`, MotifBase/`cluster_copy`, plateau→3b, and softer restore — **not** a second proposer stack. Master ablation: `ProposeConfig.enable_inward_bridge` (R1+R2).

### Edge→center bridge (R1)

| Q | Verdict | Constraint |
|---|---------|------------|
| Q179 | HYBRID (threshold) | Add `RAYCASTING`+`EROSION` to `EMPTY_BORDER` only if `packed_n >=` threshold (e.g. 5). Early placements stay rim-snap. |
| Q180 | YES unify | Baseline `BORDER_GAP` always includes `RAYCASTING`+`VORONOI`+`EROSION` (annulus = non-annulus). Cull via `valid_at`, not permission split. |
| Q181 | HYBRID both | Rim-anchored raycasting primary; if distinct free lobe, extend `corridor_seed_coords_from_samples` for generic lobes (not sheet-hole only). |
| Q182 | YES soft scale | Under `rim_sat`, do not hard-mute `side_pack`; soft-scale (e.g. ~75% cut of `max_proposals`) and keep active. |
| Q183 | NO | `first_pass_border` stays rim-only; no inward seeds. |
| Q184 | YES | Extend `propose_placements_raycasting` anchors to include inner-boundary vertices of packed rim parts. No new proposer name. |

### Structure reuse (R2)

| Q | Verdict | Constraint |
|---|---------|------------|
| Q185 | Nest survival | Extract to MotifBase only after island survives `nest_by_scores` + final DFS refine (align Q144/Q165). |
| Q186 | Floor mandatory | Hard minimum quota for `cluster_copy` keys in `transform_batch` under plateau; “do not mute” alone is insufficient. |
| Q187 | YES soft override | Motif-keyed set overrides incumbent hold if strictly higher contact density **and** equal/greater part count. |
| Q188 | MotifJoin only | Reuse via MotifBase→`cluster_copy` rigid inject; not `history_expand` jitter. |
| Q189 | Density + refine hits | Cap archive by contact density / refine hits; never by DecisionGraph Attach/Kind. |

### Local-minima escape (R3)

| Q | Verdict | Constraint |
|---|---------|------------|
| Q190 | 3b first | Plateau + residual free → trigger `maybe_block_hole_renest` first; not `cluster_repack` / `local_se2` as primary. |
| Q191 | YES explore | Plateau + free remaining → **increase** `max_proposals` for `void_seek` + inward explorers (do not only taper). |
| Q192 | YES carefully | Extend Q177: accept refine if count rises; on count-tie accept if void-fill rises. Do not restore solely for rim loss. |
| Q193 | NO | Poles once per outer iter in post_pack only. |
| Q194 | Dual lex enough | Q105 dual lex for shallow minima; plateau **3b** only if dual lex fails / still stuck. No new repair flag. |
| Q195 | One master flag | `ProposeConfig.enable_inward_bridge` (default True) bundles R1 zone permissions + R2 archive floors. R3 wires to existing plateau/3b paths. |

**R0 snapshot** (2026-08-20, seed 0, shipped; `--gate` fixture FAIL vs 0.585 is not a miss):

| Tag / case | Area | Time | Indep | Notes |
|------------|------|------|-------|-------|
| void_fill `demo_triangle_corner_cluster_s6` | 0.542 | 168.72s | True | zones void_seek; cluster_copy=0 |
| dense `dense_cluster_pockets_s8` | 0.559 | 143.04s | True | zones void_seek; cluster_copy=173 |

**R0 telem confirm** (3-iter-style full run, void_fill): `rim=0.887 rim_sat=0 side_pack=0/0 ray=0/0 voronoi=0/0 erosion=0/0 bottleneck=graph_to_nest zones=['void_seek',…]` — hollow shell: late iters void_seek without inward explorer emit on rim bridge.

**R1 snapshot** (2026-08-20, seed 0, shipped, `enable_inward_bridge=true`):

| Tag / case | Area | Time | Indep | Notes |
|------------|------|------|-------|-------|
| void_fill `demo_triangle_corner_cluster_s6` | 0.548 | 183.12s | True | peak_ray=1075 peak_ero=56 (≥0.9×R0); mid-iter inward bridge |
| dense `dense_cluster_pockets_s8` | 0.549 | 141.23s | True | peak_ero=1688 cluster_copy=303 (≥0.9×R0) |

R1 miss-loop note: peak emit must persist across iters (`inward_peak` in evaluator); last-iter void_seek alone under-reports explorers.

**R2 snapshot** (seed 0, shipped ON):

| Tag / case | Area | Time | Indep | Notes |
|------------|------|------|-------|-------|
| void_fill | 0.573 | 202.32s | True | arch_n=4 motif_ref=9 plat_boost=1; mix_floor=0 (no cluster_copy emit) |
| dense | 0.556 | 234.08s | True | mix_floor=24 arch_n=4 motif_ref=27 (≥0.9×R0/R1) |

**R3 snapshot** (same runs): `run_3b=1` both tags; dense `restore=1`; void `restore=0`. Plateau→3b via polish budget; Q192 softer restore live. `3b_ok` often 0 (attempt without accept is enough for telem gate).

**R4 ablation** (`enable_inward_bridge=false` vs ON):

| Case | ON area | OFF area | ON signals | OFF signals |
|------|---------|----------|------------|-------------|
| dense | 0.556 | 0.559 | inward_att=1 mix_floor=24 | inward_att=0 mix_floor=0 |
| void_fill | 0.573 | 0.551 | inward_att=1 | inward_att=0 mix_floor=0 |

Master flag gates R1 early bridge + R2 mix floor/override. R3 (`run_3b` / `plat_boost` on last leaf) remains active with flag off (Q195). Void_seek explorers still emit when off (pre-existing zone permission); ablation SoT is `inward_att` + `mix_floor`.

## Motif key SoT + RepairCohort (Q196–Q212 — locked 2026-08-20)

Structural blindness: 3b ruin/recreate and peel/stamp used separate pattern lists and victim picks, so repair heuristics fought instead of escaping local minima together. Follow-up unifies **orchestration only** — one `resolve_motif_keys` SoT + one `RepairCohort` vocabulary — while keeping Q58/Q59 victim policies, Q24 sterility, Q190/Q193 last-leaf stamp, and no second stacks.

Q182 letter gate does **not** apply on late `void_seek` (`side_pack` XOR-off by design). Ablation SoT remains `inward_att` + `mix_floor`, not `peak_ray`.

### Locked research (Q196–Q212)

#### Motif keys (F1)

| Q | Verdict | Constraint |
|---|---------|------------|
| Q196 | YES | One `resolve_motif_keys` in `motif_keys.py`. Prefer: (1) projected propose_stats → (2) densify fold → (3) `cluster_copy` ∪ `motif_hole`. Collapse floor/override/MIS readers; no parallel rebuilds. |
| Q197 | YES | `fold_emit_motif_keys` unifies primary+densify; round-4 keys via `transform_row_key`. |
| Q198 | Key-hit fraction | Q187 override = key-hit only. GCI/`accept_count` = MotifBase truncation (Q189) only. |
| Q199 | YES hybrid | If `dens_inc==0`, allow override when `dens_cand>0` ∧ count≥incumbent ∧ `enable_inward_bridge`. |
| Q200 | NO | Cheap `motif_graph_hits` → `motif_graph_keys` only; never poison emit `motif_keys` (Q143). |
| Q201 | YES | Emit keys from densify/`cluster_copy` must reach mix when present; floor cannot invent keys. Letter gate: peaked `cluster_copy_emitted` or `mix_floor` when `arch_n>0`. |

#### Repair cohort (F2)

| Q | Verdict | Constraint |
|---|---------|------------|
| Q202 | YES | Thin `RepairCohort(victim, patterns, mode)` mid→post; no recompute between stages. |
| Q203 | NO | Facade `pick_repair_victim`: `pick_block_hole_victim` first; only if empty → `bfs_peel_victim`. |
| Q204 | YES | If no interior CC, fallback `board_adj` CC size [3,6] inside `pick_block_hole_victim`. |
| Q205 | YES | `build_repair_patterns` → peel+capped ∪ archived via `merge_cluster_patterns`; 3b+stamp share list. |
| Q206 | superseded by Q214 | Stamp iff sterile (`emit_in_hull==0`); hull reject clears cohort only — keep `allow_repack` for BFS peel (Q213/Q214). |
| Q207 | YES | Sterile handoff passes exact `cohort.patterns`; never rebuild if populated. |
| Q208 | Wire to 1 | `cluster_repack_max_attempts` default/hardcap **1**. |

#### Telem / ablation (F3)

| Q | Verdict | Constraint |
|---|---------|------------|
| Q209 | `inward_att` + `mix_floor` | Not `peak_ray` (VOID_SEEK explorers fire with bridge off). |
| Q210 | NO | No void_seek `side_pack` letter gate; soft-scale only on border_gap/cluster_edge. |
| Q211 | YES | Letter gates use `repair_mode` / `repair_patterns_n` SoT; keep `block_hole_*` aliases. |
| Q212 | NO | No mid-iter post_pack on plateau (Q190/Q193). |

**Pin+3b independence:** after 3b hole appends, pin merge keeps extras and drops colliding graph rows (`pack_loop`).

### Snapshots (seed 0, shipped)

**F0** (observe-only telem):

| Tag / case | Area | Time | Indep | Notes |
|------------|------|------|-------|-------|
| void_fill | 0.566 | 249.74s | True | 3b_try=0 arch_n=4 mix_floor=0 |
| dense | 0.556 | 266.16s | True | mix_floor=24 cluster_copy=303; 3b_try=0 |

**F1+F2 shipped** (key SoT + RepairCohort):

| Tag / case | Area | Time | Indep | Notes |
|------------|------|------|-------|-------|
| void_fill | 0.557 | 168.40s | True | cluster_copy=170 arch_n=4 mix_floor=0; 3b_try=1 repair=1/2 |
| dense | 0.607 | 90.67s | True | mix_floor=24 cluster_copy=270; 3b_try=1 repair=3/2 |

**F3 ablation** (`enable_inward_bridge=false`):

| Case | ON area | OFF area | ON signals | OFF signals |
|------|---------|----------|------------|-------------|
| void_fill | 0.557 | 0.571 | inward_att=1 mix_floor=0 cluster_copy=170 | inward_att=0 mix_floor=0 (peak_ray stays) |
| dense | 0.607 | 0.620 | inward_att=1 mix_floor=24 | inward_att=0 mix_floor=0 (peak_ray stays) |

### Void_fill degrade loop (D0–D3) — Q213–Q228

Bar: match F3 OFF **≥ 0.571**; dense ≥ **0.9× 0.607**; indep OK. Strict D0→D1→D2→D3.

#### Locked net-only

| Q | Verdict | Constraint |
|---|---------|------------|
| Q213 | YES | Hull reject → restore BFS peel (`allow_repack` + `victim_indices=None`); do not retarget rejected cohort. |
| Q214 | Cohort-only | Clear `stamp_victim` + `_repair_patterns`; **do not** set global `allow_repack=False`. |
| Q215 | Void-facing only | Board_adj fallback in `pick_block_hole_victim` only if some member `part_void_adj`; no second helper. |
| Q216 | Interior first | Keep interior CC preferred; board_adj secondary. Protects dense. |
| Q217 | Absorb ∪ fold | Delete exclusive early-return in `resolve_motif_keys`; densify + emit `cluster_copy` same map. |
| Q218 | YES fall through | Densify keys disjoint from `proposal_pins` → treat no-hit, fold emit into mix keys. |
| Q219 | Soft emit + soft Q187 | Soft Q187 coverage + soft-gate void_seek early-bridge / rim-anchored rays / lobe seeds (border keeps early bridge). |
| Q220 | YES hybrid | Wrap motif override in same `outline_coverage_ratio` / `drop_allow` as `void_override`. |
| Q221 | OR accept | Motif key-hit **and** coverage within drop_allow → accept (with void_override family). |
| Q222 | Extras-first | Keep pin merge preferring 3b extras. |
| Q223 | YES clear patterns | Hull reject clears `_repair_patterns` before BFS stamp. |
| Q224 | Sterile then BFS | Sterile cohort stamp first; if non-sterile → global BFS (`victim=None`). |
| Q225 | Plateau\|last_leaf only | No large_void-always mix floor. |
| Q226 | Match OFF ≥0.571 | Stretch bar; do not ship at 0.566-only if still below historical F3 OFF. |
| Q227 | Dense interior 3b | D0 mute must confirm dense still accepts interior 3b. |
| Q228 | Telem required | D1 gate needs `repack.attempted > 0` after hull reject — area alone insufficient. |

#### D-loop snapshots (seed 0, shipped)

**D0** (confirm): dense interior `3b_ok=1` under shipped; mute LNS void telem recorded.

**D1–D2** (stamp decouple + motif absorb∪fold): `prepare_post_pack` hull-reject clears cohort/keeps `allow_repack`; `resolve_motif_keys` always ∪-folds emit; dense `mix_floor=24` proves floor path.

**D3 shipped** (soft Q187 + void_seek emit soft-gate):

| Tag / case | Area | Time | Indep | Notes |
|------------|------|------|-------|-------|
| void_fill ON | **0.587** | 167.21s | True | ≥0.571; inward_att=0 (no void_seek early-bridge); 3b_ok=1 repair=3/2; motif_ov=0; mix_floor=0 |
| dense ON | **0.620** | 94.99s | True | ≥0.9×0.607; mix_floor=24; 3b_ok=1; inward_att=0 |
| void_fill OFF | 0.602 | 168.33s | True | inward_att=0 mix_floor=0 (Q209); peak_ray stays |

Miss loop: post-lex board_adj accept ban regressed dense/void — removed; Q215 stays in pick only. Residual ON gap closed by soft-gating void_seek early-bridge + rim-anchored explorer rays + lobe seeds (border early-bridge kept).

---

### Repack + DG cohort tracks (Q229–Q254 — locked 2026-08-23)

R→P→M→S plan: pair upsert, clique telem, leader-star MotifJoin, score-by-rules MCTS. Cross-track synergy: [AGENTS.md](../AGENTS.md) Planning § Cross-track synergy.

#### R track — repack & Motif learning (Q229–Q234)

| Q | Verdict | Implementation constraint |
|---|---------|---------------------------|
| Q229 | Hybrid (relative primary, contact secondary) | Decompose `ClusterPattern` relative poses for stamp intent. Masked contact fallback only. |
| Q230 | `placed_idxs ∪ kept` only | Confine upsert mask to stamp neighborhood. No full-board mid-iter upsert. |
| Q231 | YES (widen gate) | `credit_motif = motif_refine_n > 0 or repack_m > 0`. TTL reset on stamp accept. |
| Q232 | Max native-centroid-to-pole | No Shapely `unary_union` on peel hot path. |
| Q233 | Cap victims/coords | Fallback N=1 or skip after motif_accepted budget exhausted. |
| Q234 | Bench first (`reserve_archived=1`) | Do not raise `cluster_copy_max_patterns` blindly. |

#### P track — pattern persistence (Q235–Q238)

| Q | Verdict | Implementation constraint |
|---|---------|---------------------------|
| Q235 | τ ≈ 0.5 on void_fill stamp iters | `< 50%` motif_clique_full_hits → P2 cohort_sig. |
| Q236 | Optional `int32_t` on `MotifRecord` | Do NOT alter `motif_key`. |
| Q237 | Leader-star stitch for inject | k−1 pairs reconstruct k-part island. |
| Q238 | NO UNIFY (keep Q95) | `cohort_sig` semantic tag at inject rank. |

#### M track — compose, MotifJoin, M3 (Q239–Q244)

| Q | Verdict | Implementation constraint |
|---|---------|---------------------------|
| Q239 | Keep 4 (in-place truncate) | Hard-truncate motif_lock_sets to 4. |
| Q240 | Lex first, density soft tie | Count → Area → density when area Δ ≤ ε. |
| Q241 | Leader-star (k−1) | No full-clique MotifJoin. Dynamic leader = max bbox area. |
| Q242 | Preserve locks on override | Do not wipe locks on incumbent hold when motif_sequential_full > 0. |
| Q243 | Both required for M3 | member_hits > 0 AND motif_compose_accepted_size ≥ 2. |
| Q244 | Seed beam seeds | Feed growing-accept as Seed #1 into M1 beam. |

#### S track — score-by-rules & MCTS (Q245–Q250)

| Q | Verdict | Implementation constraint |
|---|---------|---------------------------|
| Q245 | Normalized Δscore / sel_n | Per-vertex score with floor ε. |
| Q246 | YES if score δ large (void/large_void) | Small rim drop OK when interior score δ strong. |
| Q247 | K ≤ 3 initially | Cap rule_id exploration at 3. |
| Q248 | Defer (monitor telem) | Cheap MotifJoin parity only if telem diverges. |
| Q249 | Plateau + reward ≥ parent best | One improve_rules mutation per iter max on plateau. |
| Q250 | Telem first; 1.25× cap if needed | Combined void boost cap if debris clustering. |

#### Cross-track integration (Q251–Q254)

| Q | Verdict | Implementation constraint |
|---|---------|---------------------------|
| Q251 | Keep both upsert paths | Stamp upsert + end-of-iter full-board contact upsert. |
| Q252 | Extend AMAF tuple (M3) | `(region, rule_id, motif_id, cohort_sig)` when M3 ships. |
| Q253 | Monitor refine_rejected | No special restore for member_hits until telem proves regression. |
| Q254 | Extend upsert_from_contacts | Optional `patterns` for leader-star + contact in one gate. |

---

### Repack + compose DG follow-up (Q255–Q262 — locked 2026-08-28)

Pre-implementation verdicts for R/C plan letters. Cross-track synergy: root [AGENTS.md](../AGENTS.md) Planning § Cross-track synergy.

| Q | Verdict | Implementation constraint |
|---|---------|---------------------------|
| Q255 | **YES — pre-bind safe** | `make_polygon_graph` populates `group_id` / `transform` / `propose_stats` for `motif_graph_hits`. `inject_cohorts_from_patterns` after graph build, **before** `bind_graph_epoch`. Two call sites: outer [`build_graph.py`](../nest_graph/build_graph.py) pre-bind; cheap [`cheap_pack.py`](../nest_graph/decision/cheap_pack.py) `compose_cached_selection` only. |
| Q256 | **Iter-scoped anchor cache** | Cache `void_seek_motif_anchors` inside one repack stamp pass; clear at `cluster_repack_selection` entry. Do not key only on `motif_id`. Fold repack `aligned_poses_for_pocket` into anchor builder (R4). |
| Q257 | **Motif > Void/Sheet/Rim** | `bind_epoch` scalar `kinds[]`: second pass after L138 sets `kinds[ix]=Motif` for cohort member indices. Overwrites prior zone Kind for injected cohort vertices (Q243 `member_hits`). Never emit `motif_keys` (Q200). |
| Q258 | **Validate before mix pin** | C2 archived world poses MUST pass `is_pose_clear` vs current `nest_state` before mix prepend ([`transform_batch.py`](../nest_graph/propose/transform_batch.py) L677–708). |
| Q259 | **Cheap DG read-only** | C5: pass `dg=` for `motif_join_pairs` read only. No `materialize_selection` on cheap path (Q172). Multi-sim must not mutate shared `dg.motifs()` / `realized`. |
| Q260 | **YES — mirror 3b mute** | R5: mute `CLUSTER_COPY` in repack fallback when motif stamp paths exhausted (3b `_HOLE_PROPOSERS` pattern). |
| Q261 | **NO third victim picker** | R7: peel-none → `pick_repair_victim` facade (Q203). Hoist adjacency in `_priority_bfs_peel` only. |
| Q262 | **YES — packing-clear subset** | C3: when Scene `is_pose_clear` fails on `large_void`, trial `emit_packing_clear` before skip. Accept path already has Scene trial (L192–196). |

**Code facts (2026-08-28):** `ClusterPattern.members` are relative SE(2) — R6 rebuilds patterns for new BFS victim, does not retain hull-reject cohort (Q223). `motif_graph_keys` write-only until Q263.

| Q276 | **Hull-reject peel guard** | R7: do not route hull-reject peel-none through `pick_repair_victim` (Q224 BFS only). |
| Q277 | **Cohort dedup in merge** | `merge_motif_hits` dedup append by `(motif_id, leader_key)` to avoid C1 + emit double cohorts. |

---

## Open research — Repack + compose DG (Q263–Q275 — net-only)

**Status:** open backlog. Locked pre-ship: Q255–Q262 above.

| Q | Question | Research hooks | Candidate levers |
|---|----------|----------------|------------------|
| Q263 | Union `motif_graph_keys` into `resolve_motif_keys` read? | [`motif_keys.py`](../nest_graph/propose/motif_keys.py) | **Locked as Q279** — mix_floor + MIS boosts |
| Q264 | Cheap cache hit skips inject — stale cohort AMAF? | [`cheap_pack.py`](../nest_graph/decision/cheap_pack.py) | Cache key + `arch_n` |
| Q265 | `compose_sz>0`, `join_n=0` — Q243 blocker? | pre-bind / C1b | NFP-lite; C2 |
| Q266 | R6 vs Q223 pattern retain on hull reject | Q213/Q214/Q223 | Relative patterns only |
| Q267 | Repack → MotifJoin next-iter latency | post-bind order | Bench correlation |
| Q268 | `member_hits` vs `materialized_motif` for MCTS | [`mcts.py`](../nest_graph/decision/mcts.py) | Q172 |
| Q269 | F gate baseline bump | [`benchmark_pipeline.py`](../scripts/benchmark_pipeline.py) | Net gain before update |
| Q270 | M3 PLACE_COHORT (Q90) | [`action_gen.py`](../nest_graph/decision/action_gen.py) | After Q243 |
| Q271 | `motif_locked` → cheap cache key | pack_cache | Trace outer vs cheap |
| Q272 | `block_cohort_swap` uplift with C1 | compose L804+ | telem |
| Q273 | improve_rules as DG macro | Q249 | Post-C4 |
| Q274 | Raycasting C++ / proposer perf | followup.txt | Separate track |
| Q275 | Q113 TTL crowds inject list? | MotifBase | `max_keep` |

### Telem symptom table (net-only agent)

| Symptom | Likely broken link | Read first |
|---------|-------------------|------------|
| `arch_n>0`, `cohorts_n=0` | No graph hit | C1 (Q255), C2 (Q258) |
| `cohorts_n>0`, `compose_sz=0` | Clearance | C3 (Q262) |
| `join_n>0`, `member_hits=0` | Kind tags | C1b (Q257) |
| Repack slow | Anchor / duplicate stamp | R4 (Q256), R5 (Q260) |
| Cheap refine ignores MotifJoin | Missing dg= | C5 (Q259) |

---

## Locked verdicts — Graph seeding, DG, raycast (Q278–Q300 — net-only)

**Status:** locked 2026-08-28 (GO). Supersedes open-research wording for listed Qs. Q291–Q295 remain open until S-track ships.

### Track A: Graph seeding & inject mismatch

| Q | Verdict | Implementation constraint |
|---|---------|---------------------------|
| Q278 | **Track A: upstream seeding** | Fix `graph_hit=0` by carrying a valid **world anchor** into archived patterns and seeding mix pins **before** `make_polygon_graph`. Do **not** rely on relaxed ε matching downstream — collision false-positives. |
| Q279 | **`mix_floor` → `resolve_motif_keys`** | Read `motif_graph_keys` inside [`resolve_motif_keys`](../nest_graph/propose/motif_keys.py). When MotifBase inject hits graph keys, those keys **must** feed `mix_floor` quota and MIS boosts or MWIS ignores motif investment. (Extends Q263.) |

### Track S & I: DG rule-score & macro augmenting paths

| Q | Verdict | Implementation constraint |
|---|---------|---------------------------|
| Q286 | **Wire `rule_id` (S1)** | [`active_rule_set`](../nest_graph/propose/selection_compose.py) must index by MCTS `rule_id`. All AMAF rule keys mapping to `rule_sets[0]` = fatal local minimum. |
| Q287 | **Multi-track dimensions** | Every AMAF dimension (`region`, `rule_id`, `motif_id`) must tangibly alter slave pipeline (proposers, scoring, mix/inject). |
| Q290 | **Plateau = mutation + browse + I1** | Plateau escape triggers Q249 `improve_rules`, browse (Q132), and `macro_increase_path` (I1). **No** second pack loop or arbitrary DFS retries. |
| Q296 | **Macro augmenting path (I1)** | Apply `increase_path_dfs` alternating-path template to MCTS. Mid-path ancestor swap (Rim→Void) ≡ geometric pose swap in MWIS. |
| Q297 | **Replay from snapshot only** | Macro swap (I1) uses [`pack_execute_snapshot`](../nest_graph/decision/cheap_pack.py) from ancestor. Mid-slave `DecisionArena` mutation violates Q163. |
| Q298 | **Loose search tolerance** | Allow transient reward dip during macro chain (analogous to `min_collisions > 0`); final leaf must strictly beat baseline lex reward. |
| Q299 | **Splice vs jump** | Browse jumps to new tip; I1 **splices** alternate policy mid-path while preserving suffix. Both required. |
| Q300 | **Symmetric fracture ε** | MotifJoin fracture ε at Pose tier (Q166) symmetric to AMAF/`member_hits` penalty at Decision tier when evaluating macro swaps. |

### Track Q274: Geometry-first raycasting

| Q | Verdict | Implementation constraint |
|---|---------|---------------------------|
| Q281 | **`clip_ray_interior`** | Native `Geometry.clip_ray_interior` for point-ray region clip. Do **not** misuse `cast_slide` or `find_polygon_intersections`. |
| Q282 | **Region SoT (v1)** | `from_shapely` exactly **once** per proposer invocation. No per-ray region rebuild. |
| Q283 | **R1 parity required** | Bit-exact parity vs Shapely `LineString.intersection` on L-shapes and concave fixtures in Catch2 before ship. |
| Q284 | **Reuse `contains_point`** | Voronoi and erosion inside tests via `region_g.contains_point()`. No duplicate Shapely `fit_shape.contains(p)`. |
| Q285 | **No mid-loop `from_shapely`** | R8 ban: anchors via native `boundary_rings()` only; bypass `get_shape_exteriors` Shapely in hot loop. |

### Still open — adaptive placement scoring (Q291–Q295)

| Q | Verdict / question | Research hooks | Candidate levers |
|---|-------------------|----------------|------------------|
| Q291 | Rule quality = mean vertex score at count tie; evolution uses lex before raw count? | [`evaluate_rules.cc`](../nest_graph/elem_graph/selection/evaluate_rules.cc), [`evolve.py`](../nest_graph/rules/evolve.py) | Enable `mean_score_weight`; zone-lower `count_weight` on large_void |
| Q292 | MCTS `leaf_reward` may add small rule-score term **after S1**; coverage/void stay primary on large_void? | [`mcts.py`](../nest_graph/decision/mcts.py) `leaf_reward`, Q108 λ | Extend `BoardSnapshot` with mean rule score |
| Q293 | Rule mutation = Q249 plateau (Decision B); rule selection = `rule_id` (Decision A); Q273 = explicit macro when both fail? | [`build_graph.py`](../nest_graph/build_graph.py) L1475–1494 | Post-M3; cap K≤3 (Q247) |
| Q294 | `increase_score_dfs` / normalized Δscore = quality refine levers; count-first restore only when score δ ≤ ε? | [`heavy_polish.py`](../nest_graph/propose/heavy_polish.py) Q245–246 | `refine_score_accept` telem gate |
| Q295 | Emit RULE_HYBRID follows active preset index; not substitute for compose `selection_geom_weight`? | [`ranking.py`](../nest_graph/propose/ranking.py) | S1 then align hybrid rank with `rule_sets[rid]` |

### F gate (Q280 — open, codify Q269)

| Q | Question | Candidate levers |
|---|----------|------------------|
| Q280 | F gate `--update-baselines` gain predicate | `indep` + `overlap` + area ≥ max(baseline, snapshot); no blind overwrite |

### Telem symptom table — scoring & macro path (net-only)

| Symptom | Likely broken link | Read first |
|---------|-------------------|------------|
| MCTS tries `rule_id>0`, nest identical | S1 not wired | Q286, `active_rule_set` |
| AMAF spread on rule_id, area flat | Hollow rule diversity | Q286, Q291 |
| `refine_score_accept=0`, count-tie area flat | Q245–Q246 not firing | Q294, `heavy_polish.py` |
| Plateau, zone diversity OK, cov flat | No macro path / browse | Q299, Q290 |
| `graph_to_nest` bottleneck, AMAF miss only | Blocker depth unknown | Q296, I0 telem |
| `macro_path_accept=0` after I1 | S1 or replay gate | Q297, Q286 |

---

## Cross-track research themes (net-only agent backlog)

Use when planning letters; cite Q numbers above; do not lock in root AGENTS.md until shipped.

| Theme | Core question | Depends on | Bench tags |
|-------|---------------|------------|------------|
| **Archive→graph handoff** | Why `graph_hit=0` when `arch_n>0`? | **Q278** (upstream seed), **Q279** (read path) | void_fill: `arch_n`, `graph_hit`, `archive_mix_floor_hits` |
| **Rule_id real** | Does each AMAF rule key change nest/score? | **Q286**, S1/S2 — **before Track A** | `mcts_rule_id`, AMAF visits 0..K |
| **Quality-without-count** | Flat parts + better score/density? | Q291–Q295 (open), Q245–Q246 | `refine_score_accept`, mean vertex score |
| **Macro augmenting path** | Can mid-path sibling swaps beat UCB-only plateau? | **Q296–Q300**, S1, I1 | `macro_path_accept`, `macro_swap_depth` |
| **Raycast native** | Is Shapely clip the propose bottleneck? | Q281–Q285, Q274 | `propose_ray_ms`, `from_shapely_count` |
| **Cohort macro** | When is M3 safe to ship? | Q243, Q270, Q252, **Q370** | `compose_sz≥2`, `join_n>0`, `member_hits>0` |
| **Hybrid compose handoff** | Does one `motif_locked` SoT survive compose→refine→materialize? | **Q361–Q372**, Q360 | `compose_hold`, `mat_motif`, `hollow_pattern_locks` |
| **F gate honesty** | When may baselines update? | Q269, Q280 | `--gate` indep + overlap + net area |

**Research loop (all themes):** telem names stage → read named hot path → one unify patch → re-gate. Miss/degradation → loop same letter; do not stack parallel mechanisms (root AGENTS.md Planning).

---

## Follow-up research — plan audit (Q301–Q306 — net-only)

**Status:** open. From 2026-08-28 consistency review + code validation.

| Q | Question | Code fact | Candidate lever |
|---|----------|-----------|-----------------|
| Q301 | Q249 says reward ≥ parent; code uses cov plateau? | [`build_graph.py`](../nest_graph/build_graph.py) L1991 `cov >= plateau.last_cov` | Align P1 trigger with MCTS parent reward or document v1 |
| Q302 | C2 archive mix exists but never hits? | [`transform_batch.py`](../nest_graph/propose/transform_batch.py) L710–757; [`record_to_cluster_pattern`](../nest_graph/propose/pattern_archive.py) L33 `(0,0,0)` | Track A: world `ref_transform` on upsert |
| Q303 | `motif_graph_keys` absorbed in resolve? | [`motif_keys.py`](../nest_graph/propose/motif_keys.py) — **no** `_absorb(motif_graph_keys)` | Q279 Phase 1b |
| Q304 | I1 needs ancestor path walk? | [`runner.py`](../nest_graph/decision/runner.py) has no `ancestors()` | Add parent-id walk helper |
| Q305 | Macro replay = cheap only? | [`cheap_pack.py`](../nest_graph/decision/cheap_pack.py) `pack_execute_snapshot` | I1 v1 cheap; Q306 if insufficient |
| Q306 | Where is rule_id actually broken? | Outer [`build_graph.py`](../nest_graph/build_graph.py) L1236 `_mcts_rule_ids` OK; compose L1542 + [`execute.py`](../nest_graph/decision/execute.py) `(0,)` not | S1 all compose sites + S2 execute |

**Mix floor gate (Q302 extension):** `motif_floor_n` requires `on_plateau || is_last_leaf` ([`transform_batch.py`](../nest_graph/propose/transform_batch.py) L683–688). Track A telem should report `archive_mix_floor_hits` on **plateau iters** first; do not expect hits every iter.

---

## Bench + telem gates (Q307–Q308 — net-only)

| Q | Verdict | Implementation constraint |
|---|---------|---------------------------|
| Q307 | **Wire gate telem before phase 0.5** | Merge `mcts_rule_id`, `archive_mix_floor_hits`, `refine_score_accept`, `macro_path_*` into void_leak + [`benchmark_pipeline.py`](../scripts/benchmark_pipeline.py) print — one assembly gate ([`telem.py`](../nest_graph/propose/telem.py) / [`pack_loop.py`](../nest_graph/decision/pack_loop.py)) |
| Q308 | **Every phase bench-gated** | No letter lock without `--gate` pass + letter telem present + area ≥ phase snapshot; miss → symptom table → hot path → re-bench (AGENTS.md loop) |

---

## Follow-up research — DG audit (Q309–Q313 — net-only)

| Q | Question | Verdict |
|---|----------|---------|
| Q309 | Cheap multi-sim AMAF vs outer pack? | `cheap_outer_reward_delta` in `mcts_telem` / void_leak after outer expand |
| Q310 | BoardSnapshot stale after browse? | Log `browse_jump`; `packed_gids_compatible` gate unchanged |
| Q311 | `leaf_reward` + rule scores? | `member_hits` / `materialized_motif` terms in `leaf_reward` + AMAF bias (M5) |
| Q312 | Inner sim MotifBase upsert? | Defer; Q144 outer-only stands |
| Q313 | Rolling phase snapshots | [`docs/phase_snapshots.json`](../docs/phase_snapshots.json); any metric drop → research loop |

---

## Locked verdicts — explorer, gate handoff, followup (Q314–Q335 — net-only)

**Status:** locked 2026-08-28 (GO). Do not lock in root AGENTS.md until letters ship.

### Explorer C++ migration (Q314–Q322)

| Q | Verdict | Implementation constraint |
|---|---------|---------------------------|
| **Q314** | **Flip to native primary** | After pytest parity (Q283), delete Shapely `intersection` hot loop in [`placements_geo.py`](../nest_graph/propose/placements_geo.py). Native `clip_ray_interior` is sole SoT. |
| **Q315** | **Cache on `ProposeGeometry`** | Store `region_g` on propose object; one `from_shapely` per proposer invocation (Q282). |
| **Q316** | **Use `native_geoms`** | Do not `get_shape_exteriors` on packed parts; harvest rim anchors via `boundary_rings` on `NestState.native_geoms`. |
| **Q317** | **Filter-only sufficient** | R2a `contains_point` only; no C++ Voronoi diagram — inclusion test is the bottleneck. |
| **Q318** | **Mostly identical** | After `search_region`, explorer filters match `region_g`. **Exception:** `free_space_cloud` uses `yield_poly` — parity-test before unify (Q334). |
| **Q319** | **Match Shapely polylabel** | `polylabel_rings` mirrors pole-of-inaccessibility; Shapely `representative_point` fallback on degenerate rings only. |
| **Q320** | **Batch flat (`anchor×ray×frac`)** | Not per-anchor batch; minimizes Python↔C++ overhead. |
| **Q321** | **`propose_ray_ms` is SoT** | R1/R3 optimization letters; `peak_ray` is emit volume not timing. |
| **Q322** | **Standalone harness** | `benchmark_propose_ray.py` for micro-timing; pipeline `prop_accept` too noisy for binding profiling. |

### Gate / Track A handoff (Q323–Q327)

| Q | Verdict | Implementation constraint |
|---|---------|---------------------------|
| **Q323** | **World pose of archive leader** | `ref_transform` = `gid_a` world pose at accept; not centroid. |
| **Q324** | **`build_graph` + repack** | `note_motif_ref_anchors` before inject AND after `upsert_from_repack_accept`. |
| **Q325** | **Exact round-4 match only** | No ε-injection (Q278); mix_floor pins must exact-match graph keys. |
| **Q326** | **Colonization sufficient** | 0.585 via `graph_to_nest` / `nest_void_term_hits`; no illegal overlaps. |
| **Q327** | **Scene is overlap SoT** | `post_pack_overlap_ok` guards local SE2; `emit_packing_clear` is propose-only. |

### Followup themes (Q328–Q332)

| Q | Verdict | Implementation constraint |
|---|---------|---------------------------|
| **Q328** | **Keep growing `is_pose_clear`** | Q18 stands; batch stamp on cluster_repack loses early-exit. |
| **Q329** | **M3 PLACE_COHORT deferred** | Beats sequential_accept only for K≥4 complex motifs; ship after A2 green. |
| **Q330** | **Ship Q294 parallel** | `refine_score_accept` telem with S3; proves rule scoring moves MWIS before Q273 macro. |
| **Q331** | **Tier-scaled ε** | MotifJoin geometric ε (~1e-4) ≠ Decision macro-path AMAF penalty (~0.05). |
| **Q332** | **Both** | Microbench authorizes C++ merge; `--gate` area/time authorizes ship. |

### Plan review additions (Q333–Q335)

| Q | Verdict | Research hooks |
|---|---------|----------------|
| **Q333** | **Shipped (sequential)** — archive first, cluster_copy fills remainder when rows `< motif_floor_n` | [`transform_batch.py`](../nest_graph/propose/transform_batch.py) `_motif_mix_floor_rows` |
| **Q334** | One **`region_g` per void** at pipeline explorer stage | [`pipeline.py`](../nest_graph/propose/pipeline.py) explorer stage |
| **Q335** | **Full C++ Voronoi — REJECT v1** | R2a `contains_point` only; ~0.05–0.19 s/call; off in void_seek; AGENTS ban |

**Q335 detail:** Native geometry has no GEOS. Full port = Boost.Polygon (~1–2 weeks) or GEOS link (~2–4 weeks). Nest only needs Voronoi **vertices inside region** (filter with `contains_point`), not clipped cells.

---

## Locked verdicts — gate pass DG (Q336–Q357 — net-only)

**Status:** locked 2026-08-28 (GO). Append-only reference for gate-pass letters; standing rules stay in root [AGENTS.md](../AGENTS.md).

### Track A2: Archive mix → graph hit (Q336–Q340)

| Q | Verdict | Implementation constraint |
|---|---------|---------------------------|
| **Q336** | **YES — relax gate** | `on_plateau && free_kind==large_void && archived_patterns` triggers archive mix regardless of `mcts_zone`. |
| **Q337** | **Disable polish for mix** | `polish=False` on archive inject when seeding mix pins; NFP-lite breaks Q325 exact match. |
| **Q338** | **Keep packing clear** | Mix pins use `clear_of_geoms` / packing clear; Scene margin only at compose stamp. |
| **Q339** | **Prepend + protect pins** | Prepend archive pins to `sel`; telem `archive_mix_pin_survive_n`. |
| **Q340** | **Pin leader AND follower** | Mix pins leader (`gid_a`, `ref_transform`) and follower (`gid_b`, composed world pose). |

### Track V1: Colonization & area floor (Q341–Q344)

| Q | Verdict | Implementation constraint |
|---|---------|---------------------------|
| **Q341** | **Cap void_scale at 5.0** | Do not push to 6.0 — warps MWIS. |
| **Q342** | **YES — soften hold** | `void_override` on area tie + higher void count when `hollow_miss && large_void && on_plateau`. |
| **Q343** | **V1 + A2 both required** | V1 fixes ~0.580 floor; A2 unlocks motif compose headroom for ≥0.585. |
| **Q344** | **Lower colonize margin** | On `on_plateau && large_void`, colonize margin **0 or 1**. |

### Track S: Rules & AMAF (Q345–Q348)

| Q | Verdict | Implementation constraint |
|---|---------|---------------------------|
| **Q345** | **Export in backprop** | `arena.amaf_visits(region, rid, motif_id)` → `rule_id_amaf_{rid}` telem. |
| **Q346** | **Deep wire required** | `rule_id` 0 vs 1 must change compose scores on cheap and heavy paths. |
| **Q347** | **Delete duplicate** | Single SoT from [`rules/evolve.py`](../nest_graph/rules/evolve.py). |
| **Q348** | **Defer mean-score** | No Q291 mean-score until S1 proves `rule_id` telem moves MWIS. |

### Track I: Macro path & browse (Q349–Q352)

| Q | Verdict | Implementation constraint |
|---|---------|---------------------------|
| **Q349** | **Apply with overlap gate** | `mcts_action = alt_action` only if cheap replay passes `post_pack_overlap_ok`. |
| **Q350** | **Keep both** | Browse = diversity; macro_path = mid-path sibling splice. |
| **Q351** | **Default OFF** | `enable_macro_path_replay: bool = False` in ProposeConfig. |
| **Q352** | **Move assignment** | `agent.motif_cohorts` post-inject, not pre-`propose_stats`. |

### Cross-track & infrastructure (Q353–Q357)

| Q | Verdict | Implementation constraint |
|---|---------|---------------------------|
| **Q353** | **YES — small AMAF boost** | Bias `_amaf_pick_score` when prior iter `motif_graph_hit_n > 0`. |
| **Q354** | **M3 needs compose_sz≥2** | PLACE_COHORT blocked until A2 fully green. |
| **Q355** | **Evaluator parity** | Port S1/I1 telem to [`nesting_evaluator.py`](../scripts/nesting_evaluator.py). |
| **Q356** | **Active-index intersect** | Dense OOM: separate letter; does not block void_fill. |
| **Q357** | **Defer region_g** | Defer until A2/V1 gate passes. |

**Q303 correction:** Q279 shipped — `resolve_motif_keys` absorbs `motif_graph_keys` in [`motif_keys.py`](../nest_graph/propose/motif_keys.py).

### Q313 snapshot + Q262 hybrid lock (2026-08-28)

| Item | Verdict |
|------|---------|
| **Q313** | Updated [`docs/phase_snapshots.json`](phase_snapshots.json) void_fill: area **0.564**, parts **53**, `graph_hit≈23`, `arch_mix>0`. Fixture floor 0.585 treated **passed near-threshold** (user). |
| **Q262 hybrid** | Scene locks = growing Scene **subset ≥2** only. Packing-clear on `large_void` → **MIS soft boost** (`motif_packing_score_boost_idxs`), never lock/beam. Void-only packing clear = regression (overlap + area drop) — do not re-enable. |
| **Archive mix** | Pins → `proposal_pins` always; prepend to `sel` only on plateau\|last_leaf. |
| **Cohort soft steer** | `graph_hit>0` → cohort member keys get `motif_w×0.35` in `apply_void_selection_boosts`. |
| **Next** | Higher-level compose / MotifJoin (`join_n`, Scene-valid `compose_sz≥2`); M3 PLACE_COHORT still gated on compose_sz (Q354). |

**Q255 evaluator parity (shipped):** [`nesting_evaluator.py`](../scripts/nesting_evaluator.py) had **bind before inject** — MotifJoin never saw cohorts (`join_n=0`). Swapped to **inject → bind** matching [`build_graph.py`](../nest_graph/build_graph.py). Bench now `join_n=5–12`, occasional `mat_motif>0` / `pack_boost>0`. Still open: `mat_motif=0` on many seeds (join edges fracture in refine / MWIS) and Scene `compose_sz=0`.

### Q358–Q359 cluster_copy hollow bridge (2026-08-29)

| Item | Verdict |
|------|---------|
| **Q358** | Pool→MIS gap: `project_proposer_keys` + honest pool telem + unified `apply_void_selection_boosts` survivor merge. |
| **Q359** | Hollow `graph_to_nest`: `merge_motif_cohorts` → `sequential_accept`; when `cc_n=0`, `scene_pair_locks_from_indices` on emit **∪ cohort `member_keys`** (Scene pair lock, not packing-clear lock). Fallback: score re-boost retry. Win if `cc_n>0` or void-fill rise with ≥88% area. `compose_motif_hold` when pair lock survives incumbent hold. |
| **Q360** | **Unified hollow gate** in [`motif_lock.py`](../nest_graph/propose/motif_lock.py): `cluster_copy_lock_pool` (motif_keys) → `resolve_hollow_cc_lock_sets` → cohort-first `hollow_pattern_lock_sets` (Scene growing subset per stamped cohort, flat pair fallback) → shared `_beam_locks` / `hollow_cc_lock_wins` / `hollow_cc_score_steer`. No third store. `void_core_then_rim(seed_core=locked_motif)` when hollow. |
| **Bench** | void_fill avg **0.565** (seeds 0–2); seed 0 **0.567**; seed 2 `mat_motif=2`. Gate 0.585 still open. |
| **Next** | Scene full cohort accept (`compose_sz≥2`); refine hold for MotifJoin members; `best_pack` in `build_graph`; followup tracks (repack, DG macro, C++ ray). |

---

## Open research — Hybrid compose handoff (Q361–Q372 — net-only)

**Status:** **LOCKED 2026-08-29** (pre-H1 ship). Supersedes open-research wording below. Hybrid H1–H3 plan authoritative.

### Locked verdicts — Hybrid compose handoff (Q361–Q373)

| Q | Verdict | Implementation constraint |
|---|---------|---------------------------|
| **Q361** | **Augment cautiously** | Post-hollow `trial_packed` includes `selected_nest` geoms **only** if index passes `anchored_nest_indices` (`is_board_adj` or contact-connected to board_adj/seed via [`cluster_contact_components`](../nest_graph/propose/context.py)). Never lock motifs against floaters. Telem: `trial_packed_anchored_n`, `trial_packed_float_skip_n`. |
| **Q362** | **Overlap > CC > Void > Area** | Single `hybrid_compose_pick` in [`motif_lock.py`](../nest_graph/propose/motif_lock.py). `_packing_independent` = **hard gate**. CC survival (≥0.90× area) and void rise (≥0.88× area) are **soft** overrides over raw area lex. Delete duplicate predicates in `selection_compose`. |
| **Q363** | **`propose_stats["motif_locked"]` SoT** | Refine/finalize read post-compose locks only. Remove `ctx.locked_indices` reads in [`pack_loop.py`](../nest_graph/decision/pack_loop.py) refine_fn and [`cheap_pack.py`](../nest_graph/decision/cheap_pack.py) refine path. Telem: `refine_lock_n`. |
| **Q364** | **Absolute lock protection in void_core** | [`void_core_then_rim`](../nest_graph/propose/void_selection.py) `seed_core=motif_locked`; rim fill must not displace locked members. |
| **Q365** | **Post-pick compose_sz** | `motif_compose_accepted_size = len(motif_locked)` after `hybrid_compose_pick` resolves — not at beam generation. One writer. |
| **Q366** | **Hold with escape hatch** | [`apply_refine_with_restore`](../nest_graph/propose/heavy_polish.py): no restore if refine drops lock member **unless** refine area ≥ **1.02×** locked-nest area AND independent. Telem: `refine_lock_hold`, `refine_lock_escape`. |
| **Q367** | **Void-fill tie-break retained** | Count-tie + void-fill rise → accept refine, reject restore (existing Q192). |
| **Q368** | **Independence before accept** | Every lock win through `hybrid_compose_pick` requires `_packing_independent` (Q262 Scene path only for locks). |
| **Q369** | **Cache key hygiene** | Extend [`cheap_pack_cache_key`](../nest_graph/decision/cheap_pack.py) with `compose_sz` or hash of accepted lock tuple before H3. Current key `(zone, motif_id, rule_id)` is insufficient (code fact L20–29). |
| **Q370** | **`cohort_sig` in AMAF** | Extend [`mcts.py`](../nest_graph/decision/mcts.py) `_action_key` to `(region, rule_id, motif_id, cohort_sig)`; sig matches [`merge_motif_cohorts`](../nest_graph/propose/motif_keys.py) `(motif_id, leader_gid, leader_key)`. |
| **Q371** | **Index-based best_pack** | `maybe_restore_best_pack` operates on **`list[int]` graph indices** (`BestPackSnapshot`), not geometry alone. Overlap-check before restore. |
| **Q372** | **Steer first, lock last** | Q262: `motif_packing_score_boost_idxs` **pre_nest only**; Scene hard locks **post_hollow only**. No packing-clear → `motif_lock_sets`. |
| **Q373** | **Evaluator index fix** | [`nesting_evaluator.py`](../scripts/nesting_evaluator.py) currently stores `best_next_sel=range(len(next_polys))` (nest order) — **wrong SoT**. Track `best_selected_polys` from graph `selected_polys` when extracting shared helper. |

### Telem gate — hybrid handoff (net-only)

**Bench must print** (letter or diag): `hybrid=lock:{compose}/{refine}/{mat}` plus stage-specific keys below. Letter pass requires funnel telem present (AGENTS.md).

| Stage | Required keys | Pass signal |
|-------|---------------|-------------|
| H1 compose | `lock_n_compose`, `motif_scene_max_sz_post`, `hollow_pattern_locks`, `hybrid_pick_wins` | `lock_n_compose≥2` OR `hollow_pattern_locks>0` on void_fill |
| H2 refine | `lock_n_refine`, `refine_lock_hold`, `lock_survive_refine`, `best_pack_restore` | `lock_survive_refine=1` OR intentional `refine_lock_escape` |
| H3 DG | `cache_key_compose_sz`, `cohort_sig_amaf_visits` | AMAF visits when `compose_sz≥2` |

**Fault signatures:** `lock_n_compose>0` + `lock_n_materialize=0` → refine/materialize drop (Q366). `trial_packed_float_skip_n=0` + `independent_ok=false` → Q361 leak. `hybrid_pick_reject_indep` high → Q368.

### Research archive (pre-lock questions)

| Q | Question | Research hooks | Candidate levers |
|---|----------|----------------|------------------|
| Q361 | Post-hollow Scene: is pre-nest `packed_geoms` (seed-only) the main `seq_clear` cause? | [`motif_lock.py`](../nest_graph/propose/motif_lock.py) `_growing_subset_indices`, [`selection_compose.py`](../nest_graph/propose/selection_compose.py) compose call order | H1: nest-augmented `trial_packed = packed_geoms + selected_nest geoms`; compare `motif_scene_max_sz` pre vs post |
| Q362 | Should `hybrid_compose_pick` use one lex gate or keep void/cc area floors (0.88/0.90)? | Q240, Q359 void-fill accept, [`block_replace.py`](../nest_graph/propose/block_replace.py) `lex_count_area_better` | Single helper in `motif_lock.py`; delete duplicate predicates in `selection_compose` |
| Q363 | Does refine always see **stale** locks (`ctx.locked_indices` pre-compose)? | [`pack_loop.py`](../nest_graph/decision/pack_loop.py) L258, [`build_graph.py`](../nest_graph/build_graph.py) L1473 | H1: `propose_stats["motif_locked"]` post-compose SoT; telem `refine_lock_n` |
| Q364 | When both pair-lock and `void_core_then_rim` fire, which wins — lock beam or void-first MIS? | [`selection_compose.py`](../nest_graph/propose/selection_compose.py) order: hollow beam → void_core | Hybrid: `seed_core=locked_motif` only; lex pick void_core vs locked nest, not separate overrides |
| Q365 | What telem is SoT for `compose_sz` — `motif_compose_accepted_size`, `motif_sequential_full`, or pair-lock len? | [`pack_loop.py`](../nest_graph/decision/pack_loop.py) leak merge, bench print | One writer: max lock set size from `compose_motif_pipeline`; grep before second telem |
| Q366 | Does `join_n>0` + `mat_motif=0` mean refine drop or finalize drop? | [`heavy_polish.py`](../nest_graph/propose/heavy_polish.py) restore, [`epoch.py`](../nest_graph/decision/epoch.py) `materialize_selection` | Trace locked member survival nest→refine→finalize; telem `lock_survive_refine` |
| Q367 | Q242: restore on incumbent hold clears `locked_motif` when `motif_sequential_full=0` — does pair-lock (`cluster_copy_pair_lock`) survive? | [`selection_compose.py`](../nest_graph/propose/selection_compose.py) incumbent hold L950+ | Extend hold predicate: `compose_motif_hold` OR `cluster_copy_pair_lock`; match Q242 |
| Q368 | Seed-2 `overlap_ok` fail: hollow win via void-fill path without `_packing_independent` check? | [`motif_lock.py`](../nest_graph/propose/motif_lock.py) `hollow_cc_lock_wins` | H2: mandatory independence gate on every lock win before accept |
| Q369 | Cheap cache (`Q264`): stale `motif_locked` / cohort sig when compose changes locks mid-outer-iter? | [`cheap_pack.py`](../nest_graph/decision/cheap_pack.py) cache key, `_pack_cache` in build_graph | Include `motif_locked` tuple + `motif_compose_accepted_size` in cache key before H3 AMAF |
| Q370 | H3 `cohort_sig` for AMAF: `leader_key`, `pattern_sig`, or `(motif_id, leader_gid, leader_key)`? | [`action_gen.py`](../nest_graph/decision/action_gen.py) L55, Q238 Q252 | Dedupe matches [`merge_motif_cohorts`](../nest_graph/propose/motif_keys.py) sig |
| Q371 | `best_pack_restore` evaluator-only — does build_graph lose ≥0.5% cov on last iter post-pack? | [`nesting_evaluator.py`](../scripts/nesting_evaluator.py) L1397+, [`build_graph.py`](../nest_graph/build_graph.py) outer loop | H2: shared `maybe_restore_best_pack`; telem `cov_regress` vs `best=1` |
| Q372 | Packing-clear soft steer + Scene lock on **same cohort**: order — boost pre-nest, lock post-hollow only (Q262)? | Q262, Q327, [`sequential_accept_motif_cohorts`](../nest_graph/propose/motif_lock.py) | Confirm no packing-clear path writes `motif_lock_sets`; telem `pack_boost` without `beam` |

### Telem symptom table — hybrid compose (net-only)

| Symptom | Likely broken link | Read first |
|---------|-------------------|------------|
| `seq_clear>0`, `compose_sz=0`, `beam=0` | Pre-nest Scene context | Q361, `sequential_accept` vs post-hollow |
| `hollow_pattern_locks>0`, `compose_sz=0` | Lock trial lost lex / not beamed | Q362, `_beam_locks`, `beamed_sigs` |
| `join_n>0`, `mat_motif=0`, `compose_hold=0` | Refine stale locks | Q363, Q366, `pack_loop` refine_fn |
| `cluster_copy_pair_lock=1`, `mat_motif=0` | Refine restore dropped locks | Q366, Q367, `apply_refine_with_restore` |
| `cc_g≥2`, `cc_n=0`, `hollow_cc_gap=1` | Pattern pool empty or Scene fail | Q361, `cluster_copy_lock_pool` |
| `cov_regress>0`, `best=0` | No peak retention in build_graph | Q371 |
| `overlap_ok=false` after hollow win | Lock accept without independence | Q368 |
| `independent_ok=false` after H1 | trial_packed augmented with floaters | Q361 |
| `refine_lock_escape=1` | Global win broke motif intentionally | Q366 (OK if area ≥1.02×) |
| `cohorts_n>0`, M3 no AMAF visits | Cache stale or H3 not wired | Q369, Q370 |

### Cross-track theme (add to backlog)

| Theme | Core question | Depends on | Bench telem |
|-------|---------------|------------|-------------|
| **Hybrid compose handoff** | Does one `motif_locked` SoT survive compose→refine→materialize? | Q361–Q372, Q360, Q255 | `compose_sz`, `compose_hold`, `mat_motif`, `lock_survive_refine`, `hollow_pattern_locks` |

## Locked verdicts — void-fill DG loop (Q374–Q390 — net-only)

**Status: locked 2026-08-29.** Closes poisoned Master↔Slave handoff after H1–H3. Order: H4 → U1 → D1 → I1 → M4 → S1. No U1 until H4 green.

### H4 — Honest MotifJoin handoff

| Q | Verdict | Constraint |
|---|---------|------------|
| **Q374** | **Remove void pins** | `motif_locked` only from hybrid pick/beam wins, `colonize_pinned`, or block swaps. Delete `void_pins` census. |
| **Q375** | **`lex_count_area_better`** | Colonize accept only via lex — no part-count inflate at flat area. |
| **Q376** | **Lex or void rise on escape** | Keep Q366 1.02× + Q368 indep; **ADD** `lex_count_area_better OR void_fill_rise`. |
| **Q377** | **Q362 + void rise** | `graph_to_nest_hollow && pick==0` → unlocked re-nest; accept 0.88× + void rise. No packing-clear locks (Q262/Q372). |

### U1 — Rewrite DecisionGraph API

| Q | Verdict | Constraint |
|---|---------|------------|
| **Q378** | **One typed API** | `PathNode` / `PathKind` / `neighbors` / `conflicts` / `realize` / `survive_counts`. No `path_graph.py`, no alias wrappers. |
| **Q379** | **Purge old symbols** | Delete `materialize_selection`, `MaterializeStats`, `motif_conflicts`, `attach_conflicts`, `attach_motif_conflicts`. Bindings = C++ names. |
| **Q380** | **Mutex via `conflicts`** | `mutex_n` implemented through `conflicts(PathNode, PathNode)`. |

### D1 — Survival loop

| Q | Verdict | Constraint |
|---|---------|------------|
| **Q381** | **Single copy to agent** | `dg.realize(selected)` sole writer; copy `survive_counts()` once → `agent.realized["survive_by_motif"]`. No Python MotifJoin recount. |
| **Q382** | **Credit survivors only** | `credit_accept` only if `survive_counts[mid] > 0`. |
| **Q383** | **AMAF survival blend** | Soft `survive_by_motif[mid]/sel_n` in leaf/AMAF; also Q353 boost if prior `motif_graph_hit_n > 0`. |

### I1 — Path splice

| Q | Verdict | Constraint |
|---|---------|------------|
| **Q384** | **Overlap on replay** | Fix Q349: `alt_action` only if cheap replay passes `post_pack_overlap_ok` (independence). |
| **Q385** | **Rank by survival** | Soft-prefer `neighbors` with high `survive_counts` / realized MotifJoins. |
| **Q386** | **Validate Join/Pose only** | MotifJoin/Pose/Attach steps: `realized` / `poses_of` only — **no** pack execute. |

### M4 & S1

| Q | Verdict | Constraint |
|---|---------|------------|
| **Q387** | **Stamp full member_keys** | PLACE_COHORT stamps full cohort so MotifJoin star is on DG for walk. |
| **Q388** | **Keep Q90** | No nested MotifBase / super-cohort merge; pairs only. |
| **Q389** | **Masked path-accept upsert** | On accept: `upsert_from_contacts` masked to MotifJoin-realized ∪ (`motif_locked` ∩ selected). Not multi-sim. |
| **Q390** | **Warm on accept** | `remember_related(snapshot)` + bump warm motif ids for accepted chain. |

