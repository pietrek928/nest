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

