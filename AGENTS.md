# Agent instructions (nest)

Standing rules for coding agents. Prefer this file over chat memory for repo conventions.
Human docs: [README.md](README.md). Deeper domain archive (not always-on): [docs/agent-domain-notes.md](docs/agent-domain-notes.md).

## Commands

```bash
# Python deps / rebuild native after C++ edits
uv pip install -e .

# Or targeted native rebuild
cmake --build build --target geometry elem_graph

# Python tests
uv run pytest tests/ -q

# C++ tests (configure once with -DNEST_GRAPH_BUILD_TESTS=ON)
cmake -S . -B build -DNEST_GRAPH_BUILD_TESTS=ON
cmake --build build --target geometry_cpp_tests elem_graph_cpp_tests
```

Native sources are listed in `tool.uv.cache-keys`.

## Code style

### Python

- Python 3.12+: **never** add `from __future__ import …` (including `annotations`).
- Prefer `X | None`; quote forward refs only when required.
- Imports **only** at module top. No mid-function / mid-branch `import` / `from … import`
  (including inside `run_build_graph`). Exception: true circular-import break with a one-line
  comment naming the cycle — prefer restructuring over lazy import.
- **No nested `def`** unless there is a concrete reason (closure over locals that cannot be
  args, or a one-shot callback required by an API). Prefer module-level / file-scope helpers.
  Do not nest helpers inside `run_build_graph` / propose hot paths for “locality.”

### C++

- No custom namespaces (no anonymous `namespace {…}`, no helper namespaces). Helpers at file scope in `.cc` or `inline` in headers.
- Namespace aliases for third-party APIs are fine (`namespace nb = nanobind;`).
- Avoid `static` on functions unless needed for linkage; prefer shared `inline` helpers in `internal/internal.h`.

### Git

- Do not commit unless the user asks.

## Planning

Do this **before locking a plan and before each implementation stage**. Not a finishing checklist.

- **Dedup / one gate.** Grep for the predicate you are about to add (lex hold, round-4 keys, prepend/union, zone skip, gravity vector, mix niches, rim restore). The plan must cite the existing function. Do not add a second helper, skip site, or restore path. Permission (`ZONE_PROPOSERS`) stays distinct from staging (`packed_n` / void / `use_*` flags). One truncation/cut per pipeline.
- **Validate vs code.** Check comments that already claim the behavior against the actual condition. Check plan claims against live helpers (`transform_row_key`, `lex_count_area_better`, `part_extents`, `placement_obstacles`, …). Prefer extending a named path over a parallel RCL/beam/hold.
- **Stage + bench.** Split the plan into letters. After each: smallest relevant `pytest`; `uv run python scripts/benchmark_pipeline.py --tags <case> --seeds 0 --propose shipped --gate`; if propose/mix/nest changed, `NEST_BUILD_GRAPH_ITERS=2 uv run python -m nest_graph.build_graph`. Snapshot a baseline before the first letter.
- **Miss → improvement loop (same letter).** Hard stop only if `independent_ok=false`. Miss if quality < 0.9× best-so-far this run (and not below 0.9× last shipped bench for that tag), or time >1.5× with no quality gain, or the letter’s expected telem is absent. On miss: **loop** — research → patch → re-gate — until the letter passes or the user redirects. Do **not** start the next letter or pile a new parallel mechanism while looping.
- Historical Qs stay in [docs/agent-domain-notes.md](docs/agent-domain-notes.md). Do not lock one-off Q-numbers in this file.

### Improvement loop (research + unify)

During each miss cycle and while a letter is still open:

- **Stuck → research, then unify.** Do not stack another special case. Read telem + the named hot-path functions; look for a better existing lever or a **hybrid** of two winners (shared helper, lex pick, soft scale). Optimize / simplify the current path before inventing a third.
- **Do not alter the problem.** Never make a gate easier by changing the fixture: no larger/smaller board, no catalog/demand/mix/iters retune, no lowered floors. Restore any such edits. Work the algorithm on the original case.
- **Simplify while looping.** Complex, duplicated, or deeply nested logic makes the next optimization impossible. If the path is getting hard to follow, **stop adding features** and unify/flatten first (one gate, one helper, delete dead flags).
- **Validate logic as you go.** Periodically check that comments, conditions, and call sites still match (hold vs override, colonize vs pin, floor vs actual predicate). Catch contradictions early — do not wait for the letter to finish.
- **Research for improvements.** Read telem + named functions on the hot path; form one hypothesis; one patch; re-bench. Prefer levers already in-tree (flags, seeds, budgets, existing helpers) before inventing a third path.
- **Hybrid unify.** If two solutions each win on different axes (density vs speed, swap-on vs swap-off, cascade vs free emit, …), do **not** keep both forever and do **not** pick one blindly. Look for a **combined / unified** form that keeps each advantage to the extent possible (lex pick, shared helper with both predicates, soft scale instead of hard skip, …). Cite both winners in the patch rationale.
- **Unsure what hurts → telemetry first.** If the failure mode is opaque, add the smallest bench/telem that names the stage (void props/graph/nest/refine, cascade stop, pin add, rim drop, …), re-run, then patch from evidence — not from guess stacks.
- **Unify as you iterate.** Every loop is also a cleanup pass: fold duplicates into one gate, flatten nested branches, delete dead flags.
- **Keep logic clean and consistent.** Same predicate → same helper; same SoT → same call site family; comments must match code. Prefer one readable path over clever special cases.

## Nesting invariants

- **Output** must be collision-free (independent set). Transient DFS overlaps OK; `refine_selection` / `finalize_selection` must not return overlaps to Python.
- Default pipeline: `compose_and_nest_selection` → `nest_by_scores` → **3a** (`block_replace` lock-swap, mid-pack) → `refine_selection` (unlocked) → `finalize_selection` → **3b** (contact-CC re-emit) → stamp fallback → post-pack gravity. Attract is finalize/tie-break only. Cheap expand: `local_swap=False` unless **large_void** (then dual lex on/off). Heavy leaf always dual (**Q105**: dual = heavy OR large_void). Natives reuse `make_polygon_graph` / `NestState.native_geoms`. DFS locks stay unset (finalize re-inserts clear locks). `nest_by_graph` remains for `score_rules` / tests only.
- Do not add `NEST_DFS_MIN_COLLISIONS_*` env vars; caps live in `refine_dfs.cc`.
- **Board membership** = locked void solids (pad complement + exterior slabs + sheet holes) ∪ packed parts — **not** `fully_inside` / `footprint_inside` (oracle/tests only).
- One obstacle assembler: `placement_obstacles(voids, packed)`.
- Clearance SoT (same obstacles, different margins):
  - emit → `emit_packing_clear` (Penetrating, margin 0)
  - selection/polish → `is_pose_clear` (Scene margin)
  - guidance → `valid_at` / `PlacementScene.is_valid` (`min_dist+ε`)
- Packing collide = `Geometry.intersects` / Penetrating only. Do not soft-filter in Python.
- Contact: use `distance() <= threshold`, never `.buffer(gap).intersects()`. Cluster: `≤ 2·gap`. Board adj: standoff `≤ min_dist + 2·gap`.

## Propose / post-pack

- **`ProposeContext`** is propose-only (emit/rank). Do **not** widen it or import it from `build_graph` / `elem_graph`. Post-pack edits use **`SelectionEditCtx`**.
- Emit order is static (not a registry). Poles → `pocket_fit` → `cluster_copy` precede sweepers. Funnel keys: `round(x,y,θ, 4)` = `build_graph._transform_row_key`.
- Mid-pack rim: `board_edge` reserve before `side_pack` key claims; late kiss in `local_se2` (cached exterior ring).
- **`enable_gravity_compaction`** gates `local_se2` floater pole SE(2) toward an explicit void pole (default on). Ban corner / min-x+y sheet gravity (`compact_selection` deleted). Distinct from propose `border_focus`.
- `cluster_relocate` = rigid island ΔT (keep). `local_se2` = per-part SE(2).

### Module map (propose)

| Module | Owns |
|--------|------|
| `propose/types.py` | `ProposeContext`, extras, `make_propose_context` |
| `propose/pipeline.py` | staged `_collect_candidates`, ranking handoff |
| `propose/selection_edit.py` | `SelectionEditCtx` for `local_se2` / repack / relocate |
| `propose/post_pack.py` | repack → relocate → local_se2 runner |
| `propose/block_replace.py` | 3a cohort lock-swap; 3b hole re-nest |
| `propose/placement_common.py` | `placement_obstacles`, `is_pose_clear`, independence helper |
| `decision/` | Macro-MCTS policy (UCB1/PW/AMAF); thin `BoardSnapshot`; NFP-lite mates |
| `elem_graph/pose_graph.*` | Pose MIS (replaces ElemGraph) |
| `elem_graph/decision_arena.*` / `motif_base.*` / `se2.*` / `contact_relation.*` | C++ arena, motifs, SE2, ContactGRG+GCI |

`build_graph` owns graph/selection + Macro-MCTS outer loop (cheap expand vs best-leaf polish); it may re-export moved names for evaluator/tests.

## Hard bans

- No Python grids / Shapely fallbacks when C++ polish or Scene fails — fix the C++.
- Hot paths: `nest_graph.geometry.Geometry` + `NestState.native_geoms`. Do not loop `Geometry.from_shapely()` in propose emit / tightness / coverage.
- Prefer raw ring samples + batch `valid_at`; do not `buffer(-min_dist)` seed erosion. Ribbon annuli OK for frontier focus.
- Do not add Clipper2, libnest2d, or C++ Voronoi.
- Do not OR guidance `valid_at` with packing independence.
- Void-fill last resorts: no corner gravity; PSO only if `props_pole≈0` after OOS-1+4. B2: no void-aware nest/refine MIS until B0/B1 exhausted; rim drop >2% = fail.

## Glossary (confusion traps)

| Term | Meaning |
|------|---------|
| `border_focus` | Propose rim/kiss zone — **not** post-refine gravity |
| `void_seek` | Large free void path; may override `packed_near_border` |
| `emit_packing_clear` | Emit packing SoT (Penetrating); allows rim Touch |
| `is_pose_clear` | Selection/polish Scene clearance |
| `valid_at` | Guidance clearance only |
| `board_edge` reserve | Claims rim keys before `side_pack` / explorers |

## Verify before finishing

- Run the smallest relevant `pytest` for touched code.
- After C++ edits: rebuild (`uv pip install -e .` or cmake targets above), then run matching C++ or binding tests.
- Confirm selection outputs stay pairwise independent when changing pack/polish paths.
- Staged impl follows Planning (gate after each letter; on miss run the improvement loop, including unify/telem before the next letter).
