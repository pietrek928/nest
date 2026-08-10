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
- Imports at file top unless there is a concrete reason otherwise.

### C++

- No custom namespaces (no anonymous `namespace {…}`, no helper namespaces). Helpers at file scope in `.cc` or `inline` in headers.
- Namespace aliases for third-party APIs are fine (`namespace nb = nanobind;`).
- Avoid `static` on functions unless needed for linkage; prefer shared `inline` helpers in `internal/internal.h`.

### Git

- Do not commit unless the user asks.

## Nesting invariants

- **Output** must be collision-free (independent set). Transient DFS overlaps OK; `refine_selection` / `finalize_selection` must not return overlaps to Python.
- Default pipeline: `nest_by_graph` → `refine_selection` (loose then tight) → `finalize_selection`.
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
| `propose/placement_common.py` | `placement_obstacles`, `is_pose_clear`, independence helper |

`build_graph` owns graph/selection only; it may re-export moved names for evaluator/tests.

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
