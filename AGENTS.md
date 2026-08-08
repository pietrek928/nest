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

## OOS Void-Fill Execution Protocol

### 0. Proposer Telemetry (DO FIRST)

- Keys: `round(x,y,θ, 4)` — must match `build_graph._transform_row_key`.
- Emit order: `pocket_fit` → `cluster_copy` → … → `raycast` (`cluster_copy` before sweepers).
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
- P3: re-add nest-void idxs only if `graph.collisions`-clear vs refine.
  Telemetry: `void_leak` `pin_candidates` / `pin_added` / `pin_blocked_collision` / `pin_ms`.
- **B1 gate:** bind `greedy_weighted_mis` / fold pin into finalize with `locked_indices`
  only if `pin_added≥1` **and** `pin_ms>30`. Post-hoc collision-clear append does not
  need finalize locks.
- **B2 blocked:** no void-aware nest/refine MIS / external scores until B0/B1 exhausted;
  rim drop >2% = fail.
