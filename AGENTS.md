# Agent instructions (nest_graph)

## Python

- **Do not use `from __future__ import …`** — not needed on this project’s Python (3.12+). Never add `__future__` imports (including `annotations`).
- If you remove `__future__` imports that were added by mistake, do not reintroduce them.
- Prefer normal type hints (`X | None`, quoted forward refs like `"BuildGraphConfig"` only when required).

## Native extensions

- After C++ changes, rebuild: `cmake --build build --target geometry elem_graph` (or `uv pip install -e .`).
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
