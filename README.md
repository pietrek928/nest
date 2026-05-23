# nest_graph

2D nesting by building a **collision graph** over many placement candidates, then selecting a large independent set with learned placement rules and DFS refinement. Native C++ extensions (nanobind) handle polygon intersection, distance, and EPA penetration; Python drives sampling, rule evolution, and rendering.

## Features

- **Dense collision graphs** — board-valid transforms become graph nodes; edges record overlaps (incremental bipartite sweep, no R-tree).
- **Placement rules** — point/angle rules scored and evolved across iterations (`improve_rules`).
- **Smart transform seeds** — erosion, raycast, and voronoi proposers plus random/shuffle batches ([`nest_graph/propose.py`](nest_graph/propose.py)).
- **Configurable loop** — sampling, propose, selection, and output via [`nest_graph/config.py`](nest_graph/config.py) and `NEST_*` env vars.

## Requirements

| Tool | Purpose |
|------|---------|
| Python ≥ 3.9 | Runtime |
| **python3-dev** (e.g. `python3.12-dev`) | Building nanobind extensions (CMake `Development.Module`) |
| CMake ≥ 3.18 | Native build |
| C++17 compiler | `g++` / `clang++` |
| [uv](https://github.com/astral-sh/uv) (recommended) or pip | Env and editable install |

Debian/Ubuntu:

```bash
sudo apt install python3.12-dev cmake build-essential
```

## Quick start

```bash
git clone <repo-url> nest && cd nest

uv venv
uv sync --extra test
uv pip install -e . --no-build-isolation

# Smoke run (2 iterations, no progress bar)
NEST_BUILD_GRAPH_ITERS=2 NEST_PROGRESS=0 uv run python -m nest_graph.build_graph
```

Outputs `test.mp4` and `test.jpg` in the current directory by default.

## Build and install

### Recommended: editable install with uv

Scikit-build runs CMake and installs `geometry` and `elem_graph` extensions into the package tree — no manual copying of `.so` files.

```bash
uv sync --extra test
uv pip install -e . --no-build-isolation
```

Re-run after C++ or CMake changes:

```bash
uv pip install -e . --no-build-isolation
```

### Alternative: pip

```bash
python -m venv .venv
source .venv/bin/activate
pip install -e ".[test]" --no-build-isolation
```

`--no-build-isolation` keeps the build on your machine’s CMake/nanobind setup (see [pyproject.toml](pyproject.toml)).

### When editable install fails (missing Python headers)

If CMake reports `Could NOT find Python (missing: Development.Module)`:

```bash
sudo apt install python3.12-dev   # match your Python minor version

# Or configure CMake manually:
mkdir -p build && cd build
cmake .. -DNEST_GRAPH_BUILD_TESTS=ON
cmake --build . --target geometry elem_graph
cd ..
```

For a one-off run without reinstalling, copy artifacts into the tree:

```bash
cmake --build build --target geometry elem_graph
# Extensions land in nest_graph/ (nest_graph/geometry*.so, nest_graph/elem_graph*.so)
```

Prefer fixing `python3-dev` and using `uv pip install -e .` so paths stay consistent.

## Running tests

### Python (pytest)

Use **`uv run`** so the project venv and dependencies are used:

```bash
uv run pytest tests/
```

Fast subset:

```bash
uv run pytest tests/test_geometry_intersect_regression.py tests/test_build_graph.py -q
```

Without uv (editable install + activated venv):

```bash
pytest tests/
```

### C++ geometry unit tests (Catch2)

Not built by `pip install -e .` by default. Enable once:

```bash
cmake -S . -B build -DNEST_GRAPH_BUILD_TESTS=ON
cmake --build build --target geometry_cpp_tests
./build/nest_graph/geometry/geometry_cpp_tests
```

Or via CTest:

```bash
cd build && ctest --output-on-failure
```

After header-only changes under `nest_graph/geometry/`, incremental rebuild:

```bash
cmake --build build --target geometry_cpp_tests geometry
uv pip install -e . --no-build-isolation   # refresh Python .so if needed
```

### First-pass parameter benchmark

```bash
PYTHONPATH=. python scripts/benchmark_first_pass.py
```

See [docs/first_pass_tuning.md](docs/first_pass_tuning.md) for preset comparison tables.

## Usage

### Demo nesting loop

```bash
uv run python -m nest_graph.build_graph
```

### Configuration

Defaults are tuned for first-iteration quality (see tuning doc). Override with environment variables or in code:

```python
from nest_graph.config import BuildGraphConfig
from nest_graph.build_graph import run_build_graph

cfg = BuildGraphConfig.from_env()
# cfg = BuildGraphConfig(...)  # or construct explicitly
run_build_graph(cfg)
```

| Preset | Example |
|--------|---------|
| Fast debug | `NEST_BUILD_GRAPH_ITERS=32 NEST_RANDOM_PER_ITER=64 NEST_MAX_TRANSFORMS=300 NEST_PROGRESS=0` |
| Balanced (defaults) | no env vars |
| Quality | `NEST_RANDOM_PER_ITER=256 NEST_MAX_TRANSFORMS=1200 NEST_IMPROVE_ROUNDS=8` |

Full reference: [docs/nest_config.md](docs/nest_config.md).

## Project layout

```
nest_graph/
  build_graph.py      # main loop: batch → graph → nest → DFS → video
  config.py           # Pydantic config + NEST_* env loading
  propose.py          # placement proposers (erosion, voronoi, raycast, PSO)
  utils.py            # Shapely helpers, transform_poly
  geometry*.so        # C++ extension (import nest_graph.geometry)
  geometry/           # C++ sources: solid/, convex/, intersect/, distance/, sweep/, guide/, bindings/
  elem_graph*.so      # C++ extension (import nest_graph.elem_graph)
  elem_graph/         # C++ sources: rules/, graph/, scoring/, selection/, refine/, bindings/
docs/
  nest_config.md      # env var reference
  first_pass_tuning.md
  debugging_guide.md  # geometry debugging snippets
scripts/
  benchmark_first_pass.py
tests/
```

## Architecture (one iteration)

```mermaid
flowchart LR
  batch[Transform batch]
  mpg[make_polygon_graph]
  rules[improve_rules]
  nest[nest_by_graph]
  dfs[DFS refine]
  batch --> mpg --> rules --> nest --> dfs
```

1. Build a transform batch (random, history, propose seeds, shuffles).
2. `make_polygon_graph` — keep all board-valid placements; add collision edges.
3. Evolve placement rule sets on recent graphs.
4. `nest_by_graph` — greedy rule-scored independent set.
5. DFS passes to grow selection; feed transforms into the next iteration.

## Debugging

- Geometry engine notes and matplotlib snippets: [docs/debugging_guide.md](docs/debugging_guide.md)
- Stale `.so` after C++ edits → rebuild with `uv pip install -e .` or `cmake --build build --target geometry`

## License

Add license text here if applicable.
