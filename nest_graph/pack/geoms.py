"""Native geometry helpers for pack stages."""

from typing import Sequence

import numpy as np

from nest_graph.geometry import Geometry
from nest_graph.propose.placement_common import as_geometry


def native_geoms_from_transforms(
    group_id: Sequence[int],
    transform: Sequence,
    bases: dict[int, Geometry],
    *,
    seed_polys: Sequence | None = None,
    seed_count: int = 0,
) -> list[Geometry]:
    """SE2 solids from part bases (no re-decomp of transformed Shapely)."""
    out: list[Geometry] = []
    n = len(group_id)
    for i in range(n):
        if seed_count > 0 and i < seed_count and seed_polys is not None and i < len(seed_polys):
            g = as_geometry(seed_polys[i])
            if g is None:
                raise ValueError("seed poly empty")
            out.append(g)
            continue
        gid = int(group_id[i])
        out.append(bases[gid].apply_transform(np.asarray(transform[i], dtype=np.float64)))
    return out


def selection_coverage_pct(
    selected_indices: Sequence[int],
    group_id: Sequence[int],
    part_areas: Sequence[float],
    board_area: float,
) -> float:
    if board_area <= 0:
        return 0.0
    parts_area = sum(part_areas[group_id[i]] for i in selected_indices)
    return 100.0 * parts_area / board_area


# Back-compat aliases for build_graph / scripts
_native_geoms_from_transforms = native_geoms_from_transforms
_selection_coverage_pct = selection_coverage_pct

__all__ = [
    "native_geoms_from_transforms",
    "selection_coverage_pct",
    "_native_geoms_from_transforms",
    "_selection_coverage_pct",
]
