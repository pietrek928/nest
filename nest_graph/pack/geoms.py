"""Native geometry helpers for pack stages."""

from typing import Callable, Sequence

import numpy as np

from nest_graph.geometry import Geometry
from nest_graph.placement.common import as_geometry


def rebuild_w1_packed(
    *,
    packed_group_id: Sequence | None,
    packed_transform: Sequence | None,
    native_geoms_fn: Callable,
    part_bases: dict,
    seed_void_geoms: Sequence | None,
    telem: dict | None = None,
) -> tuple[list | None, int | None]:
    """W1: rebuild packed solids as seeds ∪ native(pgid,ptf). Returns (packed, n) or (None, None)."""
    pgid = packed_group_id
    ptf = packed_transform
    if (
        pgid is None
        or ptf is None
        or len(pgid) != len(ptf)
        or len(pgid) <= 0
    ):
        return None, None
    try:
        rebuilt = native_geoms_fn(pgid, ptf, part_bases)
        if not rebuilt:
            return None, None
        seed_geoms = [g for g in (seed_void_geoms or []) if g is not None]
        packed_geoms = list(seed_geoms) + list(rebuilt)
        if telem is not None:
            telem["w1_obstacle_sot"] = 1
            telem["w1_packed_rebuilt_n"] = int(len(rebuilt))
        return packed_geoms, int(len(rebuilt))
    except Exception:
        return None, None


def kiss_geoms_for_selection(
    selected: Sequence[int],
    *,
    candidate_geoms: Sequence | None,
    polys: Sequence,
) -> list:
    """Candidate-pool geom else ``as_geometry(poly)`` for kiss ledger walks."""
    kiss_geoms = []
    for i in selected:
        ii = int(i)
        if candidate_geoms is not None and 0 <= ii < len(candidate_geoms):
            g = candidate_geoms[ii]
            if g is not None:
                kiss_geoms.append(g)
                continue
        if 0 <= ii < len(polys):
            gg = as_geometry(polys[ii])
            if gg is not None:
                kiss_geoms.append(gg)
    return kiss_geoms


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
    "kiss_geoms_for_selection",
    "native_geoms_from_transforms",
    "rebuild_w1_packed",
    "selection_coverage_pct",
    "_native_geoms_from_transforms",
    "_selection_coverage_pct",
]
