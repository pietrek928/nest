"""Shared bag for post-pack selection editors (local_se2 / cluster_repack)."""

from dataclasses import dataclass
from typing import Sequence

from shapely import Point, Polygon
from shapely.geometry.base import BaseGeometry

from nest_graph.config import ProposeConfig
from nest_graph.geometry import Geometry


@dataclass
class SelectionEditCtx:
    """Parameter object for selection mutation passes."""

    sheet: Polygon
    polys: list[BaseGeometry]
    transforms: list
    group_ids: Sequence[int]
    selected_indices: Sequence[int]
    part_by_group: dict[int, Polygon]
    min_dist: float
    propose_cfg: ProposeConfig | None = None
    pole: Point | None = None
    poles: Sequence[Point] | None = None
    fixed_obstacles: Sequence[BaseGeometry] | None = None
    void_geoms: Sequence | None = None
    board_adj_indices: Sequence[int] | None = None
    part_bases: dict[int, Geometry] | None = None
    board_geom: Geometry | None = None
