from dataclasses import dataclass
from typing import Sequence

from shapely import Point, Polygon
from shapely.geometry.base import BaseGeometry

import numpy as np

from nest_graph.config import ProposeConfig

from nest_graph.propose.context import should_use_border_focus
from nest_graph.propose.geometry import ProposeGeometry
from nest_graph.propose.placement_common import placement_angle_grid


@dataclass
class ProposeContext:
    """Invariant inputs for a single placement propose query."""

    base_shape: BaseGeometry
    shape_to_place: Polygon
    sheet: Polygon
    propose_cfg: ProposeConfig
    min_dist: float
    pt_push: Point
    propose_geom: ProposeGeometry
    focal_shape: BaseGeometry | None = None
    enabled_proposers: frozenset[str] | None = None
    proposer_counts: dict[str, int] | None = None
    guidance_seed_coords: Sequence[tuple[float, float, float]] | None = None

    @property
    def pool(self) -> int:
        return self.propose_cfg.candidate_pool

    @property
    def border_focus(self) -> bool:
        return should_use_border_focus(self.base_shape, self.propose_cfg)

    @property
    def use_free_region(self) -> bool:
        return self.propose_cfg.use_free_region_search

    @property
    def placement_angles(self) -> np.ndarray:
        return placement_angle_grid(
            self.sheet,
            self.base_shape,
            self.propose_cfg.placement_num_angles,
        )

    @property
    def n_angles(self) -> int:
        return len(self.placement_angles)
