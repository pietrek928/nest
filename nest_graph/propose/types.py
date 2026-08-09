"""Propose query parameter objects (Config companions)."""

from dataclasses import dataclass, field
from typing import Sequence

from shapely import Point, Polygon
from shapely.geometry.base import BaseGeometry

import numpy as np

from nest_graph.config import ProposeConfig

from nest_graph.propose.context import FreeSpaceSnapshot, should_use_border_focus
from nest_graph.propose.geometry import ProposeGeometry
from nest_graph.propose.placement_common import placement_angle_grid


@dataclass
class PocketStats:
    """Mutable pocket_fit telemetry outs during collect."""

    tags: list = field(default_factory=list)
    attempts: int = 0
    skip_reasons: dict = field(default_factory=dict)


@dataclass
class PackedProposeExtras:
    """Mutable / packed-state bag for a propose collect or rank call."""

    packed_polys: Sequence | None = None
    packed_group_ids: Sequence[int] | None = None
    packed_transforms: Sequence | None = None
    native_geoms: Sequence | None = None
    free_space: FreeSpaceSnapshot | None = None
    cluster_patterns: Sequence | None = None
    pocket_stats: PocketStats | None = None
    prop_accept: dict | None = None
    void_pole: Point | None = None


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
    placement_angles_override: np.ndarray | None = None
    border_focus_override: bool | None = None
    void_pole: Point | None = None

    @property
    def pool(self) -> int:
        return self.propose_cfg.candidate_pool

    @property
    def border_focus(self) -> bool:
        if self.border_focus_override is not None:
            return bool(self.border_focus_override)
        return should_use_border_focus(self.base_shape, self.propose_cfg)

    @property
    def use_free_region(self) -> bool:
        return self.propose_cfg.use_free_region_search

    @property
    def placement_angles(self) -> np.ndarray:
        if self.placement_angles_override is not None and len(self.placement_angles_override) > 0:
            return np.asarray(self.placement_angles_override, dtype=np.float64)
        return placement_angle_grid(
            self.sheet,
            self.base_shape,
            self.propose_cfg.placement_num_angles,
        )

    @property
    def n_angles(self) -> int:
        return len(self.placement_angles)


def make_propose_context(
    *,
    base_shape: BaseGeometry,
    shape_to_place: Polygon,
    sheet: Polygon,
    propose_cfg: ProposeConfig,
    min_dist: float,
    pt_push: Point,
    propose_geom: ProposeGeometry,
    focal_shape: BaseGeometry | None = None,
    enabled_proposers: frozenset[str] | None = None,
    proposer_counts: dict[str, int] | None = None,
    guidance_seed_coords: Sequence[tuple[float, float, float]] | None = None,
    placement_angles_override: np.ndarray | None = None,
    border_focus: bool | None = None,
    void_pole: Point | None = None,
) -> ProposeContext:
    """Single factory for pipeline and build_graph propose queries."""
    return ProposeContext(
        base_shape=base_shape,
        shape_to_place=shape_to_place,
        sheet=sheet,
        propose_cfg=propose_cfg,
        min_dist=min_dist,
        pt_push=pt_push,
        propose_geom=propose_geom,
        focal_shape=focal_shape,
        enabled_proposers=enabled_proposers,
        proposer_counts=proposer_counts,
        guidance_seed_coords=guidance_seed_coords,
        placement_angles_override=placement_angles_override,
        border_focus_override=border_focus,
        void_pole=void_pole,
    )
