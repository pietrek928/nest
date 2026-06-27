from typing import Sequence, Tuple

import numpy as np
from shapely import Point, Polygon
from shapely.geometry.base import BaseGeometry

from nest_graph.config import ProposeConfig
from nest_graph.geometry import Geometry, GuidanceConfig
from nest_graph.placement_scene import (
    PLACEMENT_EPSILON_RATIO,
    best_proposition,
    build_placement_scene,
    guidance_config_for_propose,
    guidance_config_for_scene,
    guidance_kwargs_for_propose,
    placement_clearance_epsilon,
    placement_footprint_inside_board,
    footprints_inside_board,
    proposition_translation,
)
from nest_graph.propose.context import should_use_border_focus


class ProposeGeometry:
    """Cached placement scene for fast propose validation and guidance."""

    def __init__(
        self,
        boundary: BaseGeometry,
        base_shape: BaseGeometry,
        part_poly: Polygon,
        min_dist: float,
        *,
        epsilon_ratio: float = PLACEMENT_EPSILON_RATIO,
        propose_cfg: ProposeConfig | None = None,
        full_packed_geoms: list[Geometry] | None = None,
    ):
        part = Geometry.from_shapely(part_poly)
        base_geoms: list[Geometry] = []
        if base_shape is not None and not base_shape.is_empty:
            base_geoms = [Geometry.from_shapely(base_shape)]
        self.scene = build_placement_scene(boundary, part, base_geoms)
        self.sheet = self.scene.sheet
        self.part = part
        self.board_geom = self.scene.board_geom
        self.full_packed_geoms = list(full_packed_geoms or [])
        ring_coords = list(self.sheet.exterior.coords)
        if len(ring_coords) >= 2 and ring_coords[0] == ring_coords[-1]:
            ring_coords = ring_coords[:-1]
        self.boundary_ring_geom = (
            Geometry.from_ring(ring_coords) if len(ring_coords) >= 2 else part
        )
        self.base_geoms = list(self.scene.base_geoms)
        self._min_dist = min_dist
        self._epsilon_ratio = epsilon_ratio
        self._propose_cfg = propose_cfg
        self._board_bounds = self.sheet.bounds
        self._border_focus = (
            propose_cfg is not None
            and should_use_border_focus(base_shape, propose_cfg)
        )
        gkw = guidance_kwargs_for_propose(propose_cfg)
        self._guidance_cfg = guidance_config_for_scene(
            min_dist,
            board_bounds=self._board_bounds,
            epsilon_ratio=epsilon_ratio,
            border_focus=self._border_focus,
            for_propose=self._border_focus,
            pt_push=self.sheet.centroid if self._border_focus else None,
            **gkw,
        )

    def _propose_guidance_cfg(
        self,
        push: Point,
        *,
        border_focus: bool | None = None,
        target_angle_rad: float = 0.0,
    ):
        gkw = guidance_kwargs_for_propose(self._propose_cfg)
        use_border = self._border_focus if border_focus is None else border_focus
        return guidance_config_for_propose(
            push,
            min_dist=self._min_dist,
            board_bounds=self._board_bounds,
            epsilon_ratio=self._epsilon_ratio,
            border_focus=use_border,
            target_angle_rad=target_angle_rad,
            **gkw,
        )

    def placed_at(self, coords: Tuple[float, float, float]) -> Geometry:
        return self.part.apply_transform(coords)

    def obstacle_geoms_for_batch(self) -> list[Geometry]:
        return [*self.base_geoms, *self.scene.void_geoms]

    def placement_guidance(
        self,
        placed: Geometry,
        xy: Tuple[float, float],
        push: Point,
        *,
        target_angle_rad: float = 0.0,
        border_focus: bool | None = None,
        guidance_cfg: GuidanceConfig | None = None,
    ):
        if guidance_cfg is not None:
            return self.scene.guidance(placed, xy, guidance_cfg)
        cfg = self._propose_guidance_cfg(
            push,
            border_focus=border_focus,
            target_angle_rad=target_angle_rad,
        )
        return self.scene.guidance(placed, xy, cfg)

    def footprint_clear_of_voids(self, placed: Geometry) -> bool:
        if not placement_footprint_inside_board(placed, self.scene.board_geom):
            return False
        cx, cy = placed.center()
        g = self.scene.guidance(placed, (cx, cy), self._guidance_cfg)
        return not g.is_penetrating

    def footprint_clear_of_voids_batch(self, placed_list: Sequence[Geometry]) -> list[bool]:
        if not placed_list:
            return []
        footprint_ok = footprints_inside_board(placed_list, self.scene.board_geom)
        out: list[bool] = []
        for placed, ok in zip(placed_list, footprint_ok, strict=True):
            if not ok:
                out.append(False)
                continue
            cx, cy = placed.center()
            g = self.scene.guidance(placed, (cx, cy), self._guidance_cfg)
            out.append(not g.is_penetrating)
        return out

    def passes_full_packed_collision(self, placed: Geometry) -> bool:
        if not self.full_packed_geoms:
            return True
        return not placed.intersects_any(self.full_packed_geoms)

    def hits_base(self, placed: Geometry) -> bool:
        if not self.scene.base_geoms:
            return False
        return placed.intersects_any(self.scene.base_geoms)

    def valid(
        self,
        placed: Geometry,
        push: Point,
        xy: Tuple[float, float],
    ) -> bool:
        cfg = self._propose_guidance_cfg(push)
        return self.scene.is_valid(
            placed, xy, self._min_dist, cfg, epsilon_ratio=self._epsilon_ratio,
        )

    def valid_at(
        self,
        coords: Tuple[float, float, float],
        pt_push: Point,
    ) -> bool:
        cfg = self._propose_guidance_cfg(pt_push)
        return self.scene.valid_at(
            coords, self._min_dist, cfg, epsilon_ratio=self._epsilon_ratio,
        )

    def attraction_unit(
        self,
        placed: Geometry,
        push: Point,
        xy: Tuple[float, float],
    ) -> np.ndarray:
        g = self.placement_guidance(placed, xy, push)
        prop = best_proposition(g)
        if prop is not None:
            tx, ty = proposition_translation(prop)
            vec = np.array([tx, ty], dtype=np.float64)
        else:
            vec = np.zeros(2, dtype=np.float64)
        norm = np.linalg.norm(vec)
        return vec / norm if norm > 1e-9 else np.zeros(2)


def batch_valid_flags(
    propose_geom: ProposeGeometry,
    transforms: Sequence[Tuple[float, float, float]],
    pt_push: Point,
    *,
    return_guidance: bool = False,
) -> list[bool] | list[object | None]:
    """Batch validity against void obstacles; optional guidance with base+void."""
    from nest_graph.geometry import batch_check_validity, batch_evaluate_local_placement

    if not transforms:
        return []
    placed_list = [propose_geom.placed_at(c) for c in transforms]
    footprint_ok = footprints_inside_board(placed_list, propose_geom.board_geom)
    cfg = propose_geom._propose_guidance_cfg(pt_push)
    margin = 0.0
    if propose_geom._min_dist > 0.0:
        margin = propose_geom._min_dist + placement_clearance_epsilon(
            propose_geom._min_dist, ratio=propose_geom._epsilon_ratio,
        )

    if not return_guidance:
        survivors = [
            (i, (float(c[0]), float(c[1]), float(c[2])))
            for i, (c, ok) in enumerate(zip(transforms, footprint_ok, strict=True))
            if ok
        ]
        if not survivors:
            return [False] * len(transforms)
        indices, survivor_transforms = zip(*survivors, strict=True)
        flags = batch_check_validity(
            propose_geom.part,
            list(survivor_transforms),
            propose_geom.scene.void_geoms,
            cfg,
            propose_geom._min_dist,
            propose_geom._epsilon_ratio,
        )
        out = [False] * len(transforms)
        for i, ok in zip(indices, flags, strict=True):
            out[i] = ok
        return out

    survivors = [
        (i, c) for i, (c, ok) in enumerate(zip(transforms, footprint_ok, strict=True)) if ok
    ]
    out_guidance: list[object | None] = [None] * len(transforms)
    if not survivors:
        return out_guidance
    survivor_transforms = [
        (float(c[0]), float(c[1]), float(c[2])) for _, c in survivors
    ]
    # Ranking needs neighbor clearance from packed base parts, not void-only.
    obstacles = propose_geom.obstacle_geoms_for_batch()
    guidance_list = batch_evaluate_local_placement(
        propose_geom.part,
        survivor_transforms,
        obstacles,
        (float(pt_push.x), float(pt_push.y)),
        cfg,
    )
    for (i, _c), g in zip(survivors, guidance_list, strict=True):
        if g.is_penetrating:
            continue
        if margin > 0.0 and float(g.clearance) < margin:
            continue
        out_guidance[i] = g
    return out_guidance
