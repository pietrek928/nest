from typing import Sequence, Tuple

import numpy as np
from shapely import Point, Polygon
from shapely.geometry.base import BaseGeometry

from nest_graph.config import ProposeConfig
from nest_graph.geometry import Geometry, GuidanceConfig, batch_evaluate_local_placement
from nest_graph.placement_scene import (
    PLACEMENT_EPSILON_RATIO,
    best_proposition,
    build_placement_scene,
    guidance_config_for_propose,
    guidance_config_for_scene,
    guidance_kwargs_for_propose,
    placement_clearance_epsilon,
    proposition_translation,
)


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
        border_focus: bool | None = None,
    ):
        from nest_graph.propose.context import should_use_border_focus

        part = Geometry.from_shapely(part_poly)
        base_geoms: list[Geometry] = []
        if base_shape is not None and not base_shape.is_empty:
            base_geoms = [Geometry.from_shapely(base_shape)]
        self.scene = build_placement_scene(boundary, part, base_geoms)
        self.sheet = self.scene.sheet
        self.part = part
        self.part_poly = part_poly
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
        if border_focus is not None:
            self._border_focus = bool(border_focus)
        else:
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
        # Per-push GuidanceConfig cache (void_seek / corridor change pt_push).
        self._push_guidance_cache: dict[
            tuple[float, float, bool, float, float],
            GuidanceConfig,
        ] = {}

    def _propose_guidance_cfg(
        self,
        push: Point,
        *,
        border_focus: bool | None = None,
        target_angle_rad: float = 0.0,
    ):
        use_border = self._border_focus if border_focus is None else border_focus
        key = (
            round(float(push.x), 6),
            round(float(push.y), 6),
            bool(use_border),
            round(float(target_angle_rad), 6),
            float(self._min_dist),
        )
        cached = self._push_guidance_cache.get(key)
        if cached is not None:
            return cached
        gkw = guidance_kwargs_for_propose(self._propose_cfg)
        cfg = guidance_config_for_propose(
            push,
            min_dist=self._min_dist,
            board_bounds=self._board_bounds,
            epsilon_ratio=self._epsilon_ratio,
            border_focus=use_border,
            target_angle_rad=target_angle_rad,
            **gkw,
        )
        self._push_guidance_cache[key] = cfg
        return cfg

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
        *,
        guidance_cfg: GuidanceConfig | None = None,
    ) -> bool:
        cfg = guidance_cfg if guidance_cfg is not None else self._propose_guidance_cfg(pt_push)
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
    guidance_cfg: GuidanceConfig | None = None,
) -> list[bool] | list[object | None]:
    """Batch validity matching ProposeGeometry.valid_at (base+void + guidance).

    Uses ``batch_evaluate_local_placement`` for both bool and guidance modes.
    """
    if not transforms:
        return []
    cfg = guidance_cfg if guidance_cfg is not None else propose_geom._propose_guidance_cfg(pt_push)
    margin = 0.0
    if propose_geom._min_dist > 0.0:
        margin = propose_geom._min_dist + placement_clearance_epsilon(
            propose_geom._min_dist, ratio=propose_geom._epsilon_ratio,
        )

    obstacles = propose_geom.obstacle_geoms_for_batch()
    survivor_transforms = [
        (float(c[0]), float(c[1]), float(c[2])) for c in transforms
    ]
    guidance_list = batch_evaluate_local_placement(
        propose_geom.part,
        survivor_transforms,
        obstacles,
        (float(pt_push.x), float(pt_push.y)),
        cfg,
    )

    if not return_guidance:
        out: list[bool] = []
        for g in guidance_list:
            if g.is_penetrating:
                out.append(False)
                continue
            if margin > 0.0 and float(g.clearance) < margin:
                out.append(False)
                continue
            out.append(True)
        return out

    out_guidance: list[object | None] = []
    for g in guidance_list:
        if g.is_penetrating:
            out_guidance.append(None)
            continue
        if margin > 0.0 and float(g.clearance) < margin:
            out_guidance.append(None)
            continue
        out_guidance.append(g)
    return out_guidance


def filter_candidates_batch(
    propose_geom: ProposeGeometry,
    transforms: Sequence[Tuple[float, float, float]],
    pt_push: Point,
    *,
    guidance_cfg: GuidanceConfig | None = None,
    max_batch: int = 512,
) -> list[Tuple[float, float, float]]:
    """Keep transforms that pass full-guidance validity; chunk large pools."""
    if not transforms:
        return []
    kept: list[Tuple[float, float, float]] = []
    chunk = max(int(max_batch), 1)
    for start in range(0, len(transforms), chunk):
        block = list(transforms[start:start + chunk])
        flags = batch_valid_flags(
            propose_geom, block, pt_push, guidance_cfg=guidance_cfg,
        )
        assert isinstance(flags, list)
        for coords, ok in zip(block, flags, strict=True):
            if ok:
                kept.append(
                    (float(coords[0]), float(coords[1]), float(coords[2])),
                )
    return kept
