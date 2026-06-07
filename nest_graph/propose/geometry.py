import math
from typing import List, Optional, Sequence, Tuple, Union

import numpy as np
from shapely import LineString, LinearRing, MultiLineString, MultiPoint, MultiPolygon, Point, Polygon
from shapely.affinity import rotate, translate
from shapely.geometry.base import BaseGeometry
from shapely.ops import nearest_points, polylabel, unary_union, voronoi_diagram

from nest_graph.board import board_context_from_geometry
from nest_graph.config import ProposeConfig, dedupe_transforms
from nest_graph.geometry import Geometry, GuidanceConfig
from nest_graph.placement_scene import (
    PLACEMENT_EPSILON_RATIO,
    best_proposition,
    build_placement_scene,
    guidance_config_for_propose,
    guidance_config_for_scene,
    guidance_ray_direction_candidates,
    is_valid_placement,
    placement_footprint_inside_board,
    footprints_inside_board,
    proposition_translation,
    tiered_propositions,
)
from nest_graph.propose.context import should_use_border_focus
from nest_graph.utils import get_shape_exteriors, get_shape_polygons_coords, transform_poly


def _guidance_kwargs(propose_cfg: ProposeConfig | None) -> dict:
    if propose_cfg is None:
        return {
            "max_propositions": 6,
            "use_tight_packing": False,
            "enable_grid_exploration": False,
            "diversity_dist_ratio": 4.0,
        }
    return {
        "max_propositions": propose_cfg.guidance_max_propositions,
        "use_tight_packing": propose_cfg.guidance_use_tight_packing,
        "use_corner_alignment": propose_cfg.guidance_use_corner_alignment,
        "enable_grid_exploration": propose_cfg.guidance_enable_grid,
        "diversity_dist_ratio": propose_cfg.guidance_diversity_dist_ratio,
    }


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
    ):
        part = Geometry.from_shapely(part_poly)
        base_geoms: list[Geometry] = []
        if base_shape is not None and not base_shape.is_empty:
            base_geoms = [Geometry.from_shapely(base_shape)]
        self.scene = build_placement_scene(boundary, part, base_geoms)
        self.sheet = self.scene.sheet
        self.boundary = self.sheet
        self.part = part
        self._min_dist = min_dist
        self._epsilon_ratio = epsilon_ratio
        self._propose_cfg = propose_cfg
        self._board_bounds = self.sheet.bounds
        self._border_focus = (
            propose_cfg is not None
            and should_use_border_focus(base_shape, propose_cfg)
        )
        gkw = _guidance_kwargs(propose_cfg)
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
        gkw = _guidance_kwargs(self._propose_cfg)
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

    def inside_board(self, placed: Geometry) -> bool:
        if not placement_footprint_inside_board(placed, self.scene.board_geom):
            return False
        cx, cy = placed.center()
        g = self.scene.guidance(placed, (cx, cy), self._guidance_cfg)
        return not g.is_penetrating

    def inside_board_batch(self, placed_list: Sequence[Geometry]) -> list[bool]:
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

    def hits_base(
        self,
        placed: Geometry,
        push: Point | None = None,
        xy: Tuple[float, float] | None = None,
    ) -> bool:
        if not self.scene.base_geoms:
            return False
        return placed.intersects_any(self.scene.base_geoms)

    def is_valid_placement(
        self,
        placed: Geometry,
        push: Point,
        xy: Tuple[float, float],
    ) -> bool:
        cfg = self._propose_guidance_cfg(push)
        return is_valid_placement(
            self.scene, placed, xy, self._min_dist, cfg,
            epsilon_ratio=self._epsilon_ratio,
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
