"""A0: batch_valid_flags(False) must match ProposeGeometry.valid_at."""

import numpy as np
from shapely.geometry import Point, Polygon
from shapely.ops import unary_union

from nest_graph.config import ProposeConfig
from nest_graph.propose.geometry import ProposeGeometry, batch_valid_flags
from nest_graph.propose.placement_common import is_pose_clear, placement_obstacles
from nest_graph.propose.placements_pattern import emit_packing_clear
from nest_graph.utils import transform_poly


def _scene():
    sheet = Polygon([(0, 0), (10, 0), (10, 10), (0, 10)])
    part = Polygon([(0, 0), (1, 0), (1, 1), (0, 1)])
    packed = [
        transform_poly(part, (2.0, 2.0, 0.0)),
        transform_poly(part, (4.0, 2.0, 0.0)),
    ]
    obstacle = unary_union(packed)
    cfg = ProposeConfig()
    geom = ProposeGeometry(
        sheet, obstacle, part, 0.05, epsilon_ratio=0.05, propose_cfg=cfg,
    )
    push = Point(5.0, 5.0)
    return geom, push, part


def test_batch_valid_flags_false_matches_valid_at():
    geom, push, _part = _scene()
    rng = np.random.default_rng(0)
    transforms = [
        (float(x), float(y), float(th))
        for x, y, th in zip(
            rng.uniform(1.5, 8.5, 64),
            rng.uniform(1.5, 8.5, 64),
            rng.uniform(0.0, 2.0 * np.pi, 64),
            strict=True,
        )
    ]
    flags = batch_valid_flags(geom, transforms, push, return_guidance=False)
    assert isinstance(flags, list)
    xor = 0
    for coords, ok in zip(transforms, flags, strict=True):
        single = geom.valid_at(coords, push)
        if bool(ok) != bool(single):
            xor += 1
    assert xor == 0


def test_guidance_cfg_reuse_matches_valid_at():
    geom, push, _ = _scene()
    cfg = geom._propose_guidance_cfg(push)
    coords = (6.0, 6.0, 0.0)
    assert geom.valid_at(coords, push, guidance_cfg=cfg) == geom.valid_at(coords, push)
    flags = batch_valid_flags(
        geom, [coords], push, return_guidance=False, guidance_cfg=cfg,
    )
    assert flags == [geom.valid_at(coords, push)]


def test_far_outside_rejected_via_voids():
    geom, push, _ = _scene()
    assert geom.scene.void_geoms
    assert not emit_packing_clear(geom, (100.0, 100.0, 0.0))
    placed = geom.placed_at((100.0, 100.0, 0.0))
    assert placed is not None
    assert not is_pose_clear(placed, geom.scene.void_geoms, [], 0.05)
    assert not geom.valid_at((100.0, 100.0, 0.0), push)


def test_interior_pose_valid_without_fully_inside():
    geom, push, _ = _scene()
    coords = (7.0, 7.0, 0.0)
    assert emit_packing_clear(geom, coords)
    assert geom.valid_at(coords, push)
    placed = geom.placed_at(coords)
    assert is_pose_clear(placed, geom.scene.void_geoms, geom.base_geoms, 0.05)


def test_placement_obstacles_merges_voids_and_packed():
    geom, _, _ = _scene()
    obs = placement_obstacles(geom.scene.void_geoms, geom.base_geoms)
    assert len(obs) >= len(geom.scene.void_geoms)
    assert len(obs) >= len(geom.base_geoms)
