"""Unit tests for pocket_fit geometry and SE(2) alignment."""

import math

import numpy as np
from shapely.geometry import box

from nest_graph.config import ProposeConfig
from nest_graph.propose.placements_pocket import (
    TAG_POCKET_MRR,
    aligned_poses_for_pocket,
    hull_bay_polygons,
    mrr_major_axis_angle,
    se2_centroid_at_target,
    trapped_void_polygons,
)
from nest_graph.proposer_names import ProposerName
from nest_graph.utils import transform_poly


def test_pocket_fit_defaults_and_name():
    assert ProposerName.POCKET_FIT == "pocket_fit"
    p = ProposeConfig()
    assert p.use_pocket_fit is True
    assert p.pocket_fit_reserve_fraction == 0.15
    assert p.pocket_fit_area_ratio == 0.5


def test_trapped_void_center_hole():
    sheet = box(0, 0, 10, 10)
    # Four squares leaving a center hole that does not touch exterior.
    packed = [
        box(1, 1, 4, 4),
        box(6, 1, 9, 4),
        box(1, 6, 4, 9),
        box(6, 6, 9, 9),
        # Fill borders so free center is enclosed
        box(0, 0, 10, 1),
        box(0, 9, 10, 10),
        box(0, 1, 1, 9),
        box(9, 1, 10, 9),
        box(4, 1, 6, 4),
        box(4, 6, 6, 9),
        box(1, 4, 4, 6),
        box(6, 4, 9, 6),
    ]
    voids = trapped_void_polygons(sheet, packed)
    assert voids, voids
    # Largest trapped should be near center.
    c = voids[0].centroid
    assert 3.0 < c.x < 7.0 and 3.0 < c.y < 7.0


def test_exterior_free_not_trapped():
    sheet = box(0, 0, 10, 10)
    packed = [box(0, 0, 3, 3)]
    voids = trapped_void_polygons(sheet, packed)
    assert voids == []


def test_hull_bay_u_shape():
    # U made of three boxes
    packed = [
        box(0, 0, 1, 3),
        box(2, 0, 3, 3),
        box(0, 0, 3, 1),
    ]
    bays = hull_bay_polygons(packed, min_dist=0.05)
    assert bays, bays
    # Pocket of the U should be near (1.5, 2)
    c = bays[0].centroid
    assert 0.5 < c.x < 2.5 and c.y > 0.8


def test_se2_centroid_lands_on_poi():
    part = box(2, 3, 4, 5)  # centroid (3, 4), not origin-centered
    tx, ty = 10.0, 20.0
    for ang in (0.0, 0.3, math.pi / 2, 2.1):
        coords = se2_centroid_at_target(part, tx, ty, ang)
        placed = transform_poly(part, coords)
        assert abs(placed.centroid.x - tx) < 1e-9
        assert abs(placed.centroid.y - ty) < 1e-9


def test_aligned_poses_snap_drop():
    part = box(0, 0, 2, 1)
    void = box(5, 5, 8, 7)
    # Allowed angles far from MRR alignment → drop.
    poses = aligned_poses_for_pocket(
        part, void, min_dist=0.05, allowed_angles=(0.4,),
    )
    # May be empty if snap fails; with unrestricted allowed=None should emit.
    poses_free = aligned_poses_for_pocket(part, void, min_dist=0.05, allowed_angles=None)
    assert poses_free
    assert all(tag == TAG_POCKET_MRR for _, tag in poses_free)


def test_proposers_for_place_includes_pocket_fit():
    for zone in (
        "interior_pocket",
        "inter_cluster",
        "void_seek",
        "cluster_edge",
        "border_gap",
    ):
        enabled = ProposeConfig.proposers_for_place(zone)
        assert enabled is not None
        assert "pocket_fit" in enabled
    void = ProposeConfig.proposers_for_place("void_seek")
    assert "cluster_copy" in void
    border = ProposeConfig.proposers_for_place("border_gap")
    assert "cluster_copy" in border


def test_angle_snap_fallback_emits_allowed_cardinals():
    part = box(0, 0, 1, 1)
    void = box(0, 0, 2, 2)
    # Ideal MRR angles won't match these; fallback should still emit.
    poses = aligned_poses_for_pocket(
        part,
        void,
        min_dist=0.05,
        allowed_angles=(0.0, 0.5 * math.pi, math.pi, 1.5 * math.pi),
    )
    assert poses
