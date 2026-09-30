"""N0 census + N1 MotifBase mate seed + N2 group_allowed_angles fill."""

import math

import numpy as np
from shapely.geometry import Polygon, box

from nest_graph.graph import MotifBase
from nest_graph.propose.placement_common import (
    count_edge_parallel_transforms,
    resolve_group_allowed_angles,
)
from nest_graph.propose.placements_pattern import (
    seed_motif_base_from_mates,
    synthesize_mate_patterns,
    triangle_mate_relative,
)
from nest_graph.propose.telem import apply_pairing_edge_slot_census


def _tri() -> Polygon:
    return Polygon([(0.0, 0.0), (1.0, 0.0), (0.0, 1.0)])


def test_n0_census_keys():
    sheet = box(0, 0, 10, 10)
    free = type("F", (), {"largest_area": 12.5, "max_void_ratio": 2.0})()
    stats = {"kiss_pairs": 3}
    transforms = [(1.0, 1.0, 0.0), (2.0, 2.0, 0.1), (3.0, 3.0, math.pi / 2)]
    out = apply_pairing_edge_slot_census(
        stats,
        free_info=free,
        transforms=transforms,
        selected=[0, 1, 2],
        sheet=sheet,
    )
    assert out["pair_contact_n"] == 3
    assert out["largest_slot_area"] == 12.5
    assert out["edge_parallel_n"] >= 2  # 0 and π/2 align to rect sheet
    assert stats["pair_contact_n"] == 3
    assert count_edge_parallel_transforms(transforms, [0, 2], sheet) == 2


def test_n1_seed_motif_base_from_mates():
    tri = _tri()
    parts = [(tri, 0), (box(0, 0, 0.5, 0.5), 1)]
    mate = triangle_mate_relative(tri, min_dist=0.01)
    assert mate is not None
    synth = synthesize_mate_patterns(parts, min_dist=0.01, max_patterns=2)
    assert synth
    mb = MotifBase()
    n = seed_motif_base_from_mates(mb, parts, min_dist=0.01)
    assert n >= 1
    assert mb.size() >= 1
    # Idempotent upsert.
    n2 = seed_motif_base_from_mates(mb, parts, min_dist=0.01)
    assert n2 >= 1
    assert mb.size() >= 1


def test_n2_resolve_group_allowed_angles_fills_empty_grain():
    sheet = box(0, 0, 10, 8)
    parts = [(_tri(), 0), (box(0, 0, 1, 1), 1)]
    filled = resolve_group_allowed_angles(sheet, parts, ())
    assert len(filled) == 2
    assert filled[0] is not None and len(filled[0]) > 0
    assert filled[1] is not None and len(filled[1]) > 0
    # Existing grain preserved.
    keep = ((0.0, math.pi), None)
    kept = resolve_group_allowed_angles(sheet, parts, keep)
    assert kept[0] == (0.0, math.pi)
    assert kept[1] is None


def test_n4a_unlock_clear_xy_skips_when_clears_kept():
    from nest_graph.propose.placements_pattern import _unlock_clear_xy_anchors

    skip = {"anchor_clear_kept": 3}
    anchors = [(1.0, 1.0, 0.0), (2.0, 2.0, 0.0)]
    out = _unlock_clear_xy_anchors(
        anchors,
        propose_geom=None,
        obstacles=[],
        void_poly=box(0, 0, 5, 5),
        pole=None,
        shape_to_place=_tri(),
        skip_reasons=skip,
    )
    assert out == anchors
