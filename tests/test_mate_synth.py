"""Geometry-derived mated-pair patterns (Phase 2)."""

import math

from shapely.geometry import Polygon

from nest_graph.propose.placements_pattern import (
    emit_packing_clear,
    synthesize_mate_patterns,
    triangle_mate_relative,
)
from nest_graph.propose.geometry import ProposeGeometry
from nest_graph.utils import transform_poly


def _right_triangle() -> Polygon:
    return Polygon([(0.0, 0.0), (1.0, 0.0), (0.0, 1.0)])


def test_triangle_mate_forms_parallelogram_with_clearance():
    tri = _right_triangle()
    min_dist = 0.05
    mate = triangle_mate_relative(tri, min_dist=min_dist)
    assert mate is not None
    assert abs(mate[2] - math.pi) < 1e-9
    placed = transform_poly(tri, mate)
    assert placed is not None and not placed.is_empty
    # Exact clearance: edges parallel/separated, no penetration.
    assert float(tri.intersection(placed).area) < 1e-8
    gap = float(tri.distance(placed))
    assert gap >= min_dist - 1e-6
    assert gap <= min_dist + 1e-3
    hull = tri.union(placed).convex_hull
    # Parallelogram-ish: hull area ≈ 2 * tri area + clearance strip.
    assert float(hull.area) < 2.5 * float(tri.area) + 1.0


def test_synthesize_mate_patterns_when_contact_empty():
    tri = _right_triangle()
    pats = synthesize_mate_patterns([(tri, 0)], min_dist=0.02, max_patterns=2)
    assert len(pats) == 1
    assert pats[0].part_count == 2
    assert pats[0].members[0][0] == 0
    assert pats[0].members[1][0] == 0


def test_mate_pair_emit_packing_clear_on_empty_sheet():
    tri = _right_triangle()
    # Rectangular sheet so the parallelogram mate stays interior.
    sheet = Polygon([(0, 0), (6, 0), (6, 6), (0, 6)])
    min_dist = 0.05
    mate = triangle_mate_relative(tri, min_dist=min_dist)
    assert mate is not None
    from nest_graph.config import ProposeConfig
    from nest_graph.utils import compose_transforms

    cfg = ProposeConfig()
    geom = ProposeGeometry(sheet, Polygon(), tri, min_dist, propose_cfg=cfg)
    anchor = (1.5, 1.5, 0.0)
    t_mate = compose_transforms(anchor, mate)
    assert emit_packing_clear(geom, anchor)
    assert emit_packing_clear(geom, t_mate)
