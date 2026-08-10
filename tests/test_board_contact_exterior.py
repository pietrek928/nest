"""Exterior-only board contact: holes must not drive d_board / is_board_adj."""

from shapely.geometry import Polygon, box

from nest_graph.board import board_context_from_geometry
from nest_graph.propose.placement_common import is_board_adj
from nest_graph.propose.placement_outline import outline_ring_geom, outline_standoff_distance
from nest_graph.utils import transform_poly


def test_board_adj_uses_exterior_not_holes():
    outline = Polygon([(0, 0), (40, 0), (40, 40), (0, 40)])
    hole = ((15, 15), (25, 15), (25, 25), (15, 25))
    sheet, _voids = board_context_from_geometry(outline, user_holes=(hole,))
    # Part near outer rim, far from hole.
    rim = transform_poly(box(0, 0, 2, 2), (1.0, 1.0, 0.0))
    assert is_board_adj(rim, sheet, min_dist=0.5)
    ring = outline_ring_geom(sheet)
    assert ring is not None
    # Standoff to exterior ring stays small; hole is ignored by ring builder.
    assert outline_standoff_distance(rim, sheet) < 5.0
    # Part hugging the hole but deep inside is not board-adjacent via exterior.
    near_hole = transform_poly(box(0, 0, 2, 2), (13.5, 18.0, 0.0))
    assert not is_board_adj(near_hole, sheet, min_dist=0.5)
