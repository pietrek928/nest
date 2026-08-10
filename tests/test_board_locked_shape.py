"""Board-as-locked-shape: exterior frame voids replace fully_inside for membership."""

from shapely.geometry import Polygon, box

from nest_graph.board import board_context_from_geometry, board_void_obstacles
from nest_graph.config import ProposeConfig
from nest_graph.geometry import Geometry
from nest_graph.propose.geometry import ProposeGeometry
from nest_graph.propose.placement_common import is_pose_clear
from nest_graph.propose.placements_pattern import emit_packing_clear


def test_exterior_frame_rejects_far_outside():
    outline = Polygon([(0, 0), (10, 0), (10, 10), (0, 10)])
    voids = board_void_obstacles(outline, padding=0.8)
    assert voids
    part = Geometry.from_shapely(box(0, 0, 1, 1))
    far = part.apply_transform((100.0, 100.0, 0.0))
    assert far.intersects_any(voids)


def test_rim_kiss_accepted_under_void_sot():
    """Touching the sheet rim must not be rejected solely by containment."""
    sheet = box(0, 0, 20, 20)
    part = box(0, 0, 2, 2)
    cfg = ProposeConfig()
    geom = ProposeGeometry(
        sheet, box(0, 0, 0.1, 0.1), part, 0.05, propose_cfg=cfg, full_packed_geoms=[],
    )
    # Part seated at lower-left with standoff ≈ min_dist (kiss band).
    coords = (0.05, 0.05, 0.0)
    placed = geom.placed_at(coords)
    assert placed is not None
    # Emit / clearance must not use fully_inside reject of rim touch.
    assert emit_packing_clear(geom, coords) or is_pose_clear(
        placed, geom.scene.void_geoms, [], 0.0,
    )


def test_hole_void_still_rejects():
    outline = Polygon([(0, 0), (20, 0), (20, 20), (0, 20)])
    hole = ((8, 8), (12, 8), (12, 12), (8, 12))
    _sheet, voids = board_context_from_geometry(outline, user_holes=(hole,))
    assert voids
    part = Geometry.from_shapely(box(0, 0, 1, 1))
    in_hole = part.apply_transform((9.5, 9.5, 0.0))
    assert in_hole.intersects_any(voids)
