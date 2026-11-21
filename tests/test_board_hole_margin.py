"""H3: sheet-hole void solids inflate with Scene-family hole_margin."""

from shapely import box

from nest_graph.board import board_void_geometries, board_sheet_from_outline


def test_sheet_hole_void_inflates_with_margin():
    outline = box(0.0, 0.0, 10.0, 10.0)
    hole = ((4.0, 4.0), (6.0, 4.0), (6.0, 6.0), (4.0, 6.0))
    sheet = board_sheet_from_outline(outline, user_holes=(hole,))
    bare = board_void_geometries(sheet, outline=outline, padding=0.5, hole_margin=0.0)
    fat = board_void_geometries(sheet, outline=outline, padding=0.5, hole_margin=0.2)
    # Escape slabs dominate area; hole solid is the small-magnitude component (~4).
    bare_small = min(abs(float(g.area())) for g in bare)
    fat_small = min(abs(float(g.area())) for g in fat)
    assert bare_small > 0.5
    assert fat_small > bare_small + 0.5
