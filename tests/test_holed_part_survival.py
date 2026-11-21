"""H0/H1: part-hole free survival via sheet.difference (pocket SoT)."""

from shapely import Polygon, box

from nest_graph.propose.context import count_part_hole_free_survival


def _donut(cx: float, cy: float, outer: float = 2.0, inner: float = 0.9) -> Polygon:
    return Polygon(
        [
            (cx - outer, cy - outer),
            (cx + outer, cy - outer),
            (cx + outer, cy + outer),
            (cx - outer, cy + outer),
        ],
        holes=[[
            (cx - inner, cy - inner),
            (cx + inner, cy - inner),
            (cx + inner, cy + inner),
            (cx - inner, cy + inner),
        ]],
    )


def test_part_hole_free_survival_via_sheet_difference():
    """H1 SoT: free components in part holes survive sheet.difference(union)."""
    sheet = box(0.0, 0.0, 16.0, 16.0)
    donut = _donut(8.0, 8.0)
    census = count_part_hole_free_survival(sheet, [donut], 0.05)
    assert census["part_hole_raw_n"] >= 1
    assert census["part_hole_free_survived"] >= 1
    assert census["free_comp_n"] >= 1


def test_non_evicting_part_hole_plus_appends_truncated():
    """H1: truncated part-hole voids can be appended without dropping exterior."""
    from nest_graph.placement.pocket import _void_rep_in_part_holes
    from nest_graph.propose.context import part_interior_polygons

    donut = _donut(8.0, 8.0)
    holes = part_interior_polygons([donut])
    hole_void = holes[0]
    giants = [box(0.0, 0.0, 5.0 + i, 5.0) for i in range(3)]
    assert _void_rep_in_part_holes(hole_void, holes)
    # Simulate cut of max_targets=2 from largest-first: giants win, hole truncated.
    pockets = sorted(giants + [hole_void], key=lambda p: float(p.area), reverse=True)
    max_targets = 2
    cut = pockets[:max_targets]
    extras = [
        p
        for p in pockets[max_targets:]
        if _void_rep_in_part_holes(p, holes)
    ]
    assert extras
    cut2 = list(cut) + extras[:2]
    assert len(cut2) == len(cut) + 1
    assert any(_void_rep_in_part_holes(p, holes) for p in cut2)
