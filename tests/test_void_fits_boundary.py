"""Letter A: void_fits_part must see holes (sponge free), not exterior-only."""

from shapely import Point, Polygon, box

from nest_graph.propose.context import void_fit_radii, void_fits_part


def test_sponge_free_with_island_hole_does_not_fit():
    """Large outer free with a packed-island hole near the pole → not fits."""
    outer = box(0.0, 0.0, 100.0, 100.0)
    island = box(40.0, 40.0, 60.0, 60.0)
    free = Polygon(outer.exterior.coords, holes=[island.exterior.coords])
    pole = Point(50.0, 50.0)  # inside island hole → on/near hole boundary
    # Catalog part whose extent needs more clearance than hole→island gap.
    part = box(0.0, 0.0, 30.0, 30.0)
    fits_boundary, in_r, need_r = void_fit_radii(
        free, shape=part, at_pt=pole, use_boundary=True
    )
    fits_exterior, in_r_ext, _need = void_fit_radii(
        free, shape=part, at_pt=pole, use_boundary=False
    )
    assert need_r > 10.0
    # Exterior-only falsely reports a large inradius (to outer shell).
    assert in_r_ext > need_r
    assert fits_exterior is True
    # Boundary (holes) sees the island — clearance is small.
    assert in_r < need_r
    assert fits_boundary is False
    assert void_fits_part(free, shape=part, at_pt=pole) is False


def test_open_void_without_holes_still_fits():
    free = box(0.0, 0.0, 100.0, 100.0)
    part = box(0.0, 0.0, 10.0, 10.0)
    pole = Point(50.0, 50.0)
    assert void_fits_part(free, shape=part, at_pt=pole) is True
