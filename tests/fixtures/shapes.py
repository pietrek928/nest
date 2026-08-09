"""Pure Shapely geometry factories for tests (no pytest)."""

from shapely.geometry import Polygon, box

from nest_graph.utils import normalize_poly


def nest_triangle_board() -> Polygon:
    return Polygon([(0, 0), (1.2, 0), (0, 1.1)])


def nest_board_large() -> Polygon:
    return Polygon([(0, 0), (30, 0), (30, 30), (0, 30)])


def sheet(w: float = 10.0, h: float | None = None) -> Polygon:
    hh = float(w if h is None else h)
    return box(0.0, 0.0, float(w), hh)


def unit_square() -> Polygon:
    return box(0.0, 0.0, 1.0, 1.0)


def simple_square() -> Polygon:
    return Polygon([(0, 0), (2, 0), (2, 2), (0, 2)])


def square_with_hole() -> Polygon:
    return Polygon(
        shell=[(0, 0), (4, 0), (4, 4), (0, 4)],
        holes=[[(1, 1), (3, 1), (3, 3), (1, 3)]],
    )


def donut(
    outer: float = 10.0,
    hole_min: float = 3.0,
    hole_max: float = 7.0,
) -> Polygon:
    return box(0, 0, outer, outer).difference(box(hole_min, hole_min, hole_max, hole_max))


def l_shape() -> Polygon:
    return normalize_poly(Polygon([(0, 0), (4, 0), (4, 2), (2, 2), (2, 4), (0, 4)]))


def l_shape_raw() -> Polygon:
    return Polygon([(0, 0), (4, 0), (4, 2), (2, 2), (2, 4), (0, 4)])


def c_shape() -> Polygon:
    # Open C: outer 6x6 with 2x4 notch from the right.
    return Polygon(
        [
            (0, 0),
            (6, 0),
            (6, 2),
            (2, 2),
            (2, 4),
            (6, 4),
            (6, 6),
            (0, 6),
        ]
    )


def notch_square() -> Polygon:
    return Polygon([(2.5, 2.5), (3.5, 2.5), (3.5, 3.5), (2.5, 3.5)])


def rect_poly() -> Polygon:
    return normalize_poly(Polygon([(0, 0), (0.1, 0), (0.1, 0.1), (0, 0.1)]))


def tri_poly() -> Polygon:
    return normalize_poly(Polygon([(0, 0), (0.15, 0), (0, 0.07)]))
