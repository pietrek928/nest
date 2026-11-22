"""Pure Shapely geometry factories for tests (no pytest)."""

import math

from shapely.geometry import MultiPolygon, Polygon, box

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


def off_center_donut() -> Polygon:
    """Outer 0..10 with hole shifted to expose centroid bias."""
    return Polygon(
        shell=[(0, 0), (10, 0), (10, 10), (0, 10)],
        holes=[[(1, 1), (4, 1), (4, 4), (1, 4)]],
    )


def plate_with_l_hole() -> Polygon:
    """Square plate with concave L-shaped hole (G2 concave-hole bar case)."""
    return Polygon(
        shell=[(0, 0), (10, 0), (10, 10), (0, 10)],
        holes=[[(2, 2), (8, 2), (8, 4), (4, 4), (4, 8), (2, 8)]],
    )


def concave_hole_crossing_bar() -> Polygon:
    """All vertices in L-hole void; body crosses solid peninsula."""
    return Polygon([(2.5, 5), (3.5, 5), (5.5, 3), (4.5, 3)])


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


def c_shape_10() -> Polygon:
    """10x10 C matching C++ polygon_c_shape_three_parts silhouette."""
    return Polygon(
        [
            (0, 0),
            (10, 0),
            (10, 2),
            (2, 2),
            (2, 8),
            (10, 8),
            (10, 10),
            (0, 10),
        ]
    )


def multi_unit_squares() -> MultiPolygon:
    return MultiPolygon(
        [
            Polygon([(0, 0), (1, 0), (1, 1), (0, 1)]),
            Polygon([(3, 0), (4, 0), (4, 1), (3, 1)]),
        ]
    )


def cw_hole_donut() -> Polygon:
    """Explicit CW hole ring (Shapely may normalize; exterior CCW)."""
    outer = [(0, 0), (6, 0), (6, 6), (0, 6)]
    hole_cw = [(2, 2), (2, 4), (4, 4), (4, 2)]  # CW
    return Polygon(shell=outer, holes=[hole_cw])


def skinny_rect() -> Polygon:
    return box(0.0, 0.0, 100.0, 1.0)


def tessellated_circle(num_segments: int = 128, radius: float = 5.0) -> Polygon:
    pts = []
    for i in range(num_segments):
        angle = 2.0 * math.pi * i / num_segments
        pts.append((radius * math.cos(angle), radius * math.sin(angle)))
    return Polygon(pts)


def notch_square() -> Polygon:
    return Polygon([(2.5, 2.5), (3.5, 2.5), (3.5, 3.5), (2.5, 3.5)])


def rect_poly() -> Polygon:
    return normalize_poly(Polygon([(0, 0), (0.1, 0), (0.1, 0.1), (0, 0.1)]))


def tri_poly() -> Polygon:
    return normalize_poly(Polygon([(0, 0), (0.15, 0), (0, 0.07)]))
