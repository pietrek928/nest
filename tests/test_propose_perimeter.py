"""Perimeter-walk and erosion proposer smoke tests."""

from shapely import Point, Polygon, box

from nest_graph.config import ProposeConfig
from shapely.ops import unary_union

from nest_graph.propose import (
    ProposeGeometry,
    propose_placements_axis_push,
    propose_placements_bottom_left,
    propose_placements_erosion,
    propose_placements_neighbor_slide,
    propose_placements_nfp_vertices,
    propose_placements_perimeter_walk,
)
from nest_graph.utils import transform_poly


def _unit_square() -> Polygon:
    return box(0, 0, 1, 1)


def _board_and_geom(base: Polygon, part: Polygon, min_dist: float = 0.1):
    board = box(-1, -1, 10, 10)
    geom = ProposeGeometry(
        board,
        base,
        part,
        min_dist,
        propose_cfg=ProposeConfig(),
    )
    sheet = geom.sheet
    return board, sheet, geom


def test_perimeter_walk_packed_near_obstacle():
    board = box(0, 0, 20, 20)
    base = box(2, 2, 6, 6)
    part = _unit_square()
    min_dist = 0.2
    geom = ProposeGeometry(board, base, part, min_dist, propose_cfg=ProposeConfig())
    pt_push = base.centroid

    coords = propose_placements_perimeter_walk(
        base,
        part,
        geom.sheet,
        min_dist,
        propose_geom=geom,
        pt_push=pt_push,
        use_free_region=True,
        border_focus=False,
        num_angles=12,
        top_n=16,
    )
    assert coords, "expected at least one perimeter candidate"

    placed_dists = [
        float(base.distance(transform_poly(part, (x, y, angle))))
        for x, y, angle in coords
    ]
    assert min(placed_dists) < 2.0


def test_perimeter_walk_empty_board():
    board = box(0, 0, 10, 10)
    base = Polygon()
    part = _unit_square()
    min_dist = 0.1
    geom = ProposeGeometry(board, base, part, min_dist, propose_cfg=ProposeConfig())
    pt_push = Point(5, 5)

    coords = propose_placements_perimeter_walk(
        base,
        part,
        geom.sheet,
        min_dist,
        propose_geom=geom,
        pt_push=pt_push,
        use_free_region=False,
        border_focus=True,
        num_angles=8,
        top_n=8,
    )
    assert coords


def test_neighbor_slide_packed_near_obstacle():
    board = box(0, 0, 20, 20)
    base = box(2, 2, 6, 6)
    part = _unit_square()
    min_dist = 0.2
    geom = ProposeGeometry(board, base, part, min_dist, propose_cfg=ProposeConfig())
    pt_push = base.centroid

    coords = propose_placements_neighbor_slide(
        base,
        part,
        geom.sheet,
        min_dist,
        propose_geom=geom,
        pt_push=pt_push,
        num_angles=12,
        top_n=16,
    )
    assert coords, "expected at least one neighbor-slide candidate"

    placed_dists = [
        float(base.distance(transform_poly(part, (x, y, angle))))
        for x, y, angle in coords
    ]
    assert min(placed_dists) <= min_dist + 0.35


def test_neighbor_slide_empty_base():
    board = box(0, 0, 10, 10)
    base = Polygon()
    part = _unit_square()
    geom = ProposeGeometry(board, base, part, 0.1, propose_cfg=ProposeConfig())

    coords = propose_placements_neighbor_slide(
        base,
        part,
        geom.sheet,
        0.1,
        propose_geom=geom,
        pt_push=Point(5, 5),
        num_angles=8,
        top_n=8,
    )
    assert coords == []


def test_axis_push_kisses_neighbor():
    board = box(0, 0, 20, 20)
    base = box(2, 2, 6, 6)
    part = _unit_square()
    min_dist = 0.2
    geom = ProposeGeometry(board, base, part, min_dist, propose_cfg=ProposeConfig())
    pt_push = Point(10, 4)

    coords = propose_placements_axis_push(
        base,
        part,
        geom.sheet,
        min_dist,
        propose_geom=geom,
        pt_push=pt_push,
        num_angles=12,
        top_n=16,
    )
    assert coords, "expected at least one axis-push candidate"

    placed_dists = [
        float(base.distance(transform_poly(part, (x, y, angle))))
        for x, y, angle in coords
    ]
    assert min(placed_dists) <= min_dist + 0.35


def test_axis_push_empty_base():
    board = box(0, 0, 10, 10)
    base = Polygon()
    part = _unit_square()
    geom = ProposeGeometry(board, base, part, 0.1, propose_cfg=ProposeConfig())

    coords = propose_placements_axis_push(
        base,
        part,
        geom.sheet,
        0.1,
        propose_geom=geom,
        pt_push=Point(5, 5),
        num_angles=8,
        top_n=8,
    )
    assert coords == []


def test_bottom_left_in_strip():
    board = box(0, 0, 20, 20)
    base = box(0, 8, 20, 20)
    part = _unit_square()
    min_dist = 0.2
    geom = ProposeGeometry(board, base, part, min_dist, propose_cfg=ProposeConfig())
    pt_push = Point(10, 2)

    coords = propose_placements_bottom_left(
        base,
        part,
        geom.sheet,
        min_dist,
        propose_geom=geom,
        pt_push=pt_push,
        use_free_region=True,
        border_focus=False,
        num_angles=12,
        top_n=16,
    )
    assert coords, "expected bottom-left candidates in strip void"

    ys = [
        transform_poly(part, (x, y, angle)).centroid.y
        for x, y, angle in coords
    ]
    assert min(ys) < 4.0


def test_nfp_vertices_packed():
    board = box(0, 0, 20, 20)
    base = unary_union([box(2, 2, 5, 5), box(10, 2, 13, 5)])
    part = _unit_square()
    min_dist = 0.2
    geom = ProposeGeometry(board, base, part, min_dist, propose_cfg=ProposeConfig())
    pt_push = base.centroid

    coords = propose_placements_nfp_vertices(
        base,
        part,
        geom.sheet,
        min_dist,
        propose_geom=geom,
        pt_push=pt_push,
        num_angles=12,
        top_n=16,
    )
    assert coords, "expected NFP-lite vertex candidates"

    placed_dists = [
        float(base.distance(transform_poly(part, (x, y, angle))))
        for x, y, angle in coords
    ]
    assert min(placed_dists) < 2.5


def test_nfp_vs_perimeter_multi_part():
    board = box(0, 0, 20, 20)
    base = unary_union([box(2, 2, 5, 5), box(10, 2, 13, 5)])
    part = _unit_square()
    min_dist = 0.2
    geom = ProposeGeometry(board, base, part, min_dist, propose_cfg=ProposeConfig())
    pt_push = base.centroid
    kwargs = dict(
        propose_geom=geom,
        pt_push=pt_push,
        num_angles=12,
        top_n=24,
    )

    nfp_coords = propose_placements_nfp_vertices(
        base, part, geom.sheet, min_dist, **kwargs,
    )
    perimeter_coords = propose_placements_perimeter_walk(
        base,
        part,
        geom.sheet,
        min_dist,
        use_free_region=True,
        border_focus=False,
        **kwargs,
    )
    assert nfp_coords
    assert perimeter_coords

    def _keys(coords):
        return {
            (round(x, 2), round(y, 2), round(a, 1))
            for x, y, a in coords
        }

    assert _keys(nfp_coords)
    assert _keys(perimeter_coords)


def test_erosion_returns_candidates_in_void():
    board = box(0, 0, 20, 20)
    base = box(1, 1, 4, 4)
    part = _unit_square()
    min_dist = 0.1
    board, sheet, geom = _board_and_geom(base, part, min_dist)

    coords = propose_placements_erosion(
        base,
        part,
        sheet,
        min_dist,
        propose_geom=geom,
        pt_push=base.centroid,
        use_free_region=True,
        border_focus=False,
        focal_shape=base,
        num_angles=8,
        top_n=4,
    )
    assert coords
