"""Legacy placement demo (formerly build_graph.test_placement)."""

import cv2 as cv
from shapely import Polygon, unary_union

from nest_graph.build_graph import render_polys
from nest_graph.propose import propose_placements_point_cloud
from nest_graph.propose.geometry import ProposeGeometry
from nest_graph.utils import normalize_poly, transform_poly


def main() -> None:
    p_board = Polygon([(0, 0), (1.2, 0), (0, 1.1)])
    p1 = normalize_poly(Polygon([(0, 0), (0.15, 0), (0, 0.07)]))
    p2 = normalize_poly(Polygon([(0, 0), (0.1, 0), (0.1, 0.1), (0, 0.1)]))

    p1_result = []
    p2_result = []
    base_shape = Polygon()
    for _ in range(100):
        geom1 = ProposeGeometry(p_board, base_shape, p1, min_dist=0.001)
        p1_places = propose_placements_point_cloud(
            base_shape,
            p1,
            p_board,
            min_dist=0.001,
            pt_push=p_board.centroid,
            top_n=100,
            propose_geom=geom1,
        )
        print("p1", len(p1_places))
        if p1_places:
            p1_result.append(p1_places[0])
            base_shape = unary_union([base_shape, transform_poly(p1, p1_places[0])])
        geom2 = ProposeGeometry(p_board, base_shape, p2, min_dist=0.001)
        p2_places = propose_placements_point_cloud(
            base_shape,
            p2,
            p_board,
            min_dist=0.001,
            pt_push=p_board.centroid,
            top_n=100,
            propose_geom=geom2,
        )
        print("p2", len(p2_places))
        if p2_places:
            p2_result.append(p2_places[0])
            base_shape = unary_union([base_shape, transform_poly(p2, p2_places[0])])

        im = render_polys(
            p_board,
            [
                [transform_poly(p1, t) for t in p1_result],
                [transform_poly(p2, t) for t in p2_result],
            ],
        )
        cv.imwrite("test.jpg", im)


if __name__ == "__main__":
    main()
