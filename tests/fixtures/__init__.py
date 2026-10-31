"""Pure Shapely geometry factories for tests."""

from tests.fixtures.shapes import (  # noqa: F401
    c_shape,
    donut,
    l_shape,
    l_shape_raw,
    nest_board_large,
    nest_triangle_board,
    notch_square,
    rect_poly,
    sheet,
    simple_square,
    square_with_hole,
    tri_poly,
    unit_square,
)
from tests.fixtures.propose_emit import (  # noqa: F401
    EMIT_CLEAR_POSE_CASES,
    EMIT_PENETRATE_CASES,
    emit_board_empty,
    emit_board_with_block,
    free_space_cloud_board,
    make_propose_geom,
    perimeter_ring_board,
    tightness_pack_geoms,
)
