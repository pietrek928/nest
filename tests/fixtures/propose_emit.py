"""Shared ProposeGeometry + emit/clear pose factories for propose letter tests."""

from shapely.geometry import Polygon, box

from nest_graph.config import ProposeConfig
from nest_graph.propose.geometry import ProposeGeometry


def make_propose_geom(
    *,
    sheet: Polygon | None = None,
    base: Polygon | None = None,
    part: Polygon | None = None,
    min_dist: float = 0.1,
    packed: list | None = None,
    propose_cfg: ProposeConfig | None = None,
) -> ProposeGeometry:
    """Build a ProposeGeometry with optional packed obstacles."""
    sh = sheet if sheet is not None else box(0, 0, 20, 20)
    ba = base if base is not None else box(0, 0, 0.1, 0.1)
    pt = part if part is not None else box(0, 0, 1, 1)
    return ProposeGeometry(
        sh,
        ba,
        pt,
        float(min_dist),
        propose_cfg=propose_cfg if propose_cfg is not None else ProposeConfig(),
        full_packed_geoms=list(packed or []),
    )


# (label, x, y, theta, expect_clear) on default 20x20 sheet, 1x1 part, empty packed.
EMIT_CLEAR_POSE_CASES: list[tuple[str, float, float, float, bool]] = [
    ("center_clear", 10.0, 10.0, 0.0, True),
    ("near_origin_clear", 2.0, 2.0, 0.0, True),
    ("far_outside", 100.0, 100.0, 0.0, False),
    ("kiss_angle", 10.0, 10.0, 0.7853981633974483, True),
]


# Obstacle covers [9,12]x[9,12]; part 1x1 — center pose penetrates.
EMIT_PENETRATE_CASES: list[tuple[str, float, float, float, bool]] = [
    ("clear_of_block", 5.0, 5.0, 0.0, True),
    ("inside_block", 10.5, 10.5, 0.0, False),
    ("kiss_edge_clear", 8.0, 10.5, 0.0, True),
]


def emit_board_empty() -> ProposeGeometry:
    return make_propose_geom()


def emit_board_with_block() -> ProposeGeometry:
    return make_propose_geom(packed=[box(9.0, 9.0, 12.0, 12.0)])


def free_space_cloud_board() -> tuple[ProposeGeometry, Polygon, ProposeConfig]:
    """Sheet/void/obstacle setup used by free_space_cloud emit baselines."""
    sheet = box(0, 0, 30, 30)
    void = box(10, 10, 25, 25)
    obstacle = box(0, 0, 8, 8)
    part = box(0, 0, 1.5, 1.5)
    cfg = ProposeConfig(use_free_space_cloud=True, free_space_cloud_samples=32)
    geom = make_propose_geom(
        sheet=sheet, base=obstacle, part=part, min_dist=0.1, propose_cfg=cfg,
    )
    return geom, void, cfg


def perimeter_ring_board(
    *,
    packed: Polygon | None = None,
) -> tuple[ProposeGeometry, Polygon, Polygon]:
    """Board + base + unit part for perimeter bipartite fixtures."""
    board = box(0, 0, 20, 20)
    base = packed if packed is not None else box(2, 2, 6, 6)
    part = box(0, 0, 1, 1)
    geom = make_propose_geom(sheet=board, base=base, part=part, min_dist=0.2)
    return geom, base, part


def tightness_pack_geoms(
    *,
    n: int = 3,
    gap: float = 0.15,
) -> tuple[list, Polygon, float]:
    """Native-friendly axis-aligned boxes for pack_tightness_cost parity."""
    from nest_graph.geometry import Geometry

    outline = box(0, 0, 20, 20)
    min_dist = 0.1
    geoms = []
    for i in range(n):
        x0 = 2.0 + i * (1.0 + gap)
        poly = box(x0, 2.0, x0 + 1.0, 3.0)
        geoms.append(Geometry.from_shapely(poly))
    return geoms, outline, min_dist
