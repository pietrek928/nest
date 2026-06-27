import math
import time
from dataclasses import dataclass

from shapely.affinity import rotate, translate
from shapely.geometry import Point, Polygon, box
from shapely.ops import unary_union

from nest_graph.geometry import Geometry
from nest_graph.placement_scene import (
    PlacementScene,
    best_proposition,
    build_placement_scene,
    guidance_config_for_propose,
    is_valid_placement,
)

# --- shared boards ---
NEST_BOARD = Polygon([(0, 0), (1.2, 0), (0, 1.1)])
NEST_BOARD_LARGE = Polygon([(0, 0), (30, 0), (30, 30), (0, 30)])
BOARD_WITH_HOLE = box(0, 0, 1.2, 1.1)
LANE_BOARD = box(0, 0, 28, 14)
JAGGED_BOARD = Polygon(
    [
        (0, 0),
        (24, 0),
        (24, 6),
        (18, 6),
        (18, 12),
        (24, 12),
        (24, 24),
        (0, 24),
    ]
)
L_SHAPED_SHEET = Polygon([(0, 0), (22, 0), (22, 9), (9, 9), (9, 22), (0, 22)])
STRIP_BOARD = Polygon([(0, 0), (40, 0), (40, 5), (0, 5)])

# --- shared parts / obstacles ---
RECT_0_1 = Polygon([(0, 0), (0.1, 0), (0.1, 0.1), (0, 0.1)])
TRI_SMALL = Polygon([(0, 0), (0.15, 0), (0, 0.07)])
L_SHAPE = Polygon([(0, 0), (4, 0), (4, 2), (2, 2), (2, 4), (0, 4)])
RECT_2 = Polygon([(0, 0), (2, 0), (2, 2), (0, 2)])
BAR = Polygon([(0, 0), (1.0, 0), (1.0, 0.1), (0, 0.1)])
L_PART = Polygon([(0, 0), (0.4, 0), (0.4, 0.2), (0.2, 0.2), (0.2, 0.4), (0, 0.4)])
DENSE_RECT = Polygon([(0, 0), (0.12, 0), (0.12, 0.12), (0, 0.12)])
DENSE_TRI = Polygon([(0, 0), (0.14, 0), (0, 0.08)])
DENSE_PART = Polygon([(0, 0), (0.09, 0), (0.045, 0.07)])
LANE_RECT = Polygon([(0, 0), (1.2, 0), (1.2, 0.8), (0, 0.8)])
LANE_PART = Polygon([(0, 0), (1.0, 0), (1.0, 0.6), (0, 0.6)])
TRAP_L = Polygon([(0, 0), (5, 0), (5, 2), (2, 2), (2, 5), (0, 5)])
JAGGED_PART = Polygon([(0, 0), (1.6, 0), (1.6, 1.0), (0.6, 1.0), (0.6, 1.6), (0, 1.6)])
HEPT_PART = Polygon(
    [
        (0, 0),
        (1.8, 0.2),
        (2.4, 1.2),
        (1.6, 2.3),
        (0.4, 2.1),
        (-0.3, 1.2),
        (-0.2, 0.4),
    ]
)
STRIP_RECT = Polygon([(0, 0), (0.8, 0), (0.8, 0.8), (0, 0.8)])
STRIP_PART = Polygon([(0, 0), (1.1, 0), (1.1, 0.5), (0, 0.5)])
OFF_AXIS = math.pi / 7

LANE_HOLES = (
    ((4, 3), (7, 3), (7, 11), (4, 11)),
    ((11, 3), (14, 3), (14, 11), (11, 11)),
    ((18, 3), (21, 3), (21, 11), (18, 11)),
)
HOLE_MOUTH = (((0.4, 0.4), (0.8, 0.4), (0.8, 0.8), (0.4, 0.8)),)


@dataclass
class GuidanceScenario:
    label: str
    scene: PlacementScene
    part_shapely: Polygon
    obstacles_shapely: list[Polygon]
    seed: tuple[float, float, float]
    pt_push: Point
    min_dist: float
    board_outline: Polygon
    border_focus: bool = False
    intentional_overlap: bool = False
    expected_move_substrings: tuple[str, ...] = ()
    notes: str = ""


@dataclass
class GuidanceScenarioResult:
    scenario_id: str
    shapes: str
    props_count: int
    is_penetrating: bool
    best_move_type: str
    move_types: list[str]
    kiss_error: float
    seed_kiss_error: float
    border_standoff_err: float
    board_valid: bool
    seed_board_valid: bool
    execution_time_ms: float


@dataclass(frozen=True)
class Placement:
    poly: Polygon
    x: float
    y: float
    theta: float = 0.0  # radians; pass positionally to _place_poly


@dataclass(frozen=True)
class ScenarioSpec:
    label: str
    board: Polygon
    part: Polygon
    obstacles: tuple[Placement, ...]
    seed: tuple[float, float, float]
    pt_push: tuple[float, float]
    min_dist: float
    user_holes: tuple[tuple[tuple[float, float], ...], ...] = ()
    border_focus: bool = False
    intentional_overlap: bool = False
    expected_move_substrings: tuple[str, ...] = ()
    notes: str = ""


def _place_poly(poly: Polygon, x: float, y: float, theta: float = 0.0) -> Polygon:
    # theta is positional; shape builders use rotation= kwarg instead.
    p = rotate(poly, theta, use_radians=True, origin=(0, 0))
    return translate(p, x, y)


def _placed_shapely(
    part: Polygon,
    seed: tuple[float, float, float],
    *,
    dx: float = 0.0,
    dy: float = 0.0,
    dtheta: float = 0.0,
) -> Polygon:
    x, y, theta = seed
    p = rotate(part, theta + dtheta, use_radians=True, origin=(0, 0))
    return translate(p, x + dx, y + dy)


def _assert_simple(poly: Polygon, *, label: str = "polygon") -> None:
    if poly.is_empty or not poly.is_valid:
        raise ValueError(f"invalid {label}: empty={poly.is_empty} valid={poly.is_valid}")


def _regular_ngon(
    n: int,
    radius: float,
    *,
    cx: float = 0.0,
    cy: float = 0.0,
    rotation: float = 0.0,
) -> Polygon:
    pts = [
        (
            cx + radius * math.cos(rotation + 2 * math.pi * i / n),
            cy + radius * math.sin(rotation + 2 * math.pi * i / n),
        )
        for i in range(n)
    ]
    return Polygon(pts)


def _c_shape(*, w: float = 4.0, h: float = 4.0, t: float = 1.0) -> Polygon:
    return Polygon(
        [
            (0, 0),
            (w, 0),
            (w, t),
            (t, t),
            (t, h - t),
            (w, h - t),
            (w, h),
            (0, h),
        ]
    )


def _trapezoid(bottom: float, top: float, height: float) -> Polygon:
    inset = (bottom - top) / 2.0
    return Polygon(
        [(0, 0), (bottom, 0), (bottom - inset, height), (inset, height)]
    )


def _star(
    points: int,
    r_outer: float,
    r_inner: float,
    *,
    rotation: float = 0.0,
) -> Polygon:
    pts = [
        (
            r_outer * math.cos(rotation + math.pi * i / points)
            if i % 2 == 0
            else r_inner * math.cos(rotation + math.pi * i / points),
            r_outer * math.sin(rotation + math.pi * i / points)
            if i % 2 == 0
            else r_inner * math.sin(rotation + math.pi * i / points),
        )
        for i in range(points * 2)
    ]
    return Polygon(pts)


def _gear(
    teeth: int,
    r_root: float,
    r_tip: float,
    *,
    rotation: float = 0.0,
) -> Polygon:
    pts = [
        (
            r_tip * math.cos(rotation + math.pi * i / teeth)
            if i % 2 == 0
            else r_root * math.cos(rotation + math.pi * i / teeth),
            r_tip * math.sin(rotation + math.pi * i / teeth)
            if i % 2 == 0
            else r_root * math.sin(rotation + math.pi * i / teeth),
        )
        for i in range(teeth * 2)
    ]
    return Polygon(pts)


def _s_shape(*, w: float = 4.0, h: float = 4.0, t: float = 1.0) -> Polygon:
    parts = [
        box(0, 0, w, t),
        box(0, h - t, w, h),
        box(w - t, t, w, h / 2.0),
        box(0, h / 2.0, t, h - t),
    ]
    merged = unary_union(parts)
    if merged.geom_type != "Polygon":
        merged = max(merged.geoms, key=lambda g: g.area)
    return merged


def _u_channel(*, w: float = 4.0, h: float = 4.0, t: float = 0.8) -> Polygon:
    return Polygon(
        [
            (0, 0),
            (w, 0),
            (w, h),
            (w - t, h),
            (w - t, t),
            (t, t),
            (t, h),
            (0, h),
        ]
    )


def _cross(*, arm: float = 2.0, t: float = 0.8) -> Polygon:
    c = arm / 2.0
    ht = t / 2.0
    return Polygon(
        [
            (-ht, -c),
            (ht, -c),
            (ht, -ht),
            (c, -ht),
            (c, ht),
            (ht, ht),
            (ht, c),
            (-ht, c),
            (-ht, ht),
            (-c, ht),
            (-c, -ht),
            (-ht, -ht),
        ]
    )


def _debris_placements(
    origin_x: float,
    origin_y: float,
    cols: int,
    rows: int,
    *,
    spacing: float = 2.0,
) -> tuple[Placement, ...]:
    templates = (
        Polygon([(0, 0), (0.8, 0), (0.8, 0.8), (0, 0.8)]),
        Polygon([(0, 0), (0.9, 0), (0.45, 0.7)]),
        _trapezoid(0.9, 0.5, 0.6),
        _star(5, 0.5, 0.25),
        _gear(6, 0.38, 0.52),
        _c_shape(w=1.2, h=1.2, t=0.35),
        _s_shape(w=1.4, h=1.1, t=0.32),
    )
    out: list[Placement] = []
    idx = 0
    for row in range(rows):
        for col in range(cols):
            tmpl = templates[idx % len(templates)]
            theta = (idx * OFF_AXIS) % (2 * math.pi)
            out.append(
                Placement(
                    tmpl,
                    origin_x + col * spacing,
                    origin_y + row * spacing,
                    theta,
                )
            )
            idx += 1
    return tuple(out)


PENT_BOARD = _regular_ngon(5, 12.0, cx=12.0, cy=12.0, rotation=-math.pi / 2)
PENT_PART = _regular_ngon(5, 1.4, rotation=math.pi / 10)
PENT_RECT = Polygon([(0, 0), (2.5, 0), (2.5, 2.5), (0, 2.5)])
WRAP_PART = _c_shape(w=5.0, h=5.0, t=1.2)
TRAP_PART = _trapezoid(2.8, 1.6, 1.8)
RING_PART = _star(5, 1.1, 0.55, rotation=OFF_AXIS)
NOTCH_PART = Polygon([(0, 0), (1.5, 0), (0, 1.5)])
L_SHEET_PART = Polygon([(0, 0), (1.2, 0), (0.6, 1.0)])
L_SHEET_RECT = Polygon([(0, 0), (1.5, 0), (1.5, 1.5), (0, 1.5)])


def _hex_ring_placements(
    center: tuple[float, float],
    radius: float,
    *,
    angle_offset: float = OFF_AXIS,
) -> tuple[Placement, ...]:
    cx, cy = center
    return tuple(
        Placement(
            _star(5, 1.5, 0.75, rotation=angle_offset + a),
            cx + radius * math.cos(angle_offset + a),
            cy + radius * math.sin(angle_offset + a),
            angle_offset + a,
        )
        for a in (
            0.0,
            math.pi / 3,
            2 * math.pi / 3,
            math.pi,
            4 * math.pi / 3,
            5 * math.pi / 3,
        )
    )


SCENARIO_SPECS: tuple[ScenarioSpec, ...] = (
    # --- simple regression baseline (geometry unchanged) ---
    ScenarioSpec(
        label="nest_border_dock",
        board=NEST_BOARD,
        part=TRI_SMALL,
        obstacles=(
            Placement(RECT_0_1, 0.0, 0.0),
            Placement(RECT_0_1, 0.1, 0.0),
        ),
        seed=(0.35, 0.18, 0.0),
        pt_push=(0.0, 0.0),
        min_dist=0.02,
        expected_move_substrings=("Corner", "Snap", "Floor Walk", "Gravity", "Dock"),
        notes="triangle sheet, dock to border near rect cluster",
    ),
    ScenarioSpec(
        label="rect_cluster_gap",
        board=NEST_BOARD,
        part=RECT_0_1,
        obstacles=(
            Placement(RECT_0_1, 0.0, 0.0),
            Placement(RECT_0_1, 0.1, 0.0),
            Placement(RECT_0_1, 0.0, 0.1),
        ),
        seed=(0.15, 0.15, 0.0),
        pt_push=(0.05, 0.05),
        min_dist=0.02,
        expected_move_substrings=("Corner", "Snap"),
        notes="L-gap between three rects on triangle sheet",
    ),
    ScenarioSpec(
        label="tri_rect_mixed",
        board=NEST_BOARD,
        part=TRI_SMALL,
        obstacles=(
            Placement(RECT_0_1, 0.0, 0.0),
            Placement(TRI_SMALL, 0.8, 0.8),
        ),
        seed=(0.4, 0.4, 0.0),
        pt_push=(0.0, 0.0),
        min_dist=0.02,
        expected_move_substrings=("Corner", "Snap"),
        notes="mixed tri+rect obstacles, tri part mid-sheet",
    ),
    ScenarioSpec(
        label="l_shape_pocket",
        board=NEST_BOARD_LARGE,
        part=RECT_2,
        obstacles=(
            Placement(L_SHAPE, 10.0, 10.0),
            Placement(L_SHAPE, 15.0, 10.0, math.pi / 2),
        ),
        seed=(12.0, 12.0, 0.0),
        pt_push=(11.0, 11.0),
        min_dist=0.5,
        intentional_overlap=True,
        expected_move_substrings=("Ejection", "Corner", "Snap"),
        notes="rect in L-pocket overlap, ejection recovery",
    ),
    ScenarioSpec(
        label="l_snap_to_bar",
        board=NEST_BOARD,
        part=L_PART,
        obstacles=(Placement(BAR, 0.0, 0.0),),
        seed=(0.2, 0.25, math.pi),
        pt_push=(0.5, 0.05),
        min_dist=0.02,
        intentional_overlap=True,
        expected_move_substrings=("Ejection", "Snap", "Corner"),
        notes="rotated L-part overlapping bar, snap after ejection",
    ),
    ScenarioSpec(
        label="overlap_recovery",
        board=NEST_BOARD,
        part=TRI_SMALL,
        obstacles=(Placement(RECT_0_1, 0.5, 0.5),),
        seed=(0.52, 0.52, 0.0),
        pt_push=(0.5, 0.5),
        min_dist=0.02,
        intentional_overlap=True,
        expected_move_substrings=("Ejection",),
        notes="deliberate overlap, primary ejection only",
    ),
    ScenarioSpec(
        label="void_corner",
        board=NEST_BOARD,
        part=TRI_SMALL,
        obstacles=(Placement(RECT_0_1, 0.4, 0.4),),
        seed=(0.05, 0.05, 0.0),
        pt_push=(0.0, 0.0),
        min_dist=0.02,
        expected_move_substrings=("Corner", "Snap", "Neighbor"),
        notes="nest corner void, tri docks to origin",
    ),
    ScenarioSpec(
        label="board_hole_mouth",
        board=BOARD_WITH_HOLE,
        part=RECT_0_1,
        obstacles=(
            Placement(RECT_0_1, 0.2, 0.5),
            Placement(RECT_0_1, 0.9, 0.5),
        ),
        seed=(0.5, 0.9, 0.0),
        pt_push=(0.6, 0.6),
        min_dist=0.02,
        user_holes=HOLE_MOUTH,
        expected_move_substrings=("Corner", "Snap", "Neighbor"),
        notes="sheet void hole with flanking rects, mouth placement",
    ),
    ScenarioSpec(
        label="concave_notch",
        board=NEST_BOARD_LARGE,
        part=NOTCH_PART,
        obstacles=(
            Placement(L_SHAPE, 10.0, 10.0),
            Placement(RECT_2, 14.0, 10.0),
        ),
        seed=(12.5, 12.5, 0.0),
        pt_push=(12.0, 11.0),
        min_dist=0.5,
        expected_move_substrings=("Corner", "Snap"),
        notes="tri part in concave notch between L and rect",
    ),
    # --- complex hardened scenarios ---
    ScenarioSpec(
        label="tri_board_dense_pack",
        board=NEST_BOARD,
        part=DENSE_PART,
        obstacles=(
            Placement(DENSE_RECT, 0.0, 0.0),
            Placement(DENSE_RECT, 0.12, 0.0, OFF_AXIS),
            Placement(DENSE_TRI, 0.0, 0.12, OFF_AXIS / 2),
            Placement(_star(5, 0.08, 0.04), 0.18, 0.14, OFF_AXIS),
            Placement(_gear(6, 0.06, 0.09), 0.24, 0.17, OFF_AXIS),
        ),
        seed=(0.26, 0.21, OFF_AXIS),
        pt_push=(0.0, 0.0),
        min_dist=0.02,
        intentional_overlap=True,
        expected_move_substrings=("Ejection", "Corner", "Snap"),
        notes="dense cluster with star/gear flanking seed at tight clearance",
    ),
    ScenarioSpec(
        label="pentagon_board_cove",
        board=PENT_BOARD,
        part=PENT_PART,
        obstacles=(
            Placement(PENT_RECT, 8.0, 2.0),
            Placement(PENT_RECT, 10.5, 2.0),
            Placement(_regular_ngon(6, 1.8), 14.0, 4.5, 0.3),
            Placement(PENT_RECT, 6.5, 5.0, math.pi / 5),
            Placement(_s_shape(w=2.2, h=1.8, t=0.55), 8.8, 3.0),
            Placement(_cross(arm=1.8, t=0.55), 10.2, 3.8, OFF_AXIS),
            Placement(_gear(8, 0.9, 1.3), 9.5, 4.5, OFF_AXIS),
        ),
        seed=(13.2, 15.2, 0.5),
        pt_push=(10.0, 3.0),
        min_dist=0.26,
        border_focus=True,
        expected_move_substrings=("Corner", "Snap", "Dock", "Gravity", "Floor Walk"),
        notes="pentagon sheet, jagged s/cross/gear trap near seed",
    ),
    ScenarioSpec(
        label="triple_void_lane",
        board=LANE_BOARD,
        part=LANE_PART,
        obstacles=(
            Placement(LANE_RECT, 2.0, 1.5),
            Placement(LANE_RECT, 22.5, 1.5),
            Placement(_trapezoid(1.4, 0.9, 0.9), 8.5, 10.5, math.pi),
            Placement(LANE_RECT, 15.0, 10.0, math.pi / 2),
            Placement(_u_channel(w=1.4, h=2.2, t=0.35), 9.0, 4.5),
            Placement(_u_channel(w=1.4, h=2.2, t=0.35), 16.5, 4.5, math.pi),
            Placement(_gear(8, 0.45, 0.75), 15.2, 6.8, OFF_AXIS),
        ),
        seed=(16.2, 7.5, 0.0),
        pt_push=(14.0, 7.0),
        min_dist=0.20,
        user_holes=LANE_HOLES,
        expected_move_substrings=("Corner", "Snap", "Floor Walk"),
        notes="three void lanes + gear teeth protruding near seed",
    ),
    ScenarioSpec(
        label="c_shape_wrap_cluster",
        board=NEST_BOARD_LARGE,
        part=WRAP_PART,
        obstacles=(
            Placement(_cross(arm=3.0, t=1.1), 10.5, 10.5),
            Placement(_s_shape(w=4.5, h=3.5, t=1.0), 14.0, 11.0, OFF_AXIS),
            Placement(_gear(8, 1.2, 1.7), 12.0, 14.5, OFF_AXIS),
        ),
        seed=(11.8, 12.2, OFF_AXIS / 2),
        pt_push=(11.0, 11.5),
        min_dist=0.35,
        intentional_overlap=True,
        expected_move_substrings=("Ejection", "Corner", "Snap"),
        notes="C-part entangled with gear/s-shape, deeper overlap seed",
    ),
    ScenarioSpec(
        label="trapezoid_l_pocket",
        board=NEST_BOARD_LARGE,
        part=TRAP_PART,
        obstacles=(
            Placement(_gear(8, 1.4, 2.0), 8.0, 8.0, OFF_AXIS),
            Placement(TRAP_L, 14.5, 8.0, math.pi),
            Placement(RECT_2, 11.0, 12.5),
            Placement(_star(5, 1.5, 0.8), 10.8, 13.8, OFF_AXIS),
            Placement(RECT_2, 11.8, 15.0),
        ),
        seed=(11.5, 10.5, math.pi / 2),
        pt_push=(11.0, 10.0),
        min_dist=0.36,
        intentional_overlap=True,
        expected_move_substrings=("Ejection", "Corner", "Snap"),
        notes="trapezoid vs gear/star pocket, seed at concave mouth",
    ),
    ScenarioSpec(
        label="hex_ring_cavity",
        board=NEST_BOARD_LARGE,
        part=RING_PART,
        obstacles=_hex_ring_placements((15.0, 15.0), 4.2, angle_offset=OFF_AXIS),
        seed=(14.55, 15.05, OFF_AXIS),
        pt_push=(15.0, 15.0),
        min_dist=0.32,
        expected_move_substrings=("Corner", "Snap", "Floor Walk"),
        notes="star part tight against jagged star-ring cavity wall",
    ),
    ScenarioSpec(
        label="jagged_inner_dock",
        board=JAGGED_BOARD,
        part=JAGGED_PART,
        obstacles=(
            Placement(LANE_RECT, 4.0, 4.0),
            Placement(LANE_RECT, 6.5, 4.0),
            Placement(_regular_ngon(5, 1.5), 5.0, 7.0, 0.2),
            Placement(LANE_RECT, 16.0, 16.0, math.pi / 4),
            Placement(_u_channel(w=3.5, h=2.8, t=0.55), 1.2, 18.5),
            Placement(_u_channel(w=2.5, h=3.5, t=0.55), 1.2, 20.5, math.pi / 2),
            Placement(_gear(8, 0.7, 1.1), 2.8, 17.8, OFF_AXIS),
            Placement(_cross(arm=1.6, t=0.5), 1.8, 19.2, math.pi / 4),
        ),
        seed=(2.5, 14.2, 0.0),
        pt_push=(0.0, 24.0),
        min_dist=0.28,
        border_focus=True,
        expected_move_substrings=("Corner", "Snap", "Dock", "Gravity", "Floor Walk"),
        notes="U-channel dock with gear/cross guarding entrance near seed",
    ),
    ScenarioSpec(
        label="irregular_heptagon_cluster",
        board=NEST_BOARD_LARGE,
        part=HEPT_PART,
        obstacles=(
            Placement(_gear(8, 1.5, 2.1), 13.5, 14.8, OFF_AXIS),
            Placement(_star(5, 1.2, 0.6), 14.5, 15.5, OFF_AXIS),
            Placement(_c_shape(w=3.5, h=3.5, t=1.0), 9.0, 5.5),
            Placement(_regular_ngon(7, 1.7), 7.0, 9.0, 0.5),
            Placement(Polygon([(0, 0), (2.5, 0), (1.2, 2.2)]), 12.0, 8.0, math.pi / 3),
            Placement(RECT_2, 6.0, 12.0),
            *_debris_placements(3.0, 14.0, 5, 2, spacing=1.6),
        ),
        seed=(12.0, 13.5, OFF_AXIS / 2),
        pt_push=(7.0, 7.0),
        min_dist=0.32,
        intentional_overlap=True,
        expected_move_substrings=("Ejection", "Corner", "Snap"),
        notes="heptagon seed amid star/gear debris field",
    ),
    ScenarioSpec(
        label="l_shaped_sheet_corner",
        board=L_SHAPED_SHEET,
        part=L_SHEET_PART,
        obstacles=(
            Placement(_s_shape(w=2.0, h=1.6, t=0.45), 1.0, 1.0),
            Placement(_cross(arm=1.8, t=0.45), 2.6, 1.2, OFF_AXIS),
            Placement(_gear(6, 0.55, 0.85), 1.2, 2.6, OFF_AXIS),
            Placement(_star(5, 0.65, 0.32), 3.8, 3.2, OFF_AXIS),
        ),
        seed=(4.6, 4.4, math.pi / 4),
        pt_push=(0.0, 0.0),
        min_dist=0.28,
        border_focus=True,
        expected_move_substrings=("Corner", "Snap", "Dock"),
        notes="L-sheet inner corner, interlocking s/cross/gear cluster near seed",
    ),
    ScenarioSpec(
        label="narrow_strip_channel",
        board=STRIP_BOARD,
        part=STRIP_PART,
        obstacles=(
            Placement(STRIP_RECT, 3.0, 0.6),
            Placement(STRIP_RECT, 8.0, 3.6, math.pi),
            Placement(STRIP_RECT, 14.0, 0.5),
            Placement(_trapezoid(1.0, 0.6, 0.7), 20.0, 3.5, math.pi),
            Placement(STRIP_RECT, 26.0, 0.7),
            Placement(STRIP_RECT, 32.0, 3.4, math.pi / 2),
            Placement(_u_channel(w=1.3, h=2.0, t=0.35), 11.0, 2.5),
            Placement(_u_channel(w=1.3, h=2.0, t=0.35), 18.0, 0.8, math.pi),
            Placement(_s_shape(w=1.5, h=1.2, t=0.35), 25.0, 2.8, OFF_AXIS),
            Placement(_gear(8, 0.42, 0.68), 16.5, 2.0, OFF_AXIS),
        ),
        seed=(17.8, 2.6, 0.0),
        pt_push=(20.0, 2.5),
        min_dist=0.16,
        expected_move_substrings=("Corner", "Snap", "Floor Walk"),
        notes="strip maze with gear pinch point beside seed",
    ),
)

SCENARIO_IDS = tuple(spec.label for spec in SCENARIO_SPECS)


def _placed_obstacles(placements: tuple[Placement, ...]) -> list[Polygon]:
    return [_place_poly(p.poly, p.x, p.y, p.theta) for p in placements]


def _make_scenario(
    label: str,
    board: Polygon,
    part: Polygon,
    obstacles: list[Polygon],
    seed: tuple[float, float, float],
    pt_push: Point,
    min_dist: float,
    *,
    user_holes: tuple[tuple[tuple[float, float], ...], ...] = (),
    border_focus: bool = False,
    intentional_overlap: bool = False,
    expected_move_substrings: tuple[str, ...] = (),
    notes: str = "",
) -> GuidanceScenario:
    _assert_simple(board, label=f"{label} board")
    _assert_simple(part, label=f"{label} part")
    for i, obs in enumerate(obstacles):
        _assert_simple(obs, label=f"{label} obstacle[{i}]")
    part_g = Geometry.from_shapely(part)
    obs_g = [Geometry.from_shapely(o) for o in obstacles]
    return GuidanceScenario(
        label=label,
        scene=build_placement_scene(
            board, part_g, obs_g, user_holes=user_holes,
        ),
        part_shapely=part,
        obstacles_shapely=obstacles,
        seed=seed,
        pt_push=pt_push,
        min_dist=min_dist,
        board_outline=board,
        border_focus=border_focus,
        intentional_overlap=intentional_overlap,
        expected_move_substrings=expected_move_substrings,
        notes=notes,
    )


def _scenario_from_spec(spec: ScenarioSpec) -> GuidanceScenario:
    obstacles = _placed_obstacles(spec.obstacles)
    return _make_scenario(
        spec.label,
        spec.board,
        spec.part,
        obstacles,
        spec.seed,
        Point(*spec.pt_push),
        spec.min_dist,
        user_holes=spec.user_holes,
        border_focus=spec.border_focus,
        intentional_overlap=spec.intentional_overlap,
        expected_move_substrings=spec.expected_move_substrings,
        notes=spec.notes,
    )


def _kiss_error(poly: Polygon, obstacles: list[Polygon], min_dist: float) -> float:
    if not obstacles:
        return 0.0
    min_d = min(poly.distance(obs) for obs in obstacles)
    return abs(min_d - min_dist)


def _border_standoff_err(poly: Polygon, board_outline: Polygon, min_dist: float) -> float:
    d = poly.distance(board_outline.exterior)
    if d > min_dist * 2:
        return 0.0
    return abs(d - min_dist)


def run_scenario(scenario: GuidanceScenario) -> tuple[GuidanceScenarioResult, object]:
    cfg = guidance_config_for_propose(
        pt_push=scenario.pt_push,
        min_dist=scenario.min_dist,
        board_bounds=scenario.scene.sheet.bounds,
        target_angle_rad=scenario.seed[2],
        border_focus=scenario.border_focus,
    )

    placed = scenario.scene.placed_at(scenario.seed)
    seed_poly = _placed_shapely(scenario.part_shapely, scenario.seed)
    seed_kiss_error = _kiss_error(
        seed_poly, scenario.obstacles_shapely, scenario.min_dist,
    )
    seed_board_valid = is_valid_placement(
        scenario.scene,
        placed,
        (scenario.seed[0], scenario.seed[1]),
        scenario.min_dist,
        cfg,
    )

    t0 = time.perf_counter()
    guidance = scenario.scene.guidance(placed, (scenario.seed[0], scenario.seed[1]), cfg)
    exec_time_ms = (time.perf_counter() - t0) * 1000.0

    best_prop = best_proposition(guidance)
    best_move_type = best_prop.move_type if best_prop else "None"
    move_types = [p.move_type for p in guidance.propositions]

    if best_prop:
        tx, ty = best_prop.translation
        new_placed = placed.apply_transform(
            (scenario.seed[0] + tx, scenario.seed[1] + ty, scenario.seed[2]),
        )
        board_valid = is_valid_placement(
            scenario.scene,
            new_placed,
            (scenario.seed[0] + tx, scenario.seed[1] + ty),
            scenario.min_dist,
            cfg,
        )
        best_poly = _placed_shapely(
            scenario.part_shapely,
            scenario.seed,
            dx=best_prop.translation[0],
            dy=best_prop.translation[1],
            dtheta=best_prop.rotation_rad,
        )
        kiss_error = _kiss_error(
            best_poly, scenario.obstacles_shapely, scenario.min_dist,
        )
        border_standoff_err = _border_standoff_err(
            best_poly, scenario.board_outline, scenario.min_dist,
        )
    else:
        board_valid = False
        kiss_error = seed_kiss_error
        border_standoff_err = 0.0

    res = GuidanceScenarioResult(
        scenario_id=scenario.label,
        shapes="part + obs",
        props_count=len(guidance.propositions),
        is_penetrating=guidance.is_penetrating,
        best_move_type=best_move_type,
        move_types=move_types,
        kiss_error=kiss_error,
        seed_kiss_error=seed_kiss_error,
        border_standoff_err=border_standoff_err,
        board_valid=board_valid,
        seed_board_valid=seed_board_valid,
        execution_time_ms=exec_time_ms,
    )
    return res, guidance


def get_scenarios() -> list[GuidanceScenario]:
    return [_scenario_from_spec(spec) for spec in SCENARIO_SPECS]
