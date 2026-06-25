import math
import time
from dataclasses import dataclass

from shapely.affinity import rotate, translate
from shapely.geometry import Point, Polygon, box

from nest_graph.geometry import Geometry
from nest_graph.placement_scene import (
    PlacementScene,
    best_proposition,
    build_placement_scene,
    guidance_config_for_propose,
    is_valid_placement,
)

SCENARIO_IDS = (
    "nest_border_dock",
    "rect_cluster_gap",
    "tri_rect_mixed",
    "l_shape_pocket",
    "l_snap_to_bar",
    "overlap_recovery",
    "void_corner",
    "board_hole_mouth",
    "concave_notch",
)


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


def _place_poly(poly: Polygon, x: float, y: float, theta: float = 0.0) -> Polygon:
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
    nest_board = Polygon([(0, 0), (1.2, 0), (0, 1.1)])
    nest_board_large = Polygon([(0, 0), (30, 0), (30, 30), (0, 30)])

    rect_poly = Polygon([(0, 0), (0.1, 0), (0.1, 0.1), (0, 0.1)])
    tri_poly = Polygon([(0, 0), (0.15, 0), (0, 0.07)])
    l_shape_poly = Polygon([(0, 0), (4, 0), (4, 2), (2, 2), (2, 4), (0, 4)])

    scenarios: list[GuidanceScenario] = []

    # 1. nest_border_dock
    obstacles = [
        _place_poly(rect_poly, 0.0, 0.0),
        _place_poly(rect_poly, 0.1, 0.0),
    ]
    part = tri_poly
    seed = (0.35, 0.18, 0.0)
    part_g = Geometry.from_shapely(part)
    obs_g = [Geometry.from_shapely(o) for o in obstacles]
    scenarios.append(
        GuidanceScenario(
            label="nest_border_dock",
            scene=build_placement_scene(nest_board, part_g, obs_g),
            part_shapely=part,
            obstacles_shapely=obstacles,
            seed=seed,
            pt_push=Point(0.0, 0.0),
            min_dist=0.02,
            board_outline=nest_board,
            expected_move_substrings=("Corner", "Snap", "Floor Walk", "Gravity", "Dock"),
        )
    )

    # 2. rect_cluster_gap
    obstacles = [
        _place_poly(rect_poly, 0.0, 0.0),
        _place_poly(rect_poly, 0.1, 0.0),
        _place_poly(rect_poly, 0.0, 0.1),
    ]
    part = rect_poly
    seed = (0.15, 0.15, 0.0)
    part_g = Geometry.from_shapely(part)
    obs_g = [Geometry.from_shapely(o) for o in obstacles]
    scenarios.append(
        GuidanceScenario(
            label="rect_cluster_gap",
            scene=build_placement_scene(nest_board, part_g, obs_g),
            part_shapely=part,
            obstacles_shapely=obstacles,
            seed=seed,
            pt_push=Point(0.05, 0.05),
            min_dist=0.02,
            board_outline=nest_board,
            expected_move_substrings=("Corner", "Snap"),
        )
    )

    # 3. tri_rect_mixed
    obstacles = [
        _place_poly(rect_poly, 0.0, 0.0),
        _place_poly(tri_poly, 0.8, 0.8),
    ]
    part = tri_poly
    seed = (0.4, 0.4, 0.0)
    part_g = Geometry.from_shapely(part)
    obs_g = [Geometry.from_shapely(o) for o in obstacles]
    scenarios.append(
        GuidanceScenario(
            label="tri_rect_mixed",
            scene=build_placement_scene(nest_board, part_g, obs_g),
            part_shapely=part,
            obstacles_shapely=obstacles,
            seed=seed,
            pt_push=Point(0.0, 0.0),
            min_dist=0.02,
            board_outline=nest_board,
            expected_move_substrings=("Corner", "Snap"),
        )
    )

    # 4. l_shape_pocket
    obstacles = [
        _place_poly(l_shape_poly, 10.0, 10.0),
        _place_poly(l_shape_poly, 15.0, 10.0, math.pi / 2),
    ]
    part = Polygon([(0, 0), (2, 0), (2, 2), (0, 2)])
    seed = (12.0, 12.0, 0.0)
    part_g = Geometry.from_shapely(part)
    obs_g = [Geometry.from_shapely(o) for o in obstacles]
    scenarios.append(
        GuidanceScenario(
            label="l_shape_pocket",
            scene=build_placement_scene(nest_board_large, part_g, obs_g),
            part_shapely=part,
            obstacles_shapely=obstacles,
            seed=seed,
            pt_push=Point(11.0, 11.0),
            min_dist=0.5,
            board_outline=nest_board_large,
            intentional_overlap=True,
            expected_move_substrings=("Ejection", "Corner", "Snap"),
        )
    )

    # 5. l_snap_to_bar
    bar = Polygon([(0, 0), (1.0, 0), (1.0, 0.1), (0, 0.1)])
    part = Polygon([(0, 0), (0.4, 0), (0.4, 0.2), (0.2, 0.2), (0.2, 0.4), (0, 0.4)])
    seed = (0.2, 0.25, math.pi)
    part_g = Geometry.from_shapely(part)
    obs_g = [Geometry.from_shapely(bar)]
    scenarios.append(
        GuidanceScenario(
            label="l_snap_to_bar",
            scene=build_placement_scene(nest_board, part_g, obs_g),
            part_shapely=part,
            obstacles_shapely=[bar],
            seed=seed,
            pt_push=Point(0.5, 0.05),
            min_dist=0.02,
            board_outline=nest_board,
            intentional_overlap=True,
            expected_move_substrings=("Ejection", "Snap", "Corner"),
        )
    )

    # 6. overlap_recovery
    obstacles = [_place_poly(rect_poly, 0.5, 0.5)]
    part = tri_poly
    seed = (0.52, 0.52, 0.0)
    part_g = Geometry.from_shapely(part)
    obs_g = [Geometry.from_shapely(o) for o in obstacles]
    scenarios.append(
        GuidanceScenario(
            label="overlap_recovery",
            scene=build_placement_scene(nest_board, part_g, obs_g),
            part_shapely=part,
            obstacles_shapely=obstacles,
            seed=seed,
            pt_push=Point(0.5, 0.5),
            min_dist=0.02,
            board_outline=nest_board,
            intentional_overlap=True,
            expected_move_substrings=("Ejection",),
        )
    )

    # 7. void_corner
    obstacles = [_place_poly(rect_poly, 0.4, 0.4)]
    part = tri_poly
    seed = (0.05, 0.05, 0.0)
    part_g = Geometry.from_shapely(part)
    obs_g = [Geometry.from_shapely(o) for o in obstacles]
    scenarios.append(
        GuidanceScenario(
            label="void_corner",
            scene=build_placement_scene(nest_board, part_g, obs_g),
            part_shapely=part,
            obstacles_shapely=obstacles,
            seed=seed,
            pt_push=Point(0.0, 0.0),
            min_dist=0.02,
            board_outline=nest_board,
            expected_move_substrings=("Corner", "Snap", "Neighbor"),
        )
    )

    # 8. board_hole_mouth
    board_with_hole = box(0, 0, 1.2, 1.1)
    user_holes = (((0.4, 0.4), (0.8, 0.4), (0.8, 0.8), (0.4, 0.8)),)
    obstacles = [
        _place_poly(rect_poly, 0.2, 0.5),
        _place_poly(rect_poly, 0.9, 0.5),
    ]
    part = rect_poly
    seed = (0.5, 0.9, 0.0)
    part_g = Geometry.from_shapely(part)
    obs_g = [Geometry.from_shapely(o) for o in obstacles]
    scenarios.append(
        GuidanceScenario(
            label="board_hole_mouth",
            scene=build_placement_scene(
                board_with_hole, part_g, obs_g, user_holes=user_holes,
            ),
            part_shapely=part,
            obstacles_shapely=obstacles,
            seed=seed,
            pt_push=Point(0.6, 0.6),
            min_dist=0.02,
            board_outline=board_with_hole,
            expected_move_substrings=("Corner", "Snap", "Neighbor"),
        )
    )

    # 9. concave_notch
    obstacles = [
        _place_poly(l_shape_poly, 10.0, 10.0),
        _place_poly(Polygon([(0, 0), (2, 0), (2, 2), (0, 2)]), 14.0, 10.0),
    ]
    part = Polygon([(0, 0), (1.5, 0), (0, 1.5)])
    seed = (12.5, 12.5, 0.0)
    part_g = Geometry.from_shapely(part)
    obs_g = [Geometry.from_shapely(o) for o in obstacles]
    scenarios.append(
        GuidanceScenario(
            label="concave_notch",
            scene=build_placement_scene(nest_board_large, part_g, obs_g),
            part_shapely=part,
            obstacles_shapely=obstacles,
            seed=seed,
            pt_push=Point(12.0, 11.0),
            min_dist=0.5,
            board_outline=nest_board_large,
            expected_move_substrings=("Corner", "Snap"),
        )
    )

    return scenarios
