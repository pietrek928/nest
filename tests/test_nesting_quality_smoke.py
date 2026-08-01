"""Smoke tests for nesting quality benchmarks."""

import math
from dataclasses import replace

from nest_graph.config import (
    BuildGraphConfig,
    OutputConfig,
    ProposeConfig,
    SamplingConfig,
    SelectionConfig,
)
from nest_graph.geometry import Geometry
from nest_graph.utils import transform_poly
from scripts.nesting_evaluator import NestingPipelineEvaluator, _angle_allowed
from scripts.nesting_fixtures import (
    CaseSpec,
    build_nest_case,
    get_all_cases,
    make_donut_parts_pack,
    make_grain_locked_strips,
    make_organic_board_pack,
    make_swiss_cheese,
    resolve_cases,
)


def _smoke_cfg() -> BuildGraphConfig:
    return BuildGraphConfig(
        sampling=SamplingConfig(
            random_per_iter=16,
            initial_random=16,
            max_transforms_per_group=48,
            history_max=32,
            seed=0,
        ),
        selection=SelectionConfig(
            improve_rules_rounds=1,
            dfs_mode="merged_loose_tight",
            dfs_passes=1,
            dfs_max_tries=2,
        ),
        propose=ProposeConfig(
            max_proposals=8,
            candidate_pool=8,
            point_cloud_particles=8,
            point_cloud_iterations=8,
            voronoi_max_sites=16,
        ),
        output=OutputConfig(n_iters=1, progress=False),
    )


def test_fixture_demand_and_hole_separation():
    cases = get_all_cases()
    names = [c.name for c in cases]
    assert len(names) == len(set(names))
    for case in cases:
        assert 0.5 <= case.demand_ratio <= 1.6, (case.name, case.demand_ratio)
        assert all(n <= 90 for n in case.group_counts), (case.name, case.group_counts)
        assert all(ch.isalnum() or ch in "_.-" for ch in case.name), case.name
        holes = list(case.board_holes)
        for i, hi in enumerate(holes):
            assert case.board.contains(hi.centroid)
            for hj in holes[i + 1:]:
                assert hi.distance(hj) > 0.5, (case.name, hi.distance(hj))
        for poly, _gid, _t in case.seed_placements:
            assert case.board.intersects(poly)


def test_swiss_cheese_holes_are_separated():
    case = make_swiss_cheese()
    holes = list(case.board_holes)
    assert len(holes) == 5
    for i, hi in enumerate(holes):
        for hj in holes[i + 1:]:
            assert hi.distance(hj) >= 1.0


def test_demo_triangle_smoke():
    cases = get_all_cases()
    case = next(c for c in cases if c.name.startswith("demo_triangle"))
    case = replace(case, iters=1)
    evaluator = NestingPipelineEvaluator(case, _smoke_cfg())
    metrics = evaluator.run_full_pipeline(seed=0)

    assert metrics.independent_ok
    assert metrics.overlap_ok
    assert metrics.void_ok
    assert metrics.parts_final > 0
    assert metrics.area_coverage > 0.0
    assert metrics.coverage_trajectory


def test_donut_void_smoke():
    cases = get_all_cases()
    case = next(c for c in cases if c.name.startswith("donut_void"))
    case = replace(case, iters=1)
    evaluator = NestingPipelineEvaluator(case, _smoke_cfg())
    metrics = evaluator.run_full_pipeline(seed=0)

    assert metrics.independent_ok
    assert metrics.overlap_ok
    assert metrics.void_ok
    assert metrics.parts_final > 0
    assert metrics.area_coverage > 0.0


def test_tight_border_ring_smoke():
    cases = get_all_cases()
    case = next(c for c in cases if c.name.startswith("tight_border_ring"))
    case = replace(case, iters=1)
    evaluator = NestingPipelineEvaluator(case, _smoke_cfg())
    metrics = evaluator.run_full_pipeline(seed=0)

    assert metrics.independent_ok
    assert metrics.overlap_ok
    assert metrics.void_ok
    assert metrics.parts_seed > 0
    assert metrics.parts_final >= metrics.parts_seed
    assert metrics.area_coverage > 0.0
    # Lean smoke caps may not always add new parts on a fully rim-seeded sheet.


def test_part_with_hole_geometry():
    case = make_donut_parts_pack()
    frame = case.groups[0][0]
    assert list(frame.interiors)
    Geometry.from_shapely(frame)
    moved = transform_poly(frame, (1.0, 2.0, 0.3))
    assert list(moved.interiors)


def test_organic_board_smoke():
    case = replace(make_organic_board_pack(), iters=1)
    evaluator = NestingPipelineEvaluator(case, _smoke_cfg())
    metrics = evaluator.run_full_pipeline(seed=0)
    assert metrics.independent_ok
    assert metrics.overlap_ok
    assert metrics.void_ok


def test_grain_angles_respected():
    case = replace(make_grain_locked_strips(), iters=1)
    assert case.group_allowed_angles
    allowed0 = case.group_allowed_angles[0]
    assert allowed0 is not None
    evaluator = NestingPipelineEvaluator(case, _smoke_cfg())
    metrics = evaluator.run_full_pipeline(seed=0)
    assert metrics.independent_ok
    result = evaluator.last_result
    assert result is not None
    gids = result["group_id"]
    transforms = result["transform"]
    for i, gid in enumerate(gids):
        # NestCase gid 0 is the grain-locked needle group.
        if int(gid) != 0:
            continue
        assert _angle_allowed(float(transforms[i][2]), allowed0), transforms[i][2]


def test_resolve_cases_by_family_and_tag():
    by_family = resolve_cases(["demo_triangle"])
    assert any(c.name.startswith("demo_triangle") for c in by_family)
    by_tag = resolve_cases(tags=["tight_pack"])
    assert by_tag
    assert all("tight_pack" in c.tags for c in by_tag)


def test_case_spec_determinism():
    spec = CaseSpec(
        family="needle_stress",
        board="rect",
        board_params=(14.0, 14.0),
        part_catalog=("needle", "small_rect"),
        part_scales=(1.0, 1.0),
        target_demand=1.1,
        iters=2,
        max_per_group=90,
        tags=frozenset(["aspect"]),
        mix=(0.55, 0.45),
    )
    a = build_nest_case(spec)
    b = build_nest_case(spec)
    assert a.name == b.name
    assert a.group_counts == b.group_counts
    assert abs(a.demand_ratio - b.demand_ratio) < 1e-9
