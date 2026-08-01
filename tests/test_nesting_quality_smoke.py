"""Smoke tests for nesting quality benchmarks."""

from dataclasses import replace

from nest_graph.config import (
    BuildGraphConfig,
    OutputConfig,
    ProposeConfig,
    SamplingConfig,
    SelectionConfig,
)
from scripts.nesting_evaluator import NestingPipelineEvaluator
from scripts.nesting_fixtures import get_all_cases, make_swiss_cheese


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
    for case in cases:
        assert 0.5 <= case.demand_ratio <= 1.6, (case.name, case.demand_ratio)
        assert all(n <= 90 for n in case.group_counts), (case.name, case.group_counts)
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
    assert metrics.parts_delta > 0
    assert metrics.parts_final > metrics.parts_seed
    assert metrics.area_coverage_delta > 0.0
