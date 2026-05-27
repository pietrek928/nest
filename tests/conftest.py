"""Shared geometry / placement fixtures for nest_graph tests."""

import numpy as np
import pytest
from shapely.geometry import Polygon

from nest_graph.utils import normalize_poly

# Shapely intersects True, Geometry.intersects False before boundary-ring fix.
REGRESSION_INTERSECT_TRANSFORMS = [
    (
        np.array([0.24065339, -0.05312546, 2.10871156]),
        np.array([0.33178908, 0.00257389, 1.11415098]),
    ),
    (
        np.array([-0.3216425, 0.16240434, 0.50278479]),
        np.array([-0.33014424, 0.15345384, -2.12770036]),
    ),
]

# Two L placements on a large board: Shapely disjoint, from_convex_polygon intersects.
CONCAVE_PLACEMENT_TRANSFORMS = (
    np.array([10.37628498, 6.93504997, 105.45521565]),
    np.array([15.04330134, 5.95749685, 76.19882152]),
)


def pytest_generate_tests(metafunc):
    if {"t1", "t2"}.issubset(metafunc.fixturenames):
        metafunc.parametrize("t1,t2", REGRESSION_INTERSECT_TRANSFORMS)


@pytest.fixture
def nest_board() -> Polygon:
    return Polygon([(0, 0), (1.2, 0), (0, 1.1)])


@pytest.fixture
def nest_board_large() -> Polygon:
    return Polygon([(0, 0), (30, 0), (30, 30), (0, 30)])


@pytest.fixture
def rect_poly():
    return normalize_poly(Polygon([(0, 0), (0.1, 0), (0.1, 0.1), (0, 0.1)]))


@pytest.fixture
def tri_poly():
    return normalize_poly(Polygon([(0, 0), (0.15, 0), (0, 0.07)]))


@pytest.fixture
def l_shape_poly():
    return normalize_poly(Polygon([(0, 0), (4, 0), (4, 2), (2, 2), (2, 4), (0, 4)]))


@pytest.fixture
def l_shape_raw() -> Polygon:
    return Polygon([(0, 0), (4, 0), (4, 2), (2, 2), (2, 4), (0, 4)])


@pytest.fixture
def notch_square() -> Polygon:
    return Polygon([(2.5, 2.5), (3.5, 2.5), (3.5, 3.5), (2.5, 3.5)])


@pytest.fixture
def concave_placement_transforms():
    return CONCAVE_PLACEMENT_TRANSFORMS


@pytest.fixture
def first_regression_intersect_transform():
    return REGRESSION_INTERSECT_TRANSFORMS[0]


@pytest.fixture
def small_transforms():
    def _draw(n: int, seed: int = 0) -> np.ndarray:
        rng = np.random.default_rng(seed)
        return rng.uniform(-0.3, 0.3, (n, 3)) * [1.0, 1.0, 2 * np.pi]

    return _draw


@pytest.fixture
def nest_board_donut() -> Polygon:
    from shapely.geometry import box
    outer = box(0, 0, 10, 10)
    hole = box(3, 3, 7, 7)
    return outer.difference(hole)


@pytest.fixture
def build_graph_config():
    """Small limits for fast CI / single-iteration smoke tests."""
    from nest_graph.config import (
        BuildGraphConfig,
        OutputConfig,
        ProposeConfig,
        SamplingConfig,
        SelectionConfig,
    )

    return BuildGraphConfig(
        sampling=SamplingConfig(
            random_per_iter=32,
            initial_random=32,
            max_transforms_per_group=80,
            history_max=64,
            seed=0,
        ),
        selection=SelectionConfig(improve_rules_rounds=1),
        propose=ProposeConfig(
            max_proposals=8,
            candidate_pool=8,
            point_cloud_particles=8,
            point_cloud_iterations=12,
            voronoi_max_sites=24,
        ),
        output=OutputConfig(n_iters=1, progress=False),
    )
