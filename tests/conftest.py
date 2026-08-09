"""Shared geometry / placement fixtures for nest_graph tests."""

import numpy as np
import pytest
from shapely.geometry import Polygon

from tests.fixtures.shapes import (
    donut,
    l_shape,
    l_shape_raw as l_shape_raw_factory,
    nest_board_large as nest_board_large_factory,
    nest_triangle_board,
    notch_square as notch_square_factory,
    rect_poly as rect_poly_factory,
    tri_poly as tri_poly_factory,
)

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
    return nest_triangle_board()


@pytest.fixture
def nest_board_large() -> Polygon:
    return nest_board_large_factory()


@pytest.fixture
def rect_poly():
    return rect_poly_factory()


@pytest.fixture
def tri_poly():
    return tri_poly_factory()


@pytest.fixture
def l_shape_poly():
    return l_shape()


@pytest.fixture
def l_shape_raw() -> Polygon:
    return l_shape_raw_factory()


@pytest.fixture
def notch_square() -> Polygon:
    return notch_square_factory()


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
    return donut()


from nest_graph.config import (
    BuildGraphConfig,
    OutputConfig,
    ProposeConfig,
    SamplingConfig,
    SelectionConfig,
)

@pytest.fixture
def build_graph_config():
    """Small limits for fast CI / single-iteration smoke tests."""
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
