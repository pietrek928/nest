"""Native-only local SE(2) polish tests."""

import ast
from pathlib import Path

import numpy as np
from shapely.geometry import Point, box

from nest_graph.config import ProposeConfig
from nest_graph.geometry import Geometry, polish_se2_part
from nest_graph.placement.common import selection_pairwise_independent
from nest_graph.propose.local_se2 import local_se2_selection
from nest_graph.utils import transform_poly


def test_local_se2_module_has_no_python_grid():
    src = Path(__file__).resolve().parents[1] / "nest_graph" / "propose" / "local_se2.py"
    tree = ast.parse(src.read_text())
    calls = [
        n.func.id
        for n in ast.walk(tree)
        if isinstance(n, ast.Call) and isinstance(n.func, ast.Name)
    ]
    assert "batch_check_validity" not in calls
    text = src.read_text()
    assert "batch_check_validity" not in text
    assert "local_se2_use_native_polish" not in text
    assert "polish_se2_part" in text


def test_config_has_no_native_polish_flag():
    assert not hasattr(ProposeConfig(), "local_se2_use_native_polish")


def test_free_slide_moves_toward_pole():
    sheet = box(0, 0, 20, 20)
    part = box(0, 0, 1, 1)
    tr0 = np.array([2.0, 2.0, 0.0])
    polys = [transform_poly(part, tr0)]
    pole = Point(15.0, 15.0)
    cfg = ProposeConfig(enable_local_se2=True, local_se2_n_angles=4)
    d0 = float(polys[0].centroid.distance(pole))

    part_g = Geometry.from_shapely(part)
    board_g = Geometry.from_shapely(sheet)
    cx, cy = float(polys[0].centroid.x), float(polys[0].centroid.y)
    dx, dy = float(pole.x) - cx, float(pole.y) - cy
    n = (dx * dx + dy * dy) ** 0.5
    ux, uy = dx / n, dy / n
    dirs = [(ux, uy)]
    polished = polish_se2_part(
        part_g,
        (float(tr0[0]), float(tr0[1]), float(tr0[2])),
        [],
        board_g,
        dirs,
        n_angles=4,
        max_t=30.0,
        min_dist=0.25,
        mode="pole",
        pole=(float(pole.x), float(pole.y)),
    )
    assert polished is not None
    bx, by, _ = polished
    moved = transform_poly(part, np.array([bx, by, polished[2]]))
    assert float(moved.centroid.distance(pole)) < d0

    out_p, out_t, stats = local_se2_selection(
        sheet,
        polys,
        [tr0],
        [0],
        [0],
        {0: part},
        min_dist=0.25,
        propose_cfg=cfg,
        pole=pole,
        board_adj_indices=[],
    )
    d1 = float(out_p[0].centroid.distance(pole))
    assert stats["attempted"] == 1
    assert stats["se2_native_hits"] == 1
    assert stats["se2_native_accepted"] >= 1
    assert stats["moved"] >= 1
    assert d1 < d0
    assert selection_pairwise_independent(out_p, [0])
    board_g2 = Geometry.from_shapely(sheet)
    assert Geometry.from_shapely(out_p[0]).footprint_inside(board_g2)


def test_jammed_part_unchanged_or_none():
    sheet = box(0, 0, 10, 10)
    part = box(0, 0, 1, 1)
    # Part in a tight cell: gaps ~0.1 so cast_slide travel after min_dist is ~0.
    tr0 = np.array([2.0, 2.0, 0.0])
    walls = [
        box(1.0, 1.5, 1.9, 3.5),
        box(3.1, 1.5, 4.0, 3.5),
        box(1.5, 1.0, 3.5, 1.9),
        box(1.5, 3.1, 3.5, 4.0),
    ]
    placed = transform_poly(part, tr0)
    pole = Point(8.0, 8.0)
    cfg = ProposeConfig(enable_local_se2=True, local_se2_n_angles=4)
    min_dist = 0.25

    part_g = Geometry.from_shapely(part)
    board_g = Geometry.from_shapely(sheet)
    wall_gs = [Geometry.from_shapely(w) for w in walls]
    polished = polish_se2_part(
        part_g,
        (float(tr0[0]), float(tr0[1]), float(tr0[2])),
        wall_gs,
        board_g,
        [(1.0, 0.0), (0.0, 1.0), (0.707, 0.707), (-1.0, 0.0)],
        n_angles=4,
        max_t=20.0,
        min_dist=min_dist,
        mode="pole",
        pole=(float(pole.x), float(pole.y)),
    )
    assert polished is None

    out_p, out_t, stats = local_se2_selection(
        sheet,
        [placed],
        [tr0],
        [0],
        [0],
        {0: part},
        min_dist=min_dist,
        propose_cfg=cfg,
        pole=pole,
        fixed_obstacles=walls,
        board_adj_indices=[],
    )
    assert stats["attempted"] == 1
    assert stats["se2_native_hits"] == 1
    assert stats["se2_native_accepted"] == 0
    assert stats["moved"] == 0
    assert np.allclose(out_t[0], tr0)
    assert out_p[0].equals(placed)
    assert selection_pairwise_independent(out_p, [0])


def test_oblique_slide_reaches_near_min_dist():
    """B: 45° approach to a wall ends near min_dist (not ~half travel)."""
    sheet = box(0, 0, 20, 20)
    part = box(0, 0, 1, 1)
    wall = box(8.0, 0.0, 12.0, 20.0)
    min_dist = 0.25
    # Start left of wall; slide toward (+1,+1) hits the wall obliquely.
    tr0 = (2.0, 8.0, 0.0)
    part_g = Geometry.from_shapely(part)
    board_g = Geometry.from_shapely(sheet)
    wall_g = Geometry.from_shapely(wall)
    n = (2.0 ** 0.5)
    polished = polish_se2_part(
        part_g,
        tr0,
        [wall_g],
        board_g,
        [(1.0 / n, 1.0 / n)],
        n_angles=1,
        max_t=20.0,
        min_dist=min_dist,
        mode="slide",
        pole=None,
    )
    assert polished is not None
    bx, by, bth = polished
    moved = part_g.rotate(bth).translate(bx, by)
    gap = moved.standoff_distance(wall_g)
    assert gap + 1e-4 >= min_dist
    # Halving left ~0.5–0.8 residual; bisect should land within 2× min_dist.
    assert gap <= min_dist * 2.0 + 1e-3


def test_restore_away_if_slack_flat_records_honest_dfit():
    from nest_graph.propose.local_se2 import _restore_away_if_slack_flat

    sheet = box(0, 0, 20, 20)
    part = box(0, 0, 2, 2)
    # Two packed parts leaving a thin crescent — fit slack stays negative.
    tr0 = np.array([2.0, 2.0, 0.0])
    tr1 = np.array([6.0, 2.0, 0.0])
    polys = [transform_poly(part, tr0), transform_poly(part, tr1)]
    # Fake "moved" state: shift first part (still packed similarly).
    moved = [transform_poly(part, np.array([2.2, 2.0, 0.0])), polys[1]]
    snap_tr = [tr0.copy(), tr1.copy()]
    moved_tr = [np.array([2.2, 2.0, 0.0]), tr1.copy()]
    snap_geoms = [None, None]
    moved_geoms = [None, None]
    stats: dict = {}
    pre_slack = -0.05
    _restore_away_if_slack_flat(
        sheet=sheet,
        out_polys=moved,
        out_tr=moved_tr,
        out_geoms=moved_geoms,
        sel=[0, 1],
        snap_polys=list(polys),
        snap_tr=snap_tr,
        snap_geoms=snap_geoms,
        pre_slack=pre_slack,
        mean_part_area=4.0,
        min_dist=0.25,
        void_ratio_threshold=2.5,
        fit_shape=part,
        stats=stats,
    )
    assert "hole_open_dfit" in stats
    assert int(stats["hole_open_slack_restore"]) == 0
    # R0: mutation off; would_restore flags honest regress without mutating.
    assert not moved[0].equals(polys[0]) or float(stats["hole_open_dfit"]) >= -1e-9


def test_restore_keeps_away_when_clearance_grew_even_if_slack_flat():
    from nest_graph.propose.local_se2 import _restore_away_if_slack_flat

    sheet = box(0, 0, 20, 20)
    part = box(0, 0, 2, 2)
    tr0 = np.array([2.0, 2.0, 0.0])
    tr1 = np.array([6.0, 2.0, 0.0])
    polys = [transform_poly(part, tr0), transform_poly(part, tr1)]
    moved = [transform_poly(part, np.array([2.2, 2.0, 0.0])), polys[1]]
    moved_tr = [np.array([2.2, 2.0, 0.0]), tr1.copy()]
    stats = {"hole_open_fit": 1}
    _restore_away_if_slack_flat(
        sheet=sheet,
        out_polys=moved,
        out_tr=moved_tr,
        out_geoms=[None, None],
        sel=[0, 1],
        snap_polys=list(polys),
        snap_tr=[tr0.copy(), tr1.copy()],
        snap_geoms=[None, None],
        pre_slack=-0.05,
        mean_part_area=4.0,
        min_dist=0.25,
        void_ratio_threshold=2.5,
        fit_shape=part,
        stats=stats,
    )
    assert int(stats["hole_open_slack_restore"]) == 0
    assert int(stats["hole_open_fit"]) == 1
    assert not moved[0].equals(polys[0])
