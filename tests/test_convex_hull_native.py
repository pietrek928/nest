"""Native convex hull + contact_hybrid ranking / selection quality (Shapely oracle)."""

import pytest
from shapely import MultiPoint, Polygon
from shapely.geometry import Point, box

from nest_graph.config import ProposeConfig
from nest_graph.elem_graph import (
    PoseGraph,
    Circle,
    Vec2,
    nest_by_scores,
    refine_selection,
    RefineSelectionOptions,
)
from nest_graph.geometry import (
    Geometry,
    PlacementRankConfig,
    PlacementRankMode,
    batch_score_placed_contact_hybrid,
    convex_hull_area_of,
)
from nest_graph.propose.geometry import ProposeGeometry
from nest_graph.propose.placement_outline import outline_kiss_tolerance, outline_ring_geom
from nest_graph.propose.ranking import (
    _batch_rank_results,
    _coord_key,
    calculate_complex_score,
)
from nest_graph.propose.void_selection import boost_selection_geom_quality


def test_convex_hull_area_matches_shapely_square():
    poly = box(0, 0, 2, 3)
    g = Geometry.from_shapely(poly)
    assert g.convex_hull_area() == pytest.approx(poly.convex_hull.area, rel=1e-6, abs=1e-6)
    assert convex_hull_area_of([g]) == pytest.approx(poly.convex_hull.area, rel=1e-6)


def test_convex_hull_area_concave_and_union_cloud():
    concave = Polygon([(0, 0), (3, 0), (3, 1), (1, 1), (1, 2), (3, 2), (3, 3), (0, 3)])
    g = Geometry.from_shapely(concave)
    assert g.convex_hull_area() == pytest.approx(
        concave.convex_hull.area, rel=1e-5, abs=1e-5,
    )
    assert g.convex_hull_area() > g.area() + 0.5

    a = Geometry.from_shapely(box(0, 0, 1, 1))
    b = Geometry.from_shapely(box(2, 2, 3, 3))
    native = convex_hull_area_of([a, b])
    pts = list(box(0, 0, 1, 1).exterior.coords) + list(box(2, 2, 3, 3).exterior.coords)
    shapely_area = MultiPoint(pts).convex_hull.area
    assert native == pytest.approx(shapely_area, rel=1e-5, abs=1e-5)


def test_calculate_complex_score_uses_native_hull():
    base = Geometry.from_shapely(box(0, 0, 1, 1))
    placed = Geometry.from_shapely(box(1, 0, 2, 1))
    score = calculate_complex_score(
        base, placed, base.convex_hull_area(), Point(0.5, 0.5), Point(0, 0),
        w_dist=0.0, w_dir=0.0, w_hull=1.0,
    )
    assert score >= 0.0


def test_quality_nonnegative_and_shelf_beats_center():
    sheet = box(0, 0, 10, 10)
    ring = outline_ring_geom(sheet)
    assert ring is not None
    packed = [Geometry.from_shapely(box(0.1, 0.1, 1.1, 1.1))]
    cfg = PlacementRankConfig()
    cfg.min_dist = 0.1
    cfg.kiss_tol = outline_kiss_tolerance(0.1)
    cfg.edge_free_band_mult = 3.5
    cfg.tight_scale = 3.5 * 0.1
    cfg.edge_free_weight = 6.0
    cfg.clearance_weight = 0.25
    cfg.tightness_weight = 0.15
    cfg.part_area = 1.0
    cfg.sheet_area = 100.0
    cfg.mode = PlacementRankMode.contact_hybrid

    shelf = Geometry.from_shapely(box(0.1, 1.2, 1.1, 2.2))
    center = Geometry.from_shapely(box(4.5, 4.5, 5.5, 5.5))
    results = batch_score_placed_contact_hybrid(
        [shelf, center], ring, packed, None, cfg,
    )
    assert all(r.quality >= 0.0 for r in results)
    assert results[0].quality > results[1].quality
    assert results[0].edge_free >= results[1].edge_free


def test_empty_pack_quality_is_board_kiss_only():
    sheet = box(0, 0, 10, 10)
    ring = outline_ring_geom(sheet)
    cfg = PlacementRankConfig()
    cfg.min_dist = 0.1
    cfg.kiss_tol = outline_kiss_tolerance(0.1)
    cfg.edge_free_band_mult = 3.5
    cfg.tight_scale = 0.35
    cfg.part_area = 1.0
    cfg.sheet_area = 100.0
    flush = Geometry.from_shapely(box(0.1, 4, 1.1, 5))
    far = Geometry.from_shapely(box(4, 4, 5, 5))
    results = batch_score_placed_contact_hybrid([flush, far], ring, [], None, cfg)
    assert results[0].edge_free == 0.0
    assert results[1].edge_free == 0.0
    assert results[0].quality >= results[1].quality
    assert all(r.quality >= 0.0 for r in results)


def test_batch_rank_valid_filter_and_propose_map():
    sheet = box(0, 0, 20, 20)
    part_poly = box(0, 0, 1, 1)
    geom = ProposeGeometry(sheet, Polygon(), part_poly, min_dist=0.1)
    cfg = ProposeConfig(edge_free_weight=6.0, ranking_mode="contact_hybrid")
    cands = [(0.2, 5.0, 0.0), (10.0, 10.0, 0.0)]
    rank_map = _batch_rank_results(
        cands, geom, cfg, 0.1, mode="contact_hybrid",
    )
    assert len(rank_map) == 2
    for c in cands:
        res = rank_map[_coord_key(c)]
        assert hasattr(res, "valid")
        assert res.quality >= 0.0


def test_nest_by_scores_prefers_high_quality():
    g = PoseGraph()
    # Overlapping nodes — only one can be selected; higher score wins.
    g.append_elem(0, Vec2(x=0.0, y=0.0), Circle.from_center_radius(0.0, 0.0, 1.0))
    g.append_elem(0, Vec2(x=0.5, y=0.0), Circle.from_center_radius(0.5, 0.0, 1.0))
    g.add_collision_pair(0, 1)
    scores = [10.0, 1.0]
    selected = nest_by_scores(g, scores)
    assert selected == [0]

    scores_rev = [1.0, 10.0]
    selected_rev = nest_by_scores(g, scores_rev)
    assert selected_rev == [1]


def test_nest_by_scores_length_mismatch_raises():
    g = PoseGraph()
    g.append_elem(0, Vec2(x=0.0, y=0.0), Circle.from_center_radius(0.0, 0.0, 0.1))
    with pytest.raises(Exception):
        nest_by_scores(g, [1.0, 2.0])


def test_dfs_grows_with_nonnegative_quality():
    g = PoseGraph()
    g.append_elem(0, Vec2(x=0.0, y=0.0), Circle.from_center_radius(0.0, 0.0, 0.1))
    g.append_elem(0, Vec2(x=2.0, y=0.0), Circle.from_center_radius(2.0, 0.0, 0.1))
    g.append_elem(0, Vec2(x=4.0, y=0.0), Circle.from_center_radius(4.0, 0.0, 0.1))
    scores = [1.0, 2.0, 3.0]
    opts = RefineSelectionOptions()
    opts.max_passes = 4
    opts.seed = 1
    refined = refine_selection(g, [0], scores, opts)
    assert len(refined) >= 1
    assert len(refined) >= len([0])


def test_signed_scores_can_block_dfs_growth():
    """Control: negative scores demonstrate why selection must use quality ≥ 0."""
    g = PoseGraph()
    g.append_elem(0, Vec2(x=0.0, y=0.0), Circle.from_center_radius(0.0, 0.0, 0.1))
    g.append_elem(0, Vec2(x=2.0, y=0.0), Circle.from_center_radius(2.0, 0.0, 0.1))
    g.append_elem(0, Vec2(x=4.0, y=0.0), Circle.from_center_radius(4.0, 0.0, 0.1))
    opts = RefineSelectionOptions()
    opts.max_passes = 8
    opts.seed = 1
    # All negative: adding a node lowers Σ scores, so refine should not grow.
    signed = [-1.0, -2.0, -3.0]
    refined = refine_selection(g, [0], signed, opts)
    assert len(refined) <= 1


def test_boost_selection_geom_quality_raises_shelf():
    sheet = box(0, 0, 10, 10)
    ring = outline_ring_geom(sheet)
    shelf = Geometry.from_shapely(box(0.1, 1.2, 1.1, 2.2))
    center = Geometry.from_shapely(box(4.5, 4.5, 5.5, 5.5))
    packed = [Geometry.from_shapely(box(0.1, 0.1, 1.1, 1.1))]
    scores = [0.0, 0.0]
    cfg = ProposeConfig(selection_geom_weight=24.0, edge_free_weight=6.0)
    stats = {}
    n = boost_selection_geom_quality(
        [shelf, center],
        scores,
        board_ring=ring,
        packed=packed,
        min_dist=0.1,
        part_areas=[1.0],
        group_id=[0, 0],
        sheet_area=100.0,
        propose_cfg=cfg,
        stats_out=stats,
    )
    assert n == 2
    assert scores[0] > scores[1]
    assert stats["geom_ms"] >= 0.0
    assert "geom_share" in stats
    assert all(s >= 0.0 for s in scores)
