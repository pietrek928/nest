from shapely.geometry import Point, Polygon
from shapely.ops import unary_union

from nest_graph.config import ProposeConfig, PLACE_ZONES
from nest_graph.propose.context import classify_propose_zone
from nest_graph.propose.feedback import ProposeFeedbackState
from nest_graph.utils import transform_poly


def _triangle_board() -> Polygon:
    return Polygon([(0, 0), (1.2, 0), (0, 1.1)])


def _rect() -> Polygon:
    return Polygon([(0, 0), (0.1, 0), (0.1, 0.1), (0, 0.1)])


def test_classify_empty_border():
    board = _triangle_board()
    cfg = ProposeConfig()
    zone = classify_propose_zone(
        board,
        Polygon(),
        _rect(),
        min_dist=0.02,
        propose_cfg=cfg,
        selected_polys=[],
    )
    assert zone == "empty_border"


def test_classify_border_gap_partial_pack():
    board = _triangle_board()
    rect = _rect()
    placed = [transform_poly(rect, (0.08, 0.08, 0.0))]
    obstacle = unary_union(placed)
    cfg = ProposeConfig(place_border_coverage_threshold=0.35)
    zone = classify_propose_zone(
        board,
        obstacle,
        transform_poly(rect, (0.4, 0.35, 0.0)),
        min_dist=0.02,
        propose_cfg=cfg,
        selected_polys=placed,
    )
    assert zone == "border_gap"


def test_classify_two_clusters_inter_cluster():
    board = _triangle_board()
    rect = _rect()
    t1 = transform_poly(rect, (0.08, 0.08, 0.0))
    t2 = transform_poly(rect, (0.55, 0.55, 0.2))
    placed = [t1, t2]
    obstacle = unary_union(placed)
    cfg = ProposeConfig(
        place_border_coverage_threshold=0.01,
        place_free_area_interior_threshold=0.9,
    )
    zone = classify_propose_zone(
        board,
        obstacle,
        transform_poly(rect, (0.4, 0.35, 0.0)),
        min_dist=0.02,
        propose_cfg=cfg,
        selected_polys=placed,
    )
    assert zone == "inter_cluster"


def test_classify_void_seek(nest_board, rect_poly):
    from shapely.ops import unary_union

    placed = [
        transform_poly(rect_poly, (0.08, 0.08, 0.0)),
        transform_poly(rect_poly, (0.95, 0.06, 0.2)),
        transform_poly(rect_poly, (0.06, 0.85, 0.4)),
    ]
    obstacle = unary_union(placed)
    part = transform_poly(rect_poly, (0.02, 0.02, 0.0))
    cfg = ProposeConfig(
        place_border_coverage_threshold=0.02,
        place_free_area_interior_threshold=0.99,
    )
    zone = classify_propose_zone(
        nest_board,
        obstacle,
        part,
        min_dist=0.02,
        propose_cfg=cfg,
        selected_polys=placed,
    )
    assert zone == "void_seek"


def test_zone_proposers_include_cast_refine():
    for zone in ("interior_pocket", "cluster_edge", "void_seek"):
        proposers = ProposeConfig.proposers_for_place(zone)
        assert proposers is not None
        assert "guidance_cast_refine" in proposers


def test_for_place_profiles_exist():
    base = ProposeConfig()
    for zone in PLACE_ZONES:
        cfg = ProposeConfig.for_place(zone, base=base)
        assert isinstance(cfg, ProposeConfig)
        proposers = ProposeConfig.proposers_for_place(zone)
        if zone != "empty_border":
            assert proposers is not None
            assert len(proposers) >= 2


def test_interior_pocket_uses_full_obstacle():
    use_full, nearest_k = ProposeConfig.obstacle_scope_for_place("interior_pocket")
    assert use_full is True
    assert nearest_k == 0
    cfg = ProposeConfig.for_place("interior_pocket")
    assert cfg.ranking_mode == "contact_hybrid"
    assert cfg.use_full_packed_obstacle is True


def test_propose_feedback_scales_on_low_yield():
    fb = ProposeFeedbackState()
    fb.record_iteration(
        proposer_counts={"board_edge": 80, "voronoi": 10},
        graph_yield=0.3,
    )
    assert "board_edge" in fb.proposer_pool_scales
    assert fb.proposer_pool_scales["board_edge"] < 1.0


def test_place_routed_propose_runs(nest_board, rect_poly):
    from nest_graph.propose.pipeline import proposed_transforms_for_groups

    cfg = ProposeConfig(place_profiles_enabled=True)
    placed = transform_poly(rect_poly, (0.35, 0.25, 0.0))
    counts: dict[str, int] = {}
    out = proposed_transforms_for_groups(
        nest_board,
        [(rect_poly, 0)],
        [placed],
        [0],
        cfg,
        min_dist=0.02,
        proposer_counts_out=counts,
    )
    assert 0 in out
    assert out[0].shape[0] > 0
    assert counts
