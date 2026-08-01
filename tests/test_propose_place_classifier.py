from shapely.geometry import Point, Polygon
from shapely.ops import unary_union

from nest_graph.board import board_sheet_from_outline
from nest_graph.config import ProposeConfig, PLACE_ZONES
from nest_graph.propose.context import (
    classify_propose_zone,
    classify_propose_zone_info,
    cluster_packed_indices,
    free_space_targets,
    placement_free_region,
)
from nest_graph.propose.feedback import ProposeFeedbackState
from nest_graph.propose.pipeline import proposed_transforms_for_groups
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
    board = Polygon([(0, 0), (16, 0), (16, 16), (0, 16)])
    rect = Polygon([(0, 0), (1.2, 0), (1.2, 1.2), (0, 1.2)])
    t1 = transform_poly(rect, (5.0, 5.0, 0.0))
    t2 = transform_poly(rect, (11.0, 11.0, 0.0))
    placed = [t1, t2]
    obstacle = unary_union(placed)
    cfg = ProposeConfig()
    zone = classify_propose_zone(
        board,
        obstacle,
        rect,
        min_dist=0.05,
        propose_cfg=cfg,
        selected_polys=placed,
    )
    assert zone == "inter_cluster"


def test_classify_void_seek_real_hole():
    outline = Polygon([(0, 0), (12, 0), (12, 12), (0, 12)])
    hole = ((5, 5), (7, 5), (7, 7), (5, 7), (5, 5))
    sheet = board_sheet_from_outline(outline, user_holes=(hole,))
    rect = Polygon([(0, 0), (1, 0), (1, 1), (0, 1)])
    # Pack around the hole mouth so free concentrates near the void.
    placed = [
        transform_poly(rect, (3.5, 5.5, 0.0)),
        transform_poly(rect, (8.5, 5.5, 0.0)),
        transform_poly(rect, (5.5, 3.5, 0.0)),
        transform_poly(rect, (5.5, 8.5, 0.0)),
        transform_poly(rect, (3.5, 3.5, 0.0)),
        transform_poly(rect, (8.5, 3.5, 0.0)),
        transform_poly(rect, (3.5, 8.5, 0.0)),
        transform_poly(rect, (8.5, 8.5, 0.0)),
    ]
    obstacle = unary_union(placed)
    cfg = ProposeConfig()
    zone = classify_propose_zone(
        outline,
        obstacle,
        rect,
        min_dist=0.05,
        propose_cfg=cfg,
        selected_polys=placed,
        user_holes=(hole,),
        sheet=sheet,
    )
    assert zone == "void_seek"


def test_classify_hole_free_not_void_seek(nest_board, rect_poly):
    placed = [
        transform_poly(rect_poly, (0.08, 0.08, 0.0)),
        transform_poly(rect_poly, (0.95, 0.06, 0.2)),
        transform_poly(rect_poly, (0.06, 0.85, 0.4)),
    ]
    obstacle = unary_union(placed)
    cfg = ProposeConfig()
    zone = classify_propose_zone(
        nest_board,
        obstacle,
        rect_poly,
        min_dist=0.02,
        propose_cfg=cfg,
        selected_polys=placed,
    )
    assert zone != "void_seek"


def test_classify_hijack_two_corner_seeds_not_inter_cluster():
    board = Polygon([(0, 0), (20, 0), (20, 20), (0, 20)])
    rect = Polygon([(0, 0), (1, 0), (1, 1), (0, 1)])
    placed = [
        transform_poly(rect, (1.0, 1.0, 0.0)),
        transform_poly(rect, (19.0, 19.0, 0.0)),
    ]
    obstacle = unary_union(placed)
    zone = classify_propose_zone(
        board,
        obstacle,
        rect,
        min_dist=0.05,
        propose_cfg=ProposeConfig(),
        selected_polys=placed,
    )
    assert zone != "inter_cluster"
    assert zone in ("border_gap", "cluster_edge", "interior_pocket")


def test_classify_hole_bypass_empty_and_corner():
    outline = Polygon([(0, 0), (20, 0), (20, 20), (0, 20)])
    hole = ((9, 9), (11, 9), (11, 11), (9, 11), (9, 9))
    sheet = board_sheet_from_outline(outline, user_holes=(hole,))
    rect = Polygon([(0, 0), (1, 0), (1, 1), (0, 1)])
    cfg = ProposeConfig()
    assert (
        classify_propose_zone(
            outline,
            Polygon(),
            rect,
            min_dist=0.05,
            propose_cfg=cfg,
            selected_polys=[],
            user_holes=(hole,),
            sheet=sheet,
        )
        == "empty_border"
    )
    placed = [transform_poly(rect, (1.0, 1.0, 0.0))]
    zone = classify_propose_zone(
        outline,
        unary_union(placed),
        rect,
        min_dist=0.05,
        propose_cfg=cfg,
        selected_polys=placed,
        user_holes=(hole,),
        sheet=sheet,
    )
    assert zone != "void_seek"


def test_classify_rim_primary_target_not_origin():
    board = Polygon([(0, 0), (16, 0), (16, 16), (0, 16)])
    rect = Polygon([(0, 0), (1.2, 0), (1.2, 1.2), (0, 1.2)])
    # Dense rim pitch so merge yields one cluster (matches tight_border fixtures).
    placed = []
    xs = [1.0 + 1.5 * i for i in range(10)]
    for x in xs:
        placed.append(transform_poly(rect, (x, 1.0, 0.0)))
        placed.append(transform_poly(rect, (x, 15.0, 0.0)))
    ys = [2.5 + 1.5 * i for i in range(8)]
    for y in ys:
        placed.append(transform_poly(rect, (1.0, y, 0.0)))
        placed.append(transform_poly(rect, (15.0, y, 0.0)))
    obstacle = unary_union(placed)
    info = classify_propose_zone_info(
        board,
        obstacle,
        rect,
        min_dist=0.05,
        propose_cfg=ProposeConfig(),
        selected_polys=placed,
    )
    assert info.zone == "border_gap"
    assert info.primary_target is not None
    assert info.primary_target.distance(Point(0.0, 0.0)) > 1.0
    assert info.n_clusters == 1


def test_cluster_merge_rim_vs_bridge():
    board = Polygon([(0, 0), (16, 0), (16, 16), (0, 16)])
    rect = Polygon([(0, 0), (1.2, 0), (1.2, 1.2), (0, 1.2)])
    rim = []
    for x in (1.0, 3.0, 5.0, 7.0, 9.0, 11.0, 13.0, 15.0):
        rim.append(transform_poly(rect, (x, 1.0, 0.0)))
    assert len(cluster_packed_indices(rim, 0.05, sheet=board)) == 1
    bridge = [
        transform_poly(rect, (4.5, 4.5, 0.0)),
        transform_poly(rect, (4.5, 6.0, 0.0)),
        transform_poly(rect, (6.0, 4.5, 0.0)),
        transform_poly(rect, (6.0, 6.0, 0.0)),
        transform_poly(rect, (11.5, 11.5, 0.0)),
        transform_poly(rect, (11.5, 10.0, 0.0)),
        transform_poly(rect, (10.0, 11.5, 0.0)),
        transform_poly(rect, (10.0, 10.0, 0.0)),
    ]
    assert len(cluster_packed_indices(bridge, 0.05, sheet=board)) >= 2


def test_zone_proposers_include_cast_refine():
    for zone in ("border_gap", "interior_pocket", "cluster_edge", "void_seek"):
        proposers = ProposeConfig.proposers_for_place(zone)
        assert proposers is not None
        assert "guidance_cast_refine" in proposers


def test_interior_pocket_includes_group_fit():
    proposers = ProposeConfig.proposers_for_place("interior_pocket")
    assert proposers is not None
    assert "group_fit" in proposers


def test_for_place_preserves_base_false():
    base = ProposeConfig(use_voronoi=False, use_cluster_copy=False)
    cfg = ProposeConfig.for_place("inter_cluster", base=base)
    assert cfg.use_voronoi is False
    assert cfg.use_cluster_copy is False


def test_for_place_border_gap_caps_samples():
    cfg = ProposeConfig.for_place("border_gap")
    assert cfg.board_edge_samples_per_edge <= 12
    assert cfg.group_edge_samples_per_edge <= 8
    assert cfg.board_edge_guidance_refine is False
    assert cfg.contact_clearance_hybrid_weight <= 0.1


def test_classify_interior_pack_not_border_gap():
    """Center-only seeds must not route through expensive border_gap proposers."""
    board = Polygon([(0, 0), (12, 0), (12, 12), (0, 12)])
    rect = Polygon([(0, 0), (1, 0), (1, 1), (0, 1)])
    placed = [transform_poly(rect, (5.5, 5.5, 0.0))]
    obstacle = unary_union(placed)
    cfg = ProposeConfig(place_border_coverage_threshold=0.35)
    zone = classify_propose_zone(
        board,
        obstacle,
        transform_poly(rect, (7.0, 5.5, 0.0)),
        min_dist=0.05,
        propose_cfg=cfg,
        selected_polys=placed,
    )
    assert zone != "border_gap"
    assert zone in ("cluster_edge", "interior_pocket", "inter_cluster")


def test_zone_proposers_include_cluster_copy():
    for zone in ("interior_pocket", "cluster_edge", "inter_cluster"):
        proposers = ProposeConfig.proposers_for_place(zone)
        assert proposers is not None
        assert "cluster_copy" in proposers


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


def test_border_gap_scope_upgrades_for_rim():
    use_full, nearest_k = ProposeConfig.obstacle_scope_for_place(
        "border_gap",
        n_clusters=1,
        free_ratio=0.74,
        outline_coverage=0.8,
        border_coverage_threshold=0.35,
    )
    assert use_full is True
    use_full_k, nearest_k = ProposeConfig.obstacle_scope_for_place(
        "border_gap",
        n_clusters=1,
        free_ratio=0.9,
        outline_coverage=0.1,
        border_coverage_threshold=0.35,
    )
    assert use_full_k is False
    assert nearest_k == 4


def test_propose_feedback_scales_on_low_yield():
    fb = ProposeFeedbackState()
    fb.record_iteration(
        proposer_counts={"board_edge": 80, "voronoi": 10},
        graph_yield=0.3,
    )
    assert "board_edge" in fb.proposer_pool_scales
    assert fb.proposer_pool_scales["board_edge"] < 1.0


def test_place_routed_propose_runs(nest_board, rect_poly):
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


def test_free_space_targets_board_frame():
    board = Polygon([(0, 0), (10, 0), (10, 10), (0, 10)])
    free = placement_free_region(board, Polygon([(1, 1), (2, 1), (2, 2), (1, 2)]), 0.1)
    targets = free_space_targets(free, 0.1)
    assert targets
    assert targets[0].distance(Point(0.0, 0.0)) > 1.0


def test_simplify_obstacle_union_preserves_rim_interior():
    from nest_graph.propose.context import simplify_obstacle_union

    board = Polygon([(0, 0), (16, 0), (16, 16), (0, 16)])
    rect = Polygon([(0, 0), (1.2, 0), (1.2, 1.2), (0, 1.2)])
    rim = []
    xs = [1.0 + 1.5 * i for i in range(10)]
    for x in xs:
        rim.append(transform_poly(rect, (x, 1.0, 0.0)))
        rim.append(transform_poly(rect, (x, 15.0, 0.0)))
    for y in [2.5 + 1.5 * i for i in range(8)]:
        rim.append(transform_poly(rect, (1.0, y, 0.0)))
        rim.append(transform_poly(rect, (15.0, y, 0.0)))
    merged = simplify_obstacle_union(rim, 0.05)
    assert merged.area < board.area * 0.5
    assert not merged.contains(board.centroid)
