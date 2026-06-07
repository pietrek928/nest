"""Cast seed selection prefers kiss-quality poses over pool append order."""

from shapely.affinity import translate
from shapely.geometry import Point, Polygon

from nest_graph.config import ProposeConfig
from nest_graph.propose import ProposeGeometry, collect_propose_candidates
from nest_graph.propose.ranking import select_guidance_cast_seeds


def test_select_guidance_cast_seeds_prefers_group_fit():
    board = Polygon([(0, 0), (10, 0), (10, 10), (0, 10)])
    base = translate(Polygon([(0, 0), (2, 0), (2, 1), (0, 1)]), 2, 2)
    part = Polygon([(0, 0), (0.5, 0), (0.5, 0.5), (0, 0.5)])
    min_dist = 0.05
    cfg = ProposeConfig(
        candidate_pool=48,
        use_neighbor_slide=False,
        use_axis_push=False,
        use_bottom_left=False,
        use_nfp_vertices=False,
        use_ribbon_seeds=False,
        use_voronoi=False,
        use_guidance_propositions=False,
        use_group_edge_seeds=True,
    )
    geom = ProposeGeometry(board, base, part, min_dist, propose_cfg=cfg)
    push = base.centroid
    loose = [(8.0, 8.0, 0.0)] * 20
    tight = collect_propose_candidates(
        base,
        part,
        board,
        cfg,
        min_dist=min_dist,
        pt_push=push,
        propose_geom=geom,
        focal_shape=base,
        enabled_proposers=frozenset({"group_fit"}),
    )
    assert tight
    pool = loose + tight
    picked = select_guidance_cast_seeds(
        pool,
        8,
        part,
        geom,
        push,
        min_dist,
        base,
    )
    assert any(
        abs(p[0] - t[0]) < 0.5 and abs(p[1] - t[1]) < 0.5
        for p in picked
        for t in tight
    )
