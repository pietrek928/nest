from nest_graph.config import ProposeConfig
from nest_graph.propose.context import should_use_border_focus
from nest_graph.propose.pipeline import propose_coords_with_strategy
from shapely.geometry import Point, Polygon


def test_local_compact_profile_fields():
    cfg = ProposeConfig.local_compact_profile()
    assert cfg.use_guidance_propositions is True
    assert cfg.guidance_use_tight_packing is True
    assert cfg.guidance_use_corner_alignment is True
    assert cfg.guidance_enable_grid is False
    assert cfg.cast_squeeze_top_k == 8
    assert cfg.cast_squeeze_passes == 1
    assert cfg.use_neighbor_slide is False
    assert cfg.use_guidance_walk is False


def test_cast_squeeze_passes_default():
    cfg = ProposeConfig()
    assert cfg.cast_squeeze_passes == 1


def test_guidance_walk_disabled_on_empty_border(nest_board, rect_poly):
    cfg = ProposeConfig.local_compact_profile(use_walk=True)
    coords = propose_coords_with_strategy(
        Polygon(),
        rect_poly,
        nest_board,
        cfg,
        min_dist=0.02,
        pt_push=Point(nest_board.centroid),
    )
    assert should_use_border_focus(Polygon(), cfg)
    assert isinstance(coords, list)
