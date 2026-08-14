"""Attract join: batch_pack pairs, sniper fill, no overlap edges."""

from shapely.geometry import box

from nest_graph.geometry import Geometry
from nest_graph.propose.first_pass_border import join_attract_pairs, kiss_attract_weight
from nest_graph.propose.void_selection import transform_row_key


def _square(x: float, y: float, s: float = 1.0) -> Geometry:
    return Geometry.from_shapely(box(x, y, x + s, y + s))


def test_kiss_weight_peaks_at_min_dist():
    assert kiss_attract_weight(0.1, 0.1, 8.0, 2.0) == 8.0
    assert kiss_attract_weight(0.31, 0.1, 8.0, 2.0) == 0.0
    assert kiss_attract_weight(0.1, 0.1, 0.0, 2.0) == 0.0


def test_overlapping_solids_get_no_attract():
    g0 = _square(0.0, 0.0)
    g1 = _square(0.2, 0.0)
    stats = {
        "sniper_keys": {
            0: {transform_row_key((0.5, 0.5, 0.0))},
            1: {transform_row_key((0.7, 0.5, 0.0))},
        },
    }
    pairs = join_attract_pairs(
        stats,
        [0, 1],
        [(0.5, 0.5, 0.0), (0.7, 0.5, 0.0)],
        [g0, g1],
        min_dist=0.1,
        contact_weight=8.0,
        kiss_band_scale=2.0,
        max_degree=8,
    )
    assert pairs == []


def test_group_fit_and_neighbor_slide_keys_attract():
    g0 = _square(0.0, 0.0)
    g1 = _square(1.1, 0.0)
    k0 = transform_row_key((0.5, 0.5, 0.0))
    k1 = transform_row_key((1.6, 0.5, 0.0))
    stats = {
        "sniper_keys": {
            0: {k0},
            1: {k1},
        },
        "proposer_keys": {
            "group_fit": {k0},
            "neighbor_slide": {k1},
        },
    }
    pairs = join_attract_pairs(
        stats,
        [0, 1],
        [(0.5, 0.5, 0.0), (1.6, 0.5, 0.0)],
        [g0, g1],
        min_dist=0.1,
        contact_weight=8.0,
        kiss_band_scale=20.0,
        max_degree=8,
    )
    verts = {(i, j) for i, j, _w in pairs}
    assert (0, 1) in verts


def test_union_sniper_keys_includes_group_fit_not_board_edge():
    from nest_graph.propose.pipeline import _union_sniper_keys

    out: dict[int, set] = {}
    _union_sniper_keys(
        out,
        0,
        {
            "group_fit": {(1.0, 2.0, 0.0)},
            "neighbor_slide": {(3.0, 4.0, 0.0)},
            "board_edge": {(9.0, 9.0, 0.0)},
            "cluster_copy": {(5.0, 6.0, 0.0)},
        },
    )
    assert (1.0, 2.0, 0.0) in out[0]
    assert (3.0, 4.0, 0.0) in out[0]
    assert (5.0, 6.0, 0.0) in out[0]
    assert (9.0, 9.0, 0.0) not in out[0]


def test_sniper_fill_skips_hist_and_board_edge_keys():
    g0 = _square(0.0, 0.0)
    g1 = _square(1.1, 0.0)
    g_hist = _square(2.2, 0.0)
    stats = {
        "sniper_keys": {
            0: {transform_row_key((0.5, 0.5, 0.0))},
        },
        "proposer_keys": {
            "board_edge": {transform_row_key((2.7, 0.5, 0.0))},
            "history_expand": {transform_row_key((2.7, 0.5, 0.0))},
            "cluster_copy": {transform_row_key((0.5, 0.5, 0.0))},
        },
    }
    pairs = join_attract_pairs(
        stats,
        [0, 0, 0],
        [(0.5, 0.5, 0.0), (1.6, 0.5, 0.0), (2.7, 0.5, 0.0)],
        [g0, g1, g_hist],
        min_dist=0.1,
        contact_weight=8.0,
        kiss_band_scale=20.0,
        max_degree=8,
    )
    verts = {(i, j) for i, j, _w in pairs}
    assert (0, 2) not in verts
    assert (1, 2) not in verts


def test_batch_pack_pairs_join_surviving_endpoints():
    g0 = _square(0.0, 0.0)
    g1 = _square(1.1, 0.0)
    ka = transform_row_key((0.5, 0.5, 0.0))
    kb = transform_row_key((1.6, 0.5, 0.0))
    stats = {"batch_pack_pairs": [(ka, kb, 0, 1)]}
    pairs = join_attract_pairs(
        stats,
        [0, 1],
        [(0.5, 0.5, 0.0), (1.6, 0.5, 0.0)],
        [g0, g1],
        min_dist=0.1,
        contact_weight=8.0,
        kiss_band_scale=20.0,
        max_degree=8,
    )
    assert len(pairs) == 1
    i, j, w = pairs[0]
    assert {i, j} == {0, 1}
    assert w > 0.0


def test_zero_weight_skips_join():
    g0 = _square(0.0, 0.0)
    g1 = _square(1.1, 0.0)
    stats = {
        "batch_pack_pairs": [
            (transform_row_key((0.5, 0.5, 0.0)), transform_row_key((1.6, 0.5, 0.0)), 0, 1),
        ],
    }
    pairs = join_attract_pairs(
        stats,
        [0, 1],
        [(0.5, 0.5, 0.0), (1.6, 0.5, 0.0)],
        [g0, g1],
        min_dist=0.1,
        contact_weight=0.0,
    )
    assert pairs == []


def test_make_polygon_graph_joins_sniper_from_propose_stats():
    from shapely.geometry import Polygon

    from nest_graph.build_graph import make_polygon_graph

    sheet = Polygon([(0, 0), (10, 0), (10, 10), (0, 10)])
    part = box(0, 0, 1, 1)
    t0 = (1.0, 1.0, 0.0)
    t1 = (2.1, 1.0, 0.0)
    stats = {
        "sniper_keys": {
            0: {transform_row_key(t0), transform_row_key(t1)},
        },
    }
    graph, _polys, _gids, _trs = make_polygon_graph(
        sheet,
        [(part, [t0, t1])],
        min_dist=0.05,
        propose_stats=stats,
        attract_contact_weight=8.0,
        attract_kiss_band_scale=20.0,
        attract_max_degree=8,
    )
    n_edges = sum(len(row) for row in graph.attract) // 2
    assert n_edges >= 1
    assert stats["attract_edges"] >= 1
