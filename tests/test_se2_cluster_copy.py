"""SE(2) helpers and cluster-copy motif extraction."""

from shapely.geometry import Point, Polygon
from shapely.ops import unary_union

from nest_graph.config import ProposeConfig
from nest_graph.propose.geometry import ProposeGeometry
from nest_graph.propose.placements_pattern import (
    extract_cluster_patterns,
    propose_placements_cluster_copy,
)
from nest_graph.utils import (
    compose_transforms,
    invert_transform,
    relative_transform,
    transform_poly,
)


def _rect() -> Polygon:
    return Polygon([(0, 0), (0.4, 0), (0.4, 0.3), (0, 0.3)])


def test_invert_compose_identity():
    t = (1.25, -0.7, 0.4)
    idt = compose_transforms(t, invert_transform(t))
    assert abs(idt[0]) < 1e-9
    assert abs(idt[1]) < 1e-9
    assert abs(idt[2]) < 1e-9


def test_relative_transform_roundtrip():
    ref = (2.0, 1.0, 0.3)
    t = (3.5, 2.2, 0.9)
    rel = relative_transform(ref, t)
    back = compose_transforms(ref, rel)
    assert abs(back[0] - t[0]) < 1e-9
    assert abs(back[1] - t[1]) < 1e-9
    assert abs(back[2] - t[2]) < 1e-9


def test_transform_poly_matches_compose():
    poly = _rect()
    a = (1.0, 0.5, 0.2)
    b = (0.3, -0.1, 0.4)
    composed = compose_transforms(a, b)
    direct = transform_poly(transform_poly(poly, b), a)
    via = transform_poly(poly, composed)
    assert direct.equals_exact(via, 1e-6)


def test_extract_cluster_patterns_two_touching():
    rect = _rect()
    t0 = (1.0, 1.0, 0.0)
    t1 = (1.42, 1.0, 0.0)  # gap 0.02 < min_dist contact threshold
    placed = [transform_poly(rect, t0), transform_poly(rect, t1)]
    patterns = extract_cluster_patterns(
        placed,
        [0, 1],
        [t0, t1],
        min_dist=0.05,
        max_patterns=2,
        min_members=2,
    )
    assert len(patterns) >= 1
    pat = patterns[0]
    assert pat.part_count == 2
    gids = {gid for gid, _rel in pat.members}
    assert gids == {0, 1}


def test_propose_cluster_copy_emits_coords():
    sheet = Polygon([(0, 0), (8, 0), (8, 8), (0, 8)])
    rect = _rect()
    t0 = (1.0, 1.0, 0.0)
    t1 = (1.42, 1.0, 0.0)
    placed = [transform_poly(rect, t0), transform_poly(rect, t1)]
    obstacle = unary_union(placed)
    patterns = extract_cluster_patterns(
        placed, [0, 1], [t0, t1], min_dist=0.05, min_members=2,
    )
    assert patterns
    cfg = ProposeConfig(cluster_copy_anchor_seeds=4)
    geom = ProposeGeometry(sheet, obstacle, rect, 0.05, propose_cfg=cfg)
    coords = propose_placements_cluster_copy(
        patterns,
        0,
        rect,
        sheet,
        obstacle,
        min_dist=0.05,
        propose_geom=geom,
        pt_push=Point(4.0, 4.0),
        propose_cfg=cfg,
        top_n=8,
    )
    assert isinstance(coords, list)
