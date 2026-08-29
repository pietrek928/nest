"""A2: archive ref_transform mix pins under cluster_copy floor gap."""

import numpy as np
from shapely import box

from nest_graph.config import BuildGraphConfig
from nest_graph.geometry import Geometry
from nest_graph.propose.placements_pattern import ClusterPattern
from nest_graph.propose.pattern_archive import note_motif_ref_anchors
from nest_graph.propose.transform_batch import build_transform_batch


class _NestStub:
    selected_indices = [0]
    seed_count = 1
    void_geoms = []
    group_id = [0]
    transform = [(5.0, 6.0, 0.0)]
    polys = [box(0, 0, 1, 1)]

    @property
    def native_geoms(self):
        return [Geometry.from_shapely(box(0, 0, 1, 1))]


def test_archive_mix_pins_ref_transform_on_plateau():
    note_motif_ref_anchors.__globals__["_motif_ref_anchors"].clear()
    pat = ClusterPattern(
        members=((0, (0.0, 0.0, 0.0)), (1, (2.0, 0.0, 0.0))),
        part_count=2,
        ref_transform=(5.0, 6.0, 0.0),
        motif_id=0,
    )
    tri = box(0, 0, 2, 1)
    parts = [(tri, 0), (tri, 1)]
    part_bases = {0: Geometry.from_shapely(tri), 1: Geometry.from_shapely(tri)}
    sheet = box(0, 0, 30, 30)
    cfg = BuildGraphConfig()
    stats: dict = {
        "on_plateau": True,
        "free_kind": "large_void",
        "mcts_zone": "",
    }
    sel0 = np.zeros((0, 3))
    sel1 = np.zeros((0, 3))
    out = build_transform_batch(
        cfg,
        (sel0, sel1),
        (sel0, sel1),
        np.random.default_rng(0),
        board=sheet,
        parts=parts,
        nest_state=_NestStub(),
        part_bases=part_bases,
        propose_stats_out=stats,
        archived_patterns=[pat],
    )
    assert int(stats.get("archive_mix_floor_hits", 0) or 0) > 0
    keys = {tuple(round(float(x), 4) for x in row) for row in out[0]}
    assert (5.0, 6.0, 0.0) in keys
