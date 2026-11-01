"""H1: hole emit passes real kept gids/transforms into propose_coords_with_strategy."""

from unittest.mock import patch

import numpy as np
from shapely.geometry import box

from nest_graph.config import ProposeConfig
from nest_graph.propose.block_replace import _emit_hole_candidates


def test_emit_hole_candidates_forwards_real_packed_poses():
    sheet = box(0, 0, 20, 20)
    kept = [box(1, 1, 3, 3), box(5, 1, 7, 3)]
    gids = [2, 4]
    trs = [(1.5, 2.0, 0.25), (6.0, 2.0, -0.1)]
    part_by_group = {0: box(0, 0, 1.5, 1.5)}
    captured: dict = {}

    def _fake_propose(*_a, **kw):
        captured["packed_group_ids"] = list(kw.get("packed_group_ids") or ())
        captured["packed_transforms"] = list(kw.get("packed_transforms") or ())
        return []

    with patch(
        "nest_graph.propose.pipeline.propose_coords_with_strategy",
        _fake_propose,
    ):
        _emit_hole_candidates(
            sheet=sheet,
            kept_geoms=kept,
            part_by_group=part_by_group,
            min_dist=0.0,
            propose_cfg=ProposeConfig(),
            pole=sheet.centroid,
            void_poly=box(8, 8, 12, 12),
            void_geoms=[],
            free_space=None,
            cluster_patterns=(),
            packed_group_ids=gids,
            packed_transforms=trs,
        )
    assert captured["packed_group_ids"] == [2, 4]
    assert captured["packed_transforms"][0][0] == 1.5
    assert captured["packed_transforms"][1][2] == -0.1
    assert not all(g == 0 for g in captured["packed_group_ids"])
    assert not all(
        abs(t[0]) < 1e-12 and abs(t[1]) < 1e-12 and abs(t[2]) < 1e-12
        for t in captured["packed_transforms"]
    )


def test_emit_hole_candidates_defaults_zeros_when_lengths_mismatch():
    sheet = box(0, 0, 10, 10)
    kept = [box(1, 1, 2, 2)]
    captured: dict = {}

    def _fake_propose(*_a, **kw):
        captured["packed_group_ids"] = list(kw.get("packed_group_ids") or ())
        captured["packed_transforms"] = [
            tuple(np.asarray(t, dtype=np.float64).reshape(3))
            for t in (kw.get("packed_transforms") or ())
        ]
        return []

    with patch(
        "nest_graph.propose.pipeline.propose_coords_with_strategy",
        _fake_propose,
    ):
        _emit_hole_candidates(
            sheet=sheet,
            kept_geoms=kept,
            part_by_group={0: box(0, 0, 1, 1)},
            min_dist=0.0,
            propose_cfg=ProposeConfig(),
            pole=None,
            void_poly=None,
            void_geoms=[],
            free_space=None,
            cluster_patterns=(),
            packed_group_ids=[9, 9],  # length mismatch → zeros
            packed_transforms=[(1.0, 1.0, 0.0)],
        )
    assert captured["packed_group_ids"] == [0]
    assert captured["packed_transforms"] == [(0.0, 0.0, 0.0)]
