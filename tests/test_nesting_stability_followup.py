"""Stability follow-up: void_seek budgets, stratified subsample, extend_counted."""

import numpy as np
from shapely.geometry import box

from nest_graph.build_graph import (
    NestState,
    _build_transform_batch,
    archive_void_elite_transforms,
    void_elite_count,
    void_elite_tuple_from_archive,
)
from nest_graph.config import (
    ProposeConfig,
    floor_void_seek_budgets,
    subsample_transforms_stratified,
)
from nest_graph.propose.pipeline import _extend_counted
from nest_graph.proposer_names import PlaceZone, ProposerName, ZONE_PROPOSERS


def test_void_seek_budget_floors_survive_lean():
    lean = ProposeConfig().with_complexity_lean(n_holes=2, max_part_vertices=8)
    assert lean.candidate_pool <= 24
    cfg = ProposeConfig.for_place("void_seek", base=lean)
    assert cfg.candidate_pool >= 1024
    assert cfg.max_proposals >= 512
    scaled = floor_void_seek_budgets(
        ProposeConfig(candidate_pool=8, max_proposals=4),
    )
    assert scaled.candidate_pool == 1024
    assert scaled.max_proposals == 512


def test_extend_counted_dedupes_claimed_keys():
    cands: list = []
    counts: dict = {}
    keys: dict = {}
    claimed: set = set()
    _extend_counted(
        cands,
        counts,
        "ribbon_free",
        [(0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (1.0, 0.0, 0.0)],
        proposer_keys=keys,
        claimed_keys=claimed,
    )
    assert counts["ribbon_free"] == 2
    assert len(cands) == 2
    assert len(keys["ribbon_free"]) == 2


def test_stratified_subsample_preserves_niches():
    rng = np.random.default_rng(0)
    sel = np.array([[10.0, 0.0, 0.0], [11.0, 0.0, 0.0]], dtype=np.float64)
    props = np.array([[20.0 + i, 0.0, 0.0] for i in range(5)], dtype=np.float64)
    void_e = np.array([[30.0 + i, 0.0, 0.0] for i in range(5)], dtype=np.float64)
    hist = np.array([[40.0 + i, 0.0, 0.0] for i in range(5)], dtype=np.float64)
    expand = np.array([[100.0 + i, 0.0, 0.0] for i in range(200)], dtype=np.float64)
    out = subsample_transforms_stratified(
        selection=sel,
        proposals=props,
        void_elite=void_e,
        history=hist,
        expand_rest=expand,
        max_n=40,
        rng=rng,
        n_props=5,
        n_void_elite=5,
        n_hist=5,
    )
    assert out.shape[0] == 40
    xs = set(float(r[0]) for r in out)
    assert 10.0 in xs and 11.0 in xs
    assert any(20.0 <= x < 30.0 for x in xs)
    assert any(30.0 <= x < 40.0 for x in xs)
    assert any(40.0 <= x < 50.0 for x in xs)


def test_stratified_proposals_keep_prefix_order():
    rng = np.random.default_rng(0)
    sel = np.zeros((0, 3), dtype=np.float64)
    props = np.array([[float(i), 0.0, 0.0] for i in range(8)], dtype=np.float64)
    empty = np.zeros((0, 3), dtype=np.float64)
    out = subsample_transforms_stratified(
        selection=sel,
        proposals=props,
        void_elite=empty,
        history=empty,
        expand_rest=empty,
        max_n=4,
        rng=rng,
        n_props=4,
        n_void_elite=0,
        n_hist=0,
    )
    assert [float(r[0]) for r in out] == [0.0, 1.0, 2.0, 3.0]


def test_void_seek_zone_includes_expand_proposers():
    zone = ZONE_PROPOSERS[PlaceZone.VOID_SEEK]
    assert ProposerName.SELECTION_EXPAND in zone
    assert ProposerName.HISTORY_EXPAND in zone
    assert ProposerName.CLUSTER_COPY in zone


def test_archive_void_elite_keeps_refine_losers():
    free = box(0.0, 0.0, 10.0, 10.0)
    polys = [box(1, 1, 2, 2), box(3, 3, 4, 4), box(5, 5, 6, 6)]
    transforms = [
        np.array([1.5, 1.5, 0.0]),
        np.array([3.5, 3.5, 0.1]),
        np.array([5.5, 5.5, 0.2]),
    ]
    archive = archive_void_elite_transforms(
        selected_nest=[0, 1, 2],
        selected_refine=[0],
        polys=polys,
        transforms=transforms,
        group_ids=[0, 1, 0],
        free_poly=free,
        scores=[1.0, 9.0, 4.0],
        max_keep=2,
        enabled=True,
    )
    assert void_elite_count(archive) == 2
    # Highest-scoring losers first: idx1 (9.0) then idx2 (4.0)
    assert len(archive[1]) == 1
    assert len(archive[0]) == 1
    assert float(archive[1][0][0]) == 3.5
    tup = void_elite_tuple_from_archive(archive, 2)
    assert tup[0].shape == (1, 3) and tup[1].shape == (1, 3)
    assert archive_void_elite_transforms(
        selected_nest=[0, 1],
        selected_refine=[0, 1],
        polys=polys,
        transforms=transforms,
        group_ids=[0, 1],
        free_poly=free,
        scores=[1.0, 1.0],
        enabled=False,
    ) == {}


def test_build_transform_batch_seeds_void_elite(build_graph_config):
    cfg = build_graph_config
    cfg.sampling.max_transforms_per_group = 64
    cfg.sampling.random_per_iter = 8
    cfg.sampling.random_per_iter_when_proposed = 8
    cfg.sampling.selection_expand_n = 2
    cfg.sampling.history_expand_n = 2
    cfg.propose.stratified_void_elite_quota = 8
    cfg.propose.stratified_history_quota = 8
    rng = np.random.default_rng(0)
    packed = box(0.0, 0.0, 0.2, 0.2)
    nest_state = NestState(
        polys=[packed],
        group_id=[0],
        transform=[np.array([0.1, 0.1, 0.0])],
        selected_indices=[0],
        seed_count=0,
    )
    elite = np.array([[90.0 + i, 1.0, 0.0] for i in range(6)], dtype=np.float64)
    hist = np.array([[40.0 + i, 2.0, 0.0] for i in range(4)], dtype=np.float64)
    sel = (np.array([[0.1, 0.1, 0.0]], dtype=np.float64), np.zeros((0, 3)))
    out = _build_transform_batch(
        cfg,
        sel,
        (hist, np.zeros((0, 3))),
        rng,
        nest_state=nest_state,
        void_elite_t=(elite, np.zeros((0, 3), dtype=np.float64)),
        keep_history_on_sterile=True,
    )
    xs = {float(r[0]) for r in out[0]}
    assert any(90.0 <= x < 96.0 for x in xs)
    # Under sterile+keep_hist, history niche should also survive.
    assert any(40.0 <= x < 44.0 for x in xs)
