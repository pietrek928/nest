"""Dckpt: NestState(+DG memory) checkpoint round-trip + resume smoke."""

from pathlib import Path

import numpy as np
from shapely.geometry import box, Polygon

from nest_graph.build_graph import NestState
from nest_graph.graph import MotifBase, MotifRecord, Se2, MacroNicheArchive, BoardSnapshot
from nest_graph.pack.checkpoint import (
    load_nest_checkpoint,
    nest_state_from_checkpoint,
    rehydrate_motif_base,
    rehydrate_niche_archive,
    save_nest_checkpoint,
    tip_board_snapshot,
)
from nest_graph.propose.placement_common import selection_pairwise_independent


def test_checkpoint_round_trip(tmp_path: Path):
    polys = [box(0, 0, 1, 1), Polygon([(2, 0), (3, 0), (2.5, 1)])]
    nest = NestState(
        polys=polys,
        group_id=[0, 1],
        transform=[(0.0, 0.0, 0.0), (2.0, 0.0, 0.1)],
        selected_indices=[0, 1],
        seed_count=0,
    )
    mb = MotifBase()
    rec = MotifRecord()
    rec.gid_a = 0
    rec.gid_b = 1
    rec.relative = Se2(1.0, 0.5, 0.2)
    rec.compactness = 0.6
    rec.area_a = 1.0
    rec.area_b = 1.0
    mb.upsert(rec, 0.0, 4)
    arch = MacroNicheArchive()
    arch.append_positive((0, 0, 0), [(0, 1.5, 2.5, 0.0)], 1, 1.0, 4)
    tip = BoardSnapshot(
        packed_gids=(0, 1),
        packed_transforms=((0.0, 0.0, 0.0), (2.0, 0.0, 0.1)),
        remaining_gids=(),
        coverage=0.42,
        kiss_pairs=3,
        mean_compactness=0.55,
        free_kind="large_void",
    )
    path = tmp_path / "ckpt.npz"
    save_nest_checkpoint(
        path,
        iter_n=20,
        seed=0,
        coverage=0.42,
        parts=2,
        independent_ok=True,
        nest_state=nest,
        best_pack_cov=0.42,
        best_pack_sel=[0, 1],
        best_pack_sig=1.0,
        best_pack_tf=[(0.0, 0.0, 0.0), (2.0, 0.0, 0.1)],
        best_pack_seed_n=0,
        void_elite_by_group={0: [np.array([1.0, 2.0, 0.0])]},
        history=(np.array([[0.1, 0.2, 0.0]]), np.zeros((0, 3))),
        graph_valid_carry=(np.zeros((0, 3)), np.array([[3.0, 4.0, 0.1]])),
        motif_base=mb,
        niche_archive=arch,
        tip_snap=tip,
        active_rule_id=1,
        n_rule_sets=2,
    )
    loaded = load_nest_checkpoint(path)
    assert loaded.iter == 20
    assert loaded.seed == 0
    assert abs(loaded.coverage - 0.42) < 1e-9
    assert loaded.parts == 2
    assert loaded.independent_ok is True
    assert loaded.group_id == [0, 1]
    assert loaded.selected_indices == [0, 1]
    assert len(loaded.motif_inject) >= 1
    assert loaded.tip_snap["kiss_pairs"] == 3
    restored = nest_state_from_checkpoint(loaded, NestState)
    assert restored.group_id == nest.group_id
    assert restored.selected_indices == nest.selected_indices
    assert restored.seed_count == nest.seed_count
    assert len(restored.polys) == 2
    assert abs(float(restored.polys[0].area) - 1.0) < 1e-6
    assert selection_pairwise_independent(restored.polys, restored.selected_indices)
    mb2 = rehydrate_motif_base(loaded)
    assert int(mb2.size()) >= 1
    arch2 = rehydrate_niche_archive(loaded)
    assert int(arch2.size) >= 1
    tip2 = tip_board_snapshot(loaded)
    assert abs(float(tip2.coverage) - 0.42) < 1e-6
    assert int(tip2.kiss_pairs) == 3
