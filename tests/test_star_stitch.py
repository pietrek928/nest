"""Q237 leader-star stitch: MotifBase pairs → k-member ClusterPattern."""

from nest_graph.graph import MotifBase, MotifRecord, Se2
from nest_graph.propose.pattern_archive import (
    patterns_from_motif_base,
    stitch_leader_star_patterns,
)


def test_stitch_three_member_star():
    mb = MotifBase()
    r0 = MotifRecord()
    r0.gid_a = 0
    r0.gid_b = 1
    r0.relative = Se2(1.0, 0.0, 0.0)
    r0.area_a = 2.0
    r0.area_b = 1.0
    r0.gci = 0.5
    r0.compactness = 0.5
    mid0 = int(mb.upsert(r0))
    r1 = MotifRecord()
    r1.gid_a = 0
    r1.gid_b = 2
    r1.relative = Se2(0.0, 1.0, 0.0)
    r1.area_a = 2.0
    r1.area_b = 1.0
    r1.gci = 0.6
    r1.compactness = 0.5
    mid1 = int(mb.upsert(r1))
    telem: dict = {}
    stitched, used = stitch_leader_star_patterns(mb, [mid0, mid1], telem=telem)
    assert len(stitched) == 1
    assert stitched[0].part_count == 3
    assert {int(m[0]) for m in stitched[0].members} == {0, 1, 2}
    assert used == {mid0, mid1}
    assert int(telem.get("star_stitch_n", 0)) == 1


def test_patterns_from_motif_base_prefers_stitch():
    mb = MotifBase()
    for gb, xy in ((1, (1.0, 0.0)), (2, (0.0, 1.0))):
        r = MotifRecord()
        r.gid_a = 0
        r.gid_b = int(gb)
        r.relative = Se2(float(xy[0]), float(xy[1]), 0.0)
        r.area_a = 2.0
        r.area_b = 1.0
        r.gci = 0.5
        r.compactness = 0.5
        mb.upsert(r)
    telem: dict = {}
    pats = patterns_from_motif_base(mb, max_keep=4, telem=telem)
    assert any(int(p.part_count) >= 3 for p in pats)
    assert int(telem.get("star_stitch_n", 0)) >= 1
