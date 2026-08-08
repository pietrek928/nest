"""Extra motif-hole unit test for pocket_fit."""

from shapely.geometry import box

from nest_graph.propose.placements_pattern import extract_cluster_patterns
from nest_graph.propose.placements_pocket import TAG_MOTIF_HOLE, propose_motif_hole_fills
from nest_graph.utils import transform_poly


def test_motif_hole_proposes_missing_member():
    # Contact-connected 2x2; group 0 appears once at (1,1).
    part = box(0, 0, 1, 1)
    full_trs = [
        (0.0, 0.0, 0.0),
        (1.0, 0.0, 0.0),
        (0.0, 1.0, 0.0),
        (1.0, 1.0, 0.0),
    ]
    full_gids = [1, 1, 2, 0]
    full_packed = [transform_poly(part, t) for t in full_trs]
    patterns = extract_cluster_patterns(
        full_packed, full_gids, full_trs, min_dist=0.05, max_patterns=1, min_members=3,
    )
    assert patterns
    # Incomplete: drop the group-0 corner.
    packed = full_packed[:3]
    gids = full_gids[:3]
    trs = full_trs[:3]
    fills = propose_motif_hole_fills(
        patterns, packed, gids, trs, group_id=0, min_dist=0.05,
    )
    assert fills, fills
    assert all(tag == TAG_MOTIF_HOLE for _, tag in fills)
    xs = [c[0] for c, _ in fills]
    ys = [c[1] for c, _ in fills]
    assert any(abs(x - 1.0) < 0.25 and abs(y - 1.0) < 0.25 for x, y in zip(xs, ys)), (xs, ys)
