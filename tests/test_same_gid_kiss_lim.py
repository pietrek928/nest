"""M1: extent-scaled same-gid kiss lim vs absolute 0.5."""

from nest_graph.placement.pattern import (
    _same_gid_kiss_xy_lim,
    _same_gid_rels_kiss,
)


def test_kiss_lim_below_triangle_mate_spacing():
    # 0.15×0.07 triangle: min extent ~0.07; mate Δxy ~0.07–0.15.
    lim = _same_gid_kiss_xy_lim(min_extent=0.07, min_dist=0.01)
    assert lim < 0.07
    assert lim == 0.25 * 0.07  # min(0.0175, 0.02)


def test_mate_rel_not_kiss_under_scaled_lim():
    lim = _same_gid_kiss_xy_lim(min_extent=0.07, min_dist=0.01)
    mate = [(0.0, 0.0, 0.0), (0.0, 0.07, 3.141592653589793)]
    assert not _same_gid_rels_kiss(mate, lim=lim)
    # Absolute 0.5 still fires (legacy pathology).
    assert _same_gid_rels_kiss(mate, lim=0.5)


def test_near_coincident_still_kiss():
    lim = _same_gid_kiss_xy_lim(min_extent=0.07, min_dist=0.01)
    near = [(0.0, 0.0, 0.0), (0.005, 0.0, 0.0)]
    assert _same_gid_rels_kiss(near, lim=lim)
