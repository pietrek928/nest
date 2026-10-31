"""P2: raycasting emits via fused hit×angle pose list + filter_candidates_batch."""

from shapely.geometry import box

from nest_graph.config import ProposeConfig
from nest_graph.propose.geometry import ProposeGeometry
from nest_graph.propose.placements_geo import propose_placements_raycasting


def test_ray_pose_fuse_emits_unique_keys():
    sheet = box(0, 0, 20, 20)
    base = box(2, 2, 6, 6)
    part = box(0, 0, 1, 1)
    geom = ProposeGeometry(
        sheet, base, part, 0.2, propose_cfg=ProposeConfig(),
    )
    out = propose_placements_raycasting(
        base,
        part,
        geom.sheet,
        0.2,
        propose_geom=geom,
        pt_push=base.centroid,
        use_free_region=True,
        border_focus=False,
        num_rays=8,
        num_angles=4,
        top_n=8,
        anchor_stride=2,
    )
    assert out
    keys = {
        (round(float(c[0]), 4), round(float(c[1]), 4), round(float(c[2]), 4))
        for c in out
    }
    assert len(keys) == len(out)
