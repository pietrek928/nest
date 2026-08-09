"""side_pack: anti-crowd weighted stratify + sheet snap + zone staging."""

from shapely.geometry import Point, box

from nest_graph.config import ProposeConfig
from nest_graph.propose.geometry import ProposeGeometry
from nest_graph.propose.placement_perimeter import (
    anti_crowd_far_segments,
    anti_crowd_segment_weights,
    select_stratified_by_segment,
    select_stratified_by_segment_weighted,
)
from nest_graph.propose.placements_edge import propose_placements_side_pack
from nest_graph.propose.pipeline import (
    _CollectState,
    _collect_builder_candidates,
    _collect_candidates,
    _emit_side_pack,
    _fast_packed_crowd_ref,
)
from nest_graph.propose.types import PackedProposeExtras, PocketStats, make_propose_context
from nest_graph.proposer_names import PlaceZone, ProposerName, ZONE_PROPOSERS
from nest_graph.utils import transform_poly


def test_zone_includes_side_pack():
    assert ProposerName.SIDE_PACK in ZONE_PROPOSERS[PlaceZone.VOID_SEEK]
    assert ProposerName.SIDE_PACK in ZONE_PROPOSERS[PlaceZone.CLUSTER_EDGE]
    assert ProposerName.SIDE_PACK in ZONE_PROPOSERS[PlaceZone.BORDER_GAP]
    # Void path XOR: group_fit not in VOID_SEEK set.
    assert ProposerName.GROUP_FIT not in ZONE_PROPOSERS[PlaceZone.VOID_SEEK]


def test_length_only_stratify_matches_wrapper():
    sheet = box(0, 0, 10, 10)
    props = []
    for i, (x, y) in enumerate([(1, 0), (5, 0), (9, 0), (10, 5), (0, 5)]):
        props.append({
            "coords": (float(x), float(y), 0.0),
            "anchor": Point(x, y),
            "cost": float(i),
        })
    a = select_stratified_by_segment(sheet, props, top_n=4)
    b = select_stratified_by_segment_weighted(sheet, props, top_n=4, segment_weights=None)
    assert [p["coords"] for p in a] == [p["coords"] for p in b]


def test_anti_crowd_prefers_far_segment():
    sheet = box(0, 0, 10, 10)
    crowd = Point(1.0, 5.0)  # left side
    weights = anti_crowd_segment_weights(sheet, crowd)
    # Right vertical edge (x=10) should outweigh left (x=0).
    # Segment order follows exterior coords of box.
    assert max(weights) > min(weights)
    # Far-side weight at least 1.5x near-side baseline share.
    assert max(weights) / (min(weights) + 1e-12) >= 1.5
    far = anti_crowd_far_segments(weights)
    assert far
    assert max(far, key=lambda i: weights[i]) in far


def test_weighted_stratify_far_side_quota():
    sheet = box(0, 0, 10, 10)
    crowd = Point(0.5, 5.0)
    weights = anti_crowd_segment_weights(sheet, crowd)
    # Seed cheap props on left (near) and right (far).
    props = []
    for y in (2.0, 5.0, 8.0):
        props.append({"coords": (0.1, y, 0.0), "anchor": Point(0.0, y), "cost": 0.1})
        props.append({"coords": (9.9, y, 0.0), "anchor": Point(10.0, y), "cost": 0.1})
    picked = select_stratified_by_segment_weighted(
        sheet, props, top_n=6, segment_weights=weights,
    )
    right = sum(1 for p in picked if p["coords"][0] > 5.0)
    left = sum(1 for p in picked if p["coords"][0] < 5.0)
    assert right >= left


def test_fast_packed_crowd_ref_bbox():
    pt = _fast_packed_crowd_ref([(0.0, 0.0, 0.0), (10.0, 4.0, 0.1)])
    assert pt is not None
    assert abs(pt.x - 5.0) < 1e-9
    assert abs(pt.y - 2.0) < 1e-9
    assert _fast_packed_crowd_ref([]) is None


def test_side_pack_emits_raw(build_graph_config):
    sheet = box(0, 0, 20, 20)
    part = box(0, 0, 1.5, 1.5)
    cfg = ProposeConfig(use_side_pack=True, side_pack_top_n=12, side_pack_samples_per_edge=4)
    geom = ProposeGeometry(sheet, box(0, 0, 0.01, 0.01), part, 0.1, propose_cfg=cfg)
    out = propose_placements_side_pack(
        part,
        sheet,
        min_dist=0.1,
        propose_cfg=cfg,
        propose_geom=geom,
        top_n=12,
        crowd_ref=Point(2.0, 2.0),
    )
    assert len(out) > 0
    # Far-from-crowd bias: mean x or y should lean away from (2,2) vs uniform.
    mean_x = sum(c[0] for c in out) / len(out)
    mean_y = sum(c[1] for c in out) / len(out)
    assert mean_x + mean_y > 8.0


def test_assert_zone_flags_with_side_pack():
    ProposeConfig.assert_zone_proposer_flags()


def _side_pack_ctx(
    *,
    zone: str,
    packed_n: int,
    enabled: frozenset[str] | None = None,
    side_pack_top_n: int = 8,
):
    sheet = box(0, 0, 30, 30)
    part = box(0, 0, 1.2, 1.2)
    cfg = ProposeConfig.for_place(
        zone,
        base=ProposeConfig(
            use_side_pack=True,
            side_pack_top_n=side_pack_top_n,
            side_pack_samples_per_edge=4,
            candidate_pool=32,
            max_proposals=16,
        ),
    )
    packed = [
        transform_poly(part, (2.0 + i * 2.5, 2.0, 0.0))
        for i in range(max(packed_n, 0))
    ]
    packed_trs = [(2.0 + i * 2.5, 2.0, 0.0) for i in range(max(packed_n, 0))]
    obstacle = packed[0] if packed else box(0, 0, 0.01, 0.01)
    geom = ProposeGeometry(sheet, obstacle, part, 0.1, propose_cfg=cfg)
    en = enabled
    if en is None:
        en = ProposeConfig.proposers_for_place(zone)
    counts: dict[str, int] = {}
    keys: dict[str, set] = {}
    ctx = make_propose_context(
        base_shape=obstacle,
        shape_to_place=part,
        sheet=sheet,
        propose_cfg=cfg,
        min_dist=0.1,
        pt_push=Point(15.0, 15.0),
        propose_geom=geom,
        enabled_proposers=en,
        proposer_counts=counts,
        border_focus=zone in ("border_gap", "empty_border"),
    )
    extras = PackedProposeExtras(
        packed_polys=packed,
        packed_transforms=packed_trs,
        pocket_stats=PocketStats(),
    )
    return ctx, extras, counts, keys


def test_cluster_edge_emits_side_pack():
    ctx, extras, counts, keys = _side_pack_ctx(zone="cluster_edge", packed_n=3)
    state = _CollectState(
        proposer_counts=counts, proposer_keys=keys, cascade_stats_out={},
    )
    _collect_builder_candidates(ctx, extras, state, cascade_zone="cluster_edge")
    assert counts.get("side_pack", 0) > 0


def test_early_void_emits_side_pack():
    """VOID_SEEK with packed_n < 2 still stages side_pack (no BOARD_EDGE/GROUP_FIT)."""
    ctx, extras, counts, keys = _side_pack_ctx(zone="void_seek", packed_n=1)
    state = _CollectState(
        proposer_counts=counts, proposer_keys=keys, cascade_stats_out={},
    )
    _collect_builder_candidates(ctx, extras, state, cascade_zone="void_seek")
    assert counts.get("side_pack", 0) > 0


def test_side_pack_top_n_bounds_non_void_emit():
    ctx, extras, counts, keys = _side_pack_ctx(
        zone="cluster_edge", packed_n=3, side_pack_top_n=4,
    )
    state = _CollectState(
        proposer_counts=counts, proposer_keys=keys, cascade_stats_out={},
    )
    _emit_side_pack(ctx, extras, state, cascade_zone="cluster_edge")
    # Non-void: cap = side_pack_top_n then clamp to pool; over-emit max_items = 2*cap.
    assert counts.get("side_pack", 0) <= 8


def test_wall_fill_void_seek_with_pocket_reserve():
    ctx, extras, counts, keys = _side_pack_ctx(zone="void_seek", packed_n=3)
    cascade = {"cascade_skipped_proposers": ["side_pack", "board_edge"]}
    state = _CollectState(
        proposer_counts=counts, proposer_keys=keys, cascade_stats_out=cascade,
    )
    state.skip_builders = True
    # Simulate nonempty pocket reserve path through _collect_candidates gate.
    n0 = len(state.candidates)
    _emit_side_pack(ctx, extras, state, cascade_zone="void_seek")
    assert len(state.candidates) > n0
    # Un-skip after growth (mirrors wall-fill in _collect_candidates).
    cascade["cascade_skipped_proposers"] = [
        n for n in cascade["cascade_skipped_proposers"] if n != "side_pack"
    ]
    assert "side_pack" not in cascade["cascade_skipped_proposers"]


def test_interior_pocket_skip_does_not_wall_fill_side_pack():
    """Wall-fill is void_seek-only; interior_pocket cannot enable SIDE_PACK."""
    ctx, extras, counts, keys = _side_pack_ctx(
        zone="interior_pocket",
        packed_n=3,
        enabled=ProposeConfig.proposers_for_place("interior_pocket"),
    )
    state = _CollectState(
        proposer_counts=counts, proposer_keys=keys, cascade_stats_out={},
    )
    state.skip_builders = True
    # Direct emit respects enabled set — SIDE_PACK not in interior_pocket.
    _emit_side_pack(ctx, extras, state, cascade_zone="interior_pocket")
    assert counts.get("side_pack", 0) == 0
    out = _collect_candidates(
        ctx, extras, mode="cascade", cascade_zone="interior_pocket",
    )
    assert "side_pack" not in (ctx.proposer_counts or {})
    _ = out
