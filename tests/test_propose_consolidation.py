"""Consolidation invariants: zone truth table, densify-on-hijack, corners-once, orphans."""

from shapely import Point, Polygon, box

from nest_graph.config import ProposeConfig, PLACE_ZONES
from nest_graph.propose import __all__ as propose_all
from nest_graph.propose.geometry import ProposeGeometry
from nest_graph.propose.pipeline import collect_propose_candidates, proposed_transforms_for_groups
from nest_graph.proposer_names import (
    ZONE_PROPOSERS,
    first_pass_packed_border_proposers,
    PlaceZone,
    ProposerName,
)
from nest_graph.utils import transform_poly


def test_zone_proposer_flags_consistent():
    ProposeConfig.assert_zone_proposer_flags()


def test_void_seek_no_voronoi_contact_hybrid():
    proposers = ProposeConfig.proposers_for_place("void_seek")
    assert proposers is not None
    assert "voronoi" not in proposers
    assert ProposerName.VORONOI not in ZONE_PROPOSERS[PlaceZone.VOID_SEEK]
    cfg = ProposeConfig.for_place("void_seek")
    assert cfg.use_voronoi is False
    assert cfg.ranking_mode == "contact_hybrid"
    assert cfg.use_neighbor_slide is False
    assert "neighbor_slide" not in proposers
    cleared = ProposeConfig.for_place(
        "void_seek",
        base=ProposeConfig(void_seek_contact_hybrid=False),
    )
    assert cleared.ranking_mode == "clearance"


def test_first_pass_packed_is_border_gap_subset():
    packed = first_pass_packed_border_proposers()
    border = ZONE_PROPOSERS[PlaceZone.BORDER_GAP]
    assert packed <= border
    assert ProposerName.CLUSTER_COPY not in packed
    assert ProposerName.POCKET_FIT not in packed
    assert ProposerName.GUIDANCE_CAST_REFINE not in packed
    assert ProposerName.PERIMETER_WALK in packed


def test_orphans_removed():
    from pathlib import Path

    propose_dir = Path(__file__).resolve().parents[1] / "nest_graph" / "propose"
    assert not (propose_dir / "legacy_shapely.py").exists()
    assert not (propose_dir / "placement_axis.py").exists()
    for name in (
        "propose_placements_axis_push",
        "propose_placements_bottom_left",
        "propose_placements_nfp_vertices",
    ):
        assert name not in propose_all
        assert not hasattr(
            __import__("nest_graph.propose", fromlist=["pipeline"]),
            name,
        )


def test_sheet_corners_skipped_when_board_edge_enabled():
    sheet = box(0, 0, 20, 20)
    part = box(0, 0, 1, 1)
    cfg = ProposeConfig(
        use_board_edge_seeds=True,
        use_border_edge_seeds=True,
        use_border_focus=True,
        use_pocket_fit=False,
        use_cluster_copy=False,
        use_batch_pack=False,
        use_voronoi=False,
        use_point_cloud=False,
        use_guidance_propositions=False,
        use_ribbon_seeds=False,
        use_group_edge_seeds=False,
        use_neighbor_slide=False,
        candidate_pool=16,
        max_proposals=8,
        placement_num_angles=4,
        board_edge_samples_per_edge=4,
    )
    geom = ProposeGeometry(sheet, Polygon(), part, 0.1, propose_cfg=cfg)
    counts: dict[str, int] = {}
    collect_propose_candidates(
        Polygon(),
        part,
        sheet,
        cfg,
        min_dist=0.1,
        pt_push=Point(10, 10),
        propose_geom=geom,
        enabled_proposers=frozenset({
            "board_edge",
            "sheet_corners",
            "perimeter_walk",
        }),
        proposer_counts=counts,
    )
    assert counts.get("board_edge", 0) > 0
    assert counts.get("sheet_corners", 0) == 0
    assert "sheet_edge" not in counts


def test_densify_fires_on_void_hijack_when_pocket_zero():
    """Mode A hijack + empty pocket should densify (telemetry _densify_fired)."""
    sheet = box(0, 0, 50, 50)
    small = box(0, 0, 2.0, 2.0)
    part = box(0, 0, 3, 3)
    rim = [
        transform_poly(small, (float(x), float(y), 0.0))
        for x in (1, 24, 47)
        for y in (1, 24, 47)
        if (x, y) != (24, 24)
    ]
    cfg = ProposeConfig(
        place_profiles_enabled=True,
        densify_on_void_hijack=True,
        late_border_void_override_ratio=1.5,
        use_pocket_fit=False,
        use_cluster_copy=False,
        use_batch_pack=False,
        use_voronoi=False,
        candidate_pool=6,
        max_proposals=6,
        placement_num_angles=2,
        raycast_num_rays=2,
        raycast_num_angles=2,
    )
    counts: dict[str, int] = {}
    zones: list[str] = []
    densify_stats: dict = {}
    proposed_transforms_for_groups(
        sheet,
        [(part, 0)],
        rim,
        [0] * len(rim),
        cfg,
        min_dist=0.2,
        proposer_counts_out=counts,
        zones_used_out=zones,
        densify_stats_out=densify_stats,
    )
    assert any("void_seek" in z for z in zones)
    assert counts.get("_densify_fired", 0) >= 1
    assert densify_stats.get("fired", 0) >= 1
    assert counts.get("_pocket_fit_emitted", 0) == 0

    counts_off: dict[str, int] = {}
    densify_off: dict = {}
    proposed_transforms_for_groups(
        sheet,
        [(part, 0)],
        rim,
        [0] * len(rim),
        cfg.model_copy(update={"densify_on_void_hijack": False}),
        min_dist=0.2,
        proposer_counts_out=counts_off,
        zones_used_out=[],
        densify_stats_out=densify_off,
    )
    assert counts_off.get("_densify_fired", 0) == 0
    assert densify_off.get("fired", 0) == 0


def test_densify_hijack_ignores_low_free_ratio():
    """Hijack densify must not require free_ratio > 0.2 (rim-heavy packs)."""
    # Pack almost the whole sheet so free_ratio is tiny, but leave a large
    # relative void for Mode A (void_ratio vs part area).
    sheet = box(0, 0, 20, 20)
    filler = box(0, 0, 4.5, 4.5)
    packed = []
    for x in (0.2, 5.0, 9.8, 14.6):
        for y in (0.2, 5.0, 9.8, 14.6):
            if x >= 9.0 and y >= 9.0:
                continue  # leave SE corner open
            packed.append(transform_poly(filler, (float(x), float(y), 0.0)))
    part = box(0, 0, 2.0, 2.0)
    cfg = ProposeConfig(
        place_profiles_enabled=True,
        densify_on_void_hijack=True,
        late_border_void_override_ratio=1.2,
        use_pocket_fit=False,
        use_cluster_copy=False,
        use_batch_pack=False,
        use_voronoi=False,
        candidate_pool=8,
        max_proposals=8,
        placement_num_angles=2,
        raycast_num_rays=2,
        raycast_num_angles=2,
    )
    counts: dict[str, int] = {}
    zones: list[str] = []
    proposed_transforms_for_groups(
        sheet,
        [(part, 0)],
        packed,
        [0] * len(packed),
        cfg,
        min_dist=0.1,
        proposer_counts_out=counts,
        zones_used_out=zones,
    )
    if any("void_seek" in z for z in zones):
        assert counts.get("_densify_fired", 0) >= 1


def test_ablation_flags_toggle():
    cfg = ProposeConfig()
    assert cfg.motif_use_topo_anchors is True
    assert cfg.unified_void_reserve is True
    assert cfg.poles_reserve_only_on_hijack is True
    assert cfg.densify_on_void_hijack is True
    assert cfg.void_seek_contact_hybrid is True
    off = cfg.model_copy(update={
        "densify_on_void_hijack": False,
        "unified_void_reserve": False,
        "void_seek_contact_hybrid": False,
        "motif_use_topo_anchors": False,
        "poles_reserve_only_on_hijack": False,
    })
    assert off.densify_on_void_hijack is False
    assert ProposeConfig.for_place("void_seek", base=off).ranking_mode == "clearance"


def test_all_place_zones_have_zone_proposers():
    for zone in PLACE_ZONES:
        assert zone in ZONE_PROPOSERS
        assert ProposeConfig.proposers_for_place(zone) is not None


def test_rim_sat_mutes_history_expand_only_for_rim_sheet():
    from nest_graph.decision.action_gen import region_to_zone
    from nest_graph.elem_graph import MacroRegion
    from nest_graph.propose.transform_batch import rim_sat_proposer_updates

    void_u = rim_sat_proposer_updates("void_seek")
    assert void_u["use_side_pack"] is False
    assert "use_history_expand" not in void_u
    assert "use_cluster_copy" not in void_u
    motif_u = rim_sat_proposer_updates(region_to_zone(MacroRegion.Motif))
    assert "use_history_expand" not in motif_u
    assert rim_sat_proposer_updates(
        region_to_zone(MacroRegion.Rim),
    )["use_history_expand"] is False
    assert rim_sat_proposer_updates(
        region_to_zone(MacroRegion.Sheet),
    )["use_history_expand"] is False
    # Unknown / empty zone is not Rim/Sheet — keep replay.
    assert "use_history_expand" not in rim_sat_proposer_updates("")
