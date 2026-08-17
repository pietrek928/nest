"""Canonical place zones, proposer identifiers, and zone→proposer enable sets.

Leaf module: no imports from nest_graph.config (config imports this).
"""

from enum import StrEnum


class PlaceZone(StrEnum):
    EMPTY_BORDER = "empty_border"
    BORDER_GAP = "border_gap"
    INTERIOR_POCKET = "interior_pocket"
    CLUSTER_EDGE = "cluster_edge"
    INTER_CLUSTER = "inter_cluster"
    VOID_SEEK = "void_seek"


PLACE_ZONES: tuple[str, ...] = tuple(z.value for z in PlaceZone)


class ProposerName(StrEnum):
    PERIMETER_WALK = "perimeter_walk"
    NEIGHBOR_SLIDE = "neighbor_slide"
    EROSION = "erosion"
    RAYCASTING = "raycasting"
    VORONOI = "voronoi"
    POINT_CLOUD = "point_cloud"
    GUIDANCE_WALK = "guidance_walk"
    RIBBON_FREE = "ribbon_free"
    GROUP_FIT = "group_fit"
    SHEET_CORNERS = "sheet_corners"
    # sheet_edge folded into sheet_corners counting; kept for pool-scale keys only.
    SHEET_EDGE = "sheet_edge"
    BOARD_EDGE = "board_edge"
    SIDE_PACK = "side_pack"
    GUIDANCE_CAST_REFINE = "guidance_cast_refine"
    BATCH_PACK = "batch_pack"
    CLUSTER_COPY = "cluster_copy"
    POCKET_FIT = "pocket_fit"
    FREE_SPACE_CLOUD = "free_space_cloud"
    SELECTION_EXPAND = "selection_expand"
    HISTORY_EXPAND = "history_expand"


ALL_PROPOSER_NAMES: tuple[str, ...] = tuple(p.value for p in ProposerName)

# Dual-flag map: proposer → ProposeConfig attribute that must be True (None = no flag).
PROPOSER_FLAG: dict[ProposerName, str | None] = {
    ProposerName.PERIMETER_WALK: None,
    ProposerName.NEIGHBOR_SLIDE: "use_neighbor_slide",
    ProposerName.EROSION: None,
    ProposerName.RAYCASTING: None,
    ProposerName.VORONOI: "use_voronoi",
    ProposerName.POINT_CLOUD: "use_point_cloud",
    ProposerName.GUIDANCE_WALK: "use_guidance_walk",
    ProposerName.RIBBON_FREE: "use_ribbon_seeds",
    ProposerName.GROUP_FIT: "use_group_edge_seeds",
    ProposerName.SHEET_CORNERS: "use_border_edge_seeds",
    ProposerName.SHEET_EDGE: "use_border_edge_seeds",
    ProposerName.BOARD_EDGE: "use_board_edge_seeds",
    ProposerName.SIDE_PACK: "use_side_pack",
    ProposerName.GUIDANCE_CAST_REFINE: "use_guidance_propositions",
    ProposerName.BATCH_PACK: "use_batch_pack",
    ProposerName.CLUSTER_COPY: "use_cluster_copy",
    ProposerName.POCKET_FIT: "use_pocket_fit",
    ProposerName.FREE_SPACE_CLOUD: "use_free_space_cloud",
    ProposerName.SELECTION_EXPAND: None,
    ProposerName.HISTORY_EXPAND: None,
}

ZONE_PROPOSERS: dict[PlaceZone, frozenset[ProposerName]] = {
    PlaceZone.EMPTY_BORDER: frozenset({
        ProposerName.BOARD_EDGE,
        ProposerName.SHEET_CORNERS,
        ProposerName.PERIMETER_WALK,
    }),
    PlaceZone.BORDER_GAP: frozenset({
        ProposerName.BOARD_EDGE,
        ProposerName.SIDE_PACK,
        ProposerName.SHEET_CORNERS,
        ProposerName.PERIMETER_WALK,
        ProposerName.GROUP_FIT,
        ProposerName.NEIGHBOR_SLIDE,
        ProposerName.RIBBON_FREE,
        ProposerName.GUIDANCE_CAST_REFINE,
        ProposerName.CLUSTER_COPY,
        ProposerName.POCKET_FIT,
        ProposerName.SELECTION_EXPAND,
        ProposerName.HISTORY_EXPAND,
    }),
    PlaceZone.INTERIOR_POCKET: frozenset({
        ProposerName.EROSION,
        ProposerName.VORONOI,
        ProposerName.RIBBON_FREE,
        ProposerName.GUIDANCE_CAST_REFINE,
        ProposerName.RAYCASTING,
        ProposerName.CLUSTER_COPY,
        ProposerName.GROUP_FIT,
        ProposerName.POCKET_FIT,
        ProposerName.SELECTION_EXPAND,
        ProposerName.HISTORY_EXPAND,
    }),
    PlaceZone.CLUSTER_EDGE: frozenset({
        ProposerName.GROUP_FIT,
        ProposerName.SIDE_PACK,
        ProposerName.NEIGHBOR_SLIDE,
        ProposerName.GUIDANCE_CAST_REFINE,
        ProposerName.PERIMETER_WALK,
        ProposerName.EROSION,
        ProposerName.CLUSTER_COPY,
        ProposerName.RIBBON_FREE,
        ProposerName.POCKET_FIT,
        ProposerName.SELECTION_EXPAND,
        ProposerName.HISTORY_EXPAND,
    }),
    PlaceZone.INTER_CLUSTER: frozenset({
        ProposerName.RIBBON_FREE,
        ProposerName.RAYCASTING,
        ProposerName.VORONOI,
        ProposerName.EROSION,
        ProposerName.CLUSTER_COPY,
        ProposerName.GUIDANCE_CAST_REFINE,
        ProposerName.POCKET_FIT,
        ProposerName.SELECTION_EXPAND,
        ProposerName.HISTORY_EXPAND,
    }),
    # No VORONOI / NEIGHBOR_SLIDE — large open void drift spam; explorers = erosion+raycast.
    # No SIDE_PACK — staging void XOR skips wall-fill; permission matches staging.
    PlaceZone.VOID_SEEK: frozenset({
        ProposerName.EROSION,
        ProposerName.RIBBON_FREE,
        ProposerName.RAYCASTING,
        ProposerName.GUIDANCE_CAST_REFINE,
        ProposerName.BOARD_EDGE,
        ProposerName.CLUSTER_COPY,
        ProposerName.POCKET_FIT,
        ProposerName.FREE_SPACE_CLOUD,
        ProposerName.SELECTION_EXPAND,
        ProposerName.HISTORY_EXPAND,
    }),
}

CORRIDOR_PROPOSERS: frozenset[ProposerName] = frozenset({
    ProposerName.RAYCASTING,
    ProposerName.RIBBON_FREE,
    ProposerName.NEIGHBOR_SLIDE,
    ProposerName.GROUP_FIT,
    ProposerName.EROSION,
    ProposerName.GUIDANCE_CAST_REFINE,
    ProposerName.CLUSTER_COPY,
    ProposerName.POCKET_FIT,
})

ANNULUS_EXTRA_PROPOSERS: frozenset[ProposerName] = frozenset({
    ProposerName.RAYCASTING,
    ProposerName.EROSION,
})

_FIRST_PASS_PACKED_EXCLUDE: frozenset[ProposerName] = frozenset({
    ProposerName.CLUSTER_COPY,
    ProposerName.POCKET_FIT,
    ProposerName.GUIDANCE_CAST_REFINE,
})

BATCH_FOLLOW_PROPOSERS: frozenset[ProposerName] = frozenset({
    ProposerName.GROUP_FIT,
    ProposerName.NEIGHBOR_SLIDE,
    ProposerName.GUIDANCE_CAST_REFINE,
    ProposerName.PERIMETER_WALK,
    ProposerName.EROSION,
    ProposerName.RIBBON_FREE,
})

FIRST_PASS_EMPTY_BORDER_PROPOSERS: frozenset[ProposerName] = (
    ZONE_PROPOSERS[PlaceZone.EMPTY_BORDER]
)


def first_pass_packed_border_proposers() -> frozenset[ProposerName]:
    """BORDER_GAP minus motif/pocket/cast (explicit subtract)."""
    return ZONE_PROPOSERS[PlaceZone.BORDER_GAP] - _FIRST_PASS_PACKED_EXCLUDE


def proposers_for_zone(
    zone: str | PlaceZone,
    *,
    annulus: bool = False,
) -> frozenset[str] | None:
    try:
        place_zone = zone if isinstance(zone, PlaceZone) else PlaceZone(zone)
    except ValueError:
        return None
    proposers = ZONE_PROPOSERS.get(place_zone)
    if proposers is None:
        return None
    out = proposers
    if annulus and place_zone == PlaceZone.BORDER_GAP:
        out = out | ANNULUS_EXTRA_PROPOSERS
    return frozenset(p.value for p in out)
