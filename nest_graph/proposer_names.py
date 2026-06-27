"""Canonical proposer identifiers and named enable sets."""

from enum import StrEnum


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
    SHEET_EDGE = "sheet_edge"
    BOARD_EDGE = "board_edge"
    GUIDANCE_CAST_REFINE = "guidance_cast_refine"
    BATCH_PACK = "batch_pack"


ALL_PROPOSER_NAMES: tuple[str, ...] = tuple(p.value for p in ProposerName)

BATCH_FOLLOW_PROPOSERS: frozenset[ProposerName] = frozenset({
    ProposerName.GROUP_FIT,
    ProposerName.NEIGHBOR_SLIDE,
    ProposerName.GUIDANCE_CAST_REFINE,
    ProposerName.PERIMETER_WALK,
    ProposerName.EROSION,
    ProposerName.RIBBON_FREE,
})

FIRST_PASS_EMPTY_BORDER_PROPOSERS: frozenset[ProposerName] = frozenset({
    ProposerName.BOARD_EDGE,
    ProposerName.SHEET_CORNERS,
    ProposerName.PERIMETER_WALK,
})


def first_pass_packed_border_proposers() -> frozenset[ProposerName]:
    return frozenset({
        ProposerName.BOARD_EDGE,
        ProposerName.SHEET_CORNERS,
        ProposerName.GROUP_FIT,
        ProposerName.NEIGHBOR_SLIDE,
        ProposerName.RIBBON_FREE,
    })
