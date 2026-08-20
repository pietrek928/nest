"""Shared 3b ↔ stamp repair cohort (Q202–Q207).

One victim + one pattern vocabulary for mid-pack hole emit and last-leaf stamp.
Victim pick policies stay separate (Q58/Q59); this module only orchestrates.
"""

from dataclasses import dataclass, field
from typing import Any, Sequence

from shapely import Point, Polygon
from shapely.geometry.base import BaseGeometry

from nest_graph.propose.block_replace import pick_block_hole_victim
from nest_graph.propose.placements_pattern import (
    ClusterPattern,
    cluster_pattern_from_indices,
    merge_cluster_patterns,
)


# repair_mode telem (Q211)
REPAIR_SKIP = 0
REPAIR_EMIT = 1
REPAIR_STAMP = 2
REPAIR_EMIT_OK = 3


@dataclass
class RepairCohort:
    """Q202: mid→post handoff; never recompute victim/patterns between stages."""

    victim: list[int] | None = None
    patterns: list[ClusterPattern] = field(default_factory=list)
    mode: int = REPAIR_SKIP
    emit_in_hull: int = 0
    accepted: bool = False
    board_adj: bool = False

    def telem_into(self, propose_stats: dict | None) -> None:
        if propose_stats is None:
            return
        propose_stats["repair_mode"] = int(self.mode)
        propose_stats["repair_patterns_n"] = int(len(self.patterns))
        propose_stats["repair_victim_n"] = int(len(self.victim or ()))
        if self.victim is not None:
            propose_stats["block_hole_victim"] = list(self.victim)
        propose_stats["block_hole_emit_in_hull"] = int(self.emit_in_hull)
        if self.accepted:
            propose_stats["block_hole_accepted"] = 1


def pick_repair_victim(
    selected: Sequence[int],
    polys: Sequence[BaseGeometry],
    *,
    min_dist: float,
    sheet: Polygon,
    pole: Point | None = None,
    void_poly: Any = None,
    min_size: int = 3,
    max_size: int = 6,
) -> tuple[list[int] | None, bool]:
    """Q203: 3b victim first; BFS peel only if mid-3b returned nothing."""
    victim = pick_block_hole_victim(
        selected,
        polys,
        min_dist=min_dist,
        sheet=sheet,
        pole=pole,
        void_poly=void_poly,
        min_size=min_size,
        max_size=max_size,
        allow_board_adj_fallback=True,
    )
    if victim:
        return list(victim), False
    # cycle: cluster_repack → pipeline; repair_cohort must not import cluster_repack at top
    from nest_graph.propose.cluster_repack import bfs_peel_victim

    peeled = bfs_peel_victim(
        selected,
        polys,
        min_dist=min_dist,
        sheet=sheet,
        pole=pole,
        void_poly=void_poly,
        min_size=min_size,
        max_size=max_size,
    )
    if not peeled:
        return None, False
    victim_bfs, board_adj = peeled
    return list(victim_bfs), bool(board_adj)


def build_repair_patterns(
    *,
    victim: Sequence[int],
    polys: Sequence[BaseGeometry],
    group_ids: Sequence[int],
    transforms: Sequence,
    kept: Sequence[int],
    min_dist: float,
    sheet: Polygon | None,
    archived: Sequence[ClusterPattern] = (),
    max_patterns: int = 8,
) -> list[ClusterPattern]:
    """Q205: peel + capped contact ∪ archived via one merge_cluster_patterns (Q10)."""
    # cycle: cluster_repack → pipeline; lazy import keeps repair_cohort import-safe
    from nest_graph.propose.cluster_repack import extract_capped_subpatterns

    contact: list[ClusterPattern] = []
    peel_pat = cluster_pattern_from_indices(victim, polys, group_ids, transforms)
    if peel_pat is not None:
        contact.append(peel_pat)
    if kept:
        contact.extend(
            extract_capped_subpatterns(
                [polys[i] for i in kept],
                [int(group_ids[i]) for i in kept],
                [transforms[i] for i in kept],
                min_dist=min_dist,
                max_members=len(victim),
                sheet=sheet,
            )
        )
    return merge_cluster_patterns(
        contact,
        (),
        max_patterns=max(int(max_patterns), 1),
        archived=list(archived or ()),
        reserve_archived=1 if archived else 0,
    )


def begin_repair_cohort(
    selected: Sequence[int],
    polys: Sequence[BaseGeometry],
    group_ids: Sequence[int],
    transforms: Sequence,
    *,
    min_dist: float,
    sheet: Polygon,
    pole: Point | None = None,
    void_poly: Any = None,
    archived: Sequence[ClusterPattern] = (),
    max_patterns: int = 8,
    min_size: int = 3,
    max_size: int = 6,
) -> RepairCohort:
    """Pick victim + build shared pattern list before 3b emit."""
    cohort = RepairCohort()
    victim, board_adj = pick_repair_victim(
        selected,
        polys,
        min_dist=min_dist,
        sheet=sheet,
        pole=pole,
        void_poly=void_poly,
        min_size=min_size,
        max_size=max_size,
    )
    if not victim:
        return cohort
    victim_set = set(int(i) for i in victim)
    kept = [int(i) for i in selected if int(i) not in victim_set]
    cohort.victim = list(victim)
    cohort.board_adj = bool(board_adj)
    cohort.patterns = build_repair_patterns(
        victim=victim,
        polys=polys,
        group_ids=group_ids,
        transforms=transforms,
        kept=kept,
        min_dist=min_dist,
        sheet=sheet,
        archived=archived,
        max_patterns=max_patterns,
    )
    cohort.mode = REPAIR_EMIT
    return cohort
