"""Sequential multi-group packing: place one part, then pack the next against it."""

from typing import Sequence

import numpy as np
from shapely.geometry import Point, Polygon
from shapely.geometry.base import BaseGeometry
from shapely.ops import unary_union

from nest_graph.board import board_context_from_geometry
from nest_graph.config import ProposeConfig, dedupe_transforms
from nest_graph.propose.context import (
    _placement_contact_error,
    effective_ranking_mode,
    focal_shape_for_propose,
    propose_push_point,
    should_use_border_focus,
)
from nest_graph.propose.geometry import ProposeGeometry
from nest_graph.utils import transform_poly

_BATCH_FOLLOW_PROPOSERS = frozenset({
    "group_fit",
    "neighbor_slide",
    "guidance_propositions",
    "perimeter_walk",
    "erosion",
    "ribbon_free",
})


def _follow_proposer_coords(
    board: BaseGeometry,
    obstacle_shape: BaseGeometry,
    follow_poly: Polygon,
    propose_cfg: ProposeConfig,
    *,
    min_dist: float,
    pt_push: Point,
    focal: BaseGeometry | None,
    sheet: Polygon,
) -> list[tuple[float, float, float]]:
    from nest_graph.propose.pipeline import (
        _propose_coords_from_candidates,
        collect_propose_candidates,
    )

    rank_mode = effective_ranking_mode(propose_cfg, obstacle_shape)
    geom = ProposeGeometry(
        board,
        obstacle_shape,
        follow_poly,
        min_dist,
        epsilon_ratio=propose_cfg.placement_clearance_epsilon_ratio,
        propose_cfg=propose_cfg,
    )
    candidates = collect_propose_candidates(
        obstacle_shape,
        follow_poly,
        sheet,
        propose_cfg,
        min_dist=min_dist,
        pt_push=pt_push,
        propose_geom=geom,
        focal_shape=focal,
        enabled_proposers=_BATCH_FOLLOW_PROPOSERS,
    )
    return _propose_coords_from_candidates(
        obstacle_shape,
        follow_poly,
        board,
        propose_cfg,
        min_dist=min_dist,
        pt_push=pt_push,
        candidates=candidates,
        rank_mode=rank_mode,
        focal_shape=focal,
    )


def _parts_by_group(
    parts: Sequence[tuple[Polygon, int]],
) -> dict[int, Polygon]:
    out: dict[int, Polygon] = {}
    for poly, gid in parts:
        if gid not in out:
            out[gid] = poly
    return out


def _top_seed_coords(
    arr: np.ndarray,
    n: int,
) -> list[tuple[float, float, float]]:
    if arr.shape[0] == 0 or n <= 0:
        return []
    take = min(n, arr.shape[0])
    return [tuple(float(x) for x in row) for row in arr[:take]]


def _pair_valid(
    sheet: Polygon,
    poly_a: Polygon,
    coords_a: tuple[float, float, float],
    poly_b: Polygon,
    coords_b: tuple[float, float, float],
    base_obstacle: BaseGeometry,
    min_dist: float,
) -> bool:
    placed_a = transform_poly(poly_a, coords_a)
    placed_b = transform_poly(poly_b, coords_b)
    if not sheet.contains(placed_a) or not sheet.contains(placed_b):
        return False
    eps = max(1e-6, min_dist * 1e-4)
    if not base_obstacle.is_empty:
        if base_obstacle.intersects(placed_a) or base_obstacle.intersects(placed_b):
            return False
        if base_obstacle.distance(placed_a) < min_dist - eps:
            return False
        if base_obstacle.distance(placed_b) < min_dist - eps:
            return False
    if placed_a.intersects(placed_b):
        return False
    if placed_a.distance(placed_b) < min_dist - eps:
        return False
    return True


def _pair_contact_score(
    placed_a: BaseGeometry,
    placed_b: BaseGeometry,
    sheet: Polygon,
    min_dist: float,
    focal: BaseGeometry | None,
) -> float:
    err_a = _placement_contact_error(placed_a, sheet, min_dist, focal)
    err_b = _placement_contact_error(placed_b, sheet, min_dist, placed_a)
    inter = abs(float(placed_a.distance(placed_b)) - min_dist)
    return err_a + err_b + inter


def _batch_pack_pair_order(
    board: BaseGeometry,
    anchor_gid: int,
    follow_gid: int,
    parts_by_group: dict[int, Polygon],
    base_obstacle: BaseGeometry,
    per_group: dict[int, np.ndarray],
    propose_cfg: ProposeConfig,
    *,
    min_dist: float,
    placed: Sequence[BaseGeometry],
) -> list[tuple[tuple[float, float, float], tuple[float, float, float], float]]:
    anchor_poly = parts_by_group[anchor_gid]
    follow_poly = parts_by_group[follow_gid]
    anchor_seeds = _top_seed_coords(
        per_group.get(anchor_gid, np.zeros((0, 3))),
        propose_cfg.batch_pack_anchor_seeds,
    )
    if not anchor_seeds:
        return []

    follow_cfg = propose_cfg.model_copy(
        update={
            "max_proposals": propose_cfg.batch_pack_follow_proposals,
            "candidate_pool": min(
                propose_cfg.candidate_pool,
                propose_cfg.batch_pack_follow_pool,
            ),
        },
    )
    sheet, _voids = board_context_from_geometry(board)
    pairs: list[tuple[tuple[float, float, float], tuple[float, float, float], float]] = []

    for coords_a in anchor_seeds:
        placed_a = transform_poly(anchor_poly, coords_a)
        if not sheet.contains(placed_a):
            continue
        if not base_obstacle.is_empty:
            if base_obstacle.intersects(placed_a):
                continue
            if base_obstacle.distance(placed_a) < min_dist - 1e-6:
                continue

        extended = (
            unary_union([base_obstacle, placed_a])
            if not base_obstacle.is_empty
            else placed_a
        )
        placed_with_anchor = list(placed) + [placed_a]
        focal = focal_shape_for_propose(
            board, placed_with_anchor, follow_poly, min_dist, propose_cfg,
        )
        border_focus = should_use_border_focus(extended, propose_cfg)
        push = propose_push_point(
            board,
            extended,
            smart_push=propose_cfg.smart_push_target,
            min_dist=min_dist,
            use_border_focus=border_focus,
        )
        follow_coords = _follow_proposer_coords(
            board,
            extended,
            follow_poly,
            follow_cfg,
            min_dist=min_dist,
            pt_push=push,
            focal=focal,
            sheet=sheet,
        )
        if not follow_coords:
            continue

        focal_anchor = (
            unary_union([focal, placed_a])
            if focal is not None and not focal.is_empty
            else placed_a
        )
        for coords_b in follow_coords[: propose_cfg.batch_pack_follow_proposals]:
            if not _pair_valid(
                sheet, anchor_poly, coords_a, follow_poly, coords_b,
                base_obstacle, min_dist,
            ):
                continue
            placed_b = transform_poly(follow_poly, coords_b)
            score = _pair_contact_score(
                placed_a, placed_b, sheet, min_dist, focal_anchor,
            )
            pairs.append((coords_a, coords_b, score))

    pairs.sort(key=lambda p: p[2])
    return pairs[: propose_cfg.batch_pack_max_pairs]


def augment_batch_pack_proposals(
    board: BaseGeometry,
    parts: Sequence[tuple[Polygon, int]],
    placed: Sequence[BaseGeometry],
    per_group: dict[int, np.ndarray],
    propose_cfg: ProposeConfig,
    *,
    min_dist: float,
) -> dict[int, list[tuple[float, float, float]]]:
    """Pack groups sequentially; return extra (x, y, angle) seeds per group."""
    if not propose_cfg.use_batch_pack:
        return {}

    parts_by_group = _parts_by_group(parts)
    group_ids = sorted(parts_by_group)
    if len(group_ids) < 2:
        return {}

    base_obstacle = unary_union(placed) if placed else Polygon()
    all_pairs: list[
        tuple[tuple[float, float, float], tuple[float, float, float], float, int, int]
    ] = []

    orders: list[tuple[int, int]] = []
    if len(group_ids) == 2:
        orders = [(group_ids[0], group_ids[1]), (group_ids[1], group_ids[0])]
    else:
        for anchor in group_ids:
            for follow in group_ids:
                if follow != anchor:
                    orders.append((anchor, follow))

    seen_orders: set[tuple[int, int]] = set()
    for anchor_gid, follow_gid in orders:
        if (anchor_gid, follow_gid) in seen_orders:
            continue
        seen_orders.add((anchor_gid, follow_gid))
        for coords_a, coords_b, score in _batch_pack_pair_order(
            board,
            anchor_gid,
            follow_gid,
            parts_by_group,
            base_obstacle,
            per_group,
            propose_cfg,
            min_dist=min_dist,
            placed=placed,
        ):
            all_pairs.append((coords_a, coords_b, score, anchor_gid, follow_gid))

    all_pairs.sort(key=lambda p: p[2])
    all_pairs = all_pairs[: propose_cfg.batch_pack_max_pairs]

    extra: dict[int, list[tuple[float, float, float]]] = {gid: [] for gid in group_ids}
    per_gid_cap = max(1, propose_cfg.batch_pack_max_pairs // len(group_ids))
    gid_counts = {gid: 0 for gid in group_ids}

    for coords_a, coords_b, _score, anchor_gid, follow_gid in all_pairs:
        if gid_counts[anchor_gid] < per_gid_cap:
            extra[anchor_gid].append(coords_a)
            gid_counts[anchor_gid] += 1
        if gid_counts[follow_gid] < per_gid_cap:
            extra[follow_gid].append(coords_b)
            gid_counts[follow_gid] += 1

    for gid in group_ids:
        if extra[gid]:
            extra[gid] = list(
                map(
                    tuple,
                    dedupe_transforms(np.asarray(extra[gid], dtype=np.float64)),
                ),
            )
    return extra
