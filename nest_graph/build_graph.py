# CFLAGS="-Wno-error=incompatible-pointer-types" pip install --force-reinstall --no-binary=shapely --upgrade shapely

import cv2 as cv
import math
import time
import numpy as np
from pydantic import BaseModel, ConfigDict
from shapely import Polygon, unary_union
from shapely.geometry import box as shapely_box
from shapely.geometry import Point
from shapely.ops import nearest_points
from shapely.geometry.base import BaseGeometry
from tqdm import tqdm
from typing import Iterator, NamedTuple, Sequence, Tuple

from .config import (
    BuildGraphConfig,
    SelectionConfig,
    dedupe_transforms,
    expand_structured_transforms,
    score_rules_options,
    shuffle_transforms,
    subsample_transforms_with_pinned,
    trim_history,
)
from .geometry import (
    Geometry,
    GuidanceConfig,
    batch_check_validity,
    find_polygon_distances,
    find_polygon_intersections,
    find_polygon_intersections_bipartite,
)
from .placement_scene import (
    PlacementScene,
    guidance_config_for_board_edge_anchor,
    guidance_config_for_graph,
    is_valid_placement,
    placement_footprint_inside_board,
    placement_ok_for_outline,
    placement_scene_for_part,
    footprints_inside_board,
    proposition_translation,
)
from .propose.placement_common import clear_of_geoms
from .propose.placement_outline import (
    outline_ring_geom,
    outline_kiss_tolerance,
    outline_standoff_distance,
)
from .propose.placement_perimeter import edge_inward_at_point
from .propose.placements_guidance import (
    candidate_from_proposition,
    is_cast_move,
    sorted_guidance_propositions,
)
from .board import (
    board_context_from_geometry,
    default_sheet_padding,
    padded_board_bounds,
)
from .utils import normalize_poly, transform_poly
from .propose import (
    propose_placements_point_cloud,
    proposed_transforms_for_groups,
    border_edge_transforms_for_group,
)
from .propose.compaction import (
    compact_selection,
    selection_pairwise_independent,
)
from .propose.cluster_repack import (
    cluster_repack_selection,
    cluster_relocate_selection,
)
from .propose.local_se2 import local_se2_selection
from .propose.context import (
    analyze_free_space,
    build_free_space_snapshot,
    outline_coverage_ratio,
    part_is_concave,
    propose_push_point,
    sheet_has_narrow_corridor,
    should_use_border_focus,
)
from .propose.feedback import ProposeFeedbackState
from .propose.ranking import score_placement_tightness
from .propose.geometry import ProposeGeometry
from .track_perf import show_performance
from .elem_graph import (
    ElemGraph, Circle, Vec2,
    PointPlaceRule, PointAngleRule, PlacementRuleSet,
    RuleMutationSettings, RefineSelectionOptions, FinalizeSelectionOptions,
    nest_by_graph, sort_graph, score_elems, augment_rules, score_rules,
    ScoreRulesOptions,
    increase_selection_dfs, increase_score_dfs,
    refine_selection, finalize_selection, selection_is_independent,
)

# Track performance
nest_by_graph = show_performance(nest_by_graph)
sort_graph = show_performance(sort_graph)
score_elems = show_performance(score_elems)
# augment_rules = show_performance(augment_rules)
score_rules = show_performance(score_rules)


class Candidate(NamedTuple):
    group_i: int
    weight: float
    t: np.ndarray
    placed: Geometry


class NestState(NamedTuple):
    polys: list
    group_id: list
    transform: list
    selected_indices: list
    # First seed_count entries in polys are locked obstacles (extra_voids), not graph nodes.
    seed_count: int = 0


def nest_state_extra_voids(nest_state: NestState | None) -> list[Geometry] | None:
    """Geometry voids for locked seed parts (clearance obstacles, not candidates)."""
    if nest_state is None or nest_state.seed_count <= 0:
        return None
    n = min(nest_state.seed_count, len(nest_state.polys))
    if n <= 0:
        return None
    return [Geometry.from_shapely(p) for p in nest_state.polys[:n]]


def _poly_and_transforms(item):
    if len(item) == 2:
        return item[0], item[1], 1.0
    return item[0], item[2], item[1]


def _selection_coverage_pct(
    selected_indices: list[int],
    group_id: list[int],
    part_areas: tuple[float, ...],
    board_area: float,
) -> float:
    if board_area <= 0:
        return 0.0
    parts_area = sum(part_areas[group_id[i]] for i in selected_indices)
    return 100.0 * parts_area / board_area


def _late_border_saturation_active(
    cfg: BuildGraphConfig,
    nest_state: NestState | None,
    board: BaseGeometry,
    *,
    had_void_override: bool = False,
) -> bool:
    """Return whether late border-only propose should run.

    Exp1: skip sat when free space is a large contiguous void.
    Exp3: when not large_void, also require pack-hull rim progress below threshold.
    """
    return _late_border_saturation_info(
        cfg, nest_state, board, had_void_override=had_void_override,
    ).active


class LateBorderSatInfo(NamedTuple):
    active: bool
    outline_cov: float
    sat_override: bool
    rim_progress: float
    free_kind: str


def _late_border_saturation_info(
    cfg: BuildGraphConfig,
    nest_state: NestState | None,
    board: BaseGeometry,
    *,
    had_void_override: bool = False,
) -> LateBorderSatInfo:
    empty = LateBorderSatInfo(
        active=False, outline_cov=0.0, sat_override=False,
        rim_progress=0.0, free_kind="",
    )
    if nest_state is None or not cfg.propose.late_border_saturation:
        return empty
    placed = [
        nest_state.polys[i]
        for i in nest_state.selected_indices
        if nest_state.polys[i] is not None and not nest_state.polys[i].is_empty
    ]
    if not placed:
        return empty
    sheet, _ = board_context_from_geometry(board)
    min_dist = cfg.board_min_dist()
    outline_cov = float(outline_coverage_ratio(placed, sheet, min_dist))
    threshold = float(cfg.propose.place_border_coverage_threshold)

    mean_part = float(np.mean([float(p.area) for p in placed]))
    void_thr = float(cfg.propose.late_border_void_override_ratio)
    if void_thr <= 0.0:
        void_thr = 2.5
    free_info = analyze_free_space(
        sheet, placed, mean_part, min_dist, void_ratio_threshold=void_thr,
    )
    free_kind = str(free_info.kind)
    void_ratio = float(free_info.max_void_ratio)
    override_ratio = float(cfg.propose.late_border_void_override_ratio)
    release_ratio = float(cfg.propose.late_border_void_release_ratio)
    entry_override = (
        override_ratio > 0.0
        and free_info.kind == "large_void"
        and void_ratio > override_ratio
    )
    # Hysteresis: once unlocked this pack, hold while void still meaningful.
    hold_override = (
        had_void_override
        and release_ratio > 0.0
        and outline_cov < threshold
        and void_ratio > release_ratio
    )
    sat_override = entry_override or hold_override

    sheet_perim = float(sheet.exterior.length) if not sheet.is_empty else 0.0
    rim_progress = 0.0
    if sheet_perim > 1e-12:
        hull = unary_union(placed).convex_hull
        if hull is not None and not hull.is_empty and hasattr(hull, "exterior"):
            rim_progress = float(
                np.clip(float(hull.exterior.length) / sheet_perim, 0.0, 1.0)
            )

    if outline_cov >= threshold:
        # Rim metric already satisfied — no late border-only.
        return LateBorderSatInfo(
            active=False,
            outline_cov=outline_cov,
            sat_override=False,
            rim_progress=rim_progress,
            free_kind=free_kind,
        )
    if sat_override:
        # Exp1 / hysteresis: unlock full propose despite low outline_kiss_cov.
        return LateBorderSatInfo(
            active=False,
            outline_cov=outline_cov,
            sat_override=True,
            rim_progress=rim_progress,
            free_kind=free_kind,
        )
    # Without large_void, keep sat only while pack hull_rim_fill is still open.
    hull_thr = float(cfg.propose.late_border_hull_threshold)
    if hull_thr > 0.0 and rim_progress >= hull_thr:
        return LateBorderSatInfo(
            active=False,
            outline_cov=outline_cov,
            sat_override=False,
            rim_progress=rim_progress,
            free_kind=free_kind,
        )
    return LateBorderSatInfo(
        active=True,
        outline_cov=outline_cov,
        sat_override=False,
        rim_progress=rim_progress,
        free_kind=free_kind,
    )


def _base_geometries(polygons) -> list[Geometry]:
    return [
        Geometry.from_shapely(_poly_and_transforms(item)[0])
        for item in polygons
    ]


def _iter_candidates(
    board: BaseGeometry,
    polygons,
    *,
    sheet=None,
    void_geoms=None,
    min_dist: float = 0.0,
    epsilon_ratio: float = 0.05,
) -> Iterator[Candidate]:
    if sheet is None or void_geoms is None:
        sheet, void_geoms = board_context_from_geometry(board)
    bounds = sheet.bounds
    guidance_cfg = guidance_config_for_graph(
        min_dist, board_bounds=bounds, epsilon_ratio=epsilon_ratio
    )
    board_geom = Geometry.from_shapely(sheet)
    bases = _base_geometries(polygons)
    for i, item in enumerate(polygons):
        _p, transforms, w = _poly_and_transforms(item)
        base = bases[i]
        for t in transforms:
            placed = base.apply_transform(t)
            scene = placement_scene_for_part(sheet, board_geom, void_geoms, base)
            cx, cy = placed.center()
            if not is_valid_placement(
                scene, placed, (cx, cy), min_dist, guidance_cfg,
                epsilon_ratio=epsilon_ratio,
            ):
                continue
            yield Candidate(i, w, t, placed)


def _fill_collision_matrix(
    M: np.ndarray,
    geoms: list[Geometry],
    bboxes: list[tuple[float, float, float, float]],
) -> None:
    hits = find_polygon_intersections(geoms)
    for i, j in hits:
        M[i, j] = M[j, i] = 1.0


def placement_board_score(
    board: BaseGeometry,
    board_geom: Geometry,
    placed: Geometry,
) -> float:
    if not placement_footprint_inside_board(placed, board_geom):
        cx, cy = placed.center()
        return -board.distance(Point(cx, cy))
    cx, cy = placed.center()
    return 2.0 * board.boundary.distance(Point(cx, cy))


def make_placement_base(base_shape, polys, exclude_p, exclude_dist=0):
    out_polys = [base_shape.exterior]
    for p in polys:
        if exclude_dist:
            if p.exterior.distance(exclude_p) > exclude_dist:
                out_polys.append(p)
        else:
            if not p.intersects(exclude_p):
                out_polys.append(p)
    return unary_union(out_polys)


def placement_base_ribbon(polys, p_center, exclued_dist, include_dist):
    out_polys = []
    for p in polys:
        dist = p.distance(p_center)
        if dist > exclued_dist and dist < include_dist:
            out_polys.append(p)
    return unary_union(out_polys)


class PolygonGroup(BaseModel):
    model_config = ConfigDict(arbitrary_types_allowed=True)

    polygon: Polygon
    weight: float
    transforms: np.ndarray


def polygon_board_distance(b: BaseGeometry, p: Polygon):
    if b.contains(p):
        return b.exterior.distance(p) + b.exterior.distance(p.centroid)
    if p.intersects(b):
        return 0
    return -b.distance(p)


def select_non_intersecting_polygons(polygons: np.ndarray):
    selected_geoms: list[Geometry] = []
    selected: list = []
    for p in polygons:
        geom = Geometry.from_shapely(p)
        if geom.intersects_any(selected_geoms):
            continue
        selected_geoms.append(geom)
        selected.append(p)
    return selected


def select_polygons_from_edges(b: BaseGeometry, polygons: Tuple[Tuple[Polygon, float, np.ndarray], ...]):
    board_geom = Geometry.from_shapely(b)
    result = [[] for _ in range(len(polygons))]
    scored: list[tuple[float, int, np.ndarray, Geometry]] = []
    bases = _base_geometries(polygons)

    for i, item in enumerate(polygons):
        _p, transforms, w = _poly_and_transforms(item)
        base = bases[i]
        for t in transforms:
            placed = base.apply_transform(t)
            score = placement_board_score(b, board_geom, placed)
            if score > 0:
                scored.append((
                    placed.center()[0] + w + np.random.rand() * 1e-4,
                    i,
                    t,
                    placed,
                ))

    scored.sort(key=lambda x: x[0])

    selected_geoms: list[Geometry] = []
    for _, pnum, t, placed in scored:
        if not placed.intersects_any(selected_geoms):
            selected_geoms.append(placed)
            result[pnum].append(t)

    return tuple(np.array(tt) for tt in result)


def make_polygon_matrix(
    b: BaseGeometry,
    polygons: Tuple[Tuple[Polygon, float, np.ndarray], ...],
    *,
    min_dist: float = 0.0,
    epsilon_ratio: float = 0.05,
):
    sheet, void_geoms = board_context_from_geometry(b)
    selected_geoms: list[Geometry] = []
    group_weights = []

    for cand in _iter_candidates(
        b, polygons, sheet=sheet, void_geoms=void_geoms,
        min_dist=min_dist, epsilon_ratio=epsilon_ratio,
    ):
        selected_geoms.append(cand.placed)
        group_weights.append(cand.weight)

    n = len(selected_geoms)
    bboxes = [g.bounds() for g in selected_geoms]
    M = np.zeros((n, n), dtype=np.float32)
    _fill_collision_matrix(M, selected_geoms, bboxes)
    for i in range(n):
        M[i, i] = -group_weights[i]

    return M, selected_geoms


@show_performance
def make_polygon_graph(
    b: BaseGeometry,
    polygons,
    *,
    min_dist: float = 0.0,
    epsilon_ratio: float = 0.05,
    user_holes: tuple[tuple[tuple[float, float], ...], ...] = (),
    extra_voids: list[Geometry] | None = None,
):
    sheet, void_geoms = board_context_from_geometry(b, user_holes=user_holes)
    if extra_voids:
        void_geoms.extend(extra_voids)
    board_geom = Geometry.from_shapely(sheet)
    pad = default_sheet_padding(b)
    guidance_cfg = guidance_config_for_graph(
        min_dist,
        board_bounds=padded_board_bounds(b, pad),
        epsilon_ratio=epsilon_ratio,
    )
    graph = ElemGraph()
    selected_polys = []
    selected_group_id = []
    selected_transform = []
    bases = _base_geometries(polygons)

    candidates: list[tuple] = []
    placed_solids: list[Geometry] = []
    for i, item in enumerate(polygons):
        if len(item) == 2:
            p, transforms = item
        else:
            p, _w, transforms = item
        base = bases[i]
        for t in transforms:
            placed = base.apply_transform(t)
            candidates.append((i, p, t, placed, base))
            placed_solids.append(placed)

    footprint_ok = (
        footprints_inside_board(placed_solids, board_geom)
        if placed_solids
        else []
    )

    by_group: dict[int, list[int]] = {}
    for k, (i, _p, _t, _placed, _base) in enumerate(candidates):
        if footprint_ok[k]:
            by_group.setdefault(i, []).append(k)

    valid_by_k: dict[int, bool] = {}
    for group_i, indices in by_group.items():
        base = bases[group_i]
        transforms = [
            (float(candidates[k][2][0]), float(candidates[k][2][1]), float(candidates[k][2][2]))
            for k in indices
        ]
        flags = batch_check_validity(
            base,
            transforms,
            void_geoms,
            guidance_cfg,
            min_dist,
            epsilon_ratio,
        )
        for k, ok in zip(indices, flags, strict=True):
            valid_by_k[k] = ok

    pending: list[tuple] = []
    for k, (i, p, t, placed, base) in enumerate(candidates):
        if not footprint_ok[k] or not valid_by_k.get(k, False):
            continue
        pending.append((i, p, t, placed))

    graph.reserve_elems(len(pending))
    pending_geoms = []
    for k, (i, p, t, placed) in enumerate(pending):
        graph.append_elem(
            i,
            Vec2(x=placed.center()[0], y=placed.center()[1]),
            Circle.from_center_radius(*placed.center(), placed.radius()),
        )
        selected_polys.append(transform_poly(p, t))
        pending_geoms.append(placed)
        selected_group_id.append(i)
        selected_transform.append(t)

    hits = find_polygon_intersections(pending_geoms)
    for i, j in hits:
        graph.add_collision(i, j)

    return graph, selected_polys, selected_group_id, selected_transform


def optimize_polygons(M: np.ndarray, v: np.ndarray):
    score = 1e9
    for it in range(1280):
        v += np.random.rand(M.shape[0]) * 1e-5
        v -= 1.5e-2 * (M @ v)
        v = np.clip(v, 0, 1)
        score = v @ M @ v
        print(f'iter {it}, score {score}, mean {v.mean()}, max {v.max()}')
    return v


def score_transforms(b: BaseGeometry, p: Polygon, transforms: np.ndarray):
    scores = np.zeros((transforms.shape[0], ))
    for i, t in enumerate(transforms):
        p_t = transform_poly(p, t)
        scores[i] = polygon_board_distance(b, p_t)
    return scores


def transforms_around(
    p: np.ndarray,
    s: Tuple[float, float, float],
    n: int,
    rng: np.random.Generator,
) -> np.ndarray:
    sx, sy, sa = s
    return np.concatenate([
        p + rng.uniform(-1, 1, (p.shape[0], 3)) * [sx, sy, sa]
        for _ in range(n)
    ])


def transform_shuffle_mix(
    sel: np.ndarray,
    hist: np.ndarray,
    count: int,
    rng: np.random.Generator,
    scale: Tuple[float, float, float],
) -> np.ndarray:
    """Resample shuffled selection/history rows with fresh jitter."""
    parts = [arr for arr in (sel, hist) if arr.shape[0] > 0]
    if not parts or count <= 0:
        return np.zeros((0, 3))
    merged = np.concatenate(parts)
    rng.shuffle(merged)
    if merged.shape[0] >= count:
        picked = merged[:count]
    else:
        extra = rng.integers(0, merged.shape[0], size=count - merged.shape[0])
        picked = np.concatenate([merged, merged[extra]])
    jitter = rng.uniform(-1, 1, (picked.shape[0], 3)) * scale
    return picked + jitter


def scale_coords(
    coords: np.ndarray,
    xstart: float, ystart: float,
    xscale: float, yscale: float
) -> np.ndarray:
    x, y = coords.T
    x = (x - xstart) * xscale
    y = (y - ystart) * yscale
    return np.stack([x, y], axis=-1).astype(np.int32)

FILL_COLORS = (
    (100, 0, 0),
    (0, 100, 0),
    (0, 0, 100),
    (100, 100, 0),
    (100, 0, 100),
    (0, 100, 100),
)


def _draw_nesting_outline(
    im,
    outline: BaseGeometry,
    xstart: float,
    ystart: float,
    xscale: float,
    yscale: float,
) -> None:
    """Draw nest outline only (not the sheet bbox or corner voids)."""
    geoms = outline.geoms if hasattr(outline, "geoms") else [outline]
    for g in geoms:
        if g.is_empty:
            continue
        if g.geom_type != "Polygon":
            continue
        cv.drawContours(im, [scale_coords(
            np.array(g.exterior.coords), xstart, ystart, xscale, yscale
        )], -1, (255, 255, 255), 3)
        for ring in g.interiors:
            cv.drawContours(im, [scale_coords(
                np.array(ring.coords), xstart, ystart, xscale, yscale
            )], -1, (160, 160, 160), 2)


def render_placement(
    b: BaseGeometry,
    elems: Tuple[Tuple[Polygon, np.ndarray], ...],
    im_shape=(1024, 1024),
    *,
    nest_outline: BaseGeometry | None = None,
):
    xstart, ystart, xend, yend = b.bounds
    xscale = im_shape[0] / (xend - xstart)
    yscale = im_shape[1] / (yend - ystart)
    im = np.zeros((im_shape[0], im_shape[1], 3), dtype=np.uint8)
    _draw_nesting_outline(im, nest_outline if nest_outline is not None else b,
                          xstart, ystart, xscale, yscale)
    for it, (p, transforms) in enumerate(elems):
        fill_col = FILL_COLORS[it % len(FILL_COLORS)]
        for t in transforms:
            p_t = transform_poly(p, t)
            cv.drawContours(im, [scale_coords(
                np.array(p_t.exterior.coords), xstart, ystart, xscale, yscale
            )], -1, fill_col, cv.FILLED)
            cv.drawContours(im, [scale_coords(
                np.array(p_t.exterior.coords), xstart, ystart, xscale, yscale
            )], -1, (255, 255, 255), 3)
    return im


def render_selection(
    b: BaseGeometry,
    polys: Tuple[Polygon, ...],
    v: np.ndarray,
    im_shape=(1024, 1024),
    *,
    nest_outline: BaseGeometry | None = None,
):
    xstart, ystart, xend, yend = b.bounds
    xscale = im_shape[0] / (xend - xstart)
    yscale = im_shape[1] / (yend - ystart)
    im = np.zeros((im_shape[0], im_shape[1], 3), dtype=np.uint8)
    _draw_nesting_outline(im, nest_outline if nest_outline is not None else b,
                          xstart, ystart, xscale, yscale)
    for w, p in sorted(zip(v, polys), key=lambda x: x[0]):
        cv.drawContours(im, [scale_coords(
            np.array(p.exterior.coords), xstart, ystart, xscale, yscale
        )], -1, (100*w, 100*w, 100*w), cv.FILLED)
        cv.drawContours(im, [scale_coords(
            np.array(p.exterior.coords), xstart, ystart, xscale, yscale
        )], -1, (255*w, 255*w, 255*w), 3)
    return im


def render_polys(
    b: BaseGeometry,
    polys: Tuple[Tuple[Polygon, ...], ...],
    im_shape=(1024, 1024),
    *,
    nest_outline: BaseGeometry | None = None,
):
    xstart, ystart, xend, yend = b.bounds
    xscale = im_shape[0] / (xend - xstart)
    yscale = im_shape[1] / (yend - ystart)
    im = np.zeros((im_shape[0], im_shape[1], 3), dtype=np.uint8)
    _draw_nesting_outline(im, nest_outline if nest_outline is not None else b,
                          xstart, ystart, xscale, yscale)
    for it, poly_set in enumerate(polys):
        fill_col = FILL_COLORS[it % len(FILL_COLORS)]
        for p in poly_set:
            cv.drawContours(im, [scale_coords(
                np.array(p.exterior.coords), xstart, ystart, xscale, yscale
            )], -1, fill_col, cv.FILLED)
            cv.drawContours(im, [scale_coords(
                np.array(p.exterior.coords), xstart, ystart, xscale, yscale
            )], -1, (255, 255, 255), 3)
    return im


def _rule_region(board: BaseGeometry) -> Circle:
    xmin, ymin, xmax, ymax = board.bounds
    return Circle.from_bounds(xmin, ymin, xmax, ymax)


def _quantize_rule_scalar(v: float, places: int = 4) -> float:
    return round(float(v), places)


def _fingerprint_rule_set(rule_set: PlacementRuleSet) -> tuple:
    parts: list[tuple] = []
    q = _quantize_rule_scalar
    for pr in rule_set.point_rules:
        parts.append(
            ("p", pr.group, q(pr.pos[0]), q(pr.pos[1]), q(pr.r), q(pr.w)),
        )
    for cr in rule_set.circle_rules:
        parts.append(
            (
                "c",
                cr.group,
                q(cr.circle.center.x),
                q(cr.circle.center.y),
                q(cr.circle.radius),
                q(cr.r),
                q(cr.w),
            ),
        )
    for pr in rule_set.point_angle_rules:
        parts.append(
            (
                "pa",
                pr.group,
                q(pr.pos[0]),
                q(pr.pos[1]),
                q(pr.a),
                q(pr.r),
                q(pr.w),
            ),
        )
    for cr in rule_set.circle_angle_rules:
        parts.append(
            (
                "ca",
                cr.group,
                q(cr.circle.center.x),
                q(cr.circle.center.y),
                q(cr.a),
                q(cr.r),
                q(cr.w),
            ),
        )
    return tuple(sorted(parts))


def dedupe_rule_sets(rule_sets: list[PlacementRuleSet]) -> list[PlacementRuleSet]:
    seen: set[tuple] = set()
    out: list[PlacementRuleSet] = []
    for rs in rule_sets:
        key = _fingerprint_rule_set(rs)
        if key in seen:
            continue
        seen.add(key)
        out.append(rs)
    return out


def truncate_rule_set(
    rule_set: PlacementRuleSet,
    max_rules: int,
) -> PlacementRuleSet:
    if rule_set.size() <= max_rules:
        return rule_set
    weighted: list[tuple[float, object]] = []
    for pr in rule_set.point_rules:
        weighted.append((pr.w, pr))
    for cr in rule_set.circle_rules:
        weighted.append((cr.w, cr))
    for pr in rule_set.point_angle_rules:
        weighted.append((pr.w, pr))
    for cr in rule_set.circle_angle_rules:
        weighted.append((cr.w, cr))
    weighted.sort(key=lambda item: abs(item[0]), reverse=True)
    out = PlacementRuleSet()
    for _, rule in weighted[:max_rules]:
        out.append_rule(rule)
    return out


def active_rule_set(rule_sets: list[PlacementRuleSet]) -> PlacementRuleSet:
    if not rule_sets:
        return PlacementRuleSet()
    return rule_sets[0]


def _copy_rule_set(rule_set: PlacementRuleSet) -> PlacementRuleSet:
    out = PlacementRuleSet()
    for pr in rule_set.point_rules:
        out.append_rule(pr)
    for cr in rule_set.circle_rules:
        out.append_rule(cr)
    for pr in rule_set.point_angle_rules:
        out.append_rule(pr)
    for cr in rule_set.circle_angle_rules:
        out.append_rule(cr)
    return out


def _repulsor_centroids(
    board: BaseGeometry,
    nest_state: NestState | None,
) -> list[tuple[float, float]]:
    sheet, _ = board_context_from_geometry(board)
    centroids: list[tuple[float, float]] = []
    c = sheet.centroid
    centroids.append((float(c.x), float(c.y)))
    if nest_state is not None and nest_state.selected_indices:
        packed = [nest_state.polys[i] for i in nest_state.selected_indices]
        merged = unary_union(packed)
        if merged is not None and not merged.is_empty:
            mc = merged.centroid
            centroids.append((float(mc.x), float(mc.y)))
    return centroids


def _inject_repulsor_rules(
    rule_sets: list[PlacementRuleSet],
    cfg: BuildGraphConfig,
    board: BaseGeometry,
    nest_state: NestState | None,
) -> list[PlacementRuleSet]:
    if not cfg.rules.use_repulsor_rules or not rule_sets:
        return rule_sets
    centroids = _repulsor_centroids(board, nest_state)
    r = max(cfg.rules.place_rule_radius * 0.5, 1e-4)
    w = cfg.rules.repulsor_weight
    ngroups = cfg.rules.ngroups
    n_touch = min(max(cfg.rules.repulsor_sets_to_touch, 0), len(rule_sets))
    out: list[PlacementRuleSet] = []
    for i, rs in enumerate(rule_sets):
        if i >= n_touch:
            out.append(rs)
            continue
        copied = _copy_rule_set(rs)
        for g in range(ngroups):
            for cx, cy in centroids[:2]:
                copied.append_rule(PointPlaceRule(
                    pos=Vec2(x=cx, y=cy), r=r, w=w, group=g,
                ))
        out.append(truncate_rule_set(copied, cfg.rules.max_rules_per_set))
    return out


def _propose_rules_for_iter(
    cfg: BuildGraphConfig,
    rule_sets: list[PlacementRuleSet],
) -> PlacementRuleSet | None:
    if not cfg.propose.use_rule_ranking:
        return None
    if rule_sets:
        return active_rule_set(rule_sets)
    return active_rule_set(_make_initial_rule_sets(cfg))


def improve_rules(
    graphs,
    rules,
    n: int,
    board: BaseGeometry | None = None,
    *,
    mutation_presets: list[RuleMutationSettings] | None = None,
    rule_score_penalty: float = 0.03,
    elite_count: int = 16,
    seed: int = 0,
    score_options: ScoreRulesOptions | None = None,
    max_rules_per_set: int = 24,
):
    if mutation_presets is None:
        region = _rule_region(board) if board is not None else Circle.from_bounds(0, 0, 1.2, 1.1)
        ng = 2
        mutation_presets = [
            RuleMutationSettings(
                region=region, dpos=0.25, dw=0.25, da=np.pi / 4,
                insert_p=0.09, remove_p=0.02, mutate_p=0.1, ngroups=ng,
            ),
            RuleMutationSettings(
                region=region, dpos=0.05, dw=0.05, da=np.pi / 32,
                insert_p=0.04, remove_p=0.01, mutate_p=0.1, ngroups=ng,
            ),
            RuleMutationSettings(
                region=region, dpos=0.01, dw=0.01, da=np.pi / 64,
                insert_p=0.01, remove_p=0.02, mutate_p=0.1, ngroups=ng,
            ),
        ]
    if score_options is None:
        score_options = ScoreRulesOptions()
        score_options.rule_complexity_penalty = rule_score_penalty
    elif score_options.rule_complexity_penalty == 0.0:
        score_options.rule_complexity_penalty = rule_score_penalty

    parents = list(rules)
    elites = parents
    if graphs and parents:
        rank_opts = ScoreRulesOptions()
        rank_opts.latest_graph_only = score_options.latest_graph_only
        rank_opts.count_weight = score_options.count_weight
        rank_opts.rule_complexity_penalty = score_options.rule_complexity_penalty
        rank_opts.select = score_options.select
        parent_scores = score_rules(graphs, parents, rank_opts)
        ranked = sorted(
            zip(parent_scores, parents),
            key=lambda item: item[0],
            reverse=True,
        )
        k = min(max(elite_count, 1), len(ranked))
        elites = [rs for _, rs in ranked[:k]]

    pool: list[PlacementRuleSet] = list(parents)
    for preset_idx, preset in enumerate(mutation_presets):
        mutate_seed = (int(seed) + preset_idx * 10007) & 0xFFFFFFFF
        children = augment_rules(elites, preset, seed=mutate_seed)
        pool.extend(children)

    pool = dedupe_rule_sets(pool)
    if max_rules_per_set > 0:
        pool = [truncate_rule_set(rs, max_rules_per_set) for rs in pool]
    if not pool:
        return []

    fitness = score_rules(graphs, pool, score_options)
    scored = sorted(zip(fitness, pool), key=lambda item: item[0], reverse=True)
    return [rs for _, rs in scored[:n]]


def score_rule_sets_with_dfs(
    graph: ElemGraph,
    rule_sets: list[PlacementRuleSet],
    selection: SelectionConfig,
    *,
    top_k: int = 4,
) -> list[float]:
    """Tier-B fitness for benchmarks: nest + DFS count on latest graph (top_k by Tier A)."""
    if not rule_sets:
        return []
    tier_a = score_rules([graph], rule_sets, score_rules_options(selection))
    order = sorted(range(len(rule_sets)), key=lambda i: tier_a[i], reverse=True)
    out = list(tier_a)
    for idx in order[: max(top_k, 0)]:
        rs = rule_sets[idx]
        selected = list(nest_by_graph(graph, [rs])[0])
        scores = score_elems(graph, rs)
        _, final, _ = apply_dfs_refinement(
            graph, rs, selected, scores, selection=selection,
        )
        out[idx] = float(len(final)) - selection.rule_score_penalty * rs.size()
    return out


def window_selected_transforms(
    selection_window: list[tuple[np.ndarray, ...]] | None,
    ngroups: int = 2,
) -> tuple[np.ndarray, ...]:
    """Merge per-iteration nest selections from the graph window (deduped per group)."""
    if not selection_window:
        return tuple(np.zeros((0, 3)) for _ in range(ngroups))
    
    out = []
    for i in range(ngroups):
        parts = [w[i] for w in selection_window if len(w) > i and w[i].shape[0] > 0]
        if parts:
            out.append(dedupe_transforms(np.concatenate(parts, axis=0)))
        else:
            out.append(np.zeros((0, 3)))
    return tuple(out)


def _append_selection_window(
    selection_window: list[tuple[np.ndarray, np.ndarray]],
    selected_t: tuple[np.ndarray, np.ndarray],
    maxlen: int,
) -> None:
    if not any(t.shape[0] for t in selected_t):
        return
    selection_window.append(selected_t)
    if len(selection_window) > maxlen:
        del selection_window[: len(selection_window) - maxlen]


def _greedy_independent_set_ordered(
    graph: ElemGraph,
    order: list[int],
) -> list[int]:
    """Greedy MIS following ``order`` (no score re-sort)."""
    kept: list[int] = []
    kept_set: set[int] = set()
    for v in order:
        if any(u in kept_set for u in graph.collisions[v]):
            continue
        kept.append(v)
        kept_set.add(v)
    return kept


def _nest_seed_from_boosted_scores(
    graph: ElemGraph,
    scores: Sequence[float],
) -> list[int]:
    """Score-descending greedy MIS so Python void boost reaches the nest seed."""
    n = min(len(scores), len(graph.collisions))
    if n <= 0:
        return []
    order = sorted(range(n), key=lambda i: float(scores[i]), reverse=True)
    return _greedy_independent_set_ordered(graph, order)


def _void_attractor_radius(min_dist: float, sheet_diag: float, place_rule_radius: float) -> float:
    """Sheet-aware PointPlaceRule radius (legacy local r is too sharp for large voids)."""
    return max(float(place_rule_radius), float(min_dist) * 4.0, 0.25 * float(sheet_diag))


def _nest_outline_boundary(outline: BaseGeometry):
    if hasattr(outline, "exterior"):
        return outline.exterior
    return outline.boundary


def _outline_standoff_distance(poly, outline: BaseGeometry) -> float:
    return outline_standoff_distance(poly, outline)


def _border_kiss_tolerance(min_dist: float, *, scale: float = 2.0) -> float:
    return outline_kiss_tolerance(min_dist, scale=scale)


def _border_kiss_indices(
    polys: list,
    outline: BaseGeometry,
    min_dist: float,
) -> list[int]:
    tol = _border_kiss_tolerance(min_dist)
    return [
        i
        for i, poly in enumerate(polys)
        if abs(_outline_standoff_distance(poly, outline) - min_dist) <= tol
    ]


def _perimeter_sort_key(poly, outline: BaseGeometry) -> float:
    ring = _nest_outline_boundary(outline)
    touch = nearest_points(ring, poly.centroid)[0]
    return float(ring.project(touch))


def _expand_border_selection(
    graph: ElemGraph,
    polys: list,
    outline: BaseGeometry,
    min_dist: float,
    scores: list[float],
    initial: list[int],
) -> list[int]:
    """Greedy MIS on outline-kiss nodes, preserving ``initial`` and walking the perimeter."""
    kept_set = set(initial)
    marked = [False] * len(polys)
    for i in kept_set:
        marked[i] = True
        for j in graph.collisions[i]:
            marked[j] = True
    border = _border_kiss_indices(polys, outline, min_dist)
    order = sorted(
        border,
        key=lambda i: (
            _perimeter_sort_key(polys[i], outline),
            abs(_outline_standoff_distance(polys[i], outline) - min_dist),
            -scores[i],
        ),
    )
    for i in order:
        if marked[i]:
            continue
        kept_set.add(i)
        marked[i] = True
        for j in graph.collisions[i]:
            marked[j] = True
    return sorted(kept_set)


def _first_pass_border_ring_selection(
    graph: ElemGraph,
    polys: list,
    outline: BaseGeometry,
    min_dist: float,
    scores: list[float],
) -> list[int]:
    """Pack as many outline-kiss nodes as possible, walking the nest perimeter."""
    border = _border_kiss_indices(polys, outline, min_dist)
    if not border:
        return []
    order = sorted(
        border,
        key=lambda i: (
            _perimeter_sort_key(polys[i], outline),
            abs(_outline_standoff_distance(polys[i], outline) - min_dist),
            -scores[i],
        ),
    )
    return _greedy_independent_set_ordered(graph, order)


def _transform_row_key(t: np.ndarray) -> tuple[float, float, float]:
    return (round(float(t[0]), 4), round(float(t[1]), 4), round(float(t[2]), 4))


def _locked_graph_indices(
    transforms: list[np.ndarray],
    phase1_transforms: list[np.ndarray],
) -> list[int]:
    keys = {_transform_row_key(t) for t in phase1_transforms}
    return [i for i, t in enumerate(transforms) if _transform_row_key(t) in keys]


def _prepend_group_transforms(
    phase1: np.ndarray,
    batch: np.ndarray,
) -> np.ndarray:
    if phase1.shape[0] == 0:
        return batch
    if batch.shape[0] == 0:
        return phase1
    return dedupe_transforms(np.concatenate([phase1, batch], axis=0))


def _border_saturation_transform_batch(
    cfg: BuildGraphConfig,
    board: BaseGeometry,
    parts: list[tuple[Polygon, int]],
    nest_state: NestState,
) -> tuple[np.ndarray, np.ndarray]:
    """Propose-only batch for outline saturation (no random/history noise)."""
    min_dist = cfg.board_min_dist(first_pass=True)
    polys = nest_state.polys
    selected = nest_state.selected_indices
    group_id = nest_state.group_id
    transform = nest_state.transform
    part_by_gid = {gid: poly for poly, gid in parts}
    pack_polys = [polys[i] for i in selected]
    phase1_by_group: list[list[np.ndarray]] = [[], []]
    for idx in selected:
        phase1_by_group[group_id[idx]].append(transform[idx])
    propose_by_group = proposed_transforms_for_groups(
        board,
        parts,
        polys,
        selected,
        cfg.first_pass_propose_config(),
        min_dist=min_dist,
        border_only_propose=True,
        use_full_packed_obstacle=cfg.propose.use_full_packed_obstacle,
        packed_group_ids=group_id,
        packed_transforms=transform,
        user_holes=cfg.rules.board_holes,
        seeded=bool(nest_state.seed_count > 0),
        pocket_keys_out={},
        densify_stats_out={},
        zones_used_out=[],
    )
    out: list[np.ndarray] = []
    for gid in range(len(phase1_by_group)):
        phase1 = (
            np.asarray(phase1_by_group[gid], dtype=np.float64)
            if phase1_by_group[gid]
            else np.zeros((0, 3))
        )
        proposed = propose_by_group.get(gid, np.zeros((0, 3)))
        augment_coords = _border_augment_coords(
            cfg,
            board,
            part_by_gid[gid],
            pack_polys,
            min_dist=min_dist,
        )
        augment = (
            np.asarray(augment_coords, dtype=np.float64)
            if augment_coords
            else np.zeros((0, 3))
        )
        extra = dedupe_transforms(
            np.concatenate([proposed, augment], axis=0),
        )
        cap = max(cfg.propose.first_pass_max_proposals * 4, 128)
        room = max(cap - phase1.shape[0], 0)
        if extra.shape[0] > room:
            extra = extra[:room]
        merged = dedupe_transforms(np.concatenate([phase1, extra], axis=0))
        out.append(merged)
    return (out[0], out[1])


def _outline_anchor_inward(
    poly,
    outline: BaseGeometry,
) -> tuple[Point, tuple[float, float]]:
    ring = _nest_outline_boundary(outline)
    anchor, _ = nearest_points(ring, poly)
    if isinstance(outline, Polygon):
        edge_info = edge_inward_at_point(outline, anchor)
        if edge_info is not None:
            return edge_info
    if hasattr(outline, "representative_point"):
        interior = outline.representative_point()
    else:
        interior = ring.interpolate(0.5, normalized=True)
    ox = anchor.x - interior.x
    oy = anchor.y - interior.y
    dist = float(np.hypot(ox, oy))
    if dist < 1e-9:
        return anchor, (-1.0, -1.0)
    return anchor, (ox / dist, oy / dist)


def _neighbor_excess_gap(geoms: list[Geometry], min_dist: float) -> float:
    if len(geoms) < 2:
        return 0.0
    results = find_polygon_distances(geoms, aura=0.5)
    nearest = [float("inf")] * len(geoms)
    for r in results:
        d = 0.0 if r.intersect else math.sqrt(r.distance_sq)
        if 0 <= r.polyA_idx < len(nearest):
            nearest[r.polyA_idx] = min(nearest[r.polyA_idx], d)
        if 0 <= r.polyB_idx < len(nearest):
            nearest[r.polyB_idx] = min(nearest[r.polyB_idx], d)
    return sum(
        max(0.0, nd - min_dist)
        for nd in nearest
        if nd < float("inf")
    )


def _border_tightness_cost(
    polys: list,
    outline: BaseGeometry,
    min_dist: float,
) -> float:
    """Lower is tighter: excess neighbor gap plus outline standoff error."""
    if not polys:
        return 0.0
    geoms = [Geometry.from_shapely(p) for p in polys]
    ring_geom = outline_ring_geom(outline)
    if ring_geom is not None:
        kiss = sum(
            abs(g.standoff_distance(ring_geom) - min_dist)
            for g in geoms
        )
    else:
        kiss = sum(
            abs(_outline_standoff_distance(poly, outline) - min_dist)
            for poly in polys
        )
    excess_gap = _neighbor_excess_gap(geoms, min_dist)
    return 2.0 * excess_gap + 1.0 * kiss


def _border_refine_candidates(
    x: float,
    y: float,
    theta: float,
    g,
    *,
    min_dist: float,
    max_props: int,
) -> list[tuple[float, float, float]]:
    """Cast snaps plus fractional slide steps along guidance propositions."""
    out: list[tuple[float, float, float]] = []
    seen: set[tuple[float, float, float]] = set()

    def add(coords: tuple[float, float, float]) -> None:
        key = (round(coords[0], 4), round(coords[1], 4), round(coords[2], 4))
        if key in seen:
            return
        seen.add(key)
        out.append(coords)

    for prop in sorted_guidance_propositions(g)[:max_props]:
        use_cast = not g.is_penetrating and is_cast_move(prop.move_type or "")
        add(candidate_from_proposition(x, y, theta, prop, use_full_cast=use_cast))
        if use_cast:
            continue
        tx, ty = proposition_translation(prop)
        mag = math.hypot(tx, ty)
        if mag < 1e-9:
            continue
        step_scale = 0.2
        step_len = step_scale * max(min_dist, 1e-4)
        if not g.is_penetrating:
            step_len = step_scale * mag
        for frac in (0.15, 0.25, 0.4, 0.7, 1.0):
            nx = x + tx / mag * step_len * frac
            ny = y + ty / mag * step_len * frac
            add((nx, ny, theta))
    return out


def _border_refine_micro_walk(
    x: float,
    y: float,
    theta: float,
    g,
    *,
    scene: PlacementScene,
    part_geom: Geometry,
    part_poly: Polygon,
    others_polys: list,
    outline: BaseGeometry,
    edge_cfg: GuidanceConfig,
    min_dist: float,
    eps: float,
    walk_steps: int = 4,
    step_scale: float = 0.2,
) -> tuple[float, float, float]:
    """Step along top guidance slides from an accepted refine pose."""
    cx, cy, ctheta = x, y, theta
    for _ in range(walk_steps):
        props = sorted_guidance_propositions(g)[:3]
        if not props:
            break
        moved = False
        for prop in props:
            tx, ty = proposition_translation(prop)
            mag = math.hypot(tx, ty)
            if mag < 1e-9:
                continue
            use_cast = not g.is_penetrating and is_cast_move(prop.move_type or "")
            if use_cast:
                step_len = mag
            else:
                step_len = step_scale * max(min_dist, 1e-4)
                if not g.is_penetrating:
                    step_len = step_scale * mag
            nx = cx + tx / mag * step_len
            ny = cy + ty / mag * step_len
            ntheta = ctheta
            if abs(float(prop.rotation_rad)) > 1e-6:
                delta = float(prop.rotation_rad) - ctheta
                while delta > math.pi:
                    delta -= 2 * math.pi
                while delta < -math.pi:
                    delta += 2 * math.pi
                ntheta = ctheta + delta * (1.0 if use_cast else 0.2)
            cand_geom = part_geom.apply_transform(
                np.asarray((nx, ny, ntheta), dtype=np.float64),
            )
            cand_poly = transform_poly(part_poly, (nx, ny, ntheta))
            if not placement_ok_for_outline(
                scene,
                cand_geom,
                cand_poly,
                outline,
                others_polys,
                min_dist,
                edge_cfg,
                epsilon_ratio=eps,
                require_outline_kiss=True,
            ):
                continue
            cx, cy, ctheta = nx, ny, ntheta
            placed = scene.placed_at((cx, cy, ctheta))
            g = scene.guidance(placed, (cx, cy), edge_cfg)
            moved = True
            break
        if not moved:
            break
    return (cx, cy, ctheta)


def _guidance_border_refine(
    cfg: BuildGraphConfig,
    board: BaseGeometry,
    parts: list[tuple[Polygon, int]],
    *,
    outline: BaseGeometry,
    pack_polys: list,
    pack_gids: list[int],
    pack_tr: list[np.ndarray],
) -> tuple[list, list[int], list[np.ndarray]]:
    """Tighten border ring placements using per-anchor guidance casts."""
    passes = max(cfg.propose.first_pass_guidance_refine_passes, 0)
    if passes <= 0 or len(pack_polys) < 2:
        return pack_polys, pack_gids, pack_tr

    min_dist = cfg.board_min_dist(first_pass=True)
    eps = cfg.placement_epsilon_ratio(first_pass=True)
    propose_cfg = cfg.first_pass_propose_config()
    sheet, voids = board_context_from_geometry(board)
    board_geom = Geometry.from_shapely(sheet)
    pad = default_sheet_padding(board)
    bounds = padded_board_bounds(board, pad)
    part_by_gid = {gid: poly for poly, gid in parts}
    bases = {gid: Geometry.from_shapely(part_by_gid[gid]) for gid in part_by_gid}

    polys = list(pack_polys)
    gids = list(pack_gids)
    trs = [np.asarray(t, dtype=np.float64) for t in pack_tr]
    geoms = [bases[gid].apply_transform(t) for gid, t in zip(gids, trs, strict=True)]
    max_props = max(propose_cfg.guidance_max_propositions, 1)

    for _ in range(passes):
        base_cost = _border_tightness_cost(polys, outline, min_dist)
        improved = False
        order = sorted(
            range(len(polys)),
            key=lambda i: _perimeter_sort_key(polys[i], outline),
        )
        for idx in order:
            gid = gids[idx]
            part_poly = part_by_gid[gid]
            x, y, theta = float(trs[idx][0]), float(trs[idx][1]), float(trs[idx][2])
            anchor, inward = _outline_anchor_inward(polys[idx], outline)
            others_geoms = [geoms[j] for j in range(len(geoms)) if j != idx]
            others_polys = [polys[j] for j in range(len(polys)) if j != idx]
            scene = placement_scene_for_part(
                sheet, board_geom, voids, bases[gid], base_geoms=others_geoms,
            )
            edge_cfg = guidance_config_for_board_edge_anchor(
                anchor,
                inward,
                min_dist=min_dist,
                board_bounds=bounds,
                epsilon_ratio=eps,
                target_angle_rad=theta,
                max_propositions=max_props,
                use_tight_packing=propose_cfg.guidance_use_tight_packing,
                use_corner_alignment=propose_cfg.guidance_use_corner_alignment,
                enable_grid_exploration=propose_cfg.guidance_enable_grid,
                diversity_dist_ratio=propose_cfg.guidance_diversity_dist_ratio,
            )
            placed = scene.placed_at((x, y, theta))
            g = scene.guidance(placed, (x, y), edge_cfg)
            best_coords: tuple[float, float, float] | None = None
            best_cost = base_cost
            for candidate in _border_refine_candidates(
                x, y, theta, g, min_dist=min_dist, max_props=max_props,
            ):
                cand_poly = transform_poly(part_poly, candidate)
                cand_geom = bases[gid].apply_transform(np.asarray(candidate, dtype=np.float64))
                if not placement_ok_for_outline(
                    scene,
                    cand_geom,
                    cand_poly,
                    outline,
                    others_polys,
                    min_dist,
                    edge_cfg,
                    epsilon_ratio=eps,
                    require_outline_kiss=True,
                ):
                    continue
                trial_polys = list(polys)
                trial_polys[idx] = cand_poly
                cost = _border_tightness_cost(trial_polys, outline, min_dist)
                if cost + 1e-9 < best_cost:
                    best_coords = candidate
                    best_cost = cost
            if best_coords is None:
                continue
            mx, my, mtheta = best_coords
            walked = _border_refine_micro_walk(
                mx, my, mtheta, g,
                scene=scene,
                part_geom=bases[gid],
                part_poly=part_poly,
                others_polys=others_polys,
                outline=outline,
                edge_cfg=edge_cfg,
                min_dist=min_dist,
                eps=eps,
            )
            walk_poly = transform_poly(part_poly, walked)
            walk_geom = bases[gid].apply_transform(
                np.asarray(walked, dtype=np.float64),
            )
            if placement_ok_for_outline(
                scene,
                walk_geom,
                walk_poly,
                outline,
                others_polys,
                min_dist,
                edge_cfg,
                epsilon_ratio=eps,
                require_outline_kiss=True,
            ):
                trial_polys = list(polys)
                trial_polys[idx] = walk_poly
                walk_cost = _border_tightness_cost(
                    trial_polys, outline, min_dist,
                )
                if walk_cost + 1e-9 < best_cost:
                    best_coords = walked
                    best_cost = walk_cost
            polys[idx] = transform_poly(part_poly, best_coords)
            trs[idx] = np.asarray(best_coords, dtype=np.float64)
            geoms[idx] = bases[gid].apply_transform(trs[idx])
            base_cost = best_cost
            improved = True
        if not improved:
            break

    return polys, gids, trs


def _border_pack_graph(
    pack_polys: list,
    pack_gids: list[int],
    pack_tr: list[np.ndarray],
    *,
    board_geom: Geometry | None = None,
) -> tuple[ElemGraph, list, list[int], list[np.ndarray], list[int]]:
    if board_geom is not None:
        kept_polys: list = []
        kept_gids: list[int] = []
        kept_tr: list[np.ndarray] = []
        for poly, gid, tr in zip(pack_polys, pack_gids, pack_tr, strict=True):
            geom = Geometry.from_shapely(poly)
            if placement_footprint_inside_board(geom, board_geom):
                kept_polys.append(poly)
                kept_gids.append(gid)
                kept_tr.append(tr)
        pack_polys, pack_gids, pack_tr = kept_polys, kept_gids, kept_tr
    placed_geoms = [Geometry.from_shapely(p) for p in pack_polys]
    graph = ElemGraph()
    graph.reserve_elems(len(placed_geoms))
    for n, geom in enumerate(placed_geoms):
        cx, cy = geom.center()
        graph.append_elem(
            pack_gids[n],
            Vec2(x=cx, y=cy),
            Circle.from_center_radius(cx, cy, geom.radius()),
        )
    hits = find_polygon_intersections(placed_geoms)
    for i, j in hits:
        graph.add_collision(i, j)
    selected_out = list(range(len(pack_polys)))
    assert selection_is_independent(graph, selected_out)
    return graph, pack_polys, pack_gids, pack_tr, selected_out


def _border_augment_coords(
    cfg: BuildGraphConfig,
    board: BaseGeometry,
    part_poly: Polygon,
    placed_polys: list,
    *,
    min_dist: float,
) -> list[tuple[float, float, float]]:
    """Fast outline candidates: board snap + chain fit against full packed union."""
    from nest_graph.propose.placements_edge import (
        propose_placements_board_edge,
        propose_placements_group_fit,
    )
    from nest_graph.propose.placements_primary import (
        propose_placements_neighbor_slide,
    )
    from nest_graph.propose.context import border_focal_for_propose, propose_push_point
    from nest_graph.propose.geometry import ProposeGeometry

    sheet, _ = board_context_from_geometry(board)
    obstacle = unary_union(placed_polys) if placed_polys else Polygon()
    propose_cfg = cfg.first_pass_propose_config()
    push = propose_push_point(
        board,
        obstacle,
        smart_push=propose_cfg.smart_push_target,
        min_dist=min_dist,
        use_border_focus=True,
    )
    geom = ProposeGeometry(
        board,
        obstacle,
        part_poly,
        min_dist,
        epsilon_ratio=propose_cfg.placement_clearance_epsilon_ratio,
        propose_cfg=propose_cfg,
    )
    coords: list[tuple[float, float, float]] = []
    coords.extend(
        propose_placements_board_edge(
            part_poly,
            sheet,
            obstacle,
            min_dist=min_dist,
            propose_cfg=propose_cfg,
            propose_geom=geom,
            pt_push=push,
            top_n=propose_cfg.first_pass_max_proposals,
            samples_per_edge=propose_cfg.board_edge_samples_per_edge,
        ),
    )
    if not obstacle.is_empty:
        focal = unary_union([border_focal_for_propose(board, min_dist), obstacle])
        coords.extend(
            propose_placements_group_fit(
                focal,
                part_poly,
                sheet,
                obstacle,
                min_dist=min_dist,
                top_n=propose_cfg.first_pass_max_proposals,
                samples_per_edge=propose_cfg.first_pass_group_edge_samples_per_edge,
                propose_geom=geom,
                pt_push=push,
            ),
        )
        coords.extend(
            propose_placements_neighbor_slide(
                obstacle,
                part_poly,
                sheet,
                min_dist,
                propose_geom=geom,
                pt_push=push,
                num_angles=propose_cfg.placement_num_angles,
                top_n=propose_cfg.first_pass_max_proposals,
            ),
        )
    seen: set[tuple[float, float, float]] = set()
    out: list[tuple[float, float, float]] = []
    for c in coords:
        key = (round(c[0], 3), round(c[1], 3), round(c[2], 2))
        if key in seen:
            continue
        seen.add(key)
        out.append(c)
    return out


def _first_pass_interior_fill(
    cfg: BuildGraphConfig,
    board: BaseGeometry,
    parts: list[tuple[Polygon, int]],
    *,
    pack_polys: list,
    pack_gids: list[int],
    pack_tr: list[np.ndarray],
    placed_geoms: list[Geometry],
    sheet: Polygon,
    board_geom: Geometry,
    voids: list,
    min_dist: float,
    eps: float,
    guidance_cfg: GuidanceConfig,
    part_by_gid: dict[int, Polygon],
    bases: dict[int, Geometry],
) -> tuple[list, list[int], list[np.ndarray], list[Geometry]]:
    """Add up to first_pass_interior_max non-outline parts after border augment."""
    interior_max = max(cfg.propose.first_pass_interior_max, 0)
    if interior_max <= 0:
        return pack_polys, pack_gids, pack_tr, placed_geoms

    propose_cfg = cfg.first_pass_propose_config().model_copy(deep=True)
    propose_cfg.ranking_mode = "contact_hybrid"
    propose_cfg.use_contact_ranking = True
    propose_cfg.place_profiles_enabled = True

    polys = list(pack_polys)
    gids = list(pack_gids)
    trs = list(pack_tr)
    geoms = list(placed_geoms)
    catalog = [(part_by_gid[gid], gid) for gid in part_by_gid]

    for _ in range(interior_max):
        densify_stats: dict = {}
        pocket_keys: dict = {}
        by_group = proposed_transforms_for_groups(
            board,
            catalog,
            polys,
            list(range(len(polys))),
            propose_cfg,
            min_dist=min_dist,
            use_full_packed_obstacle=True,
            packed_group_ids=gids,
            packed_transforms=trs,
            user_holes=cfg.rules.board_holes,
            pocket_keys_out=pocket_keys,
            densify_stats_out=densify_stats,
            seeded=False,
        )
        pack_union = unary_union(polys) if polys else Polygon()
        push = propose_push_point(
            board,
            pack_union,
            smart_push=propose_cfg.smart_push_target,
            min_dist=min_dist,
            use_border_focus=False,
        )
        candidates: list[tuple[float, int, np.ndarray, Polygon, Geometry]] = []
        for gid, part_poly in part_by_gid.items():
            arr = by_group.get(gid)
            if arr is None or arr.shape[0] == 0:
                continue
            for row in arr:
                coords = np.asarray(row, dtype=np.float64).reshape(3)
                shapely_placed = transform_poly(part_poly, coords)
                geom = bases[gid].apply_transform(coords)
                scene = placement_scene_for_part(
                    sheet, board_geom, voids, bases[gid], base_geoms=geoms,
                )
                if not placement_ok_for_outline(
                    scene,
                    geom,
                    shapely_placed,
                    board,
                    polys,
                    min_dist,
                    guidance_cfg,
                    epsilon_ratio=eps,
                    require_outline_kiss=False,
                ):
                    continue
                pg = ProposeGeometry(
                    board, pack_union, part_poly, min_dist,
                    epsilon_ratio=eps, propose_cfg=propose_cfg,
                )
                tight = score_placement_tightness(
                    (float(coords[0]), float(coords[1]), float(coords[2])),
                    pg, push, min_dist,
                )
                cost = -tight if tight > float("-inf") else float("inf")
                candidates.append((cost, gid, coords, shapely_placed, geom))
        if not candidates:
            break
        candidates.sort(key=lambda row: row[0])
        added: list[tuple[int, np.ndarray, Polygon, Geometry]] = []
        for cost, gid, coords, shapely_placed, geom in candidates:
            blockers = polys + [row[2] for row in added]
            blocker_geoms = [Geometry.from_shapely(p) for p in blockers]
            if not clear_of_geoms(geom, blocker_geoms, min_dist):
                continue
            if not placement_footprint_inside_board(geom, board_geom):
                continue
            added.append((gid, coords, shapely_placed, geom))
            break
        if not added:
            break
        for gid, coords, shapely_placed, geom in added:
            polys.append(shapely_placed)
            gids.append(gid)
            trs.append(coords)
            geoms.append(geom)

    return polys, gids, trs, geoms


def _sequential_border_augment(
    cfg: BuildGraphConfig,
    board: BaseGeometry,
    parts: list[tuple[Polygon, int]],
    *,
    outline: BaseGeometry,
    polys: list,
    group_id: list[int],
    transform: list[np.ndarray],
    selected: list[int],
    skip_guidance_refine: bool = False,
) -> tuple[ElemGraph, list, list[int], list[np.ndarray], list[int]]:
    """Fill outline gaps by proposing against the full packed union each step."""
    min_dist = cfg.board_min_dist(first_pass=True)
    eps = cfg.placement_epsilon_ratio(first_pass=True)
    sheet, voids = board_context_from_geometry(board)
    board_geom = Geometry.from_shapely(sheet)
    pad = default_sheet_padding(board)
    guidance_cfg = guidance_config_for_graph(
        min_dist,
        board_bounds=padded_board_bounds(board, pad),
        epsilon_ratio=eps,
    )
    part_by_gid = {gid: poly for poly, gid in parts}
    bases = {gid: Geometry.from_shapely(part_by_gid[gid]) for gid in part_by_gid}

    pack_polys = [polys[i] for i in selected]
    pack_gids = [group_id[i] for i in selected]
    pack_tr = [transform[i] for i in selected]
    placed_geoms = [Geometry.from_shapely(p) for p in pack_polys]
    max_rounds = max(cfg.propose.first_pass_sequential_augment_max, 0)

    for _ in range(max_rounds):
        candidates: list[tuple[float, int, np.ndarray, Polygon, Geometry]] = []
        for gid, part_poly in part_by_gid.items():
            for c in _border_augment_coords(
                cfg, board, part_poly, pack_polys, min_dist=min_dist,
            ):
                coords = np.asarray(c, dtype=np.float64)
                shapely_placed = transform_poly(part_poly, coords)
                geom = bases[gid].apply_transform(coords)
                scene = placement_scene_for_part(
                    sheet, board_geom, voids, bases[gid], base_geoms=placed_geoms,
                )
                if not placement_ok_for_outline(
                    scene,
                    geom,
                    shapely_placed,
                    outline,
                    pack_polys,
                    min_dist,
                    guidance_cfg,
                    epsilon_ratio=eps,
                    require_outline_kiss=True,
                ):
                    continue
                cost = abs(_outline_standoff_distance(shapely_placed, outline) - min_dist)
                candidates.append((cost, gid, coords, shapely_placed, geom))
        if not candidates:
            break
        candidates.sort(key=lambda row: row[0])
        added: list[tuple[int, np.ndarray, Polygon, Geometry]] = []
        for cost, gid, coords, shapely_placed, geom in candidates:
            blockers = pack_polys + [row[2] for row in added]
            blocker_geoms = [Geometry.from_shapely(p) for p in blockers]
            if not clear_of_geoms(geom, blocker_geoms, min_dist):
                continue
            if not placement_footprint_inside_board(geom, board_geom):
                continue
            added.append((gid, coords, shapely_placed, geom))
        if not added:
            break
        for gid, coords, shapely_placed, geom in added:
            pack_polys.append(shapely_placed)
            pack_gids.append(gid)
            pack_tr.append(coords)
            placed_geoms.append(geom)

    pack_polys, pack_gids, pack_tr, placed_geoms = _first_pass_interior_fill(
        cfg,
        board,
        parts,
        pack_polys=pack_polys,
        pack_gids=pack_gids,
        pack_tr=pack_tr,
        placed_geoms=placed_geoms,
        sheet=sheet,
        board_geom=board_geom,
        voids=voids,
        min_dist=min_dist,
        eps=eps,
        guidance_cfg=guidance_cfg,
        part_by_gid=part_by_gid,
        bases=bases,
    )

    if not skip_guidance_refine:
        pack_polys, pack_gids, pack_tr = _guidance_border_refine(
            cfg,
            board,
            parts,
            outline=outline,
            pack_polys=pack_polys,
            pack_gids=pack_gids,
            pack_tr=pack_tr,
        )
    return _border_pack_graph(pack_polys, pack_gids, pack_tr, board_geom=board_geom)


def _first_pass_layered_selection(
    cfg: BuildGraphConfig,
    board: BaseGeometry,
    parts: list[tuple[Polygon, int]],
    *,
    graph: ElemGraph,
    p1: Polygon,
    p2: Polygon,
    selected_t: tuple[np.ndarray, np.ndarray],
    history: tuple[np.ndarray, np.ndarray],
    rng: np.random.Generator,
    selection_window: list[tuple[np.ndarray, np.ndarray]] | None,
    polys: list,
    group_id: list[int],
    transform: list[np.ndarray],
    phase1_selected: list[int],
    rule_set: PlacementRuleSet,
    scores: list[float],
    selection: SelectionConfig,
    skip_guidance_refine: bool = False,
) -> tuple[ElemGraph, list, list[int], list[np.ndarray], list[int]]:
    """Rebuild with packed obstacles; saturate outline-kiss placements along the perimeter."""
    min_dist = cfg.board_min_dist(first_pass=True)
    outline = board
    sheet, _ = board_context_from_geometry(board)
    board_geom = Geometry.from_shapely(sheet)
    polys_cur = polys
    group_id_cur = group_id
    transform_cur = transform
    sel_cur = list(phase1_selected)
    passes = max(cfg.propose.first_pass_border_saturation_passes, 0)

    for _ in range(passes):
        phase1_by_group: list[list[np.ndarray]] = [[], []]
        for idx in sel_cur:
            phase1_by_group[group_id_cur[idx]].append(transform_cur[idx])
        phase1_t = tuple(
            np.asarray(rows, dtype=np.float64) if rows else np.zeros((0, 3))
            for rows in phase1_by_group
        )
        nest_state = NestState(
            polys=polys_cur,
            group_id=group_id_cur,
            transform=transform_cur,
            selected_indices=list(sel_cur),
        )
        batch2 = _border_saturation_transform_batch(
            cfg,
            board,
            parts,
            nest_state,
        )
        combined = (
            _prepend_group_transforms(phase1_t[0], batch2[0]),
            _prepend_group_transforms(phase1_t[1], batch2[1]),
        )
        graph2, polys2, group_id2, transform2 = make_polygon_graph(
            board,
            [(p1, combined[0]), (p2, combined[1])],
            min_dist=min_dist,
            epsilon_ratio=cfg.placement_epsilon_ratio(first_pass=True),
        )
        locked = _locked_graph_indices(
            transform2,
            [transform_cur[i] for i in sel_cur],
        )
        if not locked:
            break
        scores2 = score_elems(graph2, rule_set)
        new_sel = _expand_border_selection(
            graph2, polys2, outline, min_dist, scores2, locked,
        )
        polys_cur = polys2
        group_id_cur = group_id2
        transform_cur = transform2
        if len(new_sel) <= len(sel_cur):
            sel_cur = new_sel
            break
        sel_cur = new_sel

    if cfg.propose.first_pass_sequential_augment_max > 0:
        return _sequential_border_augment(
            cfg,
            board,
            parts,
            outline=outline,
            polys=polys_cur,
            group_id=group_id_cur,
            transform=transform_cur,
            selected=sel_cur,
            skip_guidance_refine=skip_guidance_refine,
        )

    pack_polys = [polys_cur[i] for i in sel_cur]
    pack_gids = [group_id_cur[i] for i in sel_cur]
    pack_tr = [transform_cur[i] for i in sel_cur]
    if not skip_guidance_refine:
        pack_polys, pack_gids, pack_tr = _guidance_border_refine(
            cfg,
            board,
            parts,
            outline=outline,
            pack_polys=pack_polys,
            pack_gids=pack_gids,
            pack_tr=pack_tr,
        )
    return _border_pack_graph(pack_polys, pack_gids, pack_tr, board_geom=board_geom)


def _boost_border_scores(
    polys: list,
    scores: list[float],
    outline: BaseGeometry,
    min_dist: float,
    *,
    weight: float = 8.0,
) -> None:
    """Favor graph nodes flush to the nest outline when scoring nest/DFS."""
    scale = _border_kiss_tolerance(min_dist)
    for i, sc in enumerate(scores):
        err = abs(_outline_standoff_distance(polys[i], outline) - min_dist)
        scores[i] = sc + weight * max(0.0, 1.0 - err / scale)


def _boost_void_island_scores(
    polys: list,
    scores: list[float],
    free_poly: BaseGeometry | None,
    *,
    weight: float,
    pole: Point | None = None,
    pole_radius: float = 0.0,
    sheet_diag: float = 0.0,
) -> int:
    """EMS-style distance-to-pole boost for Python DFS/finalize scores.

    ``weight`` scales ``max(0, 1 - dist/pole_norm)`` for nodes in the free
    poly (or within ``pole_radius`` of the pole). Flat legacy boost when
    ``sheet_diag <= 0`` and pole is unset beyond the in_free check.
    """
    if weight <= 0.0 or not scores:
        return 0
    diag = float(sheet_diag)
    if diag <= 1e-12 and free_poly is not None and not free_poly.is_empty:
        minx, miny, maxx, maxy = free_poly.bounds
        diag = float(np.hypot(maxx - minx, maxy - miny))
    if diag <= 1e-12 and pole is not None and pole_radius > 0.0:
        diag = float(pole_radius) * 4.0
    if diag <= 1e-12:
        diag = 1.0

    n = 0
    for i, sc in enumerate(scores):
        if i >= len(polys):
            break
        poly = polys[i]
        if poly is None or poly.is_empty:
            continue
        c = poly.centroid
        in_free = (
            free_poly is not None
            and not free_poly.is_empty
            and (free_poly.contains(c) or free_poly.intersects(c))
        )
        near_pole = (
            pole is not None
            and pole_radius > 0.0
            and float(c.distance(pole)) <= float(pole_radius)
        )
        if not (in_free or near_pole):
            continue
        if pole is not None:
            dist = float(c.distance(pole))
            factor = max(0.0, 1.0 - dist / diag)
        else:
            factor = 1.0
        scores[i] = float(sc) + float(weight) * factor
        n += 1
    return n


def _boost_keyed_proposal_scores(
    group_id: Sequence[int],
    transforms: Sequence,
    scores: list[float],
    keys_by_group: dict[int, set[tuple[float, float, float]]] | None,
    *,
    weight: float,
) -> int:
    """Add ``weight`` to scores whose (group, transform key) is in ``keys_by_group``."""
    if weight <= 0.0 or not scores or not keys_by_group:
        return 0
    n = 0
    for i, sc in enumerate(scores):
        if i >= len(group_id) or i >= len(transforms):
            break
        gid = int(group_id[i])
        keys = keys_by_group.get(gid)
        if not keys:
            continue
        key = _transform_row_key(np.asarray(transforms[i], dtype=np.float64))
        if key not in keys:
            continue
        scores[i] = float(sc) + float(weight)
        n += 1
    return n


def _boost_small_part_scores(
    group_id: Sequence[int],
    scores: list[float],
    part_areas: Sequence[float],
    *,
    weight: float,
) -> int:
    """Boost smaller catalog groups: weight * (1 - area/max_area) on large_void."""
    if weight <= 0.0 or not scores or not part_areas:
        return 0
    areas = [float(a) for a in part_areas]
    max_a = max(areas) if areas else 0.0
    if max_a <= 1e-12:
        return 0
    n = 0
    for i, sc in enumerate(scores):
        if i >= len(group_id):
            break
        gid = int(group_id[i])
        if gid < 0 or gid >= len(areas):
            continue
        factor = max(0.0, 1.0 - areas[gid] / max_a)
        if factor <= 0.0:
            continue
        scores[i] = float(sc) + float(weight) * factor
        n += 1
    return n


def apply_void_selection_boosts(
    *,
    polys: list,
    group_id: Sequence[int],
    transform: Sequence,
    scores: list[float],
    free_info,
    free_poly,
    part_areas: Sequence[float],
    propose_stats: dict | None,
    cfg: "BuildGraphConfig",
    sheet_diag: float,
    void_r: float,
) -> dict[str, int]:
    """Apply void-island, pocket-key, and small-part score boosts (demo + evaluator)."""
    hits = {
        "void_island": 0,
        "pocket_keys": 0,
        "small_part": 0,
    }
    pole_w = float(cfg.propose.void_island_score_boost)
    pocket_w = float(getattr(cfg.propose, "pocket_score_boost", 0.0) or 0.0)
    small_w = float(getattr(cfg.propose, "small_part_void_score_boost", 0.0) or 0.0)
    if free_info.kind == "large_void" and pole_w > 0.0:
        hits["void_island"] = _boost_void_island_scores(
            polys,
            scores,
            free_poly,
            weight=pole_w,
            pole=free_info.target_pt,
            pole_radius=void_r,
            sheet_diag=sheet_diag,
        )
    if pocket_w > 0.0 and propose_stats is not None:
        keys = propose_stats.get("pocket_keys") or {}
        hits["pocket_keys"] = _boost_keyed_proposal_scores(
            group_id,
            transform,
            scores,
            keys,
            weight=pocket_w,
        )
    if free_info.kind == "large_void" and small_w > 0.0:
        hits["small_part"] = _boost_small_part_scores(
            group_id,
            scores,
            part_areas,
            weight=small_w,
        )
    return hits


def _make_void_attractor_rule_set(
    pole: Point,
    *,
    ngroups: int,
    radius: float,
    weight: float,
) -> PlacementRuleSet:
    """Strong PointPlaceRule attractors at the free-space pole for nest_by_graph."""
    rs = PlacementRuleSet()
    px, py = float(pole.x), float(pole.y)
    r = max(float(radius), 1e-4)
    w = float(weight)
    for g in range(max(int(ngroups), 1)):
        rs.append_rule(PointPlaceRule(pos=Vec2(x=px, y=py), r=r, w=w, group=g))
    return rs


def _merge_void_attractor_into_rule_sets(
    rule_sets: list[PlacementRuleSet],
    attractor: PlacementRuleSet,
    *,
    nest_rule_sets_used: int,
    max_rules_per_set: int,
) -> list[PlacementRuleSet]:
    """Prepend void attractors onto the rule sets used by nest_by_graph."""
    if not rule_sets or attractor is None:
        return rule_sets
    n_touch = min(max(int(nest_rule_sets_used), 1), len(rule_sets))
    out: list[PlacementRuleSet] = []
    for i, rs in enumerate(rule_sets):
        if i >= n_touch:
            out.append(rs)
            continue
        merged = _copy_rule_set(attractor)
        for pr in rs.point_rules:
            merged.append_rule(pr)
        for cr in rs.circle_rules:
            merged.append_rule(cr)
        for pr in rs.point_angle_rules:
            merged.append_rule(pr)
        for cr in rs.circle_angle_rules:
            merged.append_rule(cr)
        out.append(truncate_rule_set(merged, max_rules_per_set))
    return out


def _centroid_in_free(poly, free_poly: BaseGeometry | None) -> bool:
    if free_poly is None or free_poly.is_empty or poly is None or poly.is_empty:
        return False
    c = poly.centroid
    return bool(free_poly.contains(c) or free_poly.intersects(c))


def _xy_in_free(x: float, y: float, free_poly: BaseGeometry | None) -> bool:
    if free_poly is None or free_poly.is_empty:
        return False
    p = Point(float(x), float(y))
    return bool(free_poly.contains(p) or free_poly.intersects(p))


def _count_selected_in_free(
    polys: list,
    selected: Sequence[int],
    free_poly: BaseGeometry | None,
) -> int:
    return sum(
        1 for i in selected if _centroid_in_free(polys[i], free_poly)
    )


def _proposer_key_owner(
    proposer_keys: dict[str, set[tuple[float, float, float]]] | None,
) -> dict[tuple[float, float, float], str]:
    """Map transform key → first-writer proposer name."""
    owner: dict[tuple[float, float, float], str] = {}
    if not proposer_keys:
        return owner
    for name, keys in proposer_keys.items():
        for key in keys:
            if key not in owner:
                owner[key] = name
    return owner


def _count_selected_by_proposer(
    transforms: Sequence,
    selected: Sequence[int],
    proposer_keys: dict[str, set[tuple[float, float, float]]] | None,
) -> dict[str, int]:
    owner = _proposer_key_owner(proposer_keys)
    counts: dict[str, int] = {}
    for i in selected:
        if i < 0 or i >= len(transforms):
            continue
        key = _transform_row_key(np.asarray(transforms[i], dtype=np.float64))
        name = owner.get(key)
        if name is None:
            continue
        counts[name] = counts.get(name, 0) + 1
    return counts


def _format_prop_accept(
    emitted: dict[str, int],
    pool: dict[str, int],
    nest: dict[str, int],
    refine: dict[str, int],
    *,
    limit: int = 8,
) -> str:
    names = sorted(
        set(emitted) | set(pool) | set(nest) | set(refine),
        key=lambda n: (-int(emitted.get(n, 0)), n),
    )
    parts = []
    for name in names[:limit]:
        parts.append(
            f"{name}:e{int(emitted.get(name, 0))}/p{int(pool.get(name, 0))}/"
            f"n{int(nest.get(name, 0))}/r{int(refine.get(name, 0))}"
        )
    return " ".join(parts)


def _pin_nest_void_independent(
    graph: ElemGraph,
    selected_nest: Sequence[int],
    selected_refine: Sequence[int],
    polys: list,
    free_poly: BaseGeometry | None,
    scores: list[float] | None = None,
    *,
    stats_out: dict | None = None,
) -> list[int]:
    """P3: re-add nest-void nodes missing from refine if collision-clear.

    Uses ``graph.collisions`` (not Shapely pose-clear). Optional ``stats_out``
    records pin_candidates / pin_added / pin_blocked_collision / pin_ms.
    """
    t0 = time.perf_counter()
    refine = list(selected_refine)
    refine_set = set(refine)
    candidates = [
        i for i in selected_nest
        if i not in refine_set and _centroid_in_free(polys[i], free_poly)
    ]
    pin_added = 0
    pin_blocked = 0
    if not candidates:
        if stats_out is not None:
            stats_out["pin_candidates"] = 0
            stats_out["pin_added"] = 0
            stats_out["pin_blocked_collision"] = 0
            stats_out["pin_ms"] = (time.perf_counter() - t0) * 1000.0
        return refine
    if scores is not None and len(scores) == len(graph.group_id):
        candidates.sort(key=lambda v: scores[v], reverse=True)
    for v in candidates:
        if any(u in refine_set for u in graph.collisions[v]):
            pin_blocked += 1
            continue
        refine.append(v)
        refine_set.add(v)
        pin_added += 1
    if stats_out is not None:
        stats_out["pin_candidates"] = len(candidates)
        stats_out["pin_added"] = pin_added
        stats_out["pin_blocked_collision"] = pin_blocked
        stats_out["pin_ms"] = (time.perf_counter() - t0) * 1000.0
    return refine


def _count_graph_in_free(polys: list, free_poly: BaseGeometry | None) -> int:
    return sum(1 for p in polys if _centroid_in_free(p, free_poly))


def _count_props_in_free(
    proposed_by_group: Sequence[np.ndarray] | None,
    free_poly: BaseGeometry | None,
) -> int:
    if not proposed_by_group or free_poly is None or free_poly.is_empty:
        return 0
    n = 0
    for arr in proposed_by_group:
        if arr is None or len(arr) == 0:
            continue
        for row in np.asarray(arr, dtype=np.float64).reshape(-1, 3):
            if _xy_in_free(float(row[0]), float(row[1]), free_poly):
                n += 1
    return n


def _count_props_near_pole(
    proposed_by_group: Sequence[np.ndarray] | None,
    pole: Point | None,
    radius: float,
) -> int:
    """Count proposals whose (x,y) lies within ``radius`` of the void pole."""
    if (
        not proposed_by_group
        or pole is None
        or float(radius) <= 0.0
    ):
        return 0
    r = float(radius)
    n = 0
    for arr in proposed_by_group:
        if arr is None or len(arr) == 0:
            continue
        for row in np.asarray(arr, dtype=np.float64).reshape(-1, 3):
            if float(np.hypot(float(row[0]) - float(pole.x), float(row[1]) - float(pole.y))) <= r:
                n += 1
    return n


def _zones_have_void_hijack(zones: Sequence[str] | None) -> bool:
    if not zones:
        return False
    return any("void_seek(large_void)" in str(z) for z in zones)


def _project_angles_to_allowed(
    transforms: np.ndarray,
    allowed: Sequence[float],
) -> np.ndarray:
    """Snap angle column to nearest allowed grain angle (mod 2π)."""
    if transforms.size == 0 or not allowed:
        return transforms
    out = np.asarray(transforms, dtype=np.float64).copy()
    allowed_arr = np.asarray(allowed, dtype=np.float64)
    two_pi = 2.0 * np.pi
    angles = np.mod(out[:, 2], two_pi)
    allowed_mod = np.mod(allowed_arr, two_pi)
    # Circular distance to each allowed angle
    diffs = angles[:, None] - allowed_mod[None, :]
    diffs = (diffs + np.pi) % two_pi - np.pi
    nearest = np.argmin(np.abs(diffs), axis=1)
    out[:, 2] = allowed_mod[nearest]
    return out


def _build_transform_batch(
    cfg: BuildGraphConfig,
    selected_t: tuple[np.ndarray, ...],
    history: tuple[np.ndarray, ...],
    rng: np.random.Generator,
    *,
    board: BaseGeometry | None = None,
    parts: list[tuple[Polygon, int]] | None = None,
    nest_state: NestState | None = None,
    selection_window: list[tuple[np.ndarray, ...]] | None = None,
    first_pass: bool = False,
    border_saturation: bool = False,
    rules: PlacementRuleSet | None = None,
    proposer_counts_out: dict[str, int] | None = None,
    propose_stats_out: dict | None = None,
    propose_feedback=None,
    group_allowed_angles: Sequence[tuple[float, ...] | None] | tuple = (),
) -> tuple[np.ndarray, ...]:
    sc = cfg.sampling
    scale = sc.transform_scale
    propose_by_group: dict[int, np.ndarray] = {}
    border_pin_by_group: dict[int, np.ndarray] = {}
    empty_sheet = (
        nest_state is None
        or not nest_state.selected_indices
    )
    if (
        board is not None
        and parts is not None
        and cfg.propose.max_proposals > 0
    ):
        polys = nest_state.polys if nest_state is not None else []
        selected = nest_state.selected_indices if nest_state is not None else []
        min_dist = cfg.board_min_dist_for(board, first_pass=first_pass)
        propose_cfg = (
            cfg.first_pass_propose_config() if first_pass else cfg.propose
        )
        seeded = bool(nest_state is not None and nest_state.seed_count > 0)
        empty_border_only = (
            empty_sheet and cfg.propose.first_pass_empty_border_only
        )
        if empty_border_only and parts and sheet_has_narrow_corridor(
            board, parts[0][0], min_dist,
        ):
            empty_border_only = False
        border_only = (
            empty_border_only
            or (border_saturation and cfg.propose.first_pass_border_pack)
        )
        zones_used: list[str] = []
        pocket_keys_raw: dict[int, set[tuple[float, float, float]]] = {}
        densify_stats: dict = {}
        propose_by_group = proposed_transforms_for_groups(
            board,
            parts,
            polys,
            selected,
            propose_cfg,
            min_dist=min_dist,
            border_only_propose=border_only,
            use_full_packed_obstacle=(
                cfg.propose.use_full_packed_obstacle and not empty_sheet
            ),
            rules=rules,
            proposer_counts_out=proposer_counts_out,
            zones_used_out=zones_used if propose_stats_out is not None else None,
            propose_feedback=propose_feedback,
            packed_group_ids=(
                nest_state.group_id if nest_state is not None else None
            ),
            packed_transforms=(
                nest_state.transform if nest_state is not None else None
            ),
            group_allowed_angles=group_allowed_angles,
            user_holes=cfg.rules.board_holes,
            seeded=seeded,
            pocket_keys_out=pocket_keys_raw,
            densify_stats_out=densify_stats,
        )
        # Project angles before keying so MIS boost matches graph transforms.
        proposal_keys: dict[int, set[tuple[float, float, float]]] = {}
        pocket_keys: dict[int, set[tuple[float, float, float]]] = {}
        for gid, arr in propose_by_group.items():
            projected = arr
            if group_allowed_angles and gid < len(group_allowed_angles):
                allowed = group_allowed_angles[gid]
                if allowed is not None:
                    projected = _project_angles_to_allowed(arr, allowed)
            proposal_keys[gid] = {_transform_row_key(r) for r in projected}
            raw = pocket_keys_raw.get(gid) or set()
            if raw and group_allowed_angles and gid < len(group_allowed_angles):
                allowed = group_allowed_angles[gid]
                if allowed is not None and len(raw) > 0:
                    raw_arr = np.asarray(list(raw), dtype=np.float64)
                    proj = _project_angles_to_allowed(raw_arr, allowed)
                    pocket_keys[gid] = {_transform_row_key(r) for r in proj}
                else:
                    pocket_keys[gid] = set(raw)
            else:
                pocket_keys[gid] = set(raw)
        if propose_stats_out is not None:
            propose_stats_out["proposal_count"] = sum(
                arr.shape[0] for arr in propose_by_group.values()
            )
            propose_stats_out["zones_used"] = zones_used
            propose_stats_out["proposal_keys"] = proposal_keys
            propose_stats_out["pocket_keys"] = pocket_keys
            propose_stats_out["densify_stats"] = dict(densify_stats)
            propose_stats_out["proposed_by_group"] = {
                gid: np.asarray(arr, dtype=np.float64)
                for gid, arr in propose_by_group.items()
            }
            propose_stats_out["border_only"] = bool(border_only)
            propose_stats_out["proposer_keys"] = densify_stats.get("proposer_keys") or {}
            propose_stats_out["emitted_by_proposer"] = densify_stats.get(
                "emitted_by_proposer"
            ) or {}
            propose_stats_out["pool_by_proposer"] = densify_stats.get(
                "pool_by_proposer"
            ) or {}
            propose_stats_out["pocket_by_tag"] = densify_stats.get("pocket_by_tag") or {}
            propose_stats_out["cascade_stopped_after"] = densify_stats.get(
                "cascade_stopped_after", "none"
            )
            propose_stats_out["cascade_skipped_proposers"] = list(
                densify_stats.get("cascade_skipped_proposers") or []
            )
            propose_stats_out["nms_kept"] = int(densify_stats.get("nms_kept", 0))
            propose_stats_out["nms_dropped"] = int(densify_stats.get("nms_dropped", 0))
            propose_stats_out["conflict_penalty_applied"] = int(
                densify_stats.get("conflict_penalty_applied", 0)
            )
        if empty_sheet and cfg.propose.use_board_edge_seeds:
            for part_poly, group_id in parts:
                border_pin_by_group[group_id] = border_edge_transforms_for_group(
                    board,
                    part_poly,
                    Polygon(),
                    propose_cfg,
                    min_dist=min_dist,
                )

    window_t = window_selected_transforms(
        selection_window, ngroups=max(len(selected_t), 2),
    )

    def one_group(
        group_id: int,
        sel: np.ndarray,
        hist: np.ndarray,
        window: np.ndarray,
    ) -> np.ndarray:
        batch_parts: list[np.ndarray] = []
        pinned = border_pin_by_group.get(group_id, np.zeros((0, 3)))
        proposed = propose_by_group.get(group_id, np.zeros((0, 3)))
        border_batch = (
            empty_sheet
            and cfg.propose.first_pass_border_pack
            and cfg.propose.first_pass_empty_border_only
        )
        if border_batch:
            if pinned.shape[0] > 0:
                batch_parts.append(pinned)
                jitter_n = sc.structured_jitter_per_proposal_empty
                if jitter_n > 0:
                    jittered = expand_structured_transforms(
                        pinned,
                        cfg.propose.structured_jitter_border_scale,
                        jitter_n,
                    )
                    if jittered.shape[0] > 0:
                        batch_parts.append(jittered)
            if proposed.shape[0] > 0:
                batch_parts.append(proposed)
        elif proposed.shape[0] > 0:
            batch_parts.append(proposed)
            if pinned.shape[0] == 0:
                jitter_n = (
                    sc.structured_jitter_per_proposal_empty
                    if empty_sheet
                    else sc.structured_jitter_per_proposal
                )
                jittered = expand_structured_transforms(
                    proposed,
                    sc.structured_jitter_scale,
                    jitter_n,
                )
                if jittered.shape[0] > 0:
                    batch_parts.append(jittered)
        n_random = (
            cfg.propose.random_per_iter_empty_border
            if empty_sheet and cfg.propose.use_border_focus and not border_batch
            else (
                sc.random_per_iter_when_proposed
                if proposed.shape[0] > 0
                else sc.random_per_iter
            )
        )
        # When proposers are sterile on a packed sheet, avoid flooding the graph with
        # colliding history/jitter that only inflate score_rules cost.
        sterile_pack = (
            not empty_sheet
            and proposed.shape[0] == 0
            and not border_batch
        )
        if sterile_pack:
            # Keep exploring near the current selection; drop stale history/shuffles.
            n_random = min(max(sc.random_per_iter_when_proposed, 32), 64)
            batch_parts.append(rng.uniform(-1, 1, (n_random, 3)) * scale)
            if sel.shape[0] > 0:
                batch_parts.append(sel)
                batch_parts.extend(
                    transform_selection(sel, max(sc.selection_expand_n, 2), rng),
                )
        else:
            batch_parts.append(rng.uniform(-1, 1, (n_random, 3)) * scale)
            if hist.shape[0] > 0:
                batch_parts.append(hist)
            if sel.shape[0] > 0:
                batch_parts.append(sel)
                batch_parts.extend(transform_selection(sel, sc.selection_expand_n, rng))
                batch_parts.extend(transform_history(hist, sc.history_expand_n, rng))
            if window.shape[0] > 0:
                batch_parts.append(window)
                batch_parts.extend(transform_selection(window, sc.selection_expand_n, rng))
            if sc.shuffle_passes > 0 and (
                sel.shape[0] > 0 or hist.shape[0] > 0 or window.shape[0] > 0
            ):
                for _ in range(sc.shuffle_passes):
                    batch_parts.append(
                        transform_shuffle_mix(
                            sel, hist, sc.shuffle_per_pass, rng, sc.shuffle_scale,
                        )
                    )
        merged = dedupe_transforms(np.concatenate(batch_parts))
        merged = shuffle_transforms(merged, rng)
        allowed = None
        if group_allowed_angles and group_id < len(group_allowed_angles):
            allowed = group_allowed_angles[group_id]
        if allowed is not None and merged.shape[0] > 0:
            merged = _project_angles_to_allowed(merged, allowed)
            merged = dedupe_transforms(merged)
        pin_budget = min(proposed.shape[0], max(cfg.propose.max_proposals, 1))
        proposal_pins = proposed[:pin_budget] if pin_budget > 0 else np.zeros((0, 3))
        if pinned.shape[0] > 0 or proposal_pins.shape[0] > 0:
            all_pinned = dedupe_transforms(
                np.concatenate([pinned, proposal_pins], axis=0)
            )
            if allowed is not None and all_pinned.shape[0] > 0:
                all_pinned = _project_angles_to_allowed(all_pinned, allowed)
                all_pinned = dedupe_transforms(all_pinned)
        else:
            all_pinned = pinned
        return subsample_transforms_with_pinned(
            merged, all_pinned, sc.max_transforms_per_group, rng,
        )

    out = []
    for i in range(len(selected_t)):
        sel = selected_t[i] if i < len(selected_t) else np.zeros((0, 3))
        hist = history[i] if i < len(history) else np.zeros((0, 3))
        win = window_t[i] if i < len(window_t) else np.zeros((0, 3))
        out.append(one_group(i, sel, hist, win))
    return tuple(out)


def _make_demo_rule_set(cfg: BuildGraphConfig) -> PlacementRuleSet:
    rc = cfg.rules
    r = rc.place_rule_radius
    wrect = rc.weight_rect
    wtri = rc.weight_tri
    aw = rc.angle_rule_weight_scale * wtri
    rule_set = PlacementRuleSet()
    rule_set.append_rule(PointPlaceRule(pos=Vec2(x=0, y=0), r=r, w=wrect, group=0))
    rule_set.append_rule(PointPlaceRule(pos=Vec2(x=0.7, y=0.7), r=r, w=wtri, group=1))
    rule_set.append_rule(PointPlaceRule(pos=Vec2(x=0, y=1.1), r=r, w=wtri, group=1))
    rule_set.append_rule(PointPlaceRule(pos=Vec2(x=1.2, y=0), r=r, w=wtri, group=1))
    for gi, wgt in ((0, wrect), (1, wtri)):
        for k in range(8):
            a = float(2.0 * np.pi * k / 8.0)
            rule_set.append_rule(
                PointAngleRule(
                    pos=Vec2(x=0.6, y=0.55),
                    r=r,
                    a=a,
                    w=aw * wgt,
                    group=gi,
                ),
            )
    return rule_set


def prune_selection_to_independent_set(
    graph: ElemGraph,
    selected: list[int],
    scores: list[float] | None = None,
) -> list[int]:
    """Greedy MIS fallback (prefer finalize_selection for score-optimal drops)."""
    if not selected:
        return []
    order = list(selected)
    if scores is not None and len(scores) == len(graph.group_id):
        order.sort(key=lambda v: scores[v], reverse=True)
    kept: list[int] = []
    kept_set: set[int] = set()
    for v in order:
        if any(u in kept_set for u in graph.collisions[v]):
            continue
        kept.append(v)
        kept_set.add(v)
    return kept


def _refine_options(
    sel: SelectionConfig,
    *,
    loose: bool,
    max_tries: int | None = None,
) -> RefineSelectionOptions:
    opts = RefineSelectionOptions()
    opts.max_tries = sel.dfs_max_tries if max_tries is None else max_tries
    opts.max_passes = sel.dfs_refine_max_passes
    opts.max_stagnant_passes = sel.dfs_refine_max_stagnant_passes
    opts.beam_width = sel.dfs_refine_beam_width
    if loose:
        opts.min_collisions = 2
        opts.max_root_collisions = 2
    else:
        opts.min_collisions = 1
        opts.max_root_collisions = 1
    return opts


def _finalize_options(sel: SelectionConfig) -> FinalizeSelectionOptions:
    opts = FinalizeSelectionOptions()
    opts.repair_passes = sel.dfs_finalize_repair_passes
    opts.max_exact_component_size = sel.dfs_finalize_max_component
    return opts


def _loose_refine_options(sel: SelectionConfig) -> RefineSelectionOptions:
    return _refine_options(sel, loose=True)


def _tight_refine_options(sel: SelectionConfig) -> RefineSelectionOptions:
    return _refine_options(sel, loose=False)


def _strict_refine_options(sel: SelectionConfig) -> RefineSelectionOptions:
    opts = _refine_options(sel, loose=False)
    opts.min_collisions = 0
    opts.max_root_collisions = 0
    return opts


def _head_loose_refine_options(sel: SelectionConfig) -> RefineSelectionOptions:
    """HEAD-style score DFS: allow transient overlaps during search."""
    return _refine_options(sel, loose=True)


def selection_score_sum(scores: list[float], selected: list[int]) -> float:
    return float(sum(scores[v] for v in selected))


def apply_dfs_refinement(
    graph: ElemGraph,
    rule_set: PlacementRuleSet,
    selected: list[int],
    scores: list[float],
    *,
    dfs_passes: int | None = None,
    dfs_max_tries: int | None = None,
    mode: str | None = None,
    selection: SelectionConfig | None = None,
) -> tuple[list[int], list[int], float]:
    """Refine selection; return (pre_finalize, final, score_sum_final)."""
    sel = selection if selection is not None else SelectionConfig()
    passes = dfs_passes if dfs_passes is not None else sel.dfs_passes
    max_tries = dfs_max_tries if dfs_max_tries is not None else sel.dfs_max_tries
    mode = mode if mode is not None else sel.dfs_mode
    finalize_opts = _finalize_options(sel)

    selected = list(selected)
    graph_sorted = sort_graph(graph, rule_set)
    graph_sorted_rev = sort_graph(graph, rule_set, reverse=True)
    pre_finalize = selected

    def _finalize() -> list[int]:
        return list(finalize_selection(graph, selected, scores, finalize_opts))

    if mode == "nest_only":
        return selected, selected, selection_score_sum(scores, selected)

    if mode == "legacy_alternating":
        for _ in range(passes):
            selected = list(increase_selection_dfs(
                graph_sorted_rev, selected, max_tries,
            ))
            selected = list(increase_selection_dfs(graph, selected, max_tries))
            selected = list(increase_score_dfs(graph_sorted_rev, selected, scores))
            selected = list(increase_selection_dfs(
                graph_sorted, selected, max_tries,
            ))
            selected = list(increase_score_dfs(graph_sorted, selected, scores))
        pre_finalize = selected
        final = _finalize()
        return pre_finalize, final, selection_score_sum(scores, final)

    if mode == "head_pipeline":
        loose = _head_loose_refine_options(sel)
        tight = RefineSelectionOptions()
        tight.min_collisions = 1
        tight.max_root_collisions = 2
        tight.max_passes = sel.dfs_refine_max_passes
        tight.max_stagnant_passes = sel.dfs_refine_max_stagnant_passes
        tight.beam_width = sel.dfs_refine_beam_width
        for _ in range(passes):
            selected = list(increase_selection_dfs(
                graph_sorted_rev, selected, max_tries,
            ))
            selected = list(increase_selection_dfs(graph, selected, max_tries))
            selected = list(increase_score_dfs(
                graph_sorted_rev, selected, scores, loose,
            ))
            selected = list(increase_selection_dfs(
                graph_sorted, selected, max_tries,
            ))
            selected = list(increase_score_dfs(graph_sorted, selected, scores, tight))
        pre_finalize = selected
        return pre_finalize, pre_finalize, selection_score_sum(scores, pre_finalize)

    if mode == "strict_no_prune":
        strict = _strict_refine_options(sel)
        for _ in range(passes):
            selected = list(refine_selection(graph_sorted_rev, selected, scores, strict))
            selected = list(refine_selection(graph, selected, scores, strict))
        pre_finalize = selected
        return pre_finalize, pre_finalize, selection_score_sum(scores, pre_finalize)

    if mode == "strict_prune":
        strict = _strict_refine_options(sel)
        for _ in range(passes):
            selected = list(refine_selection(graph_sorted_rev, selected, scores, strict))
            selected = list(refine_selection(graph, selected, scores, strict))
        pre_finalize = selected
        final = prune_selection_to_independent_set(graph, selected, scores)
        return pre_finalize, final, selection_score_sum(scores, final)

    loose = _loose_refine_options(sel)
    tight = _tight_refine_options(sel)

    if mode == "merged_single_pass":
        for _ in range(passes):
            selected = list(refine_selection(graph_sorted_rev, selected, scores, loose))
            pre_finalize = selected
            final = _finalize()
        return pre_finalize, final, selection_score_sum(scores, final)

    if mode == "merged_loose_finalize_end":
        for _ in range(passes):
            selected = list(refine_selection(graph_sorted_rev, selected, scores, loose))
        pre_finalize = selected
        final = _finalize()
        return pre_finalize, final, selection_score_sum(scores, final)

    if mode in ("merged_loose_tight_finalize_end", "high_pass_loose"):
        for _ in range(passes):
            selected = list(refine_selection(graph_sorted_rev, selected, scores, loose))
            selected = list(refine_selection(graph, selected, scores, tight))
        pre_finalize = selected
        final = _finalize()
        return pre_finalize, final, selection_score_sum(scores, final)

    # merged_loose_tight: finalize after each outer pass
    for _ in range(passes):
        selected = list(refine_selection(graph_sorted_rev, selected, scores, loose))
        selected = list(refine_selection(graph, selected, scores, tight))
    pre_finalize = selected
    final = _finalize()
    return pre_finalize, final, selection_score_sum(scores, final)


def _make_seed_rule_sets(cfg: BuildGraphConfig) -> list[PlacementRuleSet]:
    first = PlacementRuleSet()
    first.append_rule(PointPlaceRule(pos=Vec2(x=0, y=0), r=0.1, w=0.1, group=0))
    first.append_rule(PointPlaceRule(pos=Vec2(x=0, y=0), r=0.1, w=0.1, group=1))
    return [first]


def _make_initial_rule_sets(cfg: BuildGraphConfig) -> list[PlacementRuleSet]:
    return _make_seed_rule_sets(cfg) + [_make_demo_rule_set(cfg)]


def run_build_graph(cfg: BuildGraphConfig) -> None:
    """Demo build loop: propose → graph → rules → nest → DFS refine.

    Shipped defaults match nest-pipeline tuning (``SelectionConfig`` merged_loose_tight,
    ``dfs_passes=3``, ``improve_rules_rounds=4``). See docs/build_graph_tuning.md.
    """
    p_outline = cfg.rules.board_polygon()
    user_holes = cfg.rules.board_holes
    p_sheet = cfg.rules.board_sheet_polygon()
    sheet_pad = cfg.rules.effective_sheet_padding()
    p1 = cfg.rules.rect_polygon()
    p2 = cfg.rules.tri_polygon()
    parts = [(p1, 0), (p2, 1)]
    max_verts = max(len(list(p.exterior.coords)) for p, _ in parts)
    max_interiors = max(len(getattr(p, "interiors", ()) or ()) for p, _ in parts)
    cfg = cfg.with_runtime_lean(
        n_holes=len(user_holes),
        max_part_vertices=max_verts,
        max_part_interiors=max_interiors,
        sheet_vertices=len(list(p_sheet.exterior.coords)),
        concave_parts=any(part_is_concave(p) for p, _ in parts),
        seeded=False,
    )
    rng = cfg.apply_seed()
    sc = cfg.sampling
    gc = cfg.graph
    sel = cfg.selection
    out = cfg.output

    selected_t = (
        rng.uniform(-1, 1, (sc.initial_random, 3)) * sc.transform_scale,
        rng.uniform(-1, 1, (sc.initial_random, 3)) * sc.transform_scale,
    )
    rule_sets = _make_initial_rule_sets(cfg)
    render_size = (out.render_size, out.render_size)
    video = cv.VideoWriter(
        out.video_path,
        cv.VideoWriter_fourcc(*'mp4v'),
        out.video_fps,
        render_size,
    )
    history = (np.zeros((1, 3)), np.zeros((1, 3)))
    graphs: list[ElemGraph] = []
    selection_window: list[tuple[np.ndarray, np.ndarray]] = []
    nest_state: NestState | None = None
    propose_feedback = ProposeFeedbackState()
    had_void_override = False
    board_area = p_sheet.area
    part_areas = (p1.area, p2.area)
    iters = tuple(range(out.n_iters))
    pbar = tqdm(
        iters,
        desc="Nesting",
        unit="iter",
        dynamic_ncols=True,
        disable=not out.progress,
    )

    for _it in pbar:
        propose_rules = _propose_rules_for_iter(cfg, rule_sets)
        sat_info = _late_border_saturation_info(
            cfg, nest_state, p_sheet, had_void_override=had_void_override,
        )
        if sat_info.sat_override:
            had_void_override = True
        border_sat = sat_info.active
        proposer_counts: dict[str, int] = {}
        propose_stats: dict = {
            "outline_cov": sat_info.outline_cov,
            "sat_override": sat_info.sat_override,
            "rim_progress": sat_info.rim_progress,
        }
        selected_t = _build_transform_batch(
            cfg,
            selected_t,
            history,
            rng,
            board=p_sheet,
            parts=parts,
            nest_state=nest_state,
            selection_window=selection_window,
            first_pass=nest_state is None,
            border_saturation=border_sat,
            rules=propose_rules,
            proposer_counts_out=proposer_counts,
            propose_stats_out=propose_stats,
            propose_feedback=propose_feedback,
        )
        first_pass = nest_state is None
        graph, polys, group_id, transform = make_polygon_graph(
            p_outline,
            [(p1, selected_t[0]), (p2, selected_t[1])],
            min_dist=cfg.board_min_dist_for(p_sheet, first_pass=first_pass),
            epsilon_ratio=cfg.placement_epsilon_ratio(first_pass=first_pass),
            user_holes=user_holes,
            extra_voids=nest_state_extra_voids(nest_state),
        )
        prop_n = int(propose_stats.get("proposal_count", 0))
        proposal_keys = propose_stats.get("proposal_keys", {})
        proposal_nodes = sum(
            1
            for gid, t in zip(group_id, transform, strict=True)
            if _transform_row_key(np.asarray(t, dtype=np.float64))
            in proposal_keys.get(gid, set())
        )
        propose_stats["proposal_nodes"] = proposal_nodes
        # Defer EMA/pool-scale updates until after refine attribution (below).
        _pending_feedback = None
        if prop_n > 0 and (proposer_counts or cfg.propose.place_profiles_enabled):
            graph_yield = min(1.0, len(polys) / prop_n)
            proposal_yield = min(1.0, proposal_nodes / prop_n)
            _pending_feedback = (graph_yield, proposal_yield)
            if not bool(cfg.propose.use_ema_proposer_scales):
                propose_feedback.record_iteration(
                    proposer_counts=proposer_counts,
                    graph_yield=graph_yield,
                    proposal_yield=proposal_yield,
                )
                if propose_feedback.proposer_pool_scales:
                    cfg = cfg.model_copy(update={
                        "propose": cfg.propose.model_copy(update={
                            "place_proposer_pool_scales": dict(
                                propose_feedback.proposer_pool_scales,
                            ),
                        }),
                    })
        graphs.append(graph)
        graphs = graphs[-gc.graphs_window:]
        first_pass = nest_state is None
        seed_rules = active_rule_set(_make_seed_rule_sets(cfg))
        free_info = None
        if first_pass and cfg.propose.first_pass_border_pack:
            scores = score_elems(graph, seed_rules)
            sheet, _ = board_context_from_geometry(p_outline, user_holes=user_holes)
            min_dist = cfg.board_min_dist_for(p_sheet, first_pass=True)
            selected_polys = _first_pass_border_ring_selection(
                graph, polys, p_sheet, min_dist, scores,
            )
            old_len = 0
            if not selected_polys:
                selected_polys = list(nest_by_graph(graph, [seed_rules])[0])
                old_len = len(selected_polys)
            if cfg.propose.first_pass_layered_pack and selected_polys:
                graph, polys, group_id, transform, selected_polys = (
                    _first_pass_layered_selection(
                        cfg,
                        p_sheet,
                        parts,
                        graph=graph,
                        p1=p1,
                        p2=p2,
                        selected_t=selected_t,
                        history=history,
                        rng=rng,
                        selection_window=selection_window,
                        polys=polys,
                        group_id=group_id,
                        transform=transform,
                        phase1_selected=list(selected_polys),
                        rule_set=seed_rules,
                        scores=scores,
                        selection=sel,
                    )
                )
            for round_idx in range(sel.improve_rules_rounds):
                rule_sets = improve_rules(
                    graphs,
                    rule_sets,
                    sel.rules_kept,
                    p_sheet,
                    mutation_presets=cfg.rules.mutation_presets(),
                    rule_score_penalty=sel.rule_score_penalty,
                    elite_count=sel.improve_rules_elite_count,
                    seed=int(rng.integers(0, 2**31)) + round_idx,
                    score_options=score_rules_options(sel),
                    max_rules_per_set=cfg.rules.max_rules_per_set,
                )
        else:
            for round_idx in range(sel.improve_rules_rounds):
                rule_sets = improve_rules(
                    graphs,
                    rule_sets,
                    sel.rules_kept,
                    p_sheet,
                    mutation_presets=cfg.rules.mutation_presets(),
                    rule_score_penalty=sel.rule_score_penalty,
                    elite_count=sel.improve_rules_elite_count,
                    seed=int(rng.integers(0, 2**31)) + round_idx,
                    score_options=score_rules_options(sel),
                    max_rules_per_set=cfg.rules.max_rules_per_set,
                )
            active_rules = active_rule_set(rule_sets)
            scores = list(score_elems(graph, active_rules))
            sheet, _ = board_context_from_geometry(p_outline, user_holes=user_holes)
            min_dist = cfg.board_min_dist_for(p_sheet, first_pass=first_pass)

            packed_for_free = (
                [nest_state.polys[i] for i in nest_state.selected_indices]
                if nest_state is not None and nest_state.selected_indices
                else []
            )
            mean_part = float(np.mean(part_areas)) if part_areas else 1.0
            void_thr = float(cfg.propose.late_border_void_override_ratio)
            if void_thr <= 0.0:
                void_thr = 2.5
            free_info = analyze_free_space(
                sheet, packed_for_free, mean_part, min_dist,
                void_ratio_threshold=void_thr,
            )
            free_poly = free_info.target_poly
            nest_rules = rule_sets
            refine_rules = active_rules
            sheet_diag = 0.0
            if sheet is not None and not sheet.is_empty:
                minx, miny, maxx, maxy = sheet.bounds
                sheet_diag = float(np.hypot(maxx - minx, maxy - miny))
            attr_w = float(cfg.propose.void_attractor_rule_weight)
            pole_w = float(cfg.propose.void_island_score_boost)
            pocket_w = float(cfg.propose.pocket_score_boost)
            small_w = float(cfg.propose.small_part_void_score_boost)
            void_r = _void_attractor_radius(
                min_dist, sheet_diag, cfg.rules.place_rule_radius,
            )
            if (
                free_info.kind == "large_void"
                and free_info.target_pt is not None
                and attr_w > 0.0
            ):
                attractor = _make_void_attractor_rule_set(
                    free_info.target_pt,
                    ngroups=cfg.rules.ngroups,
                    radius=void_r,
                    weight=attr_w,
                )
                nest_rules = _merge_void_attractor_into_rule_sets(
                    rule_sets,
                    attractor,
                    nest_rule_sets_used=sel.nest_rule_sets_used,
                    max_rules_per_set=cfg.rules.max_rules_per_set,
                )
                refine_rules = active_rule_set(nest_rules)
                scores = list(score_elems(graph, refine_rules))
            boost_hits = apply_void_selection_boosts(
                polys=polys,
                group_id=group_id,
                transform=transform,
                scores=scores,
                free_info=free_info,
                free_poly=free_poly,
                part_areas=part_areas,
                propose_stats=propose_stats,
                cfg=cfg,
                sheet_diag=sheet_diag,
                void_r=void_r,
            )

            use_greedy_nest = (
                bool(cfg.propose.void_greedy_nest_seed)
                and free_info.kind == "large_void"
                and scores
                and (pole_w > 0.0 or pocket_w > 0.0 or small_w > 0.0)
            )
            if use_greedy_nest:
                selected_nest = _nest_seed_from_boosted_scores(graph, scores)
            else:
                selected_nest = list(
                    nest_by_graph(graph, nest_rules[: sel.nest_rule_sets_used])[0]
                )
            old_len = len(selected_nest)
            n_void_nest = _count_selected_in_free(polys, selected_nest, free_poly)
            # Nest uses void-boosted scores. Keep a copy for refine so first-pass
            # border-kiss boost cannot tilt DFS against void islands (P0+P1).
            refine_scores = list(scores)
            if (
                first_pass
                and should_use_border_focus(Polygon(), cfg.propose)
                and free_info.kind != "large_void"
            ):
                _boost_border_scores(
                    polys, refine_scores, p_sheet, min_dist,
                    weight=cfg.propose.border_selection_score_boost,
                )
            _, selected_polys, _ = apply_dfs_refinement(
                graph,
                refine_rules,
                list(selected_nest),
                refine_scores,
                selection=sel,
            )
            pin_stats: dict = {}
            if (
                bool(getattr(cfg.propose, "enable_void_nest_pin", True))
                and free_poly is not None
                and not free_poly.is_empty
            ):
                selected_polys = _pin_nest_void_independent(
                    graph,
                    selected_nest,
                    selected_polys,
                    polys,
                    free_poly,
                    refine_scores,
                    stats_out=pin_stats,
                )
            else:
                pin_stats = {
                    "pin_candidates": 0,
                    "pin_added": 0,
                    "pin_blocked_collision": 0,
                    "pin_ms": 0.0,
                }
            n_void_refine = _count_selected_in_free(
                polys, selected_polys, free_poly,
            )
            proposed_map = propose_stats.get("proposed_by_group") or {}
            proposed_list = [
                proposed_map[g] for g in sorted(proposed_map)
            ] if proposed_map else None
            n_void_props = _count_props_in_free(proposed_list, free_poly)
            n_void_graph = _count_graph_in_free(polys, free_poly)
            pole_radius_metric = 0.25 * sheet_diag if sheet_diag > 0.0 else 0.0
            n_props_pole = _count_props_near_pole(
                proposed_list, free_info.target_pt, pole_radius_metric,
            )
            zones = propose_stats.get("zones_used") or []
            hijack = int(_zones_have_void_hijack(zones))
            outline_cov = float(propose_stats.get("outline_cov", 0.0))
            sat_override = int(bool(propose_stats.get("sat_override", False)))
            rim_progress = float(propose_stats.get("rim_progress", 0.0))
            zone_snip = ",".join(str(z) for z in zones[:4])
            pf_em = int(proposer_counts.get("_pocket_fit_emitted", 0) or proposer_counts.get("pocket_fit", 0))
            pf_att = int(proposer_counts.get("_pocket_fit_attempted", 0))
            pf_sel = int(proposer_counts.get("_pocket_fit_selected", 0))
            pf_surv = int(proposer_counts.get("_pocket_fit_survival_pct", -1))
            densify = propose_stats.get("densify_stats") or {}
            densify_f = int(densify.get("fired", proposer_counts.get("_densify_fired", 0)))
            densify_a = int(densify.get("accepted", proposer_counts.get("_densify_accepted", 0)))
            densify_reason = densify.get("densify_reason")
            pocket_skip = densify.get("pocket_skip") or propose_stats.get("pocket_skip") or []
            if isinstance(pocket_skip, str):
                pocket_skip = [pocket_skip]
            pocket_key_hits = int(boost_hits.get("pocket_keys", 0))
            small_hits = int(boost_hits.get("small_part", 0))
            island_hits = int(boost_hits.get("void_island", 0))
            nest_refine_delta = int(n_void_nest) - int(n_void_refine)
            skip_snip = ",".join(str(s) for s in list(pocket_skip)[:4])
            proposer_keys = propose_stats.get("proposer_keys") or densify.get("proposer_keys") or {}
            emitted_bp = dict(propose_stats.get("emitted_by_proposer") or densify.get("emitted_by_proposer") or {})
            pool_bp = dict(propose_stats.get("pool_by_proposer") or densify.get("pool_by_proposer") or {})
            nest_bp = _count_selected_by_proposer(transform, selected_nest, proposer_keys)
            refine_bp = _count_selected_by_proposer(transform, selected_polys, proposer_keys)
            prop_accept = _format_prop_accept(emitted_bp, pool_bp, nest_bp, refine_bp)
            pocket_by_tag = dict(propose_stats.get("pocket_by_tag") or densify.get("pocket_by_tag") or {})
            void_leak = (
                f"void_leak free={free_info.kind} ratio={free_info.max_void_ratio:.1f} "
                f"props={n_void_props} props_pole={n_props_pole} hijack={hijack} "
                f"graph={n_void_graph} "
                f"nest={n_void_nest} refine={n_void_refine} delta={nest_refine_delta} "
                f"border_only={bool(propose_stats.get('border_only', False))} "
                f"outline_cov={outline_cov:.3f} sat_override={sat_override} "
                f"rim={rim_progress:.3f} zones=[{zone_snip}] "
                f"pocket={pf_em}/{pf_att} sel={pf_sel} valid={pf_surv}% "
                f"key_boost={pocket_key_hits} island_boost={island_hits} "
                f"small_boost={small_hits} "
                f"pin={pin_stats.get('pin_added', 0)}/"
                f"{pin_stats.get('pin_candidates', 0)}"
                f"(block={pin_stats.get('pin_blocked_collision', 0)},"
                f"{pin_stats.get('pin_ms', 0.0):.1f}ms) "
                f"densify={densify_a}/{densify_f}"
                f"{f' reason={densify_reason}' if densify_reason else ''}"
                f"{f' pocket_skip=[{skip_snip}]' if skip_snip else ''}"
                f" cascade={densify.get('cascade_stopped_after', 'none')}"
                f" nms={int(densify.get('nms_kept', 0))}/{int(densify.get('nms_dropped', 0))}"
                f"{f' prop_accept {prop_accept}' if prop_accept else ''}"
            )
            # Append post-DFS relocate stats after they run (updated below).
            propose_stats["void_leak"] = {
                "free_kind": free_info.kind,
                "max_void_ratio": float(free_info.max_void_ratio),
                "props": n_void_props,
                "props_pole": n_props_pole,
                "hijack": bool(hijack),
                "graph": n_void_graph,
                "nest": n_void_nest,
                "refine": n_void_refine,
                "nest_refine_delta": nest_refine_delta,
                "border_only": bool(propose_stats.get("border_only", False)),
                "outline_cov": outline_cov,
                "sat_override": bool(sat_override),
                "rim_progress": rim_progress,
                "zones_used": list(zones),
                "pocket_fit_emitted": pf_em,
                "pocket_fit_attempts": pf_att,
                "pocket_fit_selected": pf_sel,
                "pin_candidates": int(pin_stats.get("pin_candidates", 0)),
                "pin_added": int(pin_stats.get("pin_added", 0)),
                "pin_blocked_collision": int(pin_stats.get("pin_blocked_collision", 0)),
                "pin_ms": float(pin_stats.get("pin_ms", 0.0)),
                "pocket_fit_survival_pct": pf_surv,
                "pocket_key_boost_hits": pocket_key_hits,
                "void_island_boost_hits": island_hits,
                "small_part_boost_hits": small_hits,
                "densify_fired": densify_f,
                "densify_accepted": densify_a,
                "densify_reason": densify_reason,
                "pocket_skip": list(pocket_skip),
                "emitted_by_proposer": emitted_bp,
                "pool_by_proposer": pool_bp,
                "nest_by_proposer": nest_bp,
                "refine_by_proposer": refine_bp,
                "pocket_by_tag": pocket_by_tag,
                "prop_accept": prop_accept,
                "cascade_stopped_after": densify.get("cascade_stopped_after", "none"),
                "cascade_skipped_proposers": list(
                    densify.get("cascade_skipped_proposers") or []
                ),
                "nms_kept": int(densify.get("nms_kept", 0)),
                "nms_dropped": int(densify.get("nms_dropped", 0)),
                "conflict_penalty_applied": int(
                    densify.get("conflict_penalty_applied", 0)
                ),
                "multi_pole_count": int(densify.get("multi_pole_count", 0) or 0),
            }
            print(void_leak)
        # EMA pool scales from refine survival (next iter).
        if (
            _pending_feedback is not None
            and bool(cfg.propose.use_ema_proposer_scales)
        ):
            gy, py = _pending_feedback
            densify = propose_stats.get("densify_stats") or {}
            emitted_bp = dict(
                propose_stats.get("emitted_by_proposer")
                or densify.get("emitted_by_proposer")
                or {}
            )
            proposer_keys = (
                propose_stats.get("proposer_keys")
                or densify.get("proposer_keys")
                or {}
            )
            refine_bp = _count_selected_by_proposer(
                transform, selected_polys, proposer_keys,
            )
            propose_feedback.record_iteration(
                proposer_counts=proposer_counts,
                graph_yield=gy,
                proposal_yield=py,
                refine_by_proposer=refine_bp,
                emitted_by_proposer=emitted_bp,
                use_ema=True,
                ema_alpha=float(cfg.propose.ema_proposer_alpha),
                ema_floor=float(cfg.propose.ema_proposer_floor),
            )
            if propose_feedback.proposer_pool_scales:
                cfg = cfg.model_copy(update={
                    "propose": cfg.propose.model_copy(update={
                        "place_proposer_pool_scales": dict(
                            propose_feedback.proposer_pool_scales,
                        ),
                    }),
                })
        assert selection_is_independent(graph, selected_polys)
        min_dist = cfg.board_min_dist_for(p_sheet, first_pass=first_pass)
        sheet_compact, _ = board_context_from_geometry(p_outline, user_holes=user_holes)
        part_by_group = {0: p1, 1: p2}
        seed_n = nest_state.seed_count if nest_state is not None else 0
        fixed_obs = list(nest_state.polys[:seed_n]) if nest_state is not None and seed_n else None
        relocate_accepted = False
        if (
            cfg.propose.enable_gravity_compaction
            and len(selected_polys) >= 2
        ):
            polys, transform = compact_selection(
                sheet_compact,
                list(polys),
                list(transform),
                group_id,
                selected_polys,
                part_by_group,
                min_dist,
                fixed_obstacles=fixed_obs,
                gravity=(
                    free_info.target_pt
                    if (
                        free_info is not None
                        and free_info.kind == "large_void"
                        and free_info.target_pt is not None
                    )
                    else None
                ),
            )
            assert selection_pairwise_independent(polys, selected_polys)
        # Post-DFS pack relocate when mid-pack free is still a large void.
        sel_geoms = [
            polys[i] for i in selected_polys
            if polys[i] is not None and not polys[i].is_empty
        ]
        mean_part_post = float(np.mean(part_areas)) if part_areas else 1.0
        void_thr_post = float(cfg.propose.late_border_void_override_ratio)
        if void_thr_post <= 0.0:
            void_thr_post = 2.5
        free_snap_post = build_free_space_snapshot(
            sheet_compact,
            sel_geoms,
            mean_part_post,
            min_dist,
            void_ratio_threshold=void_thr_post,
        )
        free_post = free_snap_post.analysis
        void_leak_stats = propose_stats.get("void_leak") if propose_stats else None
        # S5: when void_leak exists, require refine>0 so peel is not wasted on strip.
        allow_repack = True
        if isinstance(void_leak_stats, dict):
            allow_repack = int(void_leak_stats.get("refine", 0)) > 0
        if free_post.kind == "large_void" and free_post.target_pt is not None:
            push_pt = free_post.target_pt
            reloc_poles: list = [free_post.target_pt]
            if (
                bool(cfg.propose.use_multi_pole_void)
                and free_post.target_poly is not None
                and not free_post.target_poly.is_empty
            ):
                from nest_graph.propose.void_topology import iterative_multi_poles
                from shapely import Point as ShapelyPoint

                xy_poles = iterative_multi_poles(
                    free_post.target_poly,
                    min_dist=min_dist,
                    max_poles=int(cfg.propose.multi_pole_max_poles),
                )
                if xy_poles:
                    reloc_poles = [ShapelyPoint(x, y) for x, y in xy_poles]
                    push_pt = reloc_poles[0]
            void_leak_prev = propose_stats.get("void_leak")
            if isinstance(void_leak_prev, dict):
                void_leak_prev["multi_pole_count"] = len(reloc_poles)
            if allow_repack:
                polys, transform, selected_polys, repack_stats = cluster_repack_selection(
                    sheet_compact,
                    list(polys),
                    list(transform),
                    group_id,
                    selected_polys,
                    part_by_group,
                    min_dist,
                    cfg.propose,
                    pole=push_pt,
                    void_poly=free_post.target_poly,
                    fixed_obstacles=fixed_obs,
                    pt_push=push_pt,
                    free_space=free_snap_post,
                )
            else:
                repack_stats = {
                    "attempted": 0,
                    "accepted": 0,
                    "motif_accepted": 0,
                    "skipped_refine_zero": 1,
                }
            # Refresh void pole after an accepted motif stamp (selection moved).
            reloc_pole = push_pt
            if int(repack_stats.get("accepted", 0)) > 0:
                sel_geoms = [
                    polys[i] for i in selected_polys
                    if polys[i] is not None and not polys[i].is_empty
                ]
                free_snap_post = build_free_space_snapshot(
                    sheet_compact,
                    sel_geoms,
                    mean_part_post,
                    min_dist,
                    void_ratio_threshold=void_thr_post,
                )
                if free_snap_post.analysis.target_pt is not None:
                    reloc_pole = free_snap_post.analysis.target_pt
                elif len(reloc_poles) > 1:
                    reloc_pole = reloc_poles[min(1, len(reloc_poles) - 1)]
            elif len(reloc_poles) > 1:
                # Alternate spine poles when repack skipped.
                reloc_pole = reloc_poles[min(1, len(reloc_poles) - 1)]
            polys, transform, reloc_stats = cluster_relocate_selection(
                sheet_compact,
                list(polys),
                list(transform),
                group_id,
                selected_polys,
                part_by_group,
                min_dist,
                cfg.propose,
                pole=reloc_pole,
                fixed_obstacles=fixed_obs,
            )
            polys, transform, se2_stats = local_se2_selection(
                sheet_compact,
                list(polys),
                list(transform),
                group_id,
                selected_polys,
                part_by_group,
                min_dist,
                cfg.propose,
                pole=reloc_pole,
                fixed_obstacles=fixed_obs,
            )
            relocate_accepted = bool(
                int(repack_stats.get("accepted", 0))
                or int(reloc_stats.get("accepted", 0))
                or int(se2_stats.get("moved", 0))
            )
            assert selection_pairwise_independent(polys, selected_polys)
            propose_stats["repack"] = repack_stats
            propose_stats["relocate"] = reloc_stats
            propose_stats["local_se2"] = se2_stats
            void_leak_prev = propose_stats.get("void_leak")
            if isinstance(void_leak_prev, dict):
                void_leak_prev["repack"] = repack_stats
                void_leak_prev["relocate"] = reloc_stats
                void_leak_prev["local_se2"] = se2_stats
            print(
                f"repack={repack_stats.get('accepted', 0)}/"
                f"{repack_stats.get('attempted', 0)} "
                f"motif={repack_stats.get('motif_accepted', 0)} "
                f"reloc={reloc_stats.get('accepted', 0)}/"
                f"{reloc_stats.get('attempted', 0)} "
                f"se2={se2_stats.get('moved', 0)}/"
                f"{se2_stats.get('attempted', 0)} "
                f"tan={se2_stats.get('tangent_moves', 0)}"
            )
        cov = _selection_coverage_pct(
            selected_polys, group_id, part_areas, board_area,
        )
        if out.progress:
            pbar.set_postfix(
                parts=len(selected_polys),
                cov=f"{cov:.1f}%",
                pool=len(polys),
                refine=f"{old_len}->{len(selected_polys)}",
                ordered=True,
            )
        else:
            print(
                len(polys), old_len, "->", len(selected_polys),
                f"cov={cov:.1f}%",
            )
        render_frame = shapely_box(*padded_board_bounds(p_outline, sheet_pad))
        im = render_polys(
            render_frame,
            [[polys[i] for i in selected_polys]],
            im_shape=render_size,
            nest_outline=p_sheet,
        )
        video.write(im)
        cv.imwrite(out.snapshot_path, im)

        selected_t = ([], [])
        for i in selected_polys:
            gi = group_id[i]
            selected_t[gi].append(transform[i])
        selected_t = tuple(np.array(t) for t in selected_t)
        if relocate_accepted:
            # Drop stale window frames before appending post-move poses.
            selection_window.clear()
        _append_selection_window(selection_window, selected_t, gc.graphs_window)
        if relocate_accepted:
            # Replace history so trim_history cannot re-inject pre-relocate poses.
            history = selected_t
        else:
            if len(history[0]) and len(selected_t[0]):
                history = (
                    trim_history(history[0], selected_t[0], sc.history_max),
                    history[1],
                )
            if len(history[1]) and len(selected_t[1]):
                history = (
                    history[0],
                    trim_history(history[1], selected_t[1], sc.history_max),
                )
        nest_state = NestState(
            polys=polys,
            group_id=group_id,
            transform=transform,
            selected_indices=list(selected_polys),
            seed_count=nest_state.seed_count if nest_state is not None else 0,
        )
        rule_sets = _inject_repulsor_rules(rule_sets, cfg, p_sheet, nest_state)

    video.release()


def transform_selection(s, n, rng: np.random.Generator):
    yield transforms_around(s, (0.1, 0.1, 1.5), n, rng)
    yield transforms_around(s, (0.1, 0.1, 0), n, rng)
    yield transforms_around(s, (0, 0, 1.5), n, rng)
    yield transforms_around(s, (0.05, 0.05, 1), n, rng)
    yield transforms_around(s, (0.05, 0.05, 0), n, rng)
    yield transforms_around(s, (0, 0, 1), n, rng)
    yield transforms_around(s, (0.01, 0.01, 0.01), n, rng)
    yield transforms_around(s, (0.01, 0.01, 0), n, rng)
    yield transforms_around(s, (0, 0, 0.01), n, rng)
    yield transforms_around(s, (0.001, 0.001, 0.001), n, rng)
    yield transforms_around(s, (0.001, 0.001, 0), n, rng)
    yield transforms_around(s, (0, 0, 0.001), n, rng)


def transform_history(h, n, rng: np.random.Generator):
    yield transforms_around(h, (0.05, 0.05, 0.1), n, rng)
    yield transforms_around(h, (0.05, 0.05, 0), n, rng)
    yield transforms_around(h, (0, 0, 0.1), n, rng)


def test_placement():
    p_board = Polygon([(0, 0), (1.2, 0), (0, 1.1)])
    p1 = normalize_poly(Polygon([(0, 0), (.15, 0), (0, .07)]))
    p2 = normalize_poly(Polygon([(0, 0), (.1, 0), (.1, .1), (0, .1)]))

    p1_result = []
    p2_result = []
    base_shape = Polygon()
    for _ in range(100):
        p1_places = propose_placements_point_cloud(
            base_shape, p1, p_board, min_dist=0.001, pt_push=p_board.centroid, top_n=100
        )
        print('p1', len(p1_places))
        if p1_places:
            p1_result.append(p1_places[0])
            base_shape = unary_union([base_shape, transform_poly(p1, p1_places[0])])
        p2_places = propose_placements_point_cloud(
            base_shape, p2, p_board, min_dist=0.001, pt_push=p_board.centroid, top_n=100
        )
        print('p2', len(p2_places))
        if p2_places:
            p2_result.append(p2_places[0])
            base_shape = unary_union([base_shape, transform_poly(p2, p2_places[0])])

        im = render_polys(p_board, [
            [transform_poly(p1, t) for t in p1_result],
            [transform_poly(p2, t) for t in p2_result]
        ])
        cv.imwrite('test.jpg', im)


def main():
    run_build_graph(BuildGraphConfig.from_env())


if __name__ == "__main__":
    main()
