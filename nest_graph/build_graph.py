# CFLAGS="-Wno-error=incompatible-pointer-types" pip install --force-reinstall --no-binary=shapely --upgrade shapely

import cv2 as cv
import numpy as np
import os
import time
from dataclasses import dataclass, field
from pydantic import BaseModel, ConfigDict
from shapely import Polygon, unary_union
from shapely.geometry import box as shapely_box
from shapely.geometry import Point
from shapely.geometry.base import BaseGeometry
from tqdm import tqdm
from typing import Iterator, NamedTuple, Sequence, Tuple

from .config import (
    BuildGraphConfig,
    ProposeConfig,
    SelectionConfig,
    score_rules_options,
    trim_history,
)
from .geometry import (
    Geometry,
    batch_check_validity,
    find_polygon_intersections,
)
from .placement_scene import (
    guidance_config_for_graph,
    placement_scene_for_part,
)
from .propose.placement_common import as_geometry
from .propose.placement_outline import outline_standoff_distance
from .board import (
    board_context_from_geometry,
    default_sheet_padding,
    padded_board_bounds,
)
from .utils import normalize_poly, transform_poly
from .track_perf import show_performance
from .propose import (
    propose_placements_point_cloud,
)
from .propose.feedback import ProposeFeedbackState
from .propose.pipeline import allow_void_repack
from .propose.post_pack import run_post_pack_passes
from .propose.pattern_archive import (
    age_motif_library,
    motif_graph_hits,
    merge_motif_hits,
    motif_patterns_for_inject,
    note_motif_hollow_miss,
)
from .propose.placements_selection_expand import (
    transforms_around as _transforms_around_impl,
)
from .propose.void_topology import iterative_multi_poles
from .propose.placement_common import selection_pairwise_independent
from .propose.cluster_repack import (
    cluster_repack_selection,
)
from .propose.context import (
    build_free_space_snapshot,
    outline_coverage_ratio,
    part_is_concave,
    prep_free_space,
    sheet_has_narrow_corridor,
    should_use_border_focus,
    void_ratio_threshold,
)
from .propose.telem import (
    assemble_void_leak,
    build_motif_telem,
)
from nest_graph.propose.block_replace import try_block_hole_renest
from .propose.heavy_polish import (
    apply_dfs_refinement,
    apply_refine_with_restore,
    polish_budget_for_iter,
    run_improve_rules_rounds,
)
from .propose.context import (
    late_border_saturation_info,
)
from .propose.first_pass_border import (
    border_pack_graph,
    border_saturation_transform_batch,
    build_elem_graph,
    guidance_border_refine,
    join_attract_pairs,
    sequential_border_augment,
)
from .propose.void_selection import (
    centroid_in_free,
    count_graph_in_free,
    count_props_in_free,
    count_props_near_pole,
    count_selected_by_proposer,
    count_selected_in_free,
    format_prop_accept,
    pin_nest_void_independent,
    transform_row_key,
    void_pole_near_radius,
    zones_have_void_hijack,
)
from .propose.selection_compose import (
    active_rule_set as compose_active_rule_set,
    compose_and_nest_selection,
    compose_nest_kwargs,
    dual_nest_for,
    kiss_lock_subset,
    nest_border_kiss_selection,
    sheet_diag_from,
)
from .propose.transform_batch import (
    build_transform_batch,
    graph_valid_carry_by_group,
    prepend_group_transforms,
)
from .propose.motif_lock import LargeVoidMotifPlateau
from .decision.action_gen import region_to_zone
from .decision.epoch import bind_epoch, materialize_selection
from .decision.browse import (
    choose_browse_parent,
    packed_gids_compatible,
    should_browse_tip,
)
from .decision.execute import (
    board_snapshot_from_selection,
    execute_pack,
    make_execute_fn,
    prep_selection_free,
    prep_selection_freeze,
    record_mcts_expand,
    run_mcts_multi_sim,
    schedule_prep_selection_free,
)
from .decision.motif_credit import (
    credit_motif_on_nest_survival,
    credit_void_niche_from_iter,
    merge_void_elite_with_archive,
    niche_amaf_key,
)
from .decision.ram_budget import evaluate_ram_band
from .decision.runner import MacroMctsRunner
from .decision.types import BoardSnapshot
from .elem_graph import (
    PoseGraph, Circle, Vec2,
    PointPlaceRule, PointAngleRule, PlacementRuleSet,
    RuleMutationSettings,
    nest_by_graph, nest_by_scores, score_elems, augment_rules, score_rules,
    ScoreRulesOptions,
    selection_is_independent,
    MacroRegion,
)

# Track performance
nest_by_graph = show_performance(nest_by_graph)
nest_by_scores = show_performance(nest_by_scores)
score_elems = show_performance(score_elems)
score_rules = show_performance(score_rules)


class Candidate(NamedTuple):
    group_i: int
    weight: float
    t: np.ndarray
    placed: Geometry


@dataclass
class NestState:
    """Packed nest snapshot for propose / graph rebuild.

    Do not mutate ``polys`` under a live NestState; construct a new NestState on
    rebuild so the lazy ``native_geoms`` cache stays coherent.
    Prefer passing ``native_geoms`` from ``make_polygon_graph`` / SE2 apply_transform
    so propose hot paths skip re-``from_shapely`` (and re-decomp) of the pack.
    """

    polys: list
    group_id: list
    transform: list
    selected_indices: list
    # First seed_count entries in polys are locked obstacles (extra_voids), not graph nodes.
    seed_count: int = 0
    _native_geoms: list | None = field(default=None, repr=False, compare=False)

    @property
    def native_geoms(self) -> list[Geometry]:
        """Cached Geometry per poly index (built once; prefer graph-placed solids)."""
        if self._native_geoms is None:
            out: list[Geometry] = []
            for p in self.polys:
                g = as_geometry(p)
                if g is None:
                    raise ValueError("NestState.native_geoms: empty/invalid poly")
                out.append(g)
            self._native_geoms = out
        return self._native_geoms


def nest_state_extra_voids(nest_state: NestState | None) -> list[Geometry] | None:
    """Geometry voids for locked seed parts (clearance obstacles, not candidates)."""
    if nest_state is None or nest_state.seed_count <= 0:
        return None
    n = min(nest_state.seed_count, len(nest_state.polys))
    if n <= 0:
        return None
    return list(nest_state.native_geoms[:n])


def _native_geoms_from_transforms(
    group_id: Sequence[int],
    transform: Sequence,
    bases: dict[int, Geometry],
    *,
    seed_polys: Sequence | None = None,
    seed_count: int = 0,
) -> list[Geometry]:
    """SE2 solids from part bases (no re-decomp of transformed Shapely)."""
    out: list[Geometry] = []
    n = len(group_id)
    for i in range(n):
        if seed_count > 0 and i < seed_count and seed_polys is not None and i < len(seed_polys):
            g = as_geometry(seed_polys[i])
            if g is None:
                raise ValueError("seed poly empty")
            out.append(g)
            continue
        gid = int(group_id[i])
        out.append(bases[gid].apply_transform(np.asarray(transform[i], dtype=np.float64)))
    return out


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


def _base_geometries(polygons) -> list[Geometry]:
    out: list[Geometry] = []
    for item in polygons:
        g = as_geometry(_poly_and_transforms(item)[0])
        if g is None:
            raise ValueError("_base_geometries: empty/invalid part")
        out.append(g)
    return out


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
            if not scene.is_valid(
                placed, (cx, cy), min_dist, guidance_cfg,
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
    """Heuristic score: prefer inside sheet near boundary (analysis, not validity SoT)."""
    del board_geom
    cx, cy = placed.center()
    if not board.contains(Point(cx, cy)):
        return -board.distance(Point(cx, cy))
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


class PolygonGroup(BaseModel):
    model_config = ConfigDict(arbitrary_types_allowed=True)

    polygon: Polygon
    weight: float
    transforms: np.ndarray


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
    attract_pairs: Sequence[tuple[int, int, float]] | None = None,
    propose_stats: dict | None = None,
    attract_contact_weight: float = 8.0,
    attract_kiss_band_scale: float = 2.0,
    attract_max_degree: int = 8,
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

    by_group: dict[int, list[int]] = {}
    for k, (i, _p, _t, _placed, _base) in enumerate(candidates):
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
        if not valid_by_k.get(k, False):
            continue
        pending.append((i, p, t, placed))

    pending_geoms = [placed for _i, _p, _t, placed in pending]
    gids = [i for i, _p, _t, _placed in pending]
    angles = [float(t[2]) for _i, _p, t, _placed in pending]
    selected_polys = [transform_poly(p, t) for _i, p, t, _placed in pending]
    selected_group_id = list(gids)
    selected_transform = [t for _i, _p, t, _placed in pending]
    if attract_pairs is None:
        attract_pairs = join_attract_pairs(
            propose_stats,
            gids,
            selected_transform,
            pending_geoms,
            min_dist=min_dist,
            contact_weight=attract_contact_weight,
            kiss_band_scale=attract_kiss_band_scale,
            max_degree=attract_max_degree,
        )
    graph = build_elem_graph(gids, pending_geoms, angles, attract_pairs=attract_pairs)

    return graph, selected_polys, selected_group_id, selected_transform


def transforms_around(
    p: np.ndarray,
    s: Tuple[float, float, float],
    n: int,
    rng: np.random.Generator,
) -> np.ndarray:
    return _transforms_around_impl(p, s, n, rng)


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
    return compose_active_rule_set(rule_sets)


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
    graph: PoseGraph,
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


def _expand_border_selection(
    graph: PoseGraph,
    polys: list,
    outline: BaseGeometry,
    min_dist: float,
    scores: list[float],
    initial: list[int],
) -> list[int]:
    """Greedy MIS on outline-kiss nodes, preserving ``initial`` (Ua → nest_border_kiss)."""
    return nest_border_kiss_selection(
        graph, polys, outline, min_dist, scores, locked=initial,
    )


def _first_pass_border_ring_selection(
    graph: PoseGraph,
    polys: list,
    outline: BaseGeometry,
    min_dist: float,
    scores: list[float],
) -> list[int]:
    """Pack as many outline-kiss nodes as possible (Ua → nest_border_kiss)."""
    return nest_border_kiss_selection(
        graph, polys, outline, min_dist, scores, locked=None,
    )


def _locked_graph_indices(
    transforms: list[np.ndarray],
    phase1_transforms: list[np.ndarray],
) -> list[int]:
    keys = {transform_row_key(t) for t in phase1_transforms}
    return [i for i, t in enumerate(transforms) if transform_row_key(t) in keys]


def _first_pass_layered_selection(
    cfg: BuildGraphConfig,
    board: BaseGeometry,
    parts: list[tuple[Polygon, int]],
    *,
    graph: PoseGraph,
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
    propose_stats: dict | None = None,
    dg=None,
) -> tuple[PoseGraph, list, list[int], list[np.ndarray], list[int]]:
    """Rebuild with packed obstacles; saturate outline-kiss placements along the perimeter."""
    min_dist = cfg.board_min_dist(first_pass=True)
    outline = board
    sheet, voids = board_context_from_geometry(board)
    board_geom = Geometry.from_shapely(sheet)
    part_bases = (
        {0: Geometry.from_shapely(parts[0][0]), 1: Geometry.from_shapely(parts[1][0])}
        if len(parts) >= 2
        else {gid: Geometry.from_shapely(poly) for poly, gid in parts}
    )
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
            _native_geoms=_native_geoms_from_transforms(
                group_id_cur,
                transform_cur,
                part_bases,
            ),
        )
        batch2 = border_saturation_transform_batch(
            cfg,
            board,
            parts,
            nest_state,
        )
        combined = (
            prepend_group_transforms(phase1_t[0], batch2[0]),
            prepend_group_transforms(phase1_t[1], batch2[1]),
        )
        graph2, polys2, group_id2, transform2 = make_polygon_graph(
            board,
            [(p1, combined[0]), (p2, combined[1])],
            min_dist=min_dist,
            epsilon_ratio=cfg.placement_epsilon_ratio(first_pass=True),
            propose_stats=propose_stats,
            attract_contact_weight=float(cfg.propose.attract_contact_weight),
            attract_kiss_band_scale=float(cfg.propose.attract_kiss_band_scale),
            attract_max_degree=int(cfg.propose.attract_max_degree),
        )
        bind_epoch(dg, graph2, propose_stats, group_id2, transform2)
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
        return sequential_border_augment(
            cfg,
            board,
            parts,
            outline=outline,
            polys=polys_cur,
            group_id=group_id_cur,
            transform=transform_cur,
            selected=sel_cur,
            skip_guidance_refine=skip_guidance_refine,
            part_bases=part_bases,
            board_geom=board_geom,
        )

    pack_polys = [polys_cur[i] for i in sel_cur]
    pack_gids = [group_id_cur[i] for i in sel_cur]
    pack_tr = [transform_cur[i] for i in sel_cur]
    if not skip_guidance_refine:
        pack_polys, pack_gids, pack_tr = guidance_border_refine(
            cfg,
            board,
            parts,
            outline=outline,
            pack_polys=pack_polys,
            pack_gids=pack_gids,
            pack_tr=pack_tr,
            part_bases=part_bases,
            board_geom=board_geom,
        )
    part_by_gid = {gid: poly for poly, gid in parts}
    bases = part_bases
    return border_pack_graph(
        pack_polys,
        pack_gids,
        pack_tr,
        void_geoms=voids,
        bases=bases,
    )


def void_elite_tuple_from_archive(
    archive: dict[int, list[np.ndarray]] | None,
    ngroups: int,
) -> tuple[np.ndarray, ...]:
    """Build per-group void-elite arrays for ``build_transform_batch``."""
    out: list[np.ndarray] = []
    for g in range(max(int(ngroups), 0)):
        rows = (archive or {}).get(g) or []
        if rows:
            out.append(np.asarray(rows, dtype=np.float64).reshape(-1, 3))
        else:
            out.append(np.zeros((0, 3), dtype=np.float64))
    return tuple(out)


def archive_void_elite_transforms(
    *,
    selected_nest: Sequence[int],
    selected_refine: Sequence[int],
    polys: Sequence,
    transforms: Sequence,
    group_ids: Sequence[int],
    free_poly,
    scores: Sequence[float] | None,
    max_keep: int = 32,
    enabled: bool = True,
) -> dict[int, list[np.ndarray]]:
    """Top-scoring nest-void losers (missing from refine) → next-iter elite seeds."""
    if not enabled or free_poly is None or getattr(free_poly, "is_empty", True):
        return {}
    refine_set = set(int(i) for i in selected_refine)
    scored: list[tuple[float, int]] = []
    for v in selected_nest:
        vi = int(v)
        if vi in refine_set:
            continue
        if not centroid_in_free(polys[vi], free_poly):
            continue
        sc_v = float(scores[vi]) if scores is not None and vi < len(scores) else 0.0
        scored.append((sc_v, vi))
    scored.sort(key=lambda x: x[0], reverse=True)
    next_elite: dict[int, list[np.ndarray]] = {}
    for _sc, v in scored[: max(int(max_keep), 0)]:
        gid = int(group_ids[v]) if v < len(group_ids) else 0
        row = np.asarray(transforms[v], dtype=np.float64).reshape(3)
        next_elite.setdefault(gid, []).append(row)
    return next_elite


def void_elite_count(archive: dict[int, list[np.ndarray]] | None) -> int:
    if not archive:
        return 0
    return sum(len(v) for v in archive.values())


@dataclass
class PlateauTracker:
    """Coverage + part-count stagnation detector for budget / search escalation."""

    flat_iters: int = 3
    cov_eps: float = 0.05
    last_cov: float | None = None
    last_parts: int | None = None
    streak: int = 0
    on_plateau: bool = False

    def update(self, cov: float, parts: int) -> bool:
        if self.last_cov is not None and self.last_parts is not None:
            flat = (
                abs(float(cov) - float(self.last_cov)) <= float(self.cov_eps)
                and int(parts) == int(self.last_parts)
            )
            self.streak = self.streak + 1 if flat else 0
        else:
            self.streak = 0
        self.last_cov = float(cov)
        self.last_parts = int(parts)
        self.on_plateau = self.streak >= max(int(self.flat_iters), 1)
        return self.on_plateau


def _selection_budget_for_iter(
    sel: SelectionConfig,
    *,
    on_plateau: bool,
) -> SelectionConfig:
    """Phase 1: taper rule evolution on plateau. Phase 4: escalate beam on plateau."""
    if not on_plateau:
        return sel
    updates: dict = {}
    if bool(getattr(sel, "enable_plateau_budget_taper", True)):
        updates["improve_rules_rounds"] = min(
            int(sel.improve_rules_rounds),
            max(int(getattr(sel, "plateau_taper_improve_rounds", 1)), 0),
        )
        updates["rules_kept"] = min(
            int(sel.rules_kept),
            max(int(getattr(sel, "plateau_taper_rules_kept", 16)), 1),
        )
        if bool(getattr(sel, "plateau_drop_local_swap", True)):
            updates["score_rules_local_swap"] = False
    plateau_beam = int(getattr(sel, "plateau_beam_width", 0) or 0)
    plateau_stag = int(getattr(sel, "plateau_max_stagnant_passes", 0) or 0)
    if plateau_beam > int(sel.dfs_refine_beam_width):
        updates["dfs_refine_beam_width"] = plateau_beam
    if plateau_stag > int(sel.dfs_refine_max_stagnant_passes):
        updates["dfs_refine_max_stagnant_passes"] = plateau_stag
    if not updates:
        return sel
    return sel.model_copy(update=updates)


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


def _make_seed_rule_sets(cfg: BuildGraphConfig) -> list[PlacementRuleSet]:
    first = PlacementRuleSet()
    first.append_rule(PointPlaceRule(pos=Vec2(x=0, y=0), r=0.1, w=0.1, group=0))
    first.append_rule(PointPlaceRule(pos=Vec2(x=0, y=0), r=0.1, w=0.1, group=1))
    return [first]


def _make_initial_rule_sets(cfg: BuildGraphConfig) -> list[PlacementRuleSet]:
    return _make_seed_rule_sets(cfg) + [_make_demo_rule_set(cfg)]


def cheap_pack_cache_key(zone, action) -> tuple[str, int]:
    """Q143: cheap cache is (zone, motif_id); motif_id=-1 when not Motif."""
    motif_id = -1
    if action is not None and int(getattr(action, "motif_id", -1) or -1) >= 0:
        motif_id = int(action.motif_id)
    return (str(zone or ""), motif_id)


def compose_cached_selection(
    pack_cache: dict,
    *,
    rule_sets: list,
    sel,
    zone,
    patterns,
    native_geoms_fn,
    coverage_pct_fn,
) -> tuple[list[int], float]:
    """Cheap compose from last outer graph; Motif miss injects cluster_patterns."""
    if not pack_cache.get("ready"):
        return list(pack_cache.get("selected") or ()), 0.0
    graph = pack_cache["graph"]
    polys = pack_cache["polys"]
    group_id = pack_cache["group_id"]
    transform = pack_cache["transform"]
    part_areas_c = pack_cache["part_areas"]
    part_bases_c = pack_cache["part_bases"]
    p_sheet_c = pack_cache["p_sheet"]
    min_dist_c = float(pack_cache["min_dist"])
    cfg_c = pack_cache["cfg"]
    free_info_c = pack_cache.get("free_info")
    propose_stats_c = dict(pack_cache.get("propose_stats") or {})
    propose_stats_c["mcts_zone"] = zone
    pats = list(patterns or [])
    n_cc = 0
    if pats:
        motif_keys, cohorts, n_cc = motif_graph_hits(pats, group_id, transform)
        merge_motif_hits(propose_stats_c, motif_keys, cohorts)
    pack_cache["cheap_cluster_copy_n"] = int(n_cc)
    active_rules = active_rule_set(rule_sets)
    scores = list(score_elems(graph, active_rules))
    sheet_diag = sheet_diag_from(pack_cache.get("sheet") or p_sheet_c)
    candidate_geoms = native_geoms_fn(
        group_id, transform, part_bases_c,
    )
    composed = compose_and_nest_selection(
        **compose_nest_kwargs(
            graph=graph,
            rule_sets=rule_sets,
            active_rules=active_rules,
            scores=scores,
            polys=polys,
            group_id=group_id,
            transform=transform,
            candidate_geoms=candidate_geoms,
            packed_geoms=list(pack_cache.get("packed_geoms") or []),
            part_areas=part_areas_c,
            free_info=free_info_c,
            cfg=cfg_c,
            selection=sel,
            first_pass=False,
            outline=p_sheet_c,
            min_dist=min_dist_c,
            sheet_area=float(pack_cache.get("sheet_area") or 0.0),
            sheet_diag=sheet_diag,
            propose_stats=propose_stats_c,
            ngroups=int(cfg_c.rules.ngroups),
            packed_group_id=pack_cache.get("packed_group_id"),
            packed_transform=pack_cache.get("packed_transform"),
            last_leaf=False,
            void_geoms=pack_cache.get("void_geoms"),
            dg=pack_cache.get("dg"),
        )
    )
    pack_cache["compose_sel"] = list(composed.selected_nest)
    pack_cache["compose_scores"] = composed.refine_scores
    pack_cache["compose_rules"] = composed.refine_rules
    pack_cache["compose_polys"] = polys
    pack_cache["compose_group_id"] = group_id
    pack_cache["compose_transform"] = transform
    pack_cache["compose_free_poly"] = composed.free_poly
    pack_cache["compose_free_info"] = composed.free_info
    selected_out = list(composed.selected_nest)
    coverage_out = 0.0
    try:
        coverage_out = coverage_pct_fn(
            selected_out, group_id, part_areas_c,
            float(pack_cache.get("board_area") or 1.0),
        ) / 100.0
    except Exception:
        pass
    return selected_out, float(coverage_out)


def refine_cached_selection(
    pack_cache: dict,
    *,
    sel,
    budget,
    apply_dfs_fn,
    native_geoms_fn,
    coverage_pct_fn,
    telem: dict,
) -> tuple[list[int], float]:
    """Cheap mid DFS via ``apply_refine_with_restore`` (DFS stays in build_graph)."""
    telem["dfs_passes"] = int(budget.dfs_passes)
    if not pack_cache.get("ready") or "compose_sel" not in pack_cache:
        return list(pack_cache.get("selected") or ()), 0.0
    graph = pack_cache["graph"]
    polys = pack_cache["compose_polys"]
    group_id = pack_cache["compose_group_id"]
    transform = pack_cache["compose_transform"]
    part_areas_c = pack_cache["part_areas"]
    part_bases_c = pack_cache["part_bases"]
    nest_sel = list(pack_cache["compose_sel"])
    node_areas = [
        float(part_areas_c[int(g)]) if int(g) < len(part_areas_c) else 0.0
        for g in group_id
    ]
    selected_out = apply_refine_with_restore(
        budget=budget,
        apply_dfs_fn=apply_dfs_fn,
        graph=graph,
        refine_rules=pack_cache["compose_rules"],
        selected_nest=nest_sel,
        refine_scores=pack_cache["compose_scores"],
        sel_iter=sel,
        node_areas=node_areas,
        refine_seed=0,
        locked_indices=[],
        polys=polys,
        group_id=group_id,
        transform=transform,
        part_areas=part_areas_c,
        part_bases=part_bases_c,
        sheet=pack_cache["p_sheet"],
        min_dist=float(pack_cache["min_dist"]),
        free_poly=pack_cache.get("compose_free_poly"),
        free_info=pack_cache.get("compose_free_info"),
        rim_before=0.0,
        rim_reject=0.02,
        propose_stats=telem,
        native_geoms_from_transforms_fn=native_geoms_fn,
    )
    coverage_out = 0.0
    try:
        coverage_out = coverage_pct_fn(
            selected_out, group_id, part_areas_c,
            float(pack_cache.get("board_area") or 1.0),
        ) / 100.0
    except Exception:
        pass
    return list(selected_out), float(coverage_out)


def pack_execute_snapshot(
    parent,
    *,
    zone=None,
    action=None,
    patterns=None,
    pack_cache: dict,
    rule_sets: list,
    sel,
    cfg,
    apply_dfs_fn,
    native_geoms_fn,
    coverage_pct_fn,
) -> BoardSnapshot:
    """Cheap expand execute_fn: cache (zone, motif_id); miss re-composes (Q143)."""
    snap = parent
    cache_key = cheap_pack_cache_key(zone, action)
    cheap_map: dict = pack_cache.setdefault("cheap_by_key", {})
    if cache_key in cheap_map:
        cached = cheap_map[cache_key]
        pack_cache["last_execute_telem"] = dict(
            pack_cache.get("cheap_telem_by_key", {}).get(cache_key) or {}
        )
        pack_cache["last_execute_telem"]["cache_hit"] = 1
        return cached

    pats = list(patterns or [])
    telem: dict = {
        "patterns_n": len(pats),
        "zone": zone,
        "cache_key": cache_key,
        "cache_hit": 0,
    }

    selected_box: dict = {"sel": list(pack_cache.get("selected") or ()), "cov": 0.0}

    def compose_fn() -> None:
        selected_box["sel"], selected_box["cov"] = compose_cached_selection(
            pack_cache,
            rule_sets=rule_sets,
            sel=sel,
            zone=zone,
            patterns=pats,
            native_geoms_fn=native_geoms_fn,
            coverage_pct_fn=coverage_pct_fn,
        )

    budget = polish_budget_for_iter(
        is_last_leaf=False,
        sel=sel,
        cheap_expand=True,
        large_void=False,
    )

    def refine_fn() -> None:
        selected_box["sel"], selected_box["cov"] = refine_cached_selection(
            pack_cache,
            sel=sel,
            budget=budget,
            apply_dfs_fn=apply_dfs_fn,
            native_geoms_fn=native_geoms_fn,
            coverage_pct_fn=coverage_pct_fn,
            telem=telem,
        )

    stage = execute_pack(
        rim_only=False,
        heavy=False,
        compose_fn=compose_fn,
        refine_fn=refine_fn,
        post_pack_fn=None,
    )
    telem.update(stage)
    telem["pack_body"] = 1
    telem["polish_budget"] = "mid"
    telem["cluster_copy"] = int(pack_cache.get("cheap_cluster_copy_n") or 0)
    if telem["cluster_copy"]:
        telem.setdefault("emitted_by_proposer", {})["cluster_copy"] = telem["cluster_copy"]
    selected_out = list(selected_box["sel"])
    coverage_out = float(selected_box["cov"])
    if not selected_out:
        selected_out = list(
            pack_cache.get("selected")
            or getattr(snap, "packed_gids", ())
            or ()
        )
        coverage_out = float(getattr(snap, "coverage", 0.0) or coverage_out)
    packed_gids = tuple(getattr(snap, "packed_gids", ()) or ())
    packed_tf = tuple(getattr(snap, "packed_transforms", ()) or ())
    rem = tuple(getattr(snap, "remaining_gids", ()) or ())
    if pack_cache.get("ready") and selected_out and "compose_group_id" in pack_cache:
        gid_list = pack_cache["compose_group_id"]
        tf_list = pack_cache["compose_transform"]
        gids = []
        tfs = []
        for i in selected_out:
            ii = int(i)
            if 0 <= ii < len(gid_list) and ii < len(tf_list):
                gids.append(int(gid_list[ii]))
                t = tf_list[ii]
                tfs.append((float(t[0]), float(t[1]), float(t[2])))
        if gids:
            packed_gids = tuple(gids)
            packed_tf = tuple(tfs)
            rem = tuple(
                g for g in range(int(pack_cache["cfg"].rules.ngroups))
                if g not in set(gids)
            )
    free_kind = str(
        getattr(pack_cache.get("compose_free_info"), "kind", None)
        or getattr(snap, "free_kind", "")
        or ""
    )
    motif_used = ()
    if cache_key[1] >= 0:
        motif_used = (int(cache_key[1]),)
    out = BoardSnapshot(
        packed_gids=packed_gids,
        packed_transforms=packed_tf,
        remaining_gids=rem,
        coverage=float(coverage_out),
        arena_node_id=int(getattr(snap, "arena_node_id", 0) or 0),
        kiss_pairs=int(getattr(snap, "kiss_pairs", 0) or 0),
        mean_compactness=float(getattr(snap, "mean_compactness", 0.0) or 0.0),
        rim_fill=float(getattr(snap, "rim_fill", 0.0) or 0.0),
        void_fill=float(getattr(snap, "void_fill", 0.0) or 0.0),
        free_kind=free_kind,
        motif_ids_used=motif_used or tuple(getattr(snap, "motif_ids_used", ()) or ()),
    )
    pack_cache["last_execute_telem"] = telem
    pack_cache.setdefault("cheap_telem_by_key", {})[cache_key] = dict(telem)
    cheap_map[cache_key] = out
    return out


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
    part_bases = {0: Geometry.from_shapely(p1), 1: Geometry.from_shapely(p2)}
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
    graphs: list[PoseGraph] = []
    selection_window: list[tuple[np.ndarray, np.ndarray]] = []
    nest_state: NestState | None = None
    propose_feedback = ProposeFeedbackState()
    had_void_override = False
    void_elite_by_group: dict[int, list[np.ndarray]] = {0: [], 1: []}
    graph_valid_carry: tuple[np.ndarray, ...] = (
        np.zeros((0, 3), dtype=np.float64),
        np.zeros((0, 3), dtype=np.float64),
    )
    board_area = p_sheet.area
    part_areas = (p1.area, p2.area)
    plateau = PlateauTracker(
        flat_iters=int(getattr(sel, "plateau_flat_iters", 3) or 3),
        cov_eps=float(getattr(sel, "plateau_cov_eps", 0.05) or 0.05),
    )
    large_void_motif_plateau = LargeVoidMotifPlateau(
        flat_iters=int(getattr(cfg.propose, "large_void_motif_plateau_iters", 5) or 5),
        cov_eps=float(getattr(cfg.propose, "large_void_motif_plateau_cov_eps", 1.0) or 1.0),
    )
    pin_all_blocked_streak = 0
    iters = tuple(range(out.n_iters))
    pbar = tqdm(
        iters,
        desc="Nesting",
        unit="iter",
        dynamic_ncols=True,
        disable=not out.progress,
    )

    mcts_runner = MacroMctsRunner()
    mcts_root = BoardSnapshot(
        remaining_gids=tuple(range(int(cfg.rules.ngroups))),
        coverage=0.0,
    )
    mcts_runner.store_snapshot(int(mcts_runner.arena.root_id()), mcts_root)
    mcts_parent_id = int(mcts_runner.arena.root_id())
    mcts_telem = {
        "pw_expand": 0,
        "motif_hit": 0,
        "place_motif_ok": 0,
        "expand_ms": 0.0,
        "from_shapely_count": 0,
        "amaf_hits": 0,
        "amaf_miss": 0,
    }
    _exec_last: dict = {"snap": mcts_root}
    # DgP: cache last outer compose/refine materials for multi-sim cheap_pack.
    _pack_cache: dict = {"ready": False}
    _mcts_n_sims_default = int(os.environ.get("NEST_MCTS_N_SIMS", "4") or 4)
    _mcts_n_sims = int(_mcts_n_sims_default)
    _prev_void_nest = 0
    _archive_feed_keys: set[tuple] = set()
    _base_max_transforms = int(
        getattr(cfg.sampling, "max_transforms_per_group", None) or 5000
    )
    _base_graphs_window = int(getattr(gc, "graphs_window", 24) or 24)

    mcts_runner.execute_fn = make_execute_fn(
        lambda parent, zone=None, action=None, patterns=None: pack_execute_snapshot(
            parent,
            zone=zone,
            action=action,
            patterns=patterns,
            pack_cache=_pack_cache,
            rule_sets=rule_sets,
            sel=sel,
            cfg=cfg,
            apply_dfs_fn=apply_dfs_refinement,
            native_geoms_fn=_native_geoms_from_transforms,
            coverage_pct_fn=_selection_coverage_pct,
        )
    )

    for _it in pbar:
        # Q69: last leaf = full DFS + post_pack; mid = dfs_passes=1 finalize_end.
        # Hybrid: parent free_kind hints mid+large_void 3b; refreshed after compose.
        is_last_leaf = int(_it) >= int(out.n_iters) - 1
        near_last = int(_it) == int(out.n_iters) - 2
        parent_free_hint = str(
            getattr(mcts_runner.snapshot_at(int(mcts_parent_id), mcts_root), "free_kind", "")
            or ""
        ) == "large_void"
        polish_budget = polish_budget_for_iter(
            is_last_leaf=is_last_leaf,
            sel=sel,
            large_void=parent_free_hint,
            cheap_expand=False,
            near_last=near_last,
        )
        t_expand0 = time.perf_counter()
        # R1: RSS bands — taper graphs_window / transforms / n_sims; freeze expand.
        ram = evaluate_ram_band(
            default_graphs_window=int(_base_graphs_window),
            default_n_sims=int(_mcts_n_sims_default),
        )
        mcts_telem["rss_mb"] = float(ram.rss_mb)
        mcts_telem["ram_band"] = str(ram.band)
        _mcts_n_sims = int(ram.n_sims)
        if mcts_runner.agent is not None:
            mcts_runner.agent.expand_frozen = bool(ram.expand_frozen)
        win = int(ram.graphs_window) if ram.graphs_window is not None else int(
            _base_graphs_window
        )
        gc = gc.model_copy(update={"graphs_window": win})
        mt = max(64, int(_base_max_transforms * float(ram.transforms_scale)))
        cfg = cfg.model_copy(
            update={
                "sampling": cfg.sampling.model_copy(
                    update={"max_transforms_per_group": mt}
                )
            }
        )
        spine_id = int(mcts_parent_id)
        spine_snap = mcts_runner.snapshot_at(spine_id, mcts_root)
        nest_packed: set[int] = set()
        if nest_state is not None and nest_state.selected_indices:
            seed_n = int(nest_state.seed_count or 0)
            for i in nest_state.selected_indices:
                if int(i) < seed_n:
                    continue
                if int(i) < len(nest_state.group_id):
                    nest_packed.add(int(nest_state.group_id[i]))
        do_browse = should_browse_tip(
            iter_idx=int(_it),
            n_iters=int(out.n_iters),
            is_last_leaf=is_last_leaf,
        )
        browse_parent_id, parent_snap, _jumped = choose_browse_parent(
            mcts_runner,
            spine_id=spine_id,
            spine_snap=spine_snap,
            nest_packed_gids=nest_packed,
            do_browse=do_browse,
            mcts_telem=mcts_telem,
        )
        # DgP / Q107: multi-sim on NestState cache (not tip geometry).
        tip_action = None
        if _pack_cache.get("ready") and not is_last_leaf and int(_mcts_n_sims) > 0:
            tip_action, tip_leaf = run_mcts_multi_sim(
                mcts_runner,
                spine_snap,
                n_sims=int(_mcts_n_sims),
                parent_id=int(spine_id),
                execute_fn=mcts_runner.execute_fn,
                mcts_telem=mcts_telem,
            )
            cheap_t = _pack_cache.get("last_execute_telem") or {}
            for k in ("uh_ran", "compose_ran", "refine_ran", "cluster_copy", "cache_hit"):
                if k in cheap_t:
                    mcts_telem[k] = cheap_t[k]
            if do_browse and tip_leaf is not None:
                tip_snap = mcts_runner.snapshot_at(int(tip_leaf), missing_ok=True)
                if tip_snap is not None and packed_gids_compatible(
                    tip_snap, nest_packed
                ):
                    browse_parent_id = int(tip_leaf)
                    parent_snap = tip_snap
                    mcts_telem["browse_leaf_id"] = int(tip_leaf)
                    mcts_telem["browse_jump"] = int(
                        int(tip_leaf) != int(spine_id)
                    )
        mcts_parent_id = int(browse_parent_id)
        mcts_action = tip_action
        # P2: seed free_kind before AMAF pick so Void/Rim bias is live on iter 0+.
        if not str(getattr(parent_snap, "free_kind", "") or ""):
            try:
                md_fk = cfg.board_min_dist_for(p_sheet, first_pass=nest_state is None)
                packed_fk = []
                if nest_state is not None and nest_state.selected_indices:
                    seed_n = int(nest_state.seed_count or 0)
                    for i in nest_state.selected_indices:
                        if int(i) < seed_n:
                            continue
                        if int(i) < len(nest_state.polys):
                            packed_fk.append(nest_state.polys[i])
                fk_prep = prep_selection_free(
                    sheet=p_sheet,
                    part_areas=part_areas,
                    min_dist=md_fk,
                    cfg_propose=cfg.propose,
                    packed_shapely=packed_fk,
                    pack_geoms=None,
                )
                parent_snap.free_kind = str(
                    getattr(fk_prep.free_info, "kind", "") or ""
                )
            except Exception:
                pass
        if mcts_action is None and mcts_runner.agent is not None:
            mcts_action = mcts_runner.agent.pick_expand_action(
                parent_snap.remaining_gids
                or tuple(range(int(cfg.rules.ngroups))),
                rule_ids=(0,),
                parent_id=mcts_parent_id,
                snapshot=parent_snap,
            )
        mcts_force_zone = None
        if mcts_action is not None:
            mcts_force_zone = region_to_zone(mcts_action.region)
            if mcts_action.region == MacroRegion.Motif and int(mcts_action.motif_id) >= 0:
                mcts_telem["motif_hit"] = int(mcts_telem["motif_hit"]) + 1
                # Honest place_motif_ok: only after emit/accept (M0), not Motif pick.

        propose_rules = _propose_rules_for_iter(cfg, rule_sets)
        sel_iter = _selection_budget_for_iter(sel, on_plateau=plateau.on_plateau)
        sel_iter, freeze_reason = prep_selection_freeze(
            sel_iter,
            freeze_cheap_expand=bool(polish_budget.freeze_improve_rules),
            on_plateau=plateau.on_plateau,
            plateau_streak=int(plateau.streak),
            flat_iters=int(plateau.flat_iters),
            enable_incumbent_loop=bool(
                getattr(cfg.propose, "enable_incumbent_loop", True)
            ),
        )
        sat_info = late_border_saturation_info(
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
            "mcts_zone": mcts_force_zone,
            "mcts_heavy": int(polish_budget.mcts_heavy),
            "dfs_passes": int(polish_budget.dfs_passes),
            "dfs_mode": str(polish_budget.dfs_mode.value),
            "freeze_improve_reason": freeze_reason,
        }
        keep_hist_sterile = bool(
            (sat_info.sat_override or had_void_override)
            and bool(getattr(cfg.propose, "keep_history_on_void_sterile", True))
        )
        ngroups = max(len(parts) if parts else len(selected_t), 2)
        void_elite_t = void_elite_tuple_from_archive(void_elite_by_group, ngroups)
        elite_n = void_elite_count(void_elite_by_group)
        # Q137: under void_seek / large_void, merge MacroNicheArchive into elite niche.
        void_seek_mix = str(mcts_force_zone or "") in ("void_seek", "void")
        if not void_seek_mix:
            void_seek_mix = bool(
                str(parent_snap.free_kind or "") == "large_void"
                or int(mcts_runner.niche_archive.size) > 0
            )
        arch_rows = mcts_runner.niche_archive.active_by_group(ngroups)
        merged_elite = void_elite_by_group
        if arch_rows and void_seek_mix:
            merged_elite = merge_void_elite_with_archive(
                void_elite_by_group,
                arch_rows,
                elite_quota=int(
                    getattr(cfg.propose, "stratified_void_elite_quota", 15) or 15
                ),
                void_seek=True,
            )
            void_elite_t = void_elite_tuple_from_archive(merged_elite, ngroups)
            # Track feed keys for Q141 place-fail
            _archive_feed_keys = set()
            for gid, rows in arch_rows.items():
                for row in rows:
                    arr = np.asarray(row, dtype=np.float64).reshape(3)
                    _archive_feed_keys.add(
                        (
                            round(float(arr[0]), 4),
                            round(float(arr[1]), 4),
                            round(float(arr[2]), 4),
                        )
                    )
            mcts_runner.niche_archive.last_feed_keys = set(_archive_feed_keys)
        elite_n = void_elite_count(merged_elite)
        propose_stats["void_elite_seeded"] = elite_n
        propose_stats["archive_elite_n"] = sum(
            len(v) for v in (arch_rows or {}).values()
        )
        propose_stats["keep_history_on_sterile"] = keep_hist_sterile
        # Q141: cut sterile hist boost when archive miss_rate high or fail streak
        cut_sterile_boost = bool(
            mcts_runner.niche_archive.any_void_miss_rate_high(0.8)
            or int(mcts_runner.niche_archive.place_fail_streak) >= 3
        )
        propose_stats["cut_sterile_hist_boost"] = cut_sterile_boost
        prefer_mid = (
            int(mcts_action.motif_id)
            if mcts_action is not None
            and mcts_action.region == MacroRegion.Motif
            else -1
        )
        archived_for_propose = (
            motif_patterns_for_inject(
                mcts_runner.motif_base,
                max_keep=int(getattr(cfg.propose, "accepted_pattern_max", 4) or 4),
                prefer_motif_id=prefer_mid,
                part_bases=part_bases,
                min_dist=float(cfg.board_min_dist_for(p_sheet, first_pass=False)),
                telem=mcts_telem,
                polish=True,
            )
            if bool(getattr(cfg.propose, "enable_accepted_pattern_archive", True))
            else []
        )
        propose_stats["accepted_patterns_n"] = len(archived_for_propose)
        propose_stats["motif_library_n"] = int(mcts_runner.motif_base.size())
        propose_stats["nfp_lite_ok"] = int(mcts_telem.get("nfp_lite_ok", 0))
        propose_stats["dg_force_zone"] = mcts_force_zone
        if mcts_force_zone:
            enabled_z = ProposeConfig.proposers_for_place(str(mcts_force_zone))
            propose_stats["enabled_proposers_n"] = (
                len(enabled_z) if enabled_z is not None else 0
            )
        else:
            propose_stats["enabled_proposers_n"] = 0
        # Q99 / M0: soft Motif gids so stamp no_rels≠default under PLACE_MOTIF.
        parts_for_propose = parts
        if mcts_action is not None:
            propose_stats["mcts_part_gid"] = int(mcts_action.part_gid)
            if (
                mcts_action.region == MacroRegion.Motif
                and int(mcts_action.motif_id) >= 0
                and int(mcts_action.motif_id) < int(mcts_runner.motif_base.size())
            ):
                rec = mcts_runner.motif_base.at(int(mcts_action.motif_id))
                propose_stats["mcts_motif_gids"] = [int(rec.gid_a), int(rec.gid_b)]
        if archived_for_propose and not bool(
            getattr(cfg.propose, "enable_motif_scene_dry_run", False)
        ):
            cfg = cfg.model_copy(update={
                "propose": cfg.propose.model_copy(update={
                    "enable_motif_scene_dry_run": True,
                }),
            })
        selected_t = build_transform_batch(
            cfg,
            selected_t,
            history,
            rng,
            board=p_sheet,
            parts=parts_for_propose,
            nest_state=nest_state,
            selection_window=selection_window,
            first_pass=nest_state is None,
            border_saturation=border_sat,
            rules=propose_rules,
            proposer_counts_out=proposer_counts,
            propose_stats_out=propose_stats,
            propose_feedback=propose_feedback,
            void_elite_t=void_elite_t,
            keep_history_on_sterile=keep_hist_sterile,
            part_bases=part_bases,
            graph_valid_carry=graph_valid_carry,
            archived_patterns=archived_for_propose,
        )
        first_pass = nest_state is None
        free_prep_mid = None
        # Q128 / T1: soft-cap attract degree when prior graph ≫ nest.
        attract_deg = int(cfg.propose.attract_max_degree)
        prev_graph_n = int(mcts_telem.get("last_graph_n", 0) or 0)
        prev_nest_n = int(mcts_telem.get("last_nest_n", 0) or 0)
        if prev_graph_n > max(4 * max(prev_nest_n, 1), 200):
            attract_deg = min(attract_deg, 3)
            propose_stats["attract_degree_capped"] = int(attract_deg)
        graph, polys, group_id, transform = make_polygon_graph(
            p_outline,
            [(p1, selected_t[0]), (p2, selected_t[1])],
            min_dist=cfg.board_min_dist_for(p_sheet, first_pass=first_pass),
            epsilon_ratio=cfg.placement_epsilon_ratio(first_pass=first_pass),
            user_holes=user_holes,
            extra_voids=nest_state_extra_voids(nest_state),
            propose_stats=propose_stats,
            attract_contact_weight=float(cfg.propose.attract_contact_weight),
            attract_kiss_band_scale=float(cfg.propose.attract_kiss_band_scale),
            attract_max_degree=int(attract_deg),
        )
        bind_epoch(mcts_runner.dg, graph, propose_stats, group_id, transform)
        carry_max = int(getattr(cfg.propose, "graph_valid_carry_max", 512) or 512)
        if bool(getattr(cfg.propose, "enable_graph_valid_carry", True)):
            graph_valid_carry = graph_valid_carry_by_group(
                group_id, transform, ngroups=ngroups, max_keep=carry_max,
            )
        else:
            graph_valid_carry = tuple(
                np.zeros((0, 3), dtype=np.float64) for _ in range(ngroups)
            )
        propose_stats["graph_valid_n"] = int(len(transform))
        propose_stats["carry_n_next"] = int(
            sum(int(a.shape[0]) for a in graph_valid_carry)
        )
        prop_n = int(propose_stats.get("proposal_count", 0))
        proposal_keys = propose_stats.get("proposal_keys", {})
        proposal_nodes = sum(
            1
            for gid, t in zip(group_id, transform, strict=True)
            if transform_row_key(np.asarray(t, dtype=np.float64))
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
        board_ctx_outline = board_context_from_geometry(p_outline, user_holes=user_holes)
        seed_rules = active_rule_set(_make_seed_rule_sets(cfg))
        free_info = None
        # Ua: one improve call above first_pass / mid-pack fork.
        rule_sets = run_improve_rules_rounds(
            improve_rules,
            graphs=graphs,
            rule_sets=rule_sets,
            board=p_sheet,
            sel_iter=sel_iter,
            rng=rng,
            score_options=score_rules_options(sel_iter),
            mutation_presets=cfg.rules.mutation_presets(),
            rule_score_penalty=sel_iter.rule_score_penalty,
            max_rules_per_set=cfg.rules.max_rules_per_set,
        )
        if first_pass and cfg.propose.first_pass_border_pack:
            scores = score_elems(graph, seed_rules)
            sheet, _ = board_ctx_outline
            min_dist = cfg.board_min_dist_for(p_sheet, first_pass=True)
            selected_polys = _first_pass_border_ring_selection(
                graph, polys, p_sheet, min_dist, scores,
            )
            old_len = 0
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
                        propose_stats=propose_stats,
                        dg=mcts_runner.dg,
                    )
                )
            # R0: free_kind after first-pass rim (would Uh fire?).
            try:
                sheet_rim, void_geoms_uh = board_ctx_outline
                md_rim = cfg.board_min_dist_for(p_sheet, first_pass=True)
                rim_geoms = [
                    polys[i] for i in selected_polys
                    if 0 <= int(i) < len(polys)
                    and polys[i] is not None
                    and not polys[i].is_empty
                ]
                rim_pack = _native_geoms_from_transforms(
                    [group_id[i] for i in selected_polys],
                    [transform[i] for i in selected_polys],
                    part_bases,
                ) if selected_polys else []
                post_rim = schedule_prep_selection_free(
                    phase="uh",
                    sheet=sheet_rim,
                    part_areas=part_areas,
                    min_dist=md_rim,
                    cfg_propose=cfg.propose,
                    packed_shapely=rim_geoms,
                    pack_geoms=rim_pack,
                    run_uh=True,
                )
                if post_rim is None:
                    raise RuntimeError("uh free prep skipped")
                propose_stats["post_rim_free_kind"] = getattr(
                    post_rim.free_info, "kind", None
                )
                propose_stats["post_rim_uh_candidate"] = int(
                    propose_stats["post_rim_free_kind"] == "large_void"
                )
                print(
                    f"r0 post_rim free={propose_stats['post_rim_free_kind']} "
                    f"uh_candidate={propose_stats['post_rim_uh_candidate']}"
                )
                # Uh: unlocked dual beamed in compose; rim via packed/incumbent;
                # optional kiss≤3 lock arm (L2).
                if propose_stats["post_rim_uh_candidate"] and selected_polys:
                    active_rules_uh = active_rule_set(rule_sets)
                    scores_uh = list(score_elems(graph, active_rules_uh))
                    sheet_diag_uh = sheet_diag_from(sheet_rim)
                    cand_uh = _native_geoms_from_transforms(
                        group_id, transform, part_bases,
                    )
                    rim_idxs = list(selected_polys)
                    kiss_lock = kiss_lock_subset(
                        polys, p_sheet, md_rim, rim_idxs, max_n=3,
                    )
                    composed_uh = compose_and_nest_selection(
                        **compose_nest_kwargs(
                            graph=graph,
                            rule_sets=rule_sets,
                            active_rules=active_rules_uh,
                            scores=scores_uh,
                            polys=polys,
                            group_id=group_id,
                            transform=transform,
                            candidate_geoms=cand_uh,
                            packed_geoms=rim_pack,
                            part_areas=part_areas,
                            free_info=post_rim.free_info,
                            cfg=cfg,
                            selection=sel_iter,
                            first_pass=True,
                            outline=p_sheet,
                            min_dist=md_rim,
                            sheet_area=float(sheet_rim.area) if sheet_rim is not None else 0.0,
                            sheet_diag=sheet_diag_uh,
                            propose_stats=propose_stats,
                            ngroups=cfg.rules.ngroups,
                            packed_group_id=[
                                int(group_id[i]) for i in rim_idxs if int(i) < len(group_id)
                            ],
                            packed_transform=[
                                transform[i] for i in rim_idxs if int(i) < len(transform)
                            ],
                            last_leaf=True,
                            void_geoms=void_geoms_uh,
                            locked_seed=kiss_lock or None,
                            dg=mcts_runner.dg,
                        )
                    )
                    selected_polys = composed_uh.selected_nest
                    free_info = composed_uh.free_info
                    propose_stats["uh_post_rim"] = 1
                    propose_stats["uh_n_void"] = int(composed_uh.n_void_nest)
                    propose_stats["uh_n_sel"] = int(len(selected_polys))
                    propose_stats["uh_kiss_lock_n"] = int(len(kiss_lock))
                    print(
                        f"uh post_rim void_nest={composed_uh.n_void_nest} "
                        f"sel={len(selected_polys)} kiss_lock={len(kiss_lock)} "
                        f"beam_unlocked={propose_stats.get('uh_beam_unlocked', 0)}"
                    )
            except Exception:
                propose_stats["post_rim_free_kind"] = None
                propose_stats["post_rim_uh_candidate"] = 0
                propose_stats["uh_post_rim"] = 0
        else:
            active_rules = active_rule_set(rule_sets)
            scores = list(score_elems(graph, active_rules))
            sheet, void_geoms_compose = board_ctx_outline
            min_dist = cfg.board_min_dist_for(p_sheet, first_pass=first_pass)

            free_prep = schedule_prep_selection_free(
                phase="mid",
                sheet=sheet,
                part_areas=part_areas,
                min_dist=min_dist,
                cfg_propose=cfg.propose,
                nest_state=nest_state,
            )
            assert free_prep is not None
            free_info = free_prep.free_info
            packed_geoms = free_prep.packed_geoms
            packed_group_id = free_prep.packed_group_id
            packed_transform = free_prep.packed_transform
            free_prep_mid = free_prep
            sheet_diag = sheet_diag_from(sheet)
            candidate_geoms = _native_geoms_from_transforms(
                group_id, transform, part_bases,
            )
            dual_nest = dual_nest_for(free_info, last_leaf=is_last_leaf)
            composed = compose_and_nest_selection(
                **compose_nest_kwargs(
                    graph=graph,
                    rule_sets=rule_sets,
                    active_rules=active_rules,
                    scores=scores,
                    polys=polys,
                    group_id=group_id,
                    transform=transform,
                    candidate_geoms=candidate_geoms,
                    packed_geoms=packed_geoms,
                    part_areas=part_areas,
                    free_info=free_info,
                    cfg=cfg,
                    selection=sel_iter,
                    first_pass=first_pass,
                    outline=p_sheet,
                    min_dist=min_dist,
                    sheet_area=float(sheet.area) if sheet is not None else 0.0,
                    sheet_diag=sheet_diag,
                    propose_stats=propose_stats,
                    ngroups=cfg.rules.ngroups,
                    packed_group_id=packed_group_id,
                    packed_transform=packed_transform,
                    last_leaf=is_last_leaf,
                    void_geoms=void_geoms_compose,
                    dg=mcts_runner.dg,
                )
            )
            propose_stats["nest_dual"] = int(dual_nest)
            scores = composed.scores
            refine_scores = composed.refine_scores
            selected_nest = composed.selected_nest
            refine_rules = composed.refine_rules
            free_poly = composed.free_poly
            free_info = composed.free_info
            # Hybrid budget: mid+large_void enables 3b; cheap expand stays no-3b.
            polish_budget = polish_budget_for_iter(
                is_last_leaf=is_last_leaf,
                sel=sel,
                large_void=bool(
                    free_info is not None and free_info.kind == "large_void"
                ),
                cheap_expand=False,
                near_last=near_last,
            )
            n_void_nest = composed.n_void_nest
            boost_hits = composed.boost_hits
            old_len = len(selected_nest)
            node_areas = [
                float(part_areas[int(g)]) if int(g) < len(part_areas) else 0.0
                for g in group_id
            ]
            nest_before_refine = list(selected_nest)
            # Same oracle for rim_before/after (pre-refine nest coverage).
            try:
                nest_geoms = [
                    polys[i] for i in selected_nest
                    if 0 <= int(i) < len(polys)
                    and polys[i] is not None
                    and not polys[i].is_empty
                ]
                rim_before = float(outline_coverage_ratio(
                    nest_geoms,
                    p_sheet,
                    min_dist,
                    pack_geoms=_native_geoms_from_transforms(
                        [group_id[i] for i in selected_nest],
                        [transform[i] for i in selected_nest],
                        part_bases,
                    ) if selected_nest else None,
                ))
            except Exception:
                rim_before = float(propose_stats.get("outline_cov", 0.0) or 0.0)
            rim_reject = float(getattr(cfg.propose, "refine_rim_drop_reject", 0.02) or 0.0)
            selected_polys = apply_refine_with_restore(
                budget=polish_budget,
                apply_dfs_fn=apply_dfs_refinement,
                graph=graph,
                refine_rules=refine_rules,
                selected_nest=selected_nest,
                refine_scores=refine_scores,
                sel_iter=sel_iter,
                node_areas=node_areas,
                refine_seed=int(rng.integers(0, 2**31)),
                locked_indices=list(propose_stats.get("motif_locked") or []),
                polys=polys,
                group_id=group_id,
                transform=transform,
                part_areas=part_areas,
                part_bases=part_bases,
                sheet=p_sheet,
                min_dist=min_dist,
                rim_before=rim_before,
                rim_reject=rim_reject,
                propose_stats=propose_stats,
                native_geoms_from_transforms_fn=_native_geoms_from_transforms,
                free_info=free_info,
                free_poly=free_poly,
            )
            # DgP: warm multi-sim cache for next-iter cheap_pack expands.
            _pack_cache.clear()
            _pack_cache.update({
                "ready": True,
                "graph": mcts_runner.dg.poses(),
                "dg": mcts_runner.dg,
                "polys": polys,
                "group_id": group_id,
                "transform": transform,
                "part_areas": part_areas,
                "part_bases": part_bases,
                "p_sheet": p_sheet,
                "min_dist": min_dist,
                "cfg": cfg,
                "free_info": free_info,
                "propose_stats": dict(propose_stats),
                "packed_geoms": list(packed_geoms),
                "packed_group_id": packed_group_id,
                "packed_transform": packed_transform,
                "void_geoms": list(void_geoms_compose or []),
                "sheet": sheet,
                "sheet_area": float(sheet.area) if sheet is not None else 0.0,
                "board_area": float(board_area),
                "selected": list(selected_polys),
            })
            mcts_telem["last_graph_n"] = int(len(transform))
            mcts_telem["last_nest_n"] = int(len(selected_polys))
            del nest_before_refine
            propose_stats["motif_sequential_repin"] = 0
            # 3b: plateau hole re-nest (replaces void-kNN leftover). Skip if 3a accepted.
            track_d = bool(large_void_motif_plateau.ready)
            propose_stats["large_void_motif_plateau"] = track_d
            propose_stats["large_void_motif_plateau_streak"] = int(
                large_void_motif_plateau.streak
            )
            propose_stats.setdefault("block_hole_accepted", 0)
            propose_stats.setdefault("block_hole_emit_in_hull", 0)
            propose_stats.setdefault("block_hole_victim", None)
            skip_3b = int(propose_stats.get("block_cohort_accepted", 0) or 0) > 0
            # 3b gated solely by polish_budget.run_3b (G1 hybrid SoT).
            allow_3b = bool(polish_budget.run_3b)
            if (
                allow_3b
                and not skip_3b
                and bool(getattr(cfg.propose, "enable_lns_rebuild", True))
                and selected_polys
                and free_info is not None
                and free_info.kind == "large_void"
            ):
                _sheet_b, void_geoms_b = board_ctx_outline
                del _sheet_b
                (
                    selected_polys,
                    polys,
                    transform,
                    group_id,
                    candidate_geoms,
                    hole_telem,
                ) = try_block_hole_renest(
                    selected=selected_polys,
                    polys=list(polys),
                    transforms=list(transform),
                    group_id=list(group_id),
                    candidate_geoms=list(candidate_geoms) if candidate_geoms is not None else None,
                    scores=refine_scores,
                    part_areas=part_areas,
                    part_by_group={0: p1, 1: p2},
                    sheet=p_sheet,
                    min_dist=min_dist,
                    propose_cfg=cfg.propose,
                    pole=getattr(free_info, "target_pt", None),
                    void_poly=free_poly,
                    void_geoms=void_geoms_b,
                    cluster_patterns=archived_for_propose or None,
                )
                propose_stats.update(hole_telem)
            pin_stats: dict = {}
            skip_pin = (
                int(getattr(cfg.propose, "pin_all_blocked_skip_after", 3) or 0) > 0
                and pin_all_blocked_streak
                >= int(getattr(cfg.propose, "pin_all_blocked_skip_after", 3) or 0)
            )
            if (
                bool(getattr(cfg.propose, "enable_void_nest_pin", True))
                and not skip_pin
                and free_poly is not None
                and not free_poly.is_empty
            ):
                n_graph = len(graph.collisions)
                extra_3b = [i for i in selected_polys if int(i) >= n_graph]
                selected_polys = pin_nest_void_independent(
                    graph,
                    selected_nest,
                    [i for i in selected_polys if int(i) < n_graph],
                    polys,
                    free_poly,
                    refine_scores,
                    stats_out=pin_stats,
                )
                if extra_3b:
                    selected_polys = list(selected_polys) + extra_3b
            else:
                pin_stats = {
                    "pin_candidates": 0,
                    "pin_added": 0,
                    "pin_blocked_collision": 0,
                    "pin_ms": 0.0,
                    "pin_skipped_streak": int(skip_pin),
                }
            pin_cands = int(pin_stats.get("pin_candidates", 0))
            pin_added = int(pin_stats.get("pin_added", 0))
            if pin_cands > 0 and pin_added == 0:
                pin_all_blocked_streak += 1
            elif not skip_pin:
                pin_all_blocked_streak = 0
            proposed_map = propose_stats.get("proposed_by_group") or {}
            proposed_list = (
                [proposed_map[g] for g in sorted(proposed_map)]
                if proposed_map else None
            )
            n_props_pole = count_props_near_pole(
                proposed_list, free_info.target_pt, void_pole_near_radius(
                    sheet_diag,
                    float(getattr(cfg.propose, "void_pole_near_diag_ratio", 0.25) or 0.25),
                ),
            )
            # Archive refine-rejected / pin-blocked void transforms for next-iter seeds.
            archive_enabled = bool(
                getattr(cfg.propose, "enable_void_elite_archive", True)
            )
            if (
                archive_enabled
                and bool(getattr(cfg.propose, "stop_elite_archive_when_pin_blocked", True))
                and pin_cands > 0
                and pin_added == 0
            ):
                densify_keep = int(
                    (propose_stats.get("densify_stats") or {}).get("accepted", 0) or 0
                ) > 0
                if not densify_keep and int(n_props_pole) <= 0:
                    archive_enabled = False
            void_elite_by_group = archive_void_elite_transforms(
                selected_nest=selected_nest,
                selected_refine=selected_polys,
                polys=polys,
                transforms=transform,
                group_ids=group_id,
                free_poly=free_poly,
                scores=refine_scores,
                max_keep=int(getattr(cfg.propose, "stratified_void_elite_quota", 15)),
                enabled=archive_enabled,
            )
            n_void_refine = count_selected_in_free(
                polys, selected_polys, free_poly,
            )
            n_void_props = count_props_in_free(proposed_list, free_poly)
            n_void_graph = count_graph_in_free(polys, free_poly)
            zones = propose_stats.get("zones_used") or []
            hijack = int(zones_have_void_hijack(zones))
            outline_cov = float(propose_stats.get("outline_cov", 0.0))
            sat_override = int(bool(propose_stats.get("sat_override", False)))
            if hijack or sat_override:
                had_void_override = True
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
            hijack_over = int(densify.get("void_hijack_over_mcts", 0) or 0)
            if hijack_over:
                mcts_telem["void_hijack_over_mcts"] = int(
                    mcts_telem.get("void_hijack_over_mcts", 0)
                ) + hijack_over
            pocket_skip = densify.get("pocket_skip") or propose_stats.get("pocket_skip") or []
            if isinstance(pocket_skip, str):
                pocket_skip = [pocket_skip]
            pocket_key_hits = int(boost_hits.get("pocket_keys", 0))
            motif_key_hits = int(boost_hits.get("motif_keys", 0))
            small_hits = int(boost_hits.get("small_part", 0))
            island_hits = int(boost_hits.get("void_island", 0))
            nest_refine_delta = int(n_void_nest) - int(n_void_refine)
            skip_snip = ",".join(str(s) for s in list(pocket_skip)[:4])
            proposer_keys = propose_stats.get("proposer_keys") or densify.get("proposer_keys") or {}
            emitted_bp = dict(propose_stats.get("emitted_by_proposer") or {})
            for name, n in (densify.get("emitted_by_proposer") or {}).items():
                emitted_bp[name] = max(int(emitted_bp.get(name, 0)), int(n))
            if not emitted_bp and densify.get("emitted_by_proposer"):
                emitted_bp = dict(densify.get("emitted_by_proposer") or {})
            pool_bp = dict(propose_stats.get("pool_by_proposer") or {})
            for name, n in (densify.get("pool_by_proposer") or {}).items():
                pool_bp[name] = max(int(pool_bp.get(name, 0)), int(n))
            if not pool_bp and densify.get("pool_by_proposer"):
                pool_bp = dict(densify.get("pool_by_proposer") or {})
            nest_bp = count_selected_by_proposer(transform, selected_nest, proposer_keys)
            refine_bp = count_selected_by_proposer(transform, selected_polys, proposer_keys)
            prop_accept = format_prop_accept(emitted_bp, pool_bp, nest_bp, refine_bp)
            pocket_by_tag = dict(propose_stats.get("pocket_by_tag") or densify.get("pocket_by_tag") or {})
            elite_seeded = int(propose_stats.get("void_elite_seeded", 0))
            elite_next = void_elite_count(void_elite_by_group)
            cc_e = int(emitted_bp.get("cluster_copy", 0))
            cc_p = int(pool_bp.get("cluster_copy", 0))
            if cc_e > 0 or cc_p > 0:
                mcts_telem["place_motif_ok"] = int(mcts_telem.get("place_motif_ok", 0)) + 1
            sp_e = int(emitted_bp.get("side_pack", 0))
            sp_p = int(pool_bp.get("side_pack", 0))
            fsc_e = int(emitted_bp.get("free_space_cloud", 0))
            fsc_p = int(pool_bp.get("free_space_cloud", 0))
            n_patterns = int(densify.get("cluster_patterns", 0))
            attract_edges = int(propose_stats.get("attract_edges", 0) or 0)
            if attract_edges <= 0:
                attract_edges = sum(len(row) for row in graph.attract) // 2
            sel_set = set(int(i) for i in selected_polys)
            attract_pairs_selected = 0
            attract_bonus = 0.0
            for i in sel_set:
                if i < 0 or i >= len(graph.attract):
                    continue
                for e in graph.attract[i]:
                    j = int(e.target)
                    if j > i and j in sel_set:
                        attract_pairs_selected += 1
                        attract_bonus += float(e.w)
            if mcts_runner.agent is not None:
                mcts_telem["amaf_hits"] = int(
                    mcts_runner.agent.telem.get("amaf_hits", 0) or 0
                )
                mcts_telem["amaf_miss"] = int(
                    mcts_runner.agent.telem.get("amaf_miss", 0) or 0
                )
            propose_stats["browse_jump"] = int(mcts_telem.get("browse_jump", 0) or 0)
            propose_stats["amaf_miss"] = int(mcts_telem.get("amaf_miss", 0) or 0)
            propose_stats["amaf_hits"] = int(mcts_telem.get("amaf_hits", 0) or 0)
            propose_stats["motif_nest_credit"] = int(
                mcts_telem.get("motif_nest_credit", 0) or 0
            )
            propose_stats["niche_hits"] = int(mcts_runner.niche_archive.total_hits())
            void_leak, leak_dict = assemble_void_leak(
                free_kind=free_info.kind,
                max_void_ratio=float(free_info.max_void_ratio),
                n_void_props=n_void_props,
                n_props_pole=n_props_pole,
                hijack=hijack,
                n_void_graph=n_void_graph,
                n_void_nest=n_void_nest,
                n_void_refine=n_void_refine,
                nest_refine_delta=nest_refine_delta,
                outline_cov=outline_cov,
                sat_override=sat_override,
                rim_progress=rim_progress,
                plateau=plateau,
                pin_all_blocked_streak=pin_all_blocked_streak,
                propose_stats=propose_stats,
                zones=zones,
                pf_em=pf_em,
                pf_att=pf_att,
                pf_sel=pf_sel,
                pin_stats=pin_stats,
                pf_surv=pf_surv,
                pocket_key_hits=pocket_key_hits,
                motif_key_hits=motif_key_hits,
                island_hits=island_hits,
                small_hits=small_hits,
                densify_f=densify_f,
                densify_a=densify_a,
                densify_reason=densify_reason,
                pocket_skip=pocket_skip,
                emitted_bp=emitted_bp,
                pool_bp=pool_bp,
                nest_bp=nest_bp,
                refine_bp=refine_bp,
                pocket_by_tag=pocket_by_tag,
                prop_accept=prop_accept,
                densify=densify,
                elite_seeded=elite_seeded,
                elite_next=elite_next,
                cc_e=cc_e,
                cc_p=cc_p,
                sp_e=sp_e,
                sp_p=sp_p,
                fsc_e=fsc_e,
                fsc_p=fsc_p,
                n_patterns=n_patterns,
                attract_edges=attract_edges,
                attract_pairs_selected=attract_pairs_selected,
                attract_bonus=attract_bonus,
                zone_snip=zone_snip,
                skip_snip=skip_snip,
            )
            propose_stats["void_leak"] = leak_dict
            funnel = leak_dict["funnel"]
            corners = leak_dict["corners"]
            propose_stats["r0_bottleneck"] = funnel["bottleneck"]
            print(
                f"r0 bottleneck={funnel['bottleneck']} "
                f"funnel={funnel['funnel_stages']} "
                f"corners_in={corners['corner_in']} kept={corners['corner_kept']}"
            )
            print(void_leak)
            # Niche archive + AMAF miss (L1) after funnel is known.
            bottleneck = str(funnel.get("bottleneck") or "")
            large_void = str(free_info.kind or "") == "large_void"
            hollow = bool(
                large_void
                and (
                    bottleneck == "graph_to_nest"
                    or int(n_void_nest) <= 0
                )
            )
            propose_stats["hollow_miss"] = hollow
            amaf_key = niche_amaf_key(mcts_action)
            if hollow and mcts_runner.agent is not None:
                mcts_runner.agent.note_macro_miss(mcts_action)
                # Q113: Motif miss via MotifBase.note_hollow_miss (no .at() copy).
                if (
                    mcts_action is not None
                    and mcts_action.region == MacroRegion.Motif
                    and int(mcts_action.motif_id) >= 0
                ):
                    note_motif_hollow_miss(
                        mcts_runner.motif_base, int(mcts_action.motif_id),
                    )
            niche_telem = credit_void_niche_from_iter(
                mcts_runner.niche_archive,
                free_kind=str(free_info.kind or ""),
                bottleneck=bottleneck,
                n_void_nest=int(n_void_nest),
                n_void_graph=int(n_void_graph),
                prev_void_nest=int(_prev_void_nest),
                polys=polys,
                transform=transform,
                group_id=group_id,
                selected=selected_nest,
                free_poly=free_poly,
                outline_cov=float(outline_cov),
                ttl=int(getattr(cfg.propose, "accepted_pattern_ttl", 4) or 4),
                max_seed=int(getattr(cfg.propose, "max_proposals", 64) or 64),
                amaf_key=amaf_key,
                proposer_keys=proposer_keys,
            )
            propose_stats.update(niche_telem)
            if isinstance(propose_stats.get("void_leak"), dict):
                propose_stats["void_leak"]["niche_pos"] = int(
                    niche_telem.get("niche_pos", 0)
                )
                propose_stats["void_leak"]["niche_rescue"] = int(
                    niche_telem.get("niche_rescue", 0)
                )
            _prev_void_nest = int(n_void_nest)
            # Q141: archive seed place outcome
            feed = mcts_runner.niche_archive.last_feed_keys
            if feed and large_void:
                nest_keys = {
                    transform_row_key(np.asarray(transform[i], dtype=np.float64))
                    for i in selected_nest
                    if 0 <= int(i) < len(transform)
                }
                placed = bool(feed & nest_keys)
                mcts_runner.niche_archive.note_place_outcome(placed)
            mcts_runner.niche_archive.last_feed_keys = set()
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
            refine_bp = count_selected_by_proposer(
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
        n_graph = len(graph.collisions)
        assert selection_is_independent(
            graph, [i for i in selected_polys if 0 <= int(i) < n_graph],
        )
        min_dist = cfg.board_min_dist_for(p_sheet, first_pass=first_pass)
        sheet_compact, void_geoms_post = board_ctx_outline
        part_by_group = {0: p1, 1: p2}
        seed_n = nest_state.seed_count if nest_state is not None else 0
        fixed_obs = list(nest_state.polys[:seed_n]) if nest_state is not None and seed_n else None
        relocate_accepted = False
        # Post-DFS pack when mid-pack free is still a large void.
        sel_geoms = [
            polys[i] for i in selected_polys
            if polys[i] is not None and not polys[i].is_empty
        ]
        sel_pack_geoms = _native_geoms_from_transforms(
            [group_id[i] for i in selected_polys],
            [transform[i] for i in selected_polys],
            part_bases,
        ) if selected_polys else []
        free_prep_post = schedule_prep_selection_free(
            phase="post",
            sheet=sheet_compact,
            part_areas=part_areas,
            min_dist=min_dist,
            cfg_propose=cfg.propose,
            packed_shapely=sel_geoms,
            pack_geoms=sel_pack_geoms,
            prior=free_prep_mid,
            selection_changed=True,
        )
        assert free_prep_post is not None
        free_snap_post = free_prep_post.free_snap
        free_post = free_prep_post.free_info
        mean_part_post = free_prep_post.mean_part
        void_leak_stats = propose_stats.get("void_leak") if propose_stats else None
        allow_repack = True
        if isinstance(void_leak_stats, dict):
            allow_repack = allow_void_repack(
                free_kind=void_leak_stats.get("free_kind")
                or (free_info.kind if free_info is not None else None),
                n_void_nest=int(void_leak_stats.get("nest", 0)),
                n_void_refine=int(void_leak_stats.get("refine", 0)),
            )
        elif free_info is not None:
            allow_repack = allow_void_repack(
                free_kind=free_info.kind,
                n_void_nest=0,
                n_void_refine=0,
            )
        hole_ok = int((propose_stats or {}).get("block_hole_accepted", 0) or 0) > 0
        stamp_victim = (propose_stats or {}).get("block_hole_victim")
        if hole_ok:
            allow_repack = False
        if free_post.kind == "large_void" and free_post.target_pt is not None:
            push_pt = free_post.target_pt
            reloc_poles: list = [free_post.target_pt]
            if (
                bool(cfg.propose.use_multi_pole_void)
                and free_post.target_poly is not None
                and not free_post.target_poly.is_empty
            ):
                xy_poles = iterative_multi_poles(
                    free_post.target_poly,
                    min_dist=min_dist,
                    max_poles=int(cfg.propose.multi_pole_max_poles),
                )
                if xy_poles:
                    reloc_poles = [Point(x, y) for x, y in xy_poles]
                    push_pt = reloc_poles[0]
            void_leak_prev = propose_stats.get("void_leak")
            if isinstance(void_leak_prev, dict):
                void_leak_prev["multi_pole_count"] = len(reloc_poles)
            if polish_budget.run_post_pack:
                polys, transform, selected_polys, pack_stats = run_post_pack_passes(
                    sheet_compact,
                    list(polys),
                    list(transform),
                    group_id,
                    selected_polys,
                    part_by_group,
                    min_dist,
                    cfg.propose,
                    pole=push_pt,
                    poles=reloc_poles,
                    fixed_obstacles=fixed_obs,
                    void_geoms=void_geoms_post,
                    allow_repack=allow_repack,
                    allow_relocate=True,
                    allow_local_se2=True,
                    void_poly=free_post.target_poly,
                    pt_push=push_pt,
                    free_space=free_snap_post,
                    victim_indices=None if hole_ok else stamp_victim,
                    part_bases=part_bases,
                    refresh_after_repack=True,
                    mean_part_area=mean_part_post,
                    native_pack_geoms_fn=lambda sel, tr, gids: (
                        _native_geoms_from_transforms(
                            [gids[i] for i in sel],
                            [tr[i] for i in sel],
                            part_bases,
                        ) if sel else None
                    ),
                )
            else:
                pack_stats = {}
            repack_stats = pack_stats.get("repack") or {
                "attempted": 0,
                "accepted": 0,
                "motif_accepted": 0,
                "skipped_refine_zero": int(not allow_repack),
            }
            reloc_stats = pack_stats.get("relocate") or {
                "attempted": 0, "accepted": 0, "moved": 0,
            }
            se2_stats = pack_stats.get("local_se2") or {
                "attempted": 0, "moved": 0, "tangent_moves": 0,
            }
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
        # MotifBase TTL + Q116 nest Motif credit + global age (Q142).
        pattern_accept = False
        motif_refine_n = 0
        motif_ttl = int(getattr(cfg.propose, "accepted_pattern_ttl", 4) or 4)
        if selected_polys and group_id is not None and transform is not None:
            motif_keys_arch = propose_stats.get("motif_keys") or {}
            for i in selected_polys:
                gi = int(i)
                if gi < 0 or gi >= len(group_id) or gi >= len(transform):
                    continue
                gid = int(group_id[gi])
                key = transform_row_key(transform[gi])
                if key in (motif_keys_arch.get(gid) or set()):
                    motif_refine_n += 1
            repack_m = int((propose_stats.get("repack") or {}).get("motif_accepted", 0))
            if motif_refine_n > 0 or repack_m > 0:
                pattern_accept = True
            materialize_selection(
                mcts_runner.dg, selected_polys, propose_stats,
            )
            # Q116/Q165: Motif accept + Kind/Attach scalars (outer leaf only).
            if mcts_runner.agent is not None:
                credit_motif_on_nest_survival(
                    mcts_runner.motif_base,
                    selected_polys=selected_polys,
                    group_id=group_id,
                    transform=transform,
                    motif_keys=motif_keys_arch,
                    ttl=motif_ttl,
                    telem=mcts_telem,
                    realized_out=mcts_runner.agent.realized,
                    kind_survive=propose_stats.get("kind_survive_hist"),
                    materialized_attach=int(
                        propose_stats.get("materialized_attach", 0) or 0
                    ),
                    member_hits=int(propose_stats.get("member_hits", 0) or 0),
                    credit_motif=int(motif_refine_n) > 0,
                )
            if isinstance(propose_stats.get("void_leak"), dict):
                propose_stats["void_leak"]["kind_survive"] = int(
                    propose_stats.get("kind_survive", 0) or 0
                )
                propose_stats["void_leak"]["materialized_attach"] = int(
                    propose_stats.get("materialized_attach", 0) or 0
                )
        # Q142: one global age tick — Motif + niche + arena idle/tombstone.
        if bool(getattr(cfg.propose, "enable_accepted_pattern_archive", True)):
            if not pattern_accept:
                age_motif_library(mcts_runner.motif_base, 1)
            mcts_runner.niche_archive.age(1)
            if mcts_runner.agent is not None:
                mcts_runner.agent.age_and_tombstone(
                    spine_id=int(mcts_parent_id),
                    idle_t=4,
                    force=bool(
                        str(mcts_telem.get("ram_band") or "") in ("pressure", "critical")
                    ),
                )
                # Tombstones stay on the agent; snapshots stay size-aligned with the arena.
        lib_n = int(mcts_runner.motif_base.size())
        propose_stats["motif_refine_hits"] = int(motif_refine_n)
        propose_stats["accepted_patterns_archived"] = lib_n
        propose_stats["motif_library_n"] = lib_n
        propose_stats["pattern_accept"] = bool(pattern_accept)
        densify_full = propose_stats.get("densify_stats") or {}
        skip_map: dict[str, int] = {}
        densify_skips = densify_full.get("pocket_skip") or []
        if isinstance(densify_skips, dict):
            skip_map = {str(k): int(v) for k, v in densify_skips.items()}
        elif isinstance(densify_skips, list):
            for s in densify_skips:
                skip_map[str(s)] = skip_map.get(str(s), 0) + 1
        for k, v in (densify_full.get("motif_skip") or {}).items():
            skip_map[str(k)] = int(v)
        motif_telem = build_motif_telem(
            skip_map=skip_map,
            motif_refine_n=motif_refine_n,
            propose_stats=propose_stats,
            archive_len=lib_n,
        )
        motif_telem["motif_library_n"] = lib_n
        motif_telem["motif_library_inject"] = int(
            propose_stats.get("accepted_patterns_n", 0)
        )
        if isinstance(propose_stats.get("void_leak"), dict):
            propose_stats["void_leak"]["motif_telem"] = motif_telem
        propose_stats["motif_telem"] = motif_telem
        cov = _selection_coverage_pct(
            selected_polys, group_id, part_areas, board_area,
        )
        # Macro-MCTS: record expand reward (Q68/Q69); upsert contact motifs when improved.
        if mcts_action is not None and mcts_runner.agent is not None:
            child_snap = board_snapshot_from_selection(
                selected_polys=selected_polys,
                group_id=group_id,
                transform=transform,
                ngroups=int(cfg.rules.ngroups),
                coverage_pct=float(cov),
                propose_stats=propose_stats,
                mcts_action=mcts_action,
                mcts_telem=mcts_telem,
            )
            mcts_parent_id = record_mcts_expand(
                mcts_runner,
                parent_id=mcts_parent_id,
                action=mcts_action,
                child_snap=child_snap,
                nest_state=nest_state,
                part_bases=part_bases,
                part_areas=part_areas,
                min_dist=float(cfg.board_min_dist_for(p_sheet)),
                t_expand0=t_expand0,
                mcts_telem=mcts_telem,
                propose_stats=propose_stats,
                motif_min_compactness=float(
                    getattr(cfg.propose, "motif_min_compactness", 0.35) or 0.35
                ),
                motif_ttl=(
                    int(getattr(cfg.propose, "accepted_pattern_ttl", 4) or 4)
                    if pattern_accept
                    else 0
                ),
                motif_max_keep=int(
                    getattr(cfg.propose, "accepted_pattern_max", 4) or 4
                ),
            )
            _exec_last["snap"] = child_snap
            if isinstance(propose_stats.get("void_leak"), dict):
                propose_stats["void_leak"]["contact_grg_upserts"] = int(
                    mcts_telem.get("contact_grg_upserts", 0) or 0
                )
        plateau.update(cov, len(selected_polys))
        refine_bp_plateau = {}
        if isinstance(propose_stats.get("void_leak"), dict):
            refine_bp_plateau = dict(
                propose_stats["void_leak"].get("refine_by_proposer") or {}
            )
        if not refine_bp_plateau:
            densify_tmp = propose_stats.get("densify_stats") or {}
            pk = (
                propose_stats.get("proposer_keys")
                or densify_tmp.get("proposer_keys")
                or {}
            )
            refine_bp_plateau = count_selected_by_proposer(
                transform, selected_polys, pk,
            )
        free_kind_plateau = None
        if "free_post" in locals() and free_post is not None:
            free_kind_plateau = getattr(free_post, "kind", None)
        elif free_info is not None:
            free_kind_plateau = getattr(free_info, "kind", None)
        elif isinstance(propose_stats.get("void_leak"), dict):
            free_kind_plateau = propose_stats["void_leak"].get("free_kind")
        large_void_motif_plateau.update(
            free_kind=free_kind_plateau,
            cov=cov,
            cluster_copy_refine=int(refine_bp_plateau.get("cluster_copy", 0) or 0),
        )
        propose_stats["large_void_motif_plateau"] = bool(large_void_motif_plateau.ready)
        propose_stats["large_void_motif_plateau_streak"] = int(
            large_void_motif_plateau.streak
        )
        # M1: sticky Scene dry-run whenever Motif patterns are in play (not only Q27).
        want_dry = bool(
            large_void_motif_plateau.ready
            or int(propose_stats.get("accepted_patterns_n", 0) or 0) > 0
            or int(mcts_runner.motif_base.size()) > 0
        )
        if bool(getattr(cfg.propose, "enable_motif_scene_dry_run", False)) != want_dry:
            cfg = cfg.model_copy(update={
                "propose": cfg.propose.model_copy(update={
                    "enable_motif_scene_dry_run": want_dry,
                }),
            })
        if propose_stats.get("void_leak") is not None:
            propose_stats["void_leak"]["on_plateau"] = bool(plateau.on_plateau)
            propose_stats["void_leak"]["plateau_streak"] = int(plateau.streak)
            propose_stats["void_leak"]["coverage"] = float(cov)
            propose_stats["void_leak"]["parts"] = int(len(selected_polys))
            propose_stats["void_leak"]["large_void_motif_plateau"] = bool(
                large_void_motif_plateau.ready
            )
        if out.progress:
            refine_pf = f"{old_len}->{len(selected_polys)}"
            pbar.set_postfix(
                parts=len(selected_polys),
                cov=f"{cov:.1f}%",
                pool=len(polys),
                refine=refine_pf,
                dfs=int(polish_budget.dfs_passes),
                ordered=True,
            )
        else:
            refine_msg = f"{old_len} -> {len(selected_polys)}"
            print(
                len(polys), refine_msg,
                f"cov={cov:.1f}%",
                f"dfs_passes={int(polish_budget.dfs_passes)}",
                f"mcts_heavy={int(polish_budget.mcts_heavy)}",
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
            # Carry keeps last make_polygon_graph board-valid rows (incl. unselected
            # swap material). Do not collapse to selected_t — local_se2 moves most
            # iters and would erase the propose-dip stabilizer.
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
        seed_n = nest_state.seed_count if nest_state is not None else 0
        nest_state = NestState(
            polys=polys,
            group_id=group_id,
            transform=transform,
            selected_indices=list(selected_polys),
            seed_count=seed_n,
            _native_geoms=_native_geoms_from_transforms(
                group_id,
                transform,
                part_bases,
                seed_polys=polys,
                seed_count=seed_n,
            ),
        )
        rule_sets = _inject_repulsor_rules(rule_sets, cfg, p_sheet, nest_state)

    # Q84: hard stop if final selection is not packing-independent.
    if nest_state is not None and nest_state.selected_indices:
        if not selection_pairwise_independent(
            nest_state.polys, nest_state.selected_indices
        ):
            raise RuntimeError(
                "Macro-MCTS cutover: final selection failed independent_ok (Q84)"
            )

    video.release()


def test_placement():
    p_board = Polygon([(0, 0), (1.2, 0), (0, 1.1)])
    p1 = normalize_poly(Polygon([(0, 0), (.15, 0), (0, .07)]))
    p2 = normalize_poly(Polygon([(0, 0), (.1, 0), (.1, .1), (0, .1)]))

    p1_result = []
    p2_result = []
    base_shape = Polygon()
    for _ in range(100):
        geom1 = ProposeGeometry(p_board, base_shape, p1, min_dist=0.001)
        p1_places = propose_placements_point_cloud(
            base_shape, p1, p_board, min_dist=0.001, pt_push=p_board.centroid,
            top_n=100, propose_geom=geom1,
        )
        print('p1', len(p1_places))
        if p1_places:
            p1_result.append(p1_places[0])
            base_shape = unary_union([base_shape, transform_poly(p1, p1_places[0])])
        geom2 = ProposeGeometry(p_board, base_shape, p2, min_dist=0.001)
        p2_places = propose_placements_point_cloud(
            base_shape, p2, p_board, min_dist=0.001, pt_push=p_board.centroid,
            top_n=100, propose_geom=geom2,
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
