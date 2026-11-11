# CFLAGS="-Wno-error=incompatible-pointer-types" pip install --force-reinstall --no-binary=shapely --upgrade shapely

import cv2 as cv
import numpy as np
import os
import time
import zlib
from dataclasses import dataclass, field
from pydantic import BaseModel, ConfigDict
from shapely import Polygon
from shapely.geometry import box as shapely_box
from shapely.geometry import Point
from shapely.geometry.base import BaseGeometry
from tqdm import tqdm
from typing import Any, Iterator, NamedTuple, Sequence, Tuple

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
from nest_graph.placement.scene import (
    guidance_config_for_graph,
    placement_scene_for_part,
)
from nest_graph.placement.common import as_geometry, selection_pairwise_independent
from .board import (
    board_context_from_geometry,
    default_sheet_padding,
    padded_board_bounds,
)
from .utils import transform_poly
from .track_perf import show_performance
from .propose.feedback import ProposeFeedbackState
from .propose.post_pack import prepare_post_pack
from .propose.pattern_archive import (
    motif_patterns_for_inject,
    note_motif_ref_anchors,
    note_motif_ref_anchors_from_nest,
)
from .propose.context import (
    part_is_concave,
)
from .propose.telem import (
    VoidLeakGatherCtx,
    void_elite_count,
    void_elite_tuple_from_archive,
)
from nest_graph.pack.cheap import (
    maybe_invalidate_cheap_cache,
    pack_execute_snapshot,
    warm_outer_pack_cache,
)
from nest_graph.pack.ctx import PackIterCtx, RefinePackBox
from nest_graph.pack.credit import (
    age_iter_archives,
    finalize_iter_mcts,
    run_void_leak_and_niche_credit,
)
from nest_graph.telem_log import configure_telem_log, write_telem
from nest_graph.pack.geoms import (
    native_geoms_from_transforms as _native_geoms_from_transforms,
    selection_coverage_pct as _selection_coverage_pct,
)
from nest_graph.pack.stages import (
    run_first_pass_border_pack,
    run_mid_pack_stages,
    run_post_pack_stage,
)
from nest_graph.rules.evolve import (
    blend_live_mutation_presets,
    credit_mut_slot_amaf_deltas,
    improve_rules,
    inject_repulsor_rules,
    make_initial_rule_sets,
    make_seed_rule_sets,
    write_mut_live_telem,
)
from .propose.heavy_polish import (
    apply_dfs_refinement,
    polish_budget_for_iter,
    run_improve_rules_rounds,
)
from .propose.context import (
    late_border_saturation_info,
)
from .propose.first_pass_border import (
    build_pose_graph,
    join_attract_pairs,
)
from .propose.void_selection import (
    count_selected_by_proposer,
    transform_row_key,
)
from .propose.selection_compose import (
    active_rule_set,
    dual_nest_for,
    sheet_diag_from,
)
from .propose.transform_batch import (
    build_transform_batch,
)
from .propose.motif_lock import LargeVoidMotifPlateau
from nest_graph.placement.pattern import seed_motif_base_from_mates
from .propose.telem import (
    BestPackSnapshot,
    best_pack_geom_sig,
    maybe_restore_best_pack,
)
from nest_graph.free_kind_util import free_kind_token, is_large_void
from nest_graph.graph import BoardSnapshot, FreeKind, region_to_zone
from nest_graph.place_zone_util import is_void_seek, place_zone_token
from nest_graph.pack.browse import (
    hold_browse_tip,
    install_iter_tip,
    should_browse_tip,
)
from nest_graph.pack.epoch import (
    inject_cohorts_and_bind_graph,
    sync_agent_motif_cohorts,
)
from nest_graph.pack.execute import (
    ensure_motif_tip_for_fuel,
    make_execute_fn,
    mcts_rule_ids,
    prep_selection_freeze,
    record_outer_iter_expand,
    schedule_prep_selection_free,
    seed_parent_free_and_poles,
)
from nest_graph.pack.macro_path import run_outer_path_probe
from nest_graph.graph import leaf_reward
from nest_graph.pack.motif_credit import (
    merge_void_elite_with_archive,
)
from nest_graph.pack.ram_budget import evaluate_ram_band
from nest_graph.pack.runner import MacroMctsRunner
from nest_graph.pack.checkpoint import (
    carry_tuple_from_ckpt,
    history_tuple_from_ckpt,
    load_nest_checkpoint,
    nest_state_from_checkpoint,
    rehydrate_motif_base,
    rehydrate_niche_archive,
    save_nest_checkpoint,
    tip_board_snapshot,
    void_elite_from_ckpt,
)
from nest_graph.pack.densify_preset import (
    apply_densify_preset,
    densify_preset_ids,
    load_densify_presets,
)
from nest_graph.pack.slave_pack import (
    upsert_from_repack_accept,
    write_propose_kiss_ledger,
)
from .graph import (
    PoseGraph,
    PlacementRuleSet,
    nest_by_graph, nest_by_scores, score_elems, score_rules,
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


def nest_state_from_proven_compose(
    proven: Any,
    *,
    seed_count: int,
    pack_cache: dict,
) -> NestState | None:
    """S2: build a new NestState from a stashed cheap compose row (or None)."""
    if not isinstance(proven, dict):
        return None
    sel = [int(i) for i in (proven.get("compose_sel") or ())]
    if not sel:
        return None
    polys = list(proven.get("compose_polys") or ())
    if not polys:
        polys = list(pack_cache.get("polys") or ())
    group_id = list(proven.get("compose_group_id") or ())
    if not group_id:
        group_id = list(pack_cache.get("group_id") or ())
    transform = list(proven.get("compose_transform") or ())
    if not transform:
        transform = list(pack_cache.get("transform") or ())
    if not polys or not group_id or not transform:
        return None
    if max(sel) >= len(polys) or max(sel) >= len(group_id) or max(sel) >= len(transform):
        return None
    if not selection_pairwise_independent(polys, sel):
        return None
    geoms = list(proven.get("compose_geoms") or ())
    natives = geoms if geoms and len(geoms) == len(polys) else None
    return NestState(
        polys=polys,
        group_id=group_id,
        transform=transform,
        selected_indices=sel,
        seed_count=int(seed_count),
        _native_geoms=natives,
    )


def history_from_nest_state(nest_state: NestState) -> tuple:
    """Replace history with adopted selected transforms (relocate_accepted pattern)."""
    buckets: list[list] = [[], []]
    for i in nest_state.selected_indices:
        ii = int(i)
        if ii < 0 or ii >= len(nest_state.group_id) or ii >= len(nest_state.transform):
            continue
        gi = int(nest_state.group_id[ii])
        if gi < 0 or gi > 1:
            continue
        buckets[gi].append(nest_state.transform[ii])
    return (np.array(buckets[0]), np.array(buckets[1]))




def nest_state_extra_voids(nest_state: NestState | None) -> list[Geometry] | None:
    """Geometry voids for locked seed parts (clearance obstacles, not candidates)."""
    if nest_state is None or nest_state.seed_count <= 0:
        return None
    n = min(nest_state.seed_count, len(nest_state.polys))
    if n <= 0:
        return None
    return list(nest_state.native_geoms[:n])


def _poly_and_transforms(item):
    if len(item) == 2:
        return item[0], item[1], 1.0
    return item[0], item[2], item[1]



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


class PolygonGroup(BaseModel):
    model_config = ConfigDict(arbitrary_types_allowed=True)

    polygon: Polygon
    weight: float
    transforms: np.ndarray


def select_polygons_from_edges(b: BaseGeometry, polygons: Tuple[Tuple[Polygon, float, np.ndarray], ...]):
    board_geom = Geometry.from_shapely(b)
    result: list[list[Any]] = [[] for _ in range(len(polygons))]
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
    part_bases: dict | None = None,
    out_native_geoms: list | None = None,
):
    _ctx_t0 = time.perf_counter()
    sheet, void_geoms = board_context_from_geometry(b, user_holes=user_holes)
    if extra_voids:
        void_geoms.extend(extra_voids)
    pad = default_sheet_padding(b)
    guidance_cfg = guidance_config_for_graph(
        min_dist,
        board_bounds=padded_board_bounds(b, pad),
        epsilon_ratio=epsilon_ratio,
    )
    bases = part_bases if part_bases is not None else _base_geometries(polygons)
    mpg_ctx_ms = (time.perf_counter() - _ctx_t0) * 1000.0

    _xform_t0 = time.perf_counter()
    # G2: collect (i, p, t) only — validity first; transform survivors after.
    candidates: list[tuple] = []
    for i, item in enumerate(polygons):
        if len(item) == 2:
            p, transforms = item
        else:
            p, _w, transforms = item
        for t in transforms:
            candidates.append((i, p, t))
    mpg_xform_ms = (time.perf_counter() - _xform_t0) * 1000.0

    by_group: dict[int, list[int]] = {}
    for k, (i, _p, _t) in enumerate(candidates):
        by_group.setdefault(i, []).append(k)

    _valid_t0 = time.perf_counter()
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
    mpg_valid_ms = (time.perf_counter() - _valid_t0) * 1000.0

    pending: list[tuple] = []
    pending_geoms: list[Geometry] = []
    _apply_t0 = time.perf_counter()
    for k, (i, p, t) in enumerate(candidates):
        if not valid_by_k.get(k, False):
            continue
        placed = bases[i].apply_transform(t)
        pending.append((i, p, t, placed))
        pending_geoms.append(placed)
    mpg_apply_ms = (time.perf_counter() - _apply_t0) * 1000.0

    gids = [i for i, _p, _t, _placed in pending]
    angles = [float(t[2]) for _i, _p, t, _placed in pending]
    _poly_t0 = time.perf_counter()
    selected_polys = [transform_poly(p, t) for _i, p, t, _placed in pending]
    mpg_poly_ms = (time.perf_counter() - _poly_t0) * 1000.0
    selected_group_id = list(gids)
    selected_transform = [t for _i, _p, t, _placed in pending]
    mpg_attract_ms = 0.0
    if attract_pairs is None:
        _attr_t0 = time.perf_counter()
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
        mpg_attract_ms = (time.perf_counter() - _attr_t0) * 1000.0
    _edge_t0 = time.perf_counter()
    edge_telem: dict | None = propose_stats if propose_stats is not None else None
    graph = build_pose_graph(
        gids,
        pending_geoms,
        angles,
        attract_pairs=attract_pairs,
        telem=edge_telem,
        transforms=selected_transform,
    )
    edge_n_box: list[int] = []
    mpg_edge_ms = (time.perf_counter() - _edge_t0) * 1000.0
    if propose_stats is not None:
        # T: do not overwrite primary mpg breakdown with mid-pack remakes.
        if not propose_stats.get("mpg_primary_locked"):
            propose_stats["mpg_xform_ms"] = float(mpg_xform_ms)
            propose_stats["mpg_valid_ms"] = float(mpg_valid_ms)
            propose_stats["mpg_edge_ms"] = float(mpg_edge_ms)
            propose_stats["mpg_poly_ms"] = float(mpg_poly_ms)
            propose_stats["mpg_attract_ms"] = float(mpg_attract_ms)
            propose_stats["mpg_apply_ms"] = float(mpg_apply_ms)
            propose_stats["mpg_ctx_ms"] = float(mpg_ctx_ms)
            propose_stats["mpg_edges_count_ms"] = 0.0
        # Edge count via PoseGraph.edge_count (no Python full-adjacency copy).
        propose_stats["graph_edges_n"] = int(graph.edge_count())
        _ = edge_n_box
    if out_native_geoms is not None:
        out_native_geoms.clear()
        out_native_geoms.extend(pending_geoms)

    return graph, selected_polys, selected_group_id, selected_transform



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





def _propose_rules_for_iter(
    cfg: BuildGraphConfig,
    rule_sets: list[PlacementRuleSet],
) -> PlacementRuleSet | None:
    if not cfg.propose.use_rule_ranking:
        return None
    if rule_sets:
        return active_rule_set(rule_sets)
    return active_rule_set(make_initial_rule_sets(cfg))


def _append_selection_window(
    selection_window: list[tuple[np.ndarray, ...]],
    selected_t: tuple[np.ndarray, ...],
    maxlen: int,
) -> None:
    if not any(t.shape[0] for t in selected_t):
        return
    selection_window.append(selected_t)
    if len(selection_window) > maxlen:
        del selection_window[: len(selection_window) - maxlen]



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



def _mcts_active_rules(rule_sets: Sequence, mcts_action: Any | None):
    rid = 0
    if mcts_action is not None:
        rid = int(getattr(mcts_action, "rule_id", 0) or 0)
    return active_rule_set(list(rule_sets), rid)


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
    # Prefer an outer driver path (bench); only truncate on a fresh session.
    _env_telem = str(os.environ.get("NEST_TELEM_LOG", "") or "").strip()
    if _env_telem:
        telem_path = configure_telem_log(_env_telem, reset=False)
    else:
        telem_path = configure_telem_log(
            getattr(out, "telem_log_path", None) or "artifacts/tune/telem.log",
            reset=True,
        )
    write_telem(f"# nest telem session path={telem_path}")

    resume_path = str(getattr(out, "resume_checkpoint", "") or "").strip()
    resume_ckpt = None
    ckpt_baseline_cov = 0.0
    ckpt_baseline_parts = 0
    if resume_path:
        resume_ckpt = load_nest_checkpoint(resume_path)
        ckpt_baseline_cov = float(resume_ckpt.coverage)
        ckpt_baseline_parts = int(resume_ckpt.parts)
        if resume_ckpt.seed is not None and cfg.sampling.seed is None:
            cfg = cfg.model_copy(
                update={"sampling": cfg.sampling.model_copy(update={"seed": int(resume_ckpt.seed)})}
            )
            rng = cfg.apply_seed()
            sc = cfg.sampling

    selected_t: tuple[np.ndarray, ...] = (
        rng.uniform(-1, 1, (sc.initial_random, 3)) * sc.transform_scale,
        rng.uniform(-1, 1, (sc.initial_random, 3)) * sc.transform_scale,
    )
    rule_sets = make_initial_rule_sets(cfg)
    render_size = (out.render_size, out.render_size)
    _fourcc = getattr(cv, "VideoWriter_fourcc")(*"mp4v")
    video = cv.VideoWriter(
        out.video_path,
        _fourcc,
        out.video_fps,
        render_size,
    )
    history: tuple[np.ndarray, ...] = (np.zeros((1, 3)), np.zeros((1, 3)))
    graphs: list[PoseGraph] = []
    selection_window: list[tuple[np.ndarray, ...]] = []
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
    if resume_ckpt is not None:
        nest_state = nest_state_from_checkpoint(resume_ckpt, NestState)
        history = history_tuple_from_ckpt(resume_ckpt)
        graph_valid_carry = carry_tuple_from_ckpt(resume_ckpt)
        void_elite_by_group = void_elite_from_ckpt(resume_ckpt)
        selected_t = history if history[0].size or history[1].size else selected_t
        n_extra = max(int(getattr(out, "extra_iters", 8) or 8), 1)
        start_it = int(resume_ckpt.iter)
        iters = tuple(range(start_it, start_it + n_extra))
        print(
            f"[checkpoint] resume {resume_path} "
            f"iter={resume_ckpt.iter} cov={ckpt_baseline_cov:.4f} "
            f"parts={ckpt_baseline_parts} +{n_extra} iters"
        )
    else:
        iters = tuple(range(out.n_iters))
    pbar = tqdm(
        iters,
        desc="Nesting",
        unit="iter",
        dynamic_ncols=True,
        disable=not out.progress,
    )

    mcts_runner = MacroMctsRunner()
    if mcts_runner.agent is not None:
        mcts_runner.agent.motif_cohorts = ()
    if resume_ckpt is not None:
        rehydrate_motif_base(resume_ckpt, mcts_runner.motif_base)
        rehydrate_niche_archive(resume_ckpt, mcts_runner.niche_archive)
        mcts_root = tip_board_snapshot(
            resume_ckpt, arena_node_id=int(mcts_runner.arena.root_id())
        )
    else:
        mcts_root = BoardSnapshot(
            remaining_gids=tuple(range(int(cfg.rules.ngroups))),
            coverage=0.0,
        )
    # N1: MotifBase cold seed from congruent mates (same relative as ClusterPattern synth).
    _mate_seed_n = seed_motif_base_from_mates(
        mcts_runner.motif_base,
        parts,
        min_dist=float(cfg.board_min_dist_for(p_sheet, first_pass=True)),
    )
    mcts_runner.store_snapshot(int(mcts_runner.arena.root_id()), mcts_root)
    mcts_parent_id = int(mcts_runner.arena.root_id())
    mcts_telem: dict[str, Any] = {
        "pw_expand": 0,
        "motif_hit": 0,
        "place_motif_ok": 0,
        "expand_ms": 0.0,
        "from_shapely_count": 0,
        "amaf_hits": 0,
        "amaf_miss": 0,
        "motif_base_seed_n": int(_mate_seed_n),
        "path_carved_streak": 0,
        "last_nest_n": int(ckpt_baseline_parts) if resume_ckpt is not None else 0,
        "last_incumbent_hold": 0,
        "last_graph_to_nest": 0,
        "last_rim_progress": 0.0,
    }
    path_carved_streak = 0
    if resume_ckpt is not None and ckpt_baseline_parts >= 60:
        # M: late resume starts dense — tip gate can arm after first hold/rim.
        mcts_telem["last_nest_n"] = int(ckpt_baseline_parts)
    _exec_last: dict = {"snap": mcts_root}
    # DgP: cache last outer compose/refine materials for multi-sim cheap_pack.
    _pack_cache: dict = {"ready": False}
    _mcts_n_sims_default = int(os.environ.get("NEST_MCTS_N_SIMS", "4") or 4)
    _mcts_n_sims = int(_mcts_n_sims_default)
    _prev_void_nest = 0
    _archive_feed_keys: set[tuple] = set()
    _densify_presets: list[dict] = []
    if bool(getattr(cfg.propose, "enable_mcts_densify_presets", False)):
        _densify_presets = load_densify_presets(
            getattr(cfg.propose, "densify_presets_path", None)
        )
    _last_mcts_preset_id = 0
    best_pack_cov = 0.0
    best_pack_sel: list[int] | None = None
    best_pack_sig: float = 0.0
    best_pack_tf: list | None = None
    best_pack_seed_n: int = 0
    if resume_ckpt is not None:
        best_pack_cov = float(resume_ckpt.best_pack_cov)
        best_pack_sel = (
            None
            if resume_ckpt.best_pack_sel is None
            else list(resume_ckpt.best_pack_sel)
        )
        best_pack_sig = float(resume_ckpt.best_pack_sig)
        best_pack_tf = (
            None
            if resume_ckpt.best_pack_tf is None
            else list(resume_ckpt.best_pack_tf)
        )
        best_pack_seed_n = int(resume_ckpt.best_pack_seed_n)
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

    last_cov_frac = float(ckpt_baseline_cov) if resume_ckpt is not None else 0.0
    last_parts = int(ckpt_baseline_parts) if resume_ckpt is not None else 0

    for _it in pbar:
        if mcts_runner.agent is not None:
            set_oi = getattr(mcts_runner.agent, "set_outer_iter", None)
            if set_oi is not None:
                set_oi(int(_it))
        # Q69: last leaf = full DFS + post_pack; mid = dfs_passes=1 finalize_end.
        # Hybrid: parent free_kind hints mid+large_void 3b; refreshed after compose.
        is_last_leaf = int(_it) == int(iters[-1]) if iters else True
        near_last = (
            len(iters) >= 2 and int(_it) == int(iters[-2])
        )
        parent_free_hint = is_large_void(
            getattr(
                mcts_runner.snapshot_at(int(mcts_parent_id), mcts_root),
                "free_kind",
                FreeKind.Empty,
            )
        )
        # mid_hole_open applied after densify pack_cfg in stages (PackIterCtx.cfg).
        polish_budget = polish_budget_for_iter(
            is_last_leaf=is_last_leaf,
            sel=sel,
            large_void=parent_free_hint,
            cheap_expand=False,
            near_last=near_last,
            on_plateau=bool(plateau.on_plateau),
            free_remaining=parent_free_hint,
            fits_part=bool(int(mcts_telem.get("last_fits_part", 1) or 0)),
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
        nest_tf_keys: list[tuple[float, float, float]] = []
        if nest_state is not None and nest_state.selected_indices:
            seed_n = int(nest_state.seed_count or 0)
            for i in nest_state.selected_indices:
                if int(i) < seed_n:
                    continue
                if int(i) < len(nest_state.group_id):
                    nest_packed.add(int(nest_state.group_id[i]))
                if int(i) < len(nest_state.transform):
                    nest_tf_keys.append(
                        transform_row_key(nest_state.transform[i])
                    )
        # E: schedule browse for choose_browse_parent; hold+rim also unlocks
        # multi-sim tip install (Motif prefer) without forcing a parent jump.
        do_browse = bool(
            should_browse_tip(
                iter_idx=int(_it),
                n_iters=int(out.n_iters),
                is_last_leaf=is_last_leaf,
            )
        )
        hold_browse = hold_browse_tip(
            parent_free_hint=bool(parent_free_hint),
            spine_snap=spine_snap,
            plateau_on=bool(plateau.on_plateau),
            mcts_telem=mcts_telem,
        )
        browse_parent_id, parent_snap, tip_action = install_iter_tip(
            mcts_runner=mcts_runner,
            spine_id=spine_id,
            spine_snap=spine_snap,
            nest_packed=nest_packed,
            nest_tf_keys=nest_tf_keys,
            do_browse=do_browse,
            hold_browse=hold_browse,
            pack_cache=_pack_cache,
            mcts_telem=mcts_telem,
            mcts_n_sims=int(_mcts_n_sims),
            is_last_leaf=is_last_leaf,
            rule_ids=mcts_rule_ids(rule_sets),
            sync_cohorts_fn=sync_agent_motif_cohorts,
        )
        mcts_parent_id = int(browse_parent_id)
        mcts_action = tip_action
        # P: seed free_kind before path probe (was after path — late path idle).
        if parent_snap is not None:
            fk = seed_parent_free_and_poles(
                parent_snap,
                cfg=cfg,
                p_sheet=p_sheet,
                part_areas=part_areas,
                nest_state=nest_state,
                agent=mcts_runner.agent,
                part_by_group={0: p1, 1: p2},
                seed_free=True,
                seed_poles=False,
            )
            if is_large_void(fk) and not parent_free_hint:
                parent_free_hint = True
                polish_budget = polish_budget_for_iter(
                    is_last_leaf=is_last_leaf,
                    sel=sel,
                    large_void=True,
                    cheap_expand=False,
                    near_last=near_last,
                    on_plateau=bool(plateau.on_plateau),
                    free_remaining=True,
                    fits_part=bool(int(mcts_telem.get("last_fits_part", 1) or 0)),
                )
        # G0–R1: path telem reset + Motif tip + probe + adopt (L4 extract).
        _path_probe = run_outer_path_probe(
            mcts_runner=mcts_runner,
            nest_state=nest_state,
            history=history,
            mcts_parent_id=int(mcts_parent_id),
            mcts_action=mcts_action,
            tip_action=tip_action,
            parent_snap=parent_snap,
            parent_free_hint=bool(parent_free_hint),
            path_carved_streak=int(path_carved_streak),
            mcts_telem=mcts_telem,
            pack_cache=_pack_cache,
            cfg=cfg,
            rule_sets=rule_sets,
            part_bases=part_bases,
            part_areas=part_areas,
            p_sheet=p_sheet,
            plateau=plateau,
            nest_state_from_proven_compose=nest_state_from_proven_compose,
            history_from_nest_state=history_from_nest_state,
        )
        nest_state = _path_probe.nest_state
        history = _path_probe.history
        mcts_parent_id = int(_path_probe.mcts_parent_id)
        mcts_action = _path_probe.mcts_action
        path_carved_streak = int(_path_probe.path_carved_streak)
        # P2/L2: post-path free_kind + multi_pole (shared helper with pre-path).
        seed_parent_free_and_poles(
            parent_snap,
            cfg=cfg,
            p_sheet=p_sheet,
            part_areas=part_areas,
            nest_state=nest_state,
            agent=mcts_runner.agent,
            part_by_group={0: p1, 1: p2},
            mcts_telem=mcts_telem,
            seed_free=True,
            seed_poles=True,
        )
        preset_ids = [0]
        densify_hold = bool(
            int(mcts_telem.get("last_incumbent_hold", 0) or 0)
        ) and bool(parent_free_hint)
        # Staging: online_ok ×K on plateau OR densify hold (late +K often never
        # flats while incumbent_hold∧large_void — L carve telem otherwise absent).
        if bool(getattr(cfg.propose, "enable_mcts_densify_presets", False)):
            online_ids = densify_preset_ids(_densify_presets)
            if len(online_ids) > 1 and (
                bool(plateau.on_plateau) or densify_hold
            ):
                preset_ids = online_ids
        if mcts_runner.agent is not None:
            mcts_runner.agent.preset_ids = preset_ids
        if mcts_action is None and mcts_runner.agent is not None:
            mcts_action = mcts_runner.agent.pick_expand_action(
                parent_snap.remaining_gids
                or tuple(range(int(cfg.rules.ngroups))),
                rule_ids=mcts_rule_ids(rule_sets),
                parent_id=mcts_parent_id,
                snapshot=parent_snap,
            )
        # D: after path/AMAF tip settle — Motif tip when MotifBase fuel (motif_id=0 valid).
        # Force only on last leaf (telem) so mid-iters keep UCB Motif/Void competition.
        mcts_action = ensure_motif_tip_for_fuel(
            mcts_action,
            motif_base=mcts_runner.motif_base,
            remaining_gids=(
                parent_snap.remaining_gids if parent_snap is not None else None
            ),
            mcts_telem=mcts_telem,
            seed_n=int(mcts_telem.get("motif_base_seed_n", 0) or 0),
            force=bool(is_last_leaf),
        )
        if mcts_action is not None and mcts_action.region == MacroRegion.Motif:
            tip_action = mcts_action
        mcts_preset_id = 0
        mcts_force_zone = None
        if mcts_action is not None:
            mcts_telem["mcts_rule_id"] = int(getattr(mcts_action, "rule_id", 0) or 0)
            mcts_preset_id = int(getattr(mcts_action, "preset_id", 0) or 0)
        # L2: AMAF tip keeps its preset_id (Motif/cohort stay 0). Pack-only
        # densify when tip is Motif@0 on densify-hold so carve/flush still run
        # without rewriting the Motif AMAF key (late −0.1678 without this).
        pack_preset_id = int(mcts_preset_id)
        if (
            len(preset_ids) > 1
            and densify_hold
            and (bool(near_last) or bool(plateau.on_plateau))
            and int(mcts_preset_id) == 0
            and mcts_action is not None
            and getattr(mcts_action, "region", None) == MacroRegion.Motif
        ):
            for _pid in preset_ids:
                if int(_pid) != 0:
                    pack_preset_id = int(_pid)
                    mcts_telem["carve_preset_pack_only"] = 1
                    break
        mcts_telem["mcts_preset_id"] = int(mcts_preset_id)
        mcts_telem["pack_preset_id"] = int(pack_preset_id)
        pack_cfg = apply_densify_preset(cfg, pack_preset_id, _densify_presets)
        # P2: cheap cache keys/invalidates on pack densify preset (may differ from tip).
        _pack_cache["pack_preset_id"] = int(pack_preset_id)
        _pack_cache["densify_presets"] = _densify_presets
        if int(pack_preset_id) != int(_last_mcts_preset_id):
            mcts_telem["cache_preset_key_only"] = 1
            maybe_invalidate_cheap_cache(
                _pack_cache, pack_preset_id=int(pack_preset_id),
            )
            mcts_telem["cache_invalidate_preset"] = int(
                _pack_cache.get("cache_invalidate_preset", 0) or 0
            )
        _last_mcts_preset_id = int(pack_preset_id)
        if mcts_action is not None:
            mcts_force_zone = place_zone_token(region_to_zone(mcts_action.region))

        propose_rules = (
            _mcts_active_rules(rule_sets, mcts_action)
            if mcts_action is not None and cfg.propose.use_rule_ranking
            else _propose_rules_for_iter(cfg, rule_sets)
        )
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
            "on_plateau": bool(plateau.on_plateau),
            "is_last_leaf": bool(is_last_leaf),
            "near_last": bool(near_last),
            "free_kind": free_kind_token(
                getattr(parent_snap, "free_kind", FreeKind.Empty)
            ),
            "mcts_zone": mcts_force_zone,
            # L1: region SoT for bind_epoch Void kind (zone alone is Q104-ambiguous).
            "mcts_region": (
                int(getattr(mcts_action.region, "value", mcts_action.region))
                if mcts_action is not None
                else None
            ),
            "mcts_heavy": int(polish_budget.mcts_heavy),
            "dfs_passes": int(polish_budget.dfs_passes),
            "dfs_mode": str(polish_budget.dfs_mode.value),
            "freeze_improve_reason": freeze_reason,
        }
        # P3: last-iter MotifJoin survival soft prior for next-iter RCL.
        _survive_prior = {}
        if mcts_runner.agent is not None:
            _survive_prior = dict(
                (mcts_runner.agent.realized or {}).get("survive_by_motif") or {}
            )
        propose_stats["survive_by_motif"] = dict(_survive_prior)
        propose_stats["survive_motif_n"] = int(
            sum(int(v) for v in _survive_prior.values())
        )
        # F1: at most one sticky plug-realize per resume session (Δ protect).
        propose_stats["f1_sticky_spent"] = int(
            _pack_cache.get("f1_sticky_spent", 0) or 0
        )
        keep_hist_sterile = bool(
            (sat_info.sat_override or had_void_override)
            and bool(getattr(cfg.propose, "keep_history_on_void_sterile", True))
        )
        ngroups = max(len(parts) if parts else len(selected_t), 2)
        void_elite_t = void_elite_tuple_from_archive(void_elite_by_group, ngroups)
        elite_n = void_elite_count(void_elite_by_group)
        # Q137: under void_seek / large_void, merge MacroNicheArchive into elite niche.
        void_seek_mix = is_void_seek(mcts_force_zone)
        if not void_seek_mix:
            void_seek_mix = bool(
                is_large_void(parent_snap)
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
                    _archive_feed_keys.add(transform_row_key(arr))
            mcts_runner.niche_archive.last_feed_keys = set(_archive_feed_keys)
        elite_n = void_elite_count(merged_elite)
        propose_stats["void_elite_seeded"] = elite_n
        arch_elite_n = sum(len(v) for v in (arch_rows or {}).values())
        maybe_invalidate_cheap_cache(
            _pack_cache,
            remaining_gids=tuple(parent_snap.remaining_gids or ()),
            void_elite_seeded=int(elite_n),
            archive_elite_n=int(arch_elite_n),
            compose_sz=len(propose_stats.get("motif_locked") or ()),
            cohort_sig=int(propose_stats.get("motif_cohort_sig", 0) or 0),
        )
        propose_stats["archive_elite_n"] = arch_elite_n
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
        # L3: path ContactGRG post-truncate mid — tip Motif id stays first.
        _path_pref_raw = mcts_telem.get("path_upsert_prefer_mid", -1)
        path_prefer_mid = -1 if _path_pref_raw is None else int(_path_pref_raw)
        if (
            nest_state is not None
            and bool(getattr(cfg.propose, "enable_accepted_pattern_archive", True))
        ):
            note_motif_ref_anchors_from_nest(mcts_runner.motif_base, nest_state)
        # L3 park + Letter B dual: hollow/graph_to_nest polish exception late-missed
        # (Δ min +0.1614; nfp_lite_keep stayed 0). Keep plateau∧LV polish skip.
        _plateau_lv = bool(plateau.on_plateau) and is_large_void(parent_snap)
        _inject_polish = not _plateau_lv
        propose_stats["inject_polish"] = int(_inject_polish)
        propose_stats["inject_polish_plateau_lv"] = int(_plateau_lv)
        propose_stats["inject_polish_hollow_nfp"] = 0
        archived_for_propose = (
            motif_patterns_for_inject(
                mcts_runner.motif_base,
                max_keep=int(getattr(cfg.propose, "accepted_pattern_max", 4) or 4),
                prefer_motif_id=prefer_mid,
                also_prefer_motif_id=path_prefer_mid,
                part_bases=part_bases,
                min_dist=float(cfg.board_min_dist_for(p_sheet, first_pass=False)),
                telem=propose_stats,
                polish=_inject_polish,
            )
            if bool(getattr(cfg.propose, "enable_accepted_pattern_archive", True))
            else []
        )
        propose_stats["accepted_patterns_n"] = len(archived_for_propose)
        propose_stats["motif_library_n"] = int(mcts_runner.motif_base.size())
        propose_stats["motif_base_seed_n"] = int(
            mcts_telem.get("motif_base_seed_n", _mate_seed_n) or 0
        )
        propose_stats["nfp_lite_ok"] = int(propose_stats.get("nfp_lite_ok", 0) or 0) + int(
            mcts_telem.get("nfp_lite_ok", 0) or 0
        )
        propose_stats["nfp_lite_keep"] = int(
            propose_stats.get("nfp_lite_keep", 0) or 0
        ) + int(mcts_telem.get("nfp_lite_keep", 0) or 0)
        propose_stats["dg_force_zone"] = mcts_force_zone
        propose_stats["last_graph_n"] = int(mcts_telem.get("last_graph_n", 0) or 0)
        propose_stats["last_nest_n"] = int(mcts_telem.get("last_nest_n", 0) or 0)
        propose_stats["last_graph_to_nest_hollow"] = int(
            mcts_telem.get("last_graph_to_nest_hollow", 0) or 0
        )
        # K: hand path_carved into propose stamp densify_clear unlock.
        propose_stats["path_carved"] = int(mcts_telem.get("path_carved", 0) or 0)
        if mcts_force_zone:
            enabled_z = ProposeConfig.proposers_for_place(
                place_zone_token(mcts_force_zone)
            )
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
        _propose_t0 = time.perf_counter()
        selected_t = build_transform_batch(
            pack_cfg,
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
        propose_stats["propose_ms"] = (time.perf_counter() - _propose_t0) * 1000.0
        pin_keys = set(propose_stats.get("archive_mix_pin_keys") or ())
        first_pass = nest_state is None
        free_prep_mid = None
        # Q128 / T1: soft-cap attract degree when prior graph ≫ nest.
        attract_deg = int(cfg.propose.attract_max_degree)
        prev_graph_n = int(mcts_telem.get("last_graph_n", 0) or 0)
        prev_nest_n = int(mcts_telem.get("last_nest_n", 0) or 0)
        if prev_graph_n > max(4 * max(prev_nest_n, 1), 200):
            attract_deg = min(attract_deg, 3)
            propose_stats["attract_degree_capped"] = int(attract_deg)
        _mpg_t0 = time.perf_counter()
        mpg_natives: list = []
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
            part_bases=part_bases,
            out_native_geoms=mpg_natives,
        )
        propose_stats["mpg_ms"] = (time.perf_counter() - _mpg_t0) * 1000.0
        propose_stats["mpg_primary_locked"] = True
        propose_stats["graph_valid_n"] = int(len(transform))
        if pin_keys:
            graph_keys = {
                transform_row_key(np.asarray(t, dtype=np.float64))
                for t in transform
            }
            propose_stats["archive_mix_pin_survive_n"] = int(
                len(pin_keys & graph_keys)
            )
        # B: spatial-stride sample (seed-phased). Incumbent-drop hurt late Δ med —
        # keep drop hook unused; stride alone unbinds lowest-x head bias.
        propose_stats["carry_drop_keys_by_group"] = None
        propose_stats["carry_sample_seed"] = int(
            zlib.crc32(
                np.int64(len(transform)).tobytes(),
                int(propose_stats.get("graph_valid_n", 0) or 0) & 0xFFFFFFFF,
            )
            % (2**31)
        )
        graph_valid_carry = inject_cohorts_and_bind_graph(
            mcts_runner.dg,
            graph,
            group_id,
            transform,
            propose_stats,
            cfg,
            patterns=archived_for_propose or None,
            agent=mcts_runner.agent,
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
        seed_rules = active_rule_set(make_seed_rule_sets(cfg))
        free_info = None
        # Ua: one improve call above first_pass / mid-pack fork.
        # K: one live_presets list across iters; credit preset slots via AMAF
        # deltas + score_rules gains; soft-shift all three after rounds.
        if "live_presets" not in mcts_telem or not mcts_telem.get("live_presets"):
            mcts_telem["live_presets"] = cfg.rules.mutation_presets()
            mcts_telem["anchor_presets"] = cfg.rules.mutation_presets()
        live_presets = mcts_telem["live_presets"]
        anchor_presets = mcts_telem["anchor_presets"]
        slot_weights = credit_mut_slot_amaf_deltas(mcts_telem)
        slot_scale: list[int] = []
        plateau_boost = (
            plateau.on_plateau
            and int(mcts_telem.get("plateau_rule_boost", 0) or 0) > 0
            and rule_sets
        )
        if plateau_boost:
            rid = int(mcts_telem.get("last_rule_id", 0) or 0)
            if 0 <= rid < len(rule_sets):
                # M1 parked: exclusive single-scale; K soft-shifts all three.
                rule_sets = improve_rules(
                    graphs,
                    rule_sets,
                    sel_iter.rules_kept,
                    p_sheet,
                    mutation_presets=live_presets,
                    rule_score_penalty=sel_iter.rule_score_penalty,
                    elite_count=max(int(sel_iter.improve_rules_elite_count), 1),
                    seed=int(rng.integers(0, 2**31)) + int(rid) * 997,
                    score_options=score_rules_options(sel_iter),
                    max_rules_per_set=cfg.rules.max_rules_per_set,
                    slot_scale_out=slot_scale,
                    slot_weights_inout=slot_weights,
                )
                mcts_telem["rules_macro_ran"] = int(
                    mcts_telem.get("rules_macro_ran", 0) or 0
                ) + 1
            mcts_telem["plateau_rule_boost"] = 0
        rule_sets = run_improve_rules_rounds(
            improve_rules,
            graphs=graphs,
            rule_sets=rule_sets,
            board=p_sheet,
            sel_iter=sel_iter,
            rng=rng,
            score_options=score_rules_options(sel_iter),
            mutation_presets=live_presets,
            rule_score_penalty=sel_iter.rule_score_penalty,
            max_rules_per_set=cfg.rules.max_rules_per_set,
            slot_scale_out=slot_scale,
            slot_weights_inout=slot_weights,
        )
        mcts_telem["mut_slot_scale"] = list(slot_scale)
        mcts_telem["mut_slot_weights"] = list(slot_weights)
        if sum(float(w) for w in slot_weights) > 1e-12:
            blend_live_mutation_presets(
                live_presets, slot_weights, anchor_presets, alpha=0.2
            )
            mcts_telem["working_mut_updates"] = int(
                mcts_telem.get("working_mut_updates", 0) or 0
            ) + 1
        write_mut_live_telem(mcts_telem, live_presets, slot_weights)
        propose_stats["working_dpos"] = float(mcts_telem.get("working_dpos", 0.0) or 0.0)
        propose_stats["working_dw"] = float(mcts_telem.get("working_dw", 0.0) or 0.0)
        propose_stats["working_mut_updates"] = int(
            mcts_telem.get("working_mut_updates", 0) or 0
        )
        for _i in range(3):
            propose_stats[f"mut_dpos_{_i}"] = float(
                mcts_telem.get(f"mut_dpos_{_i}", 0.0) or 0.0
            )
        if first_pass and cfg.propose.first_pass_border_pack:
            fp_ctx = PackIterCtx(
                graph=graph,
                polys=polys,
                group_id=group_id,
                transform=transform,
                part_areas=part_areas,
                part_bases=part_bases,
                cfg=cfg,
                sel=sel,
                propose_stats=propose_stats,
                dg=mcts_runner.dg,
                sheet=p_sheet,
                rule_sets=rule_sets,
                native_geoms_fn=_native_geoms_from_transforms,
            )
            fp_result = run_first_pass_border_pack(
                fp_ctx,
                seed_rules=seed_rules,
                parts=parts,
                p1=p1,
                p2=p2,
                board_ctx_outline=board_ctx_outline,
                sel_iter=sel_iter,
                active_rules=_mcts_active_rules(rule_sets, mcts_action),
                make_polygon_graph_fn=make_polygon_graph,
            )
            graph = fp_result.graph
            polys = fp_result.polys
            group_id = fp_result.group_id
            transform = fp_result.transform
            selected_polys = fp_result.selected_polys
            free_info = fp_result.free_info
            old_len = fp_result.old_len
        else:
            active_rules = _mcts_active_rules(rule_sets, mcts_action)
            scores = list(score_elems(graph, active_rules))
            sheet, void_geoms_compose = board_ctx_outline
            min_dist = cfg.board_min_dist_for(p_sheet, first_pass=first_pass)

            rem_mid = (
                list(getattr(parent_snap, "remaining_gids", ()) or ())
                if parent_snap is not None
                else None
            )
            free_prep = schedule_prep_selection_free(
                phase="mid",
                sheet=sheet,
                part_areas=part_areas,
                min_dist=min_dist,
                cfg_propose=cfg.propose,
                nest_state=nest_state,
                part_by_group={0: p1, 1: p2},
                remaining_gids=rem_mid,
            )
            assert free_prep is not None
            free_info = free_prep.free_info
            packed_geoms = free_prep.packed_geoms
            packed_group_id = free_prep.packed_group_id
            packed_transform = free_prep.packed_transform
            free_prep_mid = free_prep
            sheet_diag = sheet_diag_from(sheet)
            dual_nest = dual_nest_for(
                free_info,
                last_leaf=is_last_leaf,
                sterile_hold=bool(
                    int(mcts_telem.get("last_incumbent_hold", 0) or 0)
                ),
            )
            seed_voids = nest_state_extra_voids(nest_state) or []
            pack_box = RefinePackBox()
            pack_ctx = PackIterCtx(
                graph=graph,
                polys=polys,
                group_id=group_id,
                transform=transform,
                part_areas=part_areas,
                part_bases=part_bases,
                cfg=pack_cfg,
                sel=sel,
                propose_stats=propose_stats,
                dg=mcts_runner.dg,
                sheet=p_sheet,
                min_dist=min_dist,
                rule_sets=rule_sets,
                active_rules=active_rules,
                scores=scores,
                free_info=free_info,
                void_geoms=void_geoms_compose,
                packed_geoms=list(packed_geoms),
                packed_group_id=packed_group_id,
                packed_transform=packed_transform,
                sheet_diag=float(sheet_diag),
                sheet_area=float(sheet.area) if sheet is not None else 0.0,
                ngroups=int(cfg.rules.ngroups),
                is_last_leaf=is_last_leaf,
                near_last=near_last,
                refine_seed=int(rng.integers(0, 2**31)),
                first_pass=first_pass,
                native_geoms_fn=_native_geoms_from_transforms,
                graph_native_geoms=list(mpg_natives) if mpg_natives else None,
                motif_base=mcts_runner.motif_base,
                survive_by_motif=dict(_survive_prior),
                seed_count=int(nest_state.seed_count or 0) if nest_state else 0,
                # W1: keep seeds in packed (not void_geoms) so 3b/board voids SoT
                # stays board-only; compose rebuilds nest solids then prepends seeds.
                seed_void_geoms=list(seed_voids),
            )
            pack_ctx.enable_3b = True
            mid_result, pin_all_blocked_streak = run_mid_pack_stages(
                pack_ctx,
                pack_box,
                graph=graph,
                part_by_group={0: p1, 1: p2},
                void_geoms_compose=void_geoms_compose,
                archived_for_propose=archived_for_propose,
                pin_all_blocked_streak=pin_all_blocked_streak,
                cheap=False,
                prev_fits_part=int(mcts_telem.get("last_fits_part", 1) or 0),
            )
            composed = pack_box.composed
            assert composed is not None
            propose_stats["nest_dual"] = int(dual_nest)
            scores = composed.scores
            refine_scores = mid_result.refine_scores
            selected_nest = mid_result.selected_nest
            refine_rules = composed.refine_rules
            free_poly = mid_result.free_poly
            free_info = mid_result.free_info
            polish_budget = mid_result.polish_budget
            n_void_nest = mid_result.n_void_nest
            boost_hits = mid_result.boost_hits
            old_len = len(selected_nest)
            selected_polys = mid_result.selected_polys
            polys = mid_result.polys
            transform = mid_result.transform
            group_id = mid_result.group_id
            candidate_geoms = mid_result.candidate_geoms
            pin_stats = mid_result.pin_stats
            # DgP: warm multi-sim cache for next-iter cheap_pack expands.
            # Preserve F1 once-per-session sticky budget across clear().
            _f1_sticky_spent = int(_pack_cache.get("f1_sticky_spent", 0) or 0)
            if int(propose_stats.get("hold_cohort_nest", 0) or 0) > 0:
                _f1_sticky_spent = 1
            # P0: path/cch telem lives on mcts_telem; reset pack_cache counters
            # for next-iter multi-sim (do not carry wiped zeros into propose_stats).
            warm_outer_pack_cache(
                _pack_cache,
                graph=mcts_runner.dg.poses(),
                dg=mcts_runner.dg,
                polys=polys,
                group_id=group_id,
                transform=transform,
                part_areas=part_areas,
                part_bases=part_bases,
                p_sheet=p_sheet,
                min_dist=min_dist,
                cfg=pack_cfg,
                densify_presets=_densify_presets,
                free_info=free_info,
                propose_stats=propose_stats,
                packed_geoms=packed_geoms,
                packed_group_id=packed_group_id,
                packed_transform=packed_transform,
                void_geoms=void_geoms_compose or [],
                seed_void_geoms=seed_voids,
                sheet=sheet,
                board_area=board_area,
                selected_polys=selected_polys,
                f1_sticky_spent=int(_f1_sticky_spent),
            )
            sync_agent_motif_cohorts(mcts_runner, propose_stats)
            _pack_cache["motif_cohort_sig"] = int(
                propose_stats.get("motif_cohort_sig", 0) or 0
            )
            _pack_cache["motif_base"] = mcts_runner.motif_base
            _pack_cache["survive_by_motif"] = dict(_survive_prior)
            mcts_telem["last_graph_n"] = int(len(transform))
            # T: reset idle carve streak on realized part gain (outer nest grew).
            _prev_nest_for_streak = int(mcts_telem.get("last_nest_n", 0) or 0)
            mcts_telem["last_nest_n"] = int(len(selected_polys))
            if int(len(selected_polys)) > _prev_nest_for_streak:
                path_carved_streak = 0
                mcts_telem["path_carved_streak"] = 0
            mcts_telem["last_incumbent_hold"] = int(
                propose_stats.get("incumbent_hold", 0) or 0
            )
            mcts_telem["last_rim_progress"] = float(
                propose_stats.get("rim_progress", 0.0) or 0.0
            )
            mcts_telem["last_fits_part"] = int(
                propose_stats.get(
                    "fits_part",
                    int(bool(getattr(free_info, "fits_part", True))),
                )
                or 0
            )
            # D1: leaf_fit_r / phantom leaf count (void_fit_r on BoardSnapshot).
            _fit_r = 1.0 if int(mcts_telem["last_fits_part"]) else 0.0
            mcts_telem["leaf_fit_r"] = float(_fit_r)
            if (
                is_large_void(free_info)
                and float(_fit_r) < 1.0
            ):
                mcts_telem["mcts_phantom_n"] = int(
                    mcts_telem.get("mcts_phantom_n", 0) or 0
                ) + 1
            mcts_telem["hybrid_pick_trials"] = int(
                propose_stats.get("hybrid_pick_trials", 0) or 0
            )
            mcts_telem["motif_compose_accepted_size"] = int(
                propose_stats.get("motif_compose_accepted_size", 0) or 0
            )
            propose_stats["motif_sequential_repin"] = 0
            track_d = bool(large_void_motif_plateau.ready)
            propose_stats["large_void_motif_plateau"] = track_d
            propose_stats["large_void_motif_plateau_streak"] = int(
                large_void_motif_plateau.streak
            )
            pin_cands = int(pin_stats.get("pin_candidates", 0))
            pin_added = int(pin_stats.get("pin_added", 0))
            proposed_map = propose_stats.get("proposed_by_group") or {}
            proposed_list = (
                [proposed_map[g] for g in sorted(proposed_map)]
                if proposed_map else None
            )
            leak_orch = run_void_leak_and_niche_credit(
                VoidLeakGatherCtx(
                    graph=graph,
                    polys=polys,
                    group_id=group_id,
                    transform=transform,
                    free_info=free_info,
                    free_poly=free_poly,
                    selected_nest=selected_nest,
                    selected_polys=selected_polys,
                    refine_scores=refine_scores,
                    propose_stats=propose_stats,
                    plateau=plateau,
                    pin_stats=pin_stats,
                    pin_all_blocked_streak=pin_all_blocked_streak,
                    n_void_nest=n_void_nest,
                    boost_hits=boost_hits,
                    void_pole_near_diag_ratio=float(
                        getattr(cfg.propose, "void_pole_near_diag_ratio", 0.25) or 0.25
                    ),
                    proposer_counts=proposer_counts,
                    sheet_diag=float(sheet_diag),
                    mcts_telem=mcts_telem,
                    mcts_runner=mcts_runner,
                    sheet=p_sheet,
                ),
                mcts_action=mcts_action,
                cfg_propose=cfg.propose,
                prev_void_nest=_prev_void_nest,
                pin_cands=pin_cands,
                pin_added=pin_added,
                proposed_list=proposed_list,
                stratified_void_elite_quota=int(
                    getattr(cfg.propose, "stratified_void_elite_quota", 15)
                ),
            )
            void_elite_by_group = leak_orch.void_elite_by_group
            if leak_orch.had_void_override:
                had_void_override = True
            n_void_graph = leak_orch.n_void_graph
            outline_cov = leak_orch.outline_cov
            proposer_keys = leak_orch.proposer_keys
            _prev_void_nest = leak_orch.prev_void_nest
            # Peak inward-explorer emit across iters (R0/R1 letter telem).
            leak = propose_stats.get("void_leak")
            if isinstance(leak, dict):
                ebp = leak.get("emitted_by_proposer") or {}
                peak = propose_stats.setdefault("inward_peak", {})
                for name in ("raycasting", "voronoi", "erosion", "side_pack"):
                    cur = int(ebp.get(name, 0) or 0)
                    peak[name] = max(int(peak.get(name, 0) or 0), cur)
                leak["inward_peak"] = dict(peak)
                zones_peak = propose_stats.setdefault("zones_peak", [])
                for z in leak.get("zones_used") or []:
                    zs = str(z)
                    if zs not in zones_peak:
                        zones_peak.append(zs)
                leak["zones_peak"] = list(zones_peak)
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
                zone=(
                    place_zone_token(propose_stats.get("dg_force_zone"))
                    or place_zone_token(propose_stats.get("mcts_zone"))
                    or (
                        str((propose_stats.get("zones_used") or [None])[0])
                        if propose_stats.get("zones_used")
                        else None
                    )
                ),
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
            # Letter B telem: zone_graph_yields now live via zone= above.
            if propose_feedback.zone_graph_yields:
                propose_stats["zone_graph_yields"] = {
                    z: list(vs[-4:])
                    for z, vs in propose_feedback.zone_graph_yields.items()
                }
                mcts_telem["zone_yield_n"] = int(
                    sum(len(v) for v in propose_feedback.zone_graph_yields.values())
                )
        n_graph = int(graph.size()) if hasattr(graph, "size") else len(graph.elems)
        assert selection_is_independent(
            graph, [i for i in selected_polys if 0 <= int(i) < n_graph],
        )
        min_dist = cfg.board_min_dist_for(p_sheet, first_pass=first_pass)
        sheet_compact, void_geoms_post = board_ctx_outline
        part_by_group = {0: p1, 1: p2}
        seed_n = nest_state.seed_count if nest_state is not None else 0
        fixed_obs = list(nest_state.polys[:seed_n]) if nest_state is not None and seed_n else None
        relocate_accepted = False
        void_leak_stats = propose_stats.get("void_leak") if propose_stats else None
        rem_post = (
            list(getattr(parent_snap, "remaining_gids", ()) or ())
            if parent_snap is not None
            else None
        )
        post_prep = prepare_post_pack(
            sheet_compact=sheet_compact,
            void_geoms_post=void_geoms_post,
            part_areas=part_areas,
            min_dist=min_dist,
            cfg_propose=cfg.propose,
            selected_polys=selected_polys,
            polys=polys,
            group_id=group_id,
            transform=transform,
            part_bases=part_bases,
            native_pack_geoms_fn=_native_geoms_from_transforms,
            free_prep_mid=free_prep_mid,
            free_info=free_info,
            void_leak_stats=void_leak_stats if isinstance(void_leak_stats, dict) else None,
            propose_stats=propose_stats,
            archived_patterns=archived_for_propose,
            part_by_group=part_by_group,
            remaining_gids=rem_post,
        )
        # L: refresh budget with densify pack_cfg (mid_hole_open) after tip settle.
        _free_post = getattr(post_prep, "free_post", None)
        polish_budget = polish_budget_for_iter(
            is_last_leaf=is_last_leaf,
            sel=sel,
            large_void=bool(
                _free_post is not None
                and is_large_void(_free_post)
            ),
            cheap_expand=False,
            near_last=near_last,
            on_plateau=bool(plateau.on_plateau),
            free_remaining=bool(
                _free_post is not None
                and is_large_void(_free_post)
            ),
            fits_part=bool(
                getattr(_free_post, "fits_part", True) if _free_post is not None else True
            ),
            mid_hole_open=bool(
                getattr(getattr(pack_cfg, "propose", None), "mid_hole_open", False)
            ),
        )
        post_pack_ctx = PackIterCtx(
            graph=graph,
            polys=list(polys),
            group_id=list(group_id),
            transform=list(transform),
            part_areas=part_areas,
            part_bases=part_bases,
            cfg=pack_cfg,
            sel=sel,
            propose_stats=propose_stats,
            sheet=p_sheet,
            min_dist=min_dist,
            native_geoms_fn=_native_geoms_from_transforms,
        )
        polys, transform, selected_polys, pack_stats = run_post_pack_stage(
            post_pack_ctx,
            post_prep,
            selected_polys=selected_polys,
            part_by_group=part_by_group,
            fixed_obstacles=fixed_obs,
            void_leak_stats=void_leak_stats if isinstance(void_leak_stats, dict) else None,
            polish_budget=polish_budget,
        )
        if post_prep.push_pt is not None and is_large_void(post_prep.free_post):
            repack_stats = pack_stats.get("repack") or {
                "attempted": 0,
                "accepted": 0,
                "motif_accepted": 0,
                "skipped_refine_zero": int(not post_prep.allow_repack),
            }
            if int(repack_stats.get("motif_accepted", 0) or 0) > 0:
                seed_n = nest_state.seed_count if nest_state is not None else 0
                pack_geoms = _native_geoms_from_transforms(
                    group_id,
                    transform,
                    part_bases,
                    seed_polys=nest_state.polys[:seed_n] if nest_state is not None and seed_n else None,
                    seed_count=seed_n,
                )
                upsert_from_repack_accept(
                    mcts_runner.motif_base,
                    repack_stats,
                    pack_geoms,
                    group_id,
                    transform,
                    gap=float(min_dist),
                    min_compactness=float(
                        getattr(cfg.propose, "motif_min_compactness", 0.35) or 0.35
                    ),
                    ttl=int(getattr(cfg.propose, "accepted_pattern_ttl", 4) or 4),
                    max_keep=int(getattr(cfg.propose, "accepted_pattern_max", 4) or 4),
                    telem=propose_stats,
                )
                note_motif_ref_anchors(
                    mcts_runner.motif_base,
                    [int(group_id[i]) for i in selected_polys if int(i) < len(group_id)],
                    [transform[i] for i in selected_polys if int(i) < len(transform)],
                )
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
            propose_stats["gls_attempted"] = int(se2_stats.get("gls_attempted", 0) or 0)
            propose_stats["gls_insert_moved"] = int(
                se2_stats.get("gls_insert_moved", 0) or 0
            )
            propose_stats["gls_separate_moved"] = int(
                se2_stats.get("gls_separate_moved", 0) or 0
            )
            propose_stats["se2_edge_snap_n"] = int(
                se2_stats.get("se2_edge_snap_n", 0) or 0
            )
            propose_stats["kiss_try"] = int(se2_stats.get("kiss_try", 0) or 0)
            propose_stats["kiss_ok"] = int(se2_stats.get("kiss_ok", 0) or 0)
            propose_stats["kiss_worse_rej"] = int(
                se2_stats.get("kiss_worse_rej", 0) or 0
            )
            propose_stats["kiss_stand0"] = float(
                se2_stats.get("kiss_stand0", 0.0) or 0.0
            )
            propose_stats["kiss_stand1"] = float(
                se2_stats.get("kiss_stand1", 0.0) or 0.0
            )
            propose_stats["hole_open_attempted"] = int(
                se2_stats.get("hole_open_attempted", 0) or 0
            )
            propose_stats["hole_open_fit"] = int(
                se2_stats.get("hole_open_fit", 0) or 0
            )
            # L3: hole_open success clears pin latch for the next iter.
            if int(propose_stats["hole_open_fit"]) > 0:
                pin_all_blocked_streak = 0
                propose_stats["pin_streak_reset"] = 3
            propose_stats["hole_open_r_entry"] = float(
                se2_stats.get("hole_open_r_entry", 0.0) or 0.0
            )
            propose_stats["hole_open_r0"] = float(
                se2_stats.get("hole_open_r0", 0.0) or 0.0
            )
            propose_stats["hole_open_r1"] = float(
                se2_stats.get("hole_open_r1", 0.0) or 0.0
            )
            propose_stats["post_ms"] = float(pack_stats.get("post_ms", 0.0) or 0.0)
            propose_stats["se2_ms"] = float(
                se2_stats.get("se2_ms", pack_stats.get("se2_ms", 0.0)) or 0.0
            )
            propose_stats["qp_ms"] = float(
                se2_stats.get("qp_ms", pack_stats.get("qp_ms", 0.0)) or 0.0
            )
            free_post = getattr(post_prep, "free_post", None)
            if free_post is not None:
                propose_stats["fit_in_r"] = float(
                    getattr(free_post, "fit_in_r", 0.0) or 0.0
                )
                propose_stats["fit_need_r"] = float(
                    getattr(free_post, "fit_need_r", 0.0) or 0.0
                )
                propose_stats["fit_need_w"] = float(
                    getattr(free_post, "fit_need_w", 0.0) or 0.0
                )
            propose_stats["hole_open_dfit"] = float(
                se2_stats.get("hole_open_dfit", 0.0) or 0.0
            )
            propose_stats["hole_open_slack_would_restore"] = int(
                se2_stats.get("hole_open_slack_would_restore", 0) or 0
            )
            # P0: prefer mcts_telem (stamped after path; mid-pack clear wipes pack_cache).
            propose_stats["cheap_cache_hit"] = int(
                mcts_telem.get(
                    "cheap_cache_hit", _pack_cache.get("cache_hit_n", 0)
                )
                or 0
            )
            propose_stats["cheap_cache_lookup"] = int(
                mcts_telem.get(
                    "cheap_cache_lookup", _pack_cache.get("cache_lookup_n", 0)
                )
                or 0
            )
            propose_stats["path_compose_ms"] = float(
                mcts_telem.get(
                    "path_compose_ms", _pack_cache.get("path_compose_ms", 0.0)
                )
                or 0.0
            )
            propose_stats["path_refine_ms"] = float(
                mcts_telem.get(
                    "path_refine_ms", _pack_cache.get("path_refine_ms", 0.0)
                )
                or 0.0
            )
            write_telem(
                f"repack={repack_stats.get('accepted', 0)}/"
                f"{repack_stats.get('attempted', 0)} "
                f"motif={repack_stats.get('motif_accepted', 0)} "
                f"reloc={reloc_stats.get('accepted', 0)}/"
                f"{reloc_stats.get('attempted', 0)} "
                f"se2={se2_stats.get('moved', 0)}/"
                f"{se2_stats.get('attempted', 0)} "
                f"tan={se2_stats.get('tangent_moves', 0)} "
                f"gls={se2_stats.get('gls_insert_moved', 0)}/"
                f"{se2_stats.get('gls_separate_moved', 0)}/"
                f"{se2_stats.get('gls_attempted', 0)} "
                f"edge_snap={se2_stats.get('se2_edge_snap_n', 0)} "
                f"kiss={se2_stats.get('kiss_try', 0)}/"
                f"{se2_stats.get('kiss_ok', 0)}/"
                f"{se2_stats.get('kiss_worse_rej', 0)} "
                f"stand={float(se2_stats.get('kiss_stand0', 0) or 0):.4f}→"
                f"{float(se2_stats.get('kiss_stand1', 0) or 0):.4f} "
                f"hole_open={se2_stats.get('hole_open_attempted', 0)}/"
                f"{se2_stats.get('hole_open_fit', 0)} "
                f"r_entry={float(se2_stats.get('hole_open_r_entry', 0) or 0):.3f} "
                f"r={float(se2_stats.get('hole_open_r0', 0) or 0):.3f}→"
                f"{float(se2_stats.get('hole_open_r1', 0) or 0):.3f} "
                f"slack={float(se2_stats.get('hole_open_slack_pre', 0) or 0):.3f}→"
                f"{float(se2_stats.get('hole_open_slack_post', 0) or 0):.3f}/"
                f"dfit={float(se2_stats.get('hole_open_dfit', 0) or 0):.3f}/"
                f"rest={int(se2_stats.get('hole_open_slack_restore', 0) or 0)} "
                f"fit_ir={float(propose_stats.get('fit_in_r', 0) or 0):.3f}/"
                f"{float(propose_stats.get('fit_need_r', 0) or 0):.3f}/"
                f"w={float(propose_stats.get('fit_need_w', 0) or 0):.3f} "
                f"would_rest={int(se2_stats.get('hole_open_slack_would_restore', 0) or 0)} "
                f"post_ms={float(propose_stats.get('post_ms', 0) or 0):.1f}/"
                f"{float(propose_stats.get('se2_ms', 0) or 0):.1f}/"
                f"{float(propose_stats.get('qp_ms', 0) or 0):.1f} "
                f"preset={int(mcts_telem.get('mcts_preset_id', 0) or 0)} "
                f"post={int(getattr(polish_budget, 'run_post_pack', False))}"
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
            refine_bp_credit: dict = {}
            emitted_bp_credit: dict = {}
            if mcts_runner.agent is not None:
                densify_tmp = propose_stats.get("densify_stats") or {}
                pk = (
                    propose_stats.get("proposer_keys")
                    or densify_tmp.get("proposer_keys")
                    or {}
                )
                refine_bp_credit = count_selected_by_proposer(
                    transform, selected_polys, pk,
                )
                emitted_bp_credit = dict(
                    propose_stats.get("emitted_by_proposer")
                    or densify_tmp.get("emitted_by_proposer")
                    or {}
                )
            finalize_iter_mcts(
                mcts_runner,
                selected_polys=selected_polys,
                group_id=group_id,
                transform=transform,
                propose_stats=propose_stats,
                mcts_telem=mcts_telem,
                motif_keys=motif_keys_arch,
                motif_ttl=motif_ttl,
                credit_motif=int(motif_refine_n) > 0 or int(repack_m) > 0,
                refine_bp=refine_bp_credit if mcts_runner.agent is not None else None,
                emitted_bp=emitted_bp_credit if mcts_runner.agent is not None else None,
            )
            # H2: finalize before outer expand so MotifJoin.realized feeds next path;
            # gainful hole skip-miss already set — record_outer stamps AMAF (no double expand).
            if int(propose_stats.get("hole_expand_gain", 0) or 0) > 0:
                propose_stats["hole_amaf_stamp"] = 1
                if isinstance(propose_stats.get("void_leak"), dict):
                    propose_stats["void_leak"]["hole_amaf_stamp"] = 1
        if mcts_runner.agent is not None:
            mcts_runner.agent.prior_motif_graph_hit_n = int(
                propose_stats.get("motif_graph_hit_n", 0) or 0
            )
        # Q142: one global age tick — Motif + niche + arena idle/tombstone.
        motif_telem = age_iter_archives(
            mcts_runner=mcts_runner,
            mcts_parent_id=int(mcts_parent_id),
            mcts_telem=mcts_telem,
            propose_stats=propose_stats,
            cfg_propose=cfg.propose,
            pattern_accept=bool(pattern_accept),
            motif_refine_n=int(motif_refine_n),
        )
        cov = _selection_coverage_pct(
            selected_polys, group_id, part_areas, board_area,
        )
        cov_frac = float(cov) / 100.0 if board_area > 0 else 0.0
        if cov_frac > best_pack_cov + 1e-9:
            best_pack_cov = cov_frac
            best_pack_sel = list(selected_polys)
            best_pack_sig = best_pack_geom_sig(polys, selected_polys)
            best_pack_tf = [tuple(t) for t in transform]
            best_pack_seed_n = int(
                nest_state.seed_count if nest_state is not None else 0
            )
        # D0: refresh kiss/fits after post-pack so leaf_reward sees final layout.
        if selected_polys and part_bases is not None:
            try:
                post_geoms = _native_geoms_from_transforms(
                    [group_id[i] for i in selected_polys if int(i) < len(group_id)],
                    [transform[i] for i in selected_polys if int(i) < len(transform)],
                    part_bases,
                )
                write_propose_kiss_ledger(
                    propose_stats,
                    post_geoms,
                    gap=float(min_dist),
                    feed_snapshot=True,
                )
            except Exception:
                pass
            free_post = getattr(post_prep, "free_post", None) if post_prep else None
            if free_post is not None:
                propose_stats["fits_part"] = int(
                    bool(getattr(free_post, "fits_part", True))
                )
                propose_stats["fit_in_r"] = float(
                    getattr(free_post, "fit_in_r", 0.0) or 0.0
                )
                propose_stats["fit_need_r"] = float(
                    getattr(free_post, "fit_need_r", 0.0) or 0.0
                )
                propose_stats["fit_need_w"] = float(
                    getattr(free_post, "fit_need_w", 0.0) or 0.0
                )
                mcts_telem["last_fits_part"] = int(propose_stats["fits_part"])
        # Macro-MCTS: record expand reward (Q68/Q69); upsert contact motifs when improved.
        if mcts_action is not None and mcts_runner.agent is not None:
            # R1 repair Δ backprop lives in record_outer_iter_expand (shared with
            # nesting_evaluator early dual) — never a second full leaf here.
            propose_stats["board_area"] = float(board_area)
            mcts_parent_id, child_snap = record_outer_iter_expand(
                mcts_runner,
                parent_id=mcts_parent_id,
                action=mcts_action,
                selected_polys=selected_polys,
                group_id=group_id,
                transform=transform,
                ngroups=int(cfg.rules.ngroups),
                coverage_pct=float(cov),
                propose_stats=propose_stats,
                mcts_telem=mcts_telem,
                nest_state=nest_state,
                part_bases=part_bases,
                part_areas=part_areas,
                min_dist=float(cfg.board_min_dist_for(p_sheet)),
                t_expand0=t_expand0,
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
                rule_ids=mcts_rule_ids(rule_sets),
                rule_sets=rule_sets,
            )
            if child_snap is not None and mcts_action is not None:
                if "cheap_outer_reward_delta" not in mcts_telem:
                    realized = getattr(
                        getattr(mcts_runner, "agent", None), "realized", None
                    ) or {}
                    outer_r = leaf_reward(
                        child_snap,
                        rule_id=int(getattr(mcts_action, "rule_id", 0) or 0),
                        survive_motif_n=int(realized.get("survive_motif_n", 0) or 0),
                        macro_survive_n=int(realized.get("macro_survive_n", 0) or 0),
                        member_hits=int(realized.get("member_hits", 0) or 0),
                        materialized_motif=int(
                            realized.get("materialized_motif", 0) or 0
                        ),
                    )
                    cheap_r = float(mcts_telem.get("last_cheap_reward", 0.0) or 0.0)
                    mcts_telem["cheap_outer_reward_delta"] = float(outer_r) - cheap_r
            _exec_last["snap"] = child_snap
            if plateau.on_plateau and float(cov) >= float(plateau.last_cov or 0.0):
                mcts_telem["plateau_rule_boost"] = 1
                mcts_telem["last_rule_id"] = int(
                    getattr(mcts_action, "rule_id", 0) or 0
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
        free_post = getattr(post_prep, "free_post", None)
        if free_post is not None:
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
            write_telem(
                str(len(polys)),
                refine_msg,
                f"cov={cov:.1f}%",
                f"dfs_passes={int(polish_budget.dfs_passes)}",
                f"mcts_heavy={int(polish_budget.mcts_heavy)}",
            )
        render_frame = shapely_box(*padded_board_bounds(p_outline, sheet_pad))
        im = render_polys(
            render_frame,
            (tuple(polys[i] for i in selected_polys),),
            im_shape=render_size,
            nest_outline=p_sheet,
        )
        video.write(im)
        cv.imwrite(out.snapshot_path, im)

        selected_arms: list[list[Any]] = [[], []]
        for i in selected_polys:
            gi = int(group_id[i])
            selected_arms[gi].append(transform[i])
        selected_t = tuple(np.array(t) for t in selected_arms)
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
        nest_natives = (
            list(mpg_natives)
            if mpg_natives and len(mpg_natives) == len(polys)
            else _native_geoms_from_transforms(
                group_id,
                transform,
                part_bases,
                seed_polys=polys,
                seed_count=seed_n,
            )
        )
        nest_state = NestState(
            polys=polys,
            group_id=group_id,
            transform=transform,
            selected_indices=list(selected_polys),
            seed_count=seed_n,
            _native_geoms=nest_natives,
        )
        rule_sets = inject_repulsor_rules(rule_sets, cfg, p_sheet, nest_state)
        last_cov_frac = float(cov) / 100.0 if board_area > 0 else 0.0
        last_parts = int(len(selected_polys))
        ckpt_iter = int(getattr(out, "checkpoint_iter", 0) or 0)
        if (
            ckpt_iter > 0
            and int(_it) + 1 == ckpt_iter
            and nest_state is not None
        ):
            tip = mcts_runner.snapshot_at(int(mcts_parent_id), mcts_root)
            ckpt_path = str(
                getattr(out, "checkpoint_path", "")
                or "artifacts/checkpoints/void_fill_seed0_iter20.npz"
            )
            indep_ok = True
            if nest_state.selected_indices:
                indep_ok = bool(
                    selection_pairwise_independent(
                        nest_state.polys, nest_state.selected_indices
                    )
                )
            save_nest_checkpoint(
                ckpt_path,
                iter_n=ckpt_iter,
                seed=cfg.sampling.seed,
                coverage=last_cov_frac,
                parts=last_parts,
                independent_ok=indep_ok,
                nest_state=nest_state,
                best_pack_cov=best_pack_cov,
                best_pack_sel=best_pack_sel,
                best_pack_sig=best_pack_sig,
                best_pack_tf=best_pack_tf,
                best_pack_seed_n=best_pack_seed_n,
                void_elite_by_group=void_elite_by_group,
                history=history,
                graph_valid_carry=graph_valid_carry,
                motif_base=mcts_runner.motif_base,
                niche_archive=mcts_runner.niche_archive,
                tip_snap=tip,
                active_rule_id=int(mcts_telem.get("last_rule_id", 0) or 0),
                n_rule_sets=len(rule_sets),
            )
            print(
                f"[checkpoint] saved iter={ckpt_iter} cov={last_cov_frac:.4f} "
                f"parts={last_parts} path={ckpt_path}"
            )

    if resume_ckpt is not None:
        d_cov = float(last_cov_frac) - float(ckpt_baseline_cov)
        d_parts = int(last_parts) - int(ckpt_baseline_parts)
        print(
            f"[checkpoint] late Δcov={d_cov:+.4f} Δparts={d_parts:+d} "
            f"(final cov={last_cov_frac:.4f} parts={last_parts} "
            f"vs ckpt cov={ckpt_baseline_cov:.4f} parts={ckpt_baseline_parts})"
        )

    if (
        nest_state is not None
        and best_pack_sel is not None
        and best_pack_cov > 0.0
        and board_area > 0
    ):
        restored_sel, did_restore, bp_telem, restored_tf = maybe_restore_best_pack(
            best=BestPackSnapshot(
                selected_polys=list(best_pack_sel),
                cov=float(best_pack_cov),
                geom_sig=float(best_pack_sig),
                transforms=list(best_pack_tf) if best_pack_tf is not None else None,
                seed_count=int(best_pack_seed_n),
            ),
            current_selected=list(nest_state.selected_indices or ()),
            graph=graph if "graph" in locals() else (graphs[-1] if graphs else None),
            polys=nest_state.polys,
            group_id=nest_state.group_id,
            part_areas=part_areas,
            sheet=p_sheet,
            holes=user_holes,
            usable_area=float(board_area),
            geoms=list(nest_state.native_geoms or ()) or None,
        )
        leak = propose_stats.get("void_leak")
        if not isinstance(leak, dict):
            leak = {}
            propose_stats["void_leak"] = leak
        leak.update(bp_telem)
        if did_restore:
            tf_use = (
                list(restored_tf)
                if restored_tf is not None
                else list(nest_state.transform)
            )
            nest_state = NestState(
                polys=nest_state.polys,
                group_id=nest_state.group_id,
                transform=tf_use,
                selected_indices=list(restored_sel),
                seed_count=int(
                    best_pack_seed_n
                    if best_pack_seed_n
                    else (nest_state.seed_count or 0)
                ),
                _native_geoms=nest_state.native_geoms,
            )

    # Q84: hard stop if final selection is not packing-independent.
    if nest_state is not None and nest_state.selected_indices:
        if not selection_pairwise_independent(
            nest_state.polys, nest_state.selected_indices
        ):
            raise RuntimeError(
                "Macro-MCTS cutover: final selection failed independent_ok (Q84)"
            )

    video.release()


def main():
    import argparse

    parser = argparse.ArgumentParser(description="Nest Macro-MCTS build_graph demo")
    parser.add_argument("--checkpoint-iter", type=int, default=None)
    parser.add_argument("--checkpoint-path", type=str, default=None)
    parser.add_argument("--resume-checkpoint", type=str, default=None)
    parser.add_argument("--extra-iters", type=int, default=None)
    args, _unknown = parser.parse_known_args()
    cfg = BuildGraphConfig.from_env()
    out_upd: dict[str, Any] = {}
    if args.checkpoint_iter is not None:
        out_upd["checkpoint_iter"] = int(args.checkpoint_iter)
    if args.checkpoint_path is not None:
        out_upd["checkpoint_path"] = str(args.checkpoint_path)
    if args.resume_checkpoint is not None:
        out_upd["resume_checkpoint"] = str(args.resume_checkpoint)
    if args.extra_iters is not None:
        out_upd["extra_iters"] = int(args.extra_iters)
    if out_upd:
        cfg = cfg.model_copy(
            update={"output": cfg.output.model_copy(update=out_upd)}
        )
    run_build_graph(cfg)


if __name__ == "__main__":
    main()
