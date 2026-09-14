"""Unified pipeline evaluator for nesting quality benchmarks."""

import time
from dataclasses import dataclass

import numpy as np
from shapely.geometry import Point, Polygon
from shapely.geometry.base import BaseGeometry
from shapely.ops import unary_union

from nest_graph.board import board_context_from_geometry
from nest_graph.build_graph import (
    NestState,
    PlateauTracker,
    _make_initial_rule_sets,
    _mcts_rule_ids,
    _native_geoms_from_transforms,
    _note_path_accept_warm,
    _pack_cache_overlap_ok,
    _path_accept_contact_upsert,
    _selection_budget_for_iter,
    _selection_coverage_pct,
    _sync_agent_motif_cohorts,
    active_rule_set,
    improve_rules,
    make_polygon_graph,
    nest_state_extra_voids,
    void_elite_tuple_from_archive,
)
from nest_graph.graph import region_to_zone, BoardSnapshot, MacroRegion
from nest_graph.pack.cheap import pack_execute_snapshot, with_isolated_pack_cache
from nest_graph.pack.ctx import PackIterCtx, RefinePackBox
from nest_graph.pack.credit import finalize_iter_mcts, run_void_leak_and_niche_credit
from nest_graph.pack.epoch import inject_cohorts_and_bind_graph
from nest_graph.pack.execute import (
    board_snapshot_from_selection,
    make_execute_fn,
    prep_selection_freeze,
    record_outer_iter_expand,
    schedule_prep_selection_free,
)
from nest_graph.pack.macro_path import ancestors, macro_increase_path
from nest_graph.graph import leaf_reward, path_reward_beats
from nest_graph.pack.stages import run_mid_pack_stages, run_post_pack_stage
from nest_graph.propose.placement_common import post_pack_overlap_ok
from nest_graph.propose.telem import (
    BestPackSnapshot,
    archive_void_elite_transforms,
    best_pack_geom_sig,
    build_run_diagnostics,
    maybe_restore_best_pack,
    record_overlap_reject,
    void_elite_count,
)
from nest_graph.pack.motif_credit import (
    credit_void_niche_from_iter,
    merge_void_elite_with_archive,
)
from nest_graph.pack.runner import MacroMctsRunner
from nest_graph.propose.pattern_archive import (
    motif_patterns_for_inject,
    note_motif_ref_anchors_from_nest,
)
from nest_graph.propose.telem import assemble_void_leak
from nest_graph.propose.heavy_polish import (
    apply_dfs_refinement,
    apply_refine_with_restore,
    polish_budget_for_iter,
    run_improve_rules_rounds,
)
from nest_graph.propose.context import late_border_saturation_info
from nest_graph.propose.transform_batch import (
    build_transform_batch,
    graph_valid_carry_by_group,
)
from nest_graph.propose.void_selection import (
    count_graph_in_free,
    count_props_in_free,
    count_props_near_pole,
    count_selected_by_proposer,
    count_selected_in_free,
    format_prop_accept,
    transform_row_key,
    void_pole_near_radius,
    zones_have_void_hijack,
)
from nest_graph.propose.selection_compose import (
    compose_and_nest_selection,
    compose_nest_kwargs,
    dual_nest_for,
    sheet_diag_from,
)
from nest_graph.config import BuildGraphConfig, ProposeConfig, score_rules_options
from nest_graph.graph import (
    FinalizeSelectionOptions,
    finalize_selection,
    score_elems,
    selection_is_independent,
)
from nest_graph.geometry import Geometry, find_polygon_distances_bipartite
from nest_graph.placement_scene import placement_clearance_epsilon
from nest_graph.propose import (
    ProposeGeometry,
    border_focal_for_propose,
    collect_propose_candidates,
    effective_ranking_mode,
    obstacle_shape_for_propose,
)
from nest_graph.propose.placement_common import selection_pairwise_independent
from nest_graph.propose.context import (
    outline_coverage_ratio,
    part_is_concave,
    propose_push_point,
    should_use_border_focus,
)
from nest_graph.propose.pipeline import propose_coords_from_candidates
from nest_graph.propose.post_pack import prepare_post_pack
from nest_graph.propose.void_selection import colonize_void_onto_base
from nest_graph.utils import transform_poly
from scripts.nesting_fixtures import NestCase


def _seed_transforms_by_group(case: NestCase) -> list[np.ndarray]:
    """Initial selected transforms from NestCase.seed_placements."""
    by_group: list[list[list[float]]] = [[] for _ in case.groups]
    gid_to_idx = {gid: i for i, (_poly, gid) in enumerate(case.groups)}
    for _poly, gid, t in case.seed_placements:
        idx = gid_to_idx.get(gid)
        if idx is None:
            continue
        by_group[idx].append([float(t[0]), float(t[1]), float(t[2])])
    out: list[np.ndarray] = []
    for rows in by_group:
        if rows:
            out.append(np.asarray(rows, dtype=np.float64))
        else:
            out.append(np.zeros((0, 3), dtype=np.float64))
    return out


def _case_user_holes(case: NestCase) -> tuple[tuple[tuple[float, float], ...], ...]:
    return tuple(tuple(h.exterior.coords) for h in case.board_holes)


def _build_seed_state(case: NestCase) -> tuple[list, list[int], list[np.ndarray]]:
    """Return (seed_polys, seed_gids, seed_transforms) from NestCase.seed_placements."""
    seed_t = _seed_transforms_by_group(case)
    seed_polys: list = []
    seed_gids: list[int] = []
    seed_tr: list[np.ndarray] = []
    if not any(t.shape[0] > 0 for t in seed_t):
        return seed_polys, seed_gids, seed_tr
    for gi, (poly, gid) in enumerate(case.groups):
        for row in seed_t[gi]:
            seed_polys.append(transform_poly(poly, row))
            seed_gids.append(int(gid))
            seed_tr.append(np.asarray(row, dtype=np.float64))
    return seed_polys, seed_gids, seed_tr


def _seed_extra_voids(seed_polys: list) -> list[Geometry] | None:
    if not seed_polys:
        return None
    return [Geometry.from_shapely(p) for p in seed_polys]


def _selection_validity(
    case: NestCase,
    sheet: Polygon,
    selected_polys: list[int],
    group_id: list[int],
    transform: np.ndarray,
    graph,
    *,
    seed_polys: list | None = None,
    min_dist: float = 0.0,
) -> tuple[bool, bool]:
    """Return (overlap_ok, void_ok) for the final selection (incl. seed clearance)."""
    if not selected_polys:
        return True, True

    selected_set = set(selected_polys)
    overlap_ok = True
    if graph is not None:
        for i in selected_polys:
            for j in graph.collisions[i]:
                if j in selected_set:
                    overlap_ok = False
                    break
            if not overlap_ok:
                break

    placed = [
        transform_poly(case.groups[group_id[i]][0], transform[i])
        for i in selected_polys
    ]
    # Selection must not collide with locked seeds (beyond clearance).
    if overlap_ok and seed_polys:
        eps = max(min_dist - 1e-6, 0.0)
        for poly in placed:
            for seed in seed_polys:
                if poly.intersects(seed) or poly.distance(seed) < eps:
                    overlap_ok = False
                    break
            if not overlap_ok:
                break

    void_ok = True
    for poly in placed:
        # Footprint must stay in the nestable sheet (tiny shrink for numeric edges).
        if not sheet.buffer(1e-5).covers(poly):
            void_ok = False
            break
        for hole in case.board_holes:
            inter = poly.intersection(hole)
            if not inter.is_empty and inter.area > 1e-6:
                void_ok = False
                break
        if not void_ok:
            break
    return overlap_ok, void_ok


@dataclass
class ProposeBenchmarkMetrics:
    preset: str
    scenario: str
    seed: int
    valid_count: int
    top_clearance_mean: float
    top_clearance_min: float
    contact_dist_mean: float
    contact_dist_min: float
    kiss_fraction: float
    raw_pool_size: int
    final_count: int
    graph_nodes: int
    graph_nodes_vs_random: int
    propose_time_s: float


def _contact_distance(placed_shapely: BaseGeometry, base_shape: BaseGeometry) -> float:
    if base_shape is None or base_shape.is_empty:
        return float("inf")
    return float(base_shape.distance(placed_shapely))


def evaluate_proposal_coords(
    coords_list: list[tuple[float, float, float]],
    board: Polygon,
    base_shape: BaseGeometry,
    part_poly: Polygon,
    min_dist: float,
    pt_push: Point,
    epsilon_ratio: float,
) -> tuple[int, float, float, float, float, float, float]:
    if not coords_list:
        return 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0

    geom = ProposeGeometry(
        board, base_shape, part_poly, min_dist, epsilon_ratio=epsilon_ratio,
    )
    margin = min_dist + placement_clearance_epsilon(min_dist, ratio=epsilon_ratio)
    clearances: list[float] = []
    contacts: list[float] = []
    kisses = 0
    valid = 0
    border_errs: list[float] = []

    for coords in coords_list:
        placed_g = geom.placed_at(coords)
        placed = transform_poly(part_poly, coords)
        if not geom.valid(placed_g, pt_push, (coords[0], coords[1])):
            continue
        valid += 1
        cd = _contact_distance(placed, base_shape)
        if cd < float("inf"):
            contacts.append(cd)
            if cd <= margin + 1e-5:
                kisses += 1
        g = geom.placement_guidance(placed_g, (coords[0], coords[1]), pt_push)
        if not g.is_penetrating:
            clearances.append(float(g.clearance))
        sheet, _ = board_context_from_geometry(board)
        border_errs.append(abs(float(placed.distance(sheet.exterior)) - min_dist))

    kiss_frac = kisses / valid if valid else 0.0
    return (
        valid,
        float(np.mean(clearances)) if clearances else 0.0,
        float(np.min(clearances)) if clearances else 0.0,
        float(np.mean(contacts)) if contacts else 0.0,
        float(np.min(contacts)) if contacts else 0.0,
        kiss_frac,
        float(np.min(border_errs)) if border_errs else 0.0,
    )


@dataclass
class NestingMetrics:
    parts_final: int
    area_coverage: float
    outline_coverage: float
    independent_ok: bool
    overlap_ok: bool
    void_ok: bool
    graph_nodes: int
    proposal_yield: float
    time_s: float
    parts_seed: int = 0
    area_coverage_seed: float = 0.0
    parts_delta: int = 0
    area_coverage_delta: float = 0.0
    kiss_fraction: float = 0.0  # alias of kiss_seed for compatibility
    kiss_seed: float = 0.0
    kiss_outline: float = 0.0
    kiss_standoff: float = 0.0
    contact_min: float = 0.0
    clearance_p50: float = 0.0
    largest_free_comp_area: float = 0.0
    largest_free_over_part: float = 0.0
    void_props: int = 0
    void_graph: int = 0
    void_selected_nest: int = 0
    void_selected_refine: int = 0
    free_kind: str = ""
    border_standoff_err: float = 0.0
    time_to_frac_final: float = -1.0
    density_auc: float = 0.0
    coverage_trajectory: tuple[tuple[float, float, int], ...] = ()


def _angle_allowed(
    angle: float,
    allowed: tuple[float, ...] | None,
    *,
    tol: float = 0.05,
) -> bool:
    if allowed is None:
        return True
    two_pi = 2.0 * np.pi
    a = float(angle) % two_pi
    for b in allowed:
        d = abs(((a - (float(b) % two_pi) + np.pi) % two_pi) - np.pi)
        if d <= tol:
            return True
    return False


def _shapes_gate_ok(shapes, sheet, holes, *, overlap_area_tol: float = 1e-6) -> bool:
    """Hard-overlap + board membership (same family as gate overlap_ok/void_ok)."""
    for a, pa in enumerate(shapes or ()):
        if pa is None or getattr(pa, "is_empty", False):
            continue
        if sheet is not None:
            try:
                outside = pa.difference(sheet.buffer(1e-4))
                if (
                    outside is not None
                    and not getattr(outside, "is_empty", True)
                    and float(outside.area) > overlap_area_tol
                ):
                    return False
            except Exception:
                if not sheet.buffer(1e-4).covers(pa):
                    return False
        for hole in holes or ():
            inter = pa.intersection(hole)
            if not inter.is_empty and inter.area > overlap_area_tol:
                return False
        for b in range(a + 1, len(shapes)):
            pb = shapes[b]
            if pb is None or getattr(pb, "is_empty", False):
                continue
            if pa.intersects(pb) and pa.intersection(pb).area > overlap_area_tol:
                return False
    return True


def _shapes_pairwise_ok(shapes, *, overlap_area_tol: float = 1e-12) -> bool:
    """Pairwise hard-overlap only (match gate overlap_ok area tol)."""
    for a, pa in enumerate(shapes or ()):
        if pa is None or getattr(pa, "is_empty", False):
            continue
        for b in range(a + 1, len(shapes)):
            pb = shapes[b]
            if pb is None or getattr(pb, "is_empty", False):
                continue
            if pa.intersects(pb) and pa.intersection(pb).area > overlap_area_tol:
                return False
    return True


def metrics_meet_floors(metrics: NestingMetrics, floors) -> list[str]:
    """Return list of failed floor names (empty if all pass)."""
    fails: list[str] = []
    if metrics.parts_final < floors.parts_final:
        fails.append("parts_final")
    if metrics.area_coverage < floors.area_coverage:
        fails.append("area_coverage")
    if metrics.time_s > floors.time_s:
        fails.append("time_s")
    if metrics.kiss_seed < floors.kiss_seed:
        fails.append("kiss_seed")
    if metrics.kiss_outline < floors.kiss_outline:
        fails.append("kiss_outline")
    if metrics.kiss_standoff < floors.kiss_standoff:
        fails.append("kiss_standoff")
    if metrics.outline_coverage < floors.outline_coverage:
        fails.append("outline_coverage")
    if metrics.density_auc < floors.density_auc:
        fails.append("density_auc")
    max_free = getattr(floors, "largest_free_over_part", float("inf"))
    if max_free < float("inf") and metrics.largest_free_over_part > max_free:
        fails.append("largest_free_over_part")
    max_clear = getattr(floors, "clearance_p50", float("inf"))
    if max_clear < float("inf") and metrics.clearance_p50 > max_clear:
        fails.append("clearance_p50")
    if not metrics.independent_ok:
        fails.append("independent_ok")
    if not metrics.overlap_ok:
        fails.append("overlap_ok")
    if not metrics.void_ok:
        fails.append("void_ok")
    return fails


class NestingPipelineEvaluator:
    def __init__(
        self,
        case: NestCase,
        cfg: BuildGraphConfig,
        *,
        always_heavy_polish: bool = False,
    ):
        self.case = case
        self.always_heavy_polish = bool(always_heavy_polish)
        self.user_holes = _case_user_holes(case)
        self.sheet, _ = board_context_from_geometry(
            case.board, user_holes=self.user_holes,
        )
        max_verts = 0
        max_interiors = 0
        concave_parts = False
        for poly, _gid in case.groups:
            max_verts = max(max_verts, len(list(poly.exterior.coords)))
            max_interiors = max(
                max_interiors, len(getattr(poly, "interiors", ()) or ()),
            )
            if part_is_concave(poly):
                concave_parts = True
        n_holes = len(case.board_holes)
        sheet_vertices = (
            len(list(self.sheet.exterior.coords))
            if hasattr(self.sheet, "exterior") else 0
        )
        seeded = bool(case.seed_placements)
        cfg = cfg.with_runtime_lean(
            n_holes=n_holes,
            max_part_vertices=max_verts,
            max_part_interiors=max_interiors,
            sheet_vertices=sheet_vertices,
            concave_parts=concave_parts,
            seeded=seeded,
        )
        # Modest void_fill elite only (DG mix); do not inflate transform pool.
        if "void_fill" in (case.tags or ()):
            cfg = cfg.model_copy(
                update={
                    "propose": cfg.propose.model_copy(
                        update={
                            "stratified_void_elite_quota": max(
                                int(cfg.propose.stratified_void_elite_quota), 24,
                            ),
                        }
                    ),
                }
            )
        self.cfg = cfg
        self.parts = list(case.groups)
        self.last_result: dict | None = None

    def _min_dist(self, *, first_pass: bool = False) -> float:
        return self.cfg.board_min_dist_for(self.sheet, first_pass=first_pass)

    def _initial_selected_t(
        self,
        rng: np.random.Generator,
    ) -> tuple[tuple[np.ndarray, ...], tuple[np.ndarray, ...]]:
        sc = self.cfg.sampling
        selected_t: list[np.ndarray] = []
        history: list[np.ndarray] = []
        for _ in self.case.groups:
            selected_t.append(
                rng.uniform(-1, 1, (sc.initial_random, 3)) * sc.transform_scale
            )
            history.append(np.zeros((1, 3), dtype=np.float64))
        return tuple(selected_t), tuple(history)

    def prepare_first_iteration(self, seed: int):
        """Build the graph for the first iteration (useful for isolated DFS/Propose benchmarks)."""
        rng = np.random.default_rng(seed)
        sel = self.cfg.selection
        selected_t, history = self._initial_selected_t(rng)
        rule_sets = _make_initial_rule_sets(self.cfg)
        seed_polys, seed_gids, seed_tr = _build_seed_state(self.case)
        nest_state = None
        if seed_polys:
            nest_state = NestState(
                polys=list(seed_polys),
                group_id=list(seed_gids),
                transform=list(seed_tr),
                selected_indices=list(range(len(seed_polys))),
                seed_count=len(seed_polys),
            )
        extra_voids = _seed_extra_voids(seed_polys)

        selected_t = build_transform_batch(
            self.cfg, selected_t, history, rng,
            board=self.sheet, parts=self.parts, nest_state=nest_state,
            first_pass=nest_state is None,
            group_allowed_angles=self.case.group_allowed_angles,
        )

        flat_parts = [
            (self.parts[group_idx][0], transforms)
            for group_idx, transforms in enumerate(selected_t)
        ]
        graph, polys, group_id, transform = make_polygon_graph(
            self.case.board,
            flat_parts,
            min_dist=self._min_dist(first_pass=nest_state is None),
            epsilon_ratio=self.cfg.propose.placement_clearance_epsilon_ratio,
            user_holes=self.user_holes,
            extra_voids=extra_voids,
        )

        graphs = [graph]
        for round_idx in range(sel.improve_rules_rounds):
            rule_sets = improve_rules(
                graphs,
                rule_sets,
                sel.rules_kept,
                self.case.board,
                mutation_presets=self.cfg.rules.mutation_presets(),
                rule_score_penalty=sel.rule_score_penalty,
                elite_count=sel.improve_rules_elite_count,
                seed=int(rng.integers(0, 2**31)) + round_idx,
                score_options=score_rules_options(sel),
                max_rules_per_set=self.cfg.rules.max_rules_per_set,
            )

        active_rules = active_rule_set(rule_sets)
        scores = score_elems(graph, active_rules)
        selected = list(nest_by_graph(graph, rule_sets[: sel.nest_rule_sets_used])[0])
        return graph, active_rules, selected, scores, polys, group_id, transform

    def run_propose_only(self, seed: int, preset_label: str) -> ProposeBenchmarkMetrics:
        """Run only the propose phase for the first part group, using seed_placements as obstacles."""
        min_dist = self._min_dist()
        eps = self.cfg.propose.placement_clearance_epsilon_ratio

        placed_polys = [p for p, _, _ in self.case.seed_placements]
        part_poly, _ = self.case.groups[0]

        obstacle = obstacle_shape_for_propose(placed_polys, part_poly, min_dist)
        border_focus = should_use_border_focus(obstacle, self.cfg.propose)
        push = propose_push_point(
            self.sheet,
            obstacle,
            smart_push=self.cfg.propose.smart_push_target,
            min_dist=min_dist,
            use_border_focus=border_focus,
        )

        focal = None
        if border_focus:
            focal = border_focal_for_propose(self.sheet, min_dist)
        elif obstacle is not None and not obstacle.is_empty:
            focal = obstacle

        geom = ProposeGeometry(
            self.sheet, obstacle, part_poly, min_dist,
            epsilon_ratio=eps,
            propose_cfg=self.cfg.propose,
        )
        rank_mode = effective_ranking_mode(self.cfg.propose, obstacle)

        t0 = time.perf_counter()
        raw = collect_propose_candidates(
            obstacle,
            part_poly,
            self.sheet,
            self.cfg.propose,
            min_dist=min_dist,
            pt_push=push,
            propose_geom=geom,
            focal_shape=focal,
        )
        final = propose_coords_from_candidates(
            obstacle,
            part_poly,
            self.sheet,
            self.cfg.propose,
            min_dist=min_dist,
            pt_push=push,
            candidates=raw,
            rank_mode=rank_mode,
            focal_shape=focal,
        )
        elapsed = time.perf_counter() - t0

        valid, c_mean, c_min, cd_mean, cd_min, kiss_frac, _border_min = evaluate_proposal_coords(
            final, self.sheet, obstacle, part_poly, min_dist, push, eps,
        )

        proposals = np.asarray(final, dtype=np.float64) if final else np.zeros((0, 3))
        rng = np.random.default_rng(seed)
        random_t = rng.uniform(-0.2, 0.2, (8, 3)) * [0.4, 0.4, np.pi]

        graph_rand, _, _, _ = make_polygon_graph(
            self.case.board, [(part_poly, random_t)], min_dist=0.0,
            user_holes=self.user_holes,
        )
        n_rand = len(graph_rand.elems)

        if proposals.shape[0] == 0:
            n_both = n_rand
        else:
            graph_both, _, _, _ = make_polygon_graph(
                self.case.board,
                [(part_poly, np.vstack([random_t, proposals]))],
                min_dist=0.0,
                user_holes=self.user_holes,
            )
            n_both = len(graph_both.elems)

        return ProposeBenchmarkMetrics(
            preset=preset_label,
            scenario=self.case.name,
            seed=seed,
            valid_count=valid,
            top_clearance_mean=c_mean,
            top_clearance_min=c_min,
            contact_dist_mean=cd_mean,
            contact_dist_min=cd_min,
            kiss_fraction=kiss_frac,
            raw_pool_size=len(raw),
            final_count=len(final),
            graph_nodes=n_both,
            graph_nodes_vs_random=n_both - n_rand,
            propose_time_s=elapsed,
        )

    def run_full_pipeline(self, seed: int) -> NestingMetrics:
        """Run a short nest loop for the case and return standardized metrics."""
        t0 = time.perf_counter()
        rng = np.random.default_rng(seed)
        sel = self.cfg.selection
        selected_t, history = self._initial_selected_t(rng)
        rule_sets = _make_initial_rule_sets(self.cfg)

        selected_polys: list[int] = []
        polys: list = []
        group_id: list[int] = []
        transform = np.zeros((0, 3))
        graph = None
        nest_state: NestState | None = None

        seed_polys, seed_gids, seed_tr = _build_seed_state(self.case)
        if seed_polys:
            nest_state = NestState(
                polys=list(seed_polys),
                group_id=list(seed_gids),
                transform=list(seed_tr),
                selected_indices=list(range(len(seed_polys))),
                seed_count=len(seed_polys),
            )
        
        extra_voids = _seed_extra_voids(seed_polys)

        next_polys = list(seed_polys)
        next_gids = list(seed_gids)
        next_tr = list(seed_tr)
        next_sel = list(range(len(seed_polys)))
        prev_packed_gid: list[int] = []
        prev_packed_tr: list = []
        trajectory: list[tuple[float, float, int]] = []
        best_cov = 0.0
        best_next_polys: list | None = None
        best_next_gids: list | None = None
        best_next_tr: list | None = None
        best_next_sel: list | None = None
        best_selected_polys: list[int] | None = None
        best_pack_sig: float = 0.0
        last_void_leak: dict = {
            "free_kind": "",
            "props": 0,
            "graph": 0,
            "nest": 0,
            "refine": 0,
        }
        last_proposal_yield = 0.0
        had_void_override = False
        void_elite_by_group: dict[int, list[np.ndarray]] = {}
        graph_valid_carry: tuple[np.ndarray, ...] = tuple(
            np.zeros((0, 3), dtype=np.float64) for _ in range(max(len(self.parts), 1))
        )
        plateau = PlateauTracker(
            flat_iters=int(getattr(sel, "plateau_flat_iters", 3) or 3),
            cov_eps=float(getattr(sel, "plateau_cov_eps", 0.05) or 0.05),
        )
        pin_all_blocked_streak = 0
        part_bases_fixed = {
            i: Geometry.from_shapely(p[0]) for i, p in enumerate(self.parts)
        }
        # P0: DG spine (niche + Motif) alongside classic evaluator harness.
        mcts_runner = MacroMctsRunner()
        if mcts_runner.agent is not None:
            mcts_runner.agent.motif_cohorts = ()
        mcts_parent_id = 0
        mcts_action = None
        pack_cache: dict = {"ready": False}
        part_areas_t = tuple(float(p[0].area) for p in self.parts)
        board_area_f = float(self.sheet.area) if self.sheet is not None else 1.0
        mcts_runner.execute_fn = make_execute_fn(
            lambda parent, zone=None, action=None, patterns=None: pack_execute_snapshot(
                parent,
                zone=zone,
                action=action,
                patterns=patterns,
                pack_cache=pack_cache,
                rule_sets=rule_sets,
                sel=sel,
                cfg=self.cfg,
                apply_dfs_fn=apply_dfs_refinement,
                native_geoms_fn=_native_geoms_from_transforms,
                coverage_pct_fn=_selection_coverage_pct,
            )
        )
        prev_void_nest = 0
        inward_peak_state: dict[str, int] = {}
        letter_peak_state: dict[str, int] = {}

        for iter_idx in range(self.case.iters):
            first_pass = nest_state is None
            # Q69: last leaf = full DFS + post; mid = dfs_passes=1; always_heavy → last budget.
            is_last_leaf = bool(self.always_heavy_polish) or (
                int(iter_idx) >= int(self.case.iters) - 1
            )
            near_last = bool(self.always_heavy_polish) or (
                int(iter_idx) == int(self.case.iters) - 2
            )
            parent_free_hint = (
                str(last_void_leak.get("free_kind") or "") == "large_void"
            )
            polish_budget = polish_budget_for_iter(
                is_last_leaf=is_last_leaf,
                sel=sel,
                near_last=near_last,
                on_plateau=bool(plateau.on_plateau),
                free_remaining=True,
                large_void=True,
            )
            # I1/Q355: path probe + AMAF pick before propose (evaluator harness).
            mcts_telem: dict = {}
            mcts_force_zone = "void_seek"
            mcts_action = None
            if (
                nest_state is not None
                and mcts_runner.agent is not None
                and pack_cache.get("ready")
            ):
                _prev_ps = pack_cache.get("propose_stats")
                _sync_agent_motif_cohorts(
                    mcts_runner,
                    _prev_ps if isinstance(_prev_ps, dict) else None,
                )
                rem = tuple(range(max(len(self.parts), 1)))
                max_void_ratio = float(last_void_leak.get("max_void_ratio", 0.0) or 0.0)
                void_fill = float(max(0.0, min(1.0, 1.0 - (max_void_ratio / 10.0))))
                if str(last_void_leak.get("free_kind") or "") == "large_void" and max_void_ratio <= 0.0:
                    void_fill = 0.0
                parent_snap = BoardSnapshot(
                    remaining_gids=rem,
                    coverage=float(last_void_leak.get("outline_cov", 0.0) or 0.0),
                    free_kind=str(last_void_leak.get("free_kind") or ""),
                    arena_node_id=int(mcts_parent_id),
                    rim_fill=float(last_void_leak.get("rim_progress", 0.0) or 0.0),
                    void_fill=void_fill,
                )
                mcts_runner.store_snapshot(int(mcts_parent_id), parent_snap)
                mcts_telem = {
                    "pw_expand": int(
                        (mcts_runner.agent.telem or {}).get("pw_expand", 0) or 0
                    ),
                    "amaf_hits": int(
                        (mcts_runner.agent.telem or {}).get("amaf_hits", 0) or 0
                    ),
                    "macro_path_accept": int(
                        last_void_leak.get("macro_path_accept", 0) or 0
                    ),
                    "path_extend_n": int(last_void_leak.get("path_extend_n", 0) or 0),
                    "path_step_macro": int(
                        last_void_leak.get("path_step_macro", 0) or 0
                    ),
                    "path_step_join": int(
                        last_void_leak.get("path_step_join", 0) or 0
                    ),
                    "path_contact_upserts": int(
                        last_void_leak.get("path_contact_upserts", 0) or 0
                    ),
                }
                path_probe = bool(plateau.on_plateau) or parent_free_hint
                if path_probe and mcts_runner.execute_fn is not None:
                    path = ancestors(mcts_runner, int(mcts_parent_id))
                    mcts_telem["policy_path_len"] = int(len(path))
                    base_cov = float(getattr(parent_snap, "coverage", 0.0) or 0.0)
                    realized = getattr(mcts_runner.agent, "realized", None) or {}
                    base_r = leaf_reward(
                        parent_snap,
                        survive_motif_n=int(realized.get("survive_motif_n", 0) or 0),
                        macro_survive_n=int(realized.get("macro_survive_n", 0) or 0),
                    )
                    # Isolate pack_cache: path execute mutates compose_* (poisons real iter).
                    path_overlap_ok = False
                    with with_isolated_pack_cache(pack_cache):
                        alt_action, alt_reward, path_accept_snap = macro_increase_path(
                            mcts_runner,
                            leaf_id=int(mcts_parent_id),
                            baseline_reward=base_r,
                            execute_fn=mcts_runner.execute_fn,
                            rule_ids=_mcts_rule_ids(rule_sets),
                            beam=int(
                                getattr(self.cfg.propose, "macro_path_beam", 4) or 4
                            ),
                            max_depth=int(
                                getattr(self.cfg.propose, "macro_path_max_depth", 3)
                                or 3
                            ),
                            telem=mcts_telem,
                            overlap_ok_fn=lambda _s=None: _pack_cache_overlap_ok(
                                pack_cache
                            ),
                        )
                        path_overlap_ok = bool(_pack_cache_overlap_ok(pack_cache))
                    if (
                        alt_action is not None
                        and path_accept_snap is not None
                        and not path_reward_beats(
                            parent_snap,
                            path_accept_snap,
                            base_reward=base_r,
                            alt_reward=alt_reward,
                        )
                    ):
                        alt_action = None
                        path_accept_snap = None
                    if alt_action is not None:
                        mcts_telem["macro_path_candidate"] = 1
                        apply_path = bool(
                            getattr(
                                self.cfg.propose, "enable_macro_path_replay", False
                            )
                        )
                        alt_cov = float(
                            getattr(path_accept_snap, "coverage", 0.0) or 0.0
                        ) if path_accept_snap is not None else 0.0
                        cov_ok = (
                            path_accept_snap is not None
                            and bool(path_overlap_ok)
                            and alt_cov + 1e-9 >= base_cov + 0.005
                        )
                        is_motif = (
                            getattr(alt_action, "region", None) == MacroRegion.Motif
                        )
                        # S1: evaluator discover telem only — do not soft-apply Motif
                        # into mcts_action (path packs poisoned overlapping peaks).
                        if apply_path and path_accept_snap is not None and path_overlap_ok:
                            mcts_telem["macro_path_accept"] = int(
                                mcts_telem.get("macro_path_accept", 0) or 0
                            ) + 1
                            if cov_ok:
                                mcts_action = alt_action
                                mcts_telem["path_credit_n"] = int(
                                    mcts_telem.get("path_credit_n", 0) or 0
                                ) + 1
                            if bool(
                                getattr(
                                    self.cfg.propose, "mutate_motif_base_on_path", False
                                )
                            ):
                                mcts_telem["path_contact_upserts"] = int(
                                    mcts_telem.get("path_contact_upserts", 0) or 0
                                ) + _path_accept_contact_upsert(
                                    mcts_runner,
                                    path_accept_snap,
                                    part_bases=part_bases_fixed,
                                    min_dist=float(
                                        self.cfg.board_min_dist_for(self.sheet)
                                    ),
                                    motif_min_compactness=float(
                                        getattr(
                                            self.cfg.propose,
                                            "motif_min_compactness",
                                            0.35,
                                        )
                                        or 0.35
                                    ),
                                    motif_ttl=int(
                                        getattr(
                                            self.cfg.propose, "accepted_pattern_ttl", 4
                                        )
                                        or 4
                                    ),
                                    motif_max_keep=int(
                                        getattr(
                                            self.cfg.propose, "accepted_pattern_max", 4
                                        )
                                        or 4
                                    ),
                                    pack_cache=pack_cache,
                                    telem=mcts_telem,
                                )
                        elif cov_ok:
                            # Letter C credit without apply (Q351 OFF / S1).
                            mcts_telem["path_credit_n"] = int(
                                mcts_telem.get("path_credit_n", 0) or 0
                            ) + 1
                            if is_motif:
                                mcts_telem["macro_path_motif_soft"] = 1
                        elif apply_path:
                            mcts_telem["macro_path_overlap_skip"] = 1
                            record_overlap_reject(mcts_telem, stage="path")
                            agent = getattr(mcts_runner, "agent", None)
                            if agent is not None:
                                agent.note_macro_miss(alt_action)
                if mcts_action is None:
                    mcts_action = mcts_runner.agent.pick_expand_action(
                        rem,
                        rule_ids=_mcts_rule_ids(rule_sets),
                        parent_id=int(mcts_parent_id),
                        snapshot=parent_snap,
                    )
                if mcts_action is not None:
                    mcts_force_zone = region_to_zone(mcts_action.region)
                    mcts_telem["mcts_rule_id"] = int(
                        getattr(mcts_action, "rule_id", 0) or 0
                    )
            sel_iter = _selection_budget_for_iter(sel, on_plateau=plateau.on_plateau)
            sel_iter, freeze_reason = prep_selection_freeze(
                sel_iter,
                freeze_cheap_expand=bool(polish_budget.freeze_improve_rules),
                on_plateau=plateau.on_plateau,
                plateau_streak=int(plateau.streak),
                flat_iters=int(plateau.flat_iters),
                enable_incumbent_loop=bool(
                    getattr(self.cfg.propose, "enable_incumbent_loop", True)
                ),
            )
            del freeze_reason
            sat_info = late_border_saturation_info(
                self.cfg, nest_state, self.sheet,
                had_void_override=had_void_override,
            )
            if sat_info.sat_override:
                had_void_override = True
            propose_stats: dict = {
                "outline_cov": sat_info.outline_cov,
                "sat_override": sat_info.sat_override,
                "rim_progress": sat_info.rim_progress,
                "on_plateau": bool(plateau.on_plateau),
                "is_last_leaf": bool(is_last_leaf),
                "near_last": bool(near_last),
                "free_kind": str(last_void_leak.get("free_kind") or ""),
            }
            if inward_peak_state:
                propose_stats["inward_peak"] = dict(inward_peak_state)
            keep_hist_sterile = bool(
                (sat_info.sat_override or had_void_override)
                and bool(getattr(self.cfg.propose, "keep_history_on_void_sterile", True))
            )
            ngroups = max(len(self.parts) if self.parts else len(selected_t), 1)
            void_elite_t = void_elite_tuple_from_archive(void_elite_by_group, ngroups)
            # P0 hybrid: DG niche/Motif after first pack (keep rim first-pass SoT).
            use_dg_mix = nest_state is not None
            arch_rows = (
                mcts_runner.niche_archive.active_by_group(ngroups) if use_dg_mix else {}
            )
            merged_elite = void_elite_by_group
            if arch_rows:
                merged_elite = merge_void_elite_with_archive(
                    void_elite_by_group,
                    arch_rows,
                    elite_quota=int(
                        getattr(self.cfg.propose, "stratified_void_elite_quota", 15)
                        or 15
                    ),
                    void_seek=True,
                )
                void_elite_t = void_elite_tuple_from_archive(merged_elite, ngroups)
            elite_n = void_elite_count(merged_elite)
            propose_stats["void_elite_seeded"] = elite_n
            propose_stats["archive_elite_n"] = sum(
                len(v) for v in (arch_rows or {}).values()
            )
            propose_stats["keep_history_on_sterile"] = keep_hist_sterile
            propose_stats["dg_force_zone"] = mcts_force_zone if use_dg_mix else None
            propose_stats["mcts_zone"] = mcts_force_zone if use_dg_mix else None
            if mcts_telem:
                propose_stats.update({
                    k: mcts_telem[k]
                    for k in (
                        "path_extend_n",
                        "path_step_macro",
                        "path_step_join",
                        "macro_path_accept",
                        "macro_path_candidate",
                        "macro_chain_accept",
                        "path_contact_upserts",
                        "mcts_rule_id",
                        "policy_path_len",
                        "macro_swap_attempts",
                        "macro_swap_depth",
                    )
                    if k in mcts_telem
                })
            archived_for_propose = []
            if use_dg_mix and bool(
                getattr(self.cfg.propose, "enable_accepted_pattern_archive", True)
            ):
                note_motif_ref_anchors_from_nest(mcts_runner.motif_base, nest_state)
                archived_for_propose = motif_patterns_for_inject(
                    mcts_runner.motif_base,
                    max_keep=int(
                        getattr(self.cfg.propose, "accepted_pattern_max", 4) or 4
                    ),
                    part_bases=part_bases_fixed,
                    min_dist=float(self._min_dist(first_pass=False)),
                    telem=propose_stats,
                    polish=True,
                )
            propose_stats["accepted_patterns_n"] = len(archived_for_propose)
            propose_stats["motif_library_n"] = int(mcts_runner.motif_base.size())
            propose_stats["enabled_proposers_n"] = (
                len(ProposeConfig.proposers_for_place("void_seek") or ())
                if use_dg_mix else 0
            )
            cut_sterile_boost = bool(
                use_dg_mix
                and (
                    mcts_runner.niche_archive.any_void_miss_rate_high(0.8)
                    or int(mcts_runner.niche_archive.place_fail_streak) >= 3
                )
            )
            propose_stats["cut_sterile_hist_boost"] = cut_sterile_boost
            selected_t = build_transform_batch(
                self.cfg, selected_t, history, rng,
                board=self.sheet,
                parts=self.parts,
                nest_state=nest_state,
                first_pass=first_pass,
                border_saturation=sat_info.active,
                group_allowed_angles=self.case.group_allowed_angles,
                propose_stats_out=propose_stats,
                void_elite_t=void_elite_t,
                keep_history_on_sterile=keep_hist_sterile,
                part_bases=part_bases_fixed,
                graph_valid_carry=graph_valid_carry,
                archived_patterns=archived_for_propose or None,
            )
            flat_parts = [
                (self.parts[group_idx][0], transforms)
                for group_idx, transforms in enumerate(selected_t)
            ]
            # Keep first-pass-tight clearance under persistent large_void scrap.
            tight_clearance = bool(first_pass) or (
                str(last_void_leak.get("free_kind") or "") == "large_void"
            )
            iter_min_dist = self._min_dist(first_pass=tight_clearance)
            graph, polys, group_id, transform = make_polygon_graph(
                self.case.board,
                flat_parts,
                min_dist=iter_min_dist,
                epsilon_ratio=self.cfg.propose.placement_clearance_epsilon_ratio,
                user_holes=self.user_holes,
                extra_voids=extra_voids,
                propose_stats=propose_stats,
                attract_contact_weight=float(self.cfg.propose.attract_contact_weight),
                attract_kiss_band_scale=float(self.cfg.propose.attract_kiss_band_scale),
                attract_max_degree=int(self.cfg.propose.attract_max_degree),
            )
            pin_keys = set(propose_stats.get("archive_mix_pin_keys") or ())
            if pin_keys:
                graph_keys = {
                    transform_row_key(np.asarray(t, dtype=np.float64))
                    for t in transform
                }
                propose_stats["archive_mix_pin_survive_n"] = int(
                    len(pin_keys & graph_keys)
                )
            graph_valid_carry = inject_cohorts_and_bind_graph(
                mcts_runner.dg,
                graph,
                group_id,
                transform,
                propose_stats,
                self.cfg,
                patterns=archived_for_propose or None,
                agent=mcts_runner.agent,
            )

            graphs = [graph]
            rule_sets = run_improve_rules_rounds(
                improve_rules,
                graphs=graphs,
                rule_sets=rule_sets,
                board=self.case.board,
                sel_iter=sel_iter,
                rng=rng,
                score_options=score_rules_options(sel_iter),
                mutation_presets=self.cfg.rules.mutation_presets(),
                rule_score_penalty=sel_iter.rule_score_penalty,
                max_rules_per_set=self.cfg.rules.max_rules_per_set,
                seed_offset=17 * iter_idx,
            )

            active_rules = active_rule_set(rule_sets)
            scores = list(score_elems(graph, active_rules))
            min_dist = float(iter_min_dist)

            part_areas = [float(p[0].area) for p in self.parts]
            _sheet_c, void_geoms_compose = board_context_from_geometry(
                self.case.board, user_holes=self.user_holes,
            )
            del _sheet_c
            free_prep = schedule_prep_selection_free(
                phase="mid",
                sheet=self.sheet,
                part_areas=part_areas,
                min_dist=min_dist,
                cfg_propose=self.cfg.propose,
                nest_state=nest_state,
                packed_group_id=prev_packed_gid,
                packed_transform=prev_packed_tr,
            )
            assert free_prep is not None
            free_info = free_prep.free_info
            packed_geoms = free_prep.packed_geoms
            packed_group_id = free_prep.packed_group_id
            packed_transform = free_prep.packed_transform
            mean_part = free_prep.mean_part
            sheet_diag = sheet_diag_from(self.sheet)
            part_bases = part_bases_fixed
            candidate_geoms = _native_geoms_from_transforms(
                group_id, transform, part_bases,
            )
            # Q105: dual = last leaf OR large_void (same SoT as build_graph).
            dual_nest = dual_nest_for(free_info, last_leaf=is_last_leaf)
            seed_voids = nest_state_extra_voids(nest_state) or []
            pack_box = RefinePackBox()
            pack_ctx = PackIterCtx(
                graph=graph,
                polys=polys,
                group_id=group_id,
                transform=transform,
                part_areas=part_areas,
                part_bases=part_bases,
                cfg=self.cfg,
                sel=sel,
                propose_stats=propose_stats,
                dg=mcts_runner.dg,
                sheet=self.sheet,
                min_dist=min_dist,
                rule_sets=rule_sets,
                active_rules=active_rules,
                scores=scores,
                free_info=free_info,
                void_geoms=void_geoms_compose,
                packed_geoms=list(packed_geoms),
                packed_group_id=packed_group_id or None,
                packed_transform=packed_transform or None,
                sheet_diag=float(sheet_diag),
                sheet_area=float(self.sheet.area) if self.sheet is not None else 0.0,
                ngroups=len(self.parts),
                is_last_leaf=is_last_leaf,
                near_last=near_last,
                refine_seed=int(rng.integers(1, 2**31)),
                first_pass=first_pass,
                native_geoms_fn=_native_geoms_from_transforms,
                motif_base=mcts_runner.motif_base,
                seed_count=int(nest_state.seed_count or 0) if nest_state else 0,
                seed_void_geoms=list(seed_voids),
            )
            pack_ctx.enable_3b = True
            mid_result, pin_all_blocked_streak = run_mid_pack_stages(
                pack_ctx,
                pack_box,
                graph=graph,
                part_by_group={
                    i: self.parts[i][0] for i in range(len(self.parts))
                },
                void_geoms_compose=void_geoms_compose,
                archived_for_propose=archived_for_propose,
                pin_all_blocked_streak=pin_all_blocked_streak,
                cheap=False,
            )
            composed = pack_box.composed
            assert composed is not None
            propose_stats["nest_dual"] = int(dual_nest)
            polish_budget = mid_result.polish_budget
            scores = composed.scores
            refine_scores = mid_result.refine_scores
            selected = mid_result.selected_nest
            free_poly = mid_result.free_poly
            free_info = mid_result.free_info
            n_void_nest = mid_result.n_void_nest
            selected_polys = mid_result.selected_polys
            polys = mid_result.polys
            transform = mid_result.transform
            group_id = mid_result.group_id
            candidate_geoms = mid_result.candidate_geoms
            pin_stats = mid_result.pin_stats
            propose_stats["mcts_heavy"] = int(polish_budget.mcts_heavy)
            propose_stats["dfs_passes"] = int(polish_budget.dfs_passes)
            propose_stats["motif_sequential_repin"] = 0
            propose_stats.setdefault("block_hole_accepted", 0)
            propose_stats.setdefault("block_hole_emit_in_hull", 0)
            pin_cands = int(pin_stats.get("pin_candidates", 0))
            pin_added = int(pin_stats.get("pin_added", 0))
            proposed_map = propose_stats.get("proposed_by_group") or {}
            proposed_list = (
                [proposed_map[g] for g in sorted(proposed_map)]
                if proposed_map else None
            )
            leak_orch = run_void_leak_and_niche_credit(
                graph=graph,
                polys=polys,
                group_id=group_id,
                transform=transform,
                free_info=free_info,
                free_poly=free_poly,
                selected_nest=selected,
                selected_polys=selected_polys,
                refine_scores=refine_scores,
                propose_stats=propose_stats,
                plateau=plateau,
                pin_stats=pin_stats,
                pin_all_blocked_streak=pin_all_blocked_streak,
                n_void_nest=n_void_nest,
                boost_hits=mid_result.boost_hits,
                void_pole_near_diag_ratio=float(
                    getattr(self.cfg.propose, "void_pole_near_diag_ratio", 0.25) or 0.25
                ),
                proposer_counts={},
                sheet_diag=float(sheet_diag),
                mcts_telem=mcts_telem or propose_stats,
                mcts_runner=mcts_runner,
                mcts_action=mcts_action,
                cfg_propose=self.cfg.propose,
                prev_void_nest=prev_void_nest,
                pin_cands=pin_cands,
                pin_added=pin_added,
                proposed_list=proposed_list,
                stratified_void_elite_quota=int(
                    getattr(self.cfg.propose, "stratified_void_elite_quota", 15)
                ),
                print_funnel=False,
            )
            last_void_leak = leak_orch.leak_dict
            void_elite_by_group = leak_orch.void_elite_by_group
            n_void_graph = leak_orch.n_void_graph
            if leak_orch.had_void_override:
                had_void_override = True
            prev_void_nest = leak_orch.prev_void_nest
            densify = propose_stats.get("densify_stats") or {}
            # Peak inward emit across iters (R0/R1 letter telem).
            ebp_peak_src = dict(
                last_void_leak.get("emitted_by_proposer")
                or densify.get("emitted_by_proposer")
                or {}
            )
            peak = propose_stats.setdefault("inward_peak", dict(inward_peak_state))
            for name in ("raycasting", "voronoi", "erosion", "side_pack"):
                cur = int(ebp_peak_src.get(name, 0) or 0)
                peak[name] = max(int(peak.get(name, 0) or 0), cur)
            inward_peak_state = dict(peak)
            last_void_leak["inward_peak"] = dict(peak)
            last_void_leak["inward_bridge_attempt"] = int(
                densify.get("inward_bridge_attempt", 0) or 0
            )
            if densify.get("inward_ray_keys") is not None:
                last_void_leak["inward_ray_keys"] = int(densify.get("inward_ray_keys") or 0)
                peak["raycasting"] = max(
                    int(peak.get("raycasting", 0) or 0),
                    int(densify.get("inward_ray_keys") or 0),
                )
                last_void_leak["inward_peak"] = dict(peak)
            last_void_leak["inward_ray_raw"] = int(densify.get("inward_ray_raw", 0) or 0)
            last_void_leak["inward_enabled"] = int(densify.get("inward_enabled", 0) or 0)
            last_void_leak["inward_rc_cap"] = int(densify.get("inward_rc_cap", 0) or 0)
            # R2/R3 letter peaks (mix floor / 3b / restore) across iters.
            for key in (
                "cluster_copy_mix_floor_hits",
                "archive_mix_floor_hits",
                "motif_graph_hit_n",
                "motif_cohorts_n",
                "motif_graph_follower_miss_n",
                "archive_mix_pin_survive_n",
                "accepted_patterns_n",
                "motif_override",
                "plateau_props_boost",
                "run_3b",
                "block_hole_accepted",
                "block_hole_tried",
                "block_hole_emit_in_hull",
                "motif_refine_hits",
                "accepted_patterns_archived",
                "refine_rejected",
                "inward_bridge_attempt",
                "repair_mode",
                "repair_patterns_n",
                "repack_motif_accepted",
                "repack_pattern_fallback",
                "repack_accepted",
                "repack_attempted",
                "cluster_copy_emitted",
            ):
                cur = int(
                    propose_stats.get(key, 0)
                    or last_void_leak.get(key, 0)
                    or densify.get(key, 0)
                    or 0
                )
                if key == "accepted_patterns_archived":
                    cur = max(
                        cur,
                        int(propose_stats.get("motif_library_n", 0) or 0),
                        int(propose_stats.get("accepted_patterns_n", 0) or 0),
                    )
                if key == "accepted_patterns_n":
                    cur = max(
                        cur,
                        int(propose_stats.get("accepted_patterns_n", 0) or 0),
                    )
                if key == "cluster_copy_emitted":
                    ebp_cur = densify.get("emitted_by_proposer") or {}
                    cur = max(
                        cur,
                        int(ebp_cur.get("cluster_copy", 0) or 0),
                        int(
                            (last_void_leak.get("emitted_by_proposer") or {}).get(
                                "cluster_copy", 0
                            )
                            or 0
                        ),
                    )
                if key.startswith("repack_"):
                    repack = propose_stats.get("repack") or {}
                    short = key.removeprefix("repack_")
                    cur = max(cur, int(repack.get(short, 0) or 0))
                letter_peak_state[key] = max(int(letter_peak_state.get(key, 0) or 0), cur)
                last_void_leak[key] = int(letter_peak_state[key])
            skip_reason = str(propose_stats.get("archive_mix_skip_reason") or "")
            if skip_reason:
                last_void_leak["archive_mix_skip_reason"] = skip_reason
            proposer_keys = leak_orch.proposer_keys
            emitted_bp = dict(
                propose_stats.get("emitted_by_proposer")
                or densify.get("emitted_by_proposer")
                or {}
            )
            pool_bp = dict(
                propose_stats.get("pool_by_proposer")
                or densify.get("pool_by_proposer")
                or {}
            )
            nest_bp = count_selected_by_proposer(transform, selected, proposer_keys)
            refine_bp = count_selected_by_proposer(
                transform, selected_polys, proposer_keys,
            )
            propose_stats["free_kind"] = str(getattr(free_info, "kind", "") or "")
            packed_area = 0.0
            for si in selected_polys:
                gi = int(group_id[int(si)]) if int(si) < len(group_id) else -1
                if 0 <= gi < len(part_areas):
                    packed_area += float(part_areas[gi])
            sheet_area = float(self.sheet.area) if self.sheet is not None else 0.0
            cov_pct = (100.0 * packed_area / sheet_area) if sheet_area > 0.0 else 0.0
            agent = mcts_runner.agent
            if agent is not None:
                propose_stats["amaf_hits"] = int(agent.telem.get("amaf_hits", 0) or 0)
                propose_stats["amaf_miss"] = int(agent.telem.get("amaf_miss", 0) or 0)
            mcts_box = propose_stats.get("mcts") or {}
            if int(propose_stats.get("amaf_hits", 0) or 0) <= 0:
                propose_stats["amaf_hits"] = int(mcts_box.get("amaf_hits", 0) or 0)
            mcts_runner.niche_archive.age(1)
            last_void_leak.update({
                "motif_sequential_full": int(
                    propose_stats.get("motif_sequential_full", 0) or 0
                ),
                "motif_sequential_repin": int(
                    propose_stats.get("motif_sequential_repin", 0) or 0
                ),
                "motif_sequential_partial": int(
                    propose_stats.get("motif_sequential_partial", 0) or 0
                ),
                "motif_cohorts_n": len(propose_stats.get("motif_cohorts") or []),
                "void_core_accepted": int(
                    propose_stats.get("void_core_accepted", 0) or 0
                ),
                "niche_pos": int(propose_stats.get("niche_pos", 0) or 0),
                "niche_rescue": int(propose_stats.get("niche_rescue", 0) or 0),
                "contact_grg_upserts": int(
                    propose_stats.get("contact_grg_upserts", 0) or 0
                ),
            })
            if int(propose_stats.get("graph_to_nest_hollow", 0) or 0):
                last_void_leak["graph_to_nest_hollow_iters"] = int(
                    last_void_leak.get("graph_to_nest_hollow_iters", 0) or 0
                ) + 1
            nvr = float(propose_stats.get("nest_void_ratio", 1.0) or 1.0)
            last_void_leak["nest_void_ratio_min"] = min(
                float(last_void_leak.get("nest_void_ratio_min", 1.0) or 1.0),
                nvr,
            )
            for dk in (
                "motif_beam_wins",
                "motif_beam_trials",
                "motif_scene_max_sz",
                "motif_pack_max_sz",
                "void_override",
                "incumbent_hold",
                "colonize_pinned",
                "motif_graph_leader_hit_n",
                "hollow_renest",
                "cluster_copy_graph_n",
                "cluster_copy_nest_n",
            ):
                last_void_leak[dk] = max(
                    int(last_void_leak.get(dk, 0) or 0),
                    int(propose_stats.get(dk, 0) or 0),
                )
            prop_n = int(propose_stats.get("proposal_count", 0))
            if prop_n > 0:
                last_proposal_yield = min(1.0, n_void_graph / prop_n) if n_void_graph else (
                    min(1.0, len(polys) / prop_n)
                )

            part_by_group = {
                i: self.parts[i][0] for i in range(len(self.parts))
            }
            post_prep = prepare_post_pack(
                sheet_compact=self.sheet,
                void_geoms_post=void_geoms_compose,
                part_areas=part_areas,
                min_dist=min_dist,
                cfg_propose=self.cfg.propose,
                selected_polys=selected_polys,
                polys=polys,
                group_id=group_id,
                transform=transform,
                part_bases=part_bases,
                native_pack_geoms_fn=_native_geoms_from_transforms,
                free_prep_mid=free_prep,
                free_info=free_info,
                void_leak_stats=last_void_leak if isinstance(last_void_leak, dict) else None,
                propose_stats=propose_stats,
                archived_patterns=archived_for_propose,
            )
            post_pack_ctx = PackIterCtx(
                graph=graph,
                polys=list(polys),
                group_id=list(group_id),
                transform=list(transform),
                part_areas=part_areas,
                part_bases=part_bases,
                cfg=self.cfg,
                sel=sel,
                propose_stats=propose_stats,
                sheet=self.sheet,
                min_dist=min_dist,
                native_geoms_fn=_native_geoms_from_transforms,
            )
            polys, transform, selected_polys, pack_stats = run_post_pack_stage(
                post_pack_ctx,
                post_prep,
                selected_polys=selected_polys,
                part_by_group=part_by_group,
                fixed_obstacles=seed_polys,
                void_leak_stats=last_void_leak if isinstance(last_void_leak, dict) else None,
                polish_budget=polish_budget,
            )
            if post_prep.push_pt is not None and post_prep.free_post.kind == "large_void":
                assert selection_pairwise_independent(polys, selected_polys)
            motif_ttl = int(
                getattr(self.cfg.propose, "accepted_pattern_ttl", 4) or 4
            )
            finalize_iter_mcts(
                mcts_runner,
                selected_polys=selected_polys,
                group_id=group_id,
                transform=transform,
                propose_stats=propose_stats,
                mcts_telem=mcts_telem or propose_stats,
                motif_keys=propose_stats.get("motif_keys") or {},
                motif_ttl=motif_ttl,
                credit_motif=True,
                refine_bp=refine_bp if mcts_runner.agent is not None else None,
                emitted_bp=emitted_bp if mcts_runner.agent is not None else None,
            )
            if mcts_runner.agent is not None:
                motif_ttl_expand = int(
                    getattr(self.cfg.propose, "accepted_pattern_ttl", 4) or 4
                )
                mcts_parent_id, _child_snap = record_outer_iter_expand(
                    mcts_runner,
                    parent_id=mcts_parent_id,
                    action=mcts_action,
                    selected_polys=selected_polys,
                    group_id=group_id,
                    transform=transform,
                    ngroups=max(len(self.parts), 1),
                    coverage_pct=cov_pct,
                    propose_stats=propose_stats,
                    mcts_telem=mcts_telem or propose_stats,
                    nest_state=nest_state,
                    part_bases=part_bases_fixed,
                    min_dist=float(min_dist),
                    motif_min_compactness=float(
                        getattr(self.cfg.propose, "motif_min_compactness", 0.35) or 0.35
                    ),
                    motif_ttl=motif_ttl_expand if len(selected_polys) >= 2 else 0,
                    motif_max_keep=int(
                        getattr(self.cfg.propose, "accepted_pattern_max", 4) or 4
                    ),
                )
                del _child_snap
                if isinstance(last_void_leak, dict):
                    last_void_leak["contact_grg_upserts"] = int(
                        (mcts_telem or propose_stats).get("contact_grg_upserts", 0) or 0
                    )
                    for pk in (
                        "path_extend_n",
                        "path_step_macro",
                        "path_step_join",
                        "macro_path_accept",
                        "macro_path_candidate",
                        "path_contact_upserts",
                        "survive_motif_n",
                        "macro_survive_n",
                        "macro_chain_accept",
                    ):
                        if pk in (mcts_telem or {}):
                            last_void_leak[pk] = int(mcts_telem.get(pk, 0) or 0)
                        elif pk in propose_stats:
                            last_void_leak[pk] = int(propose_stats.get(pk, 0) or 0)
            # Warm cheap-pack cache for next-iter path replay.
            pack_cache.clear()
            pack_cache.update({
                "ready": True,
                "graph": mcts_runner.dg.poses() if mcts_runner.dg is not None else graph,
                "dg": mcts_runner.dg,
                "polys": list(polys),
                "group_id": list(group_id),
                "transform": list(transform),
                "part_areas": part_areas_t,
                "part_bases": part_bases_fixed,
                "p_sheet": self.sheet,
                "min_dist": float(min_dist),
                "cfg": self.cfg,
                "free_info": free_info,
                "propose_stats": dict(propose_stats),
                "packed_geoms": list(packed_geoms),
                "void_geoms": list(void_geoms_compose or []),
                "seed_void_geoms": list(seed_voids),
                "sheet": self.sheet,
                "sheet_area": board_area_f,
                "board_area": board_area_f,
                "compose_sel": list(selected_polys),
                "compose_polys": list(polys),
                "compose_group_id": list(group_id),
                "compose_transform": list(transform),
                "motif_locked": list(propose_stats.get("motif_locked") or ()),
                "selected": list(selected_polys),
                "motif_base": mcts_runner.motif_base,
            })
            _sync_agent_motif_cohorts(mcts_runner, propose_stats)
            pack_cache["motif_cohort_sig"] = int(
                propose_stats.get("motif_cohort_sig", 0) or 0
            )
            if isinstance(last_void_leak, dict):
                last_void_leak["place_cohort_ready"] = int(
                    propose_stats.get("place_cohort_ready", 0) or 0
                )
                last_void_leak["place_cohort_specs_n"] = int(
                    propose_stats.get("place_cohort_specs_n", 0) or 0
                )
                last_void_leak["motif_cohort_sig"] = int(
                    propose_stats.get("motif_cohort_sig", 0) or 0
                )
            # Q185/Q189 archive telem after Motif upsert site (post-expand).
            lib_n = int(mcts_runner.motif_base.size())
            motif_refine_n = 0
            motif_keys_arch = propose_stats.get("motif_keys") or {}
            if selected_polys and group_id is not None and transform is not None:
                for i in selected_polys:
                    gi = int(i)
                    if gi < 0 or gi >= len(group_id) or gi >= len(transform):
                        continue
                    gid = int(group_id[gi])
                    key = transform_row_key(transform[gi])
                    if key in (motif_keys_arch.get(gid) or set()):
                        motif_refine_n += 1
            propose_stats["motif_library_n"] = lib_n
            propose_stats["accepted_patterns_archived"] = lib_n
            propose_stats["motif_refine_hits"] = int(motif_refine_n)
            if isinstance(last_void_leak, dict):
                for key, cur in (
                    ("accepted_patterns_archived", lib_n),
                    ("motif_refine_hits", motif_refine_n),
                    ("cluster_copy_mix_floor_hits", int(
                        propose_stats.get("cluster_copy_mix_floor_hits", 0) or 0
                    )),
                    ("motif_override", int(propose_stats.get("motif_override", 0) or 0)),
                    ("run_3b", int(propose_stats.get("run_3b", 0) or 0)),
                    ("block_hole_accepted", int(
                        propose_stats.get("block_hole_accepted", 0) or 0
                    )),
                    ("block_hole_tried", int(
                        propose_stats.get("block_hole_tried", 0) or 0
                    )),
                    ("block_hole_emit_in_hull", int(
                        propose_stats.get("block_hole_emit_in_hull", 0) or 0
                    )),
                    ("repair_mode", int(propose_stats.get("repair_mode", 0) or 0)),
                    ("repair_patterns_n", int(
                        propose_stats.get("repair_patterns_n", 0) or 0
                    )),
                    ("repack_motif_accepted", int(
                        (propose_stats.get("repack") or {}).get("motif_accepted", 0) or 0
                    )),
                    ("repack_pattern_fallback", int(
                        (propose_stats.get("repack") or {}).get("pattern_fallback", 0) or 0
                    )),
                    ("repack_accepted", int(
                        (propose_stats.get("repack") or {}).get("accepted", 0)
                        or (last_void_leak.get("repack") or {}).get("accepted", 0)
                        or last_void_leak.get("repack_accepted", 0)
                        or 0
                    )),
                    ("repack_attempted", int(
                        (propose_stats.get("repack") or {}).get("attempted", 0)
                        or (last_void_leak.get("repack") or {}).get("attempted", 0)
                        or last_void_leak.get("repack_attempted", 0)
                        or 0
                    )),
                    ("refine_rejected", int(bool(propose_stats.get("refine_rejected", False)))),
                    ("plateau_props_boost", int(
                        propose_stats.get("plateau_props_boost", 0) or 0
                    )),
                    ("inward_bridge_attempt", int(
                        propose_stats.get("inward_bridge_attempt", 0)
                        or (propose_stats.get("densify_stats") or {}).get(
                            "inward_bridge_attempt", 0
                        )
                        or 0
                    )),
                    ("cluster_copy_emitted", int(
                        emitted_bp.get("cluster_copy", 0)
                        or last_void_leak.get("cluster_copy_emitted", 0)
                        or 0
                    )),
                ):
                    letter_peak_state[key] = max(
                        int(letter_peak_state.get(key, 0) or 0), int(cur)
                    )
                    last_void_leak[key] = int(letter_peak_state[key])

            new_selected_t = [[] for _ in range(len(self.case.groups))]
            for i in selected_polys:
                new_selected_t[group_id[i]].append(transform[i])
            selected_t = tuple(
                np.asarray(t, dtype=np.float64) if t else np.zeros((0, 3))
                for t in new_selected_t
            )
            history = selected_t

            # Obstacles for the next propose pass: locked seeds + this iter's selection only.
            # group_id from make_polygon_graph is the parts-list index; map to NestCase gid.
            placed_new = [polys[i] for i in selected_polys]
            placed_gids = [int(self.case.groups[group_id[i]][1]) for i in selected_polys]
            placed_tr = [
                np.asarray(transform[i], dtype=np.float64) for i in selected_polys
            ]
            prev_packed_gid = [int(group_id[i]) for i in selected_polys]
            prev_packed_tr = list(placed_tr)
            next_polys = list(seed_polys) + placed_new
            next_gids = list(seed_gids) + placed_gids
            next_tr = list(seed_tr) + placed_tr
            next_sel = list(range(len(next_polys)))
            n_seed = len(seed_polys)

            nest_state = NestState(
                polys=next_polys,
                group_id=next_gids,
                transform=next_tr,
                selected_indices=next_sel,
                seed_count=n_seed,
            )
            # Per-iter coverage sample for trajectory metrics.
            t_elapsed = time.perf_counter() - t0
            usable = self.case.usable_area
            cov = 0.0
            if usable > 0:
                part_area = sum(p.area for p in seed_polys) + sum(
                    self.parts[group_id[i]][0].area for i in selected_polys
                )
                cov = part_area / usable
            trajectory.append((t_elapsed, cov, len(seed_polys) + len(selected_polys)))
            if usable > 0 and next_polys:
                poly_cov = sum(float(p.area) for p in next_polys) / usable
                # Peak track with gate Shapely pairwise (Q371); packing-clear SoT
                # remains for post_pack / path. Avoid C++ false-pen rejects starving peaks.
                if poly_cov > best_cov + 1e-9 and _shapes_pairwise_ok(next_polys):
                    best_cov = float(poly_cov)
                    best_next_polys = list(next_polys)
                    best_next_gids = list(next_gids)
                    best_next_tr = list(next_tr)
                    best_next_sel = list(next_sel)
                    best_selected_polys = list(selected_polys)
                    best_pack_sig = best_pack_geom_sig(
                        polys if polys else next_polys, selected_polys,
                    )
                    if isinstance(last_void_leak, dict):
                        last_void_leak["best_pack_peak_cov"] = float(poly_cov)
            plateau.update(cov * 100.0, len(seed_polys) + len(selected_polys))
            if isinstance(last_void_leak, dict):
                last_void_leak["on_plateau"] = bool(plateau.on_plateau)
                last_void_leak["plateau_streak"] = int(plateau.streak)

        time_s = time.perf_counter() - t0
        
        # Metrics calculation — use final compacted nest pack when available.
        initial_seed_n = len(_build_seed_state(self.case)[0])
        parts_seed = initial_seed_n
        area_coverage_seed = 0.0
        if self.case.usable_area > 0 and parts_seed > 0:
            area_coverage_seed = (
                sum(p.area for p, _g, _t in self.case.seed_placements)
                / self.case.usable_area
            )

        parts_final = len(next_polys) if next_polys else (len(seed_polys) + len(selected_polys))
        usable_area = float(self.case.usable_area)
        # Prefer peak nest polys over last-iter graph indices (wrong graph SoT).
        if (
            best_next_polys is None
            and best_selected_polys is not None
            and best_cov > 0.0
            and usable_area > 0
        ):
            restored_sel, did_restore, bp_telem, _restored_tf = maybe_restore_best_pack(
                best=BestPackSnapshot(
                    selected_polys=list(best_selected_polys),
                    cov=float(best_cov),
                    geom_sig=float(best_pack_sig),
                ),
                current_selected=list(selected_polys),
                graph=graph,
                polys=polys if polys else next_polys,
                group_id=group_id if group_id is not None else next_gids,
                part_areas=[float(p[0].area) for p in self.parts],
                sheet=self.sheet,
                holes=self.case.board_holes,
                usable_area=usable_area,
            )
            if isinstance(last_void_leak, dict):
                last_void_leak.update(bp_telem)
            if did_restore:
                selected_polys = list(restored_sel)
                if isinstance(last_void_leak, dict):
                    last_void_leak["best_cov"] = float(best_cov)
        graph_nodes = len(polys)
        min_dist = self._min_dist()
        independent_ok = (
            selection_is_independent(graph, selected_polys) if graph is not None else False
        )
        # Geometric overlap: Shapely area tol matches historical gate floors;
        # packing-clear remains post_pack/path SoT (Letter E). Restore prefers
        # peak when final fails packing-clear or regresses ≥0.5pp.
        if next_polys:
            placed_shapes = list(next_polys)
        else:
            placed_shapes = list(seed_polys) + [
                transform_poly(self.parts[group_id[i]][0], transform[i])
                for i in selected_polys
            ]
        overlap_ok = bool(_shapes_pairwise_ok(placed_shapes))
        packing_clear_ok = bool(
            post_pack_overlap_ok(placed_shapes, range(len(placed_shapes)))
        )
        void_ok = True
        for a, pa in enumerate(placed_shapes):
            if pa is None or pa.is_empty:
                continue
            if not self.sheet.buffer(1e-5).covers(pa):
                void_ok = False
            for hole in self.case.board_holes:
                inter = pa.intersection(hole)
                if not inter.is_empty and inter.area > 1e-6:
                    void_ok = False

        area_coverage = 0.0
        if usable_area > 0 and parts_final > 0:
            part_area = sum(float(p.area) for p in placed_shapes)
            area_coverage = part_area / usable_area
        # Q371 / Letter B: restore peak when regress or packing-clear fails.
        if best_next_polys is not None and usable_area > 0:
            peak_area = sum(float(p.area) for p in best_next_polys) / usable_area
            final_before = float(area_coverage)
            gate_ok = _shapes_pairwise_ok(best_next_polys)
            want_restore = gate_ok and (
                peak_area > final_before + 0.005
                or (not packing_clear_ok and peak_area + 1e-9 >= final_before)
            )
            if want_restore:
                # Re-validate void/board before accepting restore.
                void_peak = True
                for pa in best_next_polys:
                    if pa is None or getattr(pa, "is_empty", False):
                        continue
                    if not self.sheet.buffer(1e-5).covers(pa):
                        void_peak = False
                        break
                    for hole in self.case.board_holes:
                        inter = pa.intersection(hole)
                        if not inter.is_empty and inter.area > 1e-6:
                            void_peak = False
                            break
                    if not void_peak:
                        break
                if void_peak:
                    placed_shapes = list(best_next_polys)
                    next_polys = list(best_next_polys)
                    next_gids = list(best_next_gids or next_gids)
                    next_tr = list(best_next_tr or next_tr)
                    next_sel = list(best_next_sel or next_sel)
                    parts_final = len(next_polys)
                    area_coverage = float(peak_area)
                    independent_ok = True
                    overlap_ok = True
                    void_ok = True
                    if isinstance(last_void_leak, dict):
                        last_void_leak["best_pack_restore"] = 1
                        last_void_leak["best_cov"] = float(peak_area)
                        last_void_leak["best_pack_peak_cov"] = float(peak_area)
                        last_void_leak["best_pack_final_cov"] = float(final_before)
                elif isinstance(last_void_leak, dict):
                    last_void_leak["best_pack_reject"] = 1
                    record_overlap_reject(last_void_leak, stage="best_pack")
            elif isinstance(last_void_leak, dict) and (
                peak_area > final_before + 0.005 or not packing_clear_ok
            ):
                last_void_leak["best_pack_reject"] = 0 if gate_ok else 1
                if not gate_ok:
                    record_overlap_reject(last_void_leak, stage="best_pack")
                last_void_leak["best_pack_peak_cov"] = float(peak_area)
                last_void_leak["best_pack_final_cov"] = float(final_before)

        out_cov = outline_coverage_ratio(
            placed_shapes,
            self.sheet,
            min_dist=min_dist,
            pack_geoms=(
                nest_state.native_geoms
                if nest_state is not None and next_polys
                else None
            ),
        )

        mean_part_area = float(np.mean([p[0].area for p in self.parts])) if self.parts else 1.0
        metrics_prep = schedule_prep_selection_free(
            phase="mid",
            sheet=self.sheet,
            part_areas=[mean_part_area],
            min_dist=min_dist,
            cfg_propose=self.cfg.propose,
            packed_shapely=placed_shapes,
            pack_geoms=(
                list(nest_state.native_geoms)
                if nest_state is not None and next_polys
                else None
            ),
        )
        assert metrics_prep is not None
        free_info = metrics_prep.free_info
        largest_free_comp_area = float(free_info.largest_area)
        largest_free_over_part = float(free_info.max_void_ratio)

        pair_gaps: list[float] = []
        for a in range(len(placed_shapes)):
            pa = placed_shapes[a]
            if pa is None or pa.is_empty:
                continue
            nearest = None
            for b in range(len(placed_shapes)):
                if a == b:
                    continue
                pb = placed_shapes[b]
                if pb is None or pb.is_empty:
                    continue
                d = float(pa.distance(pb))
                if nearest is None or d < nearest:
                    nearest = d
            if nearest is not None:
                pair_gaps.append(nearest)
        clearance_p50 = float(np.median(pair_gaps)) if pair_gaps else 0.0
        
        # Kiss metrics
        kiss_seed = 0.0
        kiss_outline = 0.0
        kiss_standoff = 0.0
        contact_min = -1.0
        new_geoms_shp = placed_shapes[initial_seed_n:]
        if new_geoms_shp:
            kiss_tol = max(min_dist * 2.0, 0.15)
            standoff_tol = max(min_dist * 0.5, 1e-3)
            outline_hits = 0
            standoff_hits = 0
            obstacle_union = None
            if seed_polys or selected_polys:
                # Obstacle for standoff: seeds + sheet exterior as distance target
                # Standoff = distance to nearest packed seed or other new parts? Plan says
                # obstacle union of seeds (locked) for new parts.
                if seed_polys:
                    obstacle_union = unary_union(seed_polys)
            for p in new_geoms_shp:
                if float(p.distance(self.sheet.exterior)) <= kiss_tol:
                    outline_hits += 1
                if obstacle_union is not None and not obstacle_union.is_empty:
                    err = abs(float(p.distance(obstacle_union)) - min_dist)
                    if err <= standoff_tol:
                        standoff_hits += 1
            kiss_outline = outline_hits / len(new_geoms_shp)
            if obstacle_union is not None:
                kiss_standoff = standoff_hits / len(new_geoms_shp)

        if seed_polys and selected_polys:
            seed_geoms = [Geometry.from_shapely(p) for p in seed_polys]
            new_geoms = [Geometry.from_shapely(p) for p in new_geoms_shp]
            dists = find_polygon_distances_bipartite(new_geoms, seed_geoms)
            if dists:
                nearest: dict[int, float] = {}
                for d in dists:
                    i = int(d.polyA_idx)
                    j_dist = float(d.distance)
                    prev = nearest.get(i)
                    if prev is None or j_dist < prev:
                        nearest[i] = j_dist
                dist_vals = list(nearest.values())
                contact_min = min(dist_vals) if dist_vals else -1.0
                kiss_tol = max(min_dist * 2.0, 0.15)
                kiss_seed = (
                    sum(1 for v in dist_vals if v <= kiss_tol) / len(new_geoms)
                    if new_geoms else 0.0
                )

        # Trajectory: time to 90% of final coverage; AUC normalized efficiency.
        time_to_frac = -1.0
        density_auc = 0.0
        if trajectory:
            c_final = trajectory[-1][1]
            t_total = max(trajectory[-1][0], 1e-12)
            target = 0.9 * c_final
            for t_i, c_i, _p in trajectory:
                if c_i >= target - 1e-12:
                    time_to_frac = t_i
                    break
            # Trapezoid ∫ c dt / (c_final * t_total)
            area = 0.0
            for i in range(1, len(trajectory)):
                t0_, c0, _ = trajectory[i - 1]
                t1_, c1, _ = trajectory[i]
                area += 0.5 * (c0 + c1) * (t1_ - t0_)
            if c_final > 1e-12:
                density_auc = float(np.clip(area / (c_final * t_total), 0.0, 1.0))

        if isinstance(last_void_leak, dict):
            last_void_leak["diag"] = build_run_diagnostics(
                leak=last_void_leak,
                trajectory=trajectory,
                area_coverage=area_coverage,
                area_coverage_seed=area_coverage_seed,
            )

        self.last_result = {
            "selected_polys": next_sel,
            "polys": next_polys,
            "group_id": next_gids,
            "transform": np.asarray(next_tr) if next_tr else np.zeros((0, 3)),
            "graph": graph,
            "attract_edges": int(last_void_leak.get("attract_edges", 0) or 0),
            "void_leak": dict(last_void_leak),
        }

        return NestingMetrics(
            parts_final=parts_final,
            area_coverage=area_coverage,
            outline_coverage=out_cov,
            independent_ok=independent_ok,
            overlap_ok=overlap_ok,
            void_ok=void_ok,
            graph_nodes=graph_nodes,
            proposal_yield=float(last_proposal_yield),
            time_s=time_s,
            parts_seed=parts_seed,
            area_coverage_seed=area_coverage_seed,
            parts_delta=parts_final - parts_seed,
            area_coverage_delta=area_coverage - area_coverage_seed,
            kiss_fraction=kiss_seed,
            kiss_seed=kiss_seed,
            kiss_outline=kiss_outline,
            kiss_standoff=kiss_standoff,
            contact_min=contact_min,
            clearance_p50=clearance_p50,
            largest_free_comp_area=largest_free_comp_area,
            largest_free_over_part=largest_free_over_part,
            void_props=int(last_void_leak.get("props", 0)),
            void_graph=int(last_void_leak.get("graph", 0)),
            void_selected_nest=int(last_void_leak.get("nest", 0)),
            void_selected_refine=int(last_void_leak.get("refine", 0)),
            free_kind=str(last_void_leak.get("free_kind", "")),
            time_to_frac_final=time_to_frac,
            density_auc=density_auc,
            coverage_trajectory=tuple(trajectory),
        )
