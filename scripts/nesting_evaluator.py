"""Unified pipeline evaluator for nesting quality benchmarks."""

import time
from dataclasses import dataclass

import numpy as np
from shapely.geometry import Point, Polygon
from shapely.geometry.base import BaseGeometry

from nest_graph.board import board_context_from_geometry
from nest_graph.build_graph import (
    NestState,
    _build_transform_batch,
    _boost_border_scores,
    _count_graph_in_free,
    _count_props_in_free,
    _count_props_near_pole,
    _count_selected_by_proposer,
    _count_selected_in_free,
    _format_prop_accept,
    _late_border_saturation_info,
    _make_initial_rule_sets,
    _make_void_attractor_rule_set,
    _merge_void_attractor_into_rule_sets,
    _nest_seed_from_boosted_scores,
    _pin_nest_void_independent,
    _void_attractor_radius,
    _zones_have_void_hijack,
    active_rule_set,
    apply_dfs_refinement,
    apply_void_selection_boosts,
    improve_rules,
    make_polygon_graph,
)
from nest_graph.config import BuildGraphConfig, score_rules_options
from nest_graph.elem_graph import nest_by_graph, score_elems, selection_is_independent
from nest_graph.placement_scene import placement_clearance_epsilon
from nest_graph.propose import (
    ProposeGeometry,
    border_focal_for_propose,
    collect_propose_candidates,
    effective_ranking_mode,
    obstacle_shape_for_propose,
)
from nest_graph.propose.compaction import compact_selection, selection_pairwise_independent
from nest_graph.propose.cluster_repack import (
    cluster_repack_selection,
    cluster_relocate_selection,
)
from nest_graph.propose.local_se2 import local_se2_selection
from nest_graph.propose.context import (
    analyze_free_space,
    build_free_space_snapshot,
    outline_coverage_ratio,
    part_is_concave,
    propose_push_point,
    should_use_border_focus,
)
from nest_graph.propose.pipeline import propose_coords_from_candidates
from nest_graph.geometry import Geometry, find_polygon_distances_bipartite
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
    def __init__(self, case: NestCase, cfg: BuildGraphConfig):
        self.case = case
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

        selected_t = _build_transform_batch(
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
        trajectory: list[tuple[float, float, int]] = []
        last_void_leak: dict = {
            "free_kind": "",
            "props": 0,
            "graph": 0,
            "nest": 0,
            "refine": 0,
        }
        last_proposal_yield = 0.0
        had_void_override = False

        for iter_idx in range(self.case.iters):
            first_pass = nest_state is None
            sat_info = _late_border_saturation_info(
                self.cfg, nest_state, self.sheet,
                had_void_override=had_void_override,
            )
            if sat_info.sat_override:
                had_void_override = True
            propose_stats: dict = {
                "outline_cov": sat_info.outline_cov,
                "sat_override": sat_info.sat_override,
                "rim_progress": sat_info.rim_progress,
            }
            selected_t = _build_transform_batch(
                self.cfg, selected_t, history, rng,
                board=self.sheet,
                parts=self.parts,
                nest_state=nest_state,
                first_pass=first_pass,
                border_saturation=sat_info.active,
                group_allowed_angles=self.case.group_allowed_angles,
                propose_stats_out=propose_stats,
            )
            flat_parts = [
                (self.parts[group_idx][0], transforms)
                for group_idx, transforms in enumerate(selected_t)
            ]
            graph, polys, group_id, transform = make_polygon_graph(
                self.case.board,
                flat_parts,
                min_dist=self._min_dist(first_pass=first_pass),
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
                    seed=int(rng.integers(0, 2**31)) + round_idx + 17 * iter_idx,
                    score_options=score_rules_options(sel),
                    max_rules_per_set=self.cfg.rules.max_rules_per_set,
                )

            active_rules = active_rule_set(rule_sets)
            scores = list(score_elems(graph, active_rules))
            min_dist = self._min_dist(first_pass=False)

            packed_for_free = list(next_polys) if next_polys else []
            part_areas = [float(p[0].area) for p in self.parts]
            mean_part = float(np.mean(part_areas)) if part_areas else 1.0
            void_thr = float(self.cfg.propose.late_border_void_override_ratio)
            if void_thr <= 0.0:
                void_thr = 2.5
            free_info = analyze_free_space(
                self.sheet, packed_for_free, mean_part, min_dist,
                void_ratio_threshold=void_thr,
            )
            free_poly = free_info.target_poly
            nest_rules = rule_sets
            refine_rules = active_rules
            sheet_diag = 0.0
            if self.sheet is not None and not self.sheet.is_empty:
                minx, miny, maxx, maxy = self.sheet.bounds
                sheet_diag = float(np.hypot(maxx - minx, maxy - miny))
            attr_w = float(self.cfg.propose.void_attractor_rule_weight)
            pole_w = float(self.cfg.propose.void_island_score_boost)
            pocket_w = float(self.cfg.propose.pocket_score_boost)
            small_w = float(self.cfg.propose.small_part_void_score_boost)
            void_r = _void_attractor_radius(
                min_dist, sheet_diag, self.cfg.rules.place_rule_radius,
            )
            if (
                free_info.kind == "large_void"
                and free_info.target_pt is not None
                and attr_w > 0.0
            ):
                attractor = _make_void_attractor_rule_set(
                    free_info.target_pt,
                    ngroups=len(self.parts),
                    radius=void_r,
                    weight=attr_w,
                )
                nest_rules = _merge_void_attractor_into_rule_sets(
                    rule_sets,
                    attractor,
                    nest_rule_sets_used=sel.nest_rule_sets_used,
                    max_rules_per_set=self.cfg.rules.max_rules_per_set,
                )
                refine_rules = active_rule_set(nest_rules)
                scores = list(score_elems(graph, refine_rules))
            apply_void_selection_boosts(
                polys=polys,
                group_id=group_id,
                transform=transform,
                scores=scores,
                free_info=free_info,
                free_poly=free_poly,
                part_areas=part_areas,
                propose_stats=propose_stats,
                cfg=self.cfg,
                sheet_diag=sheet_diag,
                void_r=void_r,
            )

            use_greedy_nest = (
                bool(self.cfg.propose.void_greedy_nest_seed)
                and free_info.kind == "large_void"
                and scores
                and (pole_w > 0.0 or pocket_w > 0.0 or small_w > 0.0)
            )
            if use_greedy_nest:
                selected = _nest_seed_from_boosted_scores(graph, scores)
            else:
                selected = list(
                    nest_by_graph(graph, nest_rules[: sel.nest_rule_sets_used])[0]
                )
            n_void_nest = _count_selected_in_free(polys, selected, free_poly)
            # Match demo: nest uses void-boosted scores; refine gets a copy so
            # first-pass border boost cannot tilt DFS against void islands.
            refine_scores = list(scores)
            if (
                first_pass
                and should_use_border_focus(Polygon(), self.cfg.propose)
                and free_info.kind != "large_void"
            ):
                _boost_border_scores(
                    polys, refine_scores, self.sheet, min_dist,
                    weight=self.cfg.propose.border_selection_score_boost,
                )
            _, selected_polys, _score_sum = apply_dfs_refinement(
                graph,
                refine_rules,
                selected,
                refine_scores,
                selection=sel,
            )
            pin_stats: dict = {
                "pin_candidates": 0,
                "pin_added": 0,
                "pin_blocked_collision": 0,
                "pin_ms": 0.0,
            }
            if (
                bool(getattr(self.cfg.propose, "enable_void_nest_pin", True))
                and free_poly is not None
                and not free_poly.is_empty
            ):
                selected_polys = _pin_nest_void_independent(
                    graph,
                    selected,
                    selected_polys,
                    polys,
                    free_poly,
                    refine_scores,
                    stats_out=pin_stats,
                )
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
            densify = propose_stats.get("densify_stats") or {}
            proposer_keys = propose_stats.get("proposer_keys") or densify.get("proposer_keys") or {}
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
            nest_bp = _count_selected_by_proposer(transform, selected, proposer_keys)
            refine_bp = _count_selected_by_proposer(
                transform, selected_polys, proposer_keys,
            )
            prop_accept = _format_prop_accept(emitted_bp, pool_bp, nest_bp, refine_bp)
            last_void_leak = {
                "free_kind": free_info.kind,
                "max_void_ratio": float(free_info.max_void_ratio),
                "props": n_void_props,
                "props_pole": n_props_pole,
                "hijack": _zones_have_void_hijack(zones),
                "graph": n_void_graph,
                "nest": n_void_nest,
                "refine": n_void_refine,
                "border_only": bool(propose_stats.get("border_only", False)),
                "outline_cov": float(propose_stats.get("outline_cov", 0.0)),
                "sat_override": bool(propose_stats.get("sat_override", False)),
                "rim_progress": float(propose_stats.get("rim_progress", 0.0)),
                "emitted_by_proposer": emitted_bp,
                "pool_by_proposer": pool_bp,
                "nest_by_proposer": nest_bp,
                "refine_by_proposer": refine_bp,
                "pocket_by_tag": dict(
                    propose_stats.get("pocket_by_tag")
                    or densify.get("pocket_by_tag")
                    or {}
                ),
                "prop_accept": prop_accept,
                "pin_candidates": int(pin_stats.get("pin_candidates", 0)),
                "pin_added": int(pin_stats.get("pin_added", 0)),
                "pin_blocked_collision": int(pin_stats.get("pin_blocked_collision", 0)),
                "pin_ms": float(pin_stats.get("pin_ms", 0.0)),
            }
            prop_n = int(propose_stats.get("proposal_count", 0))
            if prop_n > 0:
                last_proposal_yield = min(1.0, n_void_graph / prop_n) if n_void_graph else (
                    min(1.0, len(polys) / prop_n)
                )

            if (
                self.cfg.propose.enable_gravity_compaction
                and len(selected_polys) >= 2
            ):
                part_by_group = {
                    i: self.parts[i][0] for i in range(len(self.parts))
                }
                polys, transform = compact_selection(
                    self.sheet,
                    list(polys),
                    list(transform),
                    group_id,
                    selected_polys,
                    part_by_group,
                    min_dist,
                    fixed_obstacles=seed_polys,
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

            part_by_group = {
                i: self.parts[i][0] for i in range(len(self.parts))
            }
            sel_geoms = [
                polys[i] for i in selected_polys
                if polys[i] is not None and not polys[i].is_empty
            ]
            free_snap = build_free_space_snapshot(
                self.sheet, sel_geoms, mean_part, min_dist,
                void_ratio_threshold=void_thr,
            )
            free_post = free_snap.analysis
            allow_repack = True
            if isinstance(last_void_leak, dict):
                allow_repack = int(last_void_leak.get("refine", 0)) > 0
            if free_post.kind == "large_void" and free_post.target_pt is not None:
                if allow_repack:
                    polys, transform, selected_polys, _repack = cluster_repack_selection(
                        self.sheet,
                        list(polys),
                        list(transform),
                        group_id,
                        selected_polys,
                        part_by_group,
                        min_dist,
                        self.cfg.propose,
                        pole=free_post.target_pt,
                        void_poly=free_post.target_poly,
                        fixed_obstacles=seed_polys,
                        pt_push=free_post.target_pt,
                        free_space=free_snap,
                    )
                else:
                    _repack = {
                        "attempted": 0,
                        "accepted": 0,
                        "motif_accepted": 0,
                        "skipped_refine_zero": 1,
                    }
                reloc_pole = free_post.target_pt
                if int(_repack.get("accepted", 0)) > 0:
                    sel_geoms = [
                        polys[i] for i in selected_polys
                        if polys[i] is not None and not polys[i].is_empty
                    ]
                    free_snap = build_free_space_snapshot(
                        self.sheet, sel_geoms, mean_part, min_dist,
                        void_ratio_threshold=void_thr,
                    )
                    if free_snap.analysis.target_pt is not None:
                        reloc_pole = free_snap.analysis.target_pt
                polys, transform, _reloc = cluster_relocate_selection(
                    self.sheet,
                    list(polys),
                    list(transform),
                    group_id,
                    selected_polys,
                    part_by_group,
                    min_dist,
                    self.cfg.propose,
                    pole=reloc_pole,
                    fixed_obstacles=seed_polys,
                )
                polys, transform, _se2 = local_se2_selection(
                    self.sheet,
                    list(polys),
                    list(transform),
                    group_id,
                    selected_polys,
                    part_by_group,
                    min_dist,
                    self.cfg.propose,
                    pole=reloc_pole,
                    fixed_obstacles=seed_polys,
                )
                assert selection_pairwise_independent(polys, selected_polys)
                if isinstance(last_void_leak, dict):
                    last_void_leak["repack"] = _repack
                    last_void_leak["relocate"] = _reloc
                    last_void_leak["local_se2"] = _se2

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
            next_polys = list(seed_polys) + placed_new
            next_gids = list(seed_gids) + placed_gids
            next_tr = list(seed_tr) + placed_tr
            next_sel = list(range(len(next_polys)))

            # Densify the full pack (incl. former seeds) so Mode B closes gaps.
            n_seed = len(seed_polys)
            if (
                self.cfg.propose.enable_gravity_compaction
                and len(next_sel) >= 2
            ):
                gid_to_part = {
                    int(gid): poly for poly, gid in self.case.groups
                }
                next_polys, next_tr = compact_selection(
                    self.sheet,
                    next_polys,
                    next_tr,
                    next_gids,
                    next_sel,
                    gid_to_part,
                    min_dist,
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
                if n_seed > 0:
                    seed_polys = list(next_polys[:n_seed])
                    seed_tr = list(next_tr[:n_seed])
                    seed_gids = list(next_gids[:n_seed])
                    extra_voids = _seed_extra_voids(seed_polys)

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
        graph_nodes = len(polys)
        min_dist = self._min_dist()
        independent_ok = (
            selection_is_independent(graph, selected_polys) if graph is not None else False
        )
        # Geometric overlap: hard intersections only. Graph independence already
        # encodes clearance edges; post-compact poses may sit slightly under
        # numeric min_dist without being graph-adjacent.
        if next_polys:
            placed_shapes = list(next_polys)
        else:
            placed_shapes = list(seed_polys) + [
                transform_poly(self.parts[group_id[i]][0], transform[i])
                for i in selected_polys
            ]
        overlap_ok = True
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
            for b in range(a + 1, len(placed_shapes)):
                pb = placed_shapes[b]
                if pb is None or pb.is_empty:
                    continue
                if pa.intersects(pb) and pa.intersection(pb).area > 1e-12:
                    overlap_ok = False

        usable_area = self.case.usable_area
        area_coverage = 0.0
        if usable_area > 0 and parts_final > 0:
            part_area = sum(float(p.area) for p in placed_shapes)
            area_coverage = part_area / usable_area

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
        free_info = analyze_free_space(
            self.sheet, placed_shapes, mean_part_area, min_dist,
        )
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
                from shapely.ops import unary_union
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

        self.last_result = {
            "selected_polys": next_sel,
            "polys": next_polys,
            "group_id": next_gids,
            "transform": np.asarray(next_tr) if next_tr else np.zeros((0, 3)),
            "graph": graph,
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
