"""Shared fixtures and metrics for propose benchmarks."""

from dataclasses import dataclass, field
from typing import Any

import numpy as np
from shapely.geometry import Point, Polygon
from shapely.geometry.base import BaseGeometry

from nest_graph.board import board_context_from_geometry
from nest_graph.config import BuildGraphConfig, ProposeConfig
from nest_graph.placement_scene import placement_clearance_epsilon
from nest_graph.propose import (
    ProposeGeometry,
    ProposerName,
    base_shape_from_selection,
    border_focal_for_propose,
    collect_propose_candidates,
    effective_ranking_mode,
    obstacle_shape_for_propose,
    propose_push_point,
    should_use_border_focus,
)
from nest_graph.propose.pipeline import propose_coords_from_candidates
from nest_graph.utils import normalize_poly, transform_poly


def shipped_propose_config(**overrides: Any) -> ProposeConfig:
    """Mirror production ProposeConfig defaults with optional overrides."""
    cfg = ProposeConfig()
    for key, val in overrides.items():
        setattr(cfg, key, val)
    return cfg


def ablation_propose_config(
    enabled_name: str,
    *,
    pool: int = 48,
) -> ProposeConfig:
    """Single-proposer isolation: all generators off except the named one."""
    off = {
        "use_neighbor_slide": False,
        "use_axis_push": False,
        "use_bottom_left": False,
        "use_nfp_vertices": False,
        "use_voronoi": False,
        "use_point_cloud": False,
        "use_guidance_walk": False,
        "use_ribbon_seeds": False,
        "use_group_edge_seeds": False,
        "use_border_edge_seeds": False,
        "use_guidance_propositions": False,
    }
    on: dict[str, Any] = {
        "candidate_pool": pool,
        "max_proposals": 24,
        "use_free_region_search": True,
        "trim_candidates_by_clearance": True,
        "use_contact_ranking": True,
        "use_contact_clearance_hybrid": True,
    }
    if enabled_name in (ProposerName.GUIDANCE_CAST_REFINE.value,):
        on["use_guidance_propositions"] = True
        on["guidance_use_tight_packing"] = True
        on["guidance_cast_refine_top_k"] = 16
    elif enabled_name in ("sheet_corners", "sheet_edge"):
        on["use_border_edge_seeds"] = True
        on["use_border_focus"] = True
    elif enabled_name == "group_fit":
        on["use_group_edge_seeds"] = True
    elif enabled_name == "ribbon_free":
        on["use_ribbon_seeds"] = True
    elif enabled_name == "voronoi":
        on["use_voronoi"] = True
    elif enabled_name == "point_cloud":
        on["use_point_cloud"] = True
    elif enabled_name == "guidance_walk":
        on["use_guidance_walk"] = True
    elif enabled_name == "neighbor_slide":
        on["use_neighbor_slide"] = True
    elif enabled_name == "axis_push":
        on["use_axis_push"] = True
    elif enabled_name == "bottom_left":
        on["use_bottom_left"] = True
    elif enabled_name == "nfp_vertices":
        on["use_nfp_vertices"] = True
    return shipped_propose_config(**{**off, **on})


GUIDANCE_ABLATION_SEEDS: dict[str, list[tuple[float, float, float]]] = {
    "empty_base": [(0.5, 0.5, 0.0), (0.4, 0.6, 0.3)],
    "partial_pack": [(0.55, 0.45, 0.0), (0.65, 0.5, 0.4)],
    "two_clusters": [(0.35, 0.35, 0.0), (0.75, 0.75, 0.2)],
    "hole_board": [(0.5, 0.15, 0.0), (0.6, 0.2, 0.5)],
}


def _triangle_board() -> Polygon:
    return Polygon([(0, 0), (1.2, 0), (0, 1.1)])


def _hole_board() -> Polygon:
    return Polygon([(0, 0), (1.2, 0), (1.2, 1.1), (0, 1.1)])


def _rect_poly() -> Polygon:
    return normalize_poly(Polygon([(0, 0), (0.1, 0), (0.1, 0.1), (0, 0.1)]))


def _tri_poly() -> Polygon:
    return normalize_poly(Polygon([(0, 0), (0.15, 0), (0, 0.07)]))


def scenario_context(
    scenario: str,
    seed: int,
) -> tuple[Polygon, Polygon, Polygon, list, BaseGeometry]:
    board = _triangle_board()
    rect = _rect_poly()
    tri = _tri_poly()
    rng = np.random.default_rng(seed)

    if scenario == "empty_base":
        return board, tri, rect, ([], []), Polygon()

    if scenario == "partial_pack":
        t_rect = np.array([0.35, 0.25, float(rng.uniform(0, 0.5))])
        placed = transform_poly(rect, t_rect)
        polys, indices = [placed], [0]
        base = base_shape_from_selection(polys, indices)
        return board, tri, rect, (polys, indices), base

    if scenario == "two_clusters":
        t1 = np.array([0.08, 0.08, float(rng.uniform(0, 0.3))])
        t2 = np.array([0.55, 0.55, float(rng.uniform(0, 0.3))])
        polys = [transform_poly(rect, t1), transform_poly(rect, t2)]
        indices = [0, 1]
        base = base_shape_from_selection(polys, indices)
        return board, tri, rect, (polys, indices), base

    if scenario == "hole_board":
        board = _hole_board()
        t_rect = np.array([0.35, 0.08, float(rng.uniform(0, 0.5))])
        placed = transform_poly(rect, t_rect)
        polys, indices = [placed], [0]
        base = base_shape_from_selection(polys, indices)
        return board, tri, rect, (polys, indices), base

    if scenario == "packed_border":
        t1 = np.array([0.06, 0.06, float(rng.uniform(0, 0.4))])
        t2 = np.array([0.95, 0.06, float(rng.uniform(0, 0.4))])
        t3 = np.array([0.06, 0.85, float(rng.uniform(0, 0.4))])
        polys = [
            transform_poly(rect, t1),
            transform_poly(rect, t2),
            transform_poly(tri, t3),
        ]
        indices = [0, 1, 2]
        base = base_shape_from_selection(polys, indices)
        return board, tri, rect, (polys, indices), base

    if scenario == "border_gap":
        t_rect = np.array([0.08, 0.08, float(rng.uniform(0, 0.4))])
        placed = transform_poly(rect, t_rect)
        polys, indices = [placed], [0]
        base = base_shape_from_selection(polys, indices)
        return board, tri, rect, (polys, indices), base

    if scenario == "interior_after_border":
        t1 = np.array([0.06, 0.06, float(rng.uniform(0, 0.3))])
        t2 = np.array([0.95, 0.06, float(rng.uniform(0, 0.3))])
        t3 = np.array([0.06, 0.85, float(rng.uniform(0, 0.3))])
        polys = [
            transform_poly(rect, t1),
            transform_poly(rect, t2),
            transform_poly(tri, t3),
        ]
        indices = [0, 1, 2]
        base = base_shape_from_selection(polys, indices)
        return board, tri, rect, (polys, indices), base

    raise ValueError(f"unknown scenario: {scenario}")


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
    per_proposer_counts: dict[str, int] = field(default_factory=dict)
    border_standoff_min: float = 0.0
    squeeze_improved_count: int = 0
    squeeze_contact_delta: float = 0.0
    graph_yield: float = 0.0
    proposal_yield: float = 0.0
    proposal_nodes: int = 0
    unique_transforms: int = 0
    rule_score_top1: float = 0.0
    place_zone: str = ""


SCENARIO_WEIGHTS: dict[str, float] = {
    "empty_base": 1.0,
    "partial_pack": 2.0,
    "two_clusters": 2.0,
    "hole_board": 1.5,
    "packed_border": 1.5,
    "border_gap": 2.0,
    "interior_after_border": 2.0,
}


def composite_place_score(metrics: "ProposeBenchmarkMetrics") -> float:
    """Balanced composite; higher is better."""
    rule_norm = metrics.rule_score_top1
    return (
        2.0 * metrics.proposal_yield
        + 1.5 * metrics.graph_yield
        + 2.0 * metrics.kiss_fraction
        - 3.0 * metrics.contact_dist_min
        + 0.5 * rule_norm
        - 0.02 * metrics.propose_time_s
    )


def composite_local_score(metrics: ProposeBenchmarkMetrics) -> float:
    """Higher is better; scenario weight applied externally."""
    return (
        -3.0 * metrics.contact_dist_min
        + 2.0 * metrics.kiss_fraction
        + 0.5 * metrics.squeeze_contact_delta
        - 0.01 * metrics.propose_time_s
    )


def _contact_distance(placed_shapely, base_shape) -> float:
    if base_shape is None or base_shape.is_empty:
        return float("inf")
    return float(base_shape.distance(placed_shapely))


def evaluate_proposal_coords(
    coords_list: list[tuple[float, float, float]],
    board: Polygon,
    base_shape,
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


def run_propose_with_metrics(
    propose_cfg: ProposeConfig,
    scenario: str,
    seed: int,
    base_cfg: BuildGraphConfig,
    *,
    preset_label: str,
    enabled_proposers: frozenset[str] | None = None,
    guidance_seed_coords: list[tuple[float, float, float]] | None = None,
    skip_graph: bool = False,
) -> ProposeBenchmarkMetrics:
    import time

    min_dist = base_cfg.board_min_dist()
    eps = propose_cfg.placement_clearance_epsilon_ratio
    board, part_poly, _, (selected_polys, selected_indices), base_shape = scenario_context(
        scenario, seed,
    )
    placed_polys = [selected_polys[i] for i in selected_indices]
    obstacle = obstacle_shape_for_propose(placed_polys, part_poly, min_dist)

    border_focus = should_use_border_focus(obstacle, propose_cfg)
    push = propose_push_point(
        board,
        obstacle,
        smart_push=propose_cfg.smart_push_target,
        min_dist=min_dist,
        use_border_focus=border_focus,
    )
    focal = None
    if border_focus:
        focal = border_focal_for_propose(board, min_dist)
    elif obstacle is not None and not obstacle.is_empty:
        focal = obstacle

    sheet, _voids = board_context_from_geometry(board)
    geom = ProposeGeometry(
        board, obstacle, part_poly, min_dist,
        epsilon_ratio=eps,
        propose_cfg=propose_cfg,
    )
    rank_mode = effective_ranking_mode(propose_cfg, obstacle)
    proposer_counts: dict[str, int] = {}

    t0 = time.perf_counter()
    raw = collect_propose_candidates(
        obstacle,
        part_poly,
        sheet,
        propose_cfg,
        min_dist=min_dist,
        pt_push=push,
        propose_geom=geom,
        focal_shape=focal,
        enabled_proposers=enabled_proposers,
        proposer_counts=proposer_counts,
        guidance_seed_coords=guidance_seed_coords,
    )
    final = propose_coords_from_candidates(
        obstacle,
        part_poly,
        board,
        propose_cfg,
        min_dist=min_dist,
        pt_push=push,
        candidates=raw,
        rank_mode=rank_mode,
        focal_shape=focal,
    )
    elapsed = time.perf_counter() - t0

    valid, c_mean, c_min, cd_mean, cd_min, kiss_frac, border_min = evaluate_proposal_coords(
        final, board, obstacle, part_poly, min_dist, push, eps,
    )
    raw_valid, _, _, _, _, _, _ = evaluate_proposal_coords(
        raw, board, obstacle, part_poly, min_dist, push, eps,
    )

    proposals = np.asarray(final, dtype=np.float64) if final else np.zeros((0, 3))
    n_rand = 0
    n_both = 0
    proposal_nodes = 0
    graph_yield = 0.0
    proposal_yield = 0.0
    if not skip_graph:
        from nest_graph.build_graph import make_polygon_graph

        rng = np.random.default_rng(seed)
        random_t = rng.uniform(-0.2, 0.2, (8, 3)) * [0.4, 0.4, np.pi]
        graph_rand, _, _, _ = make_polygon_graph(
            board, [(part_poly, random_t)], min_dist=0.0,
        )
        n_rand = len(graph_rand.elems)
        if proposals.shape[0] == 0:
            n_both = n_rand
        else:
            graph_prop, _, _, _ = make_polygon_graph(
                board, [(part_poly, proposals)], min_dist=min_dist, epsilon_ratio=eps,
            )
            proposal_nodes = len(graph_prop.elems)
            prop_n = len(final)
            if prop_n > 0:
                proposal_yield = min(1.0, proposal_nodes / prop_n)
                graph_yield = proposal_yield
            graph_both, _, _, _ = make_polygon_graph(
                board, [(part_poly, np.concatenate([random_t, proposals]))],
                min_dist=0.0,
            )
            n_both = len(graph_both.elems)

    return ProposeBenchmarkMetrics(
        preset=preset_label,
        scenario=scenario,
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
        graph_nodes_vs_random=max(0, n_both - n_rand),
        propose_time_s=elapsed,
        per_proposer_counts=dict(proposer_counts),
        border_standoff_min=border_min,
        graph_yield=graph_yield,
        proposal_yield=proposal_yield,
        proposal_nodes=proposal_nodes,
    )


def _coords_moved(
    before: list[tuple[float, float, float]],
    after: list[tuple[float, float, float]],
    *,
    tol: float = 1e-3,
) -> int:
    n = min(len(before), len(after))
    moved = 0
    for i in range(n):
        b, a = before[i], after[i]
        if (
            abs(b[0] - a[0]) > tol
            or abs(b[1] - a[1]) > tol
            or abs(b[2] - a[2]) > tol
        ):
            moved += 1
    return moved


def _top_k_contact_delta(
    before: list[tuple[float, float, float]],
    after: list[tuple[float, float, float]],
    board: Polygon,
    base_shape,
    part_poly: Polygon,
    min_dist: float,
    pt_push: Point,
    epsilon_ratio: float,
    k: int,
) -> tuple[int, float]:
    if k <= 0 or not before:
        return 0, 0.0
    kb = before[:k]
    ka = after[:k] if after else before[:k]
    _, _, _, _, cd_before, _, _ = evaluate_proposal_coords(
        kb, board, base_shape, part_poly, min_dist, pt_push, epsilon_ratio,
    )
    _, _, _, _, cd_after, _, _ = evaluate_proposal_coords(
        ka, board, base_shape, part_poly, min_dist, pt_push, epsilon_ratio,
    )
    if cd_before >= float("inf") or cd_after >= float("inf"):
        return _coords_moved(kb, ka), 0.0
    return _coords_moved(kb, ka), max(0.0, cd_before - cd_after)


def run_propose_with_squeeze_metrics(
    propose_cfg: ProposeConfig,
    scenario: str,
    seed: int,
    base_cfg: BuildGraphConfig,
    *,
    preset_label: str,
    enabled_proposers: frozenset[str] | None = None,
    guidance_seed_coords: list[tuple[float, float, float]] | None = None,
) -> ProposeBenchmarkMetrics:
    """Run propose; record cast_squeeze movement and contact improvement."""
    import copy as copy_mod
    import time

    t0 = time.perf_counter()
    no_squeeze = copy_mod.copy(propose_cfg)
    no_squeeze.cast_squeeze_top_k = 0
    no_squeeze.cast_squeeze_passes = 0
    pre_final = _run_propose_coords_only(
        no_squeeze, scenario, seed, base_cfg,
        enabled_proposers=enabled_proposers,
        guidance_seed_coords=guidance_seed_coords,
    )
    post_final = _run_propose_coords_only(
        propose_cfg, scenario, seed, base_cfg,
        enabled_proposers=enabled_proposers,
        guidance_seed_coords=guidance_seed_coords,
    )
    elapsed = time.perf_counter() - t0

    min_dist = base_cfg.board_min_dist()
    eps = propose_cfg.placement_clearance_epsilon_ratio
    board, part_poly, _, (selected_polys, selected_indices), _ = scenario_context(
        scenario, seed,
    )
    placed_polys = [selected_polys[i] for i in selected_indices]
    obstacle = obstacle_shape_for_propose(placed_polys, part_poly, min_dist)
    border_focus = should_use_border_focus(obstacle, propose_cfg)
    push = propose_push_point(
        board,
        obstacle,
        smart_push=propose_cfg.smart_push_target,
        min_dist=min_dist,
        use_border_focus=border_focus,
    )

    valid, c_mean, c_min, cd_mean, cd_min, kiss_frac, border_min = (
        evaluate_proposal_coords(
            post_final, board, obstacle, part_poly, min_dist, push, eps,
        )
    )
    k = max(propose_cfg.cast_squeeze_top_k, 0)
    moved, delta = _top_k_contact_delta(
        pre_final, post_final, board, obstacle, part_poly,
        min_dist, push, eps, k,
    )

    return ProposeBenchmarkMetrics(
        preset=preset_label,
        scenario=scenario,
        seed=seed,
        valid_count=valid,
        top_clearance_mean=c_mean,
        top_clearance_min=c_min,
        contact_dist_mean=cd_mean,
        contact_dist_min=cd_min,
        kiss_fraction=kiss_frac,
        raw_pool_size=0,
        final_count=len(post_final),
        graph_nodes=0,
        graph_nodes_vs_random=0,
        propose_time_s=elapsed,
        per_proposer_counts={},
        border_standoff_min=border_min,
        squeeze_improved_count=moved,
        squeeze_contact_delta=delta,
    )


def _run_propose_coords_only(
    propose_cfg: ProposeConfig,
    scenario: str,
    seed: int,
    base_cfg: BuildGraphConfig,
    *,
    enabled_proposers: frozenset[str] | None = None,
    guidance_seed_coords: list[tuple[float, float, float]] | None = None,
) -> list[tuple[float, float, float]]:
    min_dist = base_cfg.board_min_dist()
    board, part_poly, _, (selected_polys, selected_indices), _ = scenario_context(
        scenario, seed,
    )
    placed_polys = [selected_polys[i] for i in selected_indices]
    obstacle = obstacle_shape_for_propose(placed_polys, part_poly, min_dist)
    border_focus = should_use_border_focus(obstacle, propose_cfg)
    push = propose_push_point(
        board,
        obstacle,
        smart_push=propose_cfg.smart_push_target,
        min_dist=min_dist,
        use_border_focus=border_focus,
    )
    focal = None
    if border_focus:
        focal = border_focal_for_propose(board, min_dist)
    elif obstacle is not None and not obstacle.is_empty:
        focal = obstacle
    sheet, _voids = board_context_from_geometry(board)
    geom = ProposeGeometry(
        board, obstacle, part_poly, min_dist,
        epsilon_ratio=propose_cfg.placement_clearance_epsilon_ratio,
        propose_cfg=propose_cfg,
    )
    rank_mode = effective_ranking_mode(propose_cfg, obstacle)
    raw = collect_propose_candidates(
        obstacle,
        part_poly,
        sheet,
        propose_cfg,
        min_dist=min_dist,
        pt_push=push,
        propose_geom=geom,
        focal_shape=focal,
        enabled_proposers=enabled_proposers,
        proposer_counts=None,
        guidance_seed_coords=guidance_seed_coords,
    )
    return propose_coords_from_candidates(
        obstacle,
        part_poly,
        board,
        propose_cfg,
        min_dist=min_dist,
        pt_push=push,
        candidates=raw,
        rank_mode=rank_mode,
        focal_shape=focal,
    )


def _unique_transform_count(
    coords: list[tuple[float, float, float]],
) -> int:
    keys = {
        (round(c[0], 4), round(c[1], 4), round(c[2], 4))
        for c in coords
    }
    return len(keys)


def run_place_propose_metrics(
    propose_cfg: ProposeConfig,
    scenario: str,
    seed: int,
    base_cfg: BuildGraphConfig,
    *,
    preset_label: str,
    force_zone: str | None = None,
) -> ProposeBenchmarkMetrics:
    """Run place-routed propose; report graph-yield and zone-stratified metrics."""
    import time

    from nest_graph.build_graph import make_polygon_graph
    from nest_graph.config import PLACE_ZONES
    from nest_graph.elem_graph import PlacementRuleSet, score_elems
    from nest_graph.propose.context import classify_propose_zone
    from nest_graph.propose.pipeline import proposed_transforms_for_groups

    min_dist = base_cfg.board_min_dist()
    eps = propose_cfg.placement_clearance_epsilon_ratio
    board, part_poly, _, (selected_polys, selected_indices), base_shape = scenario_context(
        scenario, seed,
    )
    placed_polys = [selected_polys[i] for i in selected_indices]
    obstacle = obstacle_shape_for_propose(
        placed_polys, part_poly, min_dist, propose_cfg=propose_cfg,
    )
    zone = force_zone or classify_propose_zone(
        board,
        obstacle,
        part_poly,
        min_dist=min_dist,
        propose_cfg=propose_cfg,
        selected_polys=placed_polys,
    )
    if force_zone and force_zone in PLACE_ZONES:
        cfg = ProposeConfig.for_place(force_zone, base=propose_cfg)
        cfg = cfg.model_copy(update={"place_profiles_enabled": True})
    elif propose_cfg.place_profiles_enabled:
        cfg = ProposeConfig.for_place(zone, base=propose_cfg)
    else:
        cfg = propose_cfg

    push = propose_push_point(
        board,
        obstacle,
        smart_push=cfg.smart_push_target,
        min_dist=min_dist,
        use_border_focus=should_use_border_focus(obstacle, cfg),
    )
    parts = [(part_poly, 0)]
    proposer_counts: dict[str, int] = {}
    zones_used: list[str] = []

    t0 = time.perf_counter()
    by_group = proposed_transforms_for_groups(
        board,
        parts,
        selected_polys,
        selected_indices,
        cfg.model_copy(update={"place_profiles_enabled": True}),
        min_dist=min_dist,
        rules=None,
        proposer_counts_out=proposer_counts,
        zones_used_out=zones_used,
    )
    proposals = by_group.get(0, np.zeros((0, 3)))
    elapsed = time.perf_counter() - t0

    final = [
        (float(t[0]), float(t[1]), float(t[2]))
        for t in proposals
    ]
    valid, c_mean, c_min, cd_mean, cd_min, kiss_frac, border_min = evaluate_proposal_coords(
        final, board, obstacle, part_poly, min_dist, push, eps,
    )

    graph_nodes = 0
    graph_yield = 0.0
    proposal_nodes = 0
    proposal_yield = 0.0
    rule_top1 = 0.0
    if proposals.shape[0] > 0:
        graph, _, _, _ = make_polygon_graph(
            board, [(part_poly, proposals)], min_dist=min_dist, epsilon_ratio=eps,
        )
        graph_nodes = len(graph.elems)
        proposal_nodes = graph_nodes
        graph_yield = graph_nodes / proposals.shape[0]
        proposal_yield = graph_yield
        if graph.elems:
            scores = score_elems(graph, PlacementRuleSet())
            rule_top1 = max(scores) if scores else 0.0

    return ProposeBenchmarkMetrics(
        preset=preset_label,
        scenario=scenario,
        seed=seed,
        valid_count=valid,
        top_clearance_mean=c_mean,
        top_clearance_min=c_min,
        contact_dist_mean=cd_mean,
        contact_dist_min=cd_min,
        kiss_fraction=kiss_frac,
        raw_pool_size=proposals.shape[0],
        final_count=len(final),
        graph_nodes=graph_nodes,
        graph_nodes_vs_random=graph_nodes,
        propose_time_s=elapsed,
        per_proposer_counts=dict(proposer_counts),
        border_standoff_min=border_min,
        graph_yield=graph_yield,
        proposal_yield=proposal_yield,
        proposal_nodes=proposal_nodes,
        unique_transforms=_unique_transform_count(final),
        rule_score_top1=rule_top1,
        place_zone=zone,
    )
