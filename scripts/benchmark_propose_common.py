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
    base_shape_from_selection,
    border_focal_for_propose,
    collect_propose_candidates,
    effective_ranking_mode,
    obstacle_shape_for_propose,
    propose_push_point,
    should_use_border_focus,
)
from nest_graph.propose.pipeline import _propose_coords_from_candidates
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
    if enabled_name == "guidance_propositions":
        on["use_guidance_propositions"] = True
        on["guidance_use_tight_packing"] = True
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
        if not geom.is_valid_placement(placed_g, pt_push, (coords[0], coords[1])):
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
) -> ProposeBenchmarkMetrics:
    import time

    from nest_graph.build_graph import make_polygon_graph

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
    final = _propose_coords_from_candidates(
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
    rng = np.random.default_rng(seed)
    random_t = rng.uniform(-0.2, 0.2, (8, 3)) * [0.4, 0.4, np.pi]
    graph_rand, _, _, _ = make_polygon_graph(
        board, [(part_poly, random_t)], min_dist=0.0,
    )
    n_rand = len(graph_rand.elems)
    if proposals.shape[0] == 0:
        n_both = n_rand
    else:
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
    )
