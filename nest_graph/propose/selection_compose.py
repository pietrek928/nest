"""Compose nest/DFS selection scores and choose the nest seed.

Shared by ``build_graph`` mid-pack and ``nesting_evaluator`` so geom/void/border
policy (G22/G24) cannot drift. Propose ranking stays in ``ranking.py``.
"""

from dataclasses import dataclass, field
from typing import Any, Sequence

from shapely import Point
from shapely.geometry.base import BaseGeometry

from nest_graph.elem_graph import (
    PlacementRuleSet,
    PointPlaceRule,
    SelectMode,
    SelectOptions,
    Vec2,
    nest_by_scores,
)
from nest_graph.geometry import Geometry
from nest_graph.propose.context import should_use_border_focus
from nest_graph.propose.void_selection import (
    apply_void_selection_boosts,
    boost_border_scores,
    count_selected_in_free,
    void_attractor_radius,
)
from nest_graph.propose.motif_lock import (
    sequential_accept_motif_cohorts,
)


@dataclass
class ComposedSelection:
    """Result of score compose + nest seed (before DFS / pin)."""

    scores: list[float]
    refine_scores: list[float]
    selected_nest: list[int]
    nest_rules: list
    refine_rules: Any
    free_info: Any
    free_poly: Any
    sheet_diag: float
    geom_w: float
    use_nest_by_scores: bool
    boost_hits: dict[str, int] = field(default_factory=dict)
    geom_stats: dict = field(default_factory=dict)
    void_r: float = 0.0
    n_void_nest: int = 0


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


def _truncate_rule_set(rule_set: PlacementRuleSet, max_rules: int) -> PlacementRuleSet:
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


def make_void_attractor_rule_set(
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


def merge_void_attractor_into_rule_sets(
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
        out.append(_truncate_rule_set(merged, max_rules_per_set))
    return out


def compose_and_nest_selection(
    *,
    graph,
    rule_sets: list,
    active_rules,
    scores: list[float],
    polys: list,
    group_id: Sequence[int],
    transform: Sequence,
    candidate_geoms: list[Geometry] | None,
    packed_geoms: list[Geometry],
    part_areas: Sequence[float],
    free_info,
    cfg,
    selection,
    first_pass: bool,
    outline: BaseGeometry,
    min_dist: float,
    sheet_area: float,
    sheet_diag: float,
    propose_stats: dict | None,
    ngroups: int,
) -> ComposedSelection:
    """Apply void/geom boosts, pick nest seed, prepare refine_scores (G22/G24)."""
    from shapely.geometry import Polygon

    free_poly = free_info.target_poly
    nest_rules = rule_sets
    refine_rules = active_rules
    scores = list(scores)
    sel = selection
    geom_w = float(getattr(cfg.propose, "selection_geom_weight", 0.0) or 0.0)
    void_r = void_attractor_radius(
        min_dist, sheet_diag, cfg.rules.place_rule_radius,
    )
    use_nest_by_scores = bool(scores)

    geom_stats: dict = {}
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
        candidate_geoms=candidate_geoms if geom_w > 0.0 else None,
        packed_geoms=packed_geoms,
        outline=outline,
        min_dist=min_dist,
        sheet_area=sheet_area,
        geom_stats_out=geom_stats,
    )
    if geom_stats and propose_stats is not None:
        propose_stats["geom_ms"] = geom_stats.get("geom_ms", 0.0)
        propose_stats["geom_share"] = geom_stats.get("geom_share", 0.0)

    n_graph = len(graph.elems) if hasattr(graph, "elems") else len(getattr(graph, "collisions", []))
    if len(scores) != n_graph:
        raise AssertionError(
            f"selection scores length {len(scores)} != graph size {n_graph}"
        )

    locked_motif: list[int] = []
    if bool(getattr(cfg.propose, "enable_motif_sequential_accept", False)):
        cohorts = (propose_stats or {}).get("motif_cohorts") or []
        void_geoms: list = []
        try:
            from nest_graph.board import board_context_from_geometry

            _sheet, void_geoms = board_context_from_geometry(outline)
            void_geoms = list(void_geoms or [])
        except Exception:
            void_geoms = []
        locked_motif, seq_telem = sequential_accept_motif_cohorts(
            graph=graph,
            scores=scores,
            group_id=group_id,
            transform=transform,
            cohorts=cohorts,
            candidate_geoms=candidate_geoms,
            void_geoms=void_geoms,
            packed_geoms=packed_geoms,
            min_dist=float(min_dist),
            pole=getattr(free_info, "target_pt", None),
            max_accept=int(
                getattr(cfg.propose, "motif_sequential_accept_max", 3) or 3
            ),
            rcl_top_k=10,
        )
        boost_hits = dict(boost_hits)
        boost_hits["motif_sequential"] = int(seq_telem.get("motif_sequential_full", 0))
        if propose_stats is not None:
            propose_stats.update(seq_telem)
            propose_stats["motif_locked"] = list(locked_motif)

    nest_opts = SelectOptions()
    nest_opts.local_swap = False
    if locked_motif:
        nest_opts.mode = SelectMode.greedy_score
        nest_opts.locked_indices = [int(i) for i in locked_motif]
    selected_nest = list(nest_by_scores(graph, scores, nest_opts)) if scores else []

    refine_scores = list(scores)
    # Refine-only non-negative void/coverage term (tie-break within lex count/area).
    if (
        free_info is not None
        and free_info.kind == "large_void"
        and free_poly is not None
        and not getattr(free_poly, "is_empty", True)
    ):
        void_term = float(getattr(cfg.propose, "void_island_score_boost", 0.0) or 0.0) * 0.25
        if void_term > 0.0:
            for i, poly in enumerate(polys):
                if i >= len(refine_scores):
                    break
                try:
                    if poly is not None and not poly.is_empty and free_poly.contains(poly.centroid):
                        refine_scores[i] = float(refine_scores[i]) + void_term
                except Exception:
                    continue
    if (
        first_pass
        and should_use_border_focus(Polygon(), cfg.propose)
        and free_info.kind != "large_void"
        and geom_w <= 0.0  # G22: skip border boost when selection geom on
    ):
        boost_border_scores(
            polys, refine_scores, outline, min_dist,
            weight=cfg.propose.border_selection_score_boost,
        )

    return ComposedSelection(
        scores=scores,
        refine_scores=refine_scores,
        selected_nest=selected_nest,
        nest_rules=nest_rules,
        refine_rules=refine_rules,
        free_info=free_info,
        free_poly=free_poly,
        sheet_diag=sheet_diag,
        geom_w=geom_w,
        use_nest_by_scores=use_nest_by_scores,
        boost_hits=boost_hits,
        geom_stats=geom_stats,
        void_r=void_r,
        n_void_nest=count_selected_in_free(polys, selected_nest, free_poly),
    )
