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
    Vec2,
    nest_by_graph,
    nest_by_scores,
    score_elems,
)
from nest_graph.geometry import Geometry
from nest_graph.propose.context import should_use_border_focus
from nest_graph.propose.void_selection import (
    apply_void_selection_boosts,
    boost_border_scores,
    count_selected_in_free,
    void_attractor_radius,
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


def _greedy_independent_set_ordered(graph, order: list[int]) -> list[int]:
    kept: list[int] = []
    kept_set: set[int] = set()
    for v in order:
        if any(u in kept_set for u in graph.collisions[v]):
            continue
        kept.append(v)
        kept_set.add(v)
    return kept


def nest_seed_from_boosted_scores(graph, scores: Sequence[float]) -> list[int]:
    """Score-descending greedy MIS so Python void boost reaches the nest seed."""
    n = min(len(scores), len(graph.collisions))
    if n <= 0:
        return []
    order = sorted(range(n), key=lambda i: float(scores[i]), reverse=True)
    return _greedy_independent_set_ordered(graph, order)


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
    attr_w = float(cfg.propose.void_attractor_rule_weight)
    pole_w = float(cfg.propose.void_island_score_boost)
    pocket_w = float(cfg.propose.pocket_score_boost)
    small_w = float(cfg.propose.small_part_void_score_boost)
    geom_w = float(getattr(cfg.propose, "selection_geom_weight", 0.0) or 0.0)
    void_r = void_attractor_radius(
        min_dist, sheet_diag, cfg.rules.place_rule_radius,
    )
    use_nest_by_scores = geom_w > 0.0 and bool(scores)

    # G24: skip void attractor rules when nest_by_scores + geom are on
    if (
        not use_nest_by_scores
        and free_info.kind == "large_void"
        and free_info.target_pt is not None
        and attr_w > 0.0
    ):
        attractor = make_void_attractor_rule_set(
            free_info.target_pt,
            ngroups=ngroups,
            radius=void_r,
            weight=attr_w,
        )
        nest_rules = merge_void_attractor_into_rule_sets(
            rule_sets,
            attractor,
            nest_rule_sets_used=sel.nest_rule_sets_used,
            max_rules_per_set=cfg.rules.max_rules_per_set,
        )
        refine_rules = active_rule_set(nest_rules)
        scores = list(score_elems(graph, refine_rules))

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

    use_greedy_nest = (
        bool(cfg.propose.void_greedy_nest_seed)
        and free_info.kind == "large_void"
        and scores
        and (pole_w > 0.0 or pocket_w > 0.0 or small_w > 0.0)
        and not use_nest_by_scores
    )
    if use_nest_by_scores:
        selected_nest = list(nest_by_scores(graph, scores))
    elif use_greedy_nest:
        selected_nest = nest_seed_from_boosted_scores(graph, scores)
    else:
        selected_nest = list(
            nest_by_graph(graph, nest_rules[: sel.nest_rule_sets_used])[0]
        )

    refine_scores = list(scores)
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
