from __future__ import annotations

from typing import Any, List, Optional

from pydantic import BaseModel

from . import _elem_graph as _m


class BBox(BaseModel):
    xstart: float
    xend: float
    ystart: float
    yend: float


class Point(BaseModel):
    x: float
    y: float


class ElemPlace(BaseModel):
    x: float
    y: float
    a: float


class PointPlaceRule(BaseModel):
    x: float
    y: float
    r: float
    w: float
    group: int


class BBoxPlaceRule(BaseModel):
    bbox: BBox
    r: float
    w: float
    group: int


class PointAngleRule(BaseModel):
    x: float
    y: float
    a: float
    r: float
    w: float
    group: int


class BBoxAngleRule(BaseModel):
    bbox: BBox
    a: float
    r: float
    w: float
    group: int


class PlacementRuleSet:
    def __init__(self, _obj: Optional[Any] = None):
        self._obj = _obj if _obj is not None else _m.PlacementRuleSet()

    def append_rule(self, rule):
        if isinstance(rule, PointPlaceRule):
            pr = _m.PointPlaceRule()
            pr.x = rule.x
            pr.y = rule.y
            pr.r = rule.r
            pr.w = rule.w
            pr.group = rule.group
            self._obj.point_rules.append(pr)
        elif isinstance(rule, BBoxPlaceRule):
            br = _m.BBoxPlaceRule()
            br.bbox.xstart = rule.bbox.xstart
            br.bbox.xend = rule.bbox.xend
            br.bbox.ystart = rule.bbox.ystart
            br.bbox.yend = rule.bbox.yend
            br.r = rule.r
            br.w = rule.w
            br.group = rule.group
            self._obj.bbox_rules.append(br)
        elif isinstance(rule, PointAngleRule):
            par = _m.PointAngleRule()
            par.x = rule.x
            par.y = rule.y
            par.a = rule.a
            par.r = rule.r
            par.w = rule.w
            par.group = rule.group
            self._obj.point_angle_rules.append(par)
        elif isinstance(rule, BBoxAngleRule):
            bar = _m.BBoxAngleRule()
            bar.bbox.xstart = rule.bbox.xstart
            bar.bbox.xend = rule.bbox.xend
            bar.bbox.ystart = rule.bbox.ystart
            bar.bbox.yend = rule.bbox.yend
            bar.a = rule.a
            bar.r = rule.r
            bar.w = rule.w
            bar.group = rule.group
            self._obj.bbox_angle_rules.append(bar)
        else:
            raise ValueError(f"Unknown rule type {type(rule).__name__}")

    def size(self):
        return self._obj.size()


class RuleMutationSettings(BaseModel):
    box: BBox
    dpos: float
    dw: float
    da: float
    insert_p: float
    remove_p: float
    mutate_p: float
    ngroups: int


class ElemGraph:
    def __init__(self, _obj: Optional[Any] = None):
        self._obj = _obj if _obj is not None else _m.ElemGraph()

    def append_elem(self, group_id: int, center: Point, coord: BBox):
        self._obj.group_id.append(group_id)
        ep = _m.ElemPlace()
        ep.x = center.x
        ep.y = center.y
        ep.a = 0.0
        self._obj.elems.append(ep)
        bb = _m.BBox()
        bb.xstart = coord.xstart
        bb.xend = coord.xend
        bb.ystart = coord.ystart
        bb.yend = coord.yend
        self._obj.coords.append(bb)
        self._obj.collisions.append([])

    def add_collision(self, group1: int, group2: int):
        self._obj.collisions[group1].append(group2)
        self._obj.collisions[group2].append(group1)


class ElemScores:
    def __init__(self, scores: Optional[List[float]] = None):
        self.scores = scores if scores is not None else []


def augment_rules(
    rules: List[PlacementRuleSet], settings: RuleMutationSettings
) -> List[PlacementRuleSet]:
    ns = _m.RuleMutationSettings()
    ns.box.xstart = settings.box.xstart
    ns.box.xend = settings.box.xend
    ns.box.ystart = settings.box.ystart
    ns.box.yend = settings.box.yend
    ns.dpos = settings.dpos
    ns.dw = settings.dw
    ns.da = settings.da
    ns.insert_p = settings.insert_p
    ns.remove_p = settings.remove_p
    ns.mutate_p = settings.mutate_p
    ns.ngroups = settings.ngroups
    out = _m.augment_rules([r._obj for r in rules], ns)
    return [PlacementRuleSet(_obj=x) for x in out]


def nest_by_graph(g: ElemGraph, cases: List[PlacementRuleSet]):
    return _m.nest_by_graph(g._obj, [c._obj for c in cases])


def sort_graph(g: ElemGraph, rules: PlacementRuleSet, reverse: bool = False):
    return ElemGraph(_obj=_m.sort_graph(g._obj, rules._obj, reverse))


def score_rules(graphs: List[ElemGraph], rules: List[PlacementRuleSet]) -> List[float]:
    return list(_m.score_rules([g._obj for g in graphs], [r._obj for r in rules]))


def score_elems(g: ElemGraph, rules: PlacementRuleSet):
    s = _m.score_elems(g._obj, rules._obj)
    return ElemScores(scores=list(s))


def increase_selection_dfs(
    g: ElemGraph, selection: List[int], max_tries: int, min_collisions: int
):
    return list(
        _m.increase_selection_dfs(g._obj, selection, max_tries, min_collisions)
    )


def increase_score_dfs(g: ElemGraph, selection: List[int], scores: ElemScores):
    return list(_m.increase_score_dfs(g._obj, selection, scores.scores))
