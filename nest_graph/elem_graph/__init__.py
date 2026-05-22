from __future__ import annotations

import math
from typing import Any, List, Optional, Tuple, Union

from pydantic import BaseModel, computed_field

from . import _elem_graph as _m

Vec2Like = Union["Vec2", Tuple[float, float], List[float]]
CircleLike = Union[
    "Circle",
    Tuple[float, float, float, float],
    Tuple[Tuple[float, float], float],
]


class Vec2(BaseModel):
    x: float
    y: float

    @classmethod
    def from_tuple(cls, t: Tuple[float, float]) -> "Vec2":
        return cls(x=t[0], y=t[1])

    def to_tuple(self) -> Tuple[float, float]:
        return (self.x, self.y)

    def to_native(self) -> _m.Vec2:
        return _m.Vec2(self.x, self.y)


class Circle(BaseModel):
    center: Vec2
    r_sq: float

    @computed_field
    @property
    def radius(self) -> float:
        return math.sqrt(self.r_sq)

    @classmethod
    def from_bounds(
        cls, xmin: float, ymin: float, xmax: float, ymax: float
    ) -> "Circle":
        cx = 0.5 * (xmin + xmax)
        cy = 0.5 * (ymin + ymax)
        hx = 0.5 * (xmax - xmin)
        hy = 0.5 * (ymax - ymin)
        return cls(center=Vec2(x=cx, y=cy), r_sq=hx * hx + hy * hy)

    @classmethod
    def from_center_radius(cls, cx: float, cy: float, radius: float) -> "Circle":
        return cls(center=Vec2(x=cx, y=cy), r_sq=radius * radius)

    @classmethod
    def from_tuple(
        cls,
        t: Union[
            Tuple[float, float, float, float],
            Tuple[Tuple[float, float], float],
        ],
    ) -> "Circle":
        if len(t) == 4 and isinstance(t[0], (int, float)):
            return cls.from_bounds(float(t[0]), float(t[1]), float(t[2]), float(t[3]))
        return cls.from_center_radius(t[0][0], t[0][1], float(t[1]))

    def to_native(self) -> _m.Circle:
        return _m.Circle(self.center.to_native(), self.r_sq)


def _coerce_vec2(v: Vec2Like) -> Vec2:
    if isinstance(v, Vec2):
        return v
    if isinstance(v, (tuple, list)) and len(v) >= 2:
        return Vec2.from_tuple((float(v[0]), float(v[1])))
    if isinstance(v, _m.Vec2):
        return Vec2(x=v.x, y=v.y)
    raise TypeError(f"Expected Vec2 or (x, y) tuple, got {type(v).__name__}")


def _coerce_circle(v: CircleLike) -> Circle:
    if isinstance(v, Circle):
        return v
    if isinstance(v, (tuple, list)):
        if len(v) == 4 and isinstance(v[0], (int, float)):
            return Circle.from_bounds(float(v[0]), float(v[1]), float(v[2]), float(v[3]))
        if len(v) == 2:
            center, radius = v[0], v[1]
            if isinstance(center, (tuple, list)):
                return Circle.from_center_radius(
                    float(center[0]), float(center[1]), float(radius)
                )
    if isinstance(v, _m.Circle):
        return Circle(
            center=Vec2(x=v.center.x, y=v.center.y),
            r_sq=v.r_sq,
        )
    raise TypeError(
        f"Expected Circle, bounds 4-tuple, or (center, radius), got {type(v).__name__}"
    )


class ElemPlace(BaseModel):
    pos: Vec2
    a: float


class PointPlaceRule(BaseModel):
    pos: Vec2
    r: float
    w: float
    group: int


class CirclePlaceRule(BaseModel):
    circle: Circle
    r: float
    w: float
    group: int


class PointAngleRule(BaseModel):
    pos: Vec2
    a: float
    r: float
    w: float
    group: int


class CircleAngleRule(BaseModel):
    circle: Circle
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
            pr.pos = rule.pos.to_native()
            pr.r = rule.r
            pr.w = rule.w
            pr.group = rule.group
            self._obj.point_rules = list(self._obj.point_rules) + [pr]
        elif isinstance(rule, CirclePlaceRule):
            br = _m.CirclePlaceRule()
            br.circle = rule.circle.to_native()
            br.r = rule.r
            br.w = rule.w
            br.group = rule.group
            self._obj.circle_rules = list(self._obj.circle_rules) + [br]
        elif isinstance(rule, PointAngleRule):
            par = _m.PointAngleRule()
            par.pos = rule.pos.to_native()
            par.a = rule.a
            par.r = rule.r
            par.w = rule.w
            par.group = rule.group
            self._obj.point_angle_rules = list(self._obj.point_angle_rules) + [par]
        elif isinstance(rule, CircleAngleRule):
            bar = _m.CircleAngleRule()
            bar.circle = rule.circle.to_native()
            bar.a = rule.a
            bar.r = rule.r
            bar.w = rule.w
            bar.group = rule.group
            self._obj.circle_angle_rules = list(self._obj.circle_angle_rules) + [bar]
        else:
            raise ValueError(f"Unknown rule type {type(rule).__name__}")

    def size(self):
        return self._obj.size()


class RuleMutationSettings(BaseModel):
    region: Circle
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

    def reserve_elems(self, n: int) -> None:
        self._obj.reserve_elems(n)

    def append_elem(
        self,
        group_id: int,
        center: Vec2Like,
        bounds: CircleLike,
    ):
        center_v = _coerce_vec2(center)
        circle = _coerce_circle(bounds)
        ep = _m.ElemPlace()
        ep.pos = center_v.to_native()
        ep.a = 0.0
        self._obj.push_elem(group_id, ep, circle.to_native())

    def add_collision(self, group1: int, group2: int):
        self._obj.add_collision_pair(group1, group2)


class ElemScores:
    def __init__(self, scores: Optional[List[float]] = None):
        self.scores = scores if scores is not None else []


def augment_rules(
    rules: List[PlacementRuleSet], settings: RuleMutationSettings
) -> List[PlacementRuleSet]:
    ns = _m.RuleMutationSettings()
    ns.region = settings.region.to_native()
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
