from typing import List
from libcpp cimport bool
from libcpp.vector cimport vector
from pydantic import BaseModel

from .elem_graph_ccexport cimport (
    Tvertex,
    BBox as BBoxCC, Point as PointCC, ElemGroup as ElemGroupCC,
    PointPlaceRule as PointPlaceRuleCC, BBoxPlaceRule as BBoxPlaceRuleCC,
    ElemGraph as ElemGraphCC, PlacementRuleSet as PlacementRuleSetCC,
    nest_by_graph as nest_by_graph_cc, sort_graph as sort_graph_cc,
    increase_selection_dfs as increase_selection_dfs_cc,
    increase_score_dfs as increase_score_dfs_cc
)


class BBox(BaseModel):
    xstart: float
    xend: float
    ystart: float
    yend: float

class Point(BaseModel):
    x: float
    y: float

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

cdef class PlacementRuleSet:
    cdef public PlacementRuleSetCC cpp_obj

    def append_point_rule(self, point_rule : PointPlaceRule):
        cdef PointPlaceRuleCC point_rule_cc
        point_rule_cc.x = point_rule.x
        point_rule_cc.y = point_rule.y
        point_rule_cc.r = point_rule.r
        point_rule_cc.w = point_rule.w
        point_rule_cc.group = point_rule.group
        self.cpp_obj.point_rules.push_back(point_rule_cc)

    def append_bbox_rule(self, bbox_rule : BBoxPlaceRule):
        cdef BBoxPlaceRuleCC bbox_rule_cc
        bbox_rule_cc.bbox.xstart = bbox_rule.bbox.xstart
        bbox_rule_cc.bbox.xend = bbox_rule.bbox.xend
        bbox_rule_cc.bbox.ystart = bbox_rule.bbox.ystart
        bbox_rule_cc.bbox.yend = bbox_rule.bbox.yend
        bbox_rule_cc.r = bbox_rule.r
        bbox_rule_cc.w = bbox_rule.w
        bbox_rule_cc.group = bbox_rule.group
        self.cpp_obj.bbox_rules.push_back(bbox_rule_cc)

cdef class ElemGraph:
    cdef public ElemGraphCC cpp_obj

    def append_elem(self, group_id: int, center: Point, coord: BBox):
        self.cpp_obj.group_id.push_back(group_id)
        cdef PointCC center_cc
        center_cc.x = center.x
        center_cc.y = center.y
        self.cpp_obj.centers.push_back(center_cc)
        cdef BBoxCC coord_cc
        coord_cc.xstart = coord.xstart
        coord_cc.xend = coord.xend
        coord_cc.ystart = coord.ystart
        coord_cc.yend = coord.yend
        self.cpp_obj.coords.push_back(coord_cc)
        self.cpp_obj.collisions.emplace_back()

    def add_collision(self, group1: int, group2: int):
        self.cpp_obj.collisions[group1].push_back(group2)
        self.cpp_obj.collisions[group2].push_back(group1)


def nest_by_graph(ElemGraph g, List[PlacementRuleSet] cases):
    cdef vector[PlacementRuleSetCC] cases_cc
    for case in cases:
        cases_cc.push_back(case.cpp_obj)
    cdef vector[vector[Tvertex]] result = nest_by_graph_cc(g.cpp_obj, cases_cc)
    return result


def sort_graph(ElemGraph g, PlacementRuleSet rules, bool reverse=False):
    r = ElemGraph()
    r.cpp_obj = sort_graph_cc(g.cpp_obj, rules.cpp_obj, reverse)
    return r


def increase_selection_dfs(ElemGraph g, List[int] selection, max_tries: int, min_collisions: int):
    return increase_selection_dfs_cc(g.cpp_obj, selection, max_tries, min_collisions)


def increase_score_dfs(ElemGraph g, List[int] selection, PlacementRuleSet rules):
    return increase_score_dfs_cc(g.cpp_obj, selection, rules.cpp_obj)
