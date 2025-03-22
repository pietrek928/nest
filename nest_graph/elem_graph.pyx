from ctypes import pointer, POINTER
from typing import List, Optional
from cython import cfunc
from cython.operator cimport dereference as deref, preincrement as inc
from libcpp cimport bool
from libcpp.vector cimport vector
from pydantic import BaseModel

from .elem_graph_ccexport cimport (
    Tvertex, Tscore, size,
    BBox as BBoxCC, Point as PointCC, ElemPlace as ElemPlaceCC, ElemGroup as ElemGroupCC,
    PointPlaceRule as PointPlaceRuleCC, BBoxPlaceRule as BBoxPlaceRuleCC,
    PointAngleRule as PointAngleRuleCC, BBoxAngleRule as BBoxAngleRuleCC,
    RuleMutationSettings as RuleMutationSettingsCC,
    ElemGraph as ElemGraphCC, PlacementRuleSet as PlacementRuleSetCC,
    augment_rules as augment_rules_cc, score_rules as score_rules_cc,
    nest_by_graph as nest_by_graph_cc, sort_graph as sort_graph_cc, score_elems as score_elems_cc,
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

cdef class PlacementRuleSet:
    cdef public PlacementRuleSetCC cc_obj

    cdef set_cc_obj(self, PlacementRuleSetCC *cc_ptr):
        self.cc_obj = cc_ptr[0]

    # cdef __cinit__(self, PlacementRuleSetCC *cc_ptr = NULL):
    #     if cc_ptr is NULL:
    #         self.cc_obj = PlacementRuleSetCC()
    #     else:
    #         self.cc_obj = cc[0]

    def append_rule(self, rule):
        cdef PointPlaceRuleCC point_rule_cc
        cdef BBoxPlaceRuleCC bbox_rule_cc
        cdef PointAngleRuleCC point_angle_rule_cc
        cdef BBoxAngleRuleCC bbox_angle_rule_cc

        if isinstance(rule, PointPlaceRule):
            point_rule_cc.x = rule.x
            point_rule_cc.y = rule.y
            point_rule_cc.r = rule.r
            point_rule_cc.w = rule.w
            point_rule_cc.group = rule.group
            self.cc_obj.point_rules.push_back(point_rule_cc)

        elif isinstance(rule, BBoxPlaceRule):
            bbox_rule_cc.bbox.xstart = rule.bbox.xstart
            bbox_rule_cc.bbox.xend = rule.bbox.xend
            bbox_rule_cc.bbox.ystart = rule.bbox.ystart
            bbox_rule_cc.bbox.yend = rule.bbox.yend
            bbox_rule_cc.r = rule.r
            bbox_rule_cc.w = rule.w
            bbox_rule_cc.group = rule.group
            self.cc_obj.bbox_rules.push_back(bbox_rule_cc)

        elif isinstance(rule, PointAngleRule):
            point_angle_rule_cc.x = rule.x
            point_angle_rule_cc.y = rule.y
            point_angle_rule_cc.a = rule.a
            point_angle_rule_cc.r = rule.r
            point_angle_rule_cc.w = rule.w
            point_angle_rule_cc.group = rule.group
            self.cc_obj.point_angle_rules.push_back(point_angle_rule_cc)
        
        elif isinstance(rule, BBoxAngleRule):
            bbox_angle_rule_cc.bbox.xstart = rule.bbox.xstart
            bbox_angle_rule_cc.bbox.xend = rule.bbox.xend
            bbox_angle_rule_cc.bbox.ystart = rule.bbox.ystart
            bbox_angle_rule_cc.bbox.yend = rule.bbox.yend
            bbox_angle_rule_cc.a = rule.a
            bbox_angle_rule_cc.r = rule.r
            bbox_angle_rule_cc.w = rule.w
            bbox_angle_rule_cc.group = rule.group
            self.cc_obj.bbox_angle_rules.push_back(bbox_angle_rule_cc)
        
        else:
            raise ValueError(f'Unknown rule type {type(rule).__name__}')

    def size(self):
        return size(self.cc_obj)

cdef PlacementRuleSet_Init(PlacementRuleSetCC *cc_obj = NULL):
    r = PlacementRuleSet()
    if cc_obj is not NULL:
        r.set_cc_obj(cc_obj)
    return r

# cdef PlacementRuleSet_Init(const PlacementRuleSetCC *cc_obj):
#     r = PlacementRuleSet()
#     r.cc_obj = cc_obj[0]
#     return r

class RuleMutationSettings(BaseModel):
    box: BBox
    dpos: float
    dw: float
    da: float
    insert_p: float
    remove_p: float
    mutate_p: float
    ngroups: int

cdef _transform_rules_to_cc(vector[PlacementRuleSetCC] &rules_cc):
    rules = []
    cdef vector[PlacementRuleSetCC].iterator it = rules_cc.begin()
    cdef PlacementRuleSetCC rule_set
    while it != rules_cc.end():
        r = PlacementRuleSet()
        rule_set = deref(it)
        r.cc_obj = rule_set
        rules.append(r)
        inc(it)
    return rules

def augment_rules(
    rules: List[PlacementRuleSet], settings: RuleMutationSettings
) -> List[PlacementRuleSet]:
    cdef RuleMutationSettingsCC settings_cc
    settings_cc.box.xstart = settings.box.xstart
    settings_cc.box.xend = settings.box.xend
    settings_cc.box.ystart = settings.box.ystart
    settings_cc.box.yend = settings.box.yend
    settings_cc.dpos = settings.dpos
    settings_cc.dw = settings.dw
    settings_cc.da = settings.da
    settings_cc.insert_p = settings.insert_p
    settings_cc.remove_p = settings.remove_p
    settings_cc.mutate_p = settings.mutate_p
    settings_cc.ngroups = settings.ngroups

    cdef vector[PlacementRuleSetCC] rules_cc
    for rule in rules:
        rules_cc.push_back(<PlacementRuleSetCC>rule.cc_obj)

    cdef vector[PlacementRuleSetCC] rules_out_cc = augment_rules_cc(rules_cc, settings_cc)
    return _transform_rules_to_cc(rules_out_cc)


cdef class ElemGraph:
    cdef public ElemGraphCC cc_obj

    def append_elem(self, group_id: int, center: Point, coord: BBox):
        self.cc_obj.group_id.push_back(group_id)
        cdef ElemPlaceCC place_cc
        place_cc.x = center.x
        place_cc.y = center.y
        self.cc_obj.elems.push_back(place_cc)
        cdef BBoxCC coord_cc
        coord_cc.xstart = coord.xstart
        coord_cc.xend = coord.xend
        coord_cc.ystart = coord.ystart
        coord_cc.yend = coord.yend
        self.cc_obj.coords.push_back(coord_cc)
        self.cc_obj.collisions.emplace_back()

    def add_collision(self, group1: int, group2: int):
        self.cc_obj.collisions[group1].push_back(group2)
        self.cc_obj.collisions[group2].push_back(group1)


cdef class ElemOrder:
    cdef public vector[Tvertex] elems


cdef class ElemScores:
    cdef public vector[Tscore] scores


def nest_by_graph(ElemGraph g, List[PlacementRuleSet] cases):
    cdef vector[PlacementRuleSetCC] cases_cc
    for case in cases:
        cases_cc.push_back(<PlacementRuleSetCC>case.cc_obj)
    cdef vector[vector[Tvertex]] result = nest_by_graph_cc(g.cc_obj, cases_cc)
    return result


def sort_graph(ElemGraph g, PlacementRuleSet rules, bool reverse=False):
    r = ElemGraph()
    r.cc_obj = sort_graph_cc(g.cc_obj, rules.cc_obj, reverse)
    return r


def score_rules(
    List[ElemGraph] graphs, List[PlacementRuleSet] rules
) -> List[Tscore]:
    cdef vector[ElemGraphCC] graphs_cc
    for graph in graphs:
        graphs_cc.push_back(graph.cc_obj)
    cdef vector[PlacementRuleSetCC] rules_cc
    for rule in rules:
        rules_cc.push_back(<PlacementRuleSetCC>rule.cc_obj)
    return score_rules_cc(graphs_cc, rules_cc)


def score_elems(ElemGraph g, PlacementRuleSet rules):
    scores = ElemScores()
    scores.scores = score_elems_cc(g.cc_obj, rules.cc_obj)
    return scores


def increase_selection_dfs(ElemGraph g, List[int] selection, max_tries: int, min_collisions: int):
    return increase_selection_dfs_cc(g.cc_obj, selection, max_tries, min_collisions)


def increase_score_dfs(ElemGraph g, List[int] selection, ElemScores scores):
    return increase_score_dfs_cc(g.cc_obj, selection, scores.scores)
