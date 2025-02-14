from libcpp cimport bool
from libcpp.vector cimport vector


cdef extern from "elem_graph.cc":
    ctypedef int Tvertex

    cdef struct BBox:
        float xstart, xend, ystart, yend

    cdef struct Point:
        float x, y

    cdef struct ElemGroup:
        int count_limit
        float priority

    cdef struct BBoxPlaceRule:
        BBox bbox
        float r, w
        Tvertex group

    cdef struct PointPlaceRule:
        float x, y, r, w
        Tvertex group

    cdef struct PointPlaceRule:
        float x, y, r, w
        Tvertex group

    cdef struct PlacementRuleSet:
        vector[PointPlaceRule] point_rules
        vector[BBoxPlaceRule] bbox_rules

    cdef struct ElemGraph:
        vector[Tvertex] group_id
        vector[Point] centers
        vector[BBox] coords
        vector[vector[Tvertex]] collisions

    vector[vector[Tvertex]] nest_by_graph(const ElemGraph& g, const vector[PlacementRuleSet]& cases)
    ElemGraph sort_graph(const ElemGraph &g, const PlacementRuleSet &rules, bool reverse)
    vector[Tvertex] increase_selection_dfs(const ElemGraph &g, const vector[Tvertex] &selected_nodes, int max_tries)
