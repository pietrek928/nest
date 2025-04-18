from libcpp cimport bool
from libcpp.vector cimport vector


cdef extern from "elem_graph.cc":
    ctypedef int Tvertex
    ctypedef float Tscore

    cdef struct BBox:
        float xstart, xend, ystart, yend

    cdef struct Point:
        float x, y

    cdef struct ElemPlace:
        float x, y, a

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

    cdef struct PointAngleRule:
        float x, y, a, r, w
        Tvertex group

    cdef struct BBoxAngleRule:
        BBox bbox
        float r, w
        Tvertex group

    cdef struct PlacementRuleSet:
        vector[PointPlaceRule] point_rules
        vector[BBoxPlaceRule] bbox_rules
        vector[PointAngleRule] point_angle_rules
        vector[BBoxAngleRule] bbox_angle_rules
    int size(const PlacementRuleSet &s)

    cdef struct RuleMutationSettings:
        BBox box
        float dpos, dw, da
        float insert_p, remove_p, mutate_p
        Tvertex ngroups

    cdef struct ElemGraph:
        vector[Tvertex] group_id
        vector[ElemPlace] elems
        vector[BBox] coords
        vector[vector[Tvertex]] collisions

    vector[PlacementRuleSet] augment_rules(
        const vector[PlacementRuleSet] &rules, const RuleMutationSettings &settings
    )

    vector[vector[Tvertex]] nest_by_graph(const ElemGraph& g, const vector[PlacementRuleSet]& cases)
    vector[Tscore] score_elems(const ElemGraph& g, const PlacementRuleSet& rules)
    ElemGraph sort_graph(const ElemGraph &g, const PlacementRuleSet &rules, bool reverse)
    vector[Tscore] score_rules(
        const vector[ElemGraph] &graphs, const vector[PlacementRuleSet] &rule_sets
    )
    vector[Tvertex] increase_selection_dfs(
        const ElemGraph &g, const vector[Tvertex] &selected_nodes, int max_tries, int min_collisions
    )
    vector[Tvertex] increase_score_dfs(
        const ElemGraph& g, const vector[Tvertex] &selected_nodes, const vector[Tscore] &scores
    )
