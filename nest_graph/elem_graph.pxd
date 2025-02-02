from libcpp.vector cimport vector

# typedef struct PlacementRuleSet {
#     std::vector<PointPlaceRule> point_rules;
#     std::vector<BBoxPlaceRule> bbox_rules;
# } PlacementRuleSet;


# typedef struct ElemGraph {
#     std::vector<Tvertex> group_id;
#     std::vector<Point> centers;
#     std::vector<BBox> coords;
#     std::vector<std::vector<Tvertex>> collisions;

#     auto size() const { return group_id.size(); }
# } ElemGraph;


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
