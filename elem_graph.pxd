from libcpp.vector cimport vector

cdef extern from "elem_graph.cc":
    ctypedef int Tvertex

    cdef struct PlacementRules:
        pass

    cdef cppclass ElemGraph:
        pass

    vector[vector[Tvertex]] nest_by_graph(const ElemGraph& g, const vector[PlacementRules]& cases)