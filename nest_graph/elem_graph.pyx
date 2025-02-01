from libcpp.vector cimport vector

from .elem_graph cimport nest_by_graph as cpp_nest_by_graph, ElemGraph, PlacementRules, Tvertex

def py_nest_by_graph(ElemGraph g, list cases):
    cdef vector[PlacementRules] cpp_cases = cases
    cdef vector[vector[Tvertex]] result = cpp_nest_by_graph(g, cpp_cases)
    return [[v for v in inner] for inner in result]
