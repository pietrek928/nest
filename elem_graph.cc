#include "elem_graph.h"


void mark_element(ElemGraph &graph, Tvertex elem, Tvertex *marked) {
    if (marked[elem]) return;
    marked[elem] = 1;
    for (Tvertex coll : graph.collisions[elem]) {
        marked[coll] = 1;
    }
}
