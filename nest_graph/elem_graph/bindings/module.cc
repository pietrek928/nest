#include <nanobind/nanobind.h>
namespace nb = nanobind;

#include "bindings.h"


NB_MODULE(elem_graph, m) {
    bind_elem_graph_types(m);
    bind_elem_graph_api(m);
}
