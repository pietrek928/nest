#include <nanobind/nanobind.h>

#include "bindings.h"

namespace nb = nanobind;

NB_MODULE(elem_graph, m) {
    bind_elem_graph_types(m);
    bind_elem_graph_api(m);
}
