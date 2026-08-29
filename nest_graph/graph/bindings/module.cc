#include <nanobind/nanobind.h>
namespace nb = nanobind;

#include "bindings.h"


NB_MODULE(graph, m) {
    bind_graph_types(m);
    bind_graph_api(m);
    bind_graph_decision(m);
    bind_graph_mcts(m);
}
