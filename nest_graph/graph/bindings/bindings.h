#pragma once

#include <nanobind/nanobind.h>
namespace nb = nanobind;


void bind_graph_types(nb::module_ &m);
void bind_graph_api(nb::module_ &m);
void bind_graph_decision(nb::module_ &m);
void bind_graph_mcts(nb::module_ &m);
