#pragma once

#include <nanobind/nanobind.h>
namespace nb = nanobind;


void bind_elem_graph_types(nb::module_ &m);
void bind_elem_graph_api(nb::module_ &m);
