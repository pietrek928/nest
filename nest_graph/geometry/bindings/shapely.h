#pragma once

#include <nanobind/nanobind.h>
namespace nb = nanobind;
#include <vector>

#include "python_converters.h"

bool geom_type_is(nb::handle geom, const char *name);
void collect_from_shapely(
    nb::handle geom,
    std::vector<std::vector<Vec2d>> &outers,
    std::vector<std::vector<Vec2d>> &holes);
