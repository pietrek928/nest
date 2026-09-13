#pragma once

#include <nanobind/nanobind.h>
namespace nb = nanobind;
#include <string>
#include <vector>

#include "python_converters.h"

bool geom_type_is(nb::handle geom, const char *name);
std::string geom_type_string(nb::handle geom);
void collect_from_shapely(
    nb::handle geom,
    std::vector<std::vector<Vec2d>> &outers,
    std::vector<std::vector<Vec2d>> &holes,
    bool include_holes = true);
