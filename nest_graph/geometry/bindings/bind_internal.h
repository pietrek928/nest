#pragma once

#include <nanobind/nanobind.h>
namespace nb = nanobind;

void bind_geometry_class(nb::module_ &m);
void bind_guidance_types(nb::module_ &m);
void bind_guidance_api(nb::module_ &m);
void bind_distance_types(nb::module_ &m);
void bind_distance_api(nb::module_ &m);
void bind_intersect_api(nb::module_ &m);
void bind_cast_types(nb::module_ &m);
void bind_cast_api(nb::module_ &m);
void bind_batch_api(nb::module_ &m);
