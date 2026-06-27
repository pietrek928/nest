#pragma once

#include <nanobind/nanobind.h>
namespace nb = nanobind;
#include <vector>

#include <vec.h>

using Vec2d = Vec<2, double>;

bool read_xy(nb::handle pt, double &x, double &y);
bool read_transform(nb::handle t, double &x, double &y, double &angle);
nb::tuple vec2d_to_tuple(const Vec2d &v);
Vec2d vec2d_from_tuple(nb::handle o);
void points_from_iterable(nb::handle points, std::vector<Vec2d> &out);
std::vector<Vec2d> ring_from_coords(nb::handle coords_iterable);
