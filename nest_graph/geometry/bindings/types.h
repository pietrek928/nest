#pragma once

#include <cstdint>
#include <random>
#include <vector>

#include <nanobind/nanobind.h>
namespace nb = nanobind;

#include "geometry_factory.h"
#include "guide/guide.h"
#include "guide/polygon_cast.h"
#include "python_converters.h"
#include "plane/plane_geometry.h"
#include "distance/polygon_distance.h"
#include "distance/static_collision_scene.h"

using PlaneGeometry2d = PlaneGeometry<Vec2d>;
using DistanceResult2d = ComplexDistanceResult<Vec2d>;
using CastResult2d = ComplexCastResult<Vec2d>;
using PairDistanceResult2d = PairDistanceResult<Vec2d>;
using GuidanceConfig2d = GuidanceConfig<Vec2d>;
using PlacementGuidance2d = PlacementGuidance<Vec2d>;
using PlacementProposition2d = PlacementProposition<Vec2d>;

struct GeometryHolder {
    PlaneGeometry2d plane;
    std::mt19937 rng;

    GeometryHolder()
        : rng(std::random_device{}()) {}

    explicit GeometryHolder(std::uint32_t seed)
        : rng(seed != 0 ? std::mt19937(seed) : std::mt19937(std::random_device{}())) {}

    explicit GeometryHolder(PlaneGeometry2d mesh)
        : plane(std::move(mesh)), rng(std::random_device{}()) {}
};

struct StaticCollisionSceneHolder {
    StaticCollisionScene<Vec2d> scene;
};

nb::tuple circle_bounds_tuple(const GeometryHolder &g);
nb::tuple plane_aabb_tuple(const GeometryHolder &g);
std::vector<PlaneGeometry2d> planes_from_holders(
    std::vector<GeometryHolder> holders);
std::vector<const PlaneGeometry2d *> plane_ptrs_from_holders(
    const std::vector<GeometryHolder> &holders);
Vec2d slide_vector_from_handle(nb::handle slide);
GeometryHolder geometry_from_line_coords(std::vector<Vec2d> pts);
GeometryHolder geometry_from_rings_coords(std::vector<std::vector<Vec2d>> rings);
GeometryHolder geometry_from_stroke_coords(std::vector<Vec2d> pts);
GeometryHolder geometry_from_point_xy(double x, double y);
std::string geometry_to_wkt(const GeometryHolder &g);
GeometryHolder geometry_holder_from_shapely(nb::handle geom);
