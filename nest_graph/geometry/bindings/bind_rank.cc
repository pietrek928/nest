#include <cmath>
#include <limits>
#include <optional>
#include <tuple>
#include <vector>

#include <nanobind/nanobind.h>
namespace nb = nanobind;
#include <nanobind/stl/optional.h>
#include <nanobind/stl/tuple.h>
#include <nanobind/stl/vector.h>

#include "bind_internal.h"
#include "convex/hull.h"
#include "rank/placement_rank.h"
#include "types.h"

void bind_rank_api(nb::module_ &m) {
    // Snake_case names match ProposeConfig / RankingMode strings; PascalCase kept as aliases.
    nb::enum_<PlacementRankMode>(m, "PlacementRankMode", nb::is_arithmetic())
        .value("clearance", PlacementRankMode::Clearance)
        .value("Clearance", PlacementRankMode::Clearance)
        .value("border", PlacementRankMode::Border)
        .value("Border", PlacementRankMode::Border)
        .value("contact", PlacementRankMode::Contact)
        .value("Contact", PlacementRankMode::Contact)
        .value("contact_hybrid", PlacementRankMode::ContactHybrid)
        .value("ContactHybrid", PlacementRankMode::ContactHybrid)
        .value("legacy", PlacementRankMode::Legacy)
        .value("Legacy", PlacementRankMode::Legacy)
        .value("hybrid", PlacementRankMode::Hybrid)
        .value("Hybrid", PlacementRankMode::Hybrid)
        .value("rule_hybrid", PlacementRankMode::RuleHybrid)
        .value("RuleHybrid", PlacementRankMode::RuleHybrid);

    nb::class_<PlacementRankConfig>(m, "PlacementRankConfig")
        .def(nb::init<>())
        .def_rw("min_dist", &PlacementRankConfig::min_dist)
        .def_rw("clearance_weight", &PlacementRankConfig::clearance_weight)
        .def_rw("tightness_weight", &PlacementRankConfig::tightness_weight)
        .def_rw("edge_free_weight", &PlacementRankConfig::edge_free_weight)
        .def_rw("edge_free_band_mult", &PlacementRankConfig::edge_free_band_mult)
        .def_rw("kiss_tol", &PlacementRankConfig::kiss_tol)
        .def_rw("tight_scale", &PlacementRankConfig::tight_scale)
        .def_rw("part_area", &PlacementRankConfig::part_area)
        .def_rw("sheet_area", &PlacementRankConfig::sheet_area)
        .def_rw("clearance_margin_eps", &PlacementRankConfig::clearance_margin_eps)
        .def_rw("mode", &PlacementRankConfig::mode);

    nb::class_<PlacementRankResult>(m, "PlacementRankResult")
        .def(nb::init<>())
        .def_ro("valid", &PlacementRankResult::valid)
        .def_ro("contact_err", &PlacementRankResult::contact_err)
        .def_ro("contact_score", &PlacementRankResult::contact_score)
        .def_ro("clearance", &PlacementRankResult::clearance)
        .def_ro("excess", &PlacementRankResult::excess)
        .def_ro("kiss", &PlacementRankResult::kiss)
        .def_ro("d_board", &PlacementRankResult::d_board)
        .def_ro("d_pack", &PlacementRankResult::d_pack)
        .def_ro("edge_free", &PlacementRankResult::edge_free)
        .def_ro("rank_score", &PlacementRankResult::rank_score)
        .def_ro("quality", &PlacementRankResult::quality)
        .def_ro("score", &PlacementRankResult::rank_score);

    m.def(
        "convex_hull_area_of",
        [](const std::vector<GeometryHolder> &geoms) {
            std::vector<const SolidGeometry2d *> ptrs;
            ptrs.reserve(geoms.size());
            for (const auto &g : geoms) {
                ptrs.push_back(&g.solid);
            }
            return static_cast<double>(
                solids_convex_hull_area(ptrs.data(), ptrs.size()));
        },
        nb::arg("geoms"));

    m.def(
        "batch_rank_local_placements",
        [](const GeometryHolder &part,
           const std::vector<std::tuple<double, double, double>> &transforms,
           const GeometryHolder &board_ring,
           std::vector<GeometryHolder> obstacles,
           nb::object focal,
           const PlacementRankConfig &config) {
            const SolidGeometry2d *focal_ptr = nullptr;
            std::optional<GeometryHolder> focal_holder;
            if (!focal.is_none()) {
                focal_holder = nb::cast<GeometryHolder>(focal);
                focal_ptr = &focal_holder->solid;
            }
            return batch_rank_local_placements<Vec2d>(
                part.solid,
                transforms,
                &board_ring.solid,
                solids_from_holders(std::move(obstacles)),
                focal_ptr,
                config);
        },
        nb::arg("part"),
        nb::arg("transforms"),
        nb::arg("board_ring"),
        nb::arg("obstacles"),
        nb::arg("focal") = nb::none(),
        nb::arg("config") = PlacementRankConfig{});

    m.def(
        "batch_score_placed_contact_hybrid",
        [](const std::vector<GeometryHolder> &placed,
           const GeometryHolder &board_ring,
           std::vector<GeometryHolder> packed,
           nb::object focal,
           const PlacementRankConfig &config) {
            std::vector<SolidGeometry2d> placed_solids;
            placed_solids.reserve(placed.size());
            for (const auto &h : placed) {
                placed_solids.push_back(h.solid);
            }
            const SolidGeometry2d *focal_ptr = nullptr;
            std::optional<GeometryHolder> focal_holder;
            if (!focal.is_none()) {
                focal_holder = nb::cast<GeometryHolder>(focal);
                focal_ptr = &focal_holder->solid;
            }
            return batch_score_placed_contact_hybrid<Vec2d>(
                placed_solids,
                &board_ring.solid,
                solids_from_holders(std::move(packed)),
                focal_ptr,
                config);
        },
        nb::arg("placed"),
        nb::arg("board_ring"),
        nb::arg("packed"),
        nb::arg("focal") = nb::none(),
        nb::arg("config") = PlacementRankConfig{});
}
