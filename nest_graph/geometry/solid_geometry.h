#pragma once

#include <cstddef>
#include <vector>
#include <cmath>

#include "circle.h"


template<class VecType>
class SolidGeometry {
    typedef struct {
        std::size_t start_point;
        Circle<VecType> bounding_circle;
        bool is_subtractive = false;  // hole boundary segments (void interior)
    } LineSolidGeometryEntry;

    struct BoundaryRing {
        std::vector<VecType> points;
        bool is_subtractive = false;
    };

public:
    std::vector<LineSolidGeometryEntry> line_parts;
    std::vector<VecType> line_points;
    std::vector<BoundaryRing> boundary_rings;
    Circle<VecType> bounding_circle;

    void add_boundary_ring(const std::vector<VecType>& ring, bool subtractive = false) {
        if (ring.size() >= 3) {
            boundary_rings.push_back({ring, subtractive});
        }
    }

    void append_line_poly(const VecType* line, int n, bool subtractive = false) {
        std::random_device rd;
        std::mt19937 gen(rd());
        line_parts.push_back({
            line_points.size(),
            compute_exact_bounding_circle<VecType>(line, n, gen),
            subtractive
        });
        line_points.insert(line_points.end(), line, line+n);
    }

    void finalize() {
        std::random_device rd;
        std::mt19937 gen(rd());
        if (!line_points.empty()) {
            bounding_circle = compute_exact_bounding_circle<VecType>(
                line_points.data(),
                static_cast<int>(line_points.size()),
                gen
            );
        } else if (!boundary_rings.empty()) {
            std::vector<VecType> all_pts;
            for (const auto& ring : boundary_rings) {
                all_pts.insert(all_pts.end(), ring.points.begin(), ring.points.end());
            }
            bounding_circle = compute_exact_bounding_circle<VecType>(
                all_pts.data(),
                static_cast<int>(all_pts.size()),
                gen
            );
        }
        line_points.shrink_to_fit();
        line_parts.shrink_to_fit();
        boundary_rings.shrink_to_fit();
    }

    const Circle<VecType>& get_bounding_circle() const {
        return bounding_circle;
    }

    // Helpers to safely extract arrays and sizes for GJK
    int get_part_size(size_t part_idx) const {
        if (part_idx + 1 < line_parts.size()) {
            return line_parts[part_idx + 1].start_point - line_parts[part_idx].start_point;
        }
        return line_points.size() - line_parts[part_idx].start_point;
    }

    const VecType* get_part_points(size_t part_idx) const {
        return &line_points[line_parts[part_idx].start_point];
    }
};
