#pragma once

#include <cstddef>
#include <vector>
#include <cmath>
#include <random>

#include "circle.h" // Changed from <circle.h> to standard local include

template <class VecType>
inline VecType rotate_point_2d(
    const VecType& p,
    typename VecType::Scalar cos_a,
    typename VecType::Scalar sin_a,
    const VecType& origin
) {
    static_assert(VecType::dim == 2, "rotate_point_2d requires 2D vectors");
    VecType d = p - origin;
    // Honoring dimension-agnostic design by using .x() and .y() instead of []
    return origin + VecType({
        cos_a * d[0] - sin_a * d[1],
        sin_a * d[0] + cos_a * d[1]
    });
}

template<class VecType>
class SolidGeometry {
    using Scalar = typename VecType::Scalar;

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

    void append_line_poly(
        const VecType* line,
        int n,
        std::mt19937& gen,
        bool subtractive = false
    ) {
        line_parts.push_back({
            line_points.size(),
            compute_exact_bounding_circle<VecType>(line, n, gen),
            subtractive
        });
        line_points.insert(line_points.end(), line, line + n);
    }

    void finalize(std::mt19937& gen) {
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

    // -------------------------------------------------------------------------
    // GEOMETRIC PROPERTIES
    // -------------------------------------------------------------------------

    // Calculates exact net area. Because rings have proper winding directions,
    // the Shoelace formula automatically yields negative area for holes,
    // providing the exact physical mass footprint of the shape.
    Scalar area() const {
        Scalar total_area = 0;

        for (const auto& ring : boundary_rings) {
            Scalar ring_area = 0;
            int n = static_cast<int>(ring.points.size());
            if (n < 3) continue;

            for (int i = 0; i < n; ++i) {
                const auto& p1 = ring.points[i];
                const auto& p2 = ring.points[(i + 1) % n];
                ring_area += (p1[0] * p2[1]) - (p1[1] * p2[0]);
            }
            total_area += ring_area;
        }

        return total_area * static_cast<Scalar>(0.5);
    }

    const Circle<VecType>& get_bounding_circle() const {
        return bounding_circle;
    }

    // -------------------------------------------------------------------------
    // PHYSICS ENGINE EXTRACTORS (GJK/EPA)
    // -------------------------------------------------------------------------
    int get_part_size(size_t part_idx) const {
        if (part_idx + 1 < line_parts.size()) {
            return line_parts[part_idx + 1].start_point - line_parts[part_idx].start_point;
        }
        return line_points.size() - line_parts[part_idx].start_point;
    }

    const VecType* get_part_points(size_t part_idx) const {
        return &line_points[line_parts[part_idx].start_point];
    }

    // -------------------------------------------------------------------------
    // TRANSFORMATIONS
    // -------------------------------------------------------------------------
    SolidGeometry translate(const VecType& offset) const {
        SolidGeometry out = *this;
        for (auto& p : out.line_points) p = p + offset;

        for (auto& ring : out.boundary_rings) {
            for (auto& p : ring.points) p = p + offset;
        }

        for (auto& part : out.line_parts) {
            part.bounding_circle.c = part.bounding_circle.c + offset;
        }
        out.bounding_circle.c = out.bounding_circle.c + offset;

        return out;
    }

    SolidGeometry rotate(
        Scalar angle,
        const VecType& origin = VecType({0, 0})
    ) const {
        static_assert(VecType::dim == 2, "SolidGeometry::rotate requires 2D VecType");

        const Scalar cos_a = std::cos(angle);
        const Scalar sin_a = std::sin(angle);
        SolidGeometry out = *this;

        for (auto& p : out.line_points) {
            p = rotate_point_2d(p, cos_a, sin_a, origin);
        }

        for (auto& ring : out.boundary_rings) {
            for (auto& p : ring.points) {
                p = rotate_point_2d(p, cos_a, sin_a, origin);
            }
        }

        for (auto& part : out.line_parts) {
            part.bounding_circle.c = rotate_point_2d(
                part.bounding_circle.c, cos_a, sin_a, origin);
        }

        out.bounding_circle.c = rotate_point_2d(
            out.bounding_circle.c, cos_a, sin_a, origin);

        return out;
    }
};