#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>
#include <cmath>
#include <random>

#include "circle.h" // Changed from <circle.h> to standard local include

enum class ChainRole : uint8_t {
    FilledConvex = 0,
    Stroke = 1,
    Point = 2
};

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

template <class VecType>
inline typename VecType::Scalar ring_shoelace_abs_area(
    const VecType *pts,
    int n
) {
    using Scalar = typename VecType::Scalar;
    if (pts == nullptr || n < 3) {
        return static_cast<Scalar>(0);
    }
    Scalar ring_area = static_cast<Scalar>(0);
    for (int i = 0; i < n; ++i) {
        const auto &p1 = pts[i];
        const auto &p2 = pts[(i + 1) % n];
        ring_area += (p1[0] * p2[1]) - (p1[1] * p2[0]);
    }
    const Scalar signed_half = ring_area * static_cast<Scalar>(0.5);
    return signed_half < static_cast<Scalar>(0) ? -signed_half : signed_half;
}

template <class VecType>
inline typename VecType::Scalar ring_shoelace_abs_area(
    const std::vector<VecType> &pts
) {
    return ring_shoelace_abs_area<VecType>(
        pts.data(), static_cast<int>(pts.size()));
}

template<class VecType>
class PlaneGeometry {
    using Scalar = typename VecType::Scalar;

public:
    struct LinePlaneGeometryEntry {
        std::size_t start_point = 0;
        Circle<VecType> bounding_circle;
        bool is_subtractive = false;  // hole boundary segments (void interior)
        ChainRole role = ChainRole::FilledConvex;
        bool closed = false;  // Stroke only: include last→first edge
    };

    struct BoundaryRing {
        std::vector<VecType> points;
        bool is_subtractive = false;
    };

    std::vector<LinePlaneGeometryEntry> line_parts;
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
        LinePlaneGeometryEntry entry;
        entry.start_point = line_points.size();
        entry.bounding_circle = compute_exact_bounding_circle<VecType>(line, n, gen);
        entry.is_subtractive = subtractive;
        entry.role = ChainRole::FilledConvex;
        entry.closed = false;
        line_parts.push_back(entry);
        line_points.insert(line_points.end(), line, line + n);
    }

    void append_stroke(
        const VecType* line,
        int n,
        std::mt19937& gen,
        bool closed = false
    ) {
        if (line == nullptr || n < 2) {
            return;
        }
        LinePlaneGeometryEntry entry;
        entry.start_point = line_points.size();
        entry.bounding_circle = compute_exact_bounding_circle<VecType>(line, n, gen);
        entry.is_subtractive = false;
        entry.role = ChainRole::Stroke;
        entry.closed = closed;
        line_parts.push_back(entry);
        line_points.insert(line_points.end(), line, line + n);
    }

    void append_point(const VecType& pt, std::mt19937& gen) {
        LinePlaneGeometryEntry entry;
        entry.start_point = line_points.size();
        entry.bounding_circle = compute_exact_bounding_circle<VecType>(&pt, 1, gen);
        entry.is_subtractive = false;
        entry.role = ChainRole::Point;
        entry.closed = false;
        line_parts.push_back(entry);
        line_points.push_back(pt);
    }

    bool has_non_filled_parts() const {
        for (const auto& part : line_parts) {
            if (part.role != ChainRole::FilledConvex) {
                return true;
            }
        }
        return false;
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

    // Net footprint area. Holes are reverse_ring'd to the same winding as
    // outers in decompose_complex_polygon, so shoelace sign alone is wrong —
    // apply is_subtractive (outer add, hole subtract).
    Scalar area() const {
        Scalar total_area = 0;

        for (const auto& ring : boundary_rings) {
            const Scalar mag = ring_shoelace_abs_area(ring.points);
            total_area += ring.is_subtractive ? -mag : mag;
        }

        return total_area;
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
    PlaneGeometry translate(const VecType& offset) const {
        PlaneGeometry out = *this;
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

    PlaneGeometry rotate(
        Scalar angle,
        const VecType& origin = VecType({0, 0})
    ) const {
        static_assert(VecType::dim == 2, "PlaneGeometry::rotate requires 2D VecType");

        const Scalar cos_a = std::cos(angle);
        const Scalar sin_a = std::sin(angle);
        PlaneGeometry out = *this;

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