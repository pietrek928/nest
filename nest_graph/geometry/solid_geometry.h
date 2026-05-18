#pragma once

#include <cstddef>
#include <vector>
#include <cmath>

#include <circle.h>


namespace solid_geometry_detail {

template <class VecType>
inline VecType rotate_point_2d(
    const VecType& p,
    typename VecType::Scalar cos_a,
    typename VecType::Scalar sin_a,
    const VecType& origin) {
    static_assert(VecType::dim == 2, "rotate_point_2d requires 2D vectors");
    VecType d = p - origin;
    return origin + VecType({cos_a * d[0] - sin_a * d[1], sin_a * d[0] + cos_a * d[1]});
}

}  // namespace solid_geometry_detail


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

    SolidGeometry translate(const VecType& offset) const {
        SolidGeometry out = *this;
        for (auto& p : out.line_points) {
            p += offset;
        }
        for (auto& ring : out.boundary_rings) {
            for (auto& p : ring.points) {
                p += offset;
            }
        }
        for (auto& part : out.line_parts) {
            part.bounding_circle.c += offset;
        }
        out.bounding_circle.c += offset;
        return out;
    }

    SolidGeometry rotate(
        typename VecType::Scalar angle,
        const VecType& origin = VecType({0, 0})) const {
        static_assert(VecType::dim == 2, "SolidGeometry::rotate requires 2D VecType");
        using Scalar = typename VecType::Scalar;
        const Scalar cos_a = std::cos(angle);
        const Scalar sin_a = std::sin(angle);
        SolidGeometry out = *this;
        for (auto& p : out.line_points) {
            p = solid_geometry_detail::rotate_point_2d(p, cos_a, sin_a, origin);
        }
        for (auto& ring : out.boundary_rings) {
            for (auto& p : ring.points) {
                p = solid_geometry_detail::rotate_point_2d(p, cos_a, sin_a, origin);
            }
        }
        for (auto& part : out.line_parts) {
            part.bounding_circle.c = solid_geometry_detail::rotate_point_2d(
                part.bounding_circle.c, cos_a, sin_a, origin);
        }
        out.bounding_circle.c = solid_geometry_detail::rotate_point_2d(
            out.bounding_circle.c, cos_a, sin_a, origin);
        return out;
    }
};
